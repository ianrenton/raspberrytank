//
// Raspberry Tank HTTP Remote Control script
// Ian Renton, July 2012
// http://ianrenton.com
//
// This code released into the public domain without licence.
// 
// Based on the GPIO example by Dom and Gert
// (http://elinux.org/Rpi_Low-level_peripherals#GPIO_Driving_Example_.28C.29)
// Using Heng Long op codes discovered by ancvi_pIII
// (http://www.rctanksaustralia.com/forum/viewtopic.php?p=1314#p1314)
//

#define BCM2708_PERI_BASE        0x20000000
#define GPIO_BASE                (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <dirent.h>
#include <fcntl.h>
#include <assert.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <pthread.h>
#include "mongoose.h"

#define PAGE_SIZE (4*1024)
#define BLOCK_SIZE (4*1024)

int  mem_fd;
char *gpio_mem, *gpio_map;
char *spi0_mem, *spi0_map;

// I/O access
volatile unsigned *gpio;

// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x) or SET_GPIO_ALT(x,y)
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))

// N.B. These have been reversed compared to Gert & Dom's original code!
// This is because the transistor board I use for talking to the Heng
// Long RX18 inverts the signal.  So the GPIO_SET pointer here actually
// sets the GPIO pin low - but that ends up as a high at the tank.
#define GPIO_CLR *(gpio+7)  // sets   bits which are 1, ignores bits which are 0
#define GPIO_SET *(gpio+10) // clears bits which are 1, ignores bits which are 0

// GPIO pin that connects to the Heng Long main board
// (Pin 7 is the top right pin on the Pi's GPIO, next to the yellow video-out)
#define PIN 7

// Heng Long tank bit-codes
int idle = 0xFE40121C;
int ignition = 0xFE401294;
int left_slow = 0xFE400608;
int left_fast = 0xFE400010;
int right_slow = 0xFE401930;
int right_fast = 0xFE401E2C;
int fwd_slow = 0xFE200F34;
int fwd_fast = 0xFE000F3C;
int rev_slow = 0xFE580F08;
int rev_fast = 0xFE780F00;
int turret_left = 0xFE408F0C;
int turret_right = 0xFE410F28;
int turret_elev = 0xFE404F3C;
int fire = 0xFE442F34;
int machine_gun = 0xFE440F78;
int recoil = 0xFE420F24;

// Mutexes
pthread_mutex_t commandMutex = PTHREAD_MUTEX_INITIALIZER;

// Mutex-controlled Variables
char* command = "00000000\0";

// Function declarations
void setup_io();
void sendCode(int code);
void sendBit(int bit);
void* launch_server();
static void *http_callback(enum mg_event event,
                      struct mg_connection *conn,
                      const struct mg_request_info *request_info);

// Main
int main(int argc, char **argv) { 

  printf("\nRaspberry Tank HTTP Remote Control script\nIan Renton, July 2012\nhttp://ianrenton.com\n\n");

  int g,rep,i;
  char inchar;
  char* copiedCommand = malloc(sizeof(char)*10);

  // Set up gpio pointer for direct register access
  setup_io();

  // Switch the relevant GPIO pin to output mode
  INP_GPIO(PIN); // must use INP_GPIO before we can use OUT_GPIO
  OUT_GPIO(PIN);

  GPIO_CLR = 1<<PIN;
  
  // Send the idle and ignition codes
  printf("Idle\n");
  for (i=0; i<40; i++) 
  {
    sendCode(idle);
  }
  printf("Ignition\n");
  for (i=0; i<10; i++) 
  {
    sendCode(ignition);
  }
  printf("Waiting for ignition\n");
  for (i=0; i<300; i++) 
  {
    sendCode(idle);
  }
  
  // Launch HTTP server
  pthread_t httpThread; 
  int httpThreadExitCode = pthread_create( &httpThread, NULL, &launch_server, (void*) NULL);
  
  // Loop, sending movement commands indefinitely
  while(1) {
    pthread_mutex_lock( &commandMutex );
    strcpy(&copiedCommand[0], &command[0]);
    pthread_mutex_unlock( &commandMutex );
    if (copiedCommand[0] == '1') {
      // Forward
      sendCode(fwd_fast);
    } else if (copiedCommand[1] == '1') {
      // Reverse
      sendCode(rev_fast);
    } else if (copiedCommand[2] == '1') {
      //Left
      sendCode(left_fast);
    } else if (copiedCommand[3] == '1') {
      //Right
      sendCode(right_fast);
    } else if (copiedCommand[4] == '1') {
      // Turret Left
      sendCode(turret_left);
    } else if (copiedCommand[5] == '1') {
      // Turret Right
      sendCode(turret_right);
    } else if (copiedCommand[6] == '1') {
      // Turret Elev
      sendCode(turret_elev);
    } else if (copiedCommand[7] == '1') {
      // Fire
      sendCode(fwd_fast);
    } else {
      // Idle
      sendCode(fire);
    }
  }
  
  printf("Ignition Off\n");
  for (i=0; i<10; i++)
  {
    sendCode(ignition);
  }
  printf("Idle\n");
  for (i=0; i<40; i++)
  {
    sendCode(idle);
  }
  
  return 0;
} // main


// Sends one individual code to the main tank controller
void sendCode(int code) {
  // Send header "bit" (not a valid Manchester code)
  GPIO_SET = 1<<PIN;
  usleep(500);
  
  // Send the code itself, bit by bit using Manchester coding
  int i;
  for (i=0; i<32; i++) {
    int bit = (code>>(31-i)) & 0x1;
    sendBit(bit);
  }
  
  // Force a 4ms gap between messages
  GPIO_CLR = 1<<PIN;
  usleep(3333);
} // sendCode


// Sends one individual bit using Manchester coding
// 1 = high-low, 0 = low-high
void sendBit(int bit) {
  //printf("%d", bit);

  if (bit == 1) {
    GPIO_SET = 1<<PIN;
    usleep(250);
    GPIO_CLR = 1<<PIN;
    usleep(250);
  } else {
    GPIO_CLR = 1<<PIN;
    usleep(250);
    GPIO_SET = 1<<PIN;
    usleep(250);
  }
} // sendBit


// Set up a memory region to access GPIO
void setup_io() {

   /* open /dev/mem */
   if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
      printf("can't open /dev/mem \n");
      exit (-1);
   }

   /* mmap GPIO */

   // Allocate MAP block
   if ((gpio_mem = malloc(BLOCK_SIZE + (PAGE_SIZE-1))) == NULL) {
      printf("allocation error \n");
      exit (-1);
   }

   // Make sure pointer is on 4K boundary
   if ((unsigned long)gpio_mem % PAGE_SIZE)
     gpio_mem += PAGE_SIZE - ((unsigned long)gpio_mem % PAGE_SIZE);

   // Now map it
   gpio_map = (unsigned char *)mmap(
      (caddr_t)gpio_mem,
      BLOCK_SIZE,
      PROT_READ|PROT_WRITE,
      MAP_SHARED|MAP_FIXED,
      mem_fd,
      GPIO_BASE
   );

   if ((long)gpio_map < 0) {
      printf("mmap error %d\n", (int)gpio_map);
      exit (-1);
   }

   // Always use volatile pointer!
   gpio = (volatile unsigned *)gpio_map;
   
} // setup_io


// Launch HTTP server
void* launch_server() {
  struct mg_context *ctx;
  const char *options[] = {"listening_ports", "3000", NULL};
  
  printf("Starting HTTP Server on port 3000\n");
  ctx = mg_start(&http_callback, NULL, options);
  getchar();  // Wait until user hits "enter".  This will never happen when this
              // code runs on the tank at startup
  mg_stop(ctx);
}

// HTTP server callback
static void *http_callback(enum mg_event event,
                      struct mg_connection *conn,
                      const struct mg_request_info *request_info) {
  if (event == MG_NEW_REQUEST) {
    // Echo requested URI back to the client
    mg_printf(conn, "HTTP/1.1 200 OK\r\n"
              "Content-Type: text/plain\r\n\r\n"
              "%s", request_info->uri);
    printf("Received command from HTTP: %s\n", request_info->query_string);
    pthread_mutex_lock( &commandMutex ); 
    command = request_info->query_string;
    pthread_mutex_unlock( &commandMutex ); 
    return "";  // Mark as processed
  } else {
    return NULL;
  }
}
