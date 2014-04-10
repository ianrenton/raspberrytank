//
// Raspberry Tank HTTP Remote Control script
// Ian Renton, April 2014
// http://raspberrytank.ianrenton.com
// 
// Based on the Raspberry Pi GPIO example by Dom and Gert
// (http://elinux.org/Rpi_Low-level_peripherals#GPIO_Driving_Example_.28C.29)
// ...and I2C Compass/Rangefinder code by James Henderson, from robot-electronics.co.uk
// Using Heng Long op codes discovered by ancvi_pIII
// (http://www.rctanksaustralia.com/forum/viewtopic.php?p=1314#p1314)
//

// Raspberry Pi setup
#define BCM2708_PERI_BASE        0x20000000
#define GPIO_BASE                (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */
#define PAGE_SIZE (4*1024)
#define BLOCK_SIZE (4*1024)

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <dirent.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <assert.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <pthread.h>
#include "mongoose.h"

// I/O access
int  mem_fd;
char *gpio_mem, *gpio_map;
char *spi0_mem, *spi0_map;
volatile unsigned *gpio;

// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x) or SET_GPIO_ALT(x,y)
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))

// N.B. The tank's transistor setup actually inverts the GPIO output signal, so
// for example using GPIO_SET sets the GPIO output high, but the tank's RX18
// board sees that as a lot.
#define GPIO_SET *(gpio+7)  // sets   bits which are 1, ignores bits which are 0
#define GPIO_CLR *(gpio+10) // clears bits which are 1, ignores bits which are 0

// GPIO pin that connects to the Heng Long main board
// (Pin 7 is the top right pin on the Pi's GPIO, next to the yellow video-out)
#define PIN 7

// Heng Long tank opcodes
int idle =         0x10048;
int ignition =     0x1004a;
int neutral =      0x0f03c;
int left_slow =    0x10018;
int left_fast =    0x10000;
int right_slow =   0x10064;
int right_fast =   0x10078;
int fwd_slow =     0x0803c;
int fwd_fast =     0x0003c;
int rev_slow =     0x1603c;
int rev_fast =     0x1e03c;
int turret_left =  0x1023c;
int turret_right = 0x1043c;
int turret_elev =  0x1013c;
int fire =         0x0f840;
int machine_gun =  0x1103d;
int recoil =       0x1083c;

// Mutexes
pthread_mutex_t userCommandMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t autonomyCommandMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t sensorDataMutex = PTHREAD_MUTEX_INITIALIZER;

// Mutex-controlled variables
char* userCommand;
char* autonomyCommand;
int range;
int bearing;
int pitch;
int roll;

// Function declarations
void setup_io();
void sendCode(int code);
void sendBit(int bit);
int CRC(int data);
void* launch_server();
static int http_callback(struct mg_connection *conn);
void* launch_sensors();
void* launch_autonomy();
void* autonomySendCommand(char* cmd);

// Main
int main(int argc, char **argv) { 

  printf("\nRaspberry Tank HTTP Remote Control script\nIan Renton, April 2014\nhttp://raspberrytank.ianrenton.com\n\n");

  int g,rep,i;
  char inchar;
  userCommand = malloc(sizeof(char)*10);
  autonomyCommand = malloc(sizeof(char)*10);
  char* copiedCommand = malloc(sizeof(char)*10);

  // Set up gpio pointer for direct register access
  setup_io();

  // Switch the relevant GPIO pin to output mode
  INP_GPIO(PIN); // must use INP_GPIO before we can use OUT_GPIO
  OUT_GPIO(PIN);

  // Set all GPIO outputs high (this is a low when the tank sees it, which is
  // the reset state.
  GPIO_SET = 1<<PIN;
  
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
  
  // Launch sensor polling thread
  pthread_t sensorThread; 
  int sensorThreadExitCode = pthread_create( &sensorThread, NULL, &launch_sensors, (void*) NULL);
  
  // Launch autonomy thread
  pthread_t autonomyThread; 
  int autonomyThreadExitCode = pthread_create( &autonomyThread, NULL, &launch_autonomy, (void*) NULL);
  
  // Loop, sending movement commands indefinitely
  while(1) {
    pthread_mutex_lock( &userCommandMutex );
    strcpy(&copiedCommand[0], &userCommand[0]);
    pthread_mutex_unlock( &userCommandMutex );

    if (copiedCommand[8] == '1') {
      // Autonomy requested, so obey autonomy's commands not the user commands.
      pthread_mutex_lock( &autonomyCommandMutex );
      strcpy(&copiedCommand[0], &autonomyCommand[0]);
      pthread_mutex_unlock( &autonomyCommandMutex );
    }

    if (copiedCommand[0] == '1') {
      // Forward
      sendCode(fwd_slow);
    } else if (copiedCommand[1] == '1') {
      // Reverse
      sendCode(rev_slow);
    } else if (copiedCommand[2] == '1') {
      //Left
      sendCode(left_fast);
    } else if (copiedCommand[3] == '1') {
      //Right
      sendCode(right_slow);
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
      sendCode(fire);
    } else {
      // Idle
      sendCode(neutral);
    }
  }
  
  return 0;
} // main


// Sends one individual opcode to the main tank controller
void sendCode(int code) {
  // Build up the header bytes and CRC
  int fullCode = 0;
  fullCode |= CRC(code) << 2;
  fullCode |= code << 6;
  fullCode |= 0xFE000000;
  
  // Send the initial high pulse
  GPIO_CLR = 1<<PIN;
  usleep(500);
  
  // Send the code itself, bit by bit using Manchester coding
  int i;
  for (i=0; i<32; i++) {
    int bit = (fullCode>>(31-i)) & 0x1;
    sendBit(bit);
  }
  
  // Force a 4ms gap between messages
  GPIO_SET = 1<<PIN;
  usleep(3333);
} // sendCode

// Calculates the CRC of a Heng Long opcode
int CRC(int data)
{
  int c;
  c = 0;
  c ^= data & 0x03;
  c ^= (data >> 2) & 0x0F;
  c ^= (data >> 6) & 0x0F;
  c ^= (data >> 10) & 0x0F;
  c ^= (data >> 14) & 0x0F;
  c ^= (data >> 18) & 0x0F;
  return c;
} // CRC

// Sends one individual bit using Manchester coding
// 1 = high-low, 0 = low-high. CLR and SET do the opposite of what you think
// due to the transistor circuit
void sendBit(int bit) {
  if (bit == 1) {
    GPIO_CLR = 1<<PIN;
    usleep(250);
    GPIO_SET = 1<<PIN;
    usleep(250);
  } else {
    GPIO_SET = 1<<PIN;
    usleep(250);
    GPIO_CLR = 1<<PIN;
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
  struct mg_callbacks callbacks;

  const char *options[] = {"listening_ports", "3000", NULL};
  memset(&callbacks, 0, sizeof(callbacks));
  callbacks.begin_request = http_callback;
  
  printf("Starting HTTP Server on port 3000\n");

  ctx = mg_start(&callbacks, NULL, options);
  getchar();  // Wait until user hits "enter".  This will never happen when this
              // code runs on the tank at startup
  mg_stop(ctx);
}

// HTTP server callback
static int http_callback(struct mg_connection *conn) {

  const struct mg_request_info *request_info = mg_get_request_info(conn);

  char* tempCommand = malloc(sizeof(char)*13);
  strncpy(&tempCommand[0], &request_info->query_string[0], 12);
  tempCommand[12] = 0;
  printf("Received command from HTTP: %.*s\n", 12, tempCommand);

  // Set received, so send it over to the control thread
  if ((tempCommand[0] == 's') && (tempCommand[1] == 'e') && (tempCommand[2] == 't')) {
    pthread_mutex_lock( &userCommandMutex );
    strncpy(&userCommand[0], &tempCommand[3], 9);
    pthread_mutex_unlock( &userCommandMutex );
    printf("Set motion command: %.*s\n", 9, userCommand);

    // Send an HTTP header back to the client
    mg_printf(conn, "HTTP/1.1 200 OK\r\n"
            "Content-Type: text/plain\r\n\r\n");
  }

  // Get received, so return sensor data
  else if ((tempCommand[0] == 'g') && (tempCommand[1] == 'e') && (tempCommand[2] == 't')) {
    // Get data from the variables while the mutex is locked
    pthread_mutex_lock( &sensorDataMutex );
    int tmpRange = range;
    int tmpBearing = bearing;
    int tmpPitch = pitch;
    int tmpRoll = roll;
    pthread_mutex_unlock( &sensorDataMutex );
    printf("Sensor data acquired.\n");

    // Prepare the response
    char response[100];
    int contentLength = snprintf(response, sizeof(response),
          "Range: %d   Bearing: %d   Pitch: %d   Roll: %d",
          tmpRange, tmpBearing, tmpPitch, tmpRoll);

    printf("Sending HTTP response: %s\n", response);

    // Send an HTTP response back to the client
    mg_printf(conn, "HTTP/1.1 200 OK\r\n"
            "Content-Type: text/plain\r\n"
            "Content-Length: %d\r\n"
            "\r\n"
            "%s",
            contentLength, response);
    
  }
  printf("Finished responding to HTTP request.\n");

  return 1;  // Mark as processed
}


// Launch sensor polling thread
void* launch_sensors() {
  printf("Starting sensor polling\n");
  while(1) {
    int fd;                           // File description
    char *fileName = "/dev/i2c-0";    // Name of the port we will be using
    int addressSRF = 0x70;            // Address of the SRF02 shifted right one bit
    int addressCMPS = 0x60;           // Address of CMPS10 shifted right one bit
    unsigned char buf[10];            // Buffer for data being read/ written on the i2c bus
    int tmpRange = 0;                 // Temp variable to store range
    int tmpBearing = 0;               // Temp variable to store bearing
    int tmpPitch = 0;                 // Temp variable to store pitch
    int tmpRoll = 0;                  // Temp variable to store roll
    char* message;                    // Char array to write an error message to
  
    if ((fd = open(fileName, O_RDWR)) < 0) {          // Open port for reading and writing
      message = "Failed to open i2c port";
    }
  
    //
    // RANGEFINDER
    //

    // Initial pause for safety
    usleep(50000);

    if (ioctl(fd, I2C_SLAVE, addressSRF) < 0) {         // Set the port options and set the address of the device we wish to speak to
      message = "Unable to get bus access to talk to slave";
    }
  
    buf[0] = 0;                         // Commands for performing a ranging
    buf[1] = 81;
  
    if ((write(fd, buf, 2)) != 2) {               // Write commands to the i2c port
      message = "Error writing to i2c slave\n";
    }
  
    usleep(750000);                       // This sleep waits for the ping to come back
  
    buf[0] = 0;                         // This is the register we wish to read from
  
    if ((write(fd, buf, 1)) != 1) {               // Send the register to read from
      message = "Error writing to i2c slave\n";
    }
  
    if (read(fd, buf, 4) != 4) {                // Read back data into buf[]
      message = "Unable to read from slave\n";
    } else {
      tmpRange = (buf[2] <<8) + buf[3];     // Calculate range as a word value
    }

    //
    // COMPASS
    //

    // Initial pause for safety
    usleep(50000);

    if (ioctl(fd, I2C_SLAVE, addressCMPS) < 0) {          // Set the port options and set the address of the device we wish to speak to
      message = "Unable to get bus access to talk to slave";
    }
  
    buf[0] = 0;                         // this is the register we wish to read from
  
    if ((write(fd, buf, 1)) != 1) {               // Send register to read from
      message = "Error writing to i2c slave\n";
    }
  
    if (read(fd, buf, 6) != 6) {                // Read back data into buf[]
      message = "Unable to read from slave\n";
    }
    else {
      tmpBearing = ((buf[2]<<8) + buf[3]) / 10;
      tmpPitch = buf[4];
      if (tmpPitch > 127) tmpPitch = tmpPitch-256;
      tmpRoll = buf[5];
      if (tmpRoll > 127) tmpRoll = tmpRoll-256;
    }

    // Output to file
    FILE* f = fopen("/var/www/sensordata.txt", "w");
    fprintf(f, "Range: %d&nbsp;&nbsp;&nbsp;&nbsp;Bearing: %d&nbsp;&nbsp;&nbsp;&nbsp;Pitch: %d&nbsp;&nbsp;&nbsp;&nbsp;Roll: %d \n", tmpRange, tmpBearing, tmpPitch, tmpRoll);
    fclose(f);

    pthread_mutex_lock( &sensorDataMutex );
    range = tmpRange;
    bearing = tmpBearing;
    pitch = tmpPitch;
    roll = tmpRoll;
    pthread_mutex_unlock( &sensorDataMutex );

  }
}


// Launch autonomy thread
void* launch_autonomy() {

  printf("Starting autonomy\n");
  printf("Autonomy: Driving forward.\n");
  while(1) {
    // Get data from the variables while the mutex is locked
    pthread_mutex_lock( &sensorDataMutex );
    int tmpRange = range;
    int tmpBearing = bearing;
    int tmpPitch = pitch;
    int tmpRoll = roll;
    pthread_mutex_unlock( &sensorDataMutex );

    // Check for forward obstacles.  Ranges <10 are errors, so ignore them.
    if ((tmpRange < 100) && (tmpRange > 10)) {
      printf("Autonomy: Forward obstacle detected.\n");
      autonomySendCommand("00000000");
      usleep(500000);
      printf("Autonomy: Reversing...\n");
      autonomySendCommand("02000000");
      usleep(1000000);
      autonomySendCommand("00000000");
      usleep(500000);
      printf("Autonomy: Shooting...\n");
      autonomySendCommand("00000001");
      usleep(500000);
      autonomySendCommand("00000000");
      printf("Autonomy: Turning...\n");
      autonomySendCommand("00010000");
      usleep(1500000);
      autonomySendCommand("00000000");
      printf("Autonomy: Recheck Pause...\n");
      usleep(2000000);
      printf("Autonomy: Recheck Pause complete.\n");
      printf("Autonomy: Driving forward.\n");
    }
    else
    {
      autonomySendCommand("10000000");
    }

    // Don't need to run that fast, sensor polling is pretty slow anyway.
    usleep(100000);
  }
}

// Send a command from autonomy to the main control thread
void* autonomySendCommand(char* cmd) {
  pthread_mutex_lock( &autonomyCommandMutex );
  strncpy(&autonomyCommand[0], &cmd[0], 8);
  autonomyCommand[9] = 0;
  pthread_mutex_unlock( &autonomyCommandMutex );
}
