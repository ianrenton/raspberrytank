//
// Heng Long Tiger I tank controller for Raspberry Pi
// Ian Renton, June 2012
// http://ianrenton.com
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
//#include <ncurses.h>

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
int idle = 0xFE3C0F00;
int ignition = 0xFE3C10B0;
int neutral = 0xFE3C0F00;
int machine_gun = 0xFE3C107C;
int cannon = 0xFE3E1030;
int turret_left = 0xFE3C9018;
int turret_right = 0xFE3D103C;
int turret_down = 0xFE3C5028;
int turret_fire = 0xFE3C3030;
int fwd_slow = 0xFE301008;
int fwd_fast = 0xFE1C1030;
int rev_slow = 0xFE4C1024; //0xFE3C1038;
int right_slow = 0xFE3C1620; //0xFE3C152C;
int left_slow = 0xFE3C081C; //0xFE3C0B10;

// Function declarations
void setup_io();
void sendCode(int code);
void sendBit(int bit);

// Main
int main(int argc, char **argv) { 

  int g,rep,i;
  char inchar;

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
  
  // Loop, sending movement commands indefinitely
  do {
    printf("Ready: ");
    inchar = getchar();
    if (inchar == 'w') {
      printf("Forward\n");
      for (i=0; i<100; i++)
      {
        sendCode(fwd_fast);
      }
      sendCode(idle);
    }
    else if (inchar == 's') {
      printf("Reverse\n");
      for (i=0; i<100; i++)
      {
        sendCode(rev_slow);
      }
      sendCode(idle);
    }
    else if (inchar == 'a') {
      printf("Left\n");
      for (i=0; i<50; i++)
      {
        sendCode(left_slow);
      }
      sendCode(idle);
    }
    else if (inchar == 'd') {
      printf("Right\n");
      for (i=0; i<50; i++)
      {
        sendCode(right_slow);
      }
      sendCode(idle);
    }
  } while (inchar != '.');

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
  usleep(600);
  
  // Send the code itself, bit by bit using Manchester coding
  int i;
  for (i=0; i<32; i++) {
    int bit = (code>>(31-i)) & 0x1;
    sendBit(bit);
  }
  
  // Force a 4ms gap between messages
  GPIO_CLR = 1<<PIN;
  usleep(4000);
} // sendCode


// Sends one individual bit using Manchester coding
// 1 = high-low, 0 = low-high
void sendBit(int bit) {
  //printf("%d", bit);

  if (bit == 1) {
    GPIO_SET = 1<<PIN;
    usleep(300);
    GPIO_CLR = 1<<PIN;
    usleep(300);
  } else {
    GPIO_CLR = 1<<PIN;
    usleep(300);
    GPIO_SET = 1<<PIN;
    usleep(300);
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
