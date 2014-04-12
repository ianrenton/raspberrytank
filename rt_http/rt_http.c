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

// HENG LONG TANK OPCODES:
// We don't yet fully understand how the opcodes we send to the tank work. Specifically,
// we don't understand how to control the speed and direction of the main motors
// properly, but we do understand how to control just about everything else. Therefore
// we have ended up with a system of "base opcodes" - premade codes for the stuff we
// don't understand, but we know works - and "delta opcodes" - the bits we can "and" on
// top of the base opcodes to trigger the extra functions that we do understand.
// More info: raspberrytank.ianrenton.com/day-30-cracking-the-code-third-time-luckier/

// BASE OPCODES
const int IDLE =         0x1003c;
const int FORWARD =      0x0803c;
const int REVERSE =      0x1803c; // Must be cancelled by a "forward", idle is not enough
const int LEFT =         0x10010; // Slower than I would like
const int RIGHT =        0x10064;

// DELTA OPCODES
const int MG_LED =       0x0001;
const int IGNITION =     0x0002;
const int FIRE =         0x0080;
const int TURRET_ELEV =  0x0100;
const int TURRET_LEFT =  0x0200;
const int TURRET_RIGHT = 0x0400;
const int RECOIL =       0x0800;
const int MG_SOUND =     0x1000;

///////////////////////////////////

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
int buildOpCode(char* cmd);
void sendOpCode(int code);
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
  userCommand = malloc(sizeof(char)*11);
  autonomyCommand = malloc(sizeof(char)*11);
  char* copiedCommand = malloc(sizeof(char)*11);

  // Set up gpio pointer for direct register access
  setup_io();

  // Switch the relevant GPIO pin to output mode
  INP_GPIO(PIN); // must use INP_GPIO before we can use OUT_GPIO
  OUT_GPIO(PIN);

  // Set all GPIO outputs high (this is a low when the tank sees it, which is
  // the reset state.
  GPIO_SET = 1<<PIN;
  
  // Send the idle and ignition codes
  printf("Waiting for ignition...\n");
  for (i=0; i<40; i++) 
  {
    sendOpCode(buildOpCode("0000000000")); // idle
  }
  for (i=0; i<10; i++) 
  {
    sendOpCode(buildOpCode("0000000010")); // ignition
  }
  for (i=0; i<300; i++) 
  {
    sendOpCode(buildOpCode("0000000000")); // idle
  }
  printf("Ignition sequence finished.\n");
  
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

    if (copiedCommand[9] == '1') {
      // Autonomy requested, so obey autonomy's commands not the user commands.
      pthread_mutex_lock( &autonomyCommandMutex );
      strcpy(&copiedCommand[0], &autonomyCommand[0]);
      pthread_mutex_unlock( &autonomyCommandMutex );
    }
    
    sendOpCode(buildOpCode(copiedCommand));
  }
  
  return 0;
} // main


// Takes a command from the web UI or autonomy (like "001000010" for "turn left and 
// fire") and builds a Heng Long format binary opcode to send to the tank.
int buildOpCode(char* cmd) {

  int opCode = 0;
  
  // The first four characters represent the motion of the tank. These are used to
  // select the "base opcode" (the bit we use without properly understanding it).
  // Because we use this "base opcode" fudge we can only select at most one of these
  // directions.
  // 0000 = idle  1000 = forwards   0100 = reverse   0010 = left    0001 = right
  if (cmd[0] == '1') {
    opCode = FORWARD;
  } else if (cmd[1] == '1') {
    opCode = REVERSE;
  } else if (cmd[2] == '1') {
    opCode = LEFT;
  } else if (cmd[3] == '1') {
    opCode = RIGHT;
  } else {
    opCode = IDLE;
  }
  
  // Now we check the other characters in the string to see what they're demanding
  // anything. These are the features in the opcode that we do understand - we can
  // just set certain bits high to achieve what we want (the "delta opcode"s). This
  // means we can have several of these active at once if we want.
  // char 4 = turret left   char 5 = turret right   char 6 = turret elevate
  // char 7 = fire          char 8 = ignition
  if (cmd[4] == '1') {
    opCode = opCode | TURRET_LEFT;
  }
  if (cmd[5] == '1') {
    opCode = opCode | TURRET_RIGHT;
  }
  if (cmd[6] == '1') {
    opCode = opCode | TURRET_ELEV;
  }
  if (cmd[7] == '1') {
    opCode = opCode | FIRE;
  }
  if (cmd[8] == '1') {
    opCode = opCode | IGNITION;
  }
  
  return opCode;
} // buildOpCode


// Sends one individual opcode to the main tank controller
void sendOpCode(int code) {
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

  char* tempCommand = malloc(sizeof(char)*14);
  strncpy(&tempCommand[0], &request_info->query_string[0], 13);
  tempCommand[13] = 0;
  //printf("Received command from HTTP: %.*s\n", 13, tempCommand);

  // Set received, so send it over to the control thread
  if ((tempCommand[0] == 's') && (tempCommand[1] == 'e') && (tempCommand[2] == 't')) {
    pthread_mutex_lock( &userCommandMutex );
    strncpy(&userCommand[0], &tempCommand[3], 10);
    pthread_mutex_unlock( &userCommandMutex );
    //printf("Set motion command: %.*s\n", 10, userCommand);

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

    //printf("Sending HTTP response: %s\n", response);

    // Send an HTTP response back to the client
    mg_printf(conn, "HTTP/1.1 200 OK\r\n"
            "Content-Type: text/plain\r\n"
            "Content-Length: %d\r\n"
            "\r\n"
            "%s",
            contentLength, response);
    
  }
  //printf("Finished responding to HTTP request.\n");

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
  //printf("Autonomy: Driving forward.\n");
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
      //printf("Autonomy: Forward obstacle detected.\n");
      autonomySendCommand("000000000"); // idle
      usleep(500000);
      //printf("Autonomy: Reversing...\n");
      autonomySendCommand("010000000"); // reverse
      usleep(1000000);
      autonomySendCommand("000000000"); // idle
      usleep(500000);
      //printf("Autonomy: Shooting...\n");
      autonomySendCommand("000000010"); // fire
      usleep(500000);
      autonomySendCommand("000000000"); // idle
      //printf("Autonomy: Turning...\n");
      autonomySendCommand("000100000"); // right
      usleep(1500000);
      autonomySendCommand("000000000"); // idle
      //printf("Autonomy: Recheck Pause...\n");
      usleep(2000000);
      //printf("Autonomy: Recheck Pause complete.\n");
      //printf("Autonomy: Driving forward.\n");
    }
    else
    {
      autonomySendCommand("100000000");
    }

    // Don't need to run that fast, sensor polling is pretty slow anyway.
    usleep(100000);
  }
}

// Send a command from autonomy to the main control thread
void* autonomySendCommand(char* cmd) {
  pthread_mutex_lock( &autonomyCommandMutex );
  strncpy(&autonomyCommand[0], &cmd[0], 9);
  autonomyCommand[10] = 0;
  pthread_mutex_unlock( &autonomyCommandMutex );
}
