//
// Raspberry Tank HTTP Remote Control script
// Ian Renton, February 2013
// http://ianrenton.com
// 
// Based on the Raspberry Pi GPIO example by Dom and Gert
// (http://elinux.org/Rpi_Low-level_peripherals#GPIO_Driving_Example_.28C.29)
// ...and I2C Compass/Rangefinder code by James Henderson, from robot-electronics.co.uk
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
#include <linux/i2c-dev.h>
#include <assert.h>
#include <sys/ioctl.h>
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

// N.B. The tank's transistor setup actually inverts the GPIO output signal, so
// for example using GPIO_SET sets the GPIO output high, but the tank's RX18
// board sees that as a lot.
#define GPIO_SET *(gpio+7)  // sets   bits which are 1, ignores bits which are 0
#define GPIO_CLR *(gpio+10) // clears bits which are 1, ignores bits which are 0

// GPIO pin that connects to the Heng Long main board
// (Pin 7 is the top right pin on the Pi's GPIO, next to the yellow video-out)
#define PIN 7

// Heng Long tank bit-codes
int idle = 0xFE40121C;
int ignition = 0xFE401294;
int neutral = 0xFE3C0F00;
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
int fire = 0xFE3E1030; //0xFE442F34;
int machine_gun = 0xFE440F78;
int recoil = 0xFE420F24;

// Mutexes
pthread_mutex_t commandMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t sensorDataMutex = PTHREAD_MUTEX_INITIALIZER;

// Mutex-controlled Variables
char* command;
int range;
int bearing;
int pitch;
int roll;

// Function declarations
void setup_io();
void sendCode(int code);
void sendBit(int bit);
void* launch_server();
static int http_callback(struct mg_connection *conn);
void* launch_sensors();

// Main
int main(int argc, char **argv) { 
	
	printf("\nRaspberry Tank HTTP Remote Control script\nIan Renton, February 2013\nhttp://ianrenton.com\n\n");
	
	int g,rep,i;
	char inchar;
	command = malloc(sizeof(char)*10);
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
	
	// Loop, sending movement commands indefinitely
	while(1) {
		pthread_mutex_lock( &commandMutex );
		strcpy(&copiedCommand[0], &command[0]);
		pthread_mutex_unlock( &commandMutex );
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
	GPIO_CLR = 1<<PIN;
	usleep(500);
	
	// Send the code itself, bit by bit using Manchester coding
	int i;
	for (i=0; i<32; i++) {
		int bit = (code>>(31-i)) & 0x1;
		sendBit(bit);
	}
	
	// Force a 4ms gap between messages
	GPIO_SET = 1<<PIN;
	usleep(3333);
} // sendCode


// Sends one individual bit using Manchester coding
// 1 = high-low, 0 = low-high
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
} // launch_server

// HTTP server callback
static int http_callback(struct mg_connection *conn) {
	
  const struct mg_request_info *request_info = mg_get_request_info(conn);
	
  char* tempCommand = malloc(sizeof(char)*12);
  strncpy(&tempCommand[0], &request_info->query_string[0], 11);
  tempCommand[11] = 0;
  printf("Received command from HTTP: %.*s\n", 11, tempCommand);
	
  // Set received, so send it over to the control thread
  if ((tempCommand[0] == 's') && (tempCommand[1] == 'e') && (tempCommand[2] == 't')) {
		pthread_mutex_lock( &commandMutex );
		strncpy(&command[0], &tempCommand[3], 8);
		pthread_mutex_unlock( &commandMutex );
		printf("Set motion command: %.*s\n", 8, command);
		
		// Send an HTTP header back to the client
		mg_printf(conn, "HTTP/1.1 200 OK\r\n"
              "Content-Type: text/plain\r\n\r\n");
  }
	
  // Get received, so return sensor data
	// This is disabled as the Javascript on the web UI won't actually
	// fetch this data using a GET request as it is on a different port
	// to the web page that calls it - thus it is treated as a cross-
	// site scripting attack and disallowed.
	// Instead, the sensor read thread just dumps its data to a file in
	// /var/www for the web server to pick up. Horrible hack, but it
	// works.
  /*else if ((tempCommand[0] == 'g') && (tempCommand[1] == 'e') && (tempCommand[2] == 't')) {
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
	 
	 }*/
  printf("Finished responding to HTTP request.\n");
	
  return 1;  // Mark as processed
} // http_callback


// Launch sensor polling thread
void* launch_sensors() {
	printf("Starting sensor polling\n");
	while(1) {
		int fd;														// File description
		char *fileName = "/dev/i2c-0";    // Name of the port we will be using
		int addressSRF = 0x70;            // Address of the SRF02 shifted right one bit
		int addressCMPS = 0x60;           // Address of CMPS10 shifted right one bit
		unsigned char buf[10];            // Buffer for data being read/ written on the i2c bus
		int tmpRange = 0;                 // Temp variable to store range
		int tmpBearing = 0;               // Temp variable to store bearing
		int tmpPitch = 0;                 // Temp variable to store pitch
		int tmpRoll = 0;                  // Temp variable to store roll
		char* message;										// Char array to write an error message to
		
		if ((fd = open(fileName, O_RDWR)) < 0) {				// Open port for reading and writing
			message = "Failed to open i2c port";
		}
		
		//
		// RANGEFINDER
		//
		
		// Initial pause for safety
		usleep(50000);
		
		if (ioctl(fd, I2C_SLAVE, addressSRF) < 0) {		  // Set the port options and set the address of the device we wish to speak to
			message = "Unable to get bus access to talk to slave";
		}
		
		buf[0] = 0;													            // Commands for performing a ranging
		buf[1] = 81;
		
		if ((write(fd, buf, 2)) != 2) {								  // Write commands to the i2c port
			message = "Error writing to i2c slave\n";
		}
		
		usleep(750000);												          // This sleep waits for the ping to come back
		
		buf[0] = 0;													            // This is the register we wish to read from
		
		if ((write(fd, buf, 1)) != 1) {								  // Send the register to read from
			message = "Error writing to i2c slave\n";
		}
		
		if (read(fd, buf, 4) != 4) {								    // Read back data into buf[]
			message = "Unable to read from slave\n";
		} else {
			tmpRange = (buf[2] <<8) + buf[3];			        // Calculate range as a word value
		}
		
		//
		// COMPASS
		//
		
		// Initial pause for safety
		usleep(50000);
		
		if (ioctl(fd, I2C_SLAVE, addressCMPS) < 0) {		// Set the port options and set the address of the device we wish to speak to
			message = "Unable to get bus access to talk to slave";
		}
		
		buf[0] = 0;													            // this is the register we wish to read from
		
		if ((write(fd, buf, 1)) != 1) {								  // Send register to read from
			message = "Error writing to i2c slave\n";
		}
		
		if (read(fd, buf, 6) != 6) {								    // Read back data into buf[]
			message = "Unable to read from slave\n";
		}
		else {
			tmpBearing = ((buf[2]<<8) + buf[3]) / 10;
			tmpPitch = buf[4];
			if (tmpPitch > 127) tmpPitch = tmpPitch-256;
			tmpRoll = buf[5];
			if (tmpRoll > 127) tmpRoll = tmpRoll-256;
		}
		
		// Debug
		printf("SENSOR POLL   Range: %d   Bearing: %d   Pitch: %d   Roll: %d \n", tmpRange, tmpBearing, tmpPitch, tmpRoll);
		
		// Output to file for the web server to pick up
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
} // launch_sensors
