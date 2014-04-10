# THIS PROGRAM IS IN EARLY DEVELOPMENT
# AND DOES **NOT** WORK YET.

# I2C sensor reading works, but control of the tank does not.
# I think the problem may be in the poor accuracy of Python's
# time.sleep() as opposed to C's usleep().
# This may mean that a pure Python version will never work :(

# Raspberry Tank Control Script
# v2 - Python via WebSockets
# Ian Renton, April 2014
# http://raspberrytank.ianrenton.com

import smbus
import time
import threading
import Queue
import RPi.GPIO as GPIO

#################################
##            SETUP            ##
#################################

# Tank control codes
IDLE = 0xFE40121C
IGNITION = 0xFE401294
TURRET_LEFT = 0xFE408F0C
TURRET_ELEV = 0xFE404F3C

# I2C Setup
i2cBus = smbus.SMBus(0)
i2cCompassAddress = 0x60
i2cRangefinderAddress = 0x70

# GPIO Setup
GPIO_PIN = 7
GPIO.setmode(GPIO.BCM)
GPIO.setup(GPIO_PIN, GPIO.OUT)
GPIO.output(GPIO_PIN, True)

# Inter-thread communication queues
bearingQueue = Queue.Queue(1)
rangeQueue = Queue.Queue(1)



#################################
##          FUNCTIONS          ##
#################################

# Send a complete command code
def sendCode(code):
  # Initial high period to start packet
  GPIO.output(GPIO_PIN, False)
  time.sleep(0.000500)
  
  print "Sending " + hex(code)
  
  # Send the code itself, bit by bit
  for i in range(0, 32):
    bit = (code>>(31-i)) & 0x1;
    sendBit(bit);
  
  # Force a gap between messages
  GPIO.output(GPIO_PIN, True)
  time.sleep(0.004)
  
# Send a single bit using the tank's Manchester encoding scheme. The high-low and
# low-high transitions are inverted compared to normal because the transistor circuit
# I use sends 0v to the tank when the GPIO pin is high, and 4v when the GPIO pin is
# low.
def sendBit(bit):
  GPIO.output(GPIO_PIN, bit)
  time.sleep(0.000250)
  GPIO.output(GPIO_PIN, not bit)
  time.sleep(0.000250)

# Get the bearing of the tank from the Compass module, in degrees
def getBearing():
  bear1 = i2cBus.read_byte_data(i2cCompassAddress, 2)
  bear2 = i2cBus.read_byte_data(i2cCompassAddress, 3)
  bear = (bear1 << 8) + bear2
  bear = bear/10.0
  return bear

# Get the range to target from the Rangefinder module, in metres
def getRange():
  i2cBus.write_byte_data(i2cRangefinderAddress, 0, 0x51)
  time.sleep(0.7)
  range1 = i2cBus.read_byte_data(i2cRangefinderAddress, 2)
  range2 = i2cBus.read_byte_data(i2cRangefinderAddress, 3)
  range3 = (range1 << 8) + range2
  return range3/100.0



#################################
##           THREADS           ##
#################################

# Control thread. Passes on the requested control signal from the GUI or autonomy
# to the tank.
class ControlThread (threading.Thread):
  def __init__(self):
    threading.Thread.__init__(self)
    print "Control thread running"
    
  def run(self):
    for i in range(0, 100):
      sendCode(IDLE)
    print "A"
    for i in range(0, 100):
      sendCode(TURRET_ELEV)
    print "B"
    for i in range(0, 100):
      sendCode(IDLE)

# Sensor thread. Acquires bearing and range data as fast as it can, and puts the
# values in the bearing and range queues. 
class SensorThread (threading.Thread):
  def __init__(self):
    threading.Thread.__init__(self)
    print "Sensor thread running"
    
  def run(self):
    while True:
      tmpBearing = getBearing()
      tmpRange = getRange()
      threadLock.acquire()
      bearingQueue.put(tmpBearing)
      rangeQueue.put(tmpRange)
      bearing = tmpBearing
      threadLock.release()
      time.sleep(0.5) # Low values (0.1-0.2) cause the program to hang
      
# Autonomy thread. Checks range values and drives accordingly
class AutonomyThread (threading.Thread):
  def __init__(self):
    threading.Thread.__init__(self)
    print "Autonomy thread running"
    
  def run(self):
    tmpBearing = 0.0
    tmpRange = 0.0

    while True:
      threadLock.acquire()
      if not bearingQueue.empty():
        tmpBearing = bearingQueue.get()
      if not rangeQueue.empty():
        tmpRange = rangeQueue.get()
      threadLock.release()
      print "Autonomy checking Bearing: " + str(tmpBearing) + "  Range: " + str(tmpRange)
      time.sleep(1)



#################################
##        MAIN PROGRAM         ##
#################################

# Start threads
threadLock = threading.Lock()
threads = []
controlThread = ControlThread()
threads.append(controlThread)
controlThread.start()
sensorThread = SensorThread()
threads.append(sensorThread)
sensorThread.start()
autonomyThread = AutonomyThread()
threads.append(autonomyThread)
autonomyThread.start()

# Wait for threads to complete
for t in threads:
  t.join()
print "All threads finished, exiting"