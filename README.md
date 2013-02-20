raspberrytank
=============

Program(s) used to control a Heng Long RC tank using a Raspberry Pi.  For more
information on the Raspberry Tank project, go to http://bit.ly/raspberrytank

rt_http
-------

rt_http is a tank control program with an HTTP server built in (using mongoose).
It takes commands in a specific format on its port as HTTP GET requests, and
uses them to control the tank.  It also reads data from an ultrasonic
rangefinder and compass module over I2C.

You can compile it by running "make" in its directory, assuming you have make
installed.  You also need libpthreads, and kernel modules for I2C if you
intend to use the I2C devices.  It should build mongoose automatically,
if not, run "cd mongoose && make linux".

You need to run it as root so that it can talk to the GPIO pins. ("sudo ./rt_http")

It was designed for use with the Web UI, though you can probably figure out
how to use it without :)

web-ui
------

web-ui is the frontend for rt_http.  It renders a web page that includes the
tank's video feed, plus buttons with Javascript that sends the appropriate
commands to rt_http, and the readings from the sensors.

The web UI comes in two flavours - a laptop/non-touch flavour where tank
commands are activated by holding the mouse down on an icon, and a phone/
tablet flavour where you just click an icon then click Stop to stop.
(Touch devices tend to do a "right-click" action when you click-and-hold,
so this form of interaction doesn't work very well there.

You can run it with any web server such as lighttpd.  If your document root
isn't /var/www, you'll need to edit the directory that rt_http writes its
sensordata.txt file too.

Old Stuff
---------

henglong_test is a program to exercise the tank's drive motors in sequence.

rt_ssh operates each of the tank's primary functions via text entry, e.g.
"W" followed by "Enter" makes the tank move forward a bit.  Designed to be
operated over SSH.
