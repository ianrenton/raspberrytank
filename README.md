raspberrytank
=============

Program(s) used to control a Heng Long RC tank using a Raspberry Pi.

henglong_test.c is a program to exercise the tank's drive motors in sequence.

rt_ssh.c operates each of the tank's primary functions via text entry, e.g.
"W" followed by "Enter" makes the tank move forward a bit.  Designed to be
operated over SSH.

rt_http is a tank control program with an HTTP server built in (using mongoose).
It takes commands in a specific format on its port as HTTP GET requests, and
uses them to control the tank.  Designed for use with the Web UI.

web-ui is the frontend for rt_http.  It renders a web page that includes the
tank's video feed, plus buttons with Javascript that sends the appropriate
commands to rt_http.
