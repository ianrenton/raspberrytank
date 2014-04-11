/* Raspberry Tank Web UI JavaScript
   Written by Ian Renton (http://ianrenton.com), February 2013
   Released into the public domain without licence. */

// All commands that could be sent to the vehicle.
var command = {
  'forward' : false,
  'reverse' : false,
  'left' : false,
  'right' : false,
  'turret_left' : false,
  'turret_right' : false,
  'turret_elev' : false,
  'fire' : false,
  'ignition' : false,
  'autonomy' : false
}

// Port on which the tank's control server runs
var CONTROL_PORT = 3000;

// Port on which the mjpg-streamer webcam server runs
var WEBCAM_PORT = 8080;

// Executes on page load.
function load() {
  createImageLayer();
  setInterval(updateSensorData, 1000);
}

// Sets a command to either true or false by name, e.g. to go forwards use
// set('forwards', true) and to stop going forwards, use set('forwards', false).
function set(name, value) {
  command[name] = value;
  send();
  return true;
}

// Toggles the state of autonomy.
function toggleAutonomy() {
  if (command['autonomy'] == true) {
    command['autonomy'] = false;
    $('span.autonomystate').html("OFF");
    $('span.autonomybutton').html("Switch ON");
  } else {
    command['autonomy'] = true;
    $('span.autonomystate').html("ON");
    $('span.autonomybutton').html("Switch OFF");
  }
  send();
}

// Set all commands to false, in case there's been a glitch and something is
// stuck on.
function stop() {
  for (var name in command) {
    command[name] = false;
  }
  send();
}

// Send the current command set to the vehicle.
function send() {
  var commandBits = "";
  for (var name in command) {
    commandBits = commandBits + (command[name] ? "1" : "0");
  }
  $.get(window.location.protocol+'//'+window.location.host + ':' + CONTROL_PORT + "?set" + commandBits);
}

// Gets the sensor data
function updateSensorData() {
  $.get(window.location.protocol+'//'+window.location.host + "/sensordata.txt", "", function(data){
    if (data != "") {
      $('div.data').html("<h1>" + data + "</h1>");
    }
    else {
//      $('div.data').html("<h1>-</h1>");
    }
  }, "html");
}

// FPS mode key input
function keydown(e) {
  keychanged(e, true);
}

function keyup(e) {
  keychanged(e, false);
}


function keychanged(e, val) {
  switch (e.keyCode) {
    case 87:
      set('forward', val);
      break;
    case 83:
      set('reverse', val);
      break;
    case 65:
      set('left', val);
      break;
    case 68:
      set('right', val);
      break;
    case 81:
      set('turret_left', val);
      break;
    case 69:
      set('turret_right', val);
      break;
    case 90:
      set('turret_elev', val);
      break;
    case 32:
      set('fire', val);
      break;
    case 73:
      set('ignition', val);
      break;
    case 13:
      stop();
      break;
  }
}
