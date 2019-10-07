var drive_node;
var publishImmidiately = true;
var robot_IP;
var manager_X;
var manager_Y;
var teleop;
var ros;
var listener;

var RPM = 0.0;
var turn = 0.0;
var turn_range = 0.5;
var speed_range = 0.2;
var socket = io.connect('http://118.139.14.186:5000')
function moveAction() {
    var rpm = Math.floor(RPM * 100);
    var steer_pct = turn * 20;
    socket.emit('control/drive_cmd',{rpm:rpm, steer_pct:steer_pct}) //Emit a dictionary with both RPM and steer percent
}

function initSliders() {
  // Add event listener for slider moves
  //Scaler between 0.15-1 for full RPM
  robotSpeedRange = document.getElementById("robot-speed");
  robotSpeedRange.oninput = function() {
    speed_range = robotSpeedRange.value / 100;
    document.getElementById("speed-label").innerHTML =
      "" + robotSpeedRange.value + " RPM";
    socket.emit('control/speed_setting',{data:robotSpeedRange.value})
  };
  //Scaler between 0.15-1 for full turn
  robotTurnRange = document.getElementById("robot-turn");
  robotTurnRange.oninput = function() {
    turn_range = robotTurnRange.value / 100;
    document.getElementById("turn-label").innerHTML =
      "" + robotTurnRange.value + "%";
    socket.emit('control/turn_setting',{data:robotTurnRange.value})
  };
}

function createJoystickX() {
  // Check if joystick was aready created
  if (manager_X == null) {
    joystickContainer = document.getElementById("joystick-X");
    // joystck configuration
    var options = {
      zone: joystickContainer,
      position: { left: 50 + "%", top: 110 + "px" },
      mode: "static",
      size: 130,
      color: "#000000",
      restJoystick: true,
      lockY: true
    };
    manager_X = nipplejs.create(options);
    // event listener for joystick move
    manager_X.on("move", function(evt, nipple) {
      // turn 90 degrees to be facing upwards
      var direction = 90 - nipple.angle.degree;
      if (direction < -180) {
        direction = 450 - nipple.angle.degree;
      }
      // convert angles to radians and scale to RPM and turn
      RPM = Math.cos(direction / 57.29) * nipple.distance * 0.015 * speed_range;
      // events triggered earlier than 50ms after last publication will be dropped
      if (publishImmidiately) {
        publishImmidiately = false;
        moveAction();
        setTimeout(function() {
          publishImmidiately = true;
        }, 50);
      }
    });
    // event listener for joystick release, always send stop message
    manager_X.on("end", function() {
      RPM = 0;
      moveAction();
    });
  }
}

function createJoystickY() {
  // Check if joystick was aready created
  if (manager_Y == null) {
    joystickContainer = document.getElementById("joystick-Y");
    // joystck configuration
    var options = {
      zone: joystickContainer,
      position: { left: 50 + "%", top: 110 + "px" },
      mode: "static",
      size: 130,
      color: "#000000",
      restJoystick: true,
      lockX: true
    };
    manager_Y = nipplejs.create(options);
    // event listener for joystick move
    manager_Y.on("move", function(evt, nipple) {
      // turn 90 degrees to be facing upwards
      var direction = 90 - nipple.angle.degree;
      if (direction < -180) {
        direction = 450 - nipple.angle.degree;
      }
      // convert angles to radians and scale to RPM and turn
      turn = Math.sin(direction / 57.29) * nipple.distance * 0.078 * turn_range;
      // events triggered earlier than 50ms after last publication will be dropped
      if (publishImmidiately) {
        publishImmidiately = false;
        moveAction();
        setTimeout(function() {
          publishImmidiately = true;
        }, 50);
      }
    });
    // event listener for joystick release, always send stop message
    manager_Y.on("end", function() {
      turn = 0;
      moveAction();
    });
  }
}

window.onload = function() {
  // get handle for video placeholder
  //video = document.getElementById("video");
  // Populate video source
  /*video.src =
    "http://" +
    robot_IP +
    ":8080/stream?topic=/camera/rgb/image_raw&type=mjpeg&quality=80";

  */

  createJoystickY();
  createJoystickX();
  initSliders();
  setInterval(moveAction, 200); //Sends the drive command every .2 seconds
  socket.emit('control/init')

  socket.on('control/response', function(msg){console.log('5'); 
  document.getElementById("robot-speed").value = msg.speed;
  document.getElementById("speed-label").innerHTML = "" + msg.speed + " RPM";
  document.getElementById("robot-turn").value = msg.turn;
  document.getElementById("turn-label").innerHTML = "" + msg.turn + " %";})
};
