var drive_cmd;
var drive_node;
var publishImmidiately = true;
var robot_IP;
var manager_X;
var manager_Y;
var manager_Z;
var teleop;
var ros;
var listener;

var llx = 0.0;
var lly = 0.0;
var ltl = 0.0;
var ltr = 0.0;

var rlx = 0.0;
var rly = 0.0;
var rtl = 0.0;
var rtr = 0.0;

var threshold = 0.2;
var turn_range = 0.5;
var speed_range = 0.2;
var socket = io.connect('http://118.139.14.186:5000')
function moveAction() {
    socket.emit('arm/arm_cmd',{llx:llx,lly:lly,ltl:ltl,ltr:ltr,rlx:rlx,rly:rly,rtl:rtl,rtr:rtr})
}

function createJoystickX() {
  // Check if joystick was aready created
  if (manager_X == null) {
    joystickContainer = document.getElementById("joystick-X");
    // joystck configuration
    var options = {
      zone: joystickContainer,
      position: { left: 50 + "%", top: 80 + "px" },
      mode: "static",
      threshold: 0.4,
      size: 130,
      color: "#000000",
      restJoystick: true,
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
      ltl = Math.sin(direction / 57.29) * nipple.distance * 0.015 * 1;
      llx = Math.cos(direction / 57.29) * nipple.distance * 0.015 * 1;
      // events triggered earlier than 50ms after last publication will be dropped
      if (Math.abs(ltl)<threshold){ltl = 0.0;}
      if (Math.abs(llx)<threshold){llx = 0.0;}
      
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
      ltl = 0;
      llx = 0;
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
      position: { left: 47 + "%", top: 80 + "px" },
      mode: "static",
      threshold: 0.4,
      size: 130,
      color: "#000000",
      restJoystick: true,

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
      rtl = Math.sin(direction / 57.29) * nipple.distance * 0.015 * 1;
      lly = Math.cos(direction / 57.29) * nipple.distance * 0.015 * 1;
      
      if (Math.abs(rtl)<threshold){rtl = 0.0;}
      if (Math.abs(lly)<threshold){lly = 0.0;}
      
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
      rtl = 0;
      lly=0;
      moveAction();
    });
  }
}

function createJoystickZ() {
  // Check if joystick was aready created
  if (manager_Z == null) {
    joystickContainer = document.getElementById("joystick-Z");
    // joystck configuration
    var options = {
      zone: joystickContainer,
      position: { left: 50 + "%", top: 80 + "px" },
      mode: "static",
      size: 130,
      color: "#000000",
      restJoystick: true,
      threshold: 0.4
    };
    manager_Z = nipplejs.create(options);
    // event listener for joystick move
    manager_Z.on("move", function(evt, nipple) {
      // turn 90 degrees to be facing upwards
      var direction = 90 - nipple.angle.degree;
      if (direction < -180) {
        direction = 450 - nipple.angle.degree;
      }
      // convert angles to radians and scale to RPM and turn
      rlx = Math.sin(direction / 57.29) * nipple.distance * 0.015 * 1;
      rly = Math.cos(direction / 57.29) * nipple.distance * 0.015 * 1;
      
      if (Math.abs(rlx)<threshold){rlx = 0.0;}
      if (Math.abs(rly)<threshold){rly = 0.0;}
      
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
    manager_Z.on("end", function() {
      rlx = 0;
      rly = 0;
      moveAction();
    });
  }
}

window.onload = function() {
  // IP address of the ros-bridge-server
  robot_IP = "118.139.3.173";

  // // Init handle for rosbridge_websocket
  ros = new ROSLIB.Ros({
    url: "ws://" + robot_IP + ":9090" //ros-bridge-server by default runs on port 9090
  });

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
  createJoystickZ();
  setInterval(moveAction, 100);
 }
