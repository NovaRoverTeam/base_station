
//--**-- ROS includes
#include "ros/ros.h"
#include <ros/console.h>
#include <nova_common/RawCtrl.h>

//--**-- General includes
#include <gamepad/gamepad.h>

//--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
// sgn():
//
//    Returns the sign of the input float (-1, 0 or 1).
//--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
int sgn(float val) {
    return (0.0 < val) - (val < 0.0);
}

