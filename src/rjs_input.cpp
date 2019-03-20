//--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
// File: input.cpp
// Author: Ben Steer
// Last modified by: Ben Steer
//
// Description:
//  This is a ROS node to handle the input from an Xbox controller. It
//  reads values from the sticks, triggers and buttons and distributes
//  them into the ROS network as "RawCtrl" messages.
//  
//--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--


//--**-- ROS includes
#include "ros/ros.h"
#include <ros/console.h>
#include <base_station/RawCtrl.h>

//--**-- General includes
#include <gamepad/gamepad.h>

//--**-- Constants definitions
#define LOOP_HZ 10


//--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
// sgn():
//
//    Returns the sign of the input integer (-1, 0 or 1).
//--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
int sgn(int val) {
    return (0 < val) - (val < 0);
}


//--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
// main():
//
//    Main function.
//--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--
int main(int argc, char **argv)
{
  ros::init(argc, argv, "rjs_input");
  ros::NodeHandle n;
  ros::Rate loop_rate(LOOP_HZ);

  ros::Publisher raw_ctrl_pub = 
    n.advertise<base_station::RawCtrl>("/base_station/rjs_raw_ctrl", 1);
 
  GamepadInit(); // Initialise the Xbox gamepad
  
  // Deadzone compensation - see more info in main ROS loop below
  const int dead_l = GAMEPAD_DEADZONE_LEFT_STICK;
  const int dead_r = GAMEPAD_DEADZONE_RIGHT_STICK;
  const float stick_max_l = 32767 - dead_l;
  const float stick_max_r = 32767 - dead_r;

  // Initialising variables to be refreshed during loop
  int stick_lx = 0;
  int stick_ly = 0;
  int stick_rx = 0;
  int stick_ry = 0;

  while (ros::ok())
  {
    GamepadUpdate(); // Updates the state of the gamepad

    base_station::RawCtrl msg; // Msg to use for stick vals
    GAMEPAD_DEVICE controller = GAMEPAD_1;
    msg.connected = GamepadIsConnected(controller); // Check Xbox connection

    // Grab the stick values
    GamepadStickXY(controller, STICK_LEFT, &stick_lx, &stick_ly);
    GamepadStickXY(controller, STICK_RIGHT, &stick_rx, &stick_ry);

    // The gamepad sticks have a deadzone - which means for a small amount of
    // movement of the stick, the reading remains at zero. This means as soon
    // as you move the stick out of the deadzone, the reading will jump from 
    // zero to some higher value. The calculations here account for this and 
    // rescale the values to remove this jump.
    float f_stick_lx = sgn(stick_lx)*((float) abs(stick_lx) - dead_l)/stick_max_l;
    float f_stick_ly = sgn(stick_ly)*((float) abs(stick_ly) - dead_l)/stick_max_l;

    float f_stick_rx = sgn(stick_rx)*((float) abs(stick_rx) - dead_l)/stick_max_r;
    float f_stick_ry = sgn(stick_ry)*((float) abs(stick_ry) - dead_l)/stick_max_r;

    // Set the values in the ROS msg
    msg.axis_lx_val = f_stick_lx; 
    msg.axis_ly_val = f_stick_ly;   
    msg.axis_rx_val = f_stick_rx; 
    msg.axis_ry_val = f_stick_ry;     

    msg.start_trg = GamepadButtonTriggered(controller, BUTTON_START);
    msg.back_trg = GamepadButtonTriggered(controller, BUTTON_BACK);

    msg.but_x_trg = GamepadButtonTriggered(controller, BUTTON_X);
    msg.but_y_trg = GamepadButtonTriggered(controller, BUTTON_Y);
    msg.but_a_trg = GamepadButtonTriggered(controller, BUTTON_A);
    msg.but_b_trg = GamepadButtonTriggered(controller, BUTTON_B);

    msg.bump_l_dwn = GamepadButtonDown(controller, BUTTON_LEFT_SHOULDER);
    msg.bump_r_dwn = GamepadButtonDown(controller, BUTTON_RIGHT_SHOULDER);

    msg.trig_l_val = GamepadTriggerLength(controller, TRIGGER_LEFT);
    msg.trig_r_val = GamepadTriggerLength(controller, TRIGGER_RIGHT);

    msg.trig_l_dwn = GamepadTriggerDown(controller, TRIGGER_LEFT);
    msg.trig_r_dwn = GamepadTriggerDown(controller, TRIGGER_RIGHT);

    bool dpad_l = GamepadButtonDown(controller, BUTTON_DPAD_LEFT);
    bool dpad_r = GamepadButtonDown(controller, BUTTON_DPAD_RIGHT);

    bool dpad_u = GamepadButtonDown(controller, BUTTON_DPAD_UP);
    bool dpad_d = GamepadButtonDown(controller, BUTTON_DPAD_DOWN);

    msg.axis_dx_dwn = dpad_r - dpad_l; // Left -1, none/both 0, right 1
    msg.axis_dy_dwn = dpad_u - dpad_d; // Down -1, none/both 0, up 1

    raw_ctrl_pub.publish(msg); // Publish the ROS msg
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}

