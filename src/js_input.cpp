/*
	Refactoring of ljs_input, rjs_input, xbox_input cpp

*/

#include <js_input.h>
#include <joystick.h>
#include <arm_joystick.h>

int main(int argc, char **argv)
{
	// ros publisher code
	// CONFUSED
	ros::init(argc, argv, "joysticks_input");
	ros::NodeHandle n;
	ros::Rate loop_rate(LOOP_HZ);


	// Intialise the gamepad library
	GamepadInit();

	// construct each controller instance
	Joystick xbox;
	ArmJoystick ljs;
	ArmJoystick rjs;

	//ros ok loop. 
	while (ros::ok())
	{
		// Updates the state of the gamepad
		GamepadUpdate() 

		//update the status of each controller
		xbox.update(GAMEPAD_0, xbox_updates_array);
		ljs.update(GAMEPAD_1, ljs_updates_array);
		rjs.update(GAMEPAD_2, rjs_updates_array);

		// publish the ros messages
		raw_ctr_pub.publish(xbox.getMessage());
		raw_ctr_pub.publish(ljs.getMessage());
		raw_ctr_pub.publish(rjs.getMessage());

		ros::spinOnce();
		loop_rate.sleep();

	}



}



