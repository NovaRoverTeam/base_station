#include <joystick.h>
#include <js_input.h>
#include <arm_joystick.h>

void ArmJoystick::setMessageValues() {

	msg_.connected = GamepadIsConnected(controller_); // Check Xbox connection

	if (msg_.connected)
	{	
	    // Set the values in the ROS msg_
	    msg_.axis_lx_val = stick_lx_; 
	    msg_.axis_ly_val = stick_ly_;   
	    msg_.axis_rx_val = stick_rx_; 
	    msg_.axis_ry_val = stick_ry_;     

	    msg_.start_trg = GamepadButtonTriggered(controller_, BUTTON_START);
	    msg_.back_trg = GamepadButtonTriggered(controller_, BUTTON_BACK);

	    msg_.but_x_trg = GamepadButtonTriggered(controller_, BUTTON_X);
	    msg_.but_y_trg = GamepadButtonTriggered(controller_, BUTTON_Y);
	    msg_.but_a_trg = GamepadButtonTriggered(controller_, BUTTON_A);
	    msg_.but_b_trg = GamepadButtonDown(controller_, BUTTON_B);

	    msg_.bump_l_dwn = GamepadButtonDown(controller_, BUTTON_LEFT_SHOULDER);
	    msg_.bump_r_dwn = GamepadButtonDown(controller_, BUTTON_RIGHT_SHOULDER);

	    // left
	    if (twist_lock_ and GamepadTriggerLength(controller_, TRIGGER_LEFT) < 0.1)
	    {
	        msg_.trig_l_val = 0.0;
	    }
	    else
	    {
			msg_.trig_l_val = GamepadTriggerLength(controller_, TRIGGER_LEFT) - OFFSET;
			msg_.trig_l_val = (msg_.trig_l_val > 0.0) ? msg_.trig_l_val/(1 - OFFSET): msg_.trig_l_val/(OFFSET);
	      
			if (abs(msg_.trig_l_val) < 0.01)
			{
				msg_.trig_l_val = 0.0;
			}

	      	twist_lock_ = false;
	    }
	    // right
	    if (hat_lock_ and GamepadTriggerLength(controller_, TRIGGER_RIGHT) < 0.1)
	    {
	    	msg_.trig_r_val = 0.0;
	    }
	    else
	    {
			msg_.trig_r_val = GamepadTriggerLength(controller_, TRIGGER_RIGHT)-OFFSET;
			msg_.trig_r_val = (msg_.trig_r_val>0.0) ? msg_.trig_r_val/(1-OFFSET): msg_.trig_r_val/(OFFSET); //Re-scale OFFSET value
		   	if (abs(msg_.trig_r_val) < 0.01) // Get rid of tiny floats
		    { 
		    	msg_.trig_r_val = 0.0;
		    }
			hat_lock_ = false;
	    }

	    //When the joystick is first connected the twist and hat give a 0.0 until moved, whereas their actual centre is 0.435. This ensures that they have been moved first so they don't make a full negative power to twist and hat on connection
	    msg_.trig_l_dwn = GamepadTriggerDown(controller_, TRIGGER_LEFT);
	    msg_.trig_r_dwn = GamepadTriggerDown(controller_, TRIGGER_RIGHT);

	    bool dpad_l = GamepadButtonDown(controller_, BUTTON_DPAD_LEFT);
	    bool dpad_r = GamepadButtonDown(controller_, BUTTON_DPAD_RIGHT);

	    bool dpad_u = GamepadButtonDown(controller_, BUTTON_DPAD_UP);
	    bool dpad_d = GamepadButtonDown(controller_, BUTTON_DPAD_DOWN);

	    msg_.axis_dx_dwn = dpad_r - dpad_l; // Left -1, none/both 0, right 1
	    msg_.axis_dy_dwn = dpad_u - dpad_d; // Down -1, none/both 0, up 1
    }
    else
    {
	    msg_.axis_lx_val = 0.0;
	    msg_.axis_ly_val = 0.0;
	    msg_.trig_l_val = 0.0;
	    msg_.trig_r_val = 0.0;
	    twist_lock_ = true;
	    hat_lock_ = true;
    }

}