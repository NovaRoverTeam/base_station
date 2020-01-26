#include <joystick.h>

void Joystick::Joystick(GAMEPAD_DEVICE controller) {

	STICK_MAX_L_ = 32767 - GAMEPAD_DEADZONE_LEFT_STICK;
	STICK_MAX_R_ = 32767 - GAMEPAD_DEADZONE_RIGHT_STICK;

	stick_lx_ = 0.0;
	stick_ly_ = 0.0;
	stick_rx_ = 0.0;
	stick_ry_ = 0.0;

    controller_ = controller;
}

void Joystick::update() {

	// grab stick values
    GamepadStickXY(controller_, STICK_LEFT, &stick_lx_, &stick_ly_);
    GamepadStickXY(controller_, STICK_LEFT, &stick_rx_, &stick_ry_);

	// correct for deadzone
    correctForDeadzone();
	// set all message values
    setMessageValues();
}

void Joystick::correctForDeadzone() {

    stick_lx_ = sgn(stick_lx_)*((float) abs(stick_lx_) - GAMEPAD_DEADZONE_LEFT_STICK)/stick_max_l_;
    stick_ly_ = sgn(stick_ly_)*((float) abs(stick_ly_) - GAMEPAD_DEADZONE_LEFT_STICK)/stick_max_l_;

    stick_rx_ = sgn(stick_rx_)*((float) abs(stick_rx_) - GAMEPAD_DEADZONE_RIGHT_STICK)/stick_max_r_;
    stick_ry_ = sgn(stick_ry_)*((float) abs(stick_ry_) - GAMEPAD_DEADZONE_RIGHT_STICK)/stick_max_r_;

}

void Joystick::setMessageValues() {

    // check connected
    msg_.connected = GamepadIsConnected(controller_);

    // Set the values in the ROS msg
    msg_.axis_lx_val = stick_lx_; 
    msg_.axis_ly_val = stick_ly_;   
    msg_.axis_rx_val = stick_rx_; 
    msg_.axis_ry_val = stick_ry_;     

    msg_.start_trg = GamepadButtonTriggered(controller_, BUTTON_START);
    msg_.back_trg = GamepadButtonTriggered(controller_, BUTTON_BACK);

    msg_.but_x_trg = GamepadButtonTriggered(controller_, BUTTON_X);
    msg_.but_y_trg = GamepadButtonTriggered(controller_, BUTTON_Y);
    msg_.but_a_trg = GamepadButtonTriggered(controller_, BUTTON_A);
    msg_.but_b_trg = GamepadButtonTriggered(controller_, BUTTON_B);

    msg_.bump_l_dwn = GamepadButtonDown(controller_, BUTTON_LEFT_SHOULDER);
    msg_.bump_r_dwn = GamepadButtonDown(controller_, BUTTON_RIGHT_SHOULDER);

    msg_.trig_l_val = GamepadTriggerLength(controller_, TRIGGER_LEFT);
    msg_.trig_r_val = GamepadTriggerLength(controller_, TRIGGER_RIGHT);

    msg_.trig_l_dwn = GamepadTriggerDown(controller_, TRIGGER_LEFT);
    msg_.trig_r_dwn = GamepadTriggerDown(controller_, TRIGGER_RIGHT);

    bool dpad_l = GamepadButtonDown(controller_, BUTTON_DPAD_LEFT);
    bool dpad_r = GamepadButtonDown(controller_, BUTTON_DPAD_RIGHT);

    bool dpad_u = GamepadButtonDown(controller_, BUTTON_DPAD_UP);
    bool dpad_d = GamepadButtonDown(controller_, BUTTON_DPAD_DOWN);

    msg_.axis_dx_dwn = dpad_r - dpad_l; // Left -1, none/both 0, right 1
    msg_.axis_dy_dwn = dpad_u - dpad_d; // Down -1, none/both 0, up 1
}

nova_common::RawCtrl Joystick::getMessage() {
    return msg_;
}