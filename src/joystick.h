#include <js_input.h>

class Joystick {

	public:
		void Joystick(GAMEPAD_DEVICE controller);
		void update();
		nova_common::RawCtrl getMessage();
	protected:
		nova_common::RawCtrl msg_;
		GAMEPAD_DEVICE controller_;

		const float STICK_MAX_L_;
		const float STICK_MAX_R_;

		int stick_lx_;
		int stick_ly_;
		int stick_rx_;
		int stick_ry_;

        void correctForDeadzone();
        void setMessageValues();

}