#include <joystick.h>
#include <js_input.h>

class ArmJoystick : Joystick {

    protected:
        const float OFFSET;

        bool twist_lock_;
        bool hat_lock_;

        void setMessageValues();
}
