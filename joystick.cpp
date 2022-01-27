#include <util/atomic.h>

#include "log.h"
#include "joystick.h"
#include "jc_dualshock.h"
#include "jc_guitarhero.h"


Joystick::~Joystick()
{
    delete p_jimpl_;
};


int Joystick::init_joystick(uint8_t clock_pin, uint8_t command_pin,
                            uint8_t attention_pin, uint8_t data_pin)
{
    // GamePad(clock, command, attention, data, Pressures?, Rumble?) 
    control_error_ = control_.config_gamepad(clock_pin, command_pin, attention_pin, data_pin, true, true);

    if ((control_error_ != jerr_success) && (control_error_ != Joystick::jerr_controller_refuse_pressures_mode)) return control_error_;

    switch (control_type_ = control_.readType())
    {
        case 0:
            return jerr_unknown_controller_type;
        break;
        case DualShockJC::controller_type:
            p_jimpl_ = new DualShockJC(control_, device_number_);
        break;
        case GuitarHeroJC::controller_type:
            p_jimpl_ = new GuitarHeroJC(control_, device_number_);
        break;
    }

    if (on_find_joystick && p_jimpl_) on_find_joystick(p_jimpl_);

    return control_error_;
}


void Joystick::read_joystick() volatile
{
   // You must Read Gamepad to get new values
   // Read GamePad and set vibration values
   // control.read_gamepad(small motor on/off, larger motor strenght from 0-255)
   // if you don't enable the rumble, use control.read_gamepad(); with no values you should call this at least once a second

    if (!p_jimpl_)
    {
        return;
    }

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        p_jimpl_->process();
    }
}
