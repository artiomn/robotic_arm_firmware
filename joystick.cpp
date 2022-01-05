#include <util/atomic.h>

#include "log.h"
#include "joystick.h"
#include "jc_dualshock.h"
#include "jc_guitarhero.h"


Joystick::~Joystick()
{
    delete p_jimpl_;
};


void Joystick::init_joystick(uint8_t clock_pin, uint8_t command_pin,
                             uint8_t attention_pin, uint8_t data_pin,
                             unsigned int serial_speed)
{
    Serial.begin(serial_speed);

    delay(100);

    // GamePad(clock, command, attention, data, Pressures?, Rumble?) 
    control_error_ = control_.config_gamepad(clock_pin, command_pin, attention_pin, data_pin, true, true);

    switch (control_error_)
    {
        case 0:
            Serial.println("Found Controller, configured successful.");
            Serial.println("Go to www.billporter.info for updates and to report bugs.");
        break;
        case 1:
            Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips.");
        break;
        case 2:
            Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips.");
        break;
        case 3:
            Serial.println("Controller refusing to enter Pressures mode, may not support it.");
        break;
        default:
            Serial.println("Unknown error.");
    }

    switch (control_type_ = control_.readType())
    {
        case 0:
            Serial.println("Unknown Controller type");
        break;
        case DualShockJC::controller_type:
            Serial.println("DualShock Controller Found");
            p_jimpl_ = new DualShockJC(control_, device_number_);
        break;
        case GuitarHeroJC::controller_type:
            Serial.println("GuitarHero Controller Found");
            p_jimpl_ = new GuitarHeroJC(control_, device_number_);
        break;
    }

    if (on_find_joystick && p_jimpl_) on_find_joystick(p_jimpl_);
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
