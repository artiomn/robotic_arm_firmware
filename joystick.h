#ifndef JOYSTICK_H
#define JOYSTICK_H

#include "log.h"
#include "joystick_controller.h"
#include "jc_dualshock.h"
#include "jc_guitarhero.h"


class Joystick
{
public:
    Joystick(unsigned int device_number = 0) : device_number_(device_number), p_jimpl_(NULL) {}
    ~Joystick()
    {
        delete p_jimpl_;
    };

    void init_joystick(unsigned int clock_pin = A0, unsigned int command_pin = 13,
                       unsigned int attention_pin = 11, unsigned int data_pin = 12,
                       unsigned int serial_speed = 57600)
    {
        Serial.begin(serial_speed);

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

    void read_joystick()
    {
       // You must Read Gamepad to get new values
       // Read GamePad and set vibration values
       // control.read_gamepad(small motor on/off, larger motor strenght from 0-255)
       // if you don't enable the rumble, use control.read_gamepad(); with no values you should call this at least once a second

        if (!p_jimpl_)
        {
            return;
        }

        p_jimpl_->process();
    }

public:
    typedef void (*CreateHandler)(JoystickController *joystick_controller);

public:
    CreateHandler on_find_joystick;

private:
    PS2X control_;
    JoystickController *p_jimpl_;
    int control_error_ = 0;
    byte control_type_ = 0;

    const unsigned int device_number_;
};

#endif  // JOYSTICK_H
