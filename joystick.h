#ifndef JOYSTICK_H
#define JOYSTICK_H

#include "log.h"
#include "joystick_controller.h"


class Joystick
{
public:
    Joystick(unsigned int device_number = 0) : device_number_(device_number), p_jimpl_(NULL) {}
    ~Joystick();

    void init_joystick(byte clock_pin = 11, byte command_pin = 9,
                       byte attention_pin = 10, byte data_pin = 8,
                       unsigned int serial_speed = 57600);
    void read_joystick();

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
