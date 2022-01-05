#ifndef JOYSTICK_H
#define JOYSTICK_H

#include "log.h"
#include "joystick_controller.h"


class Joystick
{
public:
    Joystick(unsigned int device_number = 0) : device_number_(device_number), p_jimpl_(NULL) {}
    ~Joystick();

    void init_joystick(uint8_t clock_pin = 11, uint8_t command_pin = 9,
                       uint8_t attention_pin = 10, uint8_t data_pin = 8,
                       unsigned int serial_speed = 57600);
    void read_joystick() volatile;

public:
    typedef void (*CreateHandler)(volatile JoystickController *joystick_controller);

public:
    CreateHandler on_find_joystick;

private:
    PS2X control_;
    volatile JoystickController *p_jimpl_;
    int control_error_ = 0;
    uint8_t control_type_ = 0;

    const unsigned int device_number_;
};

#endif  // JOYSTICK_H
