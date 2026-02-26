#pragma once

#include "log.h"
#include "joystick_controller.h"
#include "nonstd.h"


class Joystick
{
public:
    enum JoystickErrorCodes
    {
        jerr_success = 0x00,
        jerr_controller_not_found = 0x01,
        jerr_controller_not_accept_commands = 0x02,
        jerr_controller_refuse_pressures_mode = 0x03,
        jerr_unknown_controller_type = 0x10
    };

public:
    Joystick(unsigned int device_number = 0) : device_number_(device_number), p_jimpl_(NULL) {}
    ~Joystick();

    int init_joystick(uint8_t clock_pin = 11, uint8_t command_pin = 9,
                      uint8_t attention_pin = 10, uint8_t data_pin = 8);
    void read_joystick() volatile;

public:
    typedef nonstd::function<void(volatile JoystickController *joystick_controller)> CreateHandler;

public:
    CreateHandler on_find_joystick_;

private:
    PS2X control_;
    volatile JoystickController *p_jimpl_;
    int control_error_ = 0;
    uint8_t control_type_ = 0;

    const unsigned int device_number_;
};