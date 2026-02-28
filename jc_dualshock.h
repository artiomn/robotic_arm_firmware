#pragma once

#include "log.h"
#include "joystick_controller.h"


class DualShockJC : public JoystickController
{
public:
  static const unsigned int controller_type = 1;
  const unsigned stick_min_value = 2;

public:
    DualShockJC(PS2X &control, unsigned int device_number) : JoystickController(control, controller_type, device_number) {}

    void process() volatile final override;
    void vibrate(unsigned long ms, uint8_t vibration_speed = 150, uint8_t vibration_count = 1) volatile final override;

public:
  ButtonClickHandler on_pad_;
  StickHandler on_left_stick_;
  StickHandler on_right_stick_;

private:
    void check_figure_buttons();
    void check_fore_buttons();
    void check_pad();
    void get_sticks();
    bool check_vibration_enabled();

    template<unsigned int x_const, unsigned int y_const, uint16_t btn_const, typename StringType>
    void check_stick(const StringType *msg, StickHandler on_stick)
    {
        volatile int x_value = zero_value_ - ps2_control_.Analog(x_const);
        volatile int y_value = zero_value_ - ps2_control_.Analog(y_const) + 1;
        volatile bool clicked = ps2_control_.Button(btn_const);

        if (selected() && on_stick && ((abs(x_value) > stick_min_value) || (abs(y_value) > stick_min_value) || clicked))
        {
            LOG_MESSAGE(msg);
            LOG_VALUE(F("X value: "), x_value);
            LOG_VALUE(F("Y value: "), y_value);
            if (on_stick)
            {
                LOG_MESSAGE(F("Call on stick handler..."));
                on_stick(this, x_value, y_value, clicked);
                LOG_MESSAGE(F("On stick handler called."));
            }
            else
            {
                LOG_MESSAGE(F("On stick handler is null."));
            }
        }

    }

private:
    volatile uint8_t vibration_speed_;
    volatile unsigned long vibration_period_ = 0;
    volatile unsigned long vibration_start_time_ = 0;
    volatile int8_t vibration_count_ = 0;
};