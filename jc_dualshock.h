#ifndef JC_DUALSHOCK_H
#define JC_DUALSHOCK_H

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
  ButtonClickHandler on_pad;
  StickHandler on_left_stick;
  StickHandler on_right_stick;

private:
    void check_figure_buttons() volatile;
    void check_fore_buttons() volatile;
    void check_pad() volatile;
    void get_sticks() volatile;
    bool check_vibration_enabled() volatile;

    template<unsigned int x_const, unsigned int y_const, uint16_t btn_const>
    void check_stick(const char *msg, StickHandler on_stick) volatile
    {
        volatile int x_value = zero_value_ - ps2_control_.Analog(x_const);
        volatile int y_value = zero_value_ - ps2_control_.Analog(y_const) + 1;
        volatile bool clicked = ps2_control_.Button(btn_const);

        if (selected() && on_stick && ((abs(x_value) > stick_min_value) || (abs(y_value) > stick_min_value) || clicked))
        {
            LOG_VALUE(msg);
            LOG_VALUE("X value: ", x_value);
            LOG_VALUE("Y value: ", y_value);
            on_stick(this, x_value, y_value, clicked);
        }

    }

private:
    volatile uint8_t vibration_speed_;
    volatile unsigned long vibration_period_ = 0;
    volatile unsigned long vibration_start_time_ = 0;
    volatile int8_t vibration_count_ = 0;
};

#endif  // JC_DUALSHOCK_H
