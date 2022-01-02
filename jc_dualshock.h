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

    void process() final override;
    void vibrate(unsigned int ms, byte vibration_speed = 150, byte vibration_count = 1) override;

public:
  ButtonClickHandler on_pad;
  StickHandler on_left_stick;
  StickHandler on_right_stick;

private:
    void check_figure_buttons();
    void check_fore_buttons();
    void check_pad();
    void get_sticks();

    template<unsigned int x_const, unsigned int y_const, unsigned int btn_const>
    void check_stick(const char *msg, StickHandler on_stick)
    {
        int x_value = zero_value_ - control_.Analog(x_const);
        int y_value = zero_value_ - control_.Analog(y_const) + 1;
        bool clicked = control_.Button(btn_const);

        if (selected() && on_stick && ((abs(x_value) > stick_min_value) || (abs(y_value) > stick_min_value) || clicked))
        {
            log_value(msg, x_value);
            log_value(msg, y_value);
            on_stick(this, x_value, y_value, clicked);
        }

    }

private:
    byte vibration_speed_;
    byte vibration_period_ = 0;
    unsigned long vibration_start_time_ = 0;
    byte vibration_count_ = 0;
};

#endif  // JC_DUALSHOCK_H
