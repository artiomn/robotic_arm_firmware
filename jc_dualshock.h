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

    void process() final override
    {
        // DualShock Controller
        // read controller and set large motor to spin at 'vibrate' speed
        boolean enable_vibration = false;

        if (vibration_count_)
        {
            unsigned int ms = millis();
            enable_vibration = ((vibration_start_time_ + vibration_period_) >= ms);

            if (!enable_vibration && ((vibration_start_time_ + 2 * vibration_period_) < ms))
            {
                if (--vibration_count_)
                {
                    vibration_start_time_ = ms;
                    enable_vibration = true;
                }
            }
        }

        control_.read_gamepad(false, enable_vibration ? vibration_speed_ : 0);

        check_control_buttons();

        // if (control_.NewButtonState())
        // Will be TRUE if any button changes state (on to off, or off to on).
        // Bad idea: I can't get buttons values.

        check_pad();
        check_figure_buttons();
        check_fore_buttons();

        get_sticks();
    }

    void vibrate(unsigned int ms, byte vibration_speed = 150, unsigned int vibration_count = 1) override
    {
        vibration_start_time_ = millis();
        vibration_period_ = ms;
        vibration_speed_ = vibration_speed;
        vibration_count_ = vibration_count;
    }

public:
  ButtonClickHandler on_pad;
  StickHandler on_left_stick;
  StickHandler on_right_stick;

private:
    void check_figure_buttons()
    {
        // ButtonPressed, ButtonReleased, NewButtonState.
        process_button_press<PSB_RED>("Circle just pressed", on_button);
        process_button_press<PSB_PINK>("Square just pressed", on_button);
        process_button_press<PSB_BLUE>("Cross just pressed", on_button);
        process_button_press<PSB_GREEN>("Triangle just pressed", on_button);
    }

    void check_fore_buttons()
    {
        process_button<PSB_L1>("L1 pressed", on_button);
        process_button<PSB_R1>("R1 pressed", on_button);
        process_button<PSB_L2>("L2 pressed", on_button);
        process_button<PSB_R2>("R2 pressed", on_button);
    }

    void check_pad()
    {
        process_analog_button<PSB_PAD_UP, PSAB_PAD_UP>("Up held", on_pad);
        process_analog_button<PSB_PAD_DOWN, PSAB_PAD_DOWN>("Down held", on_pad);
        process_analog_button<PSB_PAD_RIGHT, PSAB_PAD_RIGHT>("Right held", on_pad);
        process_analog_button<PSB_PAD_LEFT, PSAB_PAD_LEFT>("Left held", on_pad);
    }

    template<unsigned int x_const, unsigned int y_const, unsigned int btn_const>
    void check_stick(const char *msg, StickHandler on_stick)
    {
        int x_value = control_.Analog(x_const) - zero_value_;
        int y_value = zero_value_ - control_.Analog(y_const) + 1;
        boolean clicked = control_.Button(btn_const);

        if (selected() && on_stick && ((abs(x_value) > stick_min_value) || (abs(y_value) > stick_min_value) || clicked))
        {
            log_value(msg, x_value);
            log_value(msg, y_value);
            on_stick(this, x_value, y_value, clicked);
        }

    }

    void get_sticks()
    {
        check_stick<PSS_LX, PSS_LY, PSB_L3>("Left ", on_left_stick);
        check_stick<PSS_RX, PSS_RY, PSB_R3>("Right ", on_right_stick);
    }

private:
    int vibration_speed_;
    unsigned int vibration_period_ = 0;
    unsigned long vibration_start_time_ = 0;
    unsigned int vibration_count_ = 0;
};

#endif  // JC_DUALSHOCK_H
