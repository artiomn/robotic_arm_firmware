#include "log.h"
#include "jc_dualshock.h"


void DualShockJC::process() volatile
{
    // Read DualShock Controller and set large motor to spin at 'vibrate' speed.
    ps2_control_.read_gamepad(false, check_vibration_enabled() ? vibration_speed_ : 0);

    check_control_buttons();

    // if (control_.NewButtonState())
    // Will be TRUE if any button changes state (on to off, or off to on).
    // Bad idea: I can't get buttons values.

    check_pad();
    check_figure_buttons();
    check_fore_buttons();

    get_sticks();
}


void DualShockJC::vibrate(unsigned long ms, uint8_t vibration_speed, uint8_t vibration_count) volatile
{
    vibration_start_time_ = millis();
    vibration_period_ = ms;
    vibration_speed_ = vibration_speed;
    vibration_count_ = vibration_count;
}


bool DualShockJC::check_vibration_enabled()
{
    volatile bool enable_vibration = false;

    if (vibration_count_)
    {
        volatile unsigned long ms = millis();
        enable_vibration = ((vibration_start_time_ + vibration_period_) >= ms);

        if (!enable_vibration && ((vibration_start_time_ + 2 * vibration_period_) < ms))
        {
            if (--vibration_count_ > 0)
            {
                vibration_start_time_ = ms;
                enable_vibration = true;
            }
        }
            LOG_VALUE("Vibration ms: ", ms);
    }

    return enable_vibration;
}


void DualShockJC::check_figure_buttons()
{
    // ButtonPressed, ButtonReleased, NewButtonState.
    process_button_press<PSB_RED>("Circle just pressed", on_button_);
    process_button_press<PSB_PINK>("Square just pressed", on_button_);
    process_button_press<PSB_BLUE>("Cross just pressed", on_button_);
    process_button_press<PSB_GREEN>("Triangle just pressed", on_button_);
}


void DualShockJC::check_fore_buttons()
{
    process_button<PSB_L1>("L1 pressed", on_button_);
    process_button<PSB_R1>("R1 pressed", on_button_);
    process_button<PSB_L2>("L2 pressed", on_button_);
    process_button<PSB_R2>("R2 pressed", on_button_);
}


void DualShockJC::check_pad()
{
    process_analog_button<PSB_PAD_UP, PSAB_PAD_UP>("Up held", on_pad_);
    process_analog_button<PSB_PAD_DOWN, PSAB_PAD_DOWN>("Down held", on_pad_);
    process_analog_button<PSB_PAD_RIGHT, PSAB_PAD_RIGHT>("Right held", on_pad_);
    process_analog_button<PSB_PAD_LEFT, PSAB_PAD_LEFT>("Left held", on_pad_);
}


void DualShockJC::get_sticks()
{
    check_stick<PSS_LX, PSS_LY, PSB_L3>("Left stick", on_left_stick_);
    check_stick<PSS_RX, PSS_RY, PSB_R3>("Right stick", on_right_stick_);
}
