#include <PS2X_lib.h>
#include "log.h"
#include "joystick_controller.h"


void JoystickController::check_control_buttons()
{
    if (!ps2_control_.NewButtonState()) return;

    if (ps2_control_.ButtonPressed(PSB_START))
    {
        LOG_VALUE("Start is being held");
        if (device_number_)
        {
            if (!is_selection_in_process())
           {
                LOG_VALUE("Device selection started. My device number = ", device_number_);
                newly_selected_number_ = 0;
                selection_time_start_ = millis();
                delay(1);
            }
            else
            {
                selected_number_ = newly_selected_number_;
                selection_time_start_ = 0;
                LOG_VALUE("Device selection finished. Selected device number = ", selected_number_);
                if (selected())
                {
                    LOG_VALUE("It's me! I have been selected. Working...");
                    vibrate(selection_complete_vibration_ms, selection_complete_vibration_speed, selected_number_);
                    //call_handler(on_select, selected_number_);
                }
            }
        }

        on_button_(this, PSB_START, ps2_control_.Analog(PSB_START));
    }

    if (ps2_control_.ButtonPressed(PSB_SELECT))
    {
        if (is_selection_in_process())
        {
            selection_time_start_ = millis();
            ++newly_selected_number_;
            LOG_VALUE("Newly selected number: ", newly_selected_number_);
        }
        else if (selected())
        {
            LOG_VALUE("Select is being held.");
            on_button_(this, PSB_SELECT, ps2_control_.Analog(PSB_SELECT));
        }
    }
}


bool JoystickController::selected() const volatile
{
    return !device_number_ || (device_number_ == selected_number_);
}


bool JoystickController::is_selection_in_process() const volatile
{
    unsigned long ms = millis();
    return (ms > selection_waiting_ms) && ((ms - selection_time_start_) <= selection_waiting_ms);
}
