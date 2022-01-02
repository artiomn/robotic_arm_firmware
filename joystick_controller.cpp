#include <PS2X_lib.h>
#include "log.h"
#include "joystick_controller.h"


void JoystickController::check_control_buttons()
{
    if (!control_.NewButtonState()) return;

    if (control_.ButtonPressed(PSB_START))
    {
        log_value("Start is being held");
        if (device_number_)
        {
            if (!is_selection_in_process())
           {
                log_value("Device selection started. My device number = ", device_number_);
                newly_selected_number_ = 0;
                selection_time_start_ = millis();
                delay(1);
            }
            else
            {
                selected_number_ = newly_selected_number_;
                selection_time_start_ = 0;
                log_value("Device selection finished. Selected device number = ", selected_number_);
                if (selected())
                {
                    log_value("It's me! I have been selected. Working...");
                    vibrate(selection_complete_vibration_ms, selection_complete_vibration_speed, selected_number_);
                    //call_handler(on_select, selected_number_);
                }
            }
        }

        call_handler(on_button, PSB_START, control_.Analog(PSB_START));
    }

    if (control_.ButtonPressed(PSB_SELECT))
    {
        log_value("Select is being held.");
        if (is_selection_in_process())
        {
            selection_time_start_ = millis();
            ++newly_selected_number_;
            log_value("Newly selected number: ", newly_selected_number_);
        }
        call_handler(on_button, PSB_SELECT, control_.Analog(PSB_SELECT));
    }
}


boolean JoystickController::selected() const
{
    return !device_number_ || (device_number_ == selected_number_);
}


void JoystickController::vibrate(unsigned int ms, byte vibration_speed, byte vibration_count) {}

boolean JoystickController::is_selection_in_process() const
{
    unsigned long ms = millis();
    return (ms > selection_waiting_ms) && ((ms - selection_time_start_) <= selection_waiting_ms);
}
