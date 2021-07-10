#ifndef JOYSTICK_CONTROLLER_H
#define JOYSTICK_CONTROLLER_H

#include <PS2X_lib.h>
#include "log.h"


class JoystickController
{
public:
    const unsigned int selection_waiting_ms = 3000;
    const unsigned int selection_complete_vibration_speed = 190;
    const unsigned int selection_complete_vibration_ms = 300;

public:
    JoystickController(PS2X &control, unsigned int controller_type, unsigned int device_number) : control_(control), controller_type_(controller_type), device_number_(device_number) {};
    virtual ~JoystickController() {};

    virtual void process() = 0;
    virtual unsigned int type() { return controller_type_; }

public:
    typedef void (*ButtonClickHandler)(JoystickController* controller, unsigned int button_code, int value);
    typedef void (*StickHandler)(JoystickController* controller, int x_value, int y_value, boolean clicked);
    typedef void (*SelectHandler)(JoystickController* controller, unsigned int device_number);

public:
    ButtonClickHandler on_button;
    SelectHandler on_select;

protected:
    template<typename HandlerType, typename ...HandlerArgs>
    boolean call_handler(HandlerType handler, HandlerArgs... handler_args)
    {
        if (selected() && handler)
        {
            handler(this, handler_args...);
        }
    }

    void check_control_buttons()
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

    boolean selected() const
    {
        return !device_number_ || (device_number_ == selected_number_);
    }

    template<const unsigned int button_id>
    boolean process_button_press(const char *log_message, ButtonClickHandler on_button_handler)
    {
        if (control_.ButtonPressed(button_id))
        {
            log_and_call<button_id, button_id>(log_message, on_button_handler);
            return true;
        }

        return false;
    }

    template<const unsigned int button_id>
    boolean process_button(const char *log_message, ButtonClickHandler on_button_handler)
    {
        if (control_.Button(button_id))
        {
            log_and_call<button_id, button_id>(log_message, on_button_handler);
            return true;
        }

        return false;
    }

    template<const unsigned int button_id, const unsigned int analog_button_id>
    boolean process_analog_button(const char *log_message, ButtonClickHandler on_button_handler)
    {
        if (control_.Button(button_id))
        {
            log_and_call<button_id, analog_button_id>(log_message, on_button_handler);
            return true;
        }

        return false;
    }

    virtual void vibrate(unsigned int ms, byte vibration_speed = 50, unsigned int vibration_count = 1) {}

protected:
    PS2X &control_;
    const unsigned zero_value_ = 128;

private:
    boolean is_selection_in_process() const
    {
        unsigned long ms = millis();
        return (ms > selection_waiting_ms) && ((ms - selection_time_start_) <= selection_waiting_ms);
    }

    template<const unsigned int button_id, const unsigned int analog_button_id>
    void log_and_call(const char *log_message, ButtonClickHandler on_button_handler)
    {
        log_value(log_message);
        call_handler(on_button_handler, button_id, control_.Analog(analog_button_id));
    }

private:
    const unsigned int controller_type_;
    const unsigned int device_number_;
    unsigned int selected_number_;
    unsigned int newly_selected_number_;
    unsigned long int selection_time_start_ = 0;
};

#endif  // JOYSTICK_CONTROLLER_H
