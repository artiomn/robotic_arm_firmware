#ifndef JOYSTICK_CONTROLLER_H
#define JOYSTICK_CONTROLLER_H

#include <PS2X_lib.h>
#include "log.h"


class JoystickController
{
public:
    const unsigned int selection_waiting_ms = 3000;
    const byte selection_complete_vibration_speed = 190;
    const unsigned int selection_complete_vibration_ms = 300;

public:
    JoystickController(PS2X &control, unsigned int controller_type, unsigned int device_number) : control_(control), controller_type_(controller_type), device_number_(device_number) {};
    virtual ~JoystickController() {};

    virtual void process() = 0;
    virtual unsigned int type() { return controller_type_; }
    virtual void vibrate(unsigned int ms, byte vibration_speed = 50, byte vibration_count = 1);

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

    void check_control_buttons();
    boolean selected() const;

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

protected:
    PS2X &control_;
    const unsigned zero_value_ = 128;

private:
    boolean is_selection_in_process() const;

    template<const byte button_id, const byte analog_button_id>
    void log_and_call(const char *log_message, ButtonClickHandler on_button_handler)
    {
        log_value(log_message);
        call_handler(on_button_handler, button_id, control_.Analog(analog_button_id));
    }

private:
    const byte controller_type_;
    const byte device_number_;
    byte selected_number_;
    byte newly_selected_number_;
    unsigned long int selection_time_start_ = 0;
};

#endif  // JOYSTICK_CONTROLLER_H
