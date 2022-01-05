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
    JoystickController(PS2X &ps2_control, unsigned int controller_type, unsigned int device_number) : ps2_control_(ps2_control), controller_type_(controller_type), device_number_(device_number)
    {
        // Need to disable side effects, such a vibration after rebooting.
        ps2_control_.read_gamepad(false, 0);
    };
    virtual ~JoystickController() = default;

    virtual void process() volatile = 0;
    virtual unsigned int type() const { return controller_type_; }
    virtual void vibrate(unsigned long ms, uint8_t vibration_speed = 50, uint8_t vibration_count = 1) volatile = 0;

public:
    typedef void (*ButtonClickHandler)(volatile JoystickController* controller, uint16_t button_code, int value);
    typedef void (*StickHandler)(volatile JoystickController* controller, int x_value, int y_value, boolean clicked);
    typedef void (*SelectHandler)(volatile JoystickController* controller, unsigned int device_number);

public:
    ButtonClickHandler on_button;
    SelectHandler on_select;

protected:
    template<typename HandlerType, typename ...HandlerArgs>
    bool call_handler(HandlerType handler, HandlerArgs... handler_args) volatile
    {
        if (selected() && handler)
        {
            handler(this, handler_args...);
        }
    }

    void check_control_buttons() volatile;
    bool selected() const volatile;

    template<const uint16_t button_id>
    bool process_button_press(const char *log_message, ButtonClickHandler on_button_handler) volatile
    {
        if (selected() && ps2_control_.ButtonPressed(button_id))
        {
            log_and_call<button_id, button_id>(log_message, on_button_handler);
            return true;
        }

        return false;
    }

    template<const uint16_t button_id>
    bool process_button(const char *log_message, ButtonClickHandler on_button_handler) volatile
    {
        if (selected() && ps2_control_.Button(button_id))
        {
            log_and_call<button_id, button_id>(log_message, on_button_handler);
            return true;
        }

        return false;
    }

    template<const uint16_t button_id, const uint16_t analog_button_id>
    bool process_analog_button(const char *log_message, ButtonClickHandler on_button_handler) volatile
    {
        if (selected() && ps2_control_.Button(button_id))
        {
            log_and_call<button_id, analog_button_id>(log_message, on_button_handler);
            return true;
        }

        return false;
    }

protected:
    PS2X &ps2_control_;
    const unsigned zero_value_ = 128;

private:
    bool is_selection_in_process() const volatile;

    template<const uint16_t button_id, const uint8_t analog_button_id>
    void log_and_call(const char *log_message, ButtonClickHandler on_button_handler) volatile
    {
        LOG_VALUE(log_message);
        call_handler(on_button_handler, button_id, ps2_control_.Analog(analog_button_id));
    }

private:
    const uint8_t controller_type_;
    const uint8_t device_number_;
    volatile uint8_t selected_number_ = 0;
    volatile uint8_t newly_selected_number_ = 0;
    unsigned long int selection_time_start_ = 0;
};

#endif  // JOYSTICK_CONTROLLER_H
