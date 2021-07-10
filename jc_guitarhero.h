#ifndef JC_GUITARHERO_H
#define JC_GUITARHERO_H

#include "log.h"
#include "joystick_controller.h"


class GuitarHeroJC : public JoystickController
{
public:
  static const unsigned int controller_type = 2;

public:
    GuitarHeroJC(PS2X &control, unsigned int device_number) : JoystickController(control, controller_type, device_number) {}

public:
  ButtonClickHandler on_button;
  StickHandler on_whammy_bar;

public:
    void process() final override
    {
        // This code WAS NOT TESTED!
        control_.read_gamepad();

        check_control_buttons();

        process_button_press<GREEN_FRET>("Green Fret Pressed", on_button);
        process_button_press<RED_FRET>("Red Fret Pressed", on_button);
        process_button_press<YELLOW_FRET>("Yellow Fret Pressed", on_button);
        process_button_press<BLUE_FRET>("Blue Fret Pressed", on_button);
        boolean orange_clicked = process_button_press<ORANGE_FRET>("Orange Fret Pressed", on_button);
        process_button_press<STAR_POWER>("Star Power Command", on_button);
        process_button<UP_STRUM>("Up Strum", on_button);
        process_button<DOWN_STRUM>("DOWN Strum", on_button);

        int value = control_.Analog(WHAMMY_BAR) - zero_value_;

        if (value || orange_clicked)
        {
            log_value("Wammy Bar Position: ", value);
            call_handler(on_whammy_bar, value, 0, orange_clicked);
        }
    }
};

#endif  // JC_GUITARHERO_H
