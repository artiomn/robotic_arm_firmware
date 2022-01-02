#include "log.h"
#include "jc_guitarhero.h"


void GuitarHeroJC::process()
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
