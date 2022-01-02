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
    void process() final override;
};

#endif  // JC_GUITARHERO_H
