# Robotic arm firmware

This is the firmware for the [robotic arm](https://aliexpress.ru/item/4000867603887.html), which is allow to control it remotely.
As a remote control device [Sony PS2 DualShock Controller](https://aliexpress.ru/item/32882754928.html) is used.
Arduino Uno is used as a control board.


## Features

- Full arm control with left and right sticks.
- Arm returning to the initial position, by the sticks clicking.
- Full manipulator control with, using a pad.
- Manipulator open and close, using triangle and cross buttons, respectively.
- Possibility to control several devices, using only joystick.
- Smooth arm motions.
- Easy-extensible C++ code.


## Dependencies

- [PS2X library](https://github.com/madsci1016/Arduino-PS2X).
- [Servo library](https://www.arduino.cc/reference/en/libraries/servo/).
