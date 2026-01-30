# Robotic arm firmware

![](https://ae04.alicdn.com/kf/H1b2a68f064a446209f33b7f3c984efebH/6-DOF-Arduino.jpg)

This is the firmware for the [robotic arm](https://aliexpress.ru/item/4000867603887.html), which is allow to control it remotely.
As a remote control device [Sony PS2 DualShock Controller](https://aliexpress.ru/item/32882754928.html) is used.
Arduino Uno is used as a control board.


## Features

- Full arm control with left and right sticks.
- Arm returning to the initial position, by the sticks clicking.
- Full manipulator control with, using a pad.
- Manipulator open and close, using triangle and cross buttons, respectively.
- Possibility to control several devices, using only joystick.
- Joystick vibration after the device was selected.
- Smooth arm motions.
- Easy-extensible C++ code.


## Hardware

- Arduino Uno.
- [Arduino Sensor Shield v5.0](https://www.amazon.in/Electrobot-Arduino-Sensor-Shield-V5-0/dp/B08D7F3D6Y).


## Dependencies

- [PS2X library](https://github.com/madsci1016/Arduino-PS2X).
- [Servo library](https://www.arduino.cc/reference/en/libraries/servo/).
- [nonstd](https://github.com/winterscar/functional-avr) - included.


## How to use

- Open the sketch in the Arduino Studio.
- Compile.
- Upload into the board.
- Connect arm servos and joystick receiver to the shield:

  * 2 - shoulder rotate servo.
  * 3 - shoulder lift servo.
  * 4 - forearm rotate servo.
  * 5 - forearm lift servo.
  * 6 - wrist lift servo.
  * 7 - manipulator open/close servo.
  * 8 - joystick radio module data pin.
  * 9 - joystick radio module command pin.
  * 10 - joystick radio module attention pin.
  * 11 - joystick radio module clock pin.

- Connect external power and controller power:

  * Remove jumper near shield power input to power servo machines from the external power unit.