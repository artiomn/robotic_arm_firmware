#include "log.h"
#include "servo_motor.h"


void ServoMotor::attach(unsigned int pin, unsigned int min_angle, unsigned int max_angle)
{
    // Min and Max are pulses in the Servo class.
    Servo::attach(pin, min_angle, max_angle);
    pin_ = pin;
    min_angle_ = min_angle;
    max_angle_ = max_angle;
}
