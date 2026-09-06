#include <Arduino.h>
#include <util/atomic.h>
#include <math.h>

#include "log.h"
#include "servo_motor.h"


bool ServoMotor::attach(unsigned int pin, unsigned int min_angle, unsigned int max_angle)
{
    // Min and Max are pulses in the Servo class.
    auto result = Servo::attach(pin, min_angle, max_angle);
    pin_ = pin;
    min_angle_ = min_angle;
    max_angle_ = max_angle;

    return INVALID_SERVO != result;
}


void ServoMotor::init(int value)
{
    rotate_to(value);
}


void ServoMotor::rotate_to(int angle)
{
    dest_angle_ = angle;
    rotation_ = true;
}


void ServoMotor::rotate()
{
    if (!rotation_ || (dest_angle_ == incorrect_angle)) return;

    const auto cur_angle = read();

    if (dest_angle_ > cur_angle)
    {
        const auto new_angle = cur_angle + dps_;
        write(min(new_angle, dest_angle_));
        rotation_ = new_angle < dest_angle_;
    }
    else if (dest_angle_ < cur_angle)
    {
        const auto new_angle = cur_angle - dps_;
        write(max(new_angle, dest_angle_));
        rotation_ = new_angle > dest_angle_;
    }
}