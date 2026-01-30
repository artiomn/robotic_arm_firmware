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


void ServoMotor::rotate_to(int angle, uint16_t degrees_per_second)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        const int cur_angle = read();
        const int new_angle = constrain(cur_angle + angle, min_angle_, max_angle_);

        if (new_angle == cur_angle) return;

        TrapeziumEaser easer(cur_angle, new_angle, 270);

        for (uint16_t i = cur_angle; i != new_angle; i = easer.ease(i))
        {
            write(i);
            delay(1);
        }

        write(new_angle);
    }
}


TrapeziumEaser::TrapeziumEaser(uint16_t start_angle, uint16_t end_angle, uint16_t max_angle_per_ms) :
  end_angle_(end_angle),
  direction_(start_angle < end_angle ? 1 : -1),
  max_angle_per_ms_(max_angle_per_ms)
{
    // Common easer function: next_angle = easer(start_angle, end_angle)(cur_angle)
    // Common easer equation (angle per second from time, where "time" is a current angle): next_angle = cur_angle + delta
    // Easer call returns delta.

    float path_len = abs(end_angle - start_angle);
    inc_path_len_ = angle_increasing_percent_ * (path_len / 100);

    // y = m * x;
    m_x_ = max_angle_per_ms_ / inc_path_len_;
}


uint16_t TrapeziumEaser::ease(uint16_t cur_angle)
{
    uint16_t delta = max_angle_per_ms_;

    if (((end_angle_ - inc_path_len_)  < cur_angle) && (cur_angle < inc_path_len_))
    {
        delta = m_x_ * cur_angle;
    }

    return cur_angle + direction_ * delta;
}
