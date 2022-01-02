#include <Arduino.h>

#include "log.h"
#include "servo_smoother.h"


bool Smoother::tick()
{
    if (started_)
    {
        int err = servo_target_pos_ - servo_current_pos_;

        // Stop condition
        if (abs(err) > SS_DEADZONE && abs(last_speed_ - speed_) < SS_DEADZONE_SP)
        {
            if (acceleration_ != 0)
            {
                // Need to stop.
                bool direction = (static_cast<float>(speed_) * speed_ / acceleration_ / 2.0 >= abs(err));
                speed_ += static_cast<float>(acceleration_) * delta_ * (direction ? -sign(speed_) : sign(err));
            }
            else
            {
                speed_ = err / delta_;
            }
            speed_ = constrain(speed_, -servo_max_speed_, servo_max_speed_);
            servo_current_pos_ += speed_ * delta_;
            if (!servo_state_)
            {
                servo_state_ = true;
                timeout_counter_ = 0;
            }
            motor_.write(servo_current_pos_);
        }
        else
        {
            //_servoCurrentPos = _servoTargetPos;
            speed_ = 0;
            if (servo_state_)
            {
                motor_.write(servo_current_pos_);
                timeout_counter_++;
            }
            if (timeout_counter_ > SS_DEADTIME && servo_state_)
            {
                servo_state_ = false;
            }
        }
        last_speed_ = speed_;
    }
    return !servo_state_;
}


int Smoother::sign(int x) { return (x > 0 ? 1 : -1); }
