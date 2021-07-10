#ifndef SERVO_SMOOTHER_H
#define SERVO_SMOOTHER_H

#include "log.h"
#include "servo_motor.h"


class Smoother
{
public:
    const int SS_SERVO_PERIOD = 20;
    const int SS_DEADTIME = 10;

    const byte SS_DEADZONE = 10;
    const byte SS_DEADZONE_SP = 3;

    Smoother(ServoMotor &motor, unsigned int max_speed = 50, unsigned int acceleration = 1000) :
      servo_max_speed_(max_speed),
      acceleration_(acceleration),
      servo_current_pos_(motor.read()),
      servo_target_pos_(servo_current_pos_),
      motor_(motor)
    {}

    boolean tick()
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

private:
    static int sign(int x) { return ((x) > 0 ? 1 : -1); }

private:
    float delta_ = SS_SERVO_PERIOD / 1000.0;
    int servo_current_pos_;
    int servo_target_pos_;
    int16_t servo_max_speed_;
    uint16_t acceleration_;
    boolean started_ = true;
    float last_speed_ = 0;
    float speed_ = 0;
    ServoMotor &motor_;
    bool servo_state_;
    byte timeout_counter_;
};

#endif
