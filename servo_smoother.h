#ifndef SERVO_SMOOTHER_H
#define SERVO_SMOOTHER_H

#include "log.h"
#include "servo_motor.h"


class ServoSmoother
{
public:
    const int SS_SERVO_PERIOD = 20;
    const int SS_DEADTIME = 10;

    const uint8_t SS_DEADZONE = 10;
    const uint8_t SS_DEADZONE_SP = 3;

    ServoSmoother(ServoMotor &motor, int angle, unsigned int max_speed = 50, unsigned int acceleration = 1000) :
      servo_max_speed_(max_speed),
      acceleration_(acceleration),
      servo_current_pos_(motor.read()),
      servo_target_pos_(angle),
      motor_(motor),
      servo_state_(true)
    {}

    bool tick();

private:
    int sign(int x);

private:
    float delta_ = SS_SERVO_PERIOD / 1000.0;
    int servo_current_pos_;
    int servo_target_pos_;
    int16_t servo_max_speed_;
    uint16_t acceleration_;
    float last_speed_ = 0;
    float speed_ = 0;
    ServoMotor &motor_;
    bool servo_state_;
    uint8_t timeout_counter_;
};

#endif
