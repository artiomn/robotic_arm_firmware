#ifndef SERVO_MOTOR_H
#define SERVO_MOTOR_H

#include <Servo.h>
#include "log.h"


class ServoMotor : public Servo
{
public:
    void attach(unsigned int pin, unsigned int min_angle, unsigned int max_angle);

public:
    unsigned min_angle() const { return min_angle_; }
    unsigned max_angle() const { return max_angle_; }
    unsigned pin() const { return pin_; }

private:
  int min_angle_;
  int max_angle_;
  uint8_t pin_;
};

#endif  // SERVO_MOTOR_H
