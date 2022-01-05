#ifndef SERVO_MOTOR_H
#define SERVO_MOTOR_H

#include <Arduino.h>
#include <Servo.h>

#include "log.h"


class ParabolicEaser
{
public:
    ParabolicEaser(uint16_t start_angle, uint16_t end_angle, uint16_t max_angle_per_ms);
    uint16_t ease(uint16_t cur_angle);

private:
    uint16_t end_angle_;
    bool check_direction_;
    float a_;
    float b_;
    float c_;
};


class ServoMotor : public Servo
{
public:
    bool attach(unsigned int pin, unsigned int min_angle, unsigned int max_angle);

public:
    unsigned min_angle() const { return min_angle_; }
    unsigned max_angle() const { return max_angle_; }
    unsigned pin() const { return pin_; }
    void rotate_to(int angle, uint16_t degrees_per_second = 10);

private:
  int min_angle_;
  int max_angle_;
  uint8_t pin_;
};

#endif  // SERVO_MOTOR_H
