#pragma once

#include <Arduino.h>
#include <Servo.h>
#include "log.h"


const int incorrect_angle = -255;

class ServoMotor : public Servo
{
public:
    bool attach(unsigned int pin, unsigned int min_angle, unsigned int max_angle);

public:
    unsigned min_angle() const { return min_angle_; }
    unsigned max_angle() const { return max_angle_; }
    unsigned pin() const { return pin_; }
    void init(int value);
    void rotate_to(int angle);
    void rotate();

private:
  int min_angle_;
  int max_angle_;
  bool rotation_ = false;
  int dest_angle_ = incorrect_angle;
  uint8_t pin_;
  int dps_ = 1;
};