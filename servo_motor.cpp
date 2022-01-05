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

        ParabolicEaser easer(cur_angle, new_angle, 350);

        /*const int rot_step = cur_angle < new_angle ? 1 : -1;

        for (int i = cur_angle; i != new_angle; i += rot_step)
        {
            ca = easer.ease(ca);
            write(i);
            delay(1);
        }*/

        for (uint16_t i = cur_angle; i != new_angle; i = easer.ease(i))
        {
            write(i);
            delay(1);
            Serial.print("A: ");
            Serial.println(i);
        }

        write(new_angle);
    }
}


ParabolicEaser::ParabolicEaser(uint16_t start_angle, uint16_t end_angle, uint16_t max_angle_per_ms) :
  end_angle_(end_angle),
  check_direction_(start_angle < end_angle)
{
    // Common easer function: next_angle = easer(start_angle, end_angle)(cur_angle)
    // Parabolic path (angle per second from time, where "time" is a current angle): next_angle = cur_angle + (a * cur_angle^2 + b * cur_angle + c)
    // Maximum speed will be developed in this point: mid_angle = |end_angle - start_angle| / 2
    // Parabolic via 3 points (time, speed), which is equal to (cur_angle, next_angle): (start_angle, 0), (mid_angle, max_angle_per_ms), (end_angle, 0)
    // 
    // |start_angle^2 start_angle 1| |a|   |0|
    // |mid_angle^2   mid_angle   1| |b| = |max_angle_per_ms|
    // |end_angle^2   end_angle   1| |c|   |0|
    //
    // a = -4 * max_angle_per_ms / ((end_angle - 3*start_angle) * (end_angle + start_angle))
    // b = 4 * max_angle_per_ms / (end_angle - 3*start_angle)
    // c = start_angle * max_angle_per_ms*(-1 + (-4 * (end_angle - start_angle) / (end_angle + start_angle))) / (end_angle - 3*start_angle)
    /*if (start_angle < end_angle)
    {
        uint16_t ea = end_angle;
        end_angle = start_angle;
        start_angle = ea;
    }*/

    float path_len = abs(end_angle - start_angle);

    // Need to save parabola shape.
    float mps = max_angle_per_ms / path_len;
    float sa;
    float ea;

    // Need to correct parabola equation.
    if (end_angle > start_angle)
    {
        ea = end_angle;
        sa = start_angle;
    }
    else
    {
        ea = start_angle;
        sa = end_angle;
    }

    float ea_sa_fma = fma(-3.0, sa, ea);
    float mps_4_div_ea_sa_fma = 4.0 * mps / ea_sa_fma;

    // a_ = -4.0 * mps / ((ea - 3.0 * sa) * (ea + sa));
    a_ = -mps_4_div_ea_sa_fma / (ea + sa);
    // b_ = 4.0 * mps / (ea - 3.0 * sa);
    b_ = mps_4_div_ea_sa_fma;
    // c_ = sa * mps * (-1.0 + (-4.0 * (ea - sa) / (ea + sa))) / (ea - 3.0 * sa);
    c_ = sa * (-mps / ea_sa_fma - mps_4_div_ea_sa_fma * (ea - sa) / (ea + sa));

  Serial.print("S: ");
  Serial.println(sa);
  Serial.print("E: ");
  Serial.println(ea);
    LOG_VALUE("a = ", a_);
    LOG_VALUE("b = ", b_);
    LOG_VALUE("c = ", c_);
}


uint16_t ParabolicEaser::ease(uint16_t cur_angle)
{
  float angle_diff = a_ * square(cur_angle) + b_ * cur_angle + c_;    // fma(a, square(result), fma(b_, result, c_));

  Serial.print("AND: ");
  Serial.print(angle_diff);
  Serial.print(", ");
  if (!angle_diff) angle_diff = 1;
  else angle_diff = angle_diff > 0 ? ceil(angle_diff) : floor(angle_diff);

  Serial.println(angle_diff);

  angle_diff = (check_direction_) ? abs(angle_diff) : -abs(angle_diff);

  long int next_angle = cur_angle + angle_diff;

  if (next_angle <= 0 || ((check_direction_ && (next_angle >= end_angle_)) || (!check_direction_ && (next_angle <= end_angle_)))) return end_angle_;

  return next_angle;
}
