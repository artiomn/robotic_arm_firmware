#include <Arduino.h>

#include "log.h"
#include "arm_servos.h"


void ArmServos::init_servos(unsigned int shoulder_rotate_pin, unsigned int shoulder_lift_pin,
                            unsigned int forearm_rotate_pin, unsigned int forearm_lift_pin,
                            unsigned int wrist_lift_pin, unsigned int manip_control_pin)
{
    init_servo(servo_motors_[smt_shoulder_rotate], shoulder_rotate_pin, initial_shoulder_angle, 0, 250); 
    init_servo(servo_motors_[smt_shoulder_lift], shoulder_lift_pin, initial_shoulder_lift);

    init_servo(servo_motors_[smt_forearm_lift], forearm_rotate_pin, initial_forearm_lift);
    init_servo(servo_motors_[smt_forearm_rotate], forearm_lift_pin, initial_forearm_angle, 0, 180);

    init_servo(servo_motors_[smt_wrist_lift], wrist_lift_pin, initial_manip_lift);
    init_servo(servo_motors_[smt_manip_control], manip_control_pin, 10, 0, initial_manip_angle);
}


void ArmServos::rotate_shoulder(int angle)
{
    rot_servo(servo_motors_[smt_shoulder_rotate], angle);
}


void ArmServos::lift_shoulder(int angle)
{
    rot_servo(servo_motors_[smt_shoulder_lift], angle);
}


void ArmServos::rotate_forearm(int angle)
{
    rot_servo(servo_motors_[smt_forearm_rotate], angle);
}


void ArmServos::lift_forearm(int angle)
{
    rot_servo(servo_motors_[smt_forearm_lift], angle);
}


void ArmServos::lift_manip(int angle)
{
    rot_servo(servo_motors_[smt_wrist_lift], angle);
}


void ArmServos::set_manip(int angle)
{
    rot_servo(servo_motors_[smt_manip_control], angle);
}


void ArmServos::open_manip()
{
    write_servo(servo_motors_[smt_manip_control], 0);
}


void ArmServos::close_manip()
{
    write_servo(servo_motors_[smt_manip_control], initial_manip_angle);
}


void ArmServos::init_shoulder(unsigned int shoulder_angle, unsigned int shoulder_lift)
{
    write_servo(servo_motors_[smt_shoulder_rotate], shoulder_angle);
    write_servo(servo_motors_[smt_shoulder_lift], shoulder_lift);
}


void ArmServos::init_shoulder()
{
    init_shoulder(initial_shoulder_angle, initial_shoulder_lift);
}


void ArmServos::init_forearm(unsigned int forearm_angle, unsigned int forearm_lift)
{
    write_servo(servo_motors_[smt_forearm_rotate], initial_forearm_angle);
    write_servo(servo_motors_[smt_forearm_lift], initial_forearm_lift);
}


void ArmServos::init_forearm()
{
    init_forearm(initial_forearm_angle, initial_forearm_lift);
}


void ArmServos::init_manip(unsigned int manip_angle, unsigned int manip_lift)
{
    write_servo(servo_motors_[smt_wrist_lift], manip_lift);
    write_servo(servo_motors_[smt_manip_control], manip_angle);
}


void ArmServos::visit(VisitorType::FunctionSignature visitor, void *data) const
{
    VisitorType f(visitor, data);

    for (int i = 0; i < sizeof(servo_motors_); ++i)
    {
        f(servo_motors_[i]);
    }
}


ServoMotor *ArmServos::servo_by_pin(unsigned int pin)
{
    for (int i = 0; i < sizeof(servo_motors_); ++i)
    {
        ServoMotor &motor = servo_motors_[i];
        if (motor.pin() == pin)
        {
            return &servo_motors_[i];
        }
    }

    return nullptr;
}


void ArmServos::set_rotate_handler(RotateHandler::FunctionSignature handler, void *data)
{
    on_rotate_ = RotateHandler(handler, data);
}


void ArmServos::write_servo(ServoMotor &servo, int angle)
{
    servo.write(angle);
    on_rotate_(this, servo, angle);
}


void ArmServos::init_servo(ServoMotor& servo, int pin, int angle, int min_angle, int max_angle, float speed, float accel)
{
    servo.attach(pin, min_angle, max_angle);
    write_servo(servo, angle);
}


void ArmServos::rotate_servo(ServoMotor &servo, int angle, int delay_ms, int delay_after_rotation)
{
    const int cur_angle = servo.read();
    const int rot_step = abs(cur_angle - angle);
    double angle_step = 1; // abs(cur_angle - angle) / rot_step;

    if (cur_angle > angle) angle_step = -angle_step;

    for (int i = 0; i < rot_step; ++i)
    {
        write_servo(servo, cur_angle + angle_step * i);
        delay(delay_ms);
    }

    write_servo(servo, angle);
    delay(delay_after_rotation);
}


void ArmServos::rot_servo(ServoMotor &servo, int angle)
{
    int cur_angle = servo.read();
    int new_angle = cur_angle + angle;

    if (new_angle > cur_angle)
    {
        for (int i = cur_angle; i <= new_angle; ++i)
        {
            write_servo(servo, i);
            delay(1);
        }
    }
    else
    {
        for (int i = cur_angle; i >= new_angle; --i)
        {
            write_servo(servo, i);
            delay(1);
        }
    }

    on_rotate_(this, servo, angle);
}
