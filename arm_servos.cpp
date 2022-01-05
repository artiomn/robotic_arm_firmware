#include <Arduino.h>
#include <util/atomic.h>

#include "log.h"
#include "arm_servos.h"
#include "servo_smoother.h"


int ArmServos::init_servos(uint8_t shoulder_rotate_pin, uint8_t shoulder_lift_pin,
                           uint8_t forearm_rotate_pin, uint8_t forearm_lift_pin,
                           uint8_t wrist_lift_pin, uint8_t manip_control_pin)
{
    if (!init_servo(servo_motors_[smt_shoulder_rotate], shoulder_rotate_pin, initial_shoulder_angle, 0, 250)) return shoulder_rotate_pin;
    if (!init_servo(servo_motors_[smt_shoulder_lift], shoulder_lift_pin, initial_shoulder_lift, 0, 160)) return shoulder_lift_pin;

    if (!init_servo(servo_motors_[smt_forearm_lift], forearm_rotate_pin, initial_forearm_lift, 5, 120)) return forearm_rotate_pin;
    if (!init_servo(servo_motors_[smt_forearm_rotate], forearm_lift_pin, initial_forearm_angle, 0, 180)) return forearm_lift_pin;

    if (!init_servo(servo_motors_[smt_wrist_lift], wrist_lift_pin, initial_manip_lift)) return wrist_lift_pin;
    if (!init_servo(servo_motors_[smt_manip_control], manip_control_pin, initial_manip_angle, 5, initial_manip_angle)) return manip_control_pin;

    return 0;
}


void ArmServos::rotate_shoulder(int angle)
{
    rotate_servo(servo_motors_[smt_shoulder_rotate], angle);
}


void ArmServos::lift_shoulder(int angle)
{
    rotate_servo(servo_motors_[smt_shoulder_lift], angle);
}


void ArmServos::rotate_forearm(int angle)
{
    rotate_servo(servo_motors_[smt_forearm_rotate], angle);
}


void ArmServos::lift_forearm(int angle)
{
    rotate_servo(servo_motors_[smt_forearm_lift], angle);
}


void ArmServos::lift_manip(int angle)
{
    rotate_servo(servo_motors_[smt_wrist_lift], angle);
}


void ArmServos::set_manip(int angle)
{
    rotate_servo(servo_motors_[smt_manip_control], angle);
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

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        for (size_t i = 0; i < servo_count; ++i)
        {
            f(servo_motors_[i]);
        }
    }
}


ServoMotor *ArmServos::servo_by_pin(uint8_t pin)
{
    ServoMotor *result = nullptr;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        for (size_t i = 0; i < servo_count; ++i)
        {
            if (servo_motors_[i].pin() == pin)
            {
                result = &servo_motors_[i];
                break;
            }
        }
    }

    return result;
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


bool ArmServos::init_servo(ServoMotor& servo, uint8_t pin, int angle, int min_angle, int max_angle, float speed, float accel)
{
    if (!servo.attach(pin, min_angle, max_angle)) return false;
    write_servo(servo, angle);

    return true;
}


void ArmServos::rotate_servo(ServoMotor &servo, int angle)
{
    servo.rotate_to(angle);
    on_rotate_(this, servo, angle);
    //rot_servo1(servo, angle);
}
