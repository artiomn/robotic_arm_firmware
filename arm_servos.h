#ifndef ARM_SERVOS_H
#define ARM_SERVOS_H

#include "log.h"
#include "servo_motor.h"
#include "call_handler.h"


class ArmServos
{
public:
    const unsigned int initial_shoulder_angle = 60;
    const unsigned int initial_shoulder_lift = 90;
    const unsigned int initial_forearm_angle = 90;
    const unsigned int initial_forearm_lift = 50;
    const unsigned int initial_manip_lift = 90;
    const unsigned int initial_manip_angle = 110;

    enum ServoMotorType
    {
        smt_shoulder_rotate = 0,
        smt_shoulder_lift,
        smt_forearm_rotate,
        smt_forearm_lift,
        smt_wrist_lift,
        smt_manip_control
    };

    static const unsigned int servo_count = smt_manip_control - smt_shoulder_rotate + 1;

    typedef CallHandler<void(const ServoMotor &servo)> VisitorType;
    typedef CallHandler<void(ArmServos *caller, const ServoMotor &servo, int angle)> RotateHandler;

public:
    void init_servos(unsigned int shoulder_rotate_pin = 6, unsigned int shoulder_lift_pin = 5,
                     unsigned int forearm_rotate_pin = 4, unsigned int forearm_lift_pin = 3,
                     unsigned int wrist_lift_pin = 2, unsigned int manip_control_pin = 7);

    void rotate_shoulder(int angle);
    void lift_shoulder(int angle);
    void rotate_forearm(int angle);
    void lift_forearm(int angle);
    void lift_manip(int angle);
    void set_manip(int angle);
    void open_manip();
    void close_manip();
    void init_shoulder(unsigned int shoulder_angle, unsigned int shoulder_lift);
    void init_shoulder();
    void init_forearm(unsigned int forearm_angle, unsigned int forearm_lift);
    void init_forearm();
    void init_manip(unsigned int manip_angle, unsigned int manip_lift);

public:
    void visit(VisitorType::FunctionSignature visitor, void *data) const;
    ServoMotor *servo_by_pin(unsigned int pin);
    void set_rotate_handler(RotateHandler::FunctionSignature handler, void *data);

private:
    void write_servo(ServoMotor &servo, int angle);

private:
    void init_servo(ServoMotor& servo, int pin, int angle, int min_angle = 0, int max_angle = 180, float speed = 10, float accel = 0.1);
    void rotate_servo(ServoMotor &servo, int angle, int delay_ms = 15, int delay_after_rotation = 50);
    void rot_servo(ServoMotor &servo, int angle);

private:
    ServoMotor servo_motors_[servo_count];
    RotateHandler on_rotate_;
};

#endif  // ARM_SERVOS_H
