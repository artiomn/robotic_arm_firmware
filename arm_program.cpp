#include <Arduino.h>

#include "log.h"
#include "arm_program.h"


static void rotate_handler(ArmServos *caller, const ServoMotor& motor, int angle, void *obj)
{
    ArmProgram *owner = reinterpret_cast<ArmProgram*>(obj);

    if (!owner->recording() || owner->started()) return;
    owner->add_action(motor, angle);
};


static void servo_visitor(const ServoMotor &motor, void *obj)
{
    ArmProgram *owner = reinterpret_cast<ArmProgram*>(obj);
    owner->add_action(motor);
}


ArmProgram::~ArmProgram()
{
    clear();
}


bool ArmProgram::start_recording(ArmServos &caller)
{
    if (recording_ || started_) return false;

    clear();

    if (!add_initial_state(caller)) return false;
    caller_ = &caller;
    // prev_on_rotate_ = caller.on_rotate;
    caller.set_rotate_handler(rotate_handler, this);
    start_time_ = millis();
    log_value("Program recording was started...");
    recording_ = true;

    return true;
}


void ArmProgram::stop_recording()
{
    if (!recording_ || started_ || !caller_) return;
    recording_ = false;
    log_value("Program recording was stopped...");
    caller_->set_rotate_handler(NULL, NULL);
    //prev_on_rotate_ = NULL;
    //caller_ = NULL;
}


bool ArmProgram::step()
{
    if (!started_ || recording_) return false;

    log_value("Program step...");
    if (current_action_->action_.interval_ + start_time_ < millis())
    {
        log_value("Program step waiting...");
        return true;
    }

    log_value("Program step continued...");
    ServoMotor* motor = caller_->servo_by_pin(current_action_->action_.pin_);
    if (!motor)
    {
        log_value("Program error: motor was not found...");
        stop();
    }
    motor->write(current_action_->action_.angle_);

    current_action_ = current_action_->list_.next;
    if (!current_action_)
    {
        current_action_ = program_;
        start_time_ = millis();
    }

    return true;
}


void ArmProgram::start(ArmServos &servos)
{
    if (!program_)
    {
        log_value("Program is empty.");
        return;
    }

    if (recording_ || started_) return;

    current_action_ = program_;
    start_time_ = millis();
    caller_ = &servos;
    started_ = true;
    log_value("Program started...");
}


void ArmProgram::stop()
{
    if (recording_ || !started_) return;

    started_ = false;
    current_action_ = NULL;
    log_value("Program stopped...");
}


void ArmProgram::visit(VisitorType::FunctionSignature visitor, void *data) const
{
    VisitorType f(visitor, data);

    for (const ProgramActionElement *pa = program_; pa != NULL; pa = pa->list_.next)
    {
        f(pa->action_);
    }
}


bool ArmProgram::add_initial_state(const ArmServos &servos)
{
    if (program_) return false;

    servos.visit(servo_visitor, this);

    return true;
}


void ArmProgram::clear()
{
    log_value("Clearing program instructions...");
    ProgramActionElement *p = program_;
    ProgramActionElement *de;

    while (p)
    {
        de = p;
        p = p->list_.next;
        delete de;
    }

    program_ = last_action_ = NULL;
    caller_ = NULL;
}


void ArmProgram::add_action(const ServoMotor &motor)
{
    if (!program_) program_ = last_action_ = new ProgramActionElement(ProgramAction(motor.pin(), const_cast<ServoMotor&>(motor).read(), millis() - start_time_));
    else add_action(motor, const_cast<ServoMotor&>(motor).read());
}


void ArmProgram::add_action(const ServoMotor &motor, int angle)
{
    log_value("Adding program action...");
/*    ProgramActionElement *new_action = new ProgramActionElement(ProgramAction(motor.pin, angle, millis() - start_time_));
    new_action->list_.next = NULL;
    new_action->list_.prev = last_action_;

    last_action_->list_.next = new_action;
    last_action_ = new_action;*/
}
