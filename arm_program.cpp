#include <Arduino.h>

#include "log.h"
#include "arm_program.h"


void rotate_handler(ArmServos *caller, const ServoMotor& motor, int angle, void *obj)
{
    ArmProgram *owner = reinterpret_cast<ArmProgram*>(obj);

    if (!owner->recording() || owner->started()) return;
    owner->add_action(motor, angle);
};


void servo_visitor(const ServoMotor &motor, void *obj)
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
    caller.set_rotate_handler([this](ArmServos *caller, const ServoMotor& motor, int angle) { rotate_handler(caller, motor, angle, this); });
    start_time_ = millis();
    LOG_VALUE(F("Program recording was started..."));
    recording_ = true;

    return true;
}


void ArmProgram::stop_recording()
{
    if (!recording_ || started_ || !caller_) return;
    recording_ = false;
    LOG_VALUE(F("Program recording was stopped..."));
    caller_->set_rotate_handler([](ArmServos *caller, const ServoMotor& motor, int angle) {});
    //prev_on_rotate_ = NULL;
    //caller_ = NULL;
}


bool ArmProgram::step()
{
    if (!started_ || recording_) return false;

    LOG_VALUE(F("Program step..."));
    if (current_action_->action_.interval_ + start_time_ < millis())
    {
        LOG_VALUE(F("Program step waiting..."));
        return true;
    }

    LOG_VALUE(F("Program step continued..."));
    ServoMotor* motor = caller_->servo_by_pin(current_action_->action_.pin_);
    if (!motor)
    {
        LOG_VALUE(F("Motor was not found, pin: "), current_action_->action_.pin_);
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
        LOG_VALUE(F("Program is empty."));
        return;
    }

    if (recording_ || started_) return;

    current_action_ = program_;
    start_time_ = millis();
    caller_ = &servos;
    started_ = true;
    LOG_VALUE(F("Program started..."));
}


void ArmProgram::stop()
{
    if (recording_ || !started_) return;

    started_ = false;
    current_action_ = nullptr;
    LOG_VALUE(F("Program stopped..."));
}


void ArmProgram::visit(VisitorType visitor, void *data) const
{
    for (const ProgramActionElement *pa = program_; pa != NULL; pa = pa->list_.next)
    {
        visitor(pa->action_);
    }
}


bool ArmProgram::add_initial_state(const ArmServos &servos)
{
    if (program_) return false;

    LOG_VALUE(F("Saving initial position..."));
    servos.visit([this](const ServoMotor &motor) { servo_visitor(motor, this); } );
    LOG_VALUE(F("Finished."));

    return true;
}


void ArmProgram::clear()
{
    LOG_VALUE(F("Clearing instructions..."));
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
    add_action(motor, const_cast<ServoMotor&>(motor).read());
}


void ArmProgram::add_action(const ServoMotor &motor, int angle)
{
    auto pin = motor.pin();
    LOG_VALUE(F("Adding action, pin: "), pin);
    LOG_VALUE(F("Angle: "), angle);

    ProgramActionElement *new_action = new ProgramActionElement(ProgramAction(pin, angle, millis() - start_time_));
    new_action->list_.next = NULL;

    if (!program_) program_ = last_action_ = new_action;
    else
    {
        last_action_->list_.next = new_action;
        last_action_ = new_action;
    }
}
