#ifndef ARM_PROGRAM_H
#define ARM_PROGRAM_H


#include "log.h"
#include "arm_motors.h"


class ArmProgram
{
public:
    template<typename ElementType>
    struct ListElement
    {
        ElementType *prev;
        ElementType *next;
    };

    struct ProgramAction
    {
        ProgramAction(unsigned int pin, int angle, unsigned long interval) : pin_(pin), angle_(angle), interval_(interval) {}

        unsigned int pin_;
        int angle_;
        unsigned long interval_;
    };

    typedef CallHandler<void(const ProgramAction&)> VisitorType;

private:
    struct ProgramActionElement
    {
        ProgramActionElement(ProgramAction action) : action_(action) {}

        ProgramAction action_;
        ListElement<ProgramActionElement> list_;
    };

public:
    ~ArmProgram()
    {
        clear();
    }

public:
    boolean started() const { return started_; }
    boolean recording() const { return recording_; }

public:
    boolean start_recording(ArmServos &caller)
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

    void stop_recording()
    {
        if (!recording_ || started_ || !caller_) return;
        recording_ = false;
        log_value("Program recording was stopped...");
        caller_->set_rotate_handler(NULL, NULL);
        //prev_on_rotate_ = NULL;
        //caller_ = NULL;
    }

    boolean step()
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

    void start(ArmServos &servos)
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

    void stop()
    {
        if (recording_ || !started_) return;

        started_ = false;
        current_action_ = NULL;
        log_value("Program stopped...");
    }

public:
    void visit(VisitorType::FunctionSignature visitor, void *data) const
    {
        VisitorType f(visitor, data);

        for (const ProgramActionElement *pa = program_; pa != NULL; pa = pa->list_.next)
        {
            f(pa->action_);
        }
    }

public:
    boolean add_initial_state(const ArmServos &servos)
    {
        if (program_) return false;

        servos.visit(servo_visitor, this);

        return true;
    }

public:
    void clear()
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

    void add_action(const ServoMotor &motor)
    {
        if (!program_) program_ = last_action_ = new ProgramActionElement(ProgramAction(motor.pin(), const_cast<ServoMotor&>(motor).read(), millis() - start_time_));
        else append_new_action(motor.pin(), const_cast<ServoMotor&>(motor).read());
    }

private:
    static void rotate_handler(ArmServos *caller, const ServoMotor& motor, int angle, void *obj)
    {
        ArmProgram *owner = reinterpret_cast<ArmProgram*>(obj);

        if (!owner->recording() || owner->started()) return;
        owner->append_new_action(motor.pin(), angle);
    };

    static void servo_visitor(const ServoMotor &motor, void *obj)
    {
        ArmProgram *owner = reinterpret_cast<ArmProgram*>(obj);
        owner->add_action(motor);
    }

    void append_new_action(unsigned int pin, int angle)
    {
        log_value("Adding program action...");
        ProgramActionElement *new_action = new ProgramActionElement(ProgramAction(pin, angle, millis() - start_time_));
        new_action->list_.next = NULL;
        new_action->list_.prev = last_action_;

        last_action_->list_.next = new_action;
        last_action_ = new_action;
    }

private:
    ProgramActionElement *program_;
    ProgramActionElement *last_action_;
    // For the program running.
    ProgramActionElement *current_action_;
    ArmServos *caller_;
    ArmServos::RotateHandler prev_on_rotate_;
    unsigned long start_time_;
    boolean started_;
    boolean recording_;
};

#endif  // ARM_PROGRAM_H
