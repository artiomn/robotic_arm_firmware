#ifndef ARM_PROGRAM_H
#define ARM_PROGRAM_H

#include "log.h"
#include "arm_servos.h"
#include "call_handler.h"


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
    ~ArmProgram();

public:
    bool started() const { return started_; }
    bool recording() const { return recording_; }

public:
    bool start_recording(ArmServos &caller);
    void stop_recording();
    bool step();
    void start(ArmServos &servos);
    void stop();

public:
    void visit(VisitorType::FunctionSignature visitor, void *data) const;

public:
    bool add_initial_state(const ArmServos &servos);

public:
    void clear();
    void add_action(const ServoMotor &motor);
    void add_action(const ServoMotor &motor, int angle);

private:
    ProgramActionElement *program_;
    ProgramActionElement *last_action_;
    // For the program running.
    ProgramActionElement *current_action_;
    ArmServos *caller_;
    ArmServos::RotateHandler prev_on_rotate_;
    unsigned long start_time_;
    bool started_;
    bool recording_;
};

#endif  // ARM_PROGRAM_H
