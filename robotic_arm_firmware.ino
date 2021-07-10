#include <Servo.h>
#include <PS2X_lib.h>
//#include <EEPROM.h>

// Uncomment to see debug messages.
#define PS2X_DEBUG
#define PS2X_COM_DEBUG

#include "log.h"
#include "call_handler.h"
#include "arm_servos.h"
#include "joystick.h"


//
// Artiom N.(cl)2021
//

/*
#define SS_SERVO_PERIOD 20		// период работы tick(), мс
#define SS_DEADTIME 10			// количество тиков до detach

byte SS_DEADZONE = 10;
byte SS_DEADZONE_SP = 3;

    int16_t _servoMaxSpeed = 1400;
    uint16_t _acceleration = 1000;


float _delta = SS_SERVO_PERIOD / 1000.0;


boolean Smooth::tickManual()
{
    if (_tickFlag)
    {
        int err = _servoTargetPos - _servoCurrentPos;

        // Stop condition
        if (abs(err) > SS_DEADZONE && abs(_lastSpeed - _speed) < SS_DEADZONE_SP)
        {
            if (_acceleration != 0)
            {
                bool thisDir = ((float)_speed * _speed / _acceleration / 2.0 >= abs(err));  	// пора тормозить
                _speed += (float)_acceleration * _delta * (thisDir ? -_sign(_speed) : _sign(err));
            }
            else
            {
                _speed = err / _delta;
            }
            _speed = constrain(_speed, -_servoMaxSpeed, _servoMaxSpeed);
            _servoCurrentPos += _speed * _delta;
            if (!_servoState)
            {
                _servoState = true;
                timeoutCounter = 0;
            }
            writeUs(_servoCurrentPos);
        }
        else
        {
            //_servoCurrentPos = _servoTargetPos;
            _speed = 0;			
            if (_servoState)
            {
                writeUs(_servoCurrentPos);
                timeoutCounter++;
            }
            if (timeoutCounter > SS_DEADTIME && _servoState)
            {
                _servoState = false;
                if (_autoDetach) detach();
            }
        }
        _lastSpeed = _speed;
    }
    return !_servoState;
}
*/


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


Joystick joystick(5);
ArmServos arm;
ArmProgram program;


void setup()
{
    tone(9, 500, 3000);
    delay(10);

    arm.init_servos();
//    pinMode(13, OUTPUT);
//    digitalWrite(13, LOW);
    joystick.on_find_joystick = [&](JoystickController *jc)
    {
        if (jc->type() != DualShockJC::controller_type)
        {
            Serial.println("Unsupported joystick type!");
            return;
        }
      
        DualShockJC *ds_jc = static_cast<DualShockJC*>(jc);

        ds_jc->on_pad = [&](JoystickController */*caller*/, unsigned int button_code, int value)
        {
            switch (button_code)
            {
                case PSB_PAD_UP:
                    arm.lift_manip(-value / 5);
                break;
                case PSB_PAD_DOWN:
                    arm.lift_manip(value / 5);
                break;
                case PSB_PAD_RIGHT:
                    arm.set_manip(value / 5);
                break;
                case PSB_PAD_LEFT:
                    arm.set_manip(-value / 5);
                break;
            }
        };
        
        ds_jc->on_button = [&](JoystickController *caller, unsigned int button_code, int value)
        {
            switch (button_code)
            {
                case PSB_GREEN:
                    arm.open_manip();
                    static_cast<DualShockJC*>(caller)->vibrate(200);
                break;
                case PSB_BLUE:
                    arm.close_manip();
                    static_cast<DualShockJC*>(caller)->vibrate(200);
                break;
                case PSB_RED:
                    // Recording user actions and then save them in the EEPROM.
                    program.start_recording(arm);
                break;
                case PSB_PINK:
                    // Stop recording without saving or stop user program execution.
                    program.stop_recording();
                    program.stop();
                break;
                case PSB_L1:
                case PSB_L2:
                    program.start(arm);
                break;
            }
        };
    
        ds_jc->on_left_stick = [&](JoystickController *caller, int x_value, int y_value, boolean clicked)
        {
            if (clicked)
            {
                arm.init_shoulder();
                static_cast<DualShockJC*>(caller)->vibrate(150);
            }
            else
            {
    //            ArmServos::ServoData stat_x(arm.shoulder_rotate_stats());
    //            ArmServos::ServoData stat_y(arm.shoulder_lift_stats());
    //            map(x_value, -128, 128, stat_x.min_angle, stat_x.max_angle)
    //            map(y_value, -128, 128, stat_y.min_angle, stat_y.max_angle)
    
                arm.rotate_shoulder(x_value / 6);
                arm.lift_shoulder(y_value / 6);
            }
        };
    
        ds_jc->on_right_stick = [&](JoystickController *caller, int x_value, int y_value, boolean clicked)
        {
            if (clicked)
            {
                arm.init_forearm();
                static_cast<DualShockJC*>(caller)->vibrate(150);
            }
            else
            {
    //            ArmServos::ServoData stat_x(arm.forearm_rotate_stats());
    //            ArmServos::ServoData stat_y(arm.forearm_lift_stats());
    //            map(x_value, -128, 128, stat_x.min_angle, stat_x.max_angle)
    //            map(y_value, -128, 128, stat_y.min_angle, stat_y.max_angle)
    
                arm.rotate_forearm(x_value / 6);
                arm.lift_forearm(y_value / 6);
            }
        };
    };
    
    joystick.init_joystick();

}


void loop()
{
   joystick.read_joystick();
   program.step();
   delay(50);
}
