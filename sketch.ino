#include <Servo.h>
//#include <EEPROM.h>


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


// Uncomment to see debug messages.

#define PS2X_DEBUG
#define PS2X_COM_DEBUG

#include <PS2X_lib.h>


void log_value(const char *message, int value)
{
#ifdef PS2X_DEBUG
    Serial.print(message);
    Serial.println(value);
#endif
}

   
void log_value(const char *message)
{
#ifdef PS2X_DEBUG
    Serial.println(message);
#endif
}


template<typename Useless>
class CallHandler;

template<typename R, typename ...Args>
class CallHandler <R(Args...)>
{
public:
    typedef R(*FunctionSignature)(Args..., void*);

    CallHandler() {}
    CallHandler(FunctionSignature f, void *data) : f_(f), data_(data) {}

public:
    R operator()(Args... args)
    {
        if (f_) return f_(args..., data_);
    }
    
private:
    FunctionSignature f_;
    void *data_;
};


template<typename ...Args>
class CallHandler <void(Args...)>
{
public:
    typedef void(*FunctionSignature)(Args..., void*);

    CallHandler() {}
    CallHandler(FunctionSignature f, void *data) : f_(f), data_(data) {}

public:
    void operator()(Args... args)
    {
        if (f_) f_(args..., data_);
    }
    
private:
    FunctionSignature f_;
    void *data_;
};


class ServoMotor : public Servo
{
public:
    void attach(unsigned int pin, unsigned int min_angle, unsigned int max_angle)
    {
        // Min and Max are pulses in the Servo class.
        Servo::attach(pin, min_angle, max_angle);
        pin_ = pin;
        min_angle_ = min_angle;
        max_angle_ = max_angle;
    }


    
public:
    unsigned min_angle() const { return min_angle_; }
    unsigned max_angle() const { return max_angle_; }
    unsigned pin() const { return pin_; }

private:
  int min_angle_;
  int max_angle_;
  unsigned int pin_;
};


//
// Servo routines.
//

class ArmServos
{
public:
    const unsigned int initial_shoulder_angle = 60;
    const unsigned int initial_shoulder_lift = 90;
    const unsigned int initial_forearm_angle = 90;
    const unsigned int initial_forearm_lift = 50;
    const unsigned int initial_manip_lift = 90;
    const unsigned int initial_manip_angle = 120;

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
    void init_servos(unsigned int shoulder_rotate_pin = 8, unsigned int shoulder_lift_pin = 7,
                     unsigned int forearm_rotate_pin = 6, unsigned int forearm_lift_pin = 5,
                     unsigned int wrist_lift_pin = 4, unsigned int manip_control_pin = 3)
    {
        init_servo(servo_motors_[smt_shoulder_rotate], shoulder_rotate_pin, initial_shoulder_angle, 0, 250); 
        init_servo(servo_motors_[smt_shoulder_lift], shoulder_lift_pin, initial_shoulder_lift);

        init_servo(servo_motors_[smt_forearm_lift], forearm_rotate_pin, initial_forearm_lift);
        init_servo(servo_motors_[smt_forearm_rotate], forearm_lift_pin, initial_forearm_angle, 0, 180);

        init_servo(servo_motors_[smt_wrist_lift], wrist_lift_pin, initial_manip_lift);
        init_servo(servo_motors_[smt_manip_control], manip_control_pin, 10, 0, initial_manip_angle);
    }
    
    void rotate_shoulder(int angle)
    {
        rot_servo(servo_motors_[smt_shoulder_rotate], angle);
    }
    
    void lift_shoulder(int angle)
    {
        rot_servo(servo_motors_[smt_shoulder_lift], angle);
    }

    void rotate_forearm(int angle)
    {
        rot_servo(servo_motors_[smt_forearm_rotate], angle);
    }

    void lift_forearm(int angle)
    {
        rot_servo(servo_motors_[smt_forearm_lift], angle);
    }

    void lift_manip(int angle)
    {
        rot_servo(servo_motors_[smt_wrist_lift], angle);
    }

    void set_manip(int angle)
    {
        rot_servo(servo_motors_[smt_manip_control], angle);
    }

    void open_manip()
    {
        write_servo(servo_motors_[smt_manip_control], 0);
    }
    
    void close_manip()
    {
        write_servo(servo_motors_[smt_manip_control], 120);
    }

    void init_shoulder(unsigned int shoulder_angle, unsigned int shoulder_lift)
    {
        write_servo(servo_motors_[smt_shoulder_rotate], shoulder_angle);
        write_servo(servo_motors_[smt_shoulder_lift], shoulder_lift);
    }
    
    void init_shoulder()
    {
        init_shoulder(initial_shoulder_angle, initial_shoulder_lift);
    }
    
    void init_forearm(unsigned int forearm_angle, unsigned int forearm_lift)
    {
        write_servo(servo_motors_[smt_forearm_rotate], initial_forearm_angle);
        write_servo(servo_motors_[smt_forearm_lift], initial_forearm_lift);
    }
    
    void init_forearm()
    {
        init_forearm(initial_forearm_angle, initial_forearm_lift);
    }

    void init_manip(unsigned int manip_angle, unsigned int manip_lift)
    {
        write_servo(servo_motors_[smt_wrist_lift], manip_lift);
        write_servo(servo_motors_[smt_manip_control], manip_angle);
    }

public:
    void visit(VisitorType::FunctionSignature visitor, void *data) const
    {
        VisitorType f(visitor, data);
      
        for (int i = 0; i < sizeof(servo_motors_); ++i)
        {
            f(servo_motors_[i]);
        }
    }
    
    ServoMotor *servo_by_pin(unsigned int pin)
    {
        for (int i = 0; i < sizeof(servo_motors_); ++i)
        {
            ServoMotor &motor = servo_motors_[i];
            if (motor.pin() == pin)
            {
                return &servo_motors_[i];
            }
        }
        
        return NULL;
    }

    void set_rotate_handler(RotateHandler::FunctionSignature handler, void *data)
    {
        on_rotate_ = RotateHandler(handler, data);
    }

private:
    void write_servo(ServoMotor &servo, int angle)
    {
        servo.write(angle);
        on_rotate_(this, servo, angle);
    }

private:
    void init_servo(ServoMotor& servo, int pin, int angle, int min_angle = 0, int max_angle = 180, float speed = 10, float accel = 0.1)
    {
        servo.attach(pin, min_angle, max_angle);
        write_servo(servo, angle);
    }
    
    void rotate_servo(ServoMotor &servo, int angle, int delay_ms = 15, int delay_after_rotation = 50)
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
    
    void rot_servo(ServoMotor &servo, int angle)
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
    
private:
    ServoMotor servo_motors_[servo_count];
    RotateHandler on_rotate_;
};


//
// Joystick routines.
//

class JoystickController
{
public:
    const unsigned int selection_waiting_ms = 3000;
    const unsigned int selection_complete_vibration_speed = 190;
    const unsigned int selection_complete_vibration_ms = 300;

public:
    JoystickController(PS2X &control, unsigned int controller_type, unsigned int device_number) : control_(control), controller_type_(controller_type), device_number_(device_number) {};
    virtual ~JoystickController() {};

    virtual void process() = 0;
    virtual unsigned int type() { return controller_type_; }

public:
    typedef void (*ButtonClickHandler)(JoystickController* controller, unsigned int button_code, int value);
    typedef void (*StickHandler)(JoystickController* controller, int x_value, int y_value, boolean clicked);
    typedef void (*SelectHandler)(JoystickController* controller, unsigned int device_number);
    
public:
    ButtonClickHandler on_button;
    SelectHandler on_select;

protected:
    template<typename HandlerType, typename ...HandlerArgs>
    boolean call_handler(HandlerType handler, HandlerArgs... handler_args)
    {
        if (selected() && handler)
        {
            handler(this, handler_args...);
        }
    }

    void check_control_buttons()
    {
        if (!control_.NewButtonState()) return;

        if (control_.ButtonPressed(PSB_START))
        {
            log_value("Start is being held");
            if (device_number_)
            {
                if (!is_selection_in_process())
               {
                    log_value("Device selection started. My device number = ", device_number_);
                    newly_selected_number_ = 0;
                    selection_time_start_ = millis();
                    delay(1);
                }
                else
                {
                    selected_number_ = newly_selected_number_;
                    selection_time_start_ = 0;
                    log_value("Device selection finished. Selected device number = ", selected_number_);
                    if (selected())
                    {
                        log_value("It's me! I have been selected. Working...");
                        vibrate(selection_complete_vibration_ms, selection_complete_vibration_speed, selected_number_);
                        //call_handler(on_select, selected_number_);
                    }
                }
            }

            call_handler(on_button, PSB_START, control_.Analog(PSB_START));
        }

        if (control_.ButtonPressed(PSB_SELECT))
        {
            log_value("Select is being held.");
            if (is_selection_in_process())
            {
                selection_time_start_ = millis();
                ++newly_selected_number_;
                log_value("Newly selected number: ", newly_selected_number_);
            }
            call_handler(on_button, PSB_SELECT, control_.Analog(PSB_SELECT));
        }
    }

    boolean selected() const
    {
        return !device_number_ || (device_number_ == selected_number_);
    }

    template<const unsigned int button_id>
    boolean process_button_press(const char *log_message, ButtonClickHandler on_button_handler)
    {
        if (control_.ButtonPressed(button_id))
        {
            log_and_call<button_id, button_id>(log_message, on_button_handler);
            return true;
        }
        
        return false;
    }
    
    template<const unsigned int button_id>
    boolean process_button(const char *log_message, ButtonClickHandler on_button_handler)
    {
        if (control_.Button(button_id))
        {
            log_and_call<button_id, button_id>(log_message, on_button_handler);
            return true;
        }
        
        return false;
    }

    template<const unsigned int button_id, const unsigned int analog_button_id>
    boolean process_analog_button(const char *log_message, ButtonClickHandler on_button_handler)
    {
        if (control_.Button(button_id))
        {
            log_and_call<button_id, analog_button_id>(log_message, on_button_handler);
            return true;
        }
        
        return false;
    }

    virtual void vibrate(unsigned int ms, byte vibration_speed = 50, unsigned int vibration_count = 1) {}

protected:
    PS2X &control_;
    const unsigned zero_value_ = 128;

private:
    boolean is_selection_in_process() const
    {
        unsigned long ms = millis();
        return (ms > selection_waiting_ms) && ((ms - selection_time_start_) <= selection_waiting_ms);
    }
    
    template<const unsigned int button_id, const unsigned int analog_button_id>
    void log_and_call(const char *log_message, ButtonClickHandler on_button_handler)
    {
        log_value(log_message);
        call_handler(on_button_handler, button_id, control_.Analog(analog_button_id));
    }

private:
    const unsigned int controller_type_;
    const unsigned int device_number_;
    unsigned int selected_number_;
    unsigned int newly_selected_number_;
    unsigned long int selection_time_start_ = 0;
};


class DualShockJC : public JoystickController
{
public:
  static const unsigned int controller_type = 1;
  const unsigned stick_min_value = 2;

public:
    DualShockJC(PS2X &control, unsigned int device_number) : JoystickController(control, controller_type, device_number) {}

    void process() final override
    {
        // DualShock Controller
        // read controller and set large motor to spin at 'vibrate' speed
        boolean enable_vibration = false;

        if (vibration_count_)
        {
            unsigned int ms = millis();
            enable_vibration = ((vibration_start_time_ + vibration_period_) >= ms);

            if (!enable_vibration && ((vibration_start_time_ + 2 * vibration_period_) < ms))
            {
                if (--vibration_count_)
                {
                    vibration_start_time_ = ms;
                    enable_vibration = true;
                }
            }
        }
        
        control_.read_gamepad(false, enable_vibration ? vibration_speed_ : 0);

        check_control_buttons();

        // if (control_.NewButtonState())
        // Will be TRUE if any button changes state (on to off, or off to on).
        // Bad idea: I can't get buttons values.
        
        check_pad();
        check_figure_buttons();
        check_fore_buttons();

        get_sticks();
    }

    void vibrate(unsigned int ms, byte vibration_speed = 150, unsigned int vibration_count = 1) override
    {
        vibration_start_time_ = millis();
        vibration_period_ = ms;
        vibration_speed_ = vibration_speed;
        vibration_count_ = vibration_count;
    }

public:
  ButtonClickHandler on_pad;
  StickHandler on_left_stick;
  StickHandler on_right_stick;

private:
    void check_figure_buttons()
    {
        // ButtonPressed, ButtonReleased, NewButtonState.
        process_button_press<PSB_RED>("Circle just pressed", on_button);
        process_button_press<PSB_PINK>("Square just pressed", on_button);
        process_button_press<PSB_BLUE>("Cross just pressed", on_button);
        process_button_press<PSB_GREEN>("Triangle just pressed", on_button);
    }

    void check_fore_buttons()
    {
        process_button<PSB_L1>("L1 pressed", on_button);
        process_button<PSB_R1>("R1 pressed", on_button);
        process_button<PSB_L2>("L2 pressed", on_button);
        process_button<PSB_R2>("R2 pressed", on_button);
    }

    void check_pad()
    {
        process_analog_button<PSB_PAD_UP, PSAB_PAD_UP>("Up held", on_pad);
        process_analog_button<PSB_PAD_DOWN, PSAB_PAD_DOWN>("Down held", on_pad);
        process_analog_button<PSB_PAD_RIGHT, PSAB_PAD_RIGHT>("Right held", on_pad);
        process_analog_button<PSB_PAD_LEFT, PSAB_PAD_LEFT>("Left held", on_pad);
    }

    template<unsigned int x_const, unsigned int y_const, unsigned int btn_const>
    void check_stick(const char *msg, StickHandler on_stick)
    {
        int x_value = control_.Analog(x_const) - zero_value_;
        int y_value = zero_value_ - control_.Analog(y_const) + 1;
        boolean clicked = control_.Button(btn_const);

        if (selected() && on_stick && ((abs(x_value) > stick_min_value) || (abs(y_value) > stick_min_value) || clicked))
        {
            log_value(msg, x_value);
            log_value(msg, y_value);
            on_stick(this, x_value, y_value, clicked);
        }

    }

    void get_sticks()
    {
        check_stick<PSS_LX, PSS_LY, PSB_L3>("Left ", on_left_stick);
        check_stick<PSS_RX, PSS_RY, PSB_R3>("Right ", on_right_stick);
    }

private:
    int vibration_speed_;
    unsigned int vibration_period_ = 0;
    unsigned long vibration_start_time_ = 0;
    unsigned int vibration_count_ = 0;
};


class GuitarHeroJC : public JoystickController
{
public:
  static const unsigned int controller_type = 2;

public:
    GuitarHeroJC(PS2X &control, unsigned int device_number) : JoystickController(control, controller_type, device_number) {}

public:
  ButtonClickHandler on_button;
  StickHandler on_whammy_bar;

public:
    void process() final override
    {
        // This code WAS NOT TESTED!
        control_.read_gamepad();
        
        check_control_buttons();
        
        process_button_press<GREEN_FRET>("Green Fret Pressed", on_button);
        process_button_press<RED_FRET>("Red Fret Pressed", on_button);
        process_button_press<YELLOW_FRET>("Yellow Fret Pressed", on_button);
        process_button_press<BLUE_FRET>("Blue Fret Pressed", on_button);
        boolean orange_clicked = process_button_press<ORANGE_FRET>("Orange Fret Pressed", on_button);
        process_button_press<STAR_POWER>("Star Power Command", on_button);
        process_button<UP_STRUM>("Up Strum", on_button);
        process_button<DOWN_STRUM>("DOWN Strum", on_button);

        int value = control_.Analog(WHAMMY_BAR) - zero_value_;

        if (value || orange_clicked)
        {
            log_value("Wammy Bar Position: ", value);
            call_handler(on_whammy_bar, value, 0, orange_clicked);
        }
    }
};


class Joystick
{
public:
    Joystick(unsigned int device_number = 0) : device_number_(device_number), p_jimpl_(NULL) {}
    ~Joystick()
    {
        delete p_jimpl_;
    };

    void init_joystick(unsigned int clock_pin = A0, unsigned int command_pin = 13,
                       unsigned int attention_pin = 11, unsigned int data_pin = 12,
                       unsigned int serial_speed = 57600)
    {
        Serial.begin(serial_speed);

        // GamePad(clock, command, attention, data, Pressures?, Rumble?) 
        control_error_ = control_.config_gamepad(clock_pin, command_pin, attention_pin, data_pin, true, true);

        switch (control_error_)
        {
            case 0:
                Serial.println("Found Controller, configured successful.");
                Serial.println("Go to www.billporter.info for updates and to report bugs.");
            break;
            case 1:
                Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips.");
            break;
            case 2: 
                Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips.");
            break;
            case 3:
                Serial.println("Controller refusing to enter Pressures mode, may not support it.");
            break;
            default:
                Serial.println("Unknown error.");
        }

        switch (control_type_ = control_.readType())
        {
            case 0:
                Serial.println("Unknown Controller type");
            break;
            case DualShockJC::controller_type:
                Serial.println("DualShock Controller Found");
                p_jimpl_ = new DualShockJC(control_, device_number_);
            break;
            case GuitarHeroJC::controller_type:
                Serial.println("GuitarHero Controller Found");
                p_jimpl_ = new GuitarHeroJC(control_, device_number_);
            break;
        }
        
        if (on_find_joystick && p_jimpl_) on_find_joystick(p_jimpl_);
    }

    void read_joystick()
    {
       // You must Read Gamepad to get new values
       // Read GamePad and set vibration values
       // control.read_gamepad(small motor on/off, larger motor strenght from 0-255)
       // if you don't enable the rumble, use control.read_gamepad(); with no values you should call this at least once a second

        if (!p_jimpl_)
        {
            return;
        }

        p_jimpl_->process();
    }

public:
    typedef void (*CreateHandler)(JoystickController *joystick_controller);

public:
    CreateHandler on_find_joystick;

private:
    PS2X control_;
    JoystickController *p_jimpl_;
    int control_error_ = 0;
    byte control_type_ = 0;

    const unsigned int device_number_;
};


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
