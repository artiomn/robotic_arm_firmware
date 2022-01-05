// Uncomment to see debug messages.
#define PS2X_DEBUG
#define PS2X_COM_DEBUG

#include <Servo.h>
#include <PS2X_lib.h>

#include <avr/sleep.h>

//#include <EEPROM.h>

#include "log.h"
#include "call_handler.h"
#include "arm_servos.h"
#include "joystick.h"
#include "jc_dualshock.h"
#include "arm_program.h"


//
// Artiom N.(cl)2022
//

class Notifier
{
public:
    Notifier(uint8_t tone_pin, unsigned int tone_frequency = 700) :
      tone_pin_(tone_pin), tone_frequency_(tone_frequency)
    {}

    Notifier(uint8_t tone_pin,
             unsigned long short_duration,
             unsigned long long_duration,
             unsigned int tone_frequency = 700) :
      tone_pin_(tone_pin),
      short_duration_(short_duration),
      long_duration_(long_duration),
      tone_frequency_(tone_frequency)
    {}

public:
    void tone(unsigned int frequency, unsigned long duration)
    {
        ::tone(tone_pin_, frequency, duration);
    }

    void tone(unsigned long duration)
    {
        tone(tone_frequency_, duration);
    }

    void blink(unsigned long blink_time)
    {
        // Blinking with built-in led.
        switch_led(true);
        delay(blink_time);
        switch_led(false);
    }

    void notify(unsigned long duration)
    {
        switch_led(true);
        tone(duration);
        delay(duration);
        switch_led(false);
    }

    void notify(uint8_t count, unsigned long duration)
    {
        for (uint8_t i = 0; i < count; ++i)
        {
            notify(duration);
            delay(duration);
        }
    }

    void notify_short(uint8_t count)
    {
        notify(count, short_duration_);
    }

    void notify_long(uint8_t count)
    {
        notify(count, long_duration_);
    }

    void stop_board()
    {
        LOG_ERROR("Stopping the board", 1);
        set_sleep_mode(SLEEP_MODE_PWR_DOWN);
        cli();
        while (true)
        {
            sleep_enable();
            delay(100);
            sleep_cpu();
        }
    }

private:
    void switch_led(bool led_on)
    {
        digitalWrite(LED_BUILTIN, led_on ? HIGH : LOW);
    }

private:
    uint8_t tone_pin_;
    unsigned long short_duration_ = 180;
    unsigned long long_duration_ = 350;
    unsigned int tone_frequency_;
};


Notifier notifier(12);
Joystick joystick(5);
ArmServos arm;
ArmProgram program;


void setup()
{
    auto si_result = arm.init_servos();
    if (si_result != 0)
    {
        LOG_ERROR("Servo initializing, pin = ", si_result);
        notifier.notify_long(1);
        notifier.notify_short(1);
        notifier.stop_board();
    }

    joystick.on_find_joystick = [&](volatile JoystickController *joystick_controller)
    {
        auto jt = joystick_controller->type();
        if (jt != DualShockJC::controller_type)
        {
            LOG_ERROR("Unsupported joystick type = ", jt);
            notifier.notify_long(3);
            notifier.notify_short(2);
            notifier.stop_board();
        }

        LOG_VALUE("Dualshock controller was found");

        volatile DualShockJC &ds_joystick_controller = *static_cast<volatile DualShockJC*>(joystick_controller);

        ds_joystick_controller.on_pad = [&](volatile JoystickController *, uint16_t button_code, int value)
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
        
        ds_joystick_controller.on_button = [&](volatile JoystickController *caller, uint16_t button_code, int value)
        {
            switch (button_code)
            {
                case PSB_GREEN:
                    arm.open_manip();
                    caller->vibrate(200);
                break;
                case PSB_BLUE:
                    arm.close_manip();
                    caller->vibrate(200);
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
    
        ds_joystick_controller.on_left_stick = [&](volatile JoystickController *caller, int x_value, int y_value, bool clicked)
        {
            if (clicked)
            {
                arm.init_shoulder();
                caller->vibrate(150);
            }
            else
            {
    //            ArmServos::ServoData stat_x(arm.shoulder_rotate_stats());
    //            ArmServos::ServoData stat_y(arm.shoulder_lift_stats());
    //            map(x_value, -128, 128, stat_x.min_angle, stat_x.max_angle)
    //            map(y_value, -128, 128, stat_y.min_angle, stat_y.max_angle)
    
                arm.rotate_shoulder(-x_value / 6);
                arm.lift_shoulder(y_value / 8);
            }
        };
    
        ds_joystick_controller.on_right_stick = [&](volatile JoystickController *caller, int x_value, int y_value, bool clicked)
        {
            if (clicked)
            {
                arm.init_forearm();
                caller->vibrate(150);
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

    auto ji_result = joystick.init_joystick();

    switch (ji_result)
    {

        case Joystick::jerr_success:
        break;
        case Joystick::jerr_controller_not_found:
            LOG_ERROR("No joystick controller found, check wiring, error code =  ", ji_result);
            notifier.notify_long(2);
            notifier.notify_short(1);
            notifier.stop_board();
        break;
        case Joystick::jerr_controller_not_accept_commands:
            LOG_ERROR("Controller found but not accepting commands, error code =  ", ji_result);
            notifier.notify_long(2);
            notifier.notify_short(2);
            notifier.stop_board();
        break;
        case Joystick::jerr_controller_refuse_pressures_mode:
            // Not an error.
            LOG_ERROR("Joystick controller refusing to enter Pressures mode, may not support it, error code = ", ji_result);
            // notifier.notify_long(2);
            // notifier.notify_short(3);
            // notifier.stop_board();
        break;
        case Joystick::jerr_unknown_controller_type:
            LOG_ERROR("Unknown joystick controller type, error code = ", ji_result);
            notifier.notify_long(2);
            notifier.notify_short(4);
            notifier.stop_board();
        break;
        default:
            LOG_ERROR("Unknown joystick error = ", ji_result);
            notifier.notify_long(3);
            notifier.notify_short(1);
            notifier.stop_board();
    }
    // Visit www.billporter.info for troubleshooting tips.");
    notifier.tone(800, 500);
    Serial.println("Initialization completed.");
}


void loop()
{
    joystick.read_joystick();
    program.step();
    delay(50);

}
