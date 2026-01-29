// Uncomment to see debug messages.
#define PS2X_DEBUG
#define PS2X_COM_DEBUG

#include <Servo.h>
#include <PS2X_lib.h>

#include <avr/sleep.h>

//#include <EEPROM.h>

#include "log.h"
#include "nonstd.h"
#include "arm_servos.h"
#include "joystick.h"
#include "jc_dualshock.h"
#include "arm_program.h"
#include "notifier.h"
#include "serial_commander.h"


void(* reboot) (void) = 0;


//
// Artiom N.(cl)2026
//

Notifier notifier(12);
Joystick joystick(5);
ArmServos arm;
ArmProgram program;
SerialCommander serial_commander;
const uint8_t Reset = 4;


void setup()
{
    Serial.begin(9600);
    delay(500);

    auto si_result = arm.init_servos();
    if (si_result != 0)
    {
        LOG_ERROR("Servo initializing, pin = ", si_result);
        notifier.notify_long(1);
        notifier.notify_short(1);
        notifier.stop_board();
    }

    joystick.on_find_joystick_ = [&](volatile JoystickController *joystick_controller)
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

        ds_joystick_controller.on_pad_ = [&](volatile JoystickController *, uint16_t button_code, byte value)
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
        
        ds_joystick_controller.on_button_ = [&](volatile JoystickController *caller, uint16_t button_code, byte value)
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
   
        ds_joystick_controller.on_left_stick_ = [&](volatile JoystickController *caller, int x_value, int y_value, bool clicked)
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
    
        ds_joystick_controller.on_right_stick_ = [&](volatile JoystickController *caller, int x_value, int y_value, bool clicked)
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

    serial_commander.set_command_handler([&](const char *command) -> bool
    {
        enum block_type
        {
            bt_shoulder = 's',
            bt_arm = 'f',
            bt_manip = 'm'
        };

        enum action_type
        {
            at_lift = 'l',
            at_rotate = 'r'
        } a_type;

        if (0 == strcmp(command, "reboot"))
        {
            reboot();
            return true;
        }

        if (command[0] != at_lift && command[0] != at_rotate)
        {
            Serial.println("Unknown action!");
            return false;
        }

        a_type = static_cast<action_type>(command[0]);

        switch (command[1])
        {
            case bt_shoulder:
                if (at_lift == a_type)
                {
                    arm.lift_shoulder(atoi(&command[2]));
                }
                else
                {
                    arm.rotate_shoulder(atoi(&command[2]));
                }
            break;
            case bt_arm:
                if (at_lift == a_type)
                {
                    arm.lift_forearm(atoi(&command[2]));
                }
                else
                {
                    arm.rotate_forearm(atoi(&command[2]));
                }
            break;
            case bt_manip:
                if (at_lift == a_type)
                {
                    arm.lift_manip(atoi(&command[2]));
                }
                else
                {
                    arm.set_manip(atoi(&command[2]));
                }
            break;
            default:
                Serial.println("Unknown block!");
                return false;
        }

        return true;
    }, nullptr);

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
    // Visit www.billporter.info for troubleshooting tips.
    notifier.tone(800, 500);
    Serial.println("Initialization completed.");
}


void loop()
{
    serial_commander.read_command();
    joystick.read_joystick();
    program.step();
    delay(50);

}
