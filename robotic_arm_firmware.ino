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
#include "arm_program.h"


//
// Artiom N.(cl)2021
//


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
