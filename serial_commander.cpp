#include "serial_commander.h"
#include <HardwareSerial.h>


extern HardwareSerial Serial;

void SerialCommander::read_command()
{
    while (Serial.available())
    {
        volatile int c = Serial.read();

        if (command_symbol_index_ >= max_command_len) c = '!';

        if (cmd_start_symbol == c)
        {
            if (command_symbol_index_ < min_command_len - 1)
            {
                command_symbol_index_ = 0;
                continue;
            }

            command_[command_symbol_index_] = '\0';
            bool command_result = on_command_(command_);
            Serial.print(command_result ? "+" : "-");
            Serial.println(command_);
            command_symbol_index_ = 0;
        }
        else
        {
            command_[command_symbol_index_++] = c;
        }
    }
};


void SerialCommander::set_command_handler(CommandHandler handler, void *data)
{
    on_command_ = handler; //, data);
}
