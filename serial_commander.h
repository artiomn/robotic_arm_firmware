#ifndef SERIAL_COMMANDER_H
#define SERIAL_COMMANDER_H

#include "nonstd.h"


class SerialCommander
{
public:
    const char cmd_start_symbol = '!';
    static const int min_command_len = 3;
    static const int max_command_len = 7;

public:
    typedef nonstd::function<bool(const char* command)> CommandHandler;

public:
    void read_command();

public:
    void set_command_handler(CommandHandler handler, void *data);

private:
    int command_symbol_index_ = 0;
    char command_[max_command_len + 1];
    CommandHandler on_command_;
};

#endif
