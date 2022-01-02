#ifndef LOG_H
#define LOG_H


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

#endif  // LOG_H
