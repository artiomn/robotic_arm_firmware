#ifndef LOG_H
#define LOG_H

#define _LOG_PREFIX_LONG()        \
    Serial.print(__FILE__);       \
    Serial.print(":");            \
    Serial.print(__LINE__);       \
    Serial.print(": ");

#define _LOG_PREFIX()             \
    Serial.print(__LINE__);       \
    Serial.print(": ");

#if defined(PS2X_DEBUG)

#define LOG_VALUE_MESSAGE(message, value) \
{                                 \
    _LOG_PREFIX();                \
    Serial.print(message);        \
    Serial.println(value);        \
}


#define LOG_MESSAGE(message)  \
{                             \
    _LOG_PREFIX();            \
    Serial.println(message);  \
}


#define _ARG2(_0, _1, _2, ...) _2
#define NARG2(...) _ARG2(__VA_ARGS__, 2, 1, 0)

#define _ONE_OR_TWO_ARGS_1(a) LOG_MESSAGE(a)
#define _ONE_OR_TWO_ARGS_2(a, b) LOG_VALUE_MESSAGE(a,b)

#define __ONE_OR_TWO_ARGS(N, ...) _ONE_OR_TWO_ARGS_ ## N (__VA_ARGS__)
#define _ONE_OR_TWO_ARGS(N, ...) __ONE_OR_TWO_ARGS(N, __VA_ARGS__)

#define LOG_VALUE(...) _ONE_OR_TWO_ARGS(NARG2(__VA_ARGS__), __VA_ARGS__)

#else
#define LOG_VALUE(...) { }
#endif // PS2X_DEBUG

static const char _err_prefix[] = "ERROR: ";

#define LOG_ERROR(message, value) \
{                                 \
    _LOG_PREFIX_LONG();           \
    Serial.print(_err_prefix);    \
    Serial.print(message);        \
    Serial.print(value);          \
}

#endif  // LOG_H
