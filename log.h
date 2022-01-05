#ifndef LOG_H
#define LOG_H

#include <Arduino.h>

template <typename T, size_t S>
inline constexpr size_t get_file_name_offset(const T (& str)[S], size_t i = S - 1)
{
    return (str[i] == '/' || str[i] == '\\') ? i + 1 : (i > 0 ? get_file_name_offset(str, i - 1) : 0);
}

template <typename T>
inline constexpr size_t get_file_name_offset(T (& str)[1])
{
    return 0;
}


#define _LOG_PREFIX()        \
    Serial.print(&__FILE__[get_file_name_offset(__FILE__)]); \
    Serial.print(":");            \
    Serial.print(__LINE__);       \
    Serial.print(": ");


//#define _LOG_PREFIX()        \
//    Serial.print(__LINE__);       \
//    Serial.print(": ");

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

static const char _err_prefix[] = " [ERROR] ";

#define LOG_ERROR(message, value) \
{                                 \
    Serial.print(_err_prefix);    \
    _LOG_PREFIX();                \
    Serial.print(message);        \
    Serial.println(value);        \
}

#endif  // LOG_H
