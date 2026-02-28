#pragma once

#include "log.h"

class Notifier
{
public:
    Notifier(uint8_t tone_pin, unsigned int tone_frequency = 700) :
      tone_pin_(tone_pin), tone_frequency_(tone_frequency), board_stopped_(false)
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
    void complete_init()
    {
        if (board_stopped_) return;
        this->tone(800, 500);
        LOG_MESSAGE(F("Initialization completed."));
    }

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
        LOG_ERROR(F("Stopping the board"), 1);
        board_stopped_ = true;
        //set_sleep_mode(SLEEP_MODE_PWR_DOWN);
        // Bad: firmware can't be performed!
        //cli();
        while (true)
        {
            //sleep_enable();
            delay(100);
            //sleep_cpu();
        }
    }

    bool is_init_completed() const { return !board_stopped_; }

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
    bool board_stopped_;
};