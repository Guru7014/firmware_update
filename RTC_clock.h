#ifndef RTC_CLOCK_H
#define RTC_CLOCK_H

#include <stdint.h>

// ----------------- PLATFORM CHECK -----------------
#if defined(ESP_PLATFORM)
    #include <Arduino.h>
    #include <uRTCLib.h>
#elif defined(STM32F1xx) || defined(STM32G0xx)
    #include "main.h"
    #include "rtc.h"
#endif

// ====================================================
//                 DateTime CLASS
// ====================================================
class DateTime {
public:
    uint16_t year = 2000;
    uint8_t month = 1, date = 1;
    uint8_t hour = 0, minute = 0, second = 0;

    uint16_t sync_year = 0;
    uint8_t sync_month = 0, sync_date = 0;
    uint8_t sync_hour = 0, sync_minute = 0;

    DateTime() {}

    void applySync();
    void setDateTime(uint16_t yr, uint8_t mon, uint8_t dt,
                     uint8_t hr, uint8_t min, uint8_t sec = 0);

    void setSync(uint16_t yr, uint8_t mon, uint8_t dt,
                 uint8_t hr, uint8_t min);

    int daysInMonth(uint8_t m, uint16_t y);
    bool isLeap(uint16_t y);
};

// ====================================================
//                ClockManager CLASS
// ====================================================
class RTC_clock {
public:
    DateTime dt;

    bool rtc_error_flag = false;
    bool clock_sync_flag = false;

    // platform dependent
    uint32_t prev_tick_ms = 0;
    uint64_t prev_us = 0;

#if defined(ESP_PLATFORM)
    uRTCLib rtc;
#endif

public:
    RTC_clock();

    void begin();         // init RTC
    void update();        // call every cycle
    void externalClock(); // RTC
    void internalClock(); // fallback

    void syncFromNetwork(); // optional user hook
};

#endif
