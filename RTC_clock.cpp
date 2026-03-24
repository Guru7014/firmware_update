#include "RTC_clock.h"

// ====================================================
//                 DateTime METHODS
// ====================================================

void DateTime::setDateTime(uint16_t yr, uint8_t mon, uint8_t dt,
                           uint8_t hr, uint8_t min, uint8_t sec) {
  year = yr;
  month = mon;
  date = dt;
  hour = hr;
  minute = min;
  second = sec;
}

bool DateTime::isLeap(uint16_t y) {
  return (y % 4 == 0) && (y % 100 != 0 || y % 400 == 0);
}

int DateTime::daysInMonth(uint8_t m, uint16_t y) {
  static const uint8_t mdays[12] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

  if (m == 2) return isLeap(y) ? 29 : 28;
  return mdays[m - 1];
}

// ====================================================
//              ClockManager IMPLEMENTATION
// ====================================================

RTC_clock::RTC_clock()
#if defined(ESP_PLATFORM)
  : rtc(0x68)
#endif
{
}

// ----------------------------------------------------
void RTC_clock::begin() {
#if defined(ESP_PLATFORM)
  URTCLIB_WIRE.begin();
  rtc.refresh();

  rtc_error_flag = !rtc.refresh();

#elif defined(STM32F1xx) || defined(STM32G0xx)
  // Using HAL RTC
  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;
  if (HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN) == HAL_OK && HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN) == HAL_OK) {
    rtc_error_flag = false;
  } else {
    rtc_error_flag = true;
  }
  prev_tick_ms = HAL_GetTick();
#endif
}

// ----------------------------------------------------
void RTC_clock::update() {
  rtc_error_flag = !rtc.refresh();
  if (rtc_error_flag)
    internalClock();
  else
    externalClock();
}

// ----------------------------------------------------
void RTC_clock::externalClock() {
#if defined(ESP_PLATFORM)

  if (clock_sync_flag) {
    rtc.set(0, dt.minute, dt.hour,
            1, dt.date, dt.month,
            dt.year % 2000);
    clock_sync_flag = false;
  }

  dt.date = rtc.day();
  dt.month = rtc.month();
  dt.year = rtc.year() + 2000;
  dt.hour = rtc.hour();
  dt.minute = rtc.minute();
  dt.second = rtc.second();

#elif defined(STM32F1xx) || defined(STM32G0xx)

  RTC_TimeTypeDef gTime;
  RTC_DateTypeDef gDate;

  if (HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN) != HAL_OK || HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN) != HAL_OK) {
    rtc_error_flag = true;
    return;
  }

  dt.year = gDate.Year + 2000;
  dt.month = gDate.Month;
  dt.date = gDate.Date;

  dt.hour = gTime.Hours;
  dt.minute = gTime.Minutes;
  dt.second = gTime.Seconds;

#endif
}

// ----------------------------------------------------
void RTC_clock::internalClock() {
#if defined(ESP_PLATFORM)
  uint64_t now_ms = esp_timer_get_time() / 1000;

#elif defined(STM32F1xx) || defined(STM32G0xx)
  uint32_t now_ms = HAL_GetTick();
#endif

  if (now_ms - prev_tick_ms >= 1000) {

    prev_tick_ms = now_ms;

    if (++dt.second >= 60) {
      dt.second = 0;
      if (++dt.minute >= 60) {
        dt.minute = 0;
        if (++dt.hour >= 24) {
          dt.hour = 0;
          if (++dt.date > dt.daysInMonth(dt.month, dt.year)) {
            dt.date = 1;
            if (++dt.month > 12) {
              dt.month = 1;
              dt.year++;
            }
          }
        }
      }
    }
  }
}

void RTC_clock::syncFromNetwork() {
  // user can fill this with NTP or CAN time sync etc.
}
