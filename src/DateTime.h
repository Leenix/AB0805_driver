// Code by JeeLabs http://news.jeelabs.org/code/
// Released to the public domain! Enjoy!

#include <Arduino.h>

#ifndef DATETIME_H
#define DATETIME_H

#define SECONDS_PER_DAY 86400L
#define SECONDS_FROM_1970_TO_2000 946684800

// Timespan which can represent changes in time with seconds accuracy.
class TimeSpan {
   public:
    TimeSpan(int32_t seconds = 0, int32_t microseconds = 0);
    TimeSpan(int16_t days, int8_t hours, int8_t minutes, int8_t seconds, int32_t microseconds);
    TimeSpan(const TimeSpan& copy);

    int16_t days() const { return _seconds / 86400L; }
    int8_t hours() const { return _seconds / 3600 % 24; }
    int8_t minutes() const { return _seconds / 60 % 60; }
    int8_t seconds() const { return _seconds % 60; }
    int32_t total_seconds() const { return _seconds; }
    uint8_t hundredths() const { return _microseconds / 10000; }
    uint16_t milliseconds() const { return _microseconds / 1000; }
    uint32_t microseconds() const { return _microseconds; }

    TimeSpan operator+(const TimeSpan& right);
    TimeSpan operator-(const TimeSpan& right);

   protected:
    int32_t _seconds;
    int32_t _microseconds;
    int32_t set_microseconds(int32_t micros);
};

class DateTime {
   public:
    DateTime(uint32_t t = 0, int32_t micros = 0);
    DateTime(uint16_t year, uint8_t month, uint8_t day, uint8_t hour = 0, uint8_t min = 0, uint8_t sec = 0,
             uint32_t micros = 0);
    DateTime(const DateTime& copy);
    DateTime(const char* date, const char* time);
    DateTime(const __FlashStringHelper* date, const __FlashStringHelper* time);

    uint16_t year() const { return 2000 + yOff; }
    uint8_t month() const { return m; }
    uint8_t day() const { return d; }
    uint8_t hour() const { return hh; }
    uint8_t minute() const { return mm; }
    uint8_t second() const { return ss; }
    uint8_t hundredth() const { return microseconds / 10000; }
    uint16_t millisecond() const { return microseconds / 1000; }
    uint32_t microsecond() const { return microseconds; }
    uint8_t day_of_the_week() const;

    void generate_timestamp(char* output);

    // 32-bit times as seconds since 1/1/2000
    long seconds_time() const;
    // 32-bit times as seconds since 1/1/1970
    uint32_t unix_time(void) const;

    DateTime operator+(const TimeSpan& span);
    DateTime operator-(const TimeSpan& span);
    TimeSpan operator-(const DateTime& right);

   protected:
    uint8_t yOff, m, d, hh, mm, ss;
    uint32_t microseconds;
    int32_t set_microseconds(int32_t micros);
};

#endif