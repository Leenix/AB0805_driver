#include "DateTime.h"

const char* TIMESTAMP_FORMAT = "%4d-%02d-%02d_%02d:%02d:%02d.%06d";
const uint8_t DAYS_IN_MONTH[] PROGMEM = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

// number of days since 2000/01/01, valid for 2001..2099
static uint16_t date_to_days(uint16_t y, uint8_t m, uint8_t d) {
    if (y >= 2000) y -= 2000;
    uint16_t days = d;
    for (uint8_t i = 1; i < m; ++i) days += pgm_read_byte(DAYS_IN_MONTH + i - 1);
    if (m > 2 && y % 4 == 0) ++days;
    return days + 365 * y + (y + 3) / 4 - 1;
}

/**
 * Convert time to total seconds
 */
static long time_to_long(uint16_t days, uint8_t h, uint8_t m, uint8_t s) {
    return ((days * 24L + h) * 60 + m) * 60 + s;
}

///////////////////////////////////////////////////////////////////////////////
// Datetime

/**
 * Constructor - DateTime
 * @param t: Total seconds since 1970-1-1
 * @param micros: microseconds since t
 */
DateTime::DateTime(uint32_t t, int32_t micros) {
    t -= SECONDS_FROM_1970_TO_2000;  // bring to 2000 timestamp from 1970

    t += set_microseconds(micros);

    ss = t % 60;
    t /= 60;
    mm = t % 60;
    t /= 60;
    hh = t % 24;
    uint16_t days = t / 24;
    uint8_t leap;
    for (yOff = 0;; ++yOff) {
        leap = yOff % 4 == 0;
        if (days < 365 + leap) break;
        days -= 365 + leap;
    }
    for (m = 1;; ++m) {
        uint8_t daysPerMonth = pgm_read_byte(DAYS_IN_MONTH + m - 1);
        if (leap && m == 2) ++daysPerMonth;
        if (days < daysPerMonth) break;
        days -= daysPerMonth;
    }
    d = days + 1;
}

/**
 * Constructor - DateTime
 * @param year: Calendar year of the timestamp
 * @param month: Calendar month of the timestamp
 * @param day: Calendar day of the timestamp (date, rather than day of week)
 * @param hour: Calendar hour of the timestamp
 * @param min: Calendar minute of the timestamp
 * @param sec: Calendar second of the timestamp
 * @param micros: Calendar microsecond of the timestamp
 */
DateTime::DateTime(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec, uint32_t micros) {
    if (year >= 2000) year -= 2000;
    yOff = year;
    m = month;
    d = day;
    hh = hour;
    mm = min;
    ss = sec;
    microseconds = micros;
}

/**
 * Constructor - Datetime
 * Copy the properties of another datetime instance.
 */
DateTime::DateTime(const DateTime& copy)
    : yOff(copy.yOff), m(copy.m), d(copy.d), hh(copy.hh), mm(copy.mm), ss(copy.ss), microseconds(copy.microseconds) {}

static uint8_t conv2d(const char* p) {
    uint8_t v = 0;
    if ('0' <= *p && *p <= '9') v = *p - '0';
    return 10 * v + *++p - '0';
}

/**
 * DateTime constructor.
 * Useful for creating a DateTime from compile time.
 * e.g: DateTime(__DATE__, __TIME__);
 */
DateTime::DateTime(const char* date, const char* time) {
    // sample input: date = "Dec 26 2009", time = "12:34:56"
    yOff = conv2d(date + 9);
    // Jan Feb Mar Apr May Jun Jul Aug Sep Oct Nov Dec
    switch (date[0]) {
        case 'J':
            m = (date[1] == 'a') ? 1 : ((date[2] == 'n') ? 6 : 7);
            break;
        case 'F':
            m = 2;
            break;
        case 'A':
            m = date[2] == 'r' ? 4 : 8;
            break;
        case 'M':
            m = date[2] == 'r' ? 3 : 5;
            break;
        case 'S':
            m = 9;
            break;
        case 'O':
            m = 10;
            break;
        case 'N':
            m = 11;
            break;
        case 'D':
            m = 12;
            break;
    }
    d = conv2d(date + 4);
    hh = conv2d(time);
    mm = conv2d(time + 3);
    ss = conv2d(time + 6);
}

/**
 * DateTime constructor.
 * Useful for creating a DateTime from compile time.
 * This version uses PROGMEM strings to conserve SRAM.
 * e.g: DateTime(F(__DATE__), F(__TIME__));
 */
DateTime::DateTime(const __FlashStringHelper* date, const __FlashStringHelper* time) {
    // sample input: date = "Dec 26 2009", time = "12:34:56"
    char buff[11];
    memcpy_P(buff, date, 11);
    yOff = conv2d(buff + 9);
    // Jan Feb Mar Apr May Jun Jul Aug Sep Oct Nov Dec
    switch (buff[0]) {
        case 'J':
            m = (buff[1] == 'a') ? 1 : ((buff[2] == 'n') ? 6 : 7);
            break;
        case 'F':
            m = 2;
            break;
        case 'A':
            m = buff[2] == 'r' ? 4 : 8;
            break;
        case 'M':
            m = buff[2] == 'r' ? 3 : 5;
            break;
        case 'S':
            m = 9;
            break;
        case 'O':
            m = 10;
            break;
        case 'N':
            m = 11;
            break;
        case 'D':
            m = 12;
            break;
    }
    d = conv2d(buff + 4);
    memcpy_P(buff, time, 8);
    hh = conv2d(buff);
    mm = conv2d(buff + 3);
    ss = conv2d(buff + 6);
    microseconds = 0;
}

/**
 * Get the day of the week of the current DateTime.
 * 0 = Sunday
 * 1 = Monday
 * 2 = Tuesday
 * 3 = Wednesday
 * 4 = Thursday
 * 5 = Friday
 * 6 = Saturday
 *
 * @return: Day of the week as an integer 0 - 6.
 */
uint8_t DateTime::day_of_the_week() const {
    uint16_t day = date_to_days(yOff, m, d);
    return (day + 6) % 7;  // Jan 1, 2000 is a Saturday, i.e. returns 6
}

/**
 * Get the current DateTime as a seconds timestamp since 1970-1-1.
 * This format fits the typical unix-style timestamp (sans microseconds).
 * @return: Unix-style timestamp in seconds.
 */
uint32_t DateTime::unix_time(void) const {
    uint32_t t;
    uint16_t days = date_to_days(yOff, m, d);
    t = time_to_long(days, hh, mm, ss);
    t += SECONDS_FROM_1970_TO_2000;  // seconds from 1970 to 2000

    return t;
}

/**
 * Get the total seconds timestamp since the 2000-1-1 epoch.
 * @return: Total seconds since 2000-1-1
 */
long DateTime::seconds_time(void) const {
    long t;
    uint16_t days = date_to_days(yOff, m, d);
    t = time_to_long(days, hh, mm, ss);
    return t;
}

/**
 * Add a datetime to the current.
 */
DateTime DateTime::operator+(const TimeSpan& span) {
    return DateTime(unix_time() + span.total_seconds(), microsecond() + span.microseconds());
}

/**
 * Subtract a datetime from the current.
 */
DateTime DateTime::operator-(const TimeSpan& span) {
    return DateTime(unix_time() - span.total_seconds(), microsecond() - span.microseconds());
}

/**
 * Subtract a timespan from the current datetime
 */
TimeSpan DateTime::operator-(const DateTime& right) {
    return TimeSpan(unix_time() - right.unix_time(), microsecond() - right.microsecond());
}

/**
 * Set the microseconds of the DateTime.
 * Overflows and underflows are handled by providing an offset to the DateTime's seconds to reflect the change.
 * @param micros: Setting for microseconds.
 * @return: Total seconds offset caused by microsecond overflows or underflows.
 */
int32_t DateTime::set_microseconds(int32_t micros) {
    int32_t seconds_offset = 0;
    while (micros >= 1000000) {
        seconds_offset++;
        micros -= 1000000;
    }
    while (micros < 0) {
        seconds_offset--;
        micros += 1000000;
    }
    microseconds = micros;
    return seconds_offset;
}

void DateTime::generate_timestamp(char* output) {  // Tidy up the datetime generations
    char buffer[strlen(TIMESTAMP_FORMAT)];
    snprintf(buffer, strlen(TIMESTAMP_FORMAT), TIMESTAMP_FORMAT, year(), month(), day(), hour(), minute(), second(),
             microsecond());
    strncpy(output, buffer, 40);
}

////////////////////////////////////////////////////////////////////////////////
// TimeSpan

/**
 * TimeSpan constructor
 * @param seconds: Total seconds
 * @param microseconds: microseconds elapsed after total seconds
 */
TimeSpan::TimeSpan(int32_t seconds, int32_t microseconds) {
    _seconds = seconds;
    _seconds += set_microseconds(microseconds);
}

/**
 * TimeSpan constructor
 */
TimeSpan::TimeSpan(int16_t days, int8_t hours, int8_t minutes, int8_t seconds, int32_t microseconds) {
    _seconds = (int32_t)days * 86400L + (int32_t)hours * 3600 + (int32_t)minutes * 60 + seconds;
    _seconds += set_microseconds(microseconds);
}

/**
 * TimeSpan constructor
 * Create a copy of another TimeSpan instance.
 * @param copy: TimeSpan instance to copy.
 */
TimeSpan::TimeSpan(const TimeSpan& copy) {
    _seconds = copy.seconds();
    _seconds += set_microseconds(copy.microseconds());
}

/**
 * Add a TimeSpan to the current instance.
 */
TimeSpan TimeSpan::operator+(const TimeSpan& right) {
    return TimeSpan(total_seconds() + right.total_seconds(), microseconds() + right.microseconds());
}

/**
 * Subtract a TimeSpan from the current instance.
 */
TimeSpan TimeSpan::operator-(const TimeSpan& right) {
    return TimeSpan(total_seconds() - right.total_seconds(), microseconds() - right.microseconds());
}

/**
 * Set the number of microseconds for a TimeSpan.
 * Overflows and underflows are handled and offset seconds are returned.
 * @return: Number of seconds offset by microsecond overflows or underflows
 */
int32_t TimeSpan::set_microseconds(int32_t micros) {
    int32_t seconds_offset = 0;
    while (micros >= 1000000) {
        seconds_offset++;
        micros -= 1000000;
    }
    while (micros < 0) {
        seconds_offset--;
        micros += 1000000;
    }
    _microseconds = micros;
    return seconds_offset;
}
