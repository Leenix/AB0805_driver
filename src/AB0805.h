#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "DateTime.h"

#ifndef AB08x5_H
#define AB08x5_H

const uint8_t DEFAULT_ABO8x5_ADDRESS = 0x69;
const uint16_t AB08x5_YEAR_OFFSET = 2000;

enum AB08x5_WATCHDOG_FREQ {
    AB08x5_WATCHDOG_FREQ_16H = 0,
    AB08x5_WATCHDOG_FREQ_4H = 1,
    AB08x5_WATCHDOG_FREQ_1H = 2,
    AB08x5_WATCHDOG_FREQ_4S = 3,
};

enum AB08x5_WATCHDOG_OUTPUT {
    AB08x5_WATCHDOG_INTERRUPT = 0,
    AB08x5_WATCHDOG_RESET = 1,
};

enum AB08x5_OSCILLATOR_SELECT {
    AB08x5_OSCILLATOR_EXT_32K = 0,
    AB08x5_OSCILLATOR_128H_RC = 1,
};

enum AB08x5_COMMUNICATION_MODE {
    AB08x5_I2C_MODE = 0,
    AB08x5_SPI_MODE = 1,
};

enum AB08x5_TRICKLE_DIODE {
    AB08x5_TRICKLE_DIODE_STANDARD = 0x10,  // Assume Vf = 0.6 V
    AB08x5_TRICKLE_DIODE_SCHOTTKY = 0x01,  // Assum Vf = 0.3 V
    AB08x5_TRICKLE_CHARGE_DISABLED = 0,
};

enum AB08x5_TRICKLE_OUTPUT_RESISTANCE {
    AB08x5_TRICKLE_OUTPUT_RESISTANCE_DISABLED = 0,
    AB08x5_TRICKLE_OUTPUT_RESISTANCE_3K = 1,
    AB08x5_TRICKLE_OUTPUT_RESISTANCE_6K = 2,
    AB08x5_TRICKLE_OUTPUT_RESISTANCE_11K = 3,
};

enum AB08x5_INTERRUPT_OUTPUT_PIN { AB08x5_NIRQ, AB08x5_NIRQ2 };

///////////////////////////////////////////////////////////////////////////////
// STATUS

typedef union {
    uint8_t raw;
    struct {
        uint8_t ext1_triggered : 1;      // External trigger detected on the EXTI pin
        uint8_t ext2_triggered : 1;      // External interrupt triggered on the WDI pin
        uint8_t alarm_triggered : 1;     // The timer counters matched the alarm registers. Subject to masking
        uint8_t countdown_reached : 1;   // The countdown time has reached elapsed
        uint8_t vbat_crosses_bref : 1;   // The battery voltage crossed the reference voltage in the defined direction
        uint8_t watchdog_triggered : 1;  // The watchdog timer has been triggered
        uint8_t battery_active : 1;      // The clock is operating on battery power as opposed to supply voltage
        uint8_t century_flag : 1;        // Toggles when year rolls over from 99 to 0
    };
} ab08x5_status_t;

///////////////////////////////////////////////////////////////////////////////
// CONTROL1

enum AB08x5_HOUR_MODE { AB08x5_24_HOUR_MODE = 0, AB08x5_12_HOUR_MODE = 1 };

/**
 * Options for how interrupts are cleared.
 */
enum AB08x5_INTERRUPT_CLEAR_MODE {
    AB08x5_INT_CLEARED_ON_STATUS_READ = 1,            // Any read of the status register clears all interrupt flags
    AB08x5_INT_CLEARED_ON_EXPLICIT_STATUS_WRITE = 0,  // Interrupt flags are cleared by writing to the status register
};

typedef union {
    uint8_t raw;
    struct {
        uint8_t time_registers_write_enabled : 1;
        uint8_t _reserved1 : 1;
        uint8_t interrupt_reset_mode : 1;
        uint8_t _reserved2 : 1;
        uint8_t nirq_output_level : 1;
        uint8_t nirq2_output_level : 1;
        uint8_t hour_mode : 1;
        uint8_t stop : 1;
    };
} ab08x5_control_1_t;

///////////////////////////////////////////////////////////////////////////////
// CONTROL2

enum AB08x5_NIRQ_MODE {
    AB08x5_NIRQ_MODE_NIRQ = 0,           // Output nIRQ if at least one interrupt is enabled, else static output
    AB08x5_NIRQ_MODE_SQW = 1,            // Output square wave if SQWE is set, else static output
    AB08x5_NIRQ_MODE_SQW_ELSE_NIRQ = 2,  // Output square wave if SQWE, else nIRQ if >0 interrupts enabled -nIRQ1 only
    AB08x5_NIRQ_MODE_NAIRQ = 3,          // Output nAIRQ if AIE is set, else static output
    AB08x5_NIRQ_MODE_TIRQ = 4,           // TIRQ if TIE is set, else static output
    AB08x5_NIRQ_MODE_NTIRQ = 5,          // nTIRQ if TIE is set, else static output
    AB08x5_NIRQ_MODE_STATIC = 7          // Static output as set by nirq2_output/OUTB in CONTROL_1
};

typedef union {
    uint8_t raw;
    struct {
        uint8_t nirq_mode : 2;
        uint8_t nirq2_mode : 3;
        uint8_t _reserved : 3;
    };
} ab08x5_control_2_t;

///////////////////////////////////////////////////////////////////////////////
// INTERRUPT_MASK

enum AB08x5_ALARM_INTERRUPT_MODE {
    AB08x5_INTERRUPT_LATCHED = 0,
    AB08x5_INTERRUPT_PULSE_MIN = 1,
    AB08x5_INTERRUPT_PULSE_16MS = 2,
    AB08x5_INTERRUPT_PULSE_250MS = 3
};

typedef union {
    uint8_t raw;
    struct {
        uint8_t ext1_interrupt_enabled : 1;
        uint8_t ext2_interrupt_enabled : 1;
        uint8_t alarm_interrupt_enabled : 1;
        uint8_t timer_interrupt_enabled : 1;
        uint8_t battery_low_interrupt_enabled : 1;
        uint8_t alarm_interrupt_output_mode : 2;
        uint8_t century_enabled : 1;
    };
} ab08x5_interrupt_mask_t;

///////////////////////////////////////////////////////////////////////////////
// SQW

enum AB08x5_SQW_FREQUENCY {
    AB08x5_SQW_FREQUENCY_CENTURY = 0,
    AB08x5_SQW_FREQUENCY_32K = 1,
    AB08x5_SQW_FREQUENCY_8K = 2,
    AB08x5_SQW_FREQUENCY_4K = 3,
    AB08x5_SQW_FREQUENCY_2K = 4,
    AB08x5_SQW_FREQUENCY_1K = 5,
    AB08x5_SQW_FREQUENCY_512H = 6,
    AB08x5_SQW_FREQUENCY_256H = 7,
    AB08x5_SQW_FREQUENCY_128H = 8,
    AB08x5_SQW_FREQUENCY_64H = 9,
    AB08x5_SQW_FREQUENCY_32H = 10,
    AB08x5_SQW_FREQUENCY_16H = 11,
    AB08x5_SQW_FREQUENCY_8H = 12,
    AB08x5_SQW_FREQUENCY_4H = 13,
    AB08x5_SQW_FREQUENCY_2H = 14,
    AB08x5_SQW_FREQUENCY_1H = 15,
    AB08x5_SQW_FREQUENCY_2S = 16,
    AB08x5_SQW_FREQUENCY_4S = 17,
    AB08x5_SQW_FREQUENCY_8S = 18,
    AB08x5_SQW_FREQUENCY_16S = 19,
    AB08x5_SQW_FREQUENCY_32S = 20,
    AB08x5_SQW_FREQUENCY_MINUTE = 21,
    AB08x5_SQW_FREQUENCY_16K = 22,
    AB08x5_SQW_FREQUENCY_100H = 23,
    AB08x5_SQW_FREQUENCY_HOUR = 24,
    AB08x5_SQW_FREQUENCY_DAY = 25,
    AB08x5_SQW_FREQUENCY_TIRQ = 26,
    AB08x5_SQW_FREQUENCY_NOT_TIRQ = 27,
    AB08x5_SQW_FREQUENCY_YEAR = 28,
    AB08x5_SQW_FREQUENCY_1H_TO_COUNTERS = 29,
    AB08x5_SQW_FREQUENCY_32S_FROM_ACAL = 30,
    AB08x5_SQW_FREQUENCY_8S_FROM_ACAL = 31
};

typedef union {
    uint8_t raw;
    struct {
        uint8_t frequency : 5;
        uint8_t _reserved : 1;
        uint8_t enabled : 1;
    };
} ab08x5_sqw_config_t;

///////////////////////////////////////////////////////////////////////////////
// CALIBRATION_XT

enum AB08x5_CALIBRATION_MODE { AB08x5_CALIBRATION_2PPM = 0, AB08x5_CALIBRATION_4PPM = 1 };

typedef union {
    uint8_t raw;
    struct {
        uint8_t offset : 7;
        int8_t adjust_mode : 1;
    };
} ab08x5_xt_calibration_t;

///////////////////////////////////////////////////////////////////////////////
// INTERRUPT_MASK

enum AB08x5_EXTI_POLARITY { AB08x5_EXTI_FALLING = 0, AB08x5_EXTI_RISING = 1 };

typedef union {
    uint8_t raw;
    struct {
        uint8_t _reserved1 : 4;
        uint8_t ext1_polarity : 1;
        uint8_t ext2_polarity : 1;
        uint8_t _reserved2 : 2;
    };

} ab08x5_exti_polarity_t;

///////////////////////////////////////////////////////////////////////////////
// INTERRUPT_MASK

enum AB08x5_ALARM_REPEAT_MODE {
    AB08x5_ALARM_DISABLED = 0,
    AB08x5_ALARM_PER_YEAR = 1,    // Hundredths, seconds, minutes, hours, date, and month match
    AB08x5_ALARM_PER_MONTH = 2,   // Hundredths, seconds, minutes, hours, and date match
    AB08x5_ALARM_PER_WEEK = 3,    // Hundredths, seconds, minutes, hours, and weekday match
    AB08x5_ALARM_PER_DAY = 4,     // Hundredths, seconds, minutes, and hours match
    AB08x5_ALARM_PER_HOUR = 5,    // Hundredths, seconds, and minutes match
    AB08x5_ALARM_PER_MINUTE = 6,  // Hundredths, and seconds match
    AB08x5_ALARM_PER_SECOND = 7,  // Hundredths match exactly or according to mask.
    // A hundredths in the alarm register of 0xFn will trigger every 100ms, when n matches.
    // A value of 0xFF in the hundredths register will trigger the alarm every 10ms.
};

typedef union {
    uint8_t raw;
    struct {
        uint8_t timer_interrupt_pulse_width : 2;
        uint8_t alarm_repeat_mode : 3;
        uint8_t timer_repeat : 1;
        uint8_t timer_interrupt_mode : 1;
        uint8_t timer_enable : 1;
    };
} ab08x5_alarm_control_t;

///////////////////////////////////////////////////////////////////////////////
// INTERRUPT_MASK

typedef union {
    uint8_t raw;
    struct {
        uint8_t frequency : 2;
        uint8_t cycles : 5;
        uint8_t steering : 1;
    };
} ab08x5_watchdog_config_t;

///////////////////////////////////////////////////////////////////////////////
// INTERRUPT_MASK

typedef union {
    uint8_t raw;
    struct {
        uint8_t autocalibration_fail_interrupt_enabled : 1;
        uint8_t oscillator_fail_interrupt_enabled : 1;
        uint8_t _reserved : 1;
        uint8_t switch_to_rc_on_xt_fail : 1;
        uint8_t switch_to_rc_on_battery : 1;
        uint8_t autocalibration_mode : 2;
        uint8_t oscillator_select : 1;
    };
} ab08x5_osc_control_t;

///////////////////////////////////////////////////////////////////////////////
// INTERRUPT_MASK

typedef union {
    uint8_t raw;
    struct {
        uint8_t autocalibration_failure : 1;
        uint8_t oscillator_failure : 1;
        uint8_t _reserved : 2;
        uint8_t oscillator_mode : 1;
        uint8_t lock_nirq2 : 1;               // If 1, nIRQ2 cannot be set to HIGH
        uint8_t extended_xt_calibration : 2;  // The crystal's generated frequency is slowed by this field * 122ppm
    };
} ab08x5_osc_status_t;

///////////////////////////////////////////////////////////////////////////////
// INTERRUPT_MASK

typedef union {
    uint8_t raw;
    struct {
        uint8_t output_resistance : 2;
        uint8_t diode_select : 2;
        uint8_t trickle_mode : 4;
    };
} ab08x5_trickle_config_t;

///////////////////////////////////////////////////////////////////////////////
// INTERRUPT_MASK

typedef union {
    uint8_t raw;
    struct {
        uint8_t _reserved : 6;
        uint8_t exti_enabled_on_battery : 1;
        uint8_t wdi_enabled_on_battery : 1;
    };
} ab08x5_output_control_t;

///////////////////////////////////////////////////////////////////////////////
// INTERRUPT_MASK

typedef union {
    uint8_t raw;
    struct {
        uint8_t _reserved1 : 1;
        uint8_t vcc_above_min : 1;
        uint8_t _reserved2 : 4;
        uint8_t vbat_above_min : 1;
        uint8_t vbat_above_bref : 1;
    };
} ab08x5_analog_status_t;

///////////////////////////////////////////////////////////////////////////////
// INTERRUPT_MASK

class AB08x5 {
   public:
    bool begin(uint8_t comms_mode = AB08x5_I2C_MODE, uint8_t address_or_pin = DEFAULT_ABO8x5_ADDRESS);
    bool comm_check();

    // Read one of the RTC's status registers
    void read_status(ab08x5_status_t& status);
    void read_status(ab08x5_osc_status_t& status);
    void read_status(ab08x5_analog_status_t& status);

    // Write to the RTC's oscillator status register (usually to clear power failure flag)
    void write_osc_status(ab08x5_osc_status_t status);

    // Write a configuration register to the RTC
    void write_config(ab08x5_control_1_t config);
    void write_config(ab08x5_control_2_t config);
    void write_config(ab08x5_interrupt_mask_t config);
    void write_config(ab08x5_sqw_config_t config);
    void write_config(ab08x5_watchdog_config_t config);
    void write_config(ab08x5_alarm_control_t config);
    void write_config(ab08x5_osc_control_t config);

    // Read a configuration register from the RTC
    void read_config(ab08x5_control_1_t& config);
    void read_config(ab08x5_control_2_t& config);
    void read_config(ab08x5_interrupt_mask_t& config);
    void read_config(ab08x5_sqw_config_t& config);
    void read_config(ab08x5_watchdog_config_t& config);
    void read_config(ab08x5_alarm_control_t& config);
    void read_config(ab08x5_osc_control_t& config);

    // Read or write from the RTC user RAM space
    bool write_ram(uint8_t* intput, uint8_t address_offset, uint8_t size = 1);
    bool read_ram(uint8_t* output, uint8_t address_offset, uint8_t size = 1);

    // Time control
    DateTime now();
    void adjust(DateTime& dt);
    void adjust_to_compile_time();
    DateTime get_last_update_time();

    // Alarm control
    void set_alarm(DateTime& dt);
    DateTime read_alarm(DateTime& dt);
    void enable_alarm(uint8_t alarm_repeat_mode = AB08x5_ALARM_PER_SECOND);
    void disable_alarm();
    void enable_alarm_interrupts(uint8_t output_pin = AB08x5_NIRQ);

    // Get an ID from one of the RTC's ID registers (0-6)
    uint8_t get_id(uint8_t id_number);

   private:
    /**
     * Register map for the AB0805 and AB0815 real-time clock
     */
    typedef enum AB08x5_REGISTER {
        HUNDREDTHS = 0x00,
        SECONDS = 0x01,
        MINUTES = 0x02,
        HOURS = 0x03,
        DATE = 0x04,
        MONTHS = 0x05,
        YEARS = 0x06,
        WEEKDAYS = 0x07,
        HUNDREDTHS_ALARM = 0x08,
        SECONDS_ALARM = 0x09,
        MINUTES_ALARM = 0x0A,
        HOURS_ALARM = 0x0B,
        DATE_ALARM = 0x0C,
        MONTHS_ALARM = 0x0D,
        WEEKDAYS_ALARM = 0x0E,
        STATUS = 0x0F,
        CONTROL_1 = 0x10,
        CONTROL_2 = 0x11,
        INT_MASK = 0x12,
        SQW = 0x13,
        CAL_XT = 0x14,
        CAL_RC_HIGH = 0x15,
        CAL_RC_LOW = 0x16,
        INT_POLARITY = 0x17,
        TIMER_CONTROL = 0x18,
        TIMER = 0x19,
        TIMER_INITIAL = 0x1A,
        WDT = 0x1B,
        OSC_CONTROL = 0x1C,
        OSC_STATUS = 0x1D,
        CONFIG_KEY = 0x1F,
        TRICKLE = 0x20,
        BREF_CONTROL,
        AF_CONTROL,
        BAT_MODE,
        ID0 = 0x28,
        ID1 = 0x29,
        ID2 = 0x2A,
        ID3 = 0x2B,
        ID4 = 0x2C,
        ID5 = 0x2D,
        ID6 = 0x2E,
        ANALOG_STATUS = 0x2F,
        O_CONTROL = 0x30,
        EXTENSION_ADDRESS,
        RAM_START = 0x40,
        I2C_ONLY_RAM = 0x80
    } ab08x5_reg_t;

    typedef enum AB08x5_CONFIG_KEY {
        LOCK_SPECIAL_CONFIG = 0x00,
        UNLOCK_OSC_CONTROL = 0xA1,
        SOFTWARE_RESET = 0x3C,
        UNLOCK_BATTERY_CONTROL = 0x9D
    } ab08x5_config_key_t;

    enum AB08x5_RAM_SPACES { RAM_START = 0x40, NORMAL_RAM_END = 0x7F, I2C_MODE_RAM_END = 0xFF };

    uint8_t _comms_mode = AB08x5_I2C_MODE;
    DateTime _last_time_update;
    uint8_t _device_address = DEFAULT_ABO8x5_ADDRESS;
    uint8_t _chip_select_pin = 0;

    // Write to registers
    bool write(uint8_t* input, ab08x5_reg_t address, uint8_t length = 1);
    bool write_i2c(uint8_t* input, ab08x5_reg_t address, uint8_t length);
    bool write_spi(uint8_t* input, ab08x5_reg_t address, uint8_t length);

    // Read from registers
    bool read(uint8_t* output, ab08x5_reg_t address, uint8_t length = 1);
    bool read_i2c(uint8_t* output, ab08x5_reg_t address, uint8_t length);
    bool read_spi(uint8_t* output, ab08x5_reg_t address, uint8_t length);

    // Time register write control
    void unlock_time_registers();
    void lock_time_registers();

    // Oscillator configuration write control
    void unlock_oscillator_registers();
    void lock_oscillator_registers();
    void write_config_key(ab08x5_config_key_t key);

    // DateTime x RTC register conversions
    static void datetime_to_registers(DateTime& dt, uint8_t* output);
    static void registers_to_datetime(DateTime& dt, uint8_t* input);
    static uint8_t bcd_to_bin(uint8_t val) { return val - 6 * (val >> 4); }
    static uint8_t bin_to_bcd(uint8_t val) { return val + 6 * (val / 10); }
};

#endif