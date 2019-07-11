#include "AB0805.h"
#include "ArduinoLog.h"

///////////////////////////////////////////////////////////////////////////////

/**
 * Write to the RTC's registers.
 *
 * @param input: Address of data to write to the register.
 * @param address: Address of register to write to.
 * @return: Success/error result of the write.
 */
bool AB08x5::write(uint8_t *input, ab08x5_reg_t address, uint8_t length) {
    bool success = false;
    if (_comms_mode == AB08x5_I2C_MODE) success = write_i2c(input, address, length);
    if (_comms_mode == AB08x5_SPI_MODE) success = write_spi(input, address, length);
    return success;
}

/**
 * Write a value to a register using I2C
 *
 * @param input: Byte to write to the register.
 * @param address: Address of register to write to.
 * @return: Success/error result of the write.
 */
bool AB08x5::write_i2c(uint8_t *input, ab08x5_reg_t address, uint8_t length) {
    bool result = true;
    Wire.beginTransmission(_device_address);
    Wire.write(address);
    for (size_t i = 0; i < length; i++) {
        Wire.write(input[i]);
    }

    if (Wire.endTransmission() != 0) {
        result = false;
    }
    return result;
}

/**
 * Write a a value to a register using SPI.
 *
 * @param input: Byte to write to the register.
 * @param address: Address of register to write to.
 * @return: Success/error result of the write.
 */
bool AB08x5::write_spi(uint8_t *input, ab08x5_reg_t address, uint8_t length) {
    digitalWrite(_chip_select_pin, LOW);
    SPI.transfer(address);
    for (size_t i = 0; i < length; i++) {
        SPI.transfer(input[i]);
    }
    digitalWrite(_chip_select_pin, HIGH);
    return true;
}

///////////////////////////////////////////////////////////////////////////////

/**
 * Read from the RTC's registers.
 * @param output: Address to read data into.
 * @param address: Address of the register to read data from.
 * @param length: Number of bytes to read.
 * @return: Success/fail of the read.
 */
bool AB08x5::read(uint8_t *output, ab08x5_reg_t address, uint8_t length) {
    bool success = false;
    if (_comms_mode == AB08x5_I2C_MODE) success = read_i2c(output, address, length);
    if (_comms_mode == AB08x5_SPI_MODE) success = read_spi(output, address, length);
    return success;
}

/**
 * Read a specified number of bytes using the I2C bus.
 * @param output: The buffer in which to store the read values.
 * @param address: Register address to read (or starting address in burst reads)
 * @param length: Number of bytes to read.
 */
bool AB08x5::read_i2c(uint8_t *output, ab08x5_reg_t address, uint8_t length) {
    bool result = true;
    Wire.beginTransmission(_device_address);
    Wire.write(address);
    if (Wire.endTransmission() != 0)
        result = false;

    else  // OK, all worked, keep going
    {
        Wire.requestFrom(_device_address, length);
        for (size_t i = 0; (i < length) and Wire.available(); i++) {
            uint8_t c = Wire.read();
            output[i] = c;
        }
    }
    return result;
}

/**
 * Read a specified number of bytes using the SPI bus.
 * @param output: The buffer in which to store the read values.
 * @param address: Register address to read (or starting address in burst reads)
 * @param length: Number of bytes to read.
 */
bool AB08x5::read_spi(uint8_t *output, ab08x5_reg_t address, uint8_t length) {
    bool result = true;
    uint8_t num_empty_bytes = 0;

    digitalWrite(_chip_select_pin, LOW);
    SPI.transfer(address | 0x80 | 0x40);
    for (size_t i = 0; i < length; i++) {
        uint8_t c = SPI.transfer(0x00);
        if (c == 0xFF) num_empty_bytes++;
        *output = c;
        output++;
    }
    if (num_empty_bytes == length) result = false;
    digitalWrite(_chip_select_pin, HIGH);

    return result;
}

///////////////////////////////////////////////////////////////////////////////

/**
 * Start communications with the RTC.
 *
 * @param comms_mode: Communications mode to use to interact with the RTC [I2C, SPI]
 * @param address_or_pin: I2C address of the RTC or CS pin connected to the RTC if using SPI mode.
 * @return: True if communications could be started successfully.
 */
bool AB08x5::begin(uint8_t comms_mode, uint8_t address_or_pin) {
    _comms_mode = comms_mode;
    if (comms_mode == AB08x5_I2C_MODE) _device_address = address_or_pin;
    if (comms_mode == AB08x5_SPI_MODE) _chip_select_pin = address_or_pin;
    return comm_check();
}

/**
 * Check communication with the RTC by reading the ID.
 */
bool AB08x5::comm_check() {
    bool success = false;
    uint8_t family_id;
    read(&family_id, AB08x5_REGISTER::ID0);
    if (family_id == 0x08) success = true;
    return success;
}

///////////////////////////////////////////////////////////////////////////////

/**
 * Read the status from the RTC.
 * @param status: Status object to read active status into.
 */
void AB08x5::read_status(ab08x5_status_t &status) { read((uint8_t *)&status, AB08x5_REGISTER::STATUS); }

/**
 * Read the status from the RTC.
 * @param status: Status object to read active status into.
 */
void AB08x5::read_status(ab08x5_osc_status_t &status) { read((uint8_t *)&status, AB08x5_REGISTER::OSC_STATUS); }

/**
 * Read the status from the RTC.
 * @param status: Status object to read active status into.
 */
void AB08x5::read_status(ab08x5_analog_status_t &status) { read((uint8_t *)&status, AB08x5_REGISTER::ANALOG_STATUS); }

///////////////////////////////////////////////////////////////////////////////

/**
 * Write to the RTC's oscillator status register.
 * Mainly used to reset the oscillator failure flag after a loss of power or oscillator malfunction.
 *
 * @param status: Status object to write to the register.
 */
void AB08x5::write_osc_status(ab08x5_osc_status_t status) { write((uint8_t *)&status, AB08x5_REGISTER::OSC_STATUS); }

///////////////////////////////////////////////////////////////////////////////

/**
 * Write a configuration to the RTC.
 * @param config: Configuration object to write to the RTC.
 */
void AB08x5::write_config(ab08x5_control_1_t config) { write((uint8_t *)&config, AB08x5_REGISTER::CONTROL_1); }

/**
 * Write a configuration to the RTC.
 * @param config: Configuration object to write to the RTC.
 */
void AB08x5::write_config(ab08x5_control_2_t config) { write((uint8_t *)&config, AB08x5_REGISTER::CONTROL_2); }

/**
 * Write a configuration to the RTC.
 * @param config: Configuration object to write to the RTC.
 */
void AB08x5::write_config(ab08x5_interrupt_mask_t config) { write((uint8_t *)&config, AB08x5_REGISTER::INT_MASK); }

/**
 * Write a configuration to the RTC.
 * @param config: Configuration object to write to the RTC.
 */
void AB08x5::write_config(ab08x5_sqw_config_t config) { write((uint8_t *)&config, AB08x5_REGISTER::SQW); }

/**
 * Write a configuration to the RTC.
 * @param config: Configuration object to write to the RTC.
 */
void AB08x5::write_config(ab08x5_watchdog_config_t config) { write((uint8_t *)&config, AB08x5_REGISTER::WDT); }

/**
 * Write a configuration to the RTC.
 * @param config: Configuration object to write to the RTC.
 */
void AB08x5::write_config(ab08x5_alarm_control_t config) { write((uint8_t *)&config, AB08x5_REGISTER::TIMER_CONTROL); }

/**
 * Write a configuration to the RTC.
 * @param config: Configuration object to write to the RTC.
 */
void AB08x5::write_config(ab08x5_osc_control_t config) {
    unlock_oscillator_registers();
    write((uint8_t *)&config, AB08x5_REGISTER::OSC_CONTROL);
    lock_oscillator_registers();
}

///////////////////////////////////////////////////////////////////////////////

/**
 * Read a configuration from the RTC.
 * @param config: Configuration object to read the active configuration into.
 */
void AB08x5::read_config(ab08x5_control_1_t &config) { read((uint8_t *)&config, AB08x5_REGISTER::CONTROL_1); }

/**
 * Read a configuration from the RTC.
 * @param config: Configuration object to read the active configuration into.
 */
void AB08x5::read_config(ab08x5_control_2_t &config) { read((uint8_t *)&config, AB08x5_REGISTER::CONTROL_2); }

/**
 * Read a configuration from the RTC.
 * @param config: Configuration object to read the active configuration into.
 */
void AB08x5::read_config(ab08x5_interrupt_mask_t &config) { read((uint8_t *)&config, AB08x5_REGISTER::INT_MASK); }

/**
 * Read a configuration from the RTC.
 * @param config: Configuration object to read the active configuration into.
 */
void AB08x5::read_config(ab08x5_sqw_config_t &config) { read((uint8_t *)&config, AB08x5_REGISTER::SQW); }

/**
 * Read a configuration from the RTC.
 * @param config: Configuration object to read the active configuration into.
 */
void AB08x5::read_config(ab08x5_watchdog_config_t &config) { read((uint8_t *)&config, AB08x5_REGISTER::WDT); }

/**
 * Read a configuration from the RTC.
 * @param config: Configuration object to read the active configuration into.
 */
void AB08x5::read_config(ab08x5_alarm_control_t &config) { read((uint8_t *)&config, AB08x5_REGISTER::TIMER_CONTROL); }

/**
 * Read a configuration from the RTC.
 * @param config: Configuration object to read the active configuration into.
 */
void AB08x5::read_config(ab08x5_osc_control_t &config) { read((uint8_t *)&config, AB08x5_REGISTER::OSC_CONTROL); }

///////////////////////////////////////////////////////////////////////////////

/**
 * Write data to the RTC's user memory.
 * Writes that are out of the memory's range will result in a failure and will not be written.
 *
 * @param input: Start address of the data to be written.
 * @param address_offset: Address in the RTC's user memory to start writing data to. [0 - 0x3F (or 0xBF in I2C mode)]
 * @param size: Number of bytes to write.
 * @return: True if the data was written.
 */
bool AB08x5::write_ram(uint8_t *input, uint8_t address_offset, uint8_t size) {
    uint8_t max_address = NORMAL_RAM_END;
    if (_comms_mode == AB08x5_I2C_MODE) max_address = I2C_MODE_RAM_END;

    bool success = false;

    // Ensure there is enough room to write the input
    if ((uint16_t(address_offset) + RAM_START + size < max_address) and size > 0) {
        address_offset += RAM_START;
        write(input, ab08x5_reg_t(address_offset), size);
        success = true;
    }
    return success;
}

/**
 * Read data from the RTC's user memory.
 * Reads that are out of the memory's range will result in a failure and will not be read.
 *
 * @param input: Start address of the data to be read.
 * @param address_offset: Address in the RTC's user memory to start reading data from. [0 - 0x3F (or 0xBF in I2C mode)]
 * @param size: Number of bytes to read.
 * @return: True if the data was read.
 */
bool AB08x5::read_ram(uint8_t *output, uint8_t address_offset, uint8_t size) {
    uint8_t max_address = NORMAL_RAM_END;
    if (_comms_mode == AB08x5_I2C_MODE) max_address = I2C_MODE_RAM_END;

    bool success = false;

    // Ensure there is enough data to read the input
    if ((uint16_t(address_offset) + RAM_START + size < max_address) and size > 0) {
        address_offset += RAM_START;
        read(output, ab08x5_reg_t(address_offset), size);
        success = true;
    }
    return success;
}

///////////////////////////////////////////////////////////////////////////////

/**
 * Get the current time from the RTC registers.
 * @return: Current time as a DateTime object.
 */
DateTime AB08x5::now() {
    uint8_t buffer[8];
    DateTime now;

    read(buffer, AB08x5_REGISTER::HUNDREDTHS, 7);
    registers_to_datetime(now, buffer);
    return now;
}

/**
 * Set the time on the RTC.
 *
 * @param dt: DateTime to set the RTC's clock to.
 */
void AB08x5::adjust(DateTime &dt) {
    uint8_t buffer[8];
    datetime_to_registers(dt, buffer);

    unlock_time_registers();
    write(buffer, AB08x5_REGISTER::HUNDREDTHS, 8);
    lock_time_registers();
    _last_time_update = DateTime(dt);
}

/**
 * Set the time on the RTC to the compile time of the code.
 * Useful for first-time programming.
 *
 * @param dt: DateTime object to set the RTC's clock to.
 */
void AB08x5::adjust_to_compile_time() {
    DateTime compile_time = DateTime(F(__DATE__), F(__TIME__));
    adjust(compile_time);
}

/**
 * Get the timestamp of the last time the RTC was updated.
 * @return: DateTime object containing the last update time of the RTC.
 */
DateTime AB08x5::get_last_update_time() { return DateTime(_last_time_update); }

/**
 * Enable writing to the RTC's counter registers.
 * Used to set the time on the RTC.
 *
 * This function should only be used internally.
 */
void AB08x5::unlock_time_registers() {
    ab08x5_control_1_t config;
    read_config(config);
    config.time_registers_write_enabled = true;
    write_config(config);
}

/**
 * Disable writing to the RTC's counter registers.
 * Used to prevent changes to the set time on the RTC.
 *
 * This function should only be used internally.
 */
void AB08x5::lock_time_registers() {
    ab08x5_control_1_t config;
    read_config(config);
    config.time_registers_write_enabled = false;
    write_config(config);
}

///////////////////////////////////////////////////////////////////////////////

/**
 * Set the RTC's alarm registers to the given time.
 * The RTC will generate and interrupt when the current time matches the alarm if configured.
 * Alarm configuration is set in // TODO - find alarm references
 *
 * @param dt: DateTime object to set the alarm to.
 */
void AB08x5::set_alarm(DateTime &dt) {
    uint8_t buffer[8];
    datetime_to_registers(dt, buffer);
    write(buffer, AB08x5_REGISTER::HUNDREDTHS_ALARM, 8);
}

/**
 * Read the current alarm time from the RTC.
 * @return: Alarm date and time as a DateTime object.
 */
DateTime AB08x5::read_alarm(DateTime &dt) {
    uint8_t buffer[8];
    DateTime now;

    read(buffer, AB08x5_REGISTER::HUNDREDTHS_ALARM, 8);
    registers_to_datetime(now, buffer);
    return now;
}

/**
 * Enable alarm interrupts.
 * The repeat mode for the alarm can be set to trigger the alarm multiple times when various sections of the alarm match
 * the current time.
 *
 * e.g: A mode of AB08x5_ALARM_PER_MONTH will cause the alarm to trigger when the days, hours, minutes, seconds, and
 * hundredths match.
 *
 * @param alarm_repeat_mode: Repeat mode of the alarm (see AB08x5_ALARM_REPEAT_MODE)
 */
void AB08x5::enable_alarm(uint8_t alarm_repeat_mode) {
    ab08x5_alarm_control_t config;
    read((uint8_t *)&config, AB08x5_REGISTER::TIMER_CONTROL);
    config.alarm_repeat_mode = alarm_repeat_mode;
    write((uint8_t *)&config, AB08x5_REGISTER::TIMER_CONTROL);
}

/**
 * Disable alarm interrupts from occurring.
 */
void AB08x5::disable_alarm() {
    ab08x5_alarm_control_t config;
    read((uint8_t *)&config, AB08x5_REGISTER::TIMER_CONTROL);
    config.alarm_repeat_mode = AB08x5_ALARM_DISABLED;
    write((uint8_t *)&config, AB08x5_REGISTER::TIMER_CONTROL);
}

/**
 * Enable alarm interrupts on the specified output pin.
 * Enabling interrupts on a pin will overwrite the pin's previous configuration.
 *
 * @param output_pin: Pin to trigger alarm interrupts on. [nIRQ1 or nIRQ2]
 */
void AB08x5::enable_alarm_interrupts(uint8_t output_pin) {
    // Map the alarm interrupt to nIRQ1
    ab08x5_control_2_t config;
    read_config(config);
    if (output_pin == AB08x5_NIRQ) config.nirq_mode = AB08x5_NIRQ_MODE_NAIRQ;
    if (output_pin == AB08x5_NIRQ2) config.nirq2_mode = AB08x5_NIRQ_MODE_NAIRQ;
    write_config(config);

    // Set level interrupts for the alarm (low until cleared)
    ab08x5_interrupt_mask_t int_mask;
    int_mask.alarm_interrupt_output_mode = AB08x5_ALARM_INTERRUPT_MODE::AB08x5_INTERRUPT_LATCHED;
    int_mask.alarm_interrupt_enabled = true;
    write_config(int_mask);
}

///////////////////////////////////////////////////////////////////////////////

/**
 * Get an ID from one of the RTC's registers
 *
 * ID numbers are contained in the following structure:
 * ID0  Part Number (0x08)
 * ID1  Part Number (0x05 or 0x15)
 * ID2  Part revision
 * ID3  Lot number
 * ID4  Unique ID (LSB)
 * ID5  Unique ID (MSB)
 * ID6  Wafer ID
 *
 * @param id_number: ID register to get the data from [0-6].
 */
uint8_t AB08x5::get_id(uint8_t id_number) {
    id_number = constrain(id_number, 0, 6);
    ab08x5_reg_t reg_address = ab08x5_reg_t(ID0 + id_number);
    uint8_t output;
    read(&output, reg_address);
    return output;
}

///////////////////////////////////////////////////////////////////////////////

/**
 * Transform an array of read registers from the RTC into a DateTime object.
 * This function should only be used internally.
 *
 * @param dt: DateTime object to read the information into.
 * @param input: Starting address of the array containing the counter register information.
 */
void AB08x5::registers_to_datetime(DateTime &dt, uint8_t *input) {
    uint32_t micros = bcd_to_bin(input[0]) * 10000;
    uint8_t seconds = bcd_to_bin(input[1] & 0x7F);
    uint8_t minutes = bcd_to_bin(input[2] & 0x7F);
    uint8_t hours = bcd_to_bin(input[3] & 0x3F);
    uint8_t date = bcd_to_bin(input[4] & 0x3F);
    uint8_t month = bcd_to_bin(input[5] & 0x1F);
    uint8_t year = bcd_to_bin(input[6] & 0x1F);

    dt = DateTime(year, month, date, hours, minutes, seconds, micros);
}

/**
 * Transform a DateTime object into register format, ready to be written to the RTC.
 * This function should only be used internally.
 *
 * @param dt: DateTime object to read the information from.
 * @param input: Starting address of the array containing the counter register information to write to.
 */
void AB08x5::datetime_to_registers(DateTime &dt, uint8_t *output) {
    output[0] = bin_to_bcd(dt.hundredth());
    output[1] = bin_to_bcd(dt.second());
    output[2] = bin_to_bcd(dt.minute());
    output[3] = bin_to_bcd(dt.hour());
    output[4] = bin_to_bcd(dt.day());
    output[5] = bin_to_bcd(dt.month());
    output[6] = bin_to_bcd(dt.year() - AB08x5_YEAR_OFFSET);
    output[7] = bin_to_bcd(dt.day_of_the_week());
}

///////////////////////////////////////////////////////////////////////////////

/**
 * Write a configuration key to the RTC.
 * Configuration keys can enable writing to the oscillator control and other special configuration registers.
 * Writing the reset key can also reset the internal memory of the RTC.
 *
 * This function should only be used internally.
 */
void AB08x5::write_config_key(ab08x5_config_key_t key) { write((uint8_t *)&key, CONFIG_KEY); }

/**
 * Enable writing to the RTC's oscillator control register.
 *
 * This function should only be used internally.
 */
void AB08x5::unlock_oscillator_registers() { write_config_key(UNLOCK_OSC_CONTROL); }

/**
 * Disable writing to the RTC's oscillator control register.
 *
 * This function should only be used internally.
 */
void AB08x5::lock_oscillator_registers() { write_config_key(LOCK_SPECIAL_CONFIG); }
