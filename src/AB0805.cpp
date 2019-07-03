#include "AB0805.h"

bool AB08x5::begin(uint8_t comms_mode, uint8_t address_or_pin) {
    _comms_mode = comms_mode;
    if (comms_mode == AB08x5_I2C_MODE) _device_address = address_or_pin;
    if (comms_mode == AB08x5_SPI_MODE) _chip_select_pin = address_or_pin;
}

bool AB08x5::write(uint8_t *input, ab08x5_reg_t address, uint8_t length) {
    bool success = false;
    if (_comms_mode == AB08x5_I2C_MODE) success = write_i2c(input, address, length);
    if (_comms_mode == AB08x5_SPI_MODE) success = write_spi(input, address, length);
    return success;
}

bool AB08x5::read(uint8_t *output, ab08x5_reg_t address, uint8_t length) {
    bool success = false;
    if (_comms_mode == AB08x5_I2C_MODE) success = read_i2c(output, address, length);
    if (_comms_mode == AB08x5_SPI_MODE) success = read_spi(output, address, length);
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
 * Read a specified number of bytes using the I2C bus.
 * @param output: The buffer in which to store the read values.
 * @param address: Register address to read (or starting address in burst reads)
 * @param length: Number of bytes to read.
 */
bool AB08x5::read_i2c(uint8_t *output, ab08x5_reg_t address, uint8_t length) {
    bool result = true;
    Wire.beginTransmission(address);
    Wire.write(address);
    if (Wire.endTransmission() != 0)
        result = false;

    else  // OK, all worked, keep going
    {
        Wire.requestFrom(address, length);
        for (size_t i = 0; (i < length) and Wire.available(); i++) {
            uint8_t c = Wire.read();
            output[i] = c;
        }
    }
    return result;
}

bool AB08x5::write_spi(uint8_t *input, ab08x5_reg_t address, uint8_t length) {
    digitalWrite(_chip_select_pin, LOW);
    SPI.transfer(address);
    for (size_t i = 0; i < length; i++) {
        SPI.transfer(input[i]);
    }
    digitalWrite(_chip_select_pin, HIGH);
    return true;
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

bool AB08x5::comm_check() {
    bool success = false;
    uint8_t family_id;
    read(&family_id, AB08x5_REGISTER::ID0);
    if (family_id == 0x08) success = true;
    return success;
}

ab08x5_status_t AB08x5::get_status() {
    ab08x5_status_t status;
    read((uint8_t *)&status, AB08x5_REGISTER::STATUS);
    return status;
}

ab08x5_osc_status_t AB08x5::get_oscillator_status() {
    ab08x5_osc_status_t status;
    read((uint8_t *)&status, AB08x5_REGISTER::OSC_STATUS);
    return status;
}

void AB08x5::set_control_config(ab08x5_control_1_t config) { write((uint8_t *)&config, AB08x5_REGISTER::CONTROL_1); }

void AB08x5::set_control_config(ab08x5_control_2_t config) { write((uint8_t *)&config, AB08x5_REGISTER::CONTROL_2); }

void AB08x5::set_interrupt_mask(ab08x5_interrupt_mask_t config) {
    write((uint8_t *)&config, AB08x5_REGISTER::INT_MASK);
}
void AB08x5::set_sqw_config(ab08x5_sqw_config_t config) { write((uint8_t *)&config, AB08x5_REGISTER::SQW); }

void AB08x5::set_watchdog_config(ab08x5_watchdog_config_t config) { write((uint8_t *)&config, AB08x5_REGISTER::WDT); }

void AB08x5::set_alarm_config(ab08x5_alarm_control_t config) {
    write((uint8_t *)&config, AB08x5_REGISTER::TIMER_CONTROL);
}

ab08x5_control_1_t AB08x5::get_control_config_1() {
    ab08x5_control_1_t config;
    read((uint8_t *)&config, AB08x5_REGISTER::CONTROL_1);
    return config;
}

ab08x5_control_2_t AB08x5::get_control_config_2() {
    ab08x5_control_2_t config;
    read((uint8_t *)&config, AB08x5_REGISTER::CONTROL_2);
    return config;
}

ab08x5_interrupt_mask_t AB08x5::get_interrupt_mask() {
    ab08x5_interrupt_mask_t config;
    read((uint8_t *)&config, AB08x5_REGISTER::INT_MASK);
    return config;
}

DateTime AB08x5::now() {
    uint8_t buffer[7];
    DateTime now;

    read(buffer, AB08x5_REGISTER::HUNDREDTHS, 7);
    registers_to_datetime(now, buffer);
    return now;
}

void AB08x5::adjust(DateTime &dt) {
    uint8_t buffer[8];
    datetime_to_registers(dt, buffer);
    unlock_time_registers();
    write(buffer, AB08x5_REGISTER::HUNDREDTHS, 8);
    lock_time_registers();
    _last_time_update = DateTime(dt);
}

void AB08x5::adjust_to_compile_time() {
    uint8_t buffer[8];
    DateTime compile_time = DateTime(F(__DATE__), F(__TIME__));
    unlock_time_registers();
    write(buffer, AB08x5_REGISTER::HUNDREDTHS), 8;
    lock_time_registers();
    _last_time_update = DateTime(compile_time);
}

void AB08x5::set_alarm(DateTime &dt) {
    uint8_t buffer[8];
    datetime_to_registers(dt, buffer);
    write(buffer, AB08x5_REGISTER::HUNDREDTHS_ALARM, 8);
}

DateTime AB08x5::read_alarm(DateTime &dt) {
    uint8_t buffer[7];
    DateTime now;

    read(buffer, AB08x5_REGISTER::HUNDREDTHS_ALARM, 7);
    registers_to_datetime(now, buffer);
    return now;
}

void AB08x5::enable_alarm(uint8_t alarm_repeat_mode) {
    ab08x5_alarm_control_t config;
    read((uint8_t *)&config, AB08x5_REGISTER::TIMER_CONTROL);
    config.alarm_repeat_mode = alarm_repeat_mode;
    write((uint8_t *)&config, AB08x5_REGISTER::TIMER_CONTROL);
}

void AB08x5::disable_alarm() {
    ab08x5_alarm_control_t config;
    read((uint8_t *)&config, AB08x5_REGISTER::TIMER_CONTROL);
    config.alarm_repeat_mode = AB08x5_ALARM_DISABLED;
    write((uint8_t *)&config, AB08x5_REGISTER::TIMER_CONTROL);
}

void AB08x5::datetime_to_registers(DateTime &dt, uint8_t *output) {
    uint32_t micros = bcd_to_bin(output[0]) * 10000;
    uint8_t seconds = bcd_to_bin(output[1] & 0x7F);
    uint8_t minutes = bcd_to_bin(output[2] & 0x7F);
    uint8_t hours = bcd_to_bin(output[3] & 0x3F);
    uint8_t date = bcd_to_bin(output[4] & 0x3F);
    uint8_t month = bcd_to_bin(output[5] & 0x1F);
    uint8_t year = bcd_to_bin(output[6] & 0x1F);

    dt = DateTime(year, month, date, hours, minutes, seconds, micros);
}

void AB08x5::registers_to_datetime(DateTime &dt, uint8_t *input) {
    input[0] = bin_to_bcd(dt.hundredth());
    input[1] = bin_to_bcd(dt.second());
    input[2] = bin_to_bcd(dt.minute());
    input[3] = bin_to_bcd(dt.hour());
    input[4] = bin_to_bcd(dt.day());
    input[5] = bin_to_bcd(dt.month());
    input[6] = bin_to_bcd(dt.year());
    input[7] = bin_to_bcd(dt.day_of_the_week());
}

DateTime AB08x5::get_last_update_time() { return DateTime(_last_time_update); }

void AB08x5::unlock_time_registers() {
    ab08x5_control_1_t config = get_control_config_1();
    config.time_registers_write_enabled = 1;
    set_control_config(config);
}

void AB08x5::lock_time_registers() {
    ab08x5_control_1_t config = get_control_config_1();
    config.time_registers_write_enabled = 0;
    set_control_config(config);
}