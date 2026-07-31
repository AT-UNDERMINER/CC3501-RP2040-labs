#include "lis3dh.h"
#include "board.h"
#include "drivers/logging/logging.h"

#include <stdio.h> // snprintf, for the one message that carries values
#include "pico/stdlib.h"
#include "hardware/gpio.h"

// Pins live in board.h (wiring); the baud rate is a bus fact, so it stays here.
static constexpr uint  LIS3DH_BAUD = 400000; // 400 kHz fast mode

// Registers used by this driver.
static constexpr uint8_t REG_WHO_AM_I  = 0x0F;
static constexpr uint8_t REG_CTRL_REG1  = 0x20;
static constexpr uint8_t REG_CTRL_REG4  = 0x23;
static constexpr uint8_t REG_OUT_X_L    = 0x28;

static constexpr uint8_t WHO_AM_I_VALUE = 0x33; // fixed device id (datasheet Table 24)
static constexpr uint8_t AUTO_INCREMENT = 0x80; // OR into sub-address for multi-byte access

// i2c_*_blocking returns the byte count on success or a negative error code,
// so every helper compares against the expected count.

// Write a single register value. True if the full transfer went through.
static bool write_reg(i2c_inst_t *i2c, uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };
    if (i2c_write_blocking(i2c, LIS3DH_ADDR, buf, 2, false) != 2) {
        log(LogLevel::ERROR, "lis3dh: failed to write register.");
        return false;
    }
    return true;
}

// Read `length` consecutive registers into `data`. Serves both single-register
// reads and the 3-axis burst, so the transaction only exists in one place.
static bool read_registers(i2c_inst_t *i2c, uint8_t reg, uint8_t *data, size_t length)
{
    // Hold the bus for a repeated start between the address write and the read.
    if (i2c_write_blocking(i2c, LIS3DH_ADDR, &reg, 1, true) != 1) {
        log(LogLevel::ERROR, "lis3dh: failed to select register address.");
        return false;
    }
    if (i2c_read_blocking(i2c, LIS3DH_ADDR, data, length, false) != (int)length) {
        log(LogLevel::ERROR, "lis3dh: failed to read register data.");
        return false;
    }
    return true;
}

bool lis3dh_init(i2c_inst_t *i2c, lis3dh_odr_t start_odr)
{
    // Bring up the I2C peripheral and route the pins. External pull-ups are
    // already fitted, so internal pulls are deliberately left disabled.
    i2c_init(i2c, LIS3DH_BAUD);
    gpio_set_function(ACCEL_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(ACCEL_SCL_PIN, GPIO_FUNC_I2C);

    // Confirm we are talking to a LIS3DH before configuring anything.
    uint8_t who = 0;
    if (!read_registers(i2c, REG_WHO_AM_I, &who, 1)) {
        return false; // read_registers has already logged why
    }
    if (who != WHO_AM_I_VALUE) {
        // Format the values in: knowing what came back separates a wrong device
        // from a silent bus. log() only takes a plain string, hence snprintf.
        char msg[64];
        snprintf(msg, sizeof(msg), "lis3dh: WHO_AM_I read 0x%02X, expected 0x%02X",
                 who, WHO_AM_I_VALUE);
        log(LogLevel::ERROR, msg);
        return false;
    }

    // CTRL_REG1 lower nibble = 0x07 (LPen=0 normal mode, Z/Y/X enabled); the ODR
    // bits go through lis3dh_set_odr() so only one code path writes them.
    if (!write_reg(i2c, REG_CTRL_REG1, 0x07)) return false;
    if (!lis3dh_set_odr(i2c, start_odr))     return false;
    // CTRL_REG4 = 0x88: BDU=1, HR=1 (12-bit), FS=00 (+-2g).
    if (!write_reg(i2c, REG_CTRL_REG4, 0x88)) return false;

    return true;
}

bool lis3dh_set_odr(i2c_inst_t *i2c, lis3dh_odr_t odr)
{
    // Read-modify-write: replace only ODR[3:0] (the upper nibble of CTRL_REG1),
    // keep LPen/Zen/Yen/Xen as they are.
    uint8_t current;
    if (!read_registers(i2c, REG_CTRL_REG1, &current, 1)) return false;
    return write_reg(i2c, REG_CTRL_REG1, (uint8_t)((current & 0x0F) | (uint8_t)odr));
}

bool lis3dh_power_down(i2c_inst_t *i2c)
{
    // ODR = 0000 is power-down mode (datasheet Table 25).
    return lis3dh_set_odr(i2c, lis3dh_odr_t::POWERDOWN);
}

bool lis3dh_read_raw(i2c_inst_t *i2c, int16_t *x, int16_t *y, int16_t *z)
{
    // Auto-increment must be requested for a multi-byte burst read (0xA8).
    uint8_t data[6] = { 0 };
    if (!read_registers(i2c, REG_OUT_X_L | AUTO_INCREMENT, data, 6)) {
        return false;
    }

    // Data is left-justified in HR mode; combine then shift down to 12-bit signed.
    *x = (int16_t)((data[1] << 8) | data[0]) >> 4;
    *y = (int16_t)((data[3] << 8) | data[2]) >> 4;
    *z = (int16_t)((data[5] << 8) | data[4]) >> 4;
    return true;
}

float lis3dh_to_g(int16_t raw)
{
    // 1 mg/digit (datasheet Table 4) -> multiply by 0.001 g.
    return (float)raw * 0.001f;
}
