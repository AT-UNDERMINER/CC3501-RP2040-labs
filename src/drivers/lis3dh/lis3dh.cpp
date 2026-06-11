#include "lis3dh.h"

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

// --- Pin / bus configuration -------------------------------------------------
static constexpr uint  LIS3DH_SDA_PIN = 16;
static constexpr uint  LIS3DH_SCL_PIN = 17;
static constexpr uint  LIS3DH_BAUD    = 400000; // 400 kHz fast mode

// --- Register map (subset used here) -----------------------------------------
static constexpr uint8_t REG_WHO_AM_I  = 0x0F;
static constexpr uint8_t REG_CTRL_REG1  = 0x20;
static constexpr uint8_t REG_CTRL_REG4  = 0x23;
static constexpr uint8_t REG_OUT_X_L    = 0x28;

static constexpr uint8_t WHO_AM_I_VALUE = 0x33; // fixed device id (datasheet Table 24)
static constexpr uint8_t AUTO_INCREMENT = 0x80; // OR into sub-address for multi-byte access

// Write a single register value to the device.
static void write_reg(i2c_inst_t *i2c, uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };
    i2c_write_blocking(i2c, LIS3DH_ADDR, buf, 2, false);
}

// Read a single register value from the device.
static uint8_t read_reg(i2c_inst_t *i2c, uint8_t reg)
{
    uint8_t value = 0;
    i2c_write_blocking(i2c, LIS3DH_ADDR, &reg, 1, true); // keep bus held for repeated start
    i2c_read_blocking(i2c, LIS3DH_ADDR, &value, 1, false);
    return value;
}

bool lis3dh_init(i2c_inst_t *i2c)
{
    // Bring up the I2C peripheral and route the pins. External pull-ups are
    // already fitted, so internal pulls are deliberately left disabled.
    i2c_init(i2c, LIS3DH_BAUD);
    gpio_set_function(LIS3DH_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(LIS3DH_SCL_PIN, GPIO_FUNC_I2C);

    // Confirm we are talking to a LIS3DH before configuring anything.
    uint8_t who = read_reg(i2c, REG_WHO_AM_I);
    if (who != WHO_AM_I_VALUE) {
        printf("LIS3DH init failed: WHO_AM_I returned 0x%02X, expected 0x%02X\n",
               who, WHO_AM_I_VALUE);
        return false;
    }

    // CTRL_REG1 = 0x57: ODR=100 Hz, normal mode (LPen=0), X/Y/Z enabled.
    write_reg(i2c, REG_CTRL_REG1, 0x57);
    // CTRL_REG4 = 0x88: BDU=1, HR=1 (12-bit), FS=00 (+-2g).
    write_reg(i2c, REG_CTRL_REG4, 0x88);

    return true;
}

void lis3dh_read_raw(i2c_inst_t *i2c, int16_t *x, int16_t *y, int16_t *z)
{
    // Auto-increment must be requested for a multi-byte burst read.
    uint8_t reg = REG_OUT_X_L | AUTO_INCREMENT; // 0xA8
    uint8_t data[6] = { 0 };

    i2c_write_blocking(i2c, LIS3DH_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c, LIS3DH_ADDR, data, 6, false);

    // Data is left-justified in HR mode; combine then shift down to 12-bit signed.
    *x = (int16_t)((data[1] << 8) | data[0]) >> 4;
    *y = (int16_t)((data[3] << 8) | data[2]) >> 4;
    *z = (int16_t)((data[5] << 8) | data[4]) >> 4;
}

float lis3dh_to_g(int16_t raw)
{
    // 1 mg/digit (datasheet Table 4) -> multiply by 0.001 g.
    return (float)raw * 0.001f;
}
