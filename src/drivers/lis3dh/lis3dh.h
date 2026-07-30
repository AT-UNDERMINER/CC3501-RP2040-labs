#pragma once

#include <stdint.h>
#include "hardware/i2c.h"

// Minimal driver for the ST LIS3DH 3-axis accelerometer over I2C.
//
// Wiring on this board:
//   Pins/bus in board.h (ACCEL_SDA_PIN, ACCEL_SCL_PIN, ACCEL_I2C_INSTANCE); 400 kHz fast mode
//   CS pulled high externally  -> I2C mode selected
//   SA0/SDO pulled up          -> 7-bit address 0x19
//   SDA/SCL have external 4.7 kOhm pull-ups, so no internal pulls are enabled.

// 7-bit I2C address (SA0 high).
static constexpr uint8_t LIS3DH_ADDR = 0x19;

// Output data rate values for CTRL_REG1 ODR[3:0], pre-shifted into the upper
// nibble (datasheet Table 25). POWERDOWN is ODR = 0000, the chip's power-down
// state rather than a sample rate.
// enum class is a scoped enum: values need the lis3dh_odr_t:: prefix and won't
// convert to int, so a raw number can't be passed by mistake.
enum class lis3dh_odr_t : uint8_t {
    POWERDOWN = 0x00,
    ODR_1HZ   = 0x10,
    ODR_10HZ  = 0x20,
    ODR_25HZ  = 0x30,
    ODR_50HZ  = 0x40,
    ODR_100HZ = 0x50,
    ODR_200HZ = 0x60,
    ODR_400HZ = 0x70,
};

// Initialise the bus and the device. Verifies WHO_AM_I (0x33), enables X/Y/Z
// in normal mode at start_odr, and configures high-resolution +-2g.
// Returns false if the device is not found.
bool lis3dh_init(i2c_inst_t *i2c, lis3dh_odr_t start_odr);

// Change the ODR without touching the other CTRL_REG1 bits (read-modify-write).
// Returns false if the I2C transaction fails.
bool lis3dh_set_odr(i2c_inst_t *i2c, lis3dh_odr_t odr);

// Put the device into power-down mode. Wake it again with lis3dh_set_odr().
bool lis3dh_power_down(i2c_inst_t *i2c);

// Read one sample from all three axes as 12-bit signed values (right-justified).
// Returns false (outputs untouched) if the I2C transaction fails.
bool lis3dh_read_raw(i2c_inst_t *i2c, int16_t *x, int16_t *y, int16_t *z);

// Convert a raw 12-bit reading to g-force (1 mg/digit in HR mode at +-2g).
float lis3dh_to_g(int16_t raw);
