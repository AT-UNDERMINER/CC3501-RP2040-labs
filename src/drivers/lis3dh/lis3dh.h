#pragma once

#include <stdint.h>
#include "hardware/i2c.h"

// Minimal driver for the ST LIS3DH 3-axis accelerometer over I2C.
//
// Wiring on this board:
//   SDA = GPIO16, SCL = GPIO17, I2C bus = i2c0, 400 kHz (fast mode)
//   CS pulled high externally  -> I2C mode selected
//   SA0/SDO pulled up          -> 7-bit address 0x19
//   SDA/SCL have external 4.7 kOhm pull-ups, so no internal pulls are enabled.

// 7-bit I2C address (SA0 high).
static constexpr uint8_t LIS3DH_ADDR = 0x19;

// Initialise the bus and the device. Verifies WHO_AM_I (0x33) and configures
// 100 Hz / high-resolution / +-2g. Returns false if the device is not found.
bool lis3dh_init(i2c_inst_t *i2c);

// Read one sample from all three axes as 12-bit signed values (right-justified).
void lis3dh_read_raw(i2c_inst_t *i2c, int16_t *x, int16_t *y, int16_t *z);

// Convert a raw 12-bit reading to g-force (1 mg/digit in HR mode at +-2g).
float lis3dh_to_g(int16_t raw);
