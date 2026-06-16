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

// Output data rate (ODR) values for CTRL_REG1 ODR[3:0], pre-shifted into the
// upper nibble (datasheet Table 25).
#define LIS3DH_ODR_1HZ      0x10
#define LIS3DH_ODR_10HZ     0x20
#define LIS3DH_ODR_25HZ     0x30
#define LIS3DH_ODR_50HZ     0x40
#define LIS3DH_ODR_100HZ    0x50
#define LIS3DH_ODR_200HZ    0x60
#define LIS3DH_ODR_400HZ    0x70

// Select the desired output data rate — uncomment one line only
// #define LIS3DH_ODR_SELECT    LIS3DH_ODR_1HZ
// #define LIS3DH_ODR_SELECT    LIS3DH_ODR_10HZ
// #define LIS3DH_ODR_SELECT    LIS3DH_ODR_25HZ
// #define LIS3DH_ODR_SELECT    LIS3DH_ODR_50HZ
#define LIS3DH_ODR_SELECT       LIS3DH_ODR_100HZ
// #define LIS3DH_ODR_SELECT    LIS3DH_ODR_200HZ
// #define LIS3DH_ODR_SELECT    LIS3DH_ODR_400HZ

// Initialise the bus and the device. Verifies WHO_AM_I (0x33) and configures
// the selected ODR (LIS3DH_ODR_SELECT, default 100 Hz), high-resolution, +-2g.
// Returns false if the device is not found.
bool lis3dh_init(i2c_inst_t *i2c);

// Read one sample from all three axes as 12-bit signed values (right-justified).
void lis3dh_read_raw(i2c_inst_t *i2c, int16_t *x, int16_t *y, int16_t *z);

// Convert a raw 12-bit reading to g-force (1 mg/digit in HR mode at +-2g).
float lis3dh_to_g(int16_t raw);
