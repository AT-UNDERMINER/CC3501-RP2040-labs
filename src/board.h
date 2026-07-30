#pragma once

// --- SW1 (Button Configuration)
#define SW1_PIN 15                 // tactile button; external RC filter + pull-down

// --- Addressable LEDs (WS2812)
#define BOARD_LED_PIN      14       // GPIO pin connected to the LED data line
#define BOARD_LED_COUNT    12       // Number of LEDs physically fitted on the board
#define BOARD_LED_IS_RGBW  false    // false = RGB strip, true = RGBW strip

// --- Accelerometer (LIS3DH)
#define ACCEL_I2C_INSTANCE  i2c0     // I2C controller the LIS3DH is on
#define ACCEL_SDA_PIN       16       // I2C SDA
#define ACCEL_SCL_PIN       17       // I2C SCL

// --- Microphone (ADC)
#define MIC_ADC_PIN         26       // microphone amplifier output -> ADC input
#define MIC_ADC_CHANNEL     0        // ADC channel for GPIO26
