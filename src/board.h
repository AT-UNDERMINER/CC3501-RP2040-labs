#pragma once

// ============================================================
//  Board: CC3501 RP2040 Lab Board v1.0
//
//  All hardware-specific constants live here.
//  When targeting a different board revision change only this
//  file — nothing else in the codebase needs to know about the
//  physical layout of the hardware.
// ============================================================

// --- Addressable LEDs (WS2812) ----------------------------------------------

#define BOARD_LED_PIN      14       // GPIO pin connected to the LED data line
#define BOARD_LED_COUNT    12       // Number of LEDs physically fitted on the board
#define BOARD_LED_FREQ     800000   // WS2812 data-line frequency in Hz
#define BOARD_LED_IS_RGBW  false    // false = RGB strip, true = RGBW strip
