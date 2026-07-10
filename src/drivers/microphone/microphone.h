#pragma once

#include <stdint.h>

// Minimal driver for a microphone whose amplified analogue output is wired to
// the RP2040 ADC on GPIO26 (ADC channel 0). Samples are 12-bit (0-4095) and
// carry a DC bias of roughly mid-scale (~2048) from the amplifier.

// Sample rate the driver configures the ADC for. Single source of truth —
// consumers (e.g. FFT bin maths) should read this rather than re-typing 44100.
static constexpr int MICROPHONE_SAMPLE_RATE_HZ = 44100;

// Configure the ADC, GPIO26 and the sample FIFO for MICROPHONE_SAMPLE_RATE_HZ.
// Does not start sampling — free-running mode is controlled by microphone_read().
void microphone_init();

// Collect exactly num_samples 12-bit samples into buffer. Starts free-running
// mode, blocks until the requested number of samples has been read, then stops
// sampling and drains any remaining samples from the FIFO.
void microphone_read(uint16_t *buffer, uint16_t num_samples);

// Stop free-running sampling and drain anything left in the FIFO so the next
// capture starts clean. Safe to call whether or not sampling is running.
void microphone_stop();
