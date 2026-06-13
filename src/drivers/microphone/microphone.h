#pragma once

#include <stdint.h>

// Minimal driver for a microphone whose amplified analogue output is wired to
// the RP2040 ADC on GPIO26 (ADC channel 0). Samples are 12-bit (0-4095) and
// carry a DC bias of roughly mid-scale (~2048) from the amplifier.

// Configure the ADC, GPIO26 and the sample FIFO for a 44.1 kHz sample rate.
// Does not start sampling — free-running mode is controlled by microphone_read().
void microphone_init();

// Collect exactly num_samples 12-bit samples into buffer. Starts free-running
// mode, blocks until the requested number of samples has been read, then stops
// sampling and drains any remaining samples from the FIFO.
void microphone_read(uint16_t *buffer, uint16_t num_samples);
