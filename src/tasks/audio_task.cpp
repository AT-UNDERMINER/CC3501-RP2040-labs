#include "audio_task.h"
#include "drivers/microphone/microphone.h"
#include "drivers/leds/leds.h"
#include "board.h"

#include <stdint.h>
#include <math.h>          // cosf, M_PI — used only in one-time window pre-compute
#include "arm_math.h"      // CMSIS-DSP: q15_t, arm_rfft_q15, arm_cmplx_mag_squared_q15
#include "hardware/adc.h"  // adc_run / FIFO drain in exit_audio_task

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// --- Constants ---------------------------------------------------------------
static constexpr int FFT_SIZE    = 1024;
static constexpr int SAMPLE_RATE = 44100;
static constexpr int NUM_LEDS    = 12;

// Magnitude-squared sum above which an LED's band is considered "active".
// Values here are in ~Q3.13 fixed point; expect to tune this on hardware.
static constexpr int32_t LEVEL_THRESHOLD = 10;

// --- Working buffers (file scope to keep them off the small main stack) -------
static int16_t fft_output[FFT_SIZE * 2];   // arm_rfft_q15 output (2*FFT_SIZE)
static q15_t   mag_squared[FFT_SIZE / 2];  // |FFT|^2 per bin
static q15_t   hanning[FFT_SIZE];          // pre-computed Hanning window

static uint16_t sample_buffer[FFT_SIZE];   // raw 12-bit ADC samples
static q15_t    time_domain[FFT_SIZE];     // centred / scaled / windowed input

// Log-spaced FFT bin boundaries: 13 edges define 12 LED bands.
static const int bin_edges[NUM_LEDS + 1] = {
    6, 8, 11, 16, 24, 35, 51, 75, 110, 161, 237, 349, 513
};

static arm_rfft_instance_q15 rfft;

// Tracks whether the FFT/mic pipeline has been initialised this session.
static bool s_initialised = false;

void run_audio_task(LedDriver &leds)
{
    // One-time setup; exit resets the flag so re-entry re-initialises cleanly.
    if (!s_initialised) {
        leds.set_busy_wait(false);
        microphone_init();
        arm_rfft_init_q15(&rfft, FFT_SIZE, 0, 1); // forward, bit-reversed output

        // Pre-compute the Hanning window. Float is fine here: runs once at init,
        // never in the per-frame processing loop.
        for (int n = 0; n < FFT_SIZE; n++) {
            hanning[n] = (int16_t)(0.5f * (1.0f - cosf(2.0f * M_PI * n / (FFT_SIZE - 1))) * 32767);
        }

        s_initialised = true;
    }

    // 1. Capture a block of samples (blocking by design).
    microphone_read(sample_buffer, FFT_SIZE);

    // 2. DC bias = mean of the block.
    int32_t sum = 0;
    for (int i = 0; i < FFT_SIZE; i++) {
        sum += sample_buffer[i];
    }
    int32_t dc_bias = sum / FFT_SIZE;

    // 3. Centre each sample on the DC bias and shift up by 5 to use more of the
    //    Q15 range (12-bit ADC data only fills the low bits otherwise).
    for (int i = 0; i < FFT_SIZE; i++) {
        int32_t centred = (int32_t)sample_buffer[i] - dc_bias;
        time_domain[i] = (q15_t)(centred << 5);
    }

    // 4. Apply the Hanning window in Q15 fixed point (Q15 * Q15 >> 15 -> Q15).
    for (int i = 0; i < FFT_SIZE; i++) {
        time_domain[i] = (q15_t)(((int32_t)time_domain[i] * (int32_t)hanning[i]) >> 15);
    }

    // 5. Forward real FFT. CMSIS-DSP rescales internally (output ~Q11.5);
    //    do not renormalise or bit-shift the result.
    arm_rfft_q15(&rfft, time_domain, fft_output);

    // 6. Magnitude squared straight from the FFT output, no shifting either
    //    side. Result is ~Q3.13 over FFT_SIZE/2 bins.
    arm_cmplx_mag_squared_q15(fft_output, mag_squared, FFT_SIZE / 2);

    // 7. Sum each log-spaced band and light its LED if it crosses the threshold.
    for (int led = 0; led < NUM_LEDS; led++) {
        int start = bin_edges[led];
        int end   = bin_edges[led + 1];
        if (end > FFT_SIZE / 2) end = FFT_SIZE / 2; // mag_squared holds FFT_SIZE/2 bins

        int32_t band = 0;
        for (int b = start; b < end; b++) {
            band += mag_squared[b];
        }

        if (band > LEVEL_THRESHOLD) {
            // Frequency colour: LED 0 red, LED 11 blue, interpolating through
            // green between them (two 0..255 ramps, all integer arithmetic).
            int pos = led * 510 / (NUM_LEDS - 1); // 0..510
            uint8_t r, g, b8;
            if (pos <= 255) {
                r  = (uint8_t)(255 - pos);
                g  = (uint8_t)pos;
                b8 = 0;
            } else {
                r  = 0;
                g  = (uint8_t)(510 - pos);
                b8 = (uint8_t)(pos - 255);
            }
            leds.set_one(led, r, g, b8);
        } else {
            leds.set_one(led, 0, 0, 0);
        }
    }
    leds.show();
}

void exit_audio_task(LedDriver &leds)
{
    // Stop sampling and clear anything left in the FIFO (is_empty() checked
    // first so the drain never blocks waiting for a sample that won't come).
    adc_run(false);
    while (!adc_fifo_is_empty()) {
        adc_fifo_get_blocking();
    }

    // Turn off every LED.
    leds.off();
    leds.show();

    // Force re-initialisation next time the task is selected.
    s_initialised = false;
}
