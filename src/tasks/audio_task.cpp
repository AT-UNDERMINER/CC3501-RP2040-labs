#include "audio_task.h"
#include "drivers/microphone/microphone.h"
#include "drivers/leds/leds.h"
#include "board.h"

#include <stdint.h>
#include <stdio.h>
#include "arm_math.h" // CMSIS-DSP: q15_t, arm_rfft_q15, arm_cmplx_mag_squared_q15, __SSAT

static constexpr int FFT_SIZE = 1024;

// A real FFT of N points has N/2 + 1 unique bins (DC through Nyquist); the rest
// of the output is the conjugate mirror and carries no new information.
static constexpr int NUM_BINS = FFT_SIZE / 2 + 1;

// How far to left-shift the 12-bit samples when packing them into Q15. The lab
// sheet suggests 5, but fixed point wastes precision on small numbers, so this
// pushes the useful bits further left. Anything above INPUT_CLIP_LEVEL is
// saturated instead of wrapping, so a loud noise clips rather than inverting.
static constexpr int INPUT_SHIFT      = 6;
static constexpr int INPUT_CLIP_LEVEL = 32767 >> INPUT_SHIFT; // largest sample swing that still fits

// Band energy above which an LED lights up. The FFT chain leaves the result in
// roughly Q3.13, so this is not a physical unit — tune it against the "max band"
// figure printed by the diagnostics below.
static constexpr int32_t LEVEL_THRESHOLD = 10;

// Prints the sample swing and strongest band once a second so INPUT_SHIFT and
// LEVEL_THRESHOLD can be set from measurements rather than guesswork. Set false
// once they are dialled in.
static constexpr bool AUDIO_DIAGNOSTICS = false;
static constexpr int  DIAG_FRAMES       = 25; // one frame is ~40 ms, so roughly 1 s

// Working buffers, kept at file scope so they stay off the small main stack.
// arm_rfft_q15 writes the conjugate mirror as well as the unique bins, so the
// output needs the full 2*FFT_SIZE even though only NUM_BINS are read back.
static int16_t  fft_output[FFT_SIZE * 2];
static q15_t    mag_squared[NUM_BINS];     // energy per bin
static uint16_t sample_buffer[FFT_SIZE];   // raw 12-bit ADC samples
static q15_t    time_domain[FFT_SIZE];     // centred, scaled and windowed input

// Log-spaced bin boundaries: 13 edges define 12 bands, one per LED. Human pitch
// perception is logarithmic, hence the spacing. Generated in Matlab with
// ceil(logspace(log10(5), log10(512), 13)). Each bin is ~43 Hz at 44.1 kHz.
static const int bin_edges[BOARD_LED_COUNT + 1] = {
    6, 8, 11, 16, 24, 35, 51, 75, 110, 161, 237, 349, 513
};

// Hanning window in Q15, from Matlab: int16(hann(1024, 'periodic') .* 2^15).
// The lecture notes specify the 'periodic' window; the lab sheet omits it and so
// gets Matlab's symmetric default. Periodic is the right one for repeated FFT
// analysis, so that is what is used here.
static const q15_t hanning[FFT_SIZE] = {
         0,      0,      1,      3,      5,      8,     11,     15,     20,     25,     31,     37,     44,     52,     60,     69,
        79,     89,    100,    111,    123,    136,    149,    163,    177,    192,    208,    224,    241,    259,    277,    296,
       315,    335,    355,    376,    398,    420,    443,    467,    491,    516,    541,    567,    593,    621,    648,    677,
       705,    735,    765,    796,    827,    859,    891,    924,    958,    992,   1027,   1062,   1098,   1134,   1171,   1209,
      1247,   1286,   1325,   1365,   1406,   1447,   1488,   1530,   1573,   1616,   1660,   1704,   1749,   1795,   1841,   1887,
      1935,   1982,   2030,   2079,   2128,   2178,   2229,   2280,   2331,   2383,   2435,   2488,   2542,   2596,   2651,   2706,
      2761,   2817,   2874,   2931,   2989,   3047,   3105,   3165,   3224,   3284,   3345,   3406,   3468,   3530,   3592,   3655,
      3719,   3783,   3847,   3912,   3978,   4044,   4110,   4177,   4244,   4312,   4380,   4449,   4518,   4587,   4657,   4728,
      4799,   4870,   4942,   5014,   5087,   5160,   5233,   5307,   5381,   5456,   5531,   5606,   5682,   5759,   5835,   5913,
      5990,   6068,   6146,   6225,   6304,   6383,   6463,   6543,   6624,   6705,   6786,   6868,   6950,   7032,   7115,   7198,
      7282,   7365,   7449,   7534,   7619,   7704,   7789,   7875,   7961,   8047,   8134,   8221,   8308,   8396,   8484,   8572,
      8661,   8749,   8839,   8928,   9018,   9108,   9198,   9288,   9379,   9470,   9561,   9653,   9745,   9837,   9929,  10021,
     10114,  10207,  10300,  10394,  10487,  10581,  10676,  10770,  10864,  10959,  11054,  11149,  11245,  11340,  11436,  11532,
     11628,  11724,  11821,  11917,  12014,  12111,  12208,  12306,  12403,  12501,  12598,  12696,  12794,  12892,  12991,  13089,
     13188,  13286,  13385,  13484,  13583,  13682,  13781,  13881,  13980,  14079,  14179,  14279,  14378,  14478,  14578,  14678,
     14778,  14878,  14978,  15078,  15179,  15279,  15379,  15480,  15580,  15680,  15781,  15881,  15982,  16082,  16183,  16283,
     16384,  16485,  16585,  16686,  16786,  16887,  16987,  17088,  17188,  17288,  17389,  17489,  17589,  17690,  17790,  17890,
     17990,  18090,  18190,  18290,  18390,  18489,  18589,  18689,  18788,  18887,  18987,  19086,  19185,  19284,  19383,  19482,
     19580,  19679,  19777,  19876,  19974,  20072,  20170,  20267,  20365,  20462,  20560,  20657,  20754,  20851,  20947,  21044,
     21140,  21236,  21332,  21428,  21523,  21619,  21714,  21809,  21904,  21998,  22092,  22187,  22281,  22374,  22468,  22561,
     22654,  22747,  22839,  22931,  23023,  23115,  23207,  23298,  23389,  23480,  23570,  23660,  23750,  23840,  23929,  24019,
     24107,  24196,  24284,  24372,  24460,  24547,  24634,  24721,  24807,  24893,  24979,  25064,  25149,  25234,  25319,  25403,
     25486,  25570,  25653,  25736,  25818,  25900,  25982,  26063,  26144,  26225,  26305,  26385,  26464,  26543,  26622,  26700,
     26778,  26855,  26933,  27009,  27086,  27162,  27237,  27312,  27387,  27461,  27535,  27608,  27681,  27754,  27826,  27898,
     27969,  28040,  28111,  28181,  28250,  28319,  28388,  28456,  28524,  28591,  28658,  28724,  28790,  28856,  28921,  28985,
     29049,  29113,  29176,  29238,  29300,  29362,  29423,  29484,  29544,  29603,  29663,  29721,  29779,  29837,  29894,  29951,
     30007,  30062,  30117,  30172,  30226,  30280,  30333,  30385,  30437,  30488,  30539,  30590,  30640,  30689,  30738,  30786,
     30833,  30881,  30927,  30973,  31019,  31064,  31108,  31152,  31195,  31238,  31280,  31321,  31362,  31403,  31443,  31482,
     31521,  31559,  31597,  31634,  31670,  31706,  31741,  31776,  31810,  31844,  31877,  31909,  31941,  31972,  32003,  32033,
     32063,  32091,  32120,  32147,  32175,  32201,  32227,  32252,  32277,  32301,  32325,  32348,  32370,  32392,  32413,  32433,
     32453,  32472,  32491,  32509,  32527,  32544,  32560,  32576,  32591,  32605,  32619,  32632,  32645,  32657,  32668,  32679,
     32689,  32699,  32708,  32716,  32724,  32731,  32737,  32743,  32748,  32753,  32757,  32760,  32763,  32765,  32767,  32767,
     32767,  32767,  32767,  32765,  32763,  32760,  32757,  32753,  32748,  32743,  32737,  32731,  32724,  32716,  32708,  32699,
     32689,  32679,  32668,  32657,  32645,  32632,  32619,  32605,  32591,  32576,  32560,  32544,  32527,  32509,  32491,  32472,
     32453,  32433,  32413,  32392,  32370,  32348,  32325,  32301,  32277,  32252,  32227,  32201,  32175,  32147,  32120,  32091,
     32063,  32033,  32003,  31972,  31941,  31909,  31877,  31844,  31810,  31776,  31741,  31706,  31670,  31634,  31597,  31559,
     31521,  31482,  31443,  31403,  31362,  31321,  31280,  31238,  31195,  31152,  31108,  31064,  31019,  30973,  30927,  30881,
     30833,  30786,  30738,  30689,  30640,  30590,  30539,  30488,  30437,  30385,  30333,  30280,  30226,  30172,  30117,  30062,
     30007,  29951,  29894,  29837,  29779,  29721,  29663,  29603,  29544,  29484,  29423,  29362,  29300,  29238,  29176,  29113,
     29049,  28985,  28921,  28856,  28790,  28724,  28658,  28591,  28524,  28456,  28388,  28319,  28250,  28181,  28111,  28040,
     27969,  27898,  27826,  27754,  27681,  27608,  27535,  27461,  27387,  27312,  27237,  27162,  27086,  27009,  26933,  26855,
     26778,  26700,  26622,  26543,  26464,  26385,  26305,  26225,  26144,  26063,  25982,  25900,  25818,  25736,  25653,  25570,
     25486,  25403,  25319,  25234,  25149,  25064,  24979,  24893,  24807,  24721,  24634,  24547,  24460,  24372,  24284,  24196,
     24107,  24019,  23929,  23840,  23750,  23660,  23570,  23480,  23389,  23298,  23207,  23115,  23023,  22931,  22839,  22747,
     22654,  22561,  22468,  22374,  22281,  22187,  22092,  21998,  21904,  21809,  21714,  21619,  21523,  21428,  21332,  21236,
     21140,  21044,  20947,  20851,  20754,  20657,  20560,  20462,  20365,  20267,  20170,  20072,  19974,  19876,  19777,  19679,
     19580,  19482,  19383,  19284,  19185,  19086,  18987,  18887,  18788,  18689,  18589,  18489,  18390,  18290,  18190,  18090,
     17990,  17890,  17790,  17690,  17589,  17489,  17389,  17288,  17188,  17088,  16987,  16887,  16786,  16686,  16585,  16485,
     16384,  16283,  16183,  16082,  15982,  15881,  15781,  15680,  15580,  15480,  15379,  15279,  15179,  15078,  14978,  14878,
     14778,  14678,  14578,  14478,  14378,  14279,  14179,  14079,  13980,  13881,  13781,  13682,  13583,  13484,  13385,  13286,
     13188,  13089,  12991,  12892,  12794,  12696,  12598,  12501,  12403,  12306,  12208,  12111,  12014,  11917,  11821,  11724,
     11628,  11532,  11436,  11340,  11245,  11149,  11054,  10959,  10864,  10770,  10676,  10581,  10487,  10394,  10300,  10207,
     10114,  10021,   9929,   9837,   9745,   9653,   9561,   9470,   9379,   9288,   9198,   9108,   9018,   8928,   8839,   8749,
      8661,   8572,   8484,   8396,   8308,   8221,   8134,   8047,   7961,   7875,   7789,   7704,   7619,   7534,   7449,   7365,
      7282,   7198,   7115,   7032,   6950,   6868,   6786,   6705,   6624,   6543,   6463,   6383,   6304,   6225,   6146,   6068,
      5990,   5913,   5835,   5759,   5682,   5606,   5531,   5456,   5381,   5307,   5233,   5160,   5087,   5014,   4942,   4870,
      4799,   4728,   4657,   4587,   4518,   4449,   4380,   4312,   4244,   4177,   4110,   4044,   3978,   3912,   3847,   3783,
      3719,   3655,   3592,   3530,   3468,   3406,   3345,   3284,   3224,   3165,   3105,   3047,   2989,   2931,   2874,   2817,
      2761,   2706,   2651,   2596,   2542,   2488,   2435,   2383,   2331,   2280,   2229,   2178,   2128,   2079,   2030,   1982,
      1935,   1887,   1841,   1795,   1749,   1704,   1660,   1616,   1573,   1530,   1488,   1447,   1406,   1365,   1325,   1286,
      1247,   1209,   1171,   1134,   1098,   1062,   1027,    992,    958,    924,    891,    859,    827,    796,    765,    735,
       705,    677,    648,    621,    593,    567,    541,    516,    491,    467,    443,    420,    398,    376,    355,    335,
       315,    296,    277,    259,    241,    224,    208,    192,    177,    163,    149,    136,    123,    111,    100,     89,
        79,     69,     60,     52,     44,     37,     31,     25,     20,     15,     11,      8,      5,      3,      1,      0,
};

static arm_rfft_instance_q15 rfft;

// Tracks whether the microphone and FFT have been set up this session.
static bool s_initialised   = false;
static bool s_bias_reported = false;

// Colour for a band: red at the low end, through green, to blue at the top.
// Integer ramps only, since this task avoids floating point.
static void band_colour(int led, uint8_t &r, uint8_t &g, uint8_t &b)
{
    int pos = led * 510 / (BOARD_LED_COUNT - 1); // 0..510 across the chain
    if (pos <= 255) {
        r = (uint8_t)(255 - pos);
        g = (uint8_t)pos;
        b = 0;
    } else {
        r = 0;
        g = (uint8_t)(510 - pos);
        b = (uint8_t)(pos - 255);
    }
}

void run_audio_task(LedDriver &leds)
{
    // One-time setup; exit resets the flag so re-entry re-initialises cleanly.
    if (!s_initialised) {
        microphone_init();
        arm_rfft_init_q15(&rfft, FFT_SIZE, 0, 1); // forward transform, bit-reversed output
        s_initialised = true;
    }

    // 1. Capture a block of samples. This blocks for ~23 ms by design.
    microphone_read(sample_buffer, FFT_SIZE);

    // 2. DC bias is the mean of the block. 1024 12-bit samples cannot overflow
    //    an int32, so no need for anything wider.
    int32_t sum = 0;
    for (int i = 0; i < FFT_SIZE; i++) {
        sum += sample_buffer[i];
    }
    int32_t dc_bias = sum / FFT_SIZE;

    // The amplifier biases the signal to mid-rail, so on a 12-bit ADC this
    // should land near 2048. Printed once per session as a sanity check.
    if (!s_bias_reported) {
        printf("Audio task: DC bias %d (expected ~2048)\n", (int)dc_bias);
        s_bias_reported = true;
    }

    // 3. Centre each sample on the bias and shift it up into Q15. __SSAT clamps
    //    to the signed 16-bit range: without it a swing past INPUT_CLIP_LEVEL
    //    would wrap and flip sign, turning a loud noise into spectral garbage.
    int32_t peak = 0;
    for (int i = 0; i < FFT_SIZE; i++) {
        int32_t centred = (int32_t)sample_buffer[i] - dc_bias;
        int32_t swing   = (centred < 0) ? -centred : centred;
        if (swing > peak) peak = swing;
        time_domain[i] = (q15_t)__SSAT(centred << INPUT_SHIFT, 16);
    }

    // 4. Window the block to reduce spectral leakage. Q15 * Q15 gives Q30, so
    //    shift back down by 15 to land in Q15 again.
    for (int i = 0; i < FFT_SIZE; i++) {
        time_domain[i] = (q15_t)(((int32_t)time_domain[i] * (int32_t)hanning[i]) >> 15);
    }

    // 5. Forward real FFT. CMSIS-DSP downscales internally to avoid overflow,
    //    so the output is ~Q11.5 rather than Q15 — do not renormalise it.
    arm_rfft_q15(&rfft, time_domain, fft_output);

    // 6. Magnitude squared is the energy spectral density. Feed the FFT output
    //    in unchanged; the result comes out in ~Q3.13.
    arm_cmplx_mag_squared_q15(fft_output, mag_squared, NUM_BINS);

    // 7. Sum the energy in each band and light its LED if it beats the threshold.
    int32_t max_band = 0;
    for (int led = 0; led < BOARD_LED_COUNT; led++) {
        int32_t band = 0;
        for (int bin = bin_edges[led]; bin < bin_edges[led + 1]; bin++) {
            band += mag_squared[bin];
        }
        if (band > max_band) max_band = band;

        if (band > LEVEL_THRESHOLD) {
            uint8_t r, g, b;
            band_colour(led, r, g, b);
            leds.set_one(led, r, g, b);
        } else {
            leds.set_one(led, 0, 0, 0);
        }
    }
    leds.show();

    if (AUDIO_DIAGNOSTICS) {
        static int diag_counter = 0;
        if (++diag_counter >= DIAG_FRAMES) {
            diag_counter = 0;
            printf("Audio: peak %d/%d  max band %d  (threshold %d)\n",
                   (int)peak, INPUT_CLIP_LEVEL, (int)max_band, (int)LEVEL_THRESHOLD);
        }
    }
}

void exit_audio_task(LedDriver &leds)
{
    // Stop sampling — the driver stops the ADC and drains the FIFO.
    microphone_stop();

    leds.off();
    leds.show();

    s_initialised   = false;
    s_bias_reported = false;
}
