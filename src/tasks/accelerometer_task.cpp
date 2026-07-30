#include "accelerometer_task.h"
#include "drivers/lis3dh/lis3dh.h"
#include "drivers/leds/leds.h"
#include "board.h"

#include <stdio.h>
#include <math.h>
#include <numbers>   // C++20 std::numbers::pi_v<float> — exact pi, no hand-typed literal
#include <algorithm> // std::clamp

static i2c_inst_t *const ACCEL_I2C = ACCEL_I2C_INSTANCE; // i2c_inst_t comes in via lis3dh.h

// Tilt thresholds in g.
static constexpr float LEVEL_THRESHOLD = 0.05f; // |X| and |Y| under this counts as level
static constexpr float MAG_RED         = 0.50f; // tilt this far or more turns the bubble red

// Bubble geometry on the 12-LED U-shape. The gap in the U is at the front,
// between LED 0 (front right) and LED 11 (front left).
static constexpr float   BACK_CENTRE_INDEX = 5.5f; // straight back sits between LEDs 5 and 6
static constexpr uint8_t FRONT_RIGHT_LED   = 0;
static constexpr uint8_t FRONT_LEFT_LED    = BOARD_LED_COUNT - 1;

static constexpr float GREEN_HUE  = 120.0f; // level
static constexpr float ORANGE_HUE = 10.0f;  // slight tilt
static constexpr float RED_HUE    = 0.0f;   // past MAG_RED

// Tracks whether the LIS3DH has been brought up for the current task session.
static bool s_initialised = false;

// One-shot guards so failure messages print once per session, not every frame
// while the condition persists. Reset on exit alongside s_initialised.
static bool s_init_error_logged = false;
static bool s_read_error_logged = false;

// Frames to wait between failed init attempts (~250 ms at main's 5 ms loop) so
// a missing sensor doesn't flood the terminal with retry messages.
static constexpr int INIT_RETRY_FRAMES = 50;
static int s_init_retry_countdown = 0; // 0 = attempt init this frame

// Pick the LED that represents the current tilt direction. Returns -1 when the
// board leans straight back, where the U's front gap means two LEDs share the
// job. Only called when the board is known to be tilted.
static int bubble_led(float gx, float gy)
{
    bool leaning_back = gy < -LEVEL_THRESHOLD;

    if (leaning_back && fabsf(gx) < LEVEL_THRESHOLD) return -1;              // straddles the gap
    if (leaning_back && gx < -LEVEL_THRESHOLD)       return FRONT_RIGHT_LED;
    if (leaning_back && gx >  LEVEL_THRESHOLD)       return FRONT_LEFT_LED;

    // Otherwise map the tilt angle onto the ring. atan2 gives 0 rad for a
    // forward lean, which belongs at the back centre; +pi/2 (right) swings the
    // bubble round to the left side. 8/pi is about 4 LED steps per 90 degrees.
    float angle = atan2f(gx, gy);
    int   idx   = (int)lroundf(BACK_CENTRE_INDEX + angle * (8.0f / std::numbers::pi_v<float>));
    return std::clamp(idx, 0, BOARD_LED_COUNT - 1);
}

void run_accelerometer_task(LedDriver &leds)
{
    // One-time setup; exit resets the flag so re-entry re-initialises cleanly.
    if (!s_initialised) {
        // Rate-limit failed attempts so a missing sensor is probed every
        // ~250 ms instead of every frame.
        if (s_init_retry_countdown > 0) {
            s_init_retry_countdown--;
            return;
        }
        if (!lis3dh_init(ACCEL_I2C, lis3dh_odr_t::ODR_100HZ)) {
            if (!s_init_error_logged) {
                printf("Accelerometer task: LIS3DH init failed, will keep retrying\n");
                s_init_error_logged = true;
            }
            s_init_retry_countdown = INIT_RETRY_FRAMES;
            return; // flag stays false, so a later frame tries again
        }
        s_initialised = true;
    }

    int16_t rx, ry, rz;
    if (!lis3dh_read_raw(ACCEL_I2C, &rx, &ry, &rz)) {
        if (!s_read_error_logged) {
            printf("Accelerometer task: LIS3DH read failed, skipping frames\n");
            s_read_error_logged = true;
        }
        return;
    }

    float gx = lis3dh_to_g(rx);
    float gy = lis3dh_to_g(ry);
    float gz = lis3dh_to_g(rz);

    printf("Accel  X=%+.3f g  Y=%+.3f g  Z=%+.3f g\n", gx, gy, gz);

    // Spirit level: all green when flat, otherwise a single bubble LED shows
    // which way the board is leaning.
    if (fabsf(gx) < LEVEL_THRESHOLD && fabsf(gy) < LEVEL_THRESHOLD) {
        leds.set_all_hsv(GREEN_HUE, 1.0f, 0.8f);
    } else {
        float magnitude = sqrtf(gx * gx + gy * gy);
        float hue = (magnitude >= MAG_RED) ? RED_HUE : ORANGE_HUE; // colour encodes how far it leans

        leds.off();
        int led = bubble_led(gx, gy);
        if (led < 0) {
            leds.set_many_hsv({FRONT_RIGHT_LED, FRONT_LEFT_LED}, hue, 1.0f, 1.0f);
        } else {
            leds.set_one_hsv(led, hue, 1.0f, 1.0f);
        }
    }

    leds.show();
}

void exit_accelerometer_task(LedDriver &leds)
{
    // Stop sampling — the driver handles the power-down register write.
    if (!lis3dh_power_down(ACCEL_I2C)) {
        printf("Accelerometer task: LIS3DH power-down failed\n");
    }

    leds.off();
    leds.show();

    // Force re-initialisation next time the task is selected, re-arm the
    // one-shot error messages, and clear the retry countdown so re-entry
    // attempts init immediately rather than waiting out a stale counter.
    s_initialised          = false;
    s_init_error_logged    = false;
    s_read_error_logged    = false;
    s_init_retry_countdown = 0;
}
