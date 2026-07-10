#include "accelerometer_task.h"
#include "drivers/lis3dh/lis3dh.h"
#include "drivers/leds/leds.h"
#include "board.h"

#include <stdio.h>
#include <math.h>
#include <numbers>   // C++20 std::numbers::pi_v<float> — exact π, no hand-typed literal
#include <algorithm> // std::clamp

static i2c_inst_t *const ACCEL_I2C = ACCEL_I2C_INSTANCE; // i2c_inst_t comes in via lis3dh.h

// A tilt of this many g on an axis is treated as "significantly tilted".
// static constexpr float TILT_THRESHOLD = 0.1f;

// Tracks whether the LIS3DH has been brought up for the current task session.
static bool s_initialised = false;

// One-shot guards so failure messages print once per session, not every 5 ms
// frame while the condition persists. Reset on exit alongside s_initialised.
static bool s_init_error_logged = false;
static bool s_read_error_logged = false;

// Frames to wait between failed init attempts (~250 ms at main's 5 ms loop) so
// the driver's own WHO_AM_I error print doesn't repeat at full frame rate.
static constexpr int INIT_RETRY_FRAMES = 50;
static int s_init_retry_countdown = 0; // 0 = attempt init this frame

void run_accelerometer_task(LedDriver &leds)
{
    // One-time setup; exit resets the flag so re-entry re-initialises cleanly.
    if (!s_initialised) {
        // Rate-limit failed attempts: burn down the countdown between tries so
        // a missing sensor is probed every ~250 ms, not every frame.
        if (s_init_retry_countdown > 0) {
            s_init_retry_countdown--;
            return;
        }
        if (!lis3dh_init(ACCEL_I2C, lis3dh_odr_t::ODR_100HZ)) {
            // Flag stays false so a later frame retries instead of streaming
            // garbage from an unconfigured device.
            if (!s_init_error_logged) {
                printf("Accelerometer task: LIS3DH init failed, will keep retrying\n");
                s_init_error_logged = true;
            }
            s_init_retry_countdown = INIT_RETRY_FRAMES;
            return;
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

    // Bubble level: a single LED marks the tilt direction around the U-shape;
    // all-green means the board is flat.
    const float LEVEL_THRESHOLD = 0.05f; // |X| and |Y| under this counts as level
    const float MAG_RED         = 0.50f; // tilt magnitude at/above this -> red bubble

    float magnitude = sqrtf(gx * gx + gy * gy);

    if (fabsf(gx) < LEVEL_THRESHOLD && fabsf(gy) < LEVEL_THRESHOLD) {
        // Level — steady green across all 12 LEDs.
        leds.set_all_hsv(120.0f, 1.0f, 0.8f);
    } else {
        // Tilted — bubble colour comes from how far it is tilted.
        float hue = (magnitude >= MAG_RED) ? 0.0f : 10.0f; // red vs orange
        float val = (magnitude >= MAG_RED) ? 1.0f : 1.0f;

        leds.off();

        if (gy < -LEVEL_THRESHOLD && fabsf(gx) < LEVEL_THRESHOLD && magnitude > LEVEL_THRESHOLD) {
            // Tilting straight back toward the front gap — light both front LEDs.
            leds.set_many_hsv({0, 11}, hue, 1.0f, val);
        } else if (gy < -LEVEL_THRESHOLD && gx < -LEVEL_THRESHOLD) {
            // Back and left — front-right LED only.
            leds.set_one_hsv(0, hue, 1.0f, val);
        } else if (gy < -LEVEL_THRESHOLD && gx > LEVEL_THRESHOLD) {
            // Back and right — front-left LED only.
            leds.set_one_hsv(11, hue, 1.0f, val);
        } else {
            // General case: map the tilt angle onto one LED around the U-shape.
            // angle 0 (forward) -> back centre, +pi/2 (right) -> left side,
            // -pi/2 (left) -> right side; back tilts are handled above.
            float angle = atan2f(gx, gy);
            // 5.5 = back-centre index; 8/π ≈ 4 LED steps per 90° of tilt.
            int idx = (int)lroundf(5.5f + angle * (8.0f / std::numbers::pi_v<float>));
            idx = std::clamp(idx, 0, 11);
            leds.set_one_hsv(idx, hue, 1.0f, val);
        }
    }

    leds.show();

    // // Stage all LEDs off, then light the group(s) for any axis past the threshold.
    // leds.off();
    // if (gx >  TILT_THRESHOLD) leds.set_range(GROUP_POS_X, GROUP_LEN, 0, 255, 0);
    // if (gx < -TILT_THRESHOLD) leds.set_range(GROUP_NEG_X, GROUP_LEN, 0, 255, 0);
    // if (gy >  TILT_THRESHOLD) leds.set_many({0, 11}, 0, 0, 255);
    // if (gy < -TILT_THRESHOLD) leds.set_range(GROUP_NEG_Y, GROUP_LEN, 0, 0, 255);
    // leds.show();

    // Different lighting approach
//     leds.off();
//     if (gx > TILT_THRESHOLD) {
//         int brightness = (int)(255.0f * (gx - TILT_THRESHOLD) / (1.0f - TILT_THRESHOLD));
//         leds.set_range(GROUP_POS_X, GROUP_LEN, brightness, 0, 0);
//     }
//     if (gx < -TILT_THRESHOLD) {
//         int brightness = (int)(255.0f * (-gx - TILT_THRESHOLD) / (1.0f - TILT_THRESHOLD));
//         leds.set_range(GROUP_NEG_X, GROUP_LEN, brightness, 0, 0);
//     }
//     if (gy > TILT_THRESHOLD) {
//         int brightness = (int)(255.0f * (gy - TILT_THRESHOLD) / (1.0f - TILT_THRESHOLD));
//         leds.set_range(GROUP_POS_Y, GROUP_LEN, 0, 0, brightness);
//     }
//     if (gy < -TILT_THRESHOLD) {
//         int brightness = (int)(255.0f * (-gy - TILT_THRESHOLD) / (1.0f - TILT_THRESHOLD));
//         leds.set_range(GROUP_NEG_Y, GROUP_LEN, 0, 0, brightness);
//     }
//     if (gx < TILT_THRESHOLD && gx > -TILT_THRESHOLD && gy < TILT_THRESHOLD && gy > -TILT_THRESHOLD) {
//         leds.set_all(0, 125, 0);
//     }
//     leds.show();

}

void exit_accelerometer_task(LedDriver &leds)
{
    // Stop sampling — the driver handles the power-down register write.
    if (!lis3dh_power_down(ACCEL_I2C)) {
        printf("Accelerometer task: LIS3DH power-down failed\n");
    }

    // Turn off every LED.
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
