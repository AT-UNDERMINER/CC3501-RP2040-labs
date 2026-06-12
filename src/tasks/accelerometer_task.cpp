#include "accelerometer_task.h"
#include "drivers/lis3dh/lis3dh.h"
#include "drivers/leds/leds.h"
#include "board.h"

#include <stdio.h>
#include <math.h>
#include "hardware/i2c.h"

// I2C bus the LIS3DH is wired to (SDA=GPIO16, SCL=GPIO17).
static i2c_inst_t *const ACCEL_I2C = i2c0;

// A tilt of this many g on an axis is treated as "significantly tilted".
static constexpr float TILT_THRESHOLD = 0.1f;

// LED groups on the 12-LED U-shape, one group per tilt direction.
// (Indices are an assumption about the physical layout — adjust the ranges
//  here if the lit side does not match the way the board actually tilts.)
//   +X (tilt right)   -> LEDs 0..2     red
//   +Y (tilt forward) -> LEDs 3..5     blue
//   -X (tilt left)    -> LEDs 6..8     red
//   -Y (tilt back)    -> LEDs 9..11    blue
static constexpr int GROUP_LEN     = 3;
static constexpr int GROUP_POS_X   = 0;  // tilt right
static constexpr int GROUP_POS_Y   = 3;  // tilt forward
static constexpr int GROUP_NEG_X   = 9;  // tilt left
static constexpr int GROUP_NEG_Y   = 6;  // tilt back

// Lazily-constructed driver instance shared by run/exit, matching the other tasks.
static LedDriver& get_leds()
{
    static LedDriver leds(BOARD_LED_COUNT);
    return leds;
}

// Tracks whether the LIS3DH has been brought up for the current task session.
static bool s_initialised = false;

void run_accelerometer_task()
{
    LedDriver &leds = get_leds();

    // One-time setup the first time the task runs (or after exit reset the flag).
    if (!s_initialised) {
        leds.set_busy_wait(false);
        lis3dh_init(ACCEL_I2C);
        s_initialised = true;
    }

    int16_t rx, ry, rz;
    lis3dh_read_raw(ACCEL_I2C, &rx, &ry, &rz);

    float gx = lis3dh_to_g(rx);
    float gy = lis3dh_to_g(ry);
    float gz = lis3dh_to_g(rz);

    printf("Accel  X=%+.3f g  Y=%+.3f g  Z=%+.3f g\n", gx, gy, gz);

    // // Stage all LEDs off, then light the group(s) for any axis past the threshold.
    // leds.off();
    // if (gx >  TILT_THRESHOLD) leds.set_range(GROUP_POS_X, GROUP_LEN, 0, 255, 0);
    // if (gx < -TILT_THRESHOLD) leds.set_range(GROUP_NEG_X, GROUP_LEN, 0, 255, 0);
    // if (gy >  TILT_THRESHOLD) leds.set_range(GROUP_POS_Y, GROUP_LEN, 0, 0, 255);
    // if (gy < -TILT_THRESHOLD) leds.set_range(GROUP_NEG_Y, GROUP_LEN, 0, 0, 255);
    // leds.show();

    // Different lighting approach
    leds.off();
    if (gx > TILT_THRESHOLD) {
        int brightness = (int)(255.0f * (gx - TILT_THRESHOLD) / (1.0f - TILT_THRESHOLD));
        leds.set_range(GROUP_POS_X, GROUP_LEN, brightness, 0, 0);
    }
    if (gx < -TILT_THRESHOLD) {
        int brightness = (int)(255.0f * (-gx - TILT_THRESHOLD) / (1.0f - TILT_THRESHOLD));
        leds.set_range(GROUP_NEG_X, GROUP_LEN, brightness, 0, 0);
    }
    if (gy > TILT_THRESHOLD) {
        int brightness = (int)(255.0f * (gy - TILT_THRESHOLD) / (1.0f - TILT_THRESHOLD));
        leds.set_range(GROUP_POS_Y, GROUP_LEN, 0, 0, brightness);
    }
    if (gy < -TILT_THRESHOLD) {
        int brightness = (int)(255.0f * (-gy - TILT_THRESHOLD) / (1.0f - TILT_THRESHOLD));
        leds.set_range(GROUP_NEG_Y, GROUP_LEN, 0, 0, brightness);
    }
    if (gx < TILT_THRESHOLD && gx > -TILT_THRESHOLD && gy < TILT_THRESHOLD && gy > -TILT_THRESHOLD) {
        leds.set_all(0, 125, 0);
    }
    leds.show();

}

void exit_accelerometer_task()
{
    // Power the LIS3DH down: CTRL_REG1 = 0x00 (ODR=0 -> power-down mode).
    uint8_t buf[2] = { 0x20, 0x00 };
    i2c_write_blocking(ACCEL_I2C, LIS3DH_ADDR, buf, 2, false);

    // Turn off every LED.
    LedDriver &leds = get_leds();
    leds.off();
    leds.show();

    // Force re-initialisation next time the task is selected.
    s_initialised = false;
}
