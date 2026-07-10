#include "idle_task.h"
#include "drivers/leds/leds.h"
#include "board.h"
#include <math.h>

// 1 s rise (off → peak) + 1 s fall (peak → off) = 2 s full cycle.
// main's loop is paced by sleep_ms(5), so steps per cycle = 2000 / 5 = 400.
// Phase step = 2π / 400 ≈ 0.015708 rad/call.
static constexpr float BREATH_STEP = 0.015708f;
static constexpr float TWO_PI      = 6.28318530f;
static constexpr float PHASE_START = 4.71238898f; // 3π/2: sin = -1, brightness opens at 0
static constexpr float BLUE_HUE    = 240.0f;
static constexpr float PEAK_VAL    = 0.07f;       // peak brightness — deliberately dim

void run_idle_task(LedDriver &leds)
{
    static bool  initialized  = false;
    static float breath_phase = PHASE_START;

    if (!initialized) {
        // The 5 ms main loop already outlasts the WS2812 reset pulse, so show()
        // needn't busy-wait for it.
        leds.set_busy_wait(false);
        initialized = true;
    }

    // (sin + 1) * 0.5 maps the wave to 0..1; PEAK_VAL caps the brightness.
    float val = PEAK_VAL * (sinf(breath_phase) + 1.0f) * 0.5f;
    leds.set_all_hsv(BLUE_HUE, 1.0f, val);
    leds.show();

    breath_phase = fmodf(breath_phase + BREATH_STEP, TWO_PI); // advance, wrap at 2π
}

void exit_idle_task(LedDriver &leds)
{
    leds.off();
    leds.show();
}
