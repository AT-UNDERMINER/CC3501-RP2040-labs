#include "idle_task.h"
#include "drivers/leds/leds.h"
#include "board.h"
#include <math.h>
#include <numbers> // C++20 std::numbers::pi_v<float> — exact π, no hand-typed literals

// 1 s rise (off → peak) + 1 s fall (peak → off) = 2 s full cycle.
// main's loop is paced by sleep_ms(5), so steps per cycle = 2000 / 5 = 400.
static constexpr float TWO_PI      = 2.0f * std::numbers::pi_v<float>;
static constexpr float BREATH_STEP = TWO_PI / 400.0f;                  // phase advance per call
static constexpr float PHASE_START = 1.5f * std::numbers::pi_v<float>; // 3π/2: sin = -1, brightness opens at 0
static constexpr float BLUE_HUE    = 240.0f;
static constexpr float PEAK_VAL    = 0.07f;       // peak brightness — deliberately dim

void run_idle_task(LedDriver &leds)
{
    static float breath_phase = PHASE_START;

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
