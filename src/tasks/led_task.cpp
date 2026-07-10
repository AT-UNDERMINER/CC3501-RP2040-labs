#include "led_task.h"
#include "drivers/leds/leds.h"
#include "drivers/logging/logging.h"
#include "pico/stdlib.h"
#include "board.h"

// Each demo step is held for this many calls before advancing.
static constexpr int FRAMES_PER_STEP = 100;

// Number of demo steps — must match the cases in run_led_task()'s switch.
static constexpr int NUM_STEPS = 6;

void run_led_task(LedDriver &leds)
{
    static int step          = 0;
    static int frame_counter = 0;

    // Each step's setup runs once (on its first frame); the rest hold the display.
    if (frame_counter == 0) {
        switch (step) {
            case 0:
                // set_one stages each LED; show() commits the whole buffer at once.
                leds.set_one(0, 255, 0,   0);   // red
                leds.set_one(1, 0,   255, 0);   // green
                leds.set_one(2, 0,   0,   255); // blue
                leds.show();
                leds.print_status();
                break;

            case 1:
                // set_one_hsv: sweep the hue around the wheel across the chain.
                for (int led = 0; led < leds.get_count(); led++) {
                    float hue = (360.0f / leds.get_count()) * led;
                    leds.set_one_hsv(led, hue, 1.0f, 1.0f);
                }
                leds.show();
                break;

            case 2:
                // set_range: two contiguous blocks, each one colour.
                leds.set_range(0, 6, 128, 255, 0);
                leds.set_range(6, 6, 0,   255, 128);
                leds.show();
                break;

            case 3:
                // set_many: a non-contiguous set in one call (off() clears the rest first).
                leds.off();
                leds.set_many({0, 2, 4, 6, 8, 10}, 255, 0, 255); // magenta
                leds.show();
                break;

            case 4:
                // set_many_hsv: same idea, HSV, on the odd LEDs.
                leds.off();
                leds.set_many_hsv({1, 3, 5, 7, 9, 11}, 180.0f, 1.0f, 1.0f); // cyan
                leds.show();
                break;

            case 5:
                // Dump state, log, then clear the chain.
                leds.print_status();
                log(LogLevel::INFORMATION, "Range update complete.");
                leds.off();
                leds.show();
                break;
        }
    }

    // Hold each step for FRAMES_PER_STEP frames, then advance and wrap.
    frame_counter++;
    if (frame_counter >= FRAMES_PER_STEP) {
        frame_counter = 0;
        step = (step + 1) % NUM_STEPS;
    }
}

void exit_led_task(LedDriver &leds)
{
    leds.off();
    leds.show();
}
