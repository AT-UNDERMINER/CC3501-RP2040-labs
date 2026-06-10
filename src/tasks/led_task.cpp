#include "led_task.h"
#include "drivers/leds/leds.h"
#include "drivers/logging/logging.h"
#include "pico/stdlib.h"
#include "board.h"

// Each demo step is held for this many calls before advancing.
static constexpr int FRAMES_PER_STEP = 100;

// Lazily-constructed driver instance shared by run_led_task() and exit_led_task().
static LedDriver& get_leds()
{
    static LedDriver leds(BOARD_LED_COUNT);
    return leds;
}

void run_led_task()
{
    LedDriver &leds = get_leds();
    static bool initialized   = false;
    static int  step          = 0;
    static int  frame_counter = 0;

    if (!initialized) {
        leds.set_busy_wait(false);
        initialized = true;
    }

    if (frame_counter == 0) {
        switch (step) {
            case 0:
                // Set individual LEDs to different colours, then commit all at once
                leds.set_one(0, 255, 0,   0);   // red
                leds.set_one(1, 0,   255, 0);   // green
                leds.set_one(2, 0,   0,   255); // blue
                leds.show();
                leds.print_status();
                break;

            case 1:
                // Use HSV to sweep the hue across all LEDs
                for (int led = 0; led < leds.get_count(); led++) {
                    float hue = (360.0f / leds.get_count()) * led;
                    leds.set_one_hsv(led, hue, 1.0f, 1.0f);
                }
                leds.show();
                break;

            case 2:
                // Set a range to one colour, then query and log the result
                leds.set_range(0, 6, 128, 255, 0);
                leds.set_range(6, 6, 0,   255, 128);
                leds.show();
                break;

            case 3:
                leds.print_status();
                log(LogLevel::INFORMATION, "Range update complete.");
                leds.off();
                leds.show();
                break;
        }
    }

    frame_counter++;
    if (frame_counter >= FRAMES_PER_STEP) {
        frame_counter = 0;
        step = (step + 1) % 4;
    }
}

void exit_led_task()
{
    LedDriver &leds = get_leds();
    leds.off();
    leds.show();
}
