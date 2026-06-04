#include "led_task.h"
#include "drivers/leds/leds.h"
#include "drivers/logging/logging.h"
#include "pico/stdlib.h"
#include "board.h"

void led_task()
{
    LedDriver leds(BOARD_LED_COUNT);

    // busy_wait is enabled by default — show() will hold for 300 µs after each
    // frame so the WS2812 reset pulse is always satisfied between updates.
    leds.set_busy_wait(false);

    for (;;) {
        // Set individual LEDs to different colours, then commit all at once
        leds.set_one(0, 255, 0,   0);   // red
        leds.set_one(1, 0,   255, 0);   // green
        leds.set_one(2, 0,   0,   255); // blue
        leds.show();
        leds.print_status();
        sleep_ms(500);

        // Use HSV to sweep the hue across all LEDs
        for (int led = 0; led < leds.get_count(); led++) {
            float hue = (360.0f / leds.get_count()) * led;
            leds.set_one_hsv(led, hue, 1.0f, 1.0f);
        }
        leds.show();
        sleep_ms(500);

        // Set a range to one colour, then query and log the result
        leds.set_range(0, 6, 128, 255, 0);
        leds.set_range(6, 6, 0,   255, 128);
        leds.show();
        sleep_ms(500);

        leds.print_status();
        log(LogLevel::INFORMATION, "Range update complete.");

        leds.off();
        leds.show();
        sleep_ms(500);
    }
}
