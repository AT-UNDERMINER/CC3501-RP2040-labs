#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "board.h" // board-specific pin and bus configuration
#include "drivers/leds/leds.h"

// task run/exit functions for each task
#include "tasks/led_task.h"
#include "tasks/accelerometer_task.h"
#include "tasks/audio_task.h"
#include "tasks/idle_task.h"


// Every task receives the one shared LedDriver by reference each frame.
static void (*const task_run[])(LedDriver&)  = { run_idle_task, run_led_task, run_accelerometer_task, run_audio_task };
static void (*const task_exit[])(LedDriver&) = { exit_idle_task, exit_led_task, exit_accelerometer_task, exit_audio_task };
static constexpr int NUM_TASKS = sizeof(task_run) / sizeof(task_run[0]); // array-length idiom; grows with the arrays

int main()
{
    stdio_init_all();

    gpio_init(SW1_PIN);
    gpio_set_dir(SW1_PIN, GPIO_IN);
    gpio_set_pulls(SW1_PIN, false, false); // external pull-down present; no internal pulls needed

    // The one LedDriver for the whole program. Constructing it loads the PIO
    // program and starts the state machine, so it must happen exactly once.
    LedDriver leds(BOARD_LED_COUNT);
    // The 5 ms loop pacing below always exceeds the WS2812 reset pulse, so
    // show() never needs to busy-wait for it — set once here for every task.
    leds.set_busy_wait(false);

    bool button_pressed = false;
    bool last_state     = false;
    int  current_task   = 0;

    for (;;) {
        // One cooperative frame of the current task, then back to polling.
        task_run[current_task](leds);

        // Poll SW1 (GPIO15); a rising edge begins press handling.
        bool state = gpio_get(SW1_PIN);

        if (state && !last_state) {
            // Confirm it's real: wait ~2τ and re-read. The 100 kΩ / 100 nF filter
            // (τ = 10 ms) already removes sub-millisecond glitches, so 20 ms settles it.
            sleep_ms(20);
            if (gpio_get(SW1_PIN)) {
                button_pressed = true;
                printf("Button pressed\n");

                // Wait for release so one physical press is one logical press.
                while (gpio_get(SW1_PIN)) {
                    sleep_ms(5);
                }
            }
            last_state = false; // pin is LOW again after the release wait
        } else {
            last_state = state;
        }

        // Confirmed press: clean up the task and advance.
        if (button_pressed) {
            task_exit[current_task](leds);
            current_task = (current_task + 1) % NUM_TASKS;
            button_pressed = false;
        }

        sleep_ms(5); // poll interval — comfortably above the RC settling time
    }

    return 0;
}
