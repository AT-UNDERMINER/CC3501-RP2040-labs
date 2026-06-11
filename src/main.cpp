#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "board.h"

// Forward declarations of task run/exit functions.
// To add a new task: declare its run/exit functions here and add them to
// task_run[] / task_exit[] below in the same order — no other changes needed.
void run_idle_task();
void exit_idle_task();
void run_led_task();
void exit_led_task();
void run_accelerometer_task();
void exit_accelerometer_task();

static void (*const task_run[])()  = { run_idle_task, run_led_task, run_accelerometer_task };
static void (*const task_exit[])() = { exit_idle_task, exit_led_task, exit_accelerometer_task };
static constexpr int NUM_TASKS = sizeof(task_run) / sizeof(task_run[0]);

int main()
{
    stdio_init_all();

    gpio_init(SW1_PIN);
    gpio_set_dir(SW1_PIN, GPIO_IN);
    gpio_set_pulls(SW1_PIN, false, false); // external pull-down present; no internal pulls needed

    bool button_pressed = false;
    bool last_state     = false;
    int  current_task   = 0;

    for (;;) {
        // a. Run one frame/step of the current task
        task_run[current_task]();

        // b. Poll GPIO15 for a button press (rising edge)
        bool state = gpio_get(SW1_PIN);

        if (state && !last_state) {
            // Rising edge detected — wait one RC time constant for the signal to settle,
            // then confirm the pin is still high before treating it as a genuine press.
            // With a 100 kΩ / 100 nF RC filter (τ = 10 ms), 20 ms covers 2τ and is
            // sufficient; the hardware has already suppressed sub-millisecond glitches.
            sleep_ms(20);
            if (gpio_get(SW1_PIN)) {
                button_pressed = true;
                printf("Button pressed\n");

                // Block until the button is released so a single press cannot
                // re-trigger on the next polling iteration.
                while (gpio_get(SW1_PIN)) {
                    sleep_ms(5);
                }
            }
            last_state = false; // pin is now LOW after the release wait
        } else {
            last_state = state;
        }

        // c. Switch to the next task on a button press
        if (button_pressed) {
            task_exit[current_task]();
            current_task = (current_task + 1) % NUM_TASKS;
            button_pressed = false;
        }

        sleep_ms(5); // polling interval — well above the RC filter's settling time
    }

    return 0;
}
