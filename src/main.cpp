#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "board.h" // board-specific pin and bus configuration
#include "drivers/leds/leds.h"
#include "drivers/logging/logging.h"

// task run/exit functions for each task
#include "tasks/led_task.h"
#include "tasks/accelerometer_task.h"
#include "tasks/audio_task.h"
#include "tasks/idle_task.h"


// Every task receives the one shared LedDriver by reference each frame.
static void (*const task_run[])(LedDriver&)  = { run_idle_task, run_led_task, run_accelerometer_task, run_audio_task };
static void (*const task_exit[])(LedDriver&) = { exit_idle_task, exit_led_task, exit_accelerometer_task, exit_audio_task };
static constexpr int NUM_TASKS = sizeof(task_run) / sizeof(task_run[0]); // array-length idiom; grows with the arrays

// Pacing for the scheduler loop. idle_task's breathing maths derives its
// steps-per-cycle from this value — change one, check the other.
static constexpr int LOOP_PERIOD_MS = 5;

// Set by the ISR, cleared in main(). volatile stops the compiler caching the
// value, since it changes outside the normal flow of main().
static volatile bool s_button_pressed = false;

// GPIO interrupt callback for SW1. Deliberately minimal — the task_exit
// functions do blocking I2C/ADC/PIO work that is unsafe in interrupt context,
// so the ISR only sets a flag and main() does the real task-switch work.
static void gpio_callback(uint gpio, uint32_t events)
{
    s_button_pressed = true;
}

int main()
{
    stdio_init_all();

    gpio_init(SW1_PIN);
    gpio_set_dir(SW1_PIN, GPIO_IN);
    gpio_set_pulls(SW1_PIN, false, false); // external pull-down present; no internal pulls needed

    // Rising edge = button press (external pull-down: idle LOW, press pulls HIGH).
    // Debounce is the hardware RC filter (100 kΩ / 100 nF, τ = 10 ms) plus the
    // GPIO input's Schmitt trigger, so no software debounce is needed here.
    gpio_set_irq_enabled_with_callback(SW1_PIN, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);

    // The one LedDriver for the whole program. Constructing it loads the PIO
    // program and starts the state machine, so it must happen exactly once.
    LedDriver leds(BOARD_LED_COUNT);
    // The 5 ms loop pacing below always exceeds the WS2812 reset pulse, so
    // show() never needs to busy-wait for it — set once here for every task.
    leds.set_busy_wait(false);

    int current_task = 0;

    for (;;) {
        // One cooperative frame of the current task.
        task_run[current_task](leds);

        // Consume a press flagged by the ISR: clean up the task and advance.
        // One rising edge = one switch, even if the button is held down.
        if (s_button_pressed) {
            s_button_pressed = false;
            log(LogLevel::INFORMATION, "Button pressed"); // safe here in main(), not in the ISR
            task_exit[current_task](leds);
            current_task = (current_task + 1) % NUM_TASKS;
        }

        sleep_ms(LOOP_PERIOD_MS); // frame pacing — the button no longer needs polling
    }

    return 0;
}
