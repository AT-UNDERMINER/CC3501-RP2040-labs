#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "tasks/led_task.h"
#include "tasks/idle_task.h"
#include "board.h"

// int main()
// {
//     stdio_init_all();

//     for (;;) {
//         run_idle_task();
//         sleep_ms(30);
//     }

//     return 0;
// }

int main()
{
    stdio_init_all();

    gpio_init(SW1_PIN);
    gpio_set_dir(SW1_PIN, GPIO_IN);
    gpio_set_pulls(SW1_PIN, false, false); // external pull-down present; no internal pulls needed

    bool button_pressed = false;
    bool last_state     = false;

    for (;;) {
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

        sleep_ms(5); // polling interval — well above the RC filter's settling time
    }

    return 0;
}