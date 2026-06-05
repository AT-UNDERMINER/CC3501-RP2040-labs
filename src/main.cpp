#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "tasks/led_task.h"
#include "tasks/idle_task.h"
#include "board.h"

int main()
{
    stdio_init_all();

    for (;;) {
        run_idle_task();
        sleep_ms(30);
    }

    return 0;
}
