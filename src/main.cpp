#include <stdio.h>
#include "pico/stdlib.h"
#include "tasks/led_task.h"

int main()
{
    stdio_init_all();
    led_task();
    return 0;
}