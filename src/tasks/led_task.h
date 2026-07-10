#pragma once

class LedDriver; // forward declaration — a reference param doesn't need the full type

void run_led_task(LedDriver &leds);
void exit_led_task(LedDriver &leds);
