#pragma once

class LedDriver; // forward declaration — a reference param doesn't need the full type

void run_accelerometer_task(LedDriver &leds);
void exit_accelerometer_task(LedDriver &leds);
