#pragma once

class LedDriver; // forward declaration — a reference param doesn't need the full type

void run_audio_task(LedDriver &leds);
void exit_audio_task(LedDriver &leds);
