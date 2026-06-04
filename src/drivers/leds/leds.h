#pragma once

#include <stdint.h>
#include <vector>

class LedDriver {
public:
    // A single LED's RGB colour — also used as the return type of get_one()
    struct Colour {
        uint8_t r, g, b;
    };

    // Initialise the chain with a fixed count. Allocates the staging buffer and
    // starts the PIO state machine. Must be called before any other method.
    explicit LedDriver(int num_leds);

    // Copying a hardware driver makes no sense — delete both copy operations
    LedDriver(const LedDriver&)            = delete;
    LedDriver& operator=(const LedDriver&) = delete;

    // --- LED count ----------------------------------------------------------

    void set_count(int num_leds); // Resize at runtime; existing values are preserved, new LEDs start off
    int  get_count() const;       // Return the current number of LEDs

    // --- RGB staging --------------------------------------------------------
    // None of these methods send data to the hardware.
    // Call show() to commit all staged changes at once.

    void set_all  (uint8_t r, uint8_t g, uint8_t b);                         // Stage every LED to one colour
    void set_one  (int index, uint8_t r, uint8_t g, uint8_t b);              // Stage one LED; others unchanged
    void set_range(int start, int count, uint8_t r, uint8_t g, uint8_t b);   // Stage a contiguous range

    // --- HSV staging --------------------------------------------------------
    // HSV is often easier for animations than RGB:
    //   hue: 0-360  position on the colour wheel (0=red, 120=green, 240=blue)
    //   sat: 0-1    vividness (0=grey, 1=pure colour)
    //   val: 0-1    brightness (0=off, 1=full brightness)

    void set_one_hsv  (int index, float hue, float sat, float val);
    void set_all_hsv  (float hue, float sat, float val);
    void set_range_hsv(int start, int count, float hue, float sat, float val);

    // --- Query --------------------------------------------------------------

    Colour get_one    (int index) const; // Return the staged colour of one LED
    bool   is_dirty   ()          const; // True if staged changes have not been sent to hardware yet
    void   print_status()         const; // Print every LED's staged colour and the dirty flag to stdout

    // --- Control ------------------------------------------------------------

    void off();   // Stage all LEDs off (equivalent to set_all(0,0,0))
    void show();  // Send all staged changes to the hardware and clear the dirty flag

    // Enable or disable the 300 µs busy-wait that show() performs after sending data.
    //
    // The WS2812 protocol requires the data line to stay idle for at least 280 µs
    // between frames so the LEDs can latch their new values (the "reset pulse").
    //
    // Enabled (default) — safe for any refresh rate; show() always waits before returning.
    // Disabled          — use when your code already guarantees more than 280 µs between
    //                     calls to show() (e.g. a 500 ms animation loop), removing a small
    //                     but unnecessary CPU stall.
    void set_busy_wait(bool enabled);

private:
    std::vector<Colour> colours_; // staging buffer; one Colour per LED
    bool                dirty_;
    bool                busy_wait_; // whether show() waits 300 µs for the WS2812 reset pulse

    void        write_leds()  const; // clock the full buffer out over PIO
    static void hsv_to_rgb(float h, float s, float v,
                           uint8_t *r, uint8_t *g, uint8_t *b);
};
