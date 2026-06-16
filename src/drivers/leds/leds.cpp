#include "leds.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "WS2812.pio.h"
#include "board.h"

#define BOARD_LED_FREQ     800000   // WS2812 data-line frequency in Hz

// Minimum idle time (µs) required between WS2812 frames for the reset pulse.
// 280 µs is the spec minimum; 300 µs adds a small margin.
static constexpr uint32_t WS2812_RESET_US = 300;

// --- Private methods 

// Clocks the full staging buffer out over PIO.
// WS2812 LEDs latch all values simultaneously when the data line goes idle,
// so the entire chain must be sent even when only one LED changes.
void LedDriver::write_leds() const
{
    for (const Colour &c : colours_) {
        // Pack into the top 24 bits in WS2812 GRB wire order: G[31:24] R[23:16] B[15:8]
        // Explicit uint32_t cast prevents undefined behaviour for values >= 128
        uint32_t word = ((uint32_t)c.g << 24)
                      | ((uint32_t)c.r << 16)
                      | ((uint32_t)c.b <<  8);
        pio_sm_put_blocking(pio0, 0, word);
    }
}

// Converts HSV to RGB, writing results into *r, *g, *b.
//
// The colour wheel is divided into six 60-degree sectors. Within each sector,
// one primary channel is at peak brightness, one interpolates up or down, and
// one sits at the minimum set by saturation and value.
void LedDriver::hsv_to_rgb(float h, float s, float v,
                            uint8_t *r, uint8_t *g, uint8_t *b)
{
    // Clamp inputs to valid ranges
    if (h <   0.0f) h =   0.0f;
    if (h >= 360.0f) h = 359.9f;
    if (s <   0.0f) s = 0.0f;  if (s > 1.0f) s = 1.0f;
    if (v <   0.0f) v = 0.0f;  if (v > 1.0f) v = 1.0f;

    if (s == 0.0f) {
        // No saturation = achromatic; brightness alone sets the level
        *r = *g = *b = (uint8_t)(v * 255.0f);
        return;
    }

    float sector = h / 60.0f;   // which 60-degree sector (0-5)
    int   i      = (int)sector;  // integer sector index
    float f      = sector - i;   // fractional position within the sector

    // Pre-compute the three transition levels
    uint8_t p  = (uint8_t)(v * (1.0f - s)              * 255.0f); // minimum level
    uint8_t q  = (uint8_t)(v * (1.0f - s * f)          * 255.0f); // falling transition
    uint8_t t  = (uint8_t)(v * (1.0f - s * (1.0f - f)) * 255.0f); // rising transition
    uint8_t vv = (uint8_t)(v                            * 255.0f); // peak level

    switch (i) {
        case 0: *r = vv; *g = t;  *b = p;  break; // red → yellow
        case 1: *r = q;  *g = vv; *b = p;  break; // yellow → green
        case 2: *r = p;  *g = vv; *b = t;  break; // green → cyan
        case 3: *r = p;  *g = q;  *b = vv; break; // cyan → blue
        case 4: *r = t;  *g = p;  *b = vv; break; // blue → magenta
        default:*r = vv; *g = p;  *b = q;  break; // magenta → red
    }
}

// --- Constructor 

// The member-initialiser list runs before the body:
//   colours_(num_leds) — value-initialises each Colour to {0,0,0} (all LEDs off)
//   dirty_(false)      — buffer matches hardware state at startup
LedDriver::LedDriver(int num_leds)
    : colours_(num_leds), dirty_(false), busy_wait_(true)
{
    uint offset = pio_add_program(pio0, &ws2812_program);
    ws2812_program_init(pio0, 0, offset, BOARD_LED_PIN, BOARD_LED_FREQ, BOARD_LED_IS_RGBW);
}

// --- LED count 

void LedDriver::set_count(int new_count)
{
    if (new_count <= 0) return;
    // resize() keeps existing entries intact and value-initialises any new ones to {0,0,0}
    colours_.resize(new_count);
    dirty_ = true;
}

int LedDriver::get_count() const
{
    return (int)colours_.size();
}

// --- RGB staging 

void LedDriver::set_all(uint8_t r, uint8_t g, uint8_t b)
{
    for (Colour &c : colours_) { c = {r, g, b}; }
    dirty_ = true;
}

void LedDriver::set_one(int index, uint8_t r, uint8_t g, uint8_t b)
{
    if (index < 0 || index >= (int)colours_.size()) return;
    colours_[index] = {r, g, b};
    dirty_ = true;
}

void LedDriver::set_range(int start, int count, uint8_t r, uint8_t g, uint8_t b)
{
    // Clamp so the range never strays outside the vector
    if (start < 0) start = 0;
    int end = start + count;
    if (end > (int)colours_.size()) end = (int)colours_.size();

    for (int i = start; i < end; i++) { colours_[i] = {r, g, b}; }
    dirty_ = true;
}

void LedDriver::set_many(std::initializer_list<uint8_t> indices,
                         uint8_t r, uint8_t g, uint8_t b)
{
    // Stage an arbitrary list of LEDs to one colour. set_one() bounds-checks
    // each index, so out-of-range entries are silently ignored.
    for (uint8_t index : indices) { set_one(index, r, g, b); }
}

void LedDriver::set_many_hsv(std::initializer_list<uint8_t> indices,
                             float h, float s, float v)
{
    // HSV counterpart to set_many(). set_one_hsv() bounds-checks each index,
    // so out-of-range entries are silently ignored.
    for (uint8_t index : indices) { set_one_hsv(index, h, s, v); }
}

// --- HSV staging 

void LedDriver::set_one_hsv(int index, float hue, float sat, float val)
{
    if (index < 0 || index >= (int)colours_.size()) return;
    hsv_to_rgb(hue, sat, val, &colours_[index].r, &colours_[index].g, &colours_[index].b);
    dirty_ = true;
}

void LedDriver::set_all_hsv(float hue, float sat, float val)
{
    // Convert once, then reuse the same RGB value across all LEDs
    uint8_t r, g, b;
    hsv_to_rgb(hue, sat, val, &r, &g, &b);
    set_all(r, g, b);
}

void LedDriver::set_range_hsv(int start, int count, float hue, float sat, float val)
{
    uint8_t r, g, b;
    hsv_to_rgb(hue, sat, val, &r, &g, &b);
    set_range(start, count, r, g, b);
}

// --- Query 

LedDriver::Colour LedDriver::get_one(int index) const
{
    if (index < 0 || index >= (int)colours_.size()) return {0, 0, 0};
    return colours_[index];
}

// True while changes are staged but unsent — set by every set_*(), cleared by show().
bool LedDriver::has_pending_changes() const
{
    return dirty_;
}

void LedDriver::print_status() const
{
    printf("Pending changes: %s\n", dirty_ ? "yes" : "no");
    for (int i = 0; i < (int)colours_.size(); i++) {
        printf("  [%2d]  R=%3u  G=%3u  B=%3u\n",
               i, colours_[i].r, colours_[i].g, colours_[i].b);
    }
}

// --- Control 

void LedDriver::off()
{
    set_all(0, 0, 0);
}

void LedDriver::set_busy_wait(bool enabled)
{
    busy_wait_ = enabled;
}

void LedDriver::show()
{
    write_leds();

    // The WS2812 needs the data line to stay idle for at least 280 µs after the
    // last bit is sent before it will latch the new values. Without this wait,
    // back-to-back show() calls faster than 280 µs can corrupt the display.
    // Skip it only when you know your loop is already slower than 280 µs.
    if (busy_wait_)
        sleep_us(WS2812_RESET_US);

    dirty_ = false;
}
