# CC3501 RP2040 Labs — Firmware Documentation

Firmware for the CC3501 custom RP2040 development board. The board carries a
12-LED WS2812B chain in a U-shape, a tactile button (SW1), a LIS3DH 3-axis
accelerometer on I²C, and a microphone amplifier feeding the on-chip ADC.

The firmware boots into an **idle task** and cycles through a fixed list of
demonstration tasks each time SW1 is pressed.

---

## Contents

- [Hardware summary](#hardware-summary)
- [Architecture](#architecture)
- [Build system](#build-system)
- [Module reference](#module-reference)
  - [`main.cpp` — task dispatch and button](#maincpp--task-dispatch-and-button)
  - [`board.h` — board pin map](#boardh--board-pin-map)
  - [`drivers/leds` — WS2812B driver (`LedDriver`)](#driversleds--ws2812b-driver-leddriver)
  - [`drivers/lis3dh` — accelerometer driver](#driverslis3dh--accelerometer-driver)
  - [`drivers/microphone` — ADC driver](#driversmicrophone--adc-driver)
  - [`tasks/idle_task`](#tasksidle_task)
  - [`tasks/led_task`](#tasksled_task)
  - [`tasks/accelerometer_task`](#tasksaccelerometer_task)
  - [`tasks/audio_task`](#tasksaudio_task)
- [Adding a new task](#adding-a-new-task)
- [Conventions](#conventions)

---

## Hardware summary

| Peripheral | Connection | Notes |
|---|---|---|
| WS2812B LEDs | data on **GPIO14**, 12 LEDs, RGB | driven by PIO, not a dedicated peripheral |
| SW1 button | **GPIO15** | external RC filter (100 kΩ / 100 nF, τ = 10 ms) and external pull-down |
| LIS3DH accelerometer | I²C0, **SDA = GPIO16, SCL = GPIO17** | 7-bit address `0x19`, external 4.7 kΩ pull-ups |
| Microphone | ADC channel 0, **GPIO26** | 12-bit samples, ~mid-scale DC bias |

Pin assignments live in [`board.h`](#boardh--board-pin-map). Standard I/O is
routed to **UART** (USB stdio disabled) — see the build configuration.

---

## Architecture

The firmware is a **cooperative single-loop scheduler**. There is no RTOS; a
flat `for(;;)` loop in `main()` repeatedly does three things:

1. Run one *frame* of the current task.
2. Poll SW1 for a press.
3. On a press, exit the current task and advance to the next.

### The task contract

Every task is a pair of plain functions that take the shared LED driver by
reference and return nothing:

```cpp
void run_<name>_task(LedDriver &leds);   // called once per loop iteration ("one frame")
void exit_<name>_task(LedDriver &leds);  // called once when leaving the task
```

`main()` constructs a **single `LedDriver`** once, before the task loop, and
passes it into every `run_*`/`exit_*` call. Task files do not construct their
own driver instance (the old per-file `get_leds()` lazy-singleton pattern is
gone), so the PIO program is loaded exactly once for the whole program.

`run_*` must be **non-blocking** — it does a small slice of work and returns
immediately so the main loop can keep polling the button. State that must
survive between frames is held in `static` local variables inside the task.
`exit_*` performs cleanup (turn LEDs off, power down a peripheral, reset the
task's one-time-init flag).

### Task dispatch

Tasks are stored in two parallel arrays of **function pointers** in `main.cpp`:

```cpp
static void (*const task_run[])(LedDriver&)  = { run_idle_task, run_led_task,
                                                 run_accelerometer_task, run_audio_task };
static void (*const task_exit[])(LedDriver&) = { exit_idle_task, exit_led_task,
                                                 exit_accelerometer_task, exit_audio_task };
```

`current_task` indexes both arrays; each call passes the shared driver
(`task_run[current_task](leds)`). Advancing wraps with modulo
(`(current_task + 1) % NUM_TASKS`). The task order is the array order:
**idle → led → accelerometer → audio → idle → …**

### Driver/task split

- **Drivers** (`src/drivers/`) own the hardware. They convert raw registers
  and protocols into a clean API (the *adapter pattern*). Drivers contain no
  task logic.
- **Tasks** (`src/tasks/`) own the behaviour. They call drivers and decide what
  to display. Tasks contain no register-level code.
- **Button handling and task switching live in `main.cpp`**, deliberately not
  in a driver.

Two driver styles are used on purpose:

| Driver | Style | Why |
|---|---|---|
| `LedDriver` | **C++ class** | holds per-instance state (the colour buffer); supports a clean staged API |
| `lis3dh` | **C-style free functions** taking an `i2c_inst_t*` | stateless; the bus is passed in each call |

---

## Build system

`CMakeLists.txt` builds **two different targets** depending on the active
compiler kit (selectable from the VS Code status bar):

- **ARM cross-compiler** (`arm-none-eabi-*`) → builds firmware for the RP2040.
- **Host compiler** → builds a native **test harness** that compiles a subset
  of the code (LED driver + a couple of tasks) against mock hardware in
  `tests/mocks/`, so logic can be unit-tested off-target.

Key points of the firmware build:

- C standard 11, C++ standard 17.
- Pulls in the Pico SDK (`pico_sdk_import.cmake`, `pico_sdk_init()`).
- Configures **CMSIS-DSP** with only the needed components enabled
  (`TRANSFORM`, `BASICMATH`, `COMPLEXMATH`) and the hard-coded
  `RFFT_Q15_1024` constants for the 1024-point real FFT.
- Generates the WS2812 PIO header at build time
  (`pico_generate_pio_header`).
- Routes stdio to UART (`pico_enable_stdio_uart(labs 1)`), USB off.
- Links the hardware libraries (`hardware_i2c`, `hardware_pio`,
  `hardware_adc`, …) and `CMSISDSP`.

> **When you add a `.cpp` file, add it to `target_sources(labs …)`** in the
> correct branch (the cross-compile branch for firmware-only code, or both
> branches if the test harness needs it). Missing this is the most common
> "my code won't build / link" cause.

---

## Module reference

### `main.cpp` — task dispatch and button

Responsibilities: initialise stdio and SW1, run the scheduler loop, debounce
and detect button presses, and switch tasks.

- **Startup:** `stdio_init_all()`, then `gpio_init(SW1_PIN)`,
  direction = input, internal pulls disabled
  (`gpio_set_pulls(SW1_PIN, false, false)`) because an external pull-down is
  fitted.
- **Per iteration:**
  1. `task_run[current_task](leds)` runs one frame.
  2. SW1 is polled. A **rising edge** (`state && !last_state`) starts press
     handling.
  3. The code waits `20 ms` (≈ 2 RC time constants) and re-reads the pin to
     confirm a real press, not a glitch. The RC filter has already removed
     sub-millisecond noise; this confirms the level has settled.
  4. It then **blocks until release** (polling every 5 ms) so one physical
     press maps to exactly one logical press.
  5. On a confirmed press, `task_exit[current_task](leds)` runs and
     `current_task` advances modulo `NUM_TASKS`.
- **Loop pacing:** `sleep_ms(5)` at the end sets the polling interval, well
  above the RC settling time.

**Design note — polling, not interrupts.** Because SW1 already has hardware RC
debouncing and the loop is fast enough, polling is simpler and sufficient; an
interrupt + handler would add complexity with no benefit here.

### `board.h` — board pin map

Single source of truth for board-specific constants:

```c
#define SW1_PIN            15
#define BOARD_LED_PIN      14
#define BOARD_LED_COUNT    12
#define BOARD_LED_IS_RGBW  false
#define ACCEL_I2C_INSTANCE i2c0
#define ACCEL_SDA_PIN      16
#define ACCEL_SCL_PIN      17
#define MIC_ADC_PIN        26
#define MIC_ADC_CHANNEL    0
```

Change these (not the driver/task source) when the hardware layout changes.

### `drivers/leds` — WS2812B driver (`LedDriver`)

A C++ class that drives the WS2812B chain over **PIO**. It uses a
**stage-then-commit** model: `set_*()` calls update an in-RAM buffer only;
`show()` sends the whole buffer to the hardware.

**Public API (header):**

| Method | Purpose |
|---|---|
| `LedDriver(int num_leds)` | allocate buffer, load + start the PIO state machine |
| `set_count` / `get_count` | resize at runtime / query LED count |
| `set_all` / `set_one` / `set_range` / `set_many` | stage RGB |
| `set_all_hsv` / `set_one_hsv` / `set_range_hsv` / `set_many_hsv` | stage HSV |
| `get_one` | read back a staged colour |
| `has_pending_changes` | true if there are unsent (`dirty`) changes |
| `print_status` | dump every LED + dirty flag to stdout |
| `off` | stage all LEDs off |
| `show` | send the buffer to the hardware |
| `set_busy_wait` | enable/disable the post-`show()` reset delay |

**Internals:**

- **Colour buffer:** `std::vector<Colour>`, one `{r,g,b}` per LED. A `dirty_`
  flag tracks unsent changes; set by every `set_*`, cleared by `show()`.
- **`write_leds()`:** packs each colour into the top 24 bits in **GRB** order
  (`g<<24 | r<<16 | b<<8`) and pushes it to the PIO TX FIFO with
  `pio_sm_put_blocking`. The whole chain is always sent because WS2812B LEDs
  latch simultaneously when the line goes idle.
- **`hsv_to_rgb()`:** standard 6-sector colour-wheel conversion
  (hue 0–360, sat/val 0–1).
- **Reset pulse:** `show()` optionally busy-waits `300 µs` (spec min 280 µs)
  so the LEDs latch. Disable with `set_busy_wait(false)` when the caller's
  own loop already guarantees > 280 µs between frames (all tasks do this).
- Copy construction/assignment are **deleted** — a hardware driver must not be
  duplicated.

### `drivers/lis3dh` — accelerometer driver

A C-style driver for the ST LIS3DH over I²C. Functions take the I²C instance
as their first argument.

**Public API (header):**

```cpp
bool  lis3dh_init(i2c_inst_t *i2c, lis3dh_odr_t start_odr);          // bring up + verify + configure
bool  lis3dh_set_odr(i2c_inst_t *i2c, lis3dh_odr_t odr);             // change ODR at runtime
bool  lis3dh_power_down(i2c_inst_t *i2c);                            // ODR = 0000 → power-down mode
void  lis3dh_read_raw(i2c_inst_t *i2c, int16_t *x, *y, *z);          // one 3-axis sample (12-bit signed)
float lis3dh_to_g(int16_t raw);                                      // raw → g
```

**Configuration (set in the driver / header):**

- Bus: the I²C instance and SDA/SCL pins live in `board.h` (`ACCEL_I2C_INSTANCE`,
  `ACCEL_SDA_PIN`, `ACCEL_SCL_PIN`); the **400 kHz** fast-mode baud stays in the driver.
- Address `0x19` (SA0 high).
- Output data rate is chosen **at runtime**: `lis3dh_init()` takes a starting
  rate from the `lis3dh_odr_t` enum (`POWERDOWN`, `ODR_1HZ` … `ODR_400HZ`,
  pre-shifted CTRL_REG1 values), and `lis3dh_set_odr()` changes it later.
- `CTRL_REG1`: init writes the lower nibble `0x07` (normal mode, X/Y/Z enabled);
  the ODR bits are applied only through `lis3dh_set_odr()`, which
  read-modify-writes the register so the mode/axis bits are preserved.
- `CTRL_REG4 = 0x88` (BDU on, high-resolution 12-bit, ±2 g).

**Key behaviours:**

- **WHO_AM_I check:** reads register `0x0F`, expects `0x33`; aborts init and
  returns `false` if it does not match — never configures an unknown device.
- **Multi-byte burst read:** the sub-address is OR-ed with `0x80`
  (`AUTO_INCREMENT`) so the LIS3DH auto-increments through `OUT_X_L..OUT_Z_H`
  in a single transaction. **This flag is mandatory on the LIS3DH** — without
  it the burst read returns the same register repeatedly.
- **Sample assembly:** data is left-justified in HR mode; each axis is
  combined (`high<<8 | low`), cast to `int16_t`, then arithmetic-shifted
  `>> 4` to a sign-correct 12-bit value.
- **Scaling:** 1 mg/digit at ±2 g HR → `raw * 0.001` g.
- External pull-ups are fitted, so internal pulls are left disabled.

### `drivers/microphone` — ADC driver

Configures the RP2040 ADC to sample the microphone on GPIO26 (channel 0) and
read fixed-size blocks.

**Public API:**

```cpp
void microphone_init();                                   // configure ADC + FIFO, 44.1 kHz
void microphone_read(uint16_t *buffer, uint16_t n);       // capture exactly n samples (blocking)
```

**Behaviour:**

- `adc_init`, `adc_gpio_init(MIC_ADC_PIN)`, `adc_select_input(MIC_ADC_CHANNEL)` —
  pin and channel come from `board.h`.
- FIFO enabled, no DMA, data-ready every sample, full 12-bit result kept.
- Sample rate set via clock divider: `clkdiv = 48 MHz / 44.1 kHz − 1`
  (one sample every `clkdiv + 1` cycles).
- `microphone_read` starts free-running mode, blocks until `n` samples are
  collected, stops the ADC, and **drains any leftover FIFO samples** so the
  next read starts clean.

### `tasks/idle_task`

Default startup task. A slow **blue "breathing"** animation across all 12 LEDs.

- Brightness follows `0.5·(sin(phase)+1)`, scaled by `PEAK_VAL = 0.07`
  (deliberately dim).
- A `static` phase accumulator advances `BREATH_STEP` each frame; one full
  breathe is ~2 s given the 5 ms loop.
- Disables the LED reset busy-wait once (the loop is already slow enough).
- `exit` turns all LEDs off.

### `tasks/led_task`

A cycling **demonstration** of the LED driver API. Six steps, each held for
`FRAMES_PER_STEP = 100` frames, then it loops:

| Step | Demonstrates |
|---|---|
| 0 | `set_one` RGB on three LEDs + `print_status` |
| 1 | `set_one_hsv` hue sweep across the chain |
| 2 | two `set_range` blocks |
| 3 | `set_many` on even LEDs (magenta) |
| 4 | `set_many_hsv` on odd LEDs (cyan) |
| 5 | `print_status`, a log line, then off |

`exit` turns all LEDs off.

### `tasks/accelerometer_task`

A **spirit/bubble level**. Reads the LIS3DH each frame, prints X/Y/Z in g over
serial, and shows tilt on the LED ring.

- One-time `lis3dh_init` at `lis3dh_odr_t::ODR_100HZ` on first run (flag reset
  on exit).
- **Level** (|X| and |Y| < `LEVEL_THRESHOLD = 0.05 g`): all LEDs steady green.
- **Tilted:** the tilt direction is mapped to a single "bubble" LED around the
  U-shape using `atan2f(gx, gy)`; bubble colour is orange below `MAG_RED` and
  red at/above it, so colour encodes tilt magnitude. Special cases handle the
  gap at the front of the U.
- `exit` powers the LIS3DH down via `lis3dh_power_down()`, turns LEDs off, and
  resets the init flag.

> Two earlier lighting approaches are kept commented out in the source as
> alternatives (threshold groups, and brightness-proportional tilt).

### `tasks/audio_task`

A **real-time spectrum analyser**: 12 LEDs show energy in 12 log-spaced
frequency bands.

Pipeline per frame:

1. Capture `FFT_SIZE = 1024` samples (`microphone_read`).
2. Compute the **DC bias** (block mean) and subtract it from each sample.
3. **Left-shift `<< 5`** to push the 12-bit data up into the Q15 range
   (maximise precision).
4. Apply a pre-computed **Hanning window** in Q15 (reduces spectral leakage).
5. `arm_rfft_q15` — forward real FFT (CMSIS-DSP rescales internally; output is
   **not** renormalised).
6. `arm_cmplx_mag_squared_q15` — magnitude² per bin (only `FFT_SIZE/2` bins are
   used; the upper half is the conjugate mirror and carries no new info).
7. Sum each band between `bin_edges[]` and light its LED (red→green→blue across
   the chain) if the band energy crosses `LEVEL_THRESHOLD`.

One-time init builds the Hanning window (float, runs once), starts the mic, and
initialises the FFT instance. `exit` stops the ADC, drains the FIFO, turns LEDs
off, and resets the init flag.

> `LEVEL_THRESHOLD` and the working buffers are tuned for hardware; the buffers
> are at file scope to keep them off the small main stack.

---

## Adding a new task

1. Create `src/tasks/<name>_task.cpp` and `.h` with
   `run_<name>_task(LedDriver &leds)` / `exit_<name>_task(LedDriver &leds)`
   (forward-declare `class LedDriver;` in the header rather than including
   `leds.h`).
2. In `main.cpp`, `#include "tasks/<name>_task.h"` and append the function
   names to `task_run[]` and `task_exit[]` **in the same order**.
3. Add the `.cpp` to `target_sources(labs …)` in `CMakeLists.txt`.

No other changes are needed — `NUM_TASKS` is computed from the array size.

## Conventions

- `run_*` is non-blocking and keeps state in `static` locals.
- One-time hardware init is guarded by a `static bool` flag that `exit_*`
  resets, so re-entering a task re-initialises cleanly.
- Drivers never reach into task logic and vice versa.
- Board-specific constants live only in `board.h`.
- LED writes are staged; nothing reaches the hardware until `show()`.
