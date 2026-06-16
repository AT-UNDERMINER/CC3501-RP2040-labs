#include "microphone.h"
#include "board.h"

#include "pico/stdlib.h"
#include "hardware/adc.h"

void microphone_init()
{
    adc_init();
    adc_gpio_init(MIC_ADC_PIN);
    adc_select_input(MIC_ADC_CHANNEL);

    // Enable the FIFO with no DMA, raise data-ready on every sample, and keep
    // the full 12-bit result (no error bit packed in, no byte shifting).
    adc_fifo_setup(true,   // en          — enable the FIFO
                   false,  // dreq_en     — no DMA
                   1,      // dreq_thresh
                   false,  // err_in_fifo
                   false); // byte_shift

    // 48 MHz ADC clock divided down to the target sample rate. The RP2040
    // produces one sample every (clkdiv + 1) cycles, hence the "- 1".
    float clkdiv = (48000000.0f / 44100.0f) - 1.0f;
    adc_set_clkdiv(clkdiv);

    // Free-running mode is left off until microphone_read() is called.
}

void microphone_read(uint16_t *buffer, uint16_t num_samples)
{
    adc_run(true);

    for (uint16_t i = 0; i < num_samples; i++) {
        buffer[i] = adc_fifo_get_blocking();
    }

    adc_run(false);

    // Discard any samples captured after our block so the next read starts
    // from a clean FIFO. is_empty() is checked first so we never block here.
    while (!adc_fifo_is_empty()) {
        adc_fifo_get_blocking();
    }
}
