/*
 * Oscilloscope RP2040
 * Copyright (C) 2024 Daniel Gorbea <danielgorbea@hotmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "oscilloscope.h"

#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include "protocol.h"


#define GPIO_COUPLING_CH1_DC 20
#define GPIO_COUPLING_CH2_DC 21
#define GPIO_CALIBRATION 22

extern config_t config_;
extern volatile oscilloscope_config_t oscilloscope_config_;
extern char debug_message_[DEBUG_BUFFER_SIZE];

static const uint dma_channel_adc_ = 0, dma_channel_reload_adc_counter_ = 1, reload_counter_ = BULK_SIZE;
static uint slice_num_;
static state_t state_ = IDLE;
static uint8_t buffer_[BUFFER_SIZE] __attribute__((aligned(BUFFER_SIZE * sizeof(uint8_t)))) = {0};

static void (*handler_)(void) = NULL;

static inline void complete_handler(void);

void oscilloscope_init(void) {
    // init pins
    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_gpio_init(26);
    adc_gpio_init(27);
    adc_gpio_init(28);
    
    protocol_init(buffer_);

    // calibration pwm
    oscilloscope_config_.calibration_freq = 1000;
    gpio_set_function(GPIO_CALIBRATION, GPIO_FUNC_PWM);
    slice_num_ = pwm_gpio_to_slice_num(GPIO_CALIBRATION);
}

void oscilloscope_start(void) {
    state_ = RUNNING;

    // adc setup
    adc_run(false);
    adc_fifo_drain();
    if (config_.no_conversion) {
        adc_fifo_setup(true,   // write to FIFO
                       true,   // enable DMA DREQ
                       1,      // assert DREQ (and IRQ) at least 1 sample present
                       false,  // omit ERR bit (bit 15) since we have 8 bit reads.
                       true    // shift each sample to 8 bits when pushing to FIFO
        );
    } else {
        adc_fifo_setup(true,   // write to FIFO
                       true,   // enable DMA DREQ
                       1,      // assert DREQ (and IRQ) at least 1 sample present
                       false,  // omit ERR bit (bit 15) since we have 8 bit reads.
                       false   // 12 bit resolution
        );
    }

    oscilloscope_set_samplerate(oscilloscope_config_.samplerate * oscilloscope_config_.channel_mask);

    // dma channel adc reload counter
    dma_channel_config config_dma_channel_reload_adc_counter =
        dma_channel_get_default_config(dma_channel_reload_adc_counter_);
    channel_config_set_transfer_data_size(&config_dma_channel_reload_adc_counter, DMA_SIZE_32);
    channel_config_set_write_increment(&config_dma_channel_reload_adc_counter, false);
    channel_config_set_read_increment(&config_dma_channel_reload_adc_counter, false);
    dma_channel_configure(dma_channel_reload_adc_counter_, &config_dma_channel_reload_adc_counter,
                          &dma_hw->ch[dma_channel_adc_].al1_transfer_count_trig,  // write address
                          &reload_counter_,                                       // read address
                          1, false);

    // dma channel adc
    dma_channel_config channel_config_adc = dma_channel_get_default_config(dma_channel_adc_);
    if (config_.no_conversion)
        channel_config_set_transfer_data_size(&channel_config_adc, DMA_SIZE_8);
    else
        channel_config_set_transfer_data_size(&channel_config_adc, DMA_SIZE_16);
    channel_config_set_ring(&channel_config_adc, true, BUFFER_RING_BITS);
    channel_config_set_write_increment(&channel_config_adc, true);
    channel_config_set_read_increment(&channel_config_adc, false);
    channel_config_set_dreq(&channel_config_adc, DREQ_ADC);
    channel_config_set_chain_to(&channel_config_adc,
                                dma_channel_reload_adc_counter_);  // reload counter when completed
    dma_channel_set_irq0_enabled(dma_channel_adc_, true);          // raise an interrupt when completed
    irq_set_exclusive_handler(DMA_IRQ_0, complete_handler);
    irq_set_enabled(DMA_IRQ_0, true);
    dma_channel_configure(dma_channel_adc_, &channel_config_adc,
                          &buffer_,       // write address
                          &adc_hw->fifo,  // read address
                          BULK_SIZE, true);

    oscilloscope_set_channels(oscilloscope_config_.channel_mask);
    adc_run(true);

    gpio_put(PICO_DEFAULT_LED_PIN, 1);
}

void oscilloscope_stop(void) {
    dma_channel_abort(dma_channel_adc_);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);
    state_ = IDLE;
    debug("\nOscilloscope stop");
}

void oscilloscope_task(void) { protocol_task(); }

state_t oscilloscope_state(void) { return state_; }

void oscilloscope_set_samplerate(uint samplerate) {
    if (!samplerate) {
        oscilloscope_config_.samplerate = 100000;
        samplerate = 100000;
    }
    float clk_div = clock_get_hz(clk_sys) / samplerate;
    debug("\nSet samplerate (clk div: %.2f): %u", clk_div, samplerate);
    adc_set_clkdiv(clk_div);
}

void oscilloscope_set_channels(uint8_t mask) {
    adc_set_round_robin(mask);  // GPIO 26-28
}

void oscilloscope_set_calibration_frequency(uint freq) {
    if (freq == 0) freq = 100;  // 100 Hz
    oscilloscope_config_.calibration_freq = freq;
    float clk_div = clock_get_hz(clk_sys) / freq / 65536.0;
    pwm_config config = pwm_get_default_config();
    uint16_t wrap;
    if (clk_div < 1) {
        clk_div = 1;
        wrap = clock_get_hz(clk_sys) / freq;
        pwm_config_set_wrap(&config, wrap);
    } else {
        wrap = 0xFFFF;
        pwm_config_set_wrap(&config, wrap);
    }
    pwm_config_set_clkdiv(&config, clk_div);
    pwm_init(slice_num_, &config, true);
    pwm_set_gpio_level(GPIO_CALIBRATION, wrap / 2);

    debug("\nSet calibration frequency: %u Hz Clk div: %.2f Wrap: %u", freq, clk_div, wrap);
}

void oscilloscope_set_coupling(channel_t channel, coupling_t coupling) {
    if (channel == CHANNEL1) {
        if (coupling == COUPLING_DC)
            gpio_put(GPIO_COUPLING_CH1_DC, true);
        else
            gpio_put(GPIO_COUPLING_CH1_DC, false);
    } else {
        if (coupling == COUPLING_DC)
            gpio_put(GPIO_COUPLING_CH2_DC, true);
        else
            gpio_put(GPIO_COUPLING_CH2_DC, false);
    }
}

static inline void complete_handler(void) {
    protocol_complete_handler();
    dma_hw->ints0 = 1u << dma_channel_adc_;
}