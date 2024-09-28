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

#include "hardware/clocks.h"
#include "pico/stdlib.h"
#include "protocol.h"
#include "usb.h"

typedef enum gpio_config_t { GPIO_DEBUG_ENABLE = 18, GPIO_NO_CONVERSION = 19 } gpio_config_t;

config_t config_;
volatile oscilloscope_config_t oscilloscope_config_;
char debug_message_[DEBUG_BUFFER_SIZE];

void set_pin_config(void);

int main() {
    if (clock_get_hz(clk_sys) != 250000000) set_sys_clock_khz(250000, true);
    volatile uint32_t *reg = (volatile uint32_t *)(CLOCKS_BASE + CLOCKS_CLK_ADC_CTRL_OFFSET);  // CLK_ADC_CTRL
    *reg = 0x820;                                                                              // clk_sys, enable
    set_pin_config();

    config_.debug_is_enabled = true; // For some mysterous reason, debug must be enabled with multicore ?

    debug_init(115200, debug_message_, &config_.debug_is_enabled);
    debug("\n\n%s - v%i.%i", DEVICE_NAME, VERSION_MAYOR, VERSION_MINOR);

    usb_device_init();
    while (!usb_is_configured()) {
    }
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
    sleep_ms(500);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);
    oscilloscope_init();

    while (1) {
    }
}

void set_pin_config(void) {
    gpio_init_mask((1 << GPIO_DEBUG_ENABLE) | (1 << GPIO_NO_CONVERSION));
    gpio_set_dir_masked((1 << GPIO_DEBUG_ENABLE) | (1 << GPIO_NO_CONVERSION), false);
    gpio_pull_up(GPIO_DEBUG_ENABLE);
    gpio_pull_up(GPIO_NO_CONVERSION);
    sleep_ms(1);  // wait for pullup
    config_.debug_is_enabled = false;
    config_.no_conversion = false;
    if (!gpio_get(GPIO_DEBUG_ENABLE)) config_.debug_is_enabled = true;
    if (!gpio_get(GPIO_NO_CONVERSION)) config_.no_conversion = true;
}
