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

#include "bsp/board.h"
#include "common.h"
#include "oscilloscope.h"

config_t config_;
char debug_message_[DEBUG_BUFFER_SIZE];

void set_pin_config(void);

int main()
{
    if (clock_get_hz(clk_sys) != 200000000)
        set_sys_clock_khz(200000, true);
    volatile uint32_t *reg = (volatile uint32_t *)(CLOCKS_BASE + CLOCKS_CLK_ADC_CTRL_OFFSET); // CLK_ADC_CTRL
    *reg = 0x820;                                                                             // clk_sys, enable

    set_pin_config();

    tud_init(BOARD_TUD_RHPORT);
    board_init();

    // debug init
    debug_init(115200, debug_message_, &config_.debug_is_enabled);
    if (config_.debug_is_enabled)
        sleep_ms(1000);
    debug("\n\n%s - v%i.%i", DEVICE_NAME, VERSION_MAYOR, VERSION_MINOR);

    // led blink
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
    sleep_ms(500);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);

    oscilloscope_init();

    while (true)
    {
        tud_task();
        oscilloscope_task();
    }
}

void set_pin_config(void)
{
    /*
     *   Connect GPIO to GND at boot to select/enable:
     *   - GPIO 18: debug enabled
     *
     *   Defaults (option not grounded):
     *   - Debug disabled
     */

    // configure pins
    gpio_init_mask(GPIO_DEBUG_ENABLE);
    gpio_set_dir_masked(GPIO_DEBUG_ENABLE, false);
    gpio_pull_up(GPIO_DEBUG_ENABLE);
    sleep_ms(1); // wait for pullup

    // set default config
    config_.debug_is_enabled = false;

    // read pin config
    if (gpio_get(GPIO_DEBUG_ENABLE) == 0)
        config_.debug_is_enabled = true;
}
