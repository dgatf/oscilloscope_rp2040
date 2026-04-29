/*
 * Oscilloscope RP2040
 * Copyright (C) 2024-2026 Daniel Gorbea <danielgorbea@hotmail.com>
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

#include "common.h"

#include <stdarg.h>
#include <stdio.h>

#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"

static char *s_buffer_debug;
static bool *s_is_enabled;
static uint s_baudrate;

void debug_init(uint baudrate, char *buffer, bool *is_enabled) {
    s_buffer_debug = buffer;
    s_baudrate = baudrate;
    s_is_enabled = is_enabled;
    if (*s_is_enabled) {
        uart_init(uart0, baudrate);
        uart_set_fifo_enabled(uart0, true);
        gpio_set_function(16, GPIO_FUNC_UART);
    }
    sleep_ms(1000);
}

void debug_reinit(void) {
    if (*s_is_enabled) {
        uart_init(uart0, s_baudrate);
        uart_set_fifo_enabled(uart0, true);
        gpio_set_function(16, GPIO_FUNC_UART);
    }
}

void debug(const char *format, ...) {
    if (*s_is_enabled) {
        va_list args;
        va_start(args, format);
        vsprintf(s_buffer_debug, format, args);
        uart_puts(uart0, s_buffer_debug);
        va_end(args);
    }
}

void debug_block(const char *format, ...) {
    if (*s_is_enabled) {
        va_list args;
        va_start(args, format);
        vsprintf(s_buffer_debug, format, args);
        uart_puts(uart0, s_buffer_debug);
        uart_tx_wait_blocking(uart0);
        va_end(args);
    }
}

bool debug_is_enabled(void) { return *s_is_enabled; }
