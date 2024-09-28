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

#include "common.h"

#include <stdarg.h>
#include <stdio.h>

#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"

static char *buffer_;
static bool *is_enabled_;
static bool baudrate_;


void debug_init(uint baudrate, char *buffer, bool *is_enabled) {
    buffer_ = buffer;
    baudrate_ = baudrate;
    is_enabled_ = is_enabled;
    if (*is_enabled_) {
        uart_init(uart0, baudrate);
        uart_set_fifo_enabled(uart0, true);
        gpio_set_function(16, GPIO_FUNC_UART);
    }
    sleep_ms(1000);
}

void debug_reinit(void) {
    if (*is_enabled_) {
        uart_init(uart0, baudrate_);
        uart_set_fifo_enabled(uart0, true);
        gpio_set_function(16, GPIO_FUNC_UART);
    }
}

void debug(const char *format, ...) {
    if (*is_enabled_) {
        va_list args;
        va_start(args, format);
        vsprintf(buffer_, format, args);
        uart_puts(uart0, buffer_);
        va_end(args);
    }
}

void debug_block(const char *format, ...) {
    if (*is_enabled_) {
        va_list args;
        va_start(args, format);
        vsprintf(buffer_, format, args);
        uart_puts(uart0, buffer_);
        uart_tx_wait_blocking(uart0);
        va_end(args);
    }
}

bool debug_is_enabled(void) { return *is_enabled_; }
