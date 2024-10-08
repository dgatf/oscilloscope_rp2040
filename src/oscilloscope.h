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

#ifndef OSCILLOSCOPE
#define OSCILLOSCOPE

#ifdef __cplusplus
extern "C" {
#endif

#include "common.h"

#define BUFFER_RING_BITS 15
#define BUFFER_SIZE (1 << BUFFER_RING_BITS)  // 1024 bytes

typedef enum state_t { IDLE, RUNNING } state_t;

typedef void (*complete_handler_t)(void);

void oscilloscope_init(void);
void oscilloscope_start(void);
void oscilloscope_stop(void);
void oscilloscope_task(void);
state_t oscilloscope_state(void);
void oscilloscope_set_samplerate(uint samplerate);
void oscilloscope_set_channels(uint8_t mask);
void oscilloscope_set_calibration_frequency(uint freq);
void oscilloscope_set_coupling(channel_t channel, coupling_t coupling);

#ifdef __cplusplus
}
#endif

#endif
