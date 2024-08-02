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

#ifndef COMMON
#define COMMON

#ifdef __cplusplus
extern "C" {
#endif

#include "pico/types.h"

// Debug buffer size
#define DEBUG_BUFFER_SIZE 256

// Define board values
#define DEVICE_NAME "RP2040 Oscilloscope"
#define VERSION_MAYOR 0
#define VERSION_MINOR 1

typedef enum coupling_t { COUPLING_DC, COUPLING_AC } coupling_t;

typedef struct calibration_t {
    uint8_t offset_ls[8][2];
    uint8_t offset_hs[8][2];
    uint8_t gain[8][2];
    uint8_t gainf_ls[8][2];
    uint8_t gainf_hs[8][2];
} calibration_t;

typedef struct config_t {
    bool debug_is_enabled;
    bool no_conversion;
} config_t;

typedef struct oscilloscope_config_t {
    uint channel_mask;
    uint samplerate;
    uint calibration_freq;
    uint ch_gain[2];
    coupling_t coupling[2];
} oscilloscope_config_t;

typedef enum channel_t { CHANNEL1, CHANNEL2 } channel_t;

void debug_init(uint baudrate, char *buffer, bool *is_enabled);
void debug_reinit(void);
void debug(const char *format, ...);
void debug_block(const char *format, ...);
bool debug_is_enabled(void);

#ifdef __cplusplus
}
#endif

#endif
