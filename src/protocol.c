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

#include "protocol.h"
#include "tusb_config.h"
#include "bsp/board.h"
#include "string.h"
#include "hardware/adc.h"

static uint8_t *buffer_;
static uint send_count_ = 0;
volatile static uint sample_count_ = 0, channel_mask_, channel_gain_factor_[2], ch2_enabled_;
volatile static bool change_channels_ = false;
volatile static command_t command_ = NONE;

static inline bool send_bulk(uint8_t *buffer);

void protocol_task(volatile oscilloscope_config_t *config)
{
    switch (command_)
    {
    case SET_SAMPLERATE:
        if (config->channel_mask == 0b01)
            oscilloscope_set_samplerate(config->samplerate);
        else
            oscilloscope_set_samplerate(config->samplerate * 2);
        command_ = NONE;
        break;
    case START:
        oscilloscope_start();
        command_ = NONE;
        break;
    case STOP:
        oscilloscope_stop();
        command_ = NONE;
        break;
    case SET_CHANNELS:
        if (config->channel_mask == 0b01)
            oscilloscope_set_samplerate(config->samplerate);
        else
            oscilloscope_set_samplerate(config->samplerate * 2);
        command_ = NONE;
        break;
    case SET_CALIBRATION_FREQ:
        oscilloscope_set_calibration_frequency(config->calibration_freq);
        command_ = NONE;
        break;
    }

    if (sample_count_ - send_count_ >= 64)
    {
        if (send_bulk(buffer_ + (send_count_ % BUFFER_SIZE)))
            send_count_ += 64;
    }
}

bool protocol_read_command_handler(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request, volatile oscilloscope_config_t *config)
{
    static uint8_t data[64];
    if (stage == CONTROL_STAGE_SETUP)
    {
        if (request->bmRequestType_bit.type)
            return tud_control_xfer(rhport, request, (void *)(uintptr_t)&data, request->wLength);
        return false;
    }
    else if (stage == CONTROL_STAGE_DATA)
    {
        switch (request->bRequest)
        {
        case 0xA2: // get calibration
        {
            debug("\nCommad get calibration");
            break;
        }
        case 0xE0: // set gain ch1
        {
            uint16_t value = data[0] | (data[1] << 8);
            switch (value)
            {
            case 0x0701:
                config->ch1_gain = 5000;
                channel_gain_factor_[0] = 50;
                break;
            case 0x0601:
                config->ch1_gain = 2000;
                channel_gain_factor_[0] = 50;
                break;
            case 0x0501:
                config->ch1_gain = 1000;
                channel_gain_factor_[0] = 50;
                break;
            case 0x0402:
                config->ch1_gain = 500;
                channel_gain_factor_[0] = 25;
                break;
            case 0x0305:
                config->ch1_gain = 200;
                channel_gain_factor_[0] = 10;
                break;
            case 0x020a:
                config->ch1_gain = 100;
                channel_gain_factor_[0] = 5;
                break;
            case 0x010a:
                config->ch1_gain = 50;
                channel_gain_factor_[0] = 5;
                break;
            case 0x000a:
                config->ch1_gain = 20;
                channel_gain_factor_[0] = 5;
                break;
            }
            debug("\nCommand set gain ch1 (0x%X): %u", value, channel_gain_factor_[0]);
            break;
        }
        case 0xE1: // set gain ch2
        {
            uint16_t value = data[0] | (data[1] << 8);
            switch (value)
            {
            case 0x0701:
                config->ch2_gain = 5000;
                channel_gain_factor_[1] = 50;
                break;
            case 0x0601:
                config->ch2_gain = 2000;
                channel_gain_factor_[1] = 50;
                break;
            case 0x0501:
                config->ch2_gain = 1000;
                channel_gain_factor_[1] = 50;
                break;
            case 0x0402:
                config->ch2_gain = 500;
                channel_gain_factor_[1] = 25;
                break;
            case 0x0305:
                config->ch2_gain = 200;
                channel_gain_factor_[1] = 10;
                break;
            case 0x020a:
                config->ch2_gain = 100;
                channel_gain_factor_[1] = 5;
                break;
            case 0x010a:
                config->ch2_gain = 50;
                channel_gain_factor_[1] = 5;
                break;
            case 0x000a:
                config->ch2_gain = 20;
                channel_gain_factor_[1] = 5;
                break;
            }
            debug("\nCommand set gain ch2 (0x%X): %u", value, channel_gain_factor_[1]);
            break;
        }
        case 0xE2: // set samplerate
        {
            uint16_t value = data[0] | (data[1] << 8);
            switch (value)
            {
            case 0x0365:
            case 0x0065: // 10 downsampling
                config->samplerate = 10e3;
                break;
            case 0x0446:
            case 0x0166: // 10 downsampling
                config->samplerate = 20e3;
                break;
            case 0x0569:
            case 0x0269: // 10 downsampling
                config->samplerate = 50e3;
                break;
            case 0x066E:
                config->samplerate = 100e3;
                break;
            case 0x0778:
                config->samplerate = 200e3;
                break;
            case 0x88C:
                config->samplerate = 400e3;
                break;
            case 0x0896:
                config->samplerate = 500e3;
                break;
            case 0x0901:
                config->samplerate = 1e6;
                break;
            case 0x0A02:
                config->samplerate = 2e6;
                break;
            }
            command_ = SET_SAMPLERATE;
            debug("\nCommand samplerate: 0x%X", value);
            break;
        }
        case 0xE3: // start/stop sampling
        {
            if (data[0] == 1)
            {
                if (oscilloscope_state() == IDLE)
                {
                    command_ = START;
                    debug("\nCommand start sampling");
                }
            }
            else
            {
                if (oscilloscope_state() == RUNNING)
                {
                    command_ = STOP;
                    debug("\nCommand stop sampling");
                }
            }
            break;
        }
        case 0xE4: // set channels
        {
            if (data[0] == 1)
            {
                config->channel_mask = 0b01;
                channel_mask_ = 0b01;
                ch2_enabled_ = false;
                command_ = SET_CHANNELS;
            }
            else if (data[0] == 2)
            {
                config->channel_mask = 0b11;
                channel_mask_ = 0b11;
                ch2_enabled_ = true;
                command_ = SET_CHANNELS;
            }
            change_channels_ = true;
            debug("\nCommand set channel mask: %u", config->channel_mask);
            break;
        }
        case 0xE5: // set coupling
        {
            debug("\nCommand set coupling. Channel 1: %s Channel 2: %s", data[0] & 0b1 ? "DC" : "AC", data[0] & 0b1 ? "DC" : "AC");
            break;
        }
        case 0xE6: // set calibration frequency
        {
            if (data[0] == 0)
                config->calibration_freq = 100;
            else if (data[0] <= 100)
                config->calibration_freq = data[0] * 1000;
            else if (data[0] <= 200)
                config->calibration_freq = (data[0] - 100) * 10;
            else
                config->calibration_freq = (data[0] - 200) * 100;
            command_ = SET_CALIBRATION_FREQ;
            debug("\nCommand set calibration frequency (%u): %u Hz", data[0], config->calibration_freq);
            break;
        }
        }
        return true;
    }
}

void protocol_sample_handler(uint rawsample)
{
    uint buffer_pos = sample_count_ % BUFFER_SIZE;
    uint channel = (sample_count_ % 2) * ch2_enabled_;
    uint value = rawsample / channel_gain_factor_[channel] + 127;
    if (value > 0xFF)
        value = 0xFF;
    *(buffer_ + buffer_pos) = value;
    sample_count_++;
}

void protocol_complete_handler(void)
{
    if (change_channels_)
    {
        if (channel_mask_ == 0b01)
            adc_hw->cs = 0x1000B;
        else
            adc_hw->cs = 0x3000B;
        change_channels_ = false;
    }
}

void protocol_set_buffer(uint8_t *buffer)
{
    buffer_ = buffer;
}

command_t protocol_read_command(void)
{
    command_t command = command_;
    command_ = NONE;
    return command;
}

void protocol_stop(void)
{
    sample_count_ = 0;
    send_count_ = 0;
}

static inline bool send_bulk(uint8_t *buffer)
{
    uint lenght = tud_vendor_n_write(0, buffer, 64);
    if (lenght != 64)
    {
        // debug("\nPacket error: %u", lenght);
        return false;
    }
    return true;
}
