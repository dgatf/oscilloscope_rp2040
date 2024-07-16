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

static uint8_t *buffer_;
static uint send_count_ = 0;
volatile static uint sample_count_ = 0, channel_mask_;
volatile static bool change_channels_ = false;
volatile static command_t command_ = NONE;

// transform = i / 2 * 33 / 50 + 127
static const uint8_t transform_[] = {127, 127, 127, 127, 128, 128, 128, 128, 129, 129, 130, 130, 130, 130, 131, 131,
                                     132, 132, 132, 132, 133, 133, 134, 134, 134, 134, 135, 135, 136, 136, 136, 136,
                                     137, 137, 138, 138, 138, 138, 139, 139, 140, 140, 140, 140, 141, 141, 142, 142,
                                     142, 142, 143, 143, 144, 144, 144, 144, 145, 145, 146, 146, 146, 146, 147, 147,
                                     148, 148, 148, 148, 149, 149, 150, 150, 150, 150, 151, 151, 152, 152, 152, 152,
                                     153, 153, 154, 154, 154, 154, 155, 155, 156, 156, 156, 156, 157, 157, 158, 158,
                                     158, 158, 159, 159, 160, 160, 160, 160, 161, 161, 161, 161, 162, 162, 163, 163,
                                     163, 163, 164, 164, 165, 165, 165, 165, 166, 166, 167, 167, 167, 167, 168, 168,
                                     169, 169, 169, 169, 170, 170, 171, 171, 171, 171, 172, 172, 173, 173, 173, 173,
                                     174, 174, 175, 175, 175, 175, 176, 176, 177, 177, 177, 177, 178, 178, 179, 179,
                                     179, 179, 180, 180, 181, 181, 181, 181, 182, 182, 183, 183, 183, 183, 184, 184,
                                     185, 185, 185, 185, 186, 186, 187, 187, 187, 187, 188, 188, 189, 189, 189, 189,
                                     190, 190, 191, 191, 191, 191, 192, 192, 193, 193, 193, 193, 194, 194, 194, 194,
                                     195, 195, 196, 196, 196, 196, 197, 197, 198, 198, 198, 198, 199, 199, 200, 200,
                                     200, 200, 201, 201, 202, 202, 202, 202, 203, 203, 204, 204, 204, 204, 205, 205,
                                     206, 206, 206, 206, 207, 207, 208, 208, 208, 208, 209, 209, 210, 210, 210, 210};

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
            uint gain = 0;
            switch (value)
            {
            case 0x0701:
                gain = 5000;
                break;
            case 0x0601:
                gain = 2000;
                break;
            case 0x0501:
                gain = 1000;
                break;
            case 0x0402:
                gain = 500;
                break;
            case 0x0305:
                gain = 200;
                break;
            case 0x020a:
                gain = 100;
                break;
            case 0x010a:
                gain = 50;
                break;
            case 0x000a:
                gain = 20;
                break;
            }
            debug("\nCommand set gain ch1 (0x%X): %u", value, gain);
            break;
        }
        case 0xE1: // set gain ch2
        {
            uint16_t value = data[0] | (data[1] << 8);
            uint gain = 0;
            switch (value)
            {
            case 0x0701:
                gain = 5000;
                break;
            case 0x0601:
                gain = 2000;
                break;
            case 0x0501:
                gain = 1000;
                break;
            case 0x0402:
                gain = 500;
                break;
            case 0x0305:
                gain = 200;
                break;
            case 0x020a:
                gain = 100;
                break;
            case 0x010a:
                gain = 50;
                break;
            case 0x000a:
                gain = 20;
                break;
            }
            debug("\nCommand set gain ch2 (0x%X): %u", value, gain);
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
                command_ = SET_CHANNELS;
            }
            else if (data[0] == 2)
            {
                config->channel_mask = 0b11;
                channel_mask_ = 0b11;
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

void protocol_sample_handler(void)
{
    uint buffer_pos = sample_count_ % BUFFER_SIZE;
    *(buffer_ + buffer_pos) = transform_[*(buffer_ + buffer_pos)];
    sample_count_++;
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

void protocol_complete_handler(void)
{
    if (change_channels_)
    {
        if (channel_mask_ == 0b01)
        {
            adc_hw->cs = 0x1000B;
        }
        else
        {
            adc_hw->cs = 0x3000B;
        }
        change_channels_ = false;
    }
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
