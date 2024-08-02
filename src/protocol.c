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

#include "hardware/adc.h"

static uint8_t *buffer_;
static uint send_count_ = 0, channel_center_;
volatile static uint sample_count_ = 0, channel_mask_, channel_factor_[2], ch2_enabled_;
volatile static bool change_channels_ = false;
volatile static command_t command_ = NONE;
static config_t *config_;
volatile static oscilloscope_config_t *oscilloscope_config_;
volatile static bool send_cal_ = false;
static inline bool send_bulk(uint8_t *buffer);

void protocol_init(config_t *config, volatile oscilloscope_config_t *oscilloscope_config) {
    config_ = config;
    oscilloscope_config_ = oscilloscope_config;
}

void protocol_task(void) {
    switch (command_) {
        case GET_CALIBRATION: {
            uint8_t buffer[255], offset, gain, gainf;
            calibration_t calibration;
            // if (!config_->circuit_is_enabled) {
            offset = 127;
            gain = 1;
            gainf = 0;
            //}
            /*for (uint channel = 0; channel < 2; channel++) {
                for (uint i = 0; i < 8; i++) {
                    calibration.offset_hs[i][channel] = offset;
                    calibration.offset_ls[i][channel] = offset;
                    calibration.gain[i][channel] = gain;
                    calibration.gainf_hs[i][channel] = gainf;
                    calibration.gainf_ls[i][channel] = gainf;
                }
            }
            memcpy(buffer, &calibration, sizeof(calibration_t));*/

            for (uint i = 0; i < 255; i++) buffer[i] = i;
            // tud_control_xfer(0, request_, (void *)(uintptr_t)&buffer, 64);
            send_cal_ = false;

            // send_bulk2(buffer, 64);
            // send_bulk2(buffer + 64, 64);
            command_ = NONE;
            // debug("\nSend calibration. Offset: %u Gain: %.4f (%u %u)", offset, gain + gainf / 250.0, gain, gainf);
            break;
        }
        case SET_SAMPLERATE:
            if (oscilloscope_config_->channel_mask == 0b01)
                oscilloscope_set_samplerate(oscilloscope_config_->samplerate);
            else
                oscilloscope_set_samplerate(oscilloscope_config_->samplerate * 2);
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
            if (oscilloscope_config_->channel_mask == 0b01)
                oscilloscope_set_samplerate(oscilloscope_config_->samplerate);
            else
                oscilloscope_set_samplerate(oscilloscope_config_->samplerate * 2);
            command_ = NONE;
            break;
        case SET_CALIBRATION_FREQ:
            oscilloscope_set_calibration_frequency(oscilloscope_config_->calibration_freq);
            command_ = NONE;
            break;
        case SET_COUPLING:
            oscilloscope_set_coupling(CHANNEL1, oscilloscope_config_->coupling[CHANNEL1]);
            oscilloscope_set_coupling(CHANNEL2, oscilloscope_config_->coupling[CHANNEL2]);
            command_ = NONE;
            break;
        case SET_GAIN_CH1:
            command_ = NONE;
            break;
        case SET_GAIN_CH2:
            command_ = NONE;
            break;
    }

    if (sample_count_ - send_count_ >= 64) {
        if (send_bulk(buffer_ + (send_count_ % BUFFER_SIZE))) send_count_ += 64;
    }
}

void protocol_stop(void) {
    sample_count_ = 0;
    send_count_ = 0;
}

bool protocol_read_command_handler(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request) {
    static uint8_t data[128];
    if (stage == CONTROL_STAGE_SETUP) {
        if (request->bmRequestType_bit.type) {
            // Here we can set the data buffer if direction is to the host (IN)
            return tud_control_xfer(rhport, request, (void *)(uintptr_t)&data, request->wLength);
        }
        return false;
    } else if (stage == CONTROL_STAGE_DATA) {
        switch (request->bRequest) {
            case 0xA2:  // get calibration
            {
                // command_ = GET_CALIBRATION;
                debug("\nCommand get calibration.");
                break;
            }
            case 0xE0:  // set gain ch1
            {
                uint16_t value = data[0] | (data[1] << 8);
                switch (value) {
                    case 0x0701:  // 5V. Gain 1
                    case 0x0601:  // 2V
                    case 0x0501:  // 1V
                        oscilloscope_config_->ch_gain[CHANNEL1] = 1;
                        break;
                    case 0x0402:  // 500mV. Gain 2
                        oscilloscope_config_->ch_gain[CHANNEL1] = 2;
                        break;
                    case 0x0305:  // 200mV. Gain 5
                        oscilloscope_config_->ch_gain[CHANNEL1] = 5;
                        break;
                    case 0x020a:  // 100mV. Gain 10
                    case 0x010a:  // 50mV
                    case 0x000a:  // 20mV
                        oscilloscope_config_->ch_gain[CHANNEL1] = 10;
                        break;
                }
                // To use full range (0-255), signal center and step down is done in openhantek with calibration file
                channel_factor_[CHANNEL1] = 16 / oscilloscope_config_->ch_gain[CHANNEL1];
                debug("\nCommand set gain ch1 (0x%X): %u", value, oscilloscope_config_->ch_gain[CHANNEL1]);
                break;
            }
            case 0xE1:  // set gain ch2
            {
                uint16_t value = data[0] | (data[1] << 8);
                switch (value) {
                    case 0x0701:  // 5V. Gain 1
                    case 0x0601:  // 2V
                    case 0x0501:  // 1V
                        oscilloscope_config_->ch_gain[CHANNEL2] = 1;
                        break;
                    case 0x0402:  // 500mV. Gain 2
                        oscilloscope_config_->ch_gain[CHANNEL2] = 2;
                        break;
                    case 0x0305:  // 200mV. Gain 5
                        oscilloscope_config_->ch_gain[CHANNEL2] = 5;
                        break;
                    case 0x020a:  // 100mV. Gain 10
                    case 0x010a:  // 50mV
                    case 0x000a:  // 20mV
                        oscilloscope_config_->ch_gain[CHANNEL2] = 10;
                        break;
                }
                // To use full range (0-255), signal center and step down is done in openhantek with calibration file
                channel_factor_[CHANNEL2] = 16 / oscilloscope_config_->ch_gain[CHANNEL2];
                debug("\nCommand set gain ch2 (0x%X): %u", value, oscilloscope_config_->ch_gain[CHANNEL2]);
                break;
            }
            case 0xE2:  // set samplerate
            {
                uint16_t value = data[0] | (data[1] << 8);
                switch (value) {
                    case 0x0365:
                    case 0x0065:  // 10 downsampling
                        oscilloscope_config_->samplerate = 10e3;
                        break;
                    case 0x0446:
                    case 0x0166:  // 10 downsampling
                        oscilloscope_config_->samplerate = 20e3;
                        break;
                    case 0x0569:
                    case 0x0269:  // 10 downsampling
                        oscilloscope_config_->samplerate = 50e3;
                        break;
                    case 0x066E:
                        oscilloscope_config_->samplerate = 100e3;
                        break;
                    case 0x0778:
                        oscilloscope_config_->samplerate = 200e3;
                        break;
                    case 0x88C:
                        oscilloscope_config_->samplerate = 400e3;
                        break;
                    case 0x0896:
                        oscilloscope_config_->samplerate = 500e3;
                        break;
                    case 0x0901:
                        oscilloscope_config_->samplerate = 1e6;
                        break;
                    case 0x0A02:
                        oscilloscope_config_->samplerate = 2e6;
                        break;
                }
                command_ = SET_SAMPLERATE;
                debug("\nCommand samplerate: 0x%X", value);
                break;
            }
            case 0xE3:  // start/stop sampling
            {
                if (data[0] == 1) {
                    if (oscilloscope_state() == IDLE) {
                        command_ = START;
                        debug("\nCommand start sampling");
                    }
                } else {
                    if (oscilloscope_state() == RUNNING) {
                        command_ = STOP;
                        debug("\nCommand stop sampling");
                    }
                }
                break;
            }
            case 0xE4:  // set channels
            {
                if (data[0] == 1) {
                    oscilloscope_config_->channel_mask = 0b01;
                    channel_mask_ = 0b01;
                    ch2_enabled_ = false;
                    command_ = SET_CHANNELS;
                } else if (data[0] == 2) {
                    oscilloscope_config_->channel_mask = 0b11;
                    channel_mask_ = 0b11;
                    ch2_enabled_ = true;
                    command_ = SET_CHANNELS;
                }
                change_channels_ = true;
                debug("\nCommand set channel mask: %u", oscilloscope_config_->channel_mask);
                break;
            }
            case 0xE5:  // set coupling
            {
                if (data[0] & 0b1)
                    oscilloscope_config_->coupling[CHANNEL1] = COUPLING_DC;
                else
                    oscilloscope_config_->coupling[CHANNEL1] = COUPLING_AC;
                if (data[1] & 0b1)
                    oscilloscope_config_->coupling[CHANNEL2] = COUPLING_DC;
                else
                    oscilloscope_config_->coupling[CHANNEL2] = COUPLING_AC;
                command_ = SET_COUPLING;
                debug("\nCommand set coupling. Channel 1: %s Channel 2: %s",
                      oscilloscope_config_->coupling[CHANNEL1] == COUPLING_DC ? "DC" : "AC",
                      oscilloscope_config_->coupling[CHANNEL2] == COUPLING_DC ? "DC" : "AC");
                break;
            }
            case 0xE6:  // set calibration frequency
            {
                if (data[0] == 0)
                    oscilloscope_config_->calibration_freq = 100;
                else if (data[0] <= 100)
                    oscilloscope_config_->calibration_freq = data[0] * 1000;
                else if (data[0] <= 200)
                    oscilloscope_config_->calibration_freq = (data[0] - 100) * 10;
                else
                    oscilloscope_config_->calibration_freq = (data[0] - 200) * 100;
                command_ = SET_CALIBRATION_FREQ;
                debug("\nCommand set calibration frequency (%u): %u Hz", data[0],
                      oscilloscope_config_->calibration_freq);
                break;
            }
        }
        return true;
    }
}

void protocol_sample_handler(void) {
    uint rawsample = adc_hw->result;
    uint buffer_pos = sample_count_ % BUFFER_SIZE;
    uint channel = (sample_count_ % 2) * ch2_enabled_;
    uint value = rawsample / channel_factor_[channel];  // + channel_center_;
    if (value > 0xFF) value = 0xFF;
    *(buffer_ + buffer_pos) = value;
    sample_count_++;
}

void protocol_complete_handler(void) {
    if (config_->no_conversion) sample_count_ += 64;
    if (change_channels_) {
        if (channel_mask_ == 0b01)
            adc_hw->cs = 0x1000B;
        else
            adc_hw->cs = 0x3000B;
        change_channels_ = false;
    }
}

void protocol_set_buffer(uint8_t *buffer) { buffer_ = buffer; }

static inline bool send_bulk(uint8_t *buffer) {
    uint lenght = tud_vendor_n_write(0, buffer, 64);
    if (lenght != 64) {
        // debug("\nPacket error: %u", lenght);
        return false;
    }
    return true;
}
