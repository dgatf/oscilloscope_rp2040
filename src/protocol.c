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

#include <string.h>

#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/regs/usb.h"
#include "hardware/structs/usb.h"
#include "pico/multicore.h"
#include "stdio.h"
#include "usb.h"

#define usb_hw_set ((usb_hw_t *)hw_set_alias_untyped(usb_hw))
#define usb_hw_clear ((usb_hw_t *)hw_clear_alias_untyped(usb_hw))

extern config_t config;
extern volatile oscilloscope_config_t oscilloscope_config;
extern char debug_message[DEBUG_BUFFER_SIZE];

static volatile uint8_t *buffer_adc;
static volatile uint sample_count = 0, channel_factor[2];
static struct usb_endpoint_configuration *ep;
static uint bof = 0;
static uint64_t ts;
static uint buffer_size, queued_count = 0;
static uint8_t *buffer_usb;
static uint buffer_usb_size;

static inline void fill_buffer_usb(volatile uint8_t *buffer_usb, volatile uint8_t *buffer_adc, uint length);

void ep6_in_handler(uint8_t *buf, uint16_t len) {}

static inline void fill_buffer_usb(volatile uint8_t *buffer_usb, volatile uint8_t *buffer_adc, uint length) {
    if (config.no_conversion) {
        for (uint i = 0; i < length; i++) *(buffer_usb + i) = *(buffer_adc + i);
    } else {
        uint ch_gain;
        for (uint i = 0; i < length; i++) {
            if (oscilloscope_config.channel_mask == 0b11 && (i % 2))
                ch_gain = oscilloscope_config.ch_gain[CHANNEL2];
            else
                ch_gain = oscilloscope_config.ch_gain[CHANNEL1];
            uint16_t *buffer_pos = (void *)buffer_adc;
            buffer_pos += i;
            uint16_t value = (*buffer_pos * ch_gain) >> 4;
            if (value > 0xFF) value = 0xFF;
            *(buffer_usb + i) = value;
        }
    }
}

void control_transfer_handler(uint8_t *buf, volatile struct usb_setup_packet *pkt, uint8_t stage) {
    // debug("\nControl transfer. Stage %u bmRequestType 0x%x bRequest 0x%x wValue 0x%x wIndex 0x%x wLength %u", stage,
    // pkt->bmRequestType, pkt->bRequest, pkt->wValue, pkt->wIndex, pkt->wLength);
    if (stage == STAGE_SETUP) {
        if (pkt->bmRequestType & USB_DIR_IN) {
            ;  // Here prepare buffer to send
        }
    } else if (stage == STAGE_DATA) {
        switch (pkt->bRequest) {
            case 0xA2:  // get calibration
            {
                debug("\nCommand get calibration.");
                break;
            }
            case 0xE0:  // set gain ch1
            {
                uint16_t value = buf[0] | (buf[1] << 8);
                switch (value) {
                    case 0x0701:  // 5V. Gain 1
                    case 0x0601:  // 2V
                    case 0x0501:  // 1V
                        oscilloscope_config.ch_gain[CHANNEL1] = 1;
                        break;
                    case 0x0402:  // 500mV. Gain 2
                        oscilloscope_config.ch_gain[CHANNEL1] = 2;
                        break;
                    case 0x0305:  // 200mV. Gain 5
                        oscilloscope_config.ch_gain[CHANNEL1] = 5;
                        break;
                    case 0x020a:  // 100mV. Gain 10
                    case 0x010a:  // 50mV
                    case 0x000a:  // 20mV
                        oscilloscope_config.ch_gain[CHANNEL1] = 10;
                        break;
                }
                // To use full range (0-255), signal center and step down is done in openhantek with calibration file
                channel_factor[CHANNEL1] = 16 / oscilloscope_config.ch_gain[CHANNEL1];
                debug("\nCommand set gain ch1 (0x%X): %u", value, oscilloscope_config.ch_gain[CHANNEL1]);
                break;
            }
            case 0xE1:  // set gain ch2
            {
                uint16_t value = buf[0] | (buf[1] << 8);
                switch (value) {
                    case 0x0701:  // 5V. Gain 1
                    case 0x0601:  // 2V
                    case 0x0501:  // 1V
                        oscilloscope_config.ch_gain[CHANNEL2] = 1;
                        break;
                    case 0x0402:  // 500mV. Gain 2
                        oscilloscope_config.ch_gain[CHANNEL2] = 2;
                        break;
                    case 0x0305:  // 200mV. Gain 5
                        oscilloscope_config.ch_gain[CHANNEL2] = 5;
                        break;
                    case 0x020a:  // 100mV. Gain 10
                    case 0x010a:  // 50mV
                    case 0x000a:  // 20mV
                        oscilloscope_config.ch_gain[CHANNEL2] = 10;
                        break;
                }
                // To use full range (0-255), signal center and step down is done in openhantek with calibration file
                channel_factor[CHANNEL2] = 16 / oscilloscope_config.ch_gain[CHANNEL2];
                debug("\nCommand set gain ch2 (0x%X): %u", value, oscilloscope_config.ch_gain[CHANNEL2]);
                break;
            }
            case 0xE2:  // set samplerate
            {
                uint16_t value = buf[0] | (buf[1] << 8);
                bool handled = true;
                switch (value) {
                    case 0x0365:
                    case 0x0065:  // 10 downsampling
                        oscilloscope_config.samplerate = 10e3;
                        break;
                    case 0x0446:
                    case 0x0166:  // 10 downsampling
                        oscilloscope_config.samplerate = 20e3;
                        break;
                    case 0x0569:
                    case 0x0269:  // 10 downsampling
                        oscilloscope_config.samplerate = 50e3;
                        break;
                    case 0x066E:
                        oscilloscope_config.samplerate = 100e3;
                        break;
                    case 0x0778:
                        oscilloscope_config.samplerate = 200e3;
                        break;
                    case 0x88C:
                        oscilloscope_config.samplerate = 400e3;
                        break;
                    case 0x0896:
                        oscilloscope_config.samplerate = 500e3;
                        break;
                    case 0x0901:
                        oscilloscope_config.samplerate = 1e6;
                        break;
                    case 0x0A02:
                        oscilloscope_config.samplerate = 2e6;
                        break;
                    default:
                        handled = false;
                }
                if (handled) {
                    debug("\nCommand samplerate: 0x%X", value);
                    if (oscilloscope_config.channel_mask == 0b01)
                        oscilloscope_set_samplerate(oscilloscope_config.samplerate);
                    else
                        oscilloscope_set_samplerate(oscilloscope_config.samplerate * 2);
                } else
                    debug("\nUnknown samplerate: 0x%X", value);
                break;
            }
            case 0xE3:  // start/stop sampling
            {
                if (buf[0] == 1) {
                    if (oscilloscope_state() == IDLE) {
                        debug("\nCommand start sampling");
                        ts = time_us_64();
                        oscilloscope_start();
                        debug("\nOscilloscope start. Samplerate: %u Ch1: %s Ch2: %s", oscilloscope_config.samplerate,
                              (oscilloscope_config.channel_mask & 0B01) ? "enabled" : "disabled",
                              (oscilloscope_config.channel_mask & 0B010) ? "enabled" : "disabled");
                    } else if (oscilloscope_state() == RUNNING) {
                        // debug("\nContinue sampling.");
                    }
                } else {
                    if (oscilloscope_state() == RUNNING) {
                        debug("\nCommand stop sampling. OBF %d OBF/s %.1f", bof,
                              bof / ((time_us_64() - ts) / 1000000.0f));
                        oscilloscope_stop();
                        usb_cancel_transfer(EP6_IN_ADDR);
                    }
                }
                break;
            }
            case 0xE4:  // set channels
            {
                if (buf[0] == 1) {
                    oscilloscope_config.channel_mask = 0b01;
                    oscilloscope_set_channels(oscilloscope_config.channel_mask);
                } else if (buf[0] == 2) {
                    oscilloscope_config.channel_mask = 0b11;
                    oscilloscope_set_channels(oscilloscope_config.channel_mask);
                }
                debug("\nCommand set channel mask: %u", oscilloscope_config.channel_mask);
                if (oscilloscope_config.channel_mask == 0b01)
                    oscilloscope_set_samplerate(oscilloscope_config.samplerate);
                else
                    oscilloscope_set_samplerate(oscilloscope_config.samplerate * 2);
                break;
            }
            case 0xE5:  // set coupling
            {
                if (buf[0] & 0b1)
                    oscilloscope_config.coupling[CHANNEL1] = COUPLING_DC;
                else
                    oscilloscope_config.coupling[CHANNEL1] = COUPLING_AC;
                if (buf[1] & 0b1)
                    oscilloscope_config.coupling[CHANNEL2] = COUPLING_DC;
                else
                    oscilloscope_config.coupling[CHANNEL2] = COUPLING_AC;
                debug("\nCommand set coupling. Channel 1: %s Channel 2: %s",
                      oscilloscope_config.coupling[CHANNEL1] == COUPLING_DC ? "DC" : "AC",
                      oscilloscope_config.coupling[CHANNEL2] == COUPLING_DC ? "DC" : "AC");
                oscilloscope_set_coupling(CHANNEL1, oscilloscope_config.coupling[CHANNEL1]);
                oscilloscope_set_coupling(CHANNEL2, oscilloscope_config.coupling[CHANNEL2]);
                break;
            }
            case 0xE6:  // set calibration frequency
            {
                if (buf[0] == 0)
                    oscilloscope_config.calibration_freq = 100;
                else if (buf[0] <= 100)
                    oscilloscope_config.calibration_freq = buf[0] * 1000;
                else if (buf[0] <= 200)
                    oscilloscope_config.calibration_freq = (buf[0] - 100) * 10;
                else
                    oscilloscope_config.calibration_freq = (buf[0] - 200) * 100;
                debug("\nCommand set calibration frequency (%u): %u Hz", buf[0], oscilloscope_config.calibration_freq);
                oscilloscope_set_calibration_frequency(oscilloscope_config.calibration_freq);
                break;
            }
        }
    } else if (stage == STAGE_STATUS) {
    }
}

void protocol_adc_complete_handler(void) {
    uint adc_pos;
    if (config.no_conversion) {
        adc_pos = sample_count % BUFFER_SIZE;
    } else {
        adc_pos = (sample_count % (BUFFER_SIZE >> 1)) * 2;
    }
    static uint usb_pos = 0;

    if (usb_pos + BULK_SIZE <= buffer_usb_size) {
        fill_buffer_usb(buffer_usb + usb_pos, buffer_adc + adc_pos, BULK_SIZE);
        usb_pos += BULK_SIZE;
    }
    sample_count += BULK_SIZE;

    uint pending = sample_count - queued_count;
    if (pending > buffer_size) {
        queued_count += buffer_size;
        bof++;
    }

    if (sample_count - queued_count >= PACKET_CHUNK_SIZE && !usb_is_busy(EP6_IN_ADDR)) {
        if (usb_init_transfer(EP6_IN_ADDR, PACKET_CHUNK_SIZE)) {
            usb_pos = 0;
            queued_count += PACKET_CHUNK_SIZE;
        }
    }
}

void protocol_init(uint8_t *buffer) {
    buffer_usb = usb_get_endpoint_buffer(EP6_IN_ADDR);
    buffer_usb_size = usb_get_endpoint_buffer_size(EP6_IN_ADDR);
    buffer_adc = buffer;
    if (config.no_conversion) {
        buffer_size = BUFFER_SIZE;
    } else {
        buffer_size = BUFFER_SIZE >> 1;
    }
}
