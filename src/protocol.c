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

extern config_t config_;
extern volatile oscilloscope_config_t oscilloscope_config_;
extern char debug_message_[DEBUG_BUFFER_SIZE];

static volatile uint8_t *buffer_;
static volatile uint sample_count_ = 0, channel_factor_[2];
static struct usb_endpoint_configuration *ep;
static volatile bool should_stop = false;
static critical_section_t lock;

static void core1_entry(void);
static void prepare_buffers(void);
static void fill_buffer(volatile uint8_t *buffer, uint length);

static void core1_entry(void) {
    while (1) {
        protocol_task();
    }
}

void protocol_task(void) {
    if (config_.is_multicore) critical_section_enter_blocking(&lock);
    if ((!ep->double_buffer && sample_count_ - ep->pos >= BULK_SIZE * 2) ||
        (ep->double_buffer &&
         (ep->pos_send - ep->pos == BULK_SIZE || ep->status == STATUS_OK || ep->pos_send == ep->length)) &&
            (sample_count_ - ep->pos >= BULK_SIZE * 2)) {
        if (config_.no_conversion) {
            if (sample_count_ - ep->pos > BUFFER_SIZE) {
                // ep->pos += BULK_SIZE;
                // ep->pos_send += BULK_SIZE;
                // debug("\nBOF");
            }
            prepare_buffers();
        } else {
            if (sample_count_ - ep->pos > (BUFFER_SIZE >> 1)) {
                // ep->pos += BULK_SIZE;
                // ep->pos_send += BULK_SIZE;
                // debug("\nBOF");
            }
            prepare_buffers();
        }
    }
    if (config_.is_multicore) critical_section_exit(&lock);
}

static void fill_buffer(volatile uint8_t *buffer, uint length) {
    if (config_.no_conversion) {
        uint pos = ep->pos_send % BUFFER_SIZE;
        for (uint i = 0; i < length; i++) *(buffer + i) = *(buffer_ + pos + i);
    } else {
        uint pos = ep->pos_send % (BUFFER_SIZE >> 1);
        uint ch_gain;
        for (uint i = 0; i < length; i++) {
            if (oscilloscope_config_.channel_mask == 0b11 && (i % 2))
                ch_gain = oscilloscope_config_.ch_gain[CHANNEL2];
            else
                ch_gain = oscilloscope_config_.ch_gain[CHANNEL1];
            uint16_t *buffer_pos = (void *)buffer_;
            buffer_pos += pos + i;
            uint16_t value = (*buffer_pos * ch_gain) >> 4;
            if (value > 0xFF) value = 0xFF;
            *(buffer + i) = value;
        }
    }
}

static inline void prepare_buffers(void) {
    if (should_stop) {
        ep->length = ep->pos_send + 124;
        should_stop = false;
    }
    if (ep->is_completed) {
        oscilloscope_stop();
        sample_count_ = 0;
        ep->pos = 0;
        ep->pos_send = 0;
        ep->status == STATUS_OK;
        ep->next_pid = 0;
        ep->is_completed = false;
        *ep->buffer_control = 0;
        ep->length = UNKNOWN_SIZE;
        return;
    }
    if (ep->status == STATUS_BUSY) {
        if (!ep->double_buffer) {
            if (!(*ep->buffer_control & USB_BUF_CTRL_AVAIL)) {
                fill_buffer(ep->dpram_buffer_a, BULK_SIZE);
                usb_continue_transfer(ep);
            }
        } else {
            if (!ep->next_pid) {
                if (!((*ep->buffer_control) & USB_BUF_CTRL_AVAIL)) {
                    fill_buffer(ep->dpram_buffer_a, BULK_SIZE);
                    usb_continue_transfer(ep);
                }
            } else {
                if (!((*ep->buffer_control >> 16) & USB_BUF_CTRL_AVAIL)) {
                    fill_buffer(ep->dpram_buffer_b, BULK_SIZE);
                    usb_continue_transfer(ep);
                }
            }
        }
    } else {
        if (!ep->double_buffer) {
            debug("\nINIT %u %u", sample_count_, ep->pos);
            fill_buffer(ep->dpram_buffer_a, BULK_SIZE);
            usb_init_transfer(ep, UNKNOWN_SIZE);
        } else if (sample_count_ >= BULK_SIZE * 2) {
            fill_buffer(ep->dpram_buffer_a, BULK_SIZE * 2);
            usb_init_transfer(ep, UNKNOWN_SIZE);
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
                        oscilloscope_config_.ch_gain[CHANNEL1] = 1;
                        break;
                    case 0x0402:  // 500mV. Gain 2
                        oscilloscope_config_.ch_gain[CHANNEL1] = 2;
                        break;
                    case 0x0305:  // 200mV. Gain 5
                        oscilloscope_config_.ch_gain[CHANNEL1] = 5;
                        break;
                    case 0x020a:  // 100mV. Gain 10
                    case 0x010a:  // 50mV
                    case 0x000a:  // 20mV
                        oscilloscope_config_.ch_gain[CHANNEL1] = 10;
                        break;
                }
                // To use full range (0-255), signal center and step down is done in openhantek with calibration file
                channel_factor_[CHANNEL1] = 16 / oscilloscope_config_.ch_gain[CHANNEL1];
                debug("\nCommand set gain ch1 (0x%X): %u", value, oscilloscope_config_.ch_gain[CHANNEL1]);
                break;
            }
            case 0xE1:  // set gain ch2
            {
                uint16_t value = buf[0] | (buf[1] << 8);
                switch (value) {
                    case 0x0701:  // 5V. Gain 1
                    case 0x0601:  // 2V
                    case 0x0501:  // 1V
                        oscilloscope_config_.ch_gain[CHANNEL2] = 1;
                        break;
                    case 0x0402:  // 500mV. Gain 2
                        oscilloscope_config_.ch_gain[CHANNEL2] = 2;
                        break;
                    case 0x0305:  // 200mV. Gain 5
                        oscilloscope_config_.ch_gain[CHANNEL2] = 5;
                        break;
                    case 0x020a:  // 100mV. Gain 10
                    case 0x010a:  // 50mV
                    case 0x000a:  // 20mV
                        oscilloscope_config_.ch_gain[CHANNEL2] = 10;
                        break;
                }
                // To use full range (0-255), signal center and step down is done in openhantek with calibration file
                channel_factor_[CHANNEL2] = 16 / oscilloscope_config_.ch_gain[CHANNEL2];
                debug("\nCommand set gain ch2 (0x%X): %u", value, oscilloscope_config_.ch_gain[CHANNEL2]);
                break;
            }
            case 0xE2:  // set samplerate
            {
                uint16_t value = buf[0] | (buf[1] << 8);
                bool handled = true;
                switch (value) {
                    case 0x0365:
                    case 0x0065:  // 10 downsampling
                        oscilloscope_config_.samplerate = 10e3;
                        break;
                    case 0x0446:
                    case 0x0166:  // 10 downsampling
                        oscilloscope_config_.samplerate = 20e3;
                        break;
                    case 0x0569:
                    case 0x0269:  // 10 downsampling
                        oscilloscope_config_.samplerate = 50e3;
                        break;
                    case 0x066E:
                        oscilloscope_config_.samplerate = 100e3;
                        break;
                    case 0x0778:
                        oscilloscope_config_.samplerate = 200e3;
                        break;
                    case 0x88C:
                        oscilloscope_config_.samplerate = 400e3;
                        break;
                    case 0x0896:
                        oscilloscope_config_.samplerate = 500e3;
                        break;
                    case 0x0901:
                        oscilloscope_config_.samplerate = 1e6;
                        break;
                    case 0x0A02:
                        oscilloscope_config_.samplerate = 2e6;
                        break;
                    default:
                        handled = false;
                }
                if (handled) {
                    debug("\nCommand samplerate: 0x%X", value);
                    if (oscilloscope_config_.channel_mask == 0b01)
                        oscilloscope_set_samplerate(oscilloscope_config_.samplerate);
                    else
                        oscilloscope_set_samplerate(oscilloscope_config_.samplerate * 2);
                } else
                    debug("\nUnknown samplerate: 0x%X", value);
                break;
            }
            case 0xE3:  // start/stop sampling
            {
                if (buf[0] == 1) {
                    if (oscilloscope_state() == IDLE) {
                        debug("\nCommand start sampling");
                        sample_count_ = 0;

                        oscilloscope_start();
                        debug("\nOscilloscope start. Samplerate: %u Ch1: %s Ch2: %s", oscilloscope_config_.samplerate,
                              (oscilloscope_config_.channel_mask & 0B01) ? "enabled" : "disabled",
                              (oscilloscope_config_.channel_mask & 0B010) ? "enabled" : "disabled");
                    }
                } else {
                    if (oscilloscope_state() == RUNNING) {
                        debug("\nCommand stop sampling");
                        should_stop = true;
                    }
                }
                break;
            }
            case 0xE4:  // set channels
            {
                if (buf[0] == 1) {
                    oscilloscope_config_.channel_mask = 0b01;
                    oscilloscope_set_channels(oscilloscope_config_.channel_mask);
                } else if (buf[0] == 2) {
                    oscilloscope_config_.channel_mask = 0b11;
                    oscilloscope_set_channels(oscilloscope_config_.channel_mask);
                }
                debug("\nCommand set channel mask: %u", oscilloscope_config_.channel_mask);
                if (oscilloscope_config_.channel_mask == 0b01)
                    oscilloscope_set_samplerate(oscilloscope_config_.samplerate);
                else
                    oscilloscope_set_samplerate(oscilloscope_config_.samplerate * 2);
                break;
            }
            case 0xE5:  // set coupling
            {
                if (buf[0] & 0b1)
                    oscilloscope_config_.coupling[CHANNEL1] = COUPLING_DC;
                else
                    oscilloscope_config_.coupling[CHANNEL1] = COUPLING_AC;
                if (buf[1] & 0b1)
                    oscilloscope_config_.coupling[CHANNEL2] = COUPLING_DC;
                else
                    oscilloscope_config_.coupling[CHANNEL2] = COUPLING_AC;
                debug("\nCommand set coupling. Channel 1: %s Channel 2: %s",
                      oscilloscope_config_.coupling[CHANNEL1] == COUPLING_DC ? "DC" : "AC",
                      oscilloscope_config_.coupling[CHANNEL2] == COUPLING_DC ? "DC" : "AC");
                oscilloscope_set_coupling(CHANNEL1, oscilloscope_config_.coupling[CHANNEL1]);
                oscilloscope_set_coupling(CHANNEL2, oscilloscope_config_.coupling[CHANNEL2]);
                break;
            }
            case 0xE6:  // set calibration frequency
            {
                if (buf[0] == 0)
                    oscilloscope_config_.calibration_freq = 100;
                else if (buf[0] <= 100)
                    oscilloscope_config_.calibration_freq = buf[0] * 1000;
                else if (buf[0] <= 200)
                    oscilloscope_config_.calibration_freq = (buf[0] - 100) * 10;
                else
                    oscilloscope_config_.calibration_freq = (buf[0] - 200) * 100;
                debug("\nCommand set calibration frequency (%u): %u Hz", buf[0], oscilloscope_config_.calibration_freq);
                oscilloscope_set_calibration_frequency(oscilloscope_config_.calibration_freq);
                break;
            }
        }
    } else if (stage == STAGE_STATUS) {
    }
}

void protocol_complete_handler(void) { sample_count_ += BULK_SIZE; }

void protocol_init(uint8_t *buffer) {
    buffer_ = buffer;
    ep = usb_get_endpoint_configuration(EP6_IN_ADDR);
    ep->pos = 0;
    if (config_.is_multicore) multicore_launch_core1(core1_entry);
}
