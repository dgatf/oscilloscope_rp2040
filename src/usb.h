/**
 * Copyright (c) 2024-2026 Daniel Gorbea
 * 
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd. author of https://github.com/raspberrypi/pico-examples/tree/master/usb
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef USB_LOWLEVEL_H
#define USB_LOWLEVEL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usb_config.h"

void usb_device_init(void);
bool usb_is_configured(void);
bool usb_init_transfer(uint8_t addr, uint len);
void usb_cancel_transfer(uint8_t addr);
uint8_t usb_get_address(void);
uint8_t *usb_get_endpoint_buffer(uint8_t addr);
void usb_set_endpoint_buffer(uint8_t addr, uint8_t *buf);
uint usb_get_endpoint_buffer_size(uint8_t addr);
bool usb_is_busy(uint8_t addr);

#ifdef __cplusplus
}
#endif

#endif