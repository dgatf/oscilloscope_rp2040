/**
 * Copyright (c) 2024 Daniel Gorbea
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

#include "usb_common.h"
#include "usb_config.h"

void usb_device_init(void);
bool usb_is_configured(void);
void usb_init_transfer(struct usb_endpoint_configuration *ep, int32_t len);
void usb_continue_transfer(struct usb_endpoint_configuration *ep);
void usb_cancel_transfer(struct usb_endpoint_configuration *ep);
struct usb_endpoint_configuration *usb_get_endpoint_configuration(uint8_t addr);
uint8_t usb_get_address(void);
bool usb_is_buffer_b(struct usb_endpoint_configuration *ep);
void set_bsh(uint bsh);

#ifdef __cplusplus
}
#endif

#endif