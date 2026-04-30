/**
 * Copyright (c) 2024 Daniel Gorbea
 *
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd. author of https://github.com/raspberrypi/pico-examples/tree/master/usb
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef USB_CONFIG_H
#define USB_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usb_common.h"

#define EP6_IN_ADDR (USB_DIR_IN | 6)

void control_transfer_handler(uint8_t *buf, volatile struct usb_setup_packet *pkt, uint8_t stage);
void ep6_in_handler(uint8_t *buf, uint16_t len);

static const struct usb_endpoint_descriptor ep0_out = {.bLength = sizeof(struct usb_endpoint_descriptor),
                                                       .bDescriptorType = USB_DT_ENDPOINT,
                                                       .bEndpointAddress = EP0_OUT_ADDR,
                                                       .bmAttributes = USB_TRANSFER_TYPE_CONTROL,
                                                       .wMaxPacketSize = PACKET_SIZE_CONTROL,
                                                       .bInterval = 0};

static const struct usb_endpoint_descriptor ep0_in = {.bLength = sizeof(struct usb_endpoint_descriptor),
                                                      .bDescriptorType = USB_DT_ENDPOINT,
                                                      .bEndpointAddress = EP0_IN_ADDR,
                                                      .bmAttributes = USB_TRANSFER_TYPE_CONTROL,
                                                      .wMaxPacketSize = PACKET_SIZE_CONTROL,
                                                      .bInterval = 0};

static const struct usb_endpoint_descriptor ep6_in = {.bLength = sizeof(struct usb_endpoint_descriptor),
                                                      .bDescriptorType = USB_DT_ENDPOINT,
                                                      .bEndpointAddress = EP6_IN_ADDR,
                                                      .bmAttributes = USB_TRANSFER_TYPE_BULK,
                                                      .wMaxPacketSize = PACKET_SIZE_BULK,
                                                      .bInterval = 1};

static const struct usb_device_descriptor device_descriptor = {
    .bLength = sizeof(struct usb_device_descriptor),
    .bDescriptorType = USB_DT_DEVICE,
    .bcdUSB = 0x0110,        // USB 1.1 device
    .bDeviceClass = 0,       // Specified in interface descriptor
    .bDeviceSubClass = 0,    // No subclass
    .bDeviceProtocol = 0,    // No protocol
    .bMaxPacketSize0 = 64,   // Max packet size for ep0
    .idVendor = 0x2e8a,      // Your vendor id
    .idProduct = 0x2041,     // Your product ID
    .bcdDevice = 0,          // No device revision number
    .iManufacturer = 1,      // Manufacturer string index
    .iProduct = 2,           // Product string index
    .iSerialNumber = 3,      // Serial number string index
    .bNumConfigurations = 1  // One configuration
};

static struct usb_interface_descriptor interface_descriptor = {.bLength = sizeof(struct usb_interface_descriptor),
                                                               .bDescriptorType = USB_DT_INTERFACE,
                                                               .bInterfaceNumber = 0,
                                                               .bAlternateSetting = 0,
                                                               .bInterfaceClass = 0xff,  // Vendor specific endpoint
                                                               .bInterfaceSubClass = 0,
                                                               .bInterfaceProtocol = 0,
                                                               .iInterface = 0};

static struct usb_configuration_descriptor config_descriptor = {
    .bLength = sizeof(struct usb_configuration_descriptor),
    .bDescriptorType = USB_DT_CONFIG,
    .bNumInterfaces = 1,
    .bConfigurationValue = 1,  // Configuration 1
    .iConfiguration = 0,       // No string
    .bmAttributes = 0xc0,      // attributes: self powered, no remote wakeup
    .bMaxPower = 0x32          // 100ma
};

static const unsigned char lang_descriptor[] = {
    4,          // bLength
    0x03,       // bDescriptorType == String Descriptor
    0x09, 0x04  // language id = us english
};

static const unsigned char *descriptor_strings[] = {
    (unsigned char *)"Raspberry Pi",         // Vendor
    (unsigned char *)"RP2040 Oscilloscope",  // Product
    (unsigned char *)"0"                     // Serial
};

extern struct usb_device_configuration dev_configs[];

#ifdef __cplusplus
}
#endif

#endif