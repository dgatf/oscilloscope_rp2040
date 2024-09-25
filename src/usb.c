/**
 * Copyright (c) 2024 Daniel Gorbea
 *
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd. author of https://github.com/raspberrypi/pico-examples/tree/master/usb
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "usb.h"

#include <stdio.h>
#include <string.h>

#include "hardware/irq.h"
#include "hardware/regs/usb.h"
#include "hardware/resets.h"
#include "hardware/structs/usb.h"
#include "pico/stdlib.h"
#include "usb_config.c"
#include "common.h"

#define ISOCHRONOUS_MASK 0x18000000u

#define usb_hw_set ((usb_hw_t *)hw_set_alias_untyped(usb_hw))
#define usb_hw_clear ((usb_hw_t *)hw_clear_alias_untyped(usb_hw))

static void bus_reset(void);
static uint8_t prepare_string_descriptor(const unsigned char *str);
static inline uint32_t buffer_offset(volatile uint8_t *buf);
static void setup_endpoint(struct usb_endpoint_configuration *ep);
static void setup_endpoints(void);
static void set_device_configuration(volatile struct usb_setup_packet *pkt);
static void handle_device_descriptor(volatile struct usb_setup_packet *pkt);
static void handle_config_descriptor(volatile struct usb_setup_packet *pkt);
static void handle_string_descriptor(volatile struct usb_setup_packet *pkt);
static void handle_setup_packet(void);
static void handle_ep_buff_done(struct usb_endpoint_configuration *ep);
static void handle_buff_done(uint ep_num, bool in);
static void handle_buff_status(void);
static void start_data_packet(struct usb_endpoint_configuration *ep);
static void acknowledge_out_request(void);
static void acknowledge_in_request(void);
static void prepare_control_packet(volatile struct usb_setup_packet *pkt);
static void ep0_in_handler(uint8_t *buf, uint16_t len);
static void ep0_in_handler(uint8_t *buf, uint16_t len);
static inline uint get_ep_bit(struct usb_endpoint_configuration *ep);
static inline bool is_ep0(struct usb_endpoint_configuration *ep);
static volatile uint32_t *get_endpoint_control(struct usb_endpoint_configuration *ep);
static volatile uint32_t *get_buffer_control(struct usb_endpoint_configuration *ep);
static volatile uint8_t *get_dpram_buffer(struct usb_endpoint_configuration *ep);

static uint8_t dev_addr = 0;
static volatile bool configured = false;
static uint buf_cpu_should_handle;

void isr_usbctrl(void) {
    uint32_t status = usb_hw->ints;
    uint32_t handled = 0;

    if (status & USB_INTS_SETUP_REQ_BITS) {
        handled |= USB_INTS_SETUP_REQ_BITS;
        usb_hw_clear->sie_status = USB_SIE_STATUS_SETUP_REC_BITS;
        handle_setup_packet();
    }

    if (status & USB_INTS_BUFF_STATUS_BITS) {
        handled |= USB_INTS_BUFF_STATUS_BITS;
        handle_buff_status();
    }

    if (status & USB_INTS_BUS_RESET_BITS) {
        handled |= USB_INTS_BUS_RESET_BITS;
        usb_hw_clear->sie_status = USB_SIE_STATUS_BUS_RESET_BITS;
        bus_reset();
    }

    if (status ^ handled) {
        panic("Unhandled IRQ 0x%x\n", (uint)(status ^ handled));
    }
}

static inline uint32_t buffer_offset(volatile uint8_t *buf) { return (uint32_t)buf ^ (uint32_t)usb_dpram; }

static inline bool ep_is_tx(struct usb_endpoint_configuration *ep) {
    return ep->descriptor->bEndpointAddress & USB_DIR_IN;
}

static void bus_reset(void) {
    dev_addr = 0;
    usb_hw->dev_addr_ctrl = 0;
    configured = false;
    // printf("\nBus Reset");
}

static uint8_t prepare_string_descriptor(const unsigned char *str) {
    // 2 for bLength + bDescriptorType + strlen * 2 because string is unicode.
    // i.e. other byte will be 0
    uint8_t bLength = 2 + (strlen((const char *)str) * 2);
    static const uint8_t bDescriptorType = 0x03;

    volatile uint8_t *buf = &ep0_buf[0];
    *buf++ = bLength;
    *buf++ = bDescriptorType;

    uint8_t c;

    do {
        c = *str++;
        *buf++ = c;
        *buf++ = 0;
    } while (c != '\0');

    return bLength;
}

static volatile uint32_t *get_endpoint_control(struct usb_endpoint_configuration *ep) {
    uint i = ep->descriptor->bEndpointAddress & 0x7F;
    if (is_ep0(ep)) return NULL;
    if (ep_is_tx(ep))
        return &usb_dpram->ep_ctrl[i - 1].in;
    else
        return &usb_dpram->ep_ctrl[i - 1].out;
}

static volatile uint32_t *get_buffer_control(struct usb_endpoint_configuration *ep) {
    uint i = ep->descriptor->bEndpointAddress & 0x7F;
    if (ep_is_tx(ep))
        return &usb_dpram->ep_buf_ctrl[i].in;
    else
        return &usb_dpram->ep_buf_ctrl[i].out;
}

static volatile uint8_t *get_dpram_buffer(struct usb_endpoint_configuration *ep) {
    static uint i = 0;
    if (is_ep0(ep)) return &usb_dpram->ep0_buf_a[0];
    uint pre = i;
    i += ep->descriptor->wMaxPacketSize >> 6;
    if (ep->double_buffer) i *= 2;
    return &usb_dpram->epx_data[pre << 6];
}

static void setup_endpoint(struct usb_endpoint_configuration *ep) {
    config_descriptor.wTotalLength += sizeof(ep->descriptor);
    ep->bit = get_ep_bit(ep);
    ep->endpoint_control = get_endpoint_control(ep);
    ep->buffer_control = get_buffer_control(ep);
    ep->dpram_buffer_a = get_dpram_buffer(ep);
    ep->status = STATUS_OK;
    if (ep->double_buffer)
        ep->dpram_buffer_b = ep->dpram_buffer_a + ep->descriptor->wMaxPacketSize;
    else
        ep->dpram_buffer_b = NULL;
    if (!is_ep0(ep)) {
        interface_descriptor.bNumEndpoints++;
        uint32_t dpram_offset = buffer_offset(ep->dpram_buffer_a);
        uint32_t reg = EP_CTRL_ENABLE_BITS | (ep->descriptor->bmAttributes << EP_CTRL_BUFFER_TYPE_LSB) |
                       (ep->double_buffer ? EP_CTRL_DOUBLE_BUFFERED_BITS : 0) | EP_CTRL_INTERRUPT_PER_BUFFER |
                       dpram_offset;
        *ep->endpoint_control = reg;
    }
}

static void setup_endpoints(void) {
    interface_descriptor.bNumEndpoints = 0;
    struct usb_endpoint_configuration *endpoints = dev_config.endpoints;
    for (int i = 0; i < USB_NUM_ENDPOINTS; i++) {
        if (endpoints[i].descriptor) {
            setup_endpoint(&endpoints[i]);
        }
    }
}

static void set_device_configuration(volatile struct usb_setup_packet *pkt) { configured = true; }

static void handle_device_descriptor(volatile struct usb_setup_packet *pkt) {
    const struct usb_device_descriptor *d = dev_config.device_descriptor;
    struct usb_endpoint_configuration *ep = usb_get_endpoint_configuration(EP0_IN_ADDR);
    memcpy(ep->data_buffer, d, sizeof(struct usb_device_descriptor));
    pkt->wLength = sizeof(struct usb_device_descriptor);
}

static void handle_config_descriptor(volatile struct usb_setup_packet *pkt) {
    uint8_t *buf = &ep0_buf[0];
    const struct usb_configuration_descriptor *d = dev_config.config_descriptor;
    memcpy((void *)buf, d, sizeof(struct usb_configuration_descriptor));
    buf += sizeof(struct usb_configuration_descriptor);

    if (pkt->wLength >= d->wTotalLength) {
        memcpy((void *)buf, dev_config.interface_descriptor, sizeof(struct usb_interface_descriptor));
        buf += sizeof(struct usb_interface_descriptor);
        const struct usb_endpoint_configuration *ep = dev_config.endpoints;

        for (uint i = 2; i < USB_NUM_ENDPOINTS; i++) {
            if (ep[i].descriptor) {
                memcpy((void *)buf, ep[i].descriptor, sizeof(struct usb_endpoint_descriptor));
                buf += sizeof(struct usb_endpoint_descriptor);
            }
        }
    }

    uint32_t len = (uint32_t)buf - (uint32_t)&ep0_buf[0];
    struct usb_endpoint_configuration *ep = usb_get_endpoint_configuration(EP0_IN_ADDR);
    pkt->wLength = len;
}

static void handle_string_descriptor(volatile struct usb_setup_packet *pkt) {
    uint8_t i = pkt->wValue & 0xff;
    uint8_t len = 0;

    if (i == 0) {
        len = 4;
        memcpy(&ep0_buf[0], dev_config.lang_descriptor, len);
    } else {
        len = prepare_string_descriptor(dev_config.descriptor_strings[i - 1]);
    }

    struct usb_endpoint_configuration *ep = usb_get_endpoint_configuration(EP0_IN_ADDR);
    pkt->wLength = len;
}

static void handle_setup_packet(void) {
    volatile struct usb_setup_packet *pkt = (volatile struct usb_setup_packet *)&usb_dpram->setup_packet;
    uint8_t bmRequestType = pkt->bmRequestType;
    uint8_t req = pkt->bRequest;
    bool handled = false;
    usb_get_endpoint_configuration(EP0_IN_ADDR)->next_pid = 1u;
    usb_get_endpoint_configuration(EP0_OUT_ADDR)->next_pid = 1u;
    if (bmRequestType == USB_DIR_OUT) {
        if (req == USB_REQUEST_SET_ADDRESS) {
            dev_addr = (pkt->wValue & 0xff);
            handled = true;
        } else if (req == USB_REQUEST_SET_CONFIGURATION) {
            set_device_configuration(pkt);
            handled = true;
        }
    } else if (bmRequestType == USB_DIR_IN) {
        if (req == USB_REQUEST_GET_DESCRIPTOR) {
            uint16_t descriptor_type = pkt->wValue >> 8;
            switch (descriptor_type) {
                case USB_DT_DEVICE:
                    handle_device_descriptor(pkt);
                    handled = true;
                    break;
                case USB_DT_CONFIG:
                    handle_config_descriptor(pkt);
                    handled = true;
                    break;
                case USB_DT_STRING:
                    handle_string_descriptor(pkt);
                    handled = true;
                    break;
            }
        }
    }
    if (dev_config.control_transfer_handler) control_transfer_handler(ep0_buf, pkt, STAGE_SETUP);
    if (!pkt->wLength)
        if (bmRequestType & USB_DIR_IN)
            acknowledge_in_request();
        else
            acknowledge_out_request();
    else {
        prepare_control_packet(pkt);
    }
}

static inline uint get_ep_bit(struct usb_endpoint_configuration *ep) {
    uint bit = ((ep->descriptor->bEndpointAddress & 0x7F) << 1) + !(ep->descriptor->bEndpointAddress >> 7);
    return 1 << bit;
}

static inline bool is_ep0(struct usb_endpoint_configuration *ep) {
    return (ep->descriptor->bEndpointAddress == EP0_IN_ADDR) || (ep->descriptor->bEndpointAddress == EP0_OUT_ADDR);
}

static void start_data_packet(struct usb_endpoint_configuration *ep) {
    uint len;
    if (ep->lenght == UNKNOWN_SIZE /*|| ep->descriptor->bEndpointAddress & USB_DIR_OUT*/)
        if (ep->double_buffer && !ep->pos_send)
            len = ep->descriptor->wMaxPacketSize * 2;
        else
            len = ep->descriptor->wMaxPacketSize;
    else {
        if (ep->double_buffer && !ep->pos_send)
            len = MIN(ep->lenght, ep->descriptor->wMaxPacketSize * 2);
        else
            len = MIN(ep->lenght - ep->pos_send, ep->descriptor->wMaxPacketSize);
    }
    volatile struct usb_setup_packet *pkt = (volatile struct usb_setup_packet *)&usb_dpram->setup_packet;
    uint32_t val = MIN(len, ep->descriptor->wMaxPacketSize) | USB_BUF_CTRL_AVAIL;

    if (!ep->pos) val |= USB_BUF_CTRL_SEL;
    if (ep_is_tx(ep)) {
        if (ep->data_buffer) {
            if (!(buf_cpu_should_handle & ep->bit) || !ep->pos_send) {
                if (ep->data_buffer)
                    memcpy((void *)ep->dpram_buffer_a, (void *)ep->data_buffer + ep->pos_send,
                           MIN(len, ep->descriptor->wMaxPacketSize));
            } else {
                if (ep->data_buffer)
                    memcpy((void *)ep->dpram_buffer_b, (void *)ep->data_buffer + ep->pos_send,
                           MIN(len, ep->descriptor->wMaxPacketSize));
            }
        } else {
            if (!(buf_cpu_should_handle & ep->bit) || !ep->pos_send) {
                if (ep->handler) ep->handler((uint8_t *)ep->dpram_buffer_a, MIN(len, ep->descriptor->wMaxPacketSize));
            } else {
                if (ep->handler) ep->handler((uint8_t *)ep->dpram_buffer_b, MIN(len, ep->descriptor->wMaxPacketSize));
            }
        }
        val |= USB_BUF_CTRL_FULL;
    }
    val |= ep->next_pid ? USB_BUF_CTRL_DATA1_PID : USB_BUF_CTRL_DATA0_PID;
    ep->next_pid ^= 1u;
    if (!(buf_cpu_should_handle & ep->bit) || !ep->pos_send) {
        *ep->buffer_control &= ~0xFFFF;
        *ep->buffer_control |= val;
    } else {
        val |= (ep->descriptor->wMaxPacketSize >> 8) << 11;
        *ep->buffer_control &= ~0xFFFF0000;
        *ep->buffer_control |= val << 16;
    }

    if (len > ep->descriptor->wMaxPacketSize ||
        (is_ep0(ep) && len == ep->descriptor->wMaxPacketSize * 2 && pkt->wLength == ep->pos_send + len)) {
        val = (len - ep->descriptor->wMaxPacketSize) | USB_BUF_CTRL_AVAIL;
        if (ep_is_tx(ep)) {
            if (ep->data_buffer) {
                memcpy((void *)ep->dpram_buffer_b,
                       (void *)(ep->data_buffer + ep->pos_send + ep->descriptor->wMaxPacketSize),
                       len - ep->descriptor->wMaxPacketSize);
            } else {
                if (ep->handler) ep->handler((uint8_t *)ep->dpram_buffer_b, len - ep->descriptor->wMaxPacketSize);
            }
            val |= USB_BUF_CTRL_FULL;
        }
        val |= (ep->next_pid ? USB_BUF_CTRL_DATA1_PID : USB_BUF_CTRL_DATA0_PID);
        val |= (ep->descriptor->wMaxPacketSize >> 8) << 11;
        ep->next_pid ^= 1u;
        *ep->buffer_control &= ~0xFFFF0000;
        *ep->buffer_control |= val << 16;
    }
    ep->pos_send += len;
}

static void handle_ep_buff_done(struct usb_endpoint_configuration *ep) {
    uint len;
    if (buf_cpu_should_handle & ep->bit) {
        len = (*ep->buffer_control >> 16) & USB_BUF_CTRL_LEN_MASK;
        if (ep->lenght != UNKNOWN_SIZE && ep->pos + len > ep->lenght) {
            len = ep->lenght - ep->pos;
            ep->status = STATUS_LENGHT_OVERFLOW;
            ep->is_completed = true;
        }
        if (ep->lenght != UNKNOWN_SIZE && ep->data_buffer && ep->pos + len > ep->data_buffer_size) {
            len = ep->data_buffer_size - ep->pos;
            ep->status = STATUS_BUFFER_OVERFLOW;
            ep->is_completed = true;
        }
        if (ep->data_buffer) {
            if (!ep_is_tx(ep)) memcpy((void *)ep->data_buffer + ep->pos, (void *)ep->dpram_buffer_b, len);
            ep->pos += len;
        } else {
            ep->pos += len;
            if (!ep_is_tx(ep) && ep->handler) ep->handler((uint8_t *)ep->dpram_buffer_b, len);
        }
    } else {
        len = *ep->buffer_control & USB_BUF_CTRL_LEN_MASK;
        if (ep->lenght != UNKNOWN_SIZE && ep->pos + len > ep->lenght) {
            len = ep->lenght - ep->pos;
            ep->status = STATUS_LENGHT_OVERFLOW;
            ep->is_completed = true;
        }
        if (ep->lenght != UNKNOWN_SIZE && ep->data_buffer && ep->pos + len > ep->data_buffer_size) {
            len = ep->data_buffer_size - ep->pos;
            ep->status = STATUS_BUFFER_OVERFLOW;
            ep->is_completed = true;
        }
        if (ep->data_buffer) {
            if (!ep_is_tx(ep)) memcpy((void *)ep->data_buffer + ep->pos, (void *)ep->dpram_buffer_a, len);
            ep->pos += len;
        } else {
            ep->pos += len;
            if (!ep_is_tx(ep) && ep->handler) ep->handler((uint8_t *)ep->dpram_buffer_a, len);
        }
    }
    //  Handle end of transfer
    if (len < ep->descriptor->wMaxPacketSize ||
        (is_ep0(ep) && len == ep->descriptor->wMaxPacketSize && ep->lenght == ep->pos) || ep->status != STATUS_BUSY) {
        ep->lenght = ep->pos;
        ep->is_completed = true;
        if (ep->status == STATUS_BUSY) ep->status = STATUS_OK;
        if (ep->data_buffer && ep->handler) ep->handler((uint8_t *)ep->data_buffer, ep->lenght);
        if (ep->status != STATUS_OK) usb_cancel_transfer(ep);
    } else {
        if ((ep->lenght == UNKNOWN_SIZE || ep->pos_send < ep->lenght) && ep->data_buffer) {
            start_data_packet(ep);
        }
    }
}

static void handle_buff_done(uint ep_num, bool in) {
    uint8_t ep_addr = ep_num | (in ? USB_DIR_IN : 0);
    for (uint i = 0; i < USB_NUM_ENDPOINTS; i++) {
        struct usb_endpoint_configuration *ep = &dev_config.endpoints[i];
        if (ep->descriptor && ep->handler) {
            if (ep->descriptor->bEndpointAddress == ep_addr) {
                handle_ep_buff_done(ep);
                return;
            }
        }
    }
}

static void handle_buff_status(void) {
    uint32_t buffers = usb_hw->buf_status;
    uint32_t remaining_buffers = buffers;
    uint bit = 1u;
    for (uint i = 0; remaining_buffers && i < USB_NUM_ENDPOINTS * 2; i++) {
        if (remaining_buffers & bit) {
            buf_cpu_should_handle = usb_hw->buf_cpu_should_handle;
            usb_hw_clear->buf_status = bit;
            handle_buff_done(i >> 1u, !(i & 1u));
            remaining_buffers &= ~bit;
        }
        bit <<= 1u;
    }
}

static void acknowledge_out_request(void) {
    struct usb_endpoint_configuration *ep = usb_get_endpoint_configuration(EP0_IN_ADDR);
    usb_init_transfer(ep, 0);
}

static void acknowledge_in_request(void) {
    struct usb_endpoint_configuration *ep = usb_get_endpoint_configuration(EP0_OUT_ADDR);
    usb_init_transfer(ep, 0);
}

static void prepare_control_packet(volatile struct usb_setup_packet *pkt) {
    if (pkt->bmRequestType & USB_DIR_IN) {
        struct usb_endpoint_configuration *ep = usb_get_endpoint_configuration(EP0_IN_ADDR);
        if (pkt->wLength) usb_init_transfer(ep, pkt->wLength);
    } else {
        struct usb_endpoint_configuration *ep = usb_get_endpoint_configuration(EP0_OUT_ADDR);
        if (pkt->wLength) usb_init_transfer(ep, pkt->wLength);
    }
}

static void ep0_in_handler(uint8_t *buf, uint16_t len) {
    volatile struct usb_setup_packet *pkt = (volatile struct usb_setup_packet *)&usb_dpram->setup_packet;
    if (!len) {  // Ack out request done
        if (pkt->bRequest == USB_REQUEST_SET_ADDRESS) {
            usb_hw->dev_addr_ctrl = dev_addr;
        }
        if (dev_config.control_transfer_handler) control_transfer_handler(ep0_buf, pkt, STAGE_STATUS);
        return;
    }
    if (dev_config.control_transfer_handler) control_transfer_handler(ep0_buf, pkt, STAGE_DATA);
    acknowledge_in_request();
}

static void ep0_out_handler(uint8_t *buf, uint16_t len) {
    volatile struct usb_setup_packet *pkt = (volatile struct usb_setup_packet *)&usb_dpram->setup_packet;
    if (!len) {  // Ack in request done
        if (dev_config.control_transfer_handler) control_transfer_handler(ep0_buf, pkt, STAGE_STATUS);
        return;
    }
    if (dev_config.control_transfer_handler) control_transfer_handler(ep0_buf, pkt, STAGE_DATA);
    acknowledge_out_request();
}

struct usb_endpoint_configuration *usb_get_endpoint_configuration(uint8_t addr) {
    struct usb_endpoint_configuration *endpoints = dev_config.endpoints;
    for (int i = 0; i < USB_NUM_ENDPOINTS; i++) {
        if (endpoints[i].descriptor && (endpoints[i].descriptor->bEndpointAddress == addr)) {
            return &endpoints[i];
        }
    }
    return NULL;
}

void usb_device_init(void) {
    reset_block(RESETS_RESET_USBCTRL_BITS);
    unreset_block_wait(RESETS_RESET_USBCTRL_BITS);
    memset(usb_dpram, 0, sizeof(*usb_dpram));
    irq_set_enabled(USBCTRL_IRQ, true);
    usb_hw->muxing = USB_USB_MUXING_TO_PHY_BITS | USB_USB_MUXING_SOFTCON_BITS;
    usb_hw->pwr = USB_USB_PWR_VBUS_DETECT_BITS | USB_USB_PWR_VBUS_DETECT_OVERRIDE_EN_BITS;
    usb_hw->main_ctrl = USB_MAIN_CTRL_CONTROLLER_EN_BITS;
    usb_hw->sie_ctrl = USB_SIE_CTRL_EP0_INT_1BUF_BITS;
    usb_hw->inte = USB_INTS_BUFF_STATUS_BITS | USB_INTS_BUS_RESET_BITS | USB_INTS_SETUP_REQ_BITS;
    usb_get_endpoint_configuration(EP0_IN_ADDR)->handler = ep0_in_handler;
    usb_get_endpoint_configuration(EP0_OUT_ADDR)->handler = ep0_out_handler;
    config_descriptor.wTotalLength = sizeof(config_descriptor) + sizeof(interface_descriptor);
    setup_endpoints();
    usb_hw_set->sie_ctrl = USB_SIE_CTRL_PULLUP_EN_BITS;
}

bool usb_is_configured(void) { return configured; }

void usb_init_transfer(struct usb_endpoint_configuration *ep, int32_t len) {
    if (len < 0) len = UNKNOWN_SIZE;
    ep->lenght = len;
    ep->pos = 0;
    ep->pos_send = 0;
    ep->is_completed = false;
    ep->status = STATUS_BUSY;
    start_data_packet(ep);
}

void usb_continue_transfer(struct usb_endpoint_configuration *ep) { start_data_packet(ep); }

void usb_cancel_transfer(struct usb_endpoint_configuration *ep) {
    usb_hw_clear->buf_status = ep->bit;
    usb_hw_clear->buf_status = ep->bit;
    ep->buffer_control = 0;
}

uint8_t usb_get_address(void) { return dev_addr; }
