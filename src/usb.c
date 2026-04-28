/**
 * Copyright (c) 2024-2026 Daniel Gorbea
 *
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd. author of https://github.com/raspberrypi/pico-examples/tree/master/usb
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "usb.h"

#include <string.h>

#include "pico/stdlib.h"
#include "usb_config.c"

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
static uint handle_ep_buff_done(struct usb_endpoint_configuration *ep);
static void handle_buff_done(uint ep_num, bool in);
static void handle_buff_status(void);
static uint start_data_packet(struct usb_endpoint_configuration *ep);
static void acknowledge_out_request(void);
static void acknowledge_in_request(void);
static void prepare_control_packet(volatile struct usb_setup_packet *pkt);
static void ep0_in_handler(uint8_t *buf, uint16_t len);
static inline uint get_ep_bit(struct usb_endpoint_configuration *ep);
static inline bool is_ep0(struct usb_endpoint_configuration *ep);
static volatile uint32_t *get_endpoint_control(struct usb_endpoint_configuration *ep);
static volatile uint32_t *get_buffer_control(struct usb_endpoint_configuration *ep);
static volatile uint8_t *get_dpram_buffer(struct usb_endpoint_configuration *ep);
static inline uint prepare_buffer_a(struct usb_endpoint_configuration *ep);
static inline uint prepare_buffer_b(struct usb_endpoint_configuration *ep);
static inline uint read_buffer(struct usb_endpoint_configuration *ep, bool is_buffer_a);
static struct usb_endpoint_configuration *usb_get_endpoint_configuration(uint8_t addr);

static uint8_t dev_addr = 0;
static volatile bool configured = false;

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
    config_descriptor.wTotalLength += sizeof(struct usb_endpoint_descriptor);
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
    else
        prepare_control_packet(pkt);
}

static inline uint get_ep_bit(struct usb_endpoint_configuration *ep) {
    uint bit = ((ep->descriptor->bEndpointAddress & 0x7F) << 1) + !(ep->descriptor->bEndpointAddress >> 7);
    return 1 << bit;
}

static inline bool is_ep0(struct usb_endpoint_configuration *ep) {
    return (ep->descriptor->bEndpointAddress == EP0_IN_ADDR) || (ep->descriptor->bEndpointAddress == EP0_OUT_ADDR);
}

static inline uint prepare_buffer_a(struct usb_endpoint_configuration *ep) {
    if (ep_is_tx(ep) && (*ep->buffer_control & USB_BUF_CTRL_FULL)) return 0;
    if (!ep_is_tx(ep) && (*ep->buffer_control & USB_BUF_CTRL_AVAIL)) return 0;
    uint len = MIN(ep->length - ep->queued_pos, ep->descriptor->wMaxPacketSize);
    uint32_t val = len;
    if (ep->is_start) val |= USB_BUF_CTRL_SEL;
    if (ep_is_tx(ep)) {
        if (ep->data_buffer) {
            memcpy((void *)ep->dpram_buffer_a, (void *)ep->data_buffer + ep->queued_pos,
                   MIN(len, ep->descriptor->wMaxPacketSize));
        } else {
            if (ep->handler) ep->handler((uint8_t *)ep->dpram_buffer_a, MIN(len, ep->descriptor->wMaxPacketSize));
        }
        val |= USB_BUF_CTRL_FULL;
    }
    val |= ep->next_pid ? USB_BUF_CTRL_DATA1_PID : USB_BUF_CTRL_DATA0_PID;
    ep->next_pid ^= 1u;
    val |= USB_BUF_CTRL_AVAIL;
    ep->queued_pos += len;
    ep->is_start = false;
    *ep->buffer_control = (*ep->buffer_control & ~(uint32_t)0xFFFF) | val;
    return len;
}

static inline uint prepare_buffer_b(struct usb_endpoint_configuration *ep) {
    if (ep_is_tx(ep) && (*ep->buffer_control & (USB_BUF_CTRL_FULL << 16))) return 0;
    if (!ep_is_tx(ep) && (*ep->buffer_control & (USB_BUF_CTRL_AVAIL << 16))) return 0;
    uint len = MIN(ep->length - ep->queued_pos, ep->descriptor->wMaxPacketSize);
    uint32_t val = len;
    if (ep->is_start) val |= USB_BUF_CTRL_SEL;
    if (ep_is_tx(ep)) {
        if (ep->data_buffer) {
            memcpy((void *)ep->dpram_buffer_b, (void *)ep->data_buffer + ep->queued_pos,
                   MIN(len, ep->descriptor->wMaxPacketSize));
        } else {
            if (ep->handler) ep->handler((uint8_t *)ep->dpram_buffer_b, MIN(len, ep->descriptor->wMaxPacketSize));
        }
        val |= USB_BUF_CTRL_FULL;
    }
    val |= ep->next_pid ? USB_BUF_CTRL_DATA1_PID : USB_BUF_CTRL_DATA0_PID;
    ep->next_pid ^= 1u;
    val |= USB_BUF_CTRL_AVAIL;
    ep->queued_pos += len;
    ep->is_start = false;
    *ep->buffer_control = (*ep->buffer_control & (uint32_t)0xFFFF) | (val << 16);
    return len;
}

static uint start_data_packet(struct usb_endpoint_configuration *ep) {
    uint len;
    if (!ep->double_buffer) {
        len = prepare_buffer_a(ep);
    } else {
        if (ep->is_start) {
            len = prepare_buffer_a(ep);
            len += prepare_buffer_b(ep);
        } else {
            if (!(usb_hw->buf_cpu_should_handle & ep->bit)) {
                len = prepare_buffer_a(ep);
                len += prepare_buffer_b(ep);
            } else {
                len = prepare_buffer_b(ep);
                len += prepare_buffer_a(ep);
            }
        }
    }

    return len;
}

static uint get_buffer_length(struct usb_endpoint_configuration *ep, bool is_buffer_a) {
    if (is_buffer_a) {
        return *ep->buffer_control & USB_BUF_CTRL_LEN_MASK;
    } else {
        return (*ep->buffer_control >> 16) & USB_BUF_CTRL_LEN_MASK;
    }
}

static inline uint read_buffer(struct usb_endpoint_configuration *ep, bool is_buffer_a) {
    usb_hw_clear->buf_status = ep->bit;
    uint len;
    volatile uint8_t *buffer;
    if (is_buffer_a) {
        len = *ep->buffer_control & USB_BUF_CTRL_LEN_MASK;
        buffer = ep->dpram_buffer_a;
    } else {
        len = (*ep->buffer_control >> 16) & USB_BUF_CTRL_LEN_MASK;
        buffer = ep->dpram_buffer_b;
    }

    // Check for overflow and completion conditions
    if (ep->completed_pos + len > ep->length) {
        len = ep->length - ep->completed_pos;
        ep->status = STATUS_LENGTH_OVERFLOW;
        ep->is_completed = true;
    }
    if (ep->data_buffer && ep->completed_pos + len > ep->data_buffer_size) {
        len = ep->data_buffer_size - ep->completed_pos;
        ep->status = STATUS_BUFFER_OVERFLOW;
        ep->is_completed = true;
    }
    // completa si: paquete corto < wMaxPacketSize, o paquete completo y es el último, o status no es busy (ej.
    // overflow) if (len < ep->descriptor->wMaxPacketSize ||
    //     (len == ep->descriptor->wMaxPacketSize && ep->length == ep->completed_pos + len) || ep->status !=
    //     STATUS_BUSY) {
    if (len < ep->descriptor->wMaxPacketSize || ep->completed_pos + len >= ep->length) {
        ep->is_completed = true;
        if (ep->status == STATUS_BUSY) ep->status = STATUS_OK;
    }

    // Copy data to buffer or call handler
    if (!ep_is_tx(ep)) {
        if (ep->data_buffer)
            memcpy((void *)ep->data_buffer + ep->completed_pos, (void *)buffer, len);
        else if (ep->handler)
            ep->handler((uint8_t *)buffer, len);
    }

    ep->completed_pos += len;

    // If transfer is completed, call handler with final buffer. Otherwise, if it's a TX transfer, prepare next packet.
    if (ep->is_completed) {
        if (ep->handler)
            if (ep->data_buffer)
                ep->handler((uint8_t *)ep->data_buffer, ep->completed_pos);
            else
                ep->handler((uint8_t *)NULL, len);
    } else {
        if ((ep->queued_pos < ep->length)) {
            if (ep->data_buffer)
                start_data_packet(ep);
            else if (ep_is_tx(ep) && ep->handler)
                ep->handler((uint8_t *)NULL, len);
        }
    }

    if (ep->status == STATUS_LENGTH_OVERFLOW || ep->status == STATUS_BUFFER_OVERFLOW) usb_cancel_transfer(ep->descriptor->bEndpointAddress);

    return len;
}

static uint handle_ep_buff_done(struct usb_endpoint_configuration *ep) {
    uint len;
    if (!ep->double_buffer) {
        len = read_buffer(ep, true);
    } else {
        if (usb_hw->buf_cpu_should_handle & ep->bit) {
            len = read_buffer(ep, false);
        } else {
            len = read_buffer(ep, true);
        }
    }
    return len;
}

static void handle_buff_done(uint ep_num, bool in) {
    uint8_t ep_addr = ep_num | (in ? USB_DIR_IN : 0);
    for (uint i = 0; i < USB_NUM_ENDPOINTS; i++) {
        struct usb_endpoint_configuration *ep = &dev_config.endpoints[i];
        if (ep->descriptor) {
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
            handle_buff_done(i >> 1u, !(i & 1u));
            remaining_buffers &= ~bit;
        }
        bit <<= 1u;
    }
}

static void acknowledge_out_request(void) {
    usb_init_transfer(EP0_IN_ADDR, 0);
}

static void acknowledge_in_request(void) {
    usb_init_transfer(EP0_OUT_ADDR, 0);
}

static void prepare_control_packet(volatile struct usb_setup_packet *pkt) {
    if (pkt->bmRequestType & USB_DIR_IN) {
        if (pkt->wLength) usb_init_transfer(EP0_IN_ADDR, pkt->wLength);
    } else {
        if (pkt->wLength) usb_init_transfer(EP0_OUT_ADDR, pkt->wLength);
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

bool usb_init_transfer(uint8_t addr, uint len) {
    struct usb_endpoint_configuration *ep = usb_get_endpoint_configuration(addr);
    if (!ep) return false;
    if (len < 0) return false;
    ep->length = len;
    ep->completed_pos = 0;
    ep->queued_pos = 0;
    ep->is_start = true;
    ep->is_completed = false;
    ep->status = STATUS_BUSY;
    if (ep->data_buffer || !ep_is_tx(ep)) start_data_packet(ep);
    return true;
}

void usb_cancel_transfer(uint8_t addr) {
    struct usb_endpoint_configuration *ep = usb_get_endpoint_configuration(addr);
    if (!ep) return;
    usb_hw_clear->buf_status = ep->bit;
    usb_hw_clear->buf_status = ep->bit;
    *ep->buffer_control = 0;
}

uint8_t usb_get_address(void) { return dev_addr; }

uint8_t *usb_get_endpoint_buffer(uint8_t addr) {
    struct usb_endpoint_configuration *ep = usb_get_endpoint_configuration(addr);
    if (!ep) return NULL;
    return ep->data_buffer;
}

void usb_set_endpoint_buffer(uint8_t addr, uint8_t *buf) {
    struct usb_endpoint_configuration *ep = usb_get_endpoint_configuration(addr);
    if (!ep) return;
    ep->data_buffer = buf;
}

uint usb_get_endpoint_buffer_size(uint8_t addr) {
    struct usb_endpoint_configuration *ep = usb_get_endpoint_configuration(addr);
    if (!ep) return 0;
    return ep->data_buffer_size;
}

bool usb_is_busy(uint8_t addr) {
    struct usb_endpoint_configuration *ep = usb_get_endpoint_configuration(addr);
    if (!ep) return false;
    return ep->status == STATUS_BUSY;
}
