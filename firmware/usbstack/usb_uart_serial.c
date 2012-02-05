/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 * $Id$
 *
 * Copyright (C) 2007 by Christian Gmeiner
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This software is distributed on an "AS IS" basis, WITHOUT WARRANTY OF ANY
 * KIND, either express or implied.
 *
 ****************************************************************************/
#include "string.h"
#include "system.h"
#include "usb_core.h"
#include "usb_drv.h"
#include "kernel.h"
#include "usb_serial.h"
#include "usb_class_driver.h"
#define LOGF_ENABLE
#include "logf.h"
#include "uart-mioc510.h"

#define UART_PORT BLUETOOTH_UART_PORT

/* serial interface */
static struct usb_interface_descriptor __attribute__((aligned(2)))
    interface_descriptor =
{
    .bLength            = sizeof(struct usb_interface_descriptor),
    .bDescriptorType    = USB_DT_INTERFACE,
    .bInterfaceNumber   = 0,
    .bAlternateSetting  = 0,
    .bNumEndpoints      = 2,
    .bInterfaceClass    = USB_CLASS_CDC_DATA,
    .bInterfaceSubClass = 0,
    .bInterfaceProtocol = 0,
    .iInterface         = 0
};


static struct usb_endpoint_descriptor __attribute__((aligned(2)))
    endpoint_descriptor =
{
    .bLength          = sizeof(struct usb_endpoint_descriptor),
    .bDescriptorType  = USB_DT_ENDPOINT,
    .bEndpointAddress = 0,
    .bmAttributes     = USB_ENDPOINT_XFER_BULK,
    .wMaxPacketSize   = 0,
    .bInterval        = 0
};

#define BUFFER_SIZE 512
static unsigned char send_buffer[BUFFER_SIZE]
    USB_DEVBSS_ATTR __attribute__((aligned(32)));
static unsigned char receive_buffer[BUFFER_SIZE]
    USB_DEVBSS_ATTR __attribute__((aligned(32)));

static int max_pkt_size;

static int ep_in, ep_out;
static int usb_interface;

int usb_uart_serial_request_endpoints(struct usb_class_driver *drv)
{
    ep_in = usb_core_request_endpoint(USB_ENDPOINT_XFER_BULK, USB_DIR_IN, drv);
    if (ep_in < 0)
        return -1;

    ep_out = usb_core_request_endpoint(USB_ENDPOINT_XFER_BULK, USB_DIR_OUT,
            drv);
    if (ep_out < 0) {
        usb_core_release_endpoint(ep_in);
        return -1;
    }

    return 0;
}

int usb_uart_serial_set_first_interface(int interface)
{
    usb_interface = interface;
    return interface + 1;
}

int usb_uart_serial_get_config_descriptor(unsigned char *dest, int max_packet_size)
{
    unsigned char *orig_dest = dest;
    max_pkt_size = max_packet_size;

    interface_descriptor.bInterfaceNumber = usb_interface;
    PACK_DATA(dest, interface_descriptor);

    endpoint_descriptor.wMaxPacketSize = max_packet_size;

    endpoint_descriptor.bEndpointAddress = ep_in;
    PACK_DATA(dest, endpoint_descriptor);

    endpoint_descriptor.bEndpointAddress = ep_out;
    PACK_DATA(dest, endpoint_descriptor);

    return (dest - orig_dest);
}

/* called by usb_core_control_request() */
bool usb_uart_serial_control_request(struct usb_ctrlrequest* req, unsigned char* dest)
{
    bool handled = false;

    (void)dest;
    switch (req->bRequest) {
        default:
            logf("serial: unhandeld req %d", req->bRequest);
    }
    return handled;
}

void usb_uart_serial_init_connection(void)
{
    logf("uart serial: init connection");
    /*
    S3C2440_GPIO_CONFIG(GPGCON, 2, GPIO_FUNCTION);
    for(int i = 0; i < 5; i++)
    {
        GPGDAT &= ~4;
        GPGDAT |= 4;
    }
    GPGDAT &= ~4;
    */
    /* GPH10 as CLKOUT1 */
    RTCCON |= 1;
    S3C2440_GPIO_CONFIG(GPHCON, 10, GPIO_FUNCTION);
    S3C2440_GPIO_PULLUP(GPHUP, 10, GPIO_PULLUP_DISABLE);
    MISCCR &= ~0x700;
    MISCCR |= 0x200;
    /* start uart */
    uart_start_device(UART_PORT);
    uart_config(UART_PORT, 115200, 8, UART_NO_PARITY, UART_1_STOP_BIT, 1);
    /* enable bluetooth chip */
    S3C2440_GPIO_CONFIG(GPCCON, 9, GPIO_OUTPUT);
    S3C2440_GPIO_PULLUP(GPCUP, 9, GPIO_PULLUP_DISABLE);
    /*
    GPCDAT &= ~(1 << 9);
    sleep(HZ / 2);
    */
    GPCDAT |= 1 << 9;
    
    usb_drv_recv(ep_out, receive_buffer, max_pkt_size);
}

/* called by usb_code_init() */
void usb_uart_serial_init(void)
{
    logf("uart serial: init");
}

void usb_uart_serial_disconnect(void)
{
    uart_stop_device(UART_PORT);
    GPCDAT &= ~(1 << 9);
    MISCCR &= ~0x700;
    S3C2440_GPIO_CONFIG(GPHCON, 10, GPIO_INPUT);
    GPGDAT |= 4;
}

static void uart_send_completion(unsigned dev, int rc, void *buf, int sz)
{
    (void) dev;
    (void) rc;
    (void) buf;
    (void) sz;
}

/* called by usb_core_transfer_complete() */
void usb_uart_serial_transfer_complete(int ep, int dir, int status, int length)
{
    (void)ep;
    (void)length;
    (void)status;

    switch (dir) {
        case USB_DIR_OUT:
            logf("recv %d bytes", length);
            /* send it to uart */
            if(uart_send_nonblocking(UART_PORT, receive_buffer, length, &uart_send_completion) != 0)
                uart_send_completion(UART_PORT, -1, receive_buffer, 0);
            break;

        case USB_DIR_IN:
            
            break;
    }
}
