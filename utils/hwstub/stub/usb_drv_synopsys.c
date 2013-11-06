/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 * $Id$
 *
 * Copyright (C) 2013 by Amaury Pouly
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
#include "usb_drv.h"
#include "config.h"
#include "memory.h"
#include "target.h"

#if 1
#define REG(x)  (*(volatile uint32_t *)(x))

#define USB_DEV     (USB_BASE + 0x800)
#define USB_DEV_IEP (USB_BASE + 0x900)
#define USB_DEV_OEP (USB_BASE + 0xb00)

/* Global AHB Configuration Register */
#define GAHBCFG     REG(USB_BASE + 0x8)
#define GAHBCFG_GINT    (1 << 0)
/* Global Reset Register */
#define GRSTCTL     REG(USB_BASE + 0x10)
#define GRSTCTL_CSRST   (1 << 0)
/* Interrupt Status Register */
#define GINTSTS     REG(USB_BASE + 0x14)
#define GINTSTS_USBRST  (1 << 12)
#define GINTSTS_ENUMDNE (1 << 13)
#define GINTSTS_IEPINT  (1 << 18)
#define GINTSTS_OEPINT  (1 << 19)
/* Interrupt Mask Register */
#define GINTMSK     REG(USB_BASE + 0x18)
/* Device Configuration Register */
#define DCFG        REG(USB_DEV)
#define DCFG_DAD_BM     (0x7f << 4)
#define DCFG_DAD_BP     4
/* Device Control Register */
#define DCTL        REG(USB_DEV + 4)
#define DCTL_SDIS       (1 << 1)
/* Device OUT Endpoint Interrupt Register */
#define DIEPMSK     REG(USB_DEV + 0x10)
/* Device OUT Endpoint Interrupt Register */
#define DOEPMSK     REG(USB_DEV + 0x14)
/* Device All Endpoint Interrupt Register */
#define DAINT       REG(USB_DEV + 0x18)
#define DAINT_IEPINT(n) (1 << (n))
#define DAINT_OEPINT(n) (1 << ((n) + 16))
/* Device All Endpoint Interrupt Mask Register */
#define DAINTMSK    REG(USB_DEV + 0x1c)
/* Device Output Endpoint Control Register */
#define DOEPCTL(n)  REG(USB_DEV_OEP + 0x0 + (n) * 0x20)
#define DOEPCTL_USBAEP  (1 << 15)
#define DOEPCTL_CNAK    (1 << 26)
#define DOEPCTL_SNAK    (1 << 27)
#define DOEPCTL_EPENA   (1 << 31)
/* Device Output Endpoint Interrupt Register */
#define DOEPINT(n)  REG(USB_DEV_OEP + 0x8 + (n) * 0x20)
#define DOEPINT_XFRC    (1 << 0)
#define DOEPINT_STUP    (1 << 3)
/* Device Output Endpoint Size Register */
#define DOEPSIZ(n)  REG(USB_DEV_OEP + 0x10 + (n) * 0x20)
#define DOEPSIZ0_XFRSIZ_BP  0
#define DOEPSIZ0_XFRSIZ_BM  (0x3f << 0)
#define DOEPSIZ0_PKTCNT     (1 << 19)
#define DOEPSIZ0_STUPCNT_BM (3 << 29)
#define DOEPSIZ0_STUPCNT_BP 29
/* Device Output Endpoint Control Register */
#define DIEPCTL(n)  REG(USB_DEV_IEP + 0x0 + (n) * 0x20)
#define DIEPCTL_USBAEP  (1 << 15)
#define DIEPCTL_CNAK    (1 << 26)
#define DIEPCTL_SNAK    (1 << 27)
#define DIEPCTL_EPENA   (1 << 31)
/* Device Input Endpoint Interrupt Register */
#define DIEPINT(n)  REG(USB_DEV_IEP + 0x8 + (n) * 0x20)
#define DIEPINT_XFRC    (1 << 0)

void usb_drv_set_address(int address)
{
    DCFG = (DCFG & ~DCFG_DAD_BM) | address << DCFG_DAD_BP;
}

int usb_drv_send(int endpoint, void* ptr, int length)
{
    return -1;
}

int usb_drv_recv(int endpoint, void* ptr, int length)
{
    return -1;
}


int usb_drv_port_speed(void)
{
    return 0; /* HS only */
}

void usb_drv_stall(int endpoint, bool stall, bool in)
{
}

void usb_drv_irq(void)
{
    uint32_t gintsts = GINTSTS;
    if(gintsts & GINTSTS_USBRST)
    {
        usb_drv_set_address(0);
    }
    if(gintsts & GINTSTS_ENUMDNE)
    {
        /* prepare EP0 */
        DIEPCTL(0) = DIEPCTL_USBAEP;
        DOEPSIZ(0) = 1 << DOEPSIZ0_STUPCNT_BP | DOEPSIZ0_PKTCNT | 8 << DOEPSIZ0_XFRSIZ_BP;
        DOEPCTL(0) = DOEPCTL_USBAEP | DOEPCTL_CNAK | DOEPCTL_EPENA;
    }
    if(gintsts & GINTSTS_OEPINT)
    {
        usb_drv_exit();
    }
    if(gintsts & GINTSTS_IEPINT)
    {
    }
    GINTSTS = gintsts;
    target_clear_usb_irq();
}

void usb_drv_init(void)
{
    /* reset core */
    //GRSTCTL |= GRSTCTL_CSRST;
    //target_mdelay(10);
    /* disconnect */
    DCTL |= DCTL_SDIS;
    target_mdelay(10);
    /* disable irq */
    GAHBCFG &= ~GAHBCFG_GINT;
    /* unmask enum done and reset interrupts */
    GINTMSK = GINTSTS_ENUMDNE | GINTSTS_USBRST | GINTSTS_IEPINT | GINTSTS_OEPINT;
    GINTSTS = 0xffffffff;
    DOEPMSK = DOEPINT_STUP | DOEPINT_XFRC;
    DOEPINT(0) = 0xffffffff;
    DIEPMSK = DIEPINT_XFRC;
    DIEPINT(0) = 0xffffffff;
    DAINTMSK = DAINT_IEPINT(0) | DAINT_OEPINT(0);
    DAINT = 0xffffffff;
    /* enable irq */
    target_enable_usb_irq();
    GAHBCFG |= GAHBCFG_GINT;
    /* reconnect */
    DCTL &= ~DCTL_SDIS;
}

void usb_drv_exit(void)
{
    /* disconnect */
    DCTL |= DCTL_SDIS;
}

int usb_drv_recv_setup(struct usb_ctrlrequest *req)
{
    /* wait for irq */
    while(true);
}
#endif

#if 0
#include "synopsysotg.h"

union usb_ep0_buffer ep0_buffer;

static struct synopsysotg_state state =
{
    .endpoints = { {}, {} },
};

static struct synopsysotg_config config =
{
    .core = (struct synopsysotg_core_regs*)USB_BASE,
    .phy_16bit = true,
    .phy_ulpi = false,
    .use_dma = false,
    .disable_double_buffering = true,
    .fifosize = 512,
    .txfifosize = {64, 64, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
};

static struct usb_instance instance =
{
    .driver = &synopsysotg_driver,
    .driver_config = &config,
    .driver_state = &state,
    .buffer = &ep0_buffer
};

void synopsysotg_target_enable_clocks(const struct usb_instance* instance)
{
    target_enable_usb_clocks();
}

void synopsysotg_target_enable_irq(const struct usb_instance* instance)
{
    target_enable_usb_irq();
}

void synopsysotg_target_disable_irq(const struct usb_instance* instance)
{
    target_disable_usb_irq();
}

void synopsysotg_target_clear_irq(const struct usb_instance* instance)
{
    target_clear_usb_irq();
}

void usb_drv_irq()
{
    synopsysotg_irq(&instance);
}

static int speed = 0;

#define EP(x, in) (union usb_endpoint_number){.number = x, .direction = in}

void usb_handle_bus_reset(const struct usb_instance* data, int highspeed)
{
    speed = highspeed;
}

void usb_handle_timeout(const struct usb_instance* data, union usb_endpoint_number epnum, int bytesleft)
{
    usb_drv_exit();
}

void usb_handle_xfer_complete(const struct usb_instance* data, union usb_endpoint_number epnum, int bytesleft)
{
    usb_drv_exit();
}

void usb_handle_setup_received(const struct usb_instance* data, union usb_endpoint_number epnum, int back2back)
{
    usb_drv_exit();
}

void usb_ep0_expect_setup(const struct usb_instance* data)
{
    instance.driver->set_stall(data, EP(0, true), 1);
    // Set up the OUT pipe for the SETUP packet, STALLing everything else.
    instance.driver->ep0_start_rx(data, 0);
}

void usb_drv_set_address(int address)
{
    instance.driver->set_address(&instance, address);
}

int usb_drv_send(int endpoint, void* ptr, int length)
{
    return -1;
}

int usb_drv_recv(int endpoint, void* ptr, int length)
{
    return -1;
}


int usb_drv_port_speed(void)
{
    return speed;
}

void usb_drv_stall(int endpoint, bool stall, bool in)
{
    instance.driver->set_stall(&instance, EP(endpoint, in), stall);
}


void usb_drv_init(void)
{
    instance.driver->init(&instance);
}

void usb_drv_exit(void)
{
    target_mdelay(10);
    volatile uint32_t *reg = (volatile uint32_t *)(USB_BASE + 0x804);
    *reg |= 2;
    while(1);
}

int usb_drv_recv_setup(struct usb_ctrlrequest *req)
{
    while(1);
}
#endif
