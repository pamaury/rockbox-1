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
#define GINTSTS_RXFLVL  (1 << 4)
#define GINTSTS_USBRST  (1 << 12)
#define GINTSTS_ENUMDNE (1 << 13)
#define GINTSTS_IEPINT  (1 << 18)
#define GINTSTS_OEPINT  (1 << 19)
/* Interrupt Mask Register */
#define GINTMSK     REG(USB_BASE + 0x18)
/* Receive Status Read and Pop Register */
#define GRXSTSP     REG(USB_BASE + 0x20)
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
/* Device Endpoint DFIFO */
#define DFIFO(n)    REG(USB_BASE + (n) * 0x1000)

static uint32_t *ep0_xfer_buf; // current buffer pointer

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
        /* reset EP0 state */
        DIEPSIZ(0) = 0;
        DIEPCTL(0) = DIEPCTL_USBAEP | DOEPCTL_SNAK;
        DOEPSIZ(0) = 0;
        DOEPCTL(0) = DOEPCTL_USBAEP | DOEPCTL_SNAK;
        /* reset address */
        usb_drv_set_address(0);
    }
    if(gintsts & GINTSTS_ENUMDNE)
    {
    }
    if(gintsts & GINTSTS_RXFLVL)
    {
        usb_drv_exit();
#if 0
        uint32_t grxstsp = GRXSTSP;
        /* extract packet status */
        unsigned epnum = grxstsp & 0xf;
        unsigned bcnt = (grxstsp >> 4) & 0x7ff;
        /* assume interrupt is only enabled at the right time and we are not
         * writing past the buffer or things like this */
        bcnt = (bcnt + 3) / 4;
        while(bcnt-- > 0)
            *ep0_xfer_buf++ = DFIFO(epnum);
#endif
    }
    if(gintsts & GINTSTS_OEPINT)
    {
        //usb_drv_exit();
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
    GINTMSK = GINTSTS_ENUMDNE | GINTSTS_USBRST | GINTSTS_IEPINT | GINTSTS_OEPINT | GINTSTS_RXFLVL;
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
    /* prepare EP0 */
    DIEPCTL(0) = DIEPCTL_USBAEP;
    DOEPSIZ(0) = 1 << DOEPSIZ0_STUPCNT_BP | DOEPSIZ0_PKTCNT | 8 << DOEPSIZ0_XFRSIZ_BP;
    DOEPCTL(0) = DOEPCTL_USBAEP | DOEPCTL_CNAK | DOEPCTL_EPENA;
    ep0_xfer_buf = (void *)req;
    /* wait for irq */
    while(true);
}
