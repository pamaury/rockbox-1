/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 * $Id$
 *
 * Copyright (C) 2011 by Amaury Pouly
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
#include "system.h"
#include "config.h"
#include "string.h"
#include "usb_ch9.h"
#include "usb_core.h"
#include "kernel.h"
#include "panic.h"
#include "usb_drv.h"
#include "power.h"

#define LOGF_ENABLE
#include "logf.h"

/***** IMPORTANT NOTES
 * Here are random notes of the usb controller which are important to understand
 * the driver:
 *
 * - This controller has a very special way of handling control transfers,
 *   especially acknowledgement. Instead of using ZLP which complete normally,
 *   a controls transfer ack as soon as a transfers has the DATA_END bit set.
 *   In such a situation, the control transfers ends the DATA_END bit is *cleared*.
 *   Even worse, when sending a ZLP by setting DATA_AND | IN_PKT_RDY, IN_PKT_RDY
 *   will never be cleared so transfer completion is not a reliable way of handling
 *   control transfer end. This makes it necessary to use a FSM for the EP0 state.
 * - The set address request needs to be handled
 *   early otherwise the usb_core will set address *after* sending the last ZLP
 *   which will confume the controller
 * - At any point the controller can set SETUP_END meaning the control transfer
 *   has ended, it can happen even before the core has time to handle the request
 *   and thus the core might send/recv data *after* control transfer has ended.
 * - The set configuration request has some automagic which automically triggers
 *   SETUP_END when the core read the control packet
 */

#define USB_EPx_MAX_PACKET_SIZE 64

struct endpoint_t
{
    bool active;
    int type;
    bool busy;
    int rc;
    unsigned char *buffer;
    int size;
    int rem_size;
    bool wait;
    struct semaphore complete;
    int max_pkt_size;
};

enum ep0_state_t
{
    WAIT_SETUP,
    IN_PHASE,
    OUT_PHASE,
    WAIT_END
};

/** NOTE
 * EP0 is bidirectional but all other EPx are unidirectional with programmable
 * direction. To keep it simple, we define structures for full bidirectional
 * endpoints but ensure only directional is active at the same time */
static struct endpoint_t endpoints[USB_NUM_ENDPOINTS][2];
static struct usb_ctrlrequest setup_pkt USB_DEVBSS_ATTR;
static enum ep0_state_t ep0_state;

static void usb_hw_connect(void)
{
    #ifdef MIO_C510
    GPJDAT |= 1 << 9;
    #endif
}

static void usb_hw_disconnect(void)
{
    #ifdef MIO_C510
    GPJDAT &= ~(1 << 9);
    #endif
}

int usb_drv_port_speed(void)
{
    return 0; /* Full-Speed only */
}

int usb_drv_request_endpoint(int type, int dir)
{
    int dir_in = dir == USB_DIR_IN ? DIR_IN : DIR_OUT;
    
    for(int i = 1; i < USB_NUM_ENDPOINTS; i++)
    {
        if(endpoints[i][dir_in].active || endpoints[i][1 - dir_in].active)
            continue;
        if(endpoints[i][1 - dir_in].active && endpoints[i][1 - dir_in].type != type)
            continue;
        endpoints[i][dir_in].active = true;
        endpoints[i][dir_in].type = type;
        INDEX_REG = i;
        IN_CSR2_REG = (dir_in ? MODE_IN : 0);
        MAXP_REG = EPx_MAXP(USB_EPx_MAX_PACKET_SIZE);
        EP_INT_REG = EPx_INTERRUPT(i);
        EP_INT_EN_REG |= EPx_INTERRUPT(i);
        logf("request ep %d %d => %d", type, dir, i | dir);
        return i | dir;
    }
    logf("request ep %d %d => -1", type, dir);
    return -1;
}

void usb_drv_release_endpoint(int ep)
{
    endpoints[EP_NUM(ep)][EP_DIR(ep)].active = false;
}

void usb_drv_set_address(int address)
{
    (void) address;
    /* handled in IRQ */
}

static void complete_transfer(int ep, bool in, int rc)
{
    logf("complete xfer %d %d %d sz=%d rsz=%d", ep, in, rc, endpoints[ep][in].size,
        endpoints[ep][in].rem_size);
    endpoints[ep][in].buffer = NULL;
    endpoints[ep][in].busy = false;
    endpoints[ep][in].rc = rc;
    usb_core_transfer_complete(ep, in ? USB_DIR_IN : USB_DIR_OUT, rc,
        endpoints[ep][in].size - endpoints[ep][in].rem_size);
    if(endpoints[ep][in].wait)
    {
        endpoints[ep][in].wait = false;
        semaphore_release(&endpoints[ep][in].complete);
    }
}

static void cancel_transfer(int ep, bool in)
{
    if(!endpoints[ep][in].busy)
        return;
    if(ep != 0)
    {
        INDEX_REG = ep;
        if(in)
            IN_CSR1_REG = IN_FIFO_FLUSH;
        else
            OUT_CSR1_REG = OUT_FIFO_FLUSH;
    }
    complete_transfer(ep, in, -1);
}

static unsigned usb_fifo_size_out(int ep)
{
    INDEX_REG = ep;
    return OUT_FIFO_CNT1_REG | (OUT_FIFO_CNT2_REG << 8);
}

unsigned usb_fifo_read(int ep, void *ptr, unsigned size)
{
    unsigned fifo_size = usb_fifo_size_out(ep);
    if(fifo_size > size)
        size = fifo_size;
    INDEX_REG = ep;
    volatile unsigned char *reg = &EPx_FIFO(ep);
    while(fifo_size-- > 0)
        *(unsigned char *)ptr++ = *reg;
    return size;
}

static void continue_ep_send(int ep)
{
    if(endpoints[ep][DIR_IN].rem_size == 0)
        return complete_transfer(ep, true, 0);

    unsigned sz = MIN(endpoints[ep][DIR_IN].rem_size, endpoints[ep][DIR_IN].max_pkt_size);
    endpoints[ep][DIR_IN].rem_size -= sz;
    logf("ep %d send %d rem %d mps=%d", ep, sz, endpoints[ep][DIR_IN].rem_size,
        endpoints[ep][DIR_IN].max_pkt_size);
    while(sz-- > 0)
        EPx_FIFO(ep) = *endpoints[ep][DIR_IN].buffer++;
    INDEX_REG = ep;
    if(ep == 0)
    {
        if(endpoints[ep][DIR_IN].rem_size == 0)
        {
            EP0_CSR = IN_PKT_RDY | DATA_END;
            ep0_state = WAIT_END;
        }
        else
            EP0_CSR = IN_PKT_RDY;
    }
    else
        IN_CSR1_REG |= IN_PKT_RDY_EPx;
}

static void continue_ep_recv(int ep)
{
    INDEX_REG = ep;
    if(!(OUT_CSR1_REG & OUT_PKT_RDY))
        return;
    int fifo_size = usb_fifo_size_out(ep);
    /* note: fifo_size can be zero ? */
    if(fifo_size > endpoints[ep][DIR_OUT].rem_size)
    {
        /* overflow: cancel and stall */
        cancel_transfer(ep, false);
        usb_drv_stall(ep, true, false);
        return;
    }
    /* read and advance */
    fifo_size = MIN(fifo_size, endpoints[ep][DIR_OUT].max_pkt_size);
    usb_fifo_read(ep, endpoints[ep][DIR_OUT].buffer, fifo_size);
    endpoints[ep][DIR_OUT].buffer += fifo_size;
    endpoints[ep][DIR_OUT].rem_size -= fifo_size;

    bool finished = (fifo_size < endpoints[ep][DIR_OUT].max_pkt_size)
                || endpoints[ep][DIR_OUT].rem_size == 0;

    logf("ep %d recv %d rem %d finished=%d", ep, fifo_size, endpoints[ep][DIR_OUT].rem_size,
        finished);

    INDEX_REG = ep;
    if(ep == 0)
    {
        if(finished)
            EP0_CSR = SRV_OUT_PKT_RDY | DATA_END;
        else
            EP0_CSR = SRV_OUT_PKT_RDY;
    }
    else
        OUT_CSR1_REG &= ~OUT_PKT_RDY;

    if(finished)
        complete_transfer(ep, false, 0);
}

int usb_drv_tx(int ep, void *ptr, int length, bool wait)
{
    ep = EP_NUM(ep);
    if(endpoints[ep][DIR_IN].busy)
        return -1;
    /* see to remark: control transfer might end before core has time to send/recv */
    if(ep == 0 && ep0_state == WAIT_SETUP)
        return -1;
    logf(">> usb_drv_send(%d,%d,%d)", ep, length, wait);

    endpoints[ep][DIR_IN].busy = true;
    endpoints[ep][DIR_IN].buffer = ptr;
    endpoints[ep][DIR_IN].size = length;
    endpoints[ep][DIR_IN].rem_size = length;
    endpoints[ep][DIR_IN].wait = wait;

    int flags = disable_irq_save();
    if(length == 0)
    {
        /* ZLP */
        INDEX_REG = ep;
        if(ep == 0)
        {
            EP0_CSR = IN_PKT_RDY | DATA_END;
            ep0_state = WAIT_END;
        }
        else
            IN_CSR1_REG = IN_PKT_RDY_EPx;
    }
    else
        continue_ep_send(ep);
    restore_irq(flags);
    if(wait)
        semaphore_wait(&endpoints[ep][DIR_IN].complete, TIMEOUT_BLOCK);
    else
        endpoints[ep][DIR_IN].rc = 0;
    logf("<< usb_drv_send(%d,%d,%d) = %d", ep, length, wait, endpoints[ep][DIR_IN].rc);
    return endpoints[ep][DIR_IN].rc;
}

int usb_drv_send_nonblocking(int ep, void *ptr, int length)
{
    return usb_drv_tx(ep, ptr, length, false);
}

int usb_drv_send(int ep, void *ptr, int length)
{
    return usb_drv_tx(ep, ptr, length, true);
}

int usb_drv_recv(int ep, void* ptr, int length)
{
    if(endpoints[ep][DIR_OUT].busy)
        return -1;
    /* see to remark: control transfer might end before core has time to send/recv */
    if(ep == 0 && ep0_state == WAIT_SETUP)
        return -1;

    int flags = disable_irq_save();
    endpoints[ep][DIR_OUT].busy = true;
    endpoints[ep][DIR_OUT].buffer = ptr;
    endpoints[ep][DIR_OUT].size = length;
    endpoints[ep][DIR_OUT].rem_size = length;
    endpoints[ep][DIR_OUT].wait = false;
    
    continue_ep_recv(ep);
    restore_irq(flags);
    
    return 0;
}

void cancel_all_transfers(bool ep0)
{
    int flags = disable_irq_save();
    for(int i = 0; i < USB_NUM_ENDPOINTS; i++)
    {
        if(i == 0 && !ep0)
            continue;
        cancel_transfer(0, true);
        cancel_transfer(0, false);
    }
    restore_irq(flags);
}

void usb_drv_cancel_all_transfers(void)
{
    cancel_all_transfers(false);
}

void usb_drv_set_test_mode(int mode)
{
    (void)mode;
}

bool usb_drv_stalled(int ep, bool in)
{
    ep = EP_NUM(ep);
    INDEX_REG = ep;
    if(ep == 0)
        return (EP0_CSR & SEND_STALL);
    else if(in)
        return (IN_CSR1_REG & SEND_STALL_IN);
    else
        return (IN_CSR1_REG & SEND_STALL_OUT);
}

void usb_drv_stall(int ep, bool stall, bool in)
{
    logf("stall ep=%d stall=%d in=%d", ep, stall, in);
    ep = EP_NUM(ep);
    if(ep == 0)
    {
        INDEX_REG = ep;
        if(stall)
            EP0_CSR = SEND_STALL;
        else
            EP0_CSR &= ~SEND_STALL;
    }
    else
    {
        INDEX_REG = ep;
        if(in)
        {
            if(stall)
                IN_CSR1_REG |= SEND_STALL_IN;
            else
                IN_CSR1_REG &= ~SEND_STALL_IN;
        }
        else
        {
            if(stall)
                OUT_CSR1_REG |= SEND_STALL_OUT;
            else
                OUT_CSR1_REG &= ~SEND_STALL_OUT;
        }
    }
}

static void usb_drv_reset(void)
{
    cancel_all_transfers(true);
    /* Enable interrupt on EP0 */
    EP_INT_REG = EPx_INTERRUPT(0);
    EP_INT_EN_REG = EPx_INTERRUPT(0);
    /* Enable reset interrupt */
    USB_INT_REG = RESET_INT;
    USB_INT_EN_REG = RESET_INT;
    /* Setup max packet size for EP0 */
    INDEX_REG = 0;
    MAXP_REG = EPx_MAXP(USB_EP0_MAX_PACKET_SIZE);
    ep0_state = WAIT_SETUP;
}

void USBD(void)
{
    /* save index register and restore at the end */
    int index = INDEX_REG;
    
    if(USB_INT_REG & RESET_INT)
    {
        logf("reset int");
        /* note: usb_drv_reset() will acknowledge interrupt */
        usb_drv_reset();
        usb_core_bus_reset();
    }
    if(EP_INT_REG & EPx_INTERRUPT(0))
    {
        logf("EP0 int, state=%d", ep0_state);
        /* acknowledge */
        EP_INT_REG = EPx_INTERRUPT(0);
        INDEX_REG = 0;
        unsigned long ep0_csr = EP0_CSR;
        logf("ep0_csr=%lx", ep0_csr);

        if(ep0_csr & SETUP_END)
        {
            logf("setup end");
            /* acknowledge */
            INDEX_REG = 0;
            EP0_CSR = SRV_SETUP_END;
            cancel_transfer(0, true);
            cancel_transfer(0, false);
            ep0_state = WAIT_SETUP;
        }
        if(ep0_csr & SENT_STALL)
        {
            logf("sent stall");
            /* acknowledge */
            INDEX_REG = 0;
            EP0_CSR &= ~SENT_STALL;
        }
        switch(ep0_state)
        {
            case IN_PHASE:
                if(!(ep0_csr & IN_PKT_RDY) && endpoints[0][DIR_IN].busy)
                {
                    logf("continue send ep0");
                    continue_ep_send(0);
                }
                break;
            case OUT_PHASE:
                if(ep0_csr & OUT_PKT_RDY && endpoints[0][DIR_OUT].busy)
                {
                    logf("continue recv ep0");
                    continue_ep_recv(0);
                }
                break;
            case WAIT_END:
                if(!(ep0_csr & DATA_END))
                {
                    logf("complete setup ep0");
                    if(endpoints[0][DIR_IN].busy) complete_transfer(0, true, 0);
                    if(endpoints[0][DIR_OUT].busy) complete_transfer(0, false, 0);
                    ep0_state = WAIT_SETUP;
                }
                break;
            case WAIT_SETUP:
            default:
                logf("packet ready ep0");
                if(usb_fifo_read(0, &setup_pkt, sizeof(setup_pkt)) == sizeof(setup_pkt))
                {
                    /* early handle of set address */
                    if(setup_pkt.bRequestType == (USB_TYPE_STANDARD | USB_RECIP_DEVICE) &&
                            setup_pkt.bRequest == USB_REQ_SET_ADDRESS)
                    {
                        logf("set address %d", setup_pkt.wValue);
                        FUNC_ADDR_REG = setup_pkt.wValue | ADDR_UPDATE;
                    }
                    usb_core_control_request(&setup_pkt);
                    if (setup_pkt.bRequestType & USB_DIR_IN)
                        ep0_state = IN_PHASE;
                    else
                        ep0_state = OUT_PHASE;
                }
                EP0_CSR = SRV_OUT_PKT_RDY;
                break;
        }
    }
    for(int i = 1; i < USB_NUM_ENDPOINTS; i++)
    {
        if(!(EP_INT_REG & EPx_INTERRUPT(i)))
            continue;
        logf("EP%d int", i);
        INDEX_REG = i;
        logf("in csr1: %x", IN_CSR1_REG);
        logf("out csr1: %x", OUT_CSR1_REG);
        EP_INT_REG = EPx_INTERRUPT(i);
        INDEX_REG = i;
        unsigned in_csr = IN_CSR1_REG;
        unsigned out_csr = OUT_CSR1_REG;
        if(in_csr & SENT_STALL_IN)
            IN_CSR1_REG &= ~SENT_STALL_IN;
        if(out_csr & SENT_STALL_OUT)
            OUT_CSR1_REG &= ~SENT_STALL_OUT;
        if(endpoints[i][DIR_IN].busy && !(in_csr & IN_PKT_RDY_EPx))
            continue_ep_send(i);
        if(endpoints[i][DIR_OUT].busy && (out_csr & OUT_PKT_RDY))
            continue_ep_recv(i);
    }
    INDEX_REG = index;
    /* Acknowledge */
    SRCPND = USBD_MASK;
    INTPND = USBD_MASK;
}

void usb_drv_init(void)
{
    logf("usb: init");
    /* Power up */
    bitset32(&CLKCON, CLKCON_USBD);
    MISCCR &= ~0x3008;

    usb_drv_reset();

    memset(endpoints, 0, sizeof(endpoints));
    endpoints[0][DIR_IN].max_pkt_size = USB_EP0_MAX_PACKET_SIZE;
    endpoints[0][DIR_OUT].max_pkt_size = USB_EP0_MAX_PACKET_SIZE;
    for(int i = 1; i < USB_NUM_ENDPOINTS; i++)
    {
        endpoints[i][DIR_IN].max_pkt_size = USB_EPx_MAX_PACKET_SIZE;
        endpoints[i][DIR_OUT].max_pkt_size = USB_EPx_MAX_PACKET_SIZE;
    }
    for(int i = 0; i < USB_NUM_ENDPOINTS; i++)
    {
        semaphore_init(&endpoints[i][DIR_IN].complete, 1, 0);
        semaphore_init(&endpoints[i][DIR_OUT].complete, 1, 0);
    }

    /* Connect */
    usb_hw_connect();
    /* Enable usb interrupt */
    SRCPND = USBD_MASK;
    INTPND = USBD_MASK;
    bitclr32(&INTMSK, USBD_MASK);
}

void usb_drv_exit(void)
{
    logf("usb: exit");
    /* Disable usb interrupt */
    bitset32(&INTMSK, USBD_MASK);
    /* Disconnect */
    usb_hw_disconnect();
    /* Power down */
    bitclr32(&CLKCON, CLKCON_USBD);
    MISCCR |= 0x3000;
}

void usb_init_device(void)
{
    logf("ep0 max packet size: %d", USB_EP0_MAX_PACKET_SIZE);
}

void usb_enable(bool on)
{
    if(on) usb_core_init();
    else usb_core_exit();
}

void usb_attach(void)
{
    usb_enable(true);
}

/** USB detection */
int usb_status = USB_EXTRACTED;

void usb_drv_usb_detect_event(void)
{
    usb_status = USB_INSERTED;
}

int usb_detect(void)
{
    return charger_inserted() ? USB_INSERTED : USB_EXTRACTED;
}
