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

/* Include Standard files */
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include "inttypes.h"
#include "string.h"
#include "cpu.h"
#include "system.h"
#include "kernel.h"
#include "thread.h"
#include "format.h"

#define LOGF_ENABLE
#include "logf.h"

#include "system-target.h"
#include "uart-mioc510.h"

struct uart_t
{
    bool busy;
    unsigned char *buf, *p;
    unsigned size, rem_size;
    int rc;
    completion_cb_t cb;
    bool wait;
    struct semaphore sema;
};

#define TX  0
#define RX  1

struct uart_t uart[3][2];

/* pins */
#define PIN_RX  0
#define PIN_TX  1
#define PIN_CTS 2
#define PIN_RTS 3

static struct
{
    int pin, function;
}uart_pin[3][4] =
{
    /* UART 0 */
    {
        {3, GPIO_FUNCTION}, /* RX */
        {2, GPIO_FUNCTION}, /* TX */
        {0, GPIO_FUNCTION}, /* CTS */
        {1, GPIO_FUNCTION} /* RTS */
    },
    /* UART 1 */
    {
        {5, GPIO_FUNCTION}, /* RX */
        {4, GPIO_FUNCTION}, /* TX */
        {7, GPIO_ALT_FUNCTION}, /* CTS */
        {6, GPIO_ALT_FUNCTION} /* RTS */
    },
    /* UART 2 */ 
    {
        {7, GPIO_FUNCTION}, /* RX */
        {6, GPIO_FUNCTION}, /* TX */
        {11, GPIO_FUNCTION}, /* non-existent */
        {12, GPIO_FUNCTION} /* non-existent */
    } 
};

unsigned uart_mask[3] = {UART0_MASK, UART1_MASK, UART2_MASK};

#define SUB_RX  0
#define SUB_TX  1
#define SUB_ERR 2

unsigned uart_submask[3][3] =
{
    {RXD0_SUBMASK, TXD0_SUBMASK, ERR0_SUBMASK},
    {RXD1_SUBMASK, TXD1_SUBMASK, ERR1_SUBMASK},
    {RXD2_SUBMASK, TXD2_SUBMASK, ERR1_SUBMASK},
};

unsigned uart_clkcon[3] = {CLKCON_UART0, CLKCON_UART1, CLKCON_UART2};

void uart_start_device(unsigned dev)
{
    logf("uart_start_dev(%d)", dev);
    S3C2440_GPIO_CONFIG(GPHCON, uart_pin[dev][PIN_RX].pin, uart_pin[dev][PIN_RX].function);
    S3C2440_GPIO_CONFIG(GPHCON, uart_pin[dev][PIN_TX].pin, uart_pin[dev][PIN_TX].function);
    S3C2440_GPIO_PULLUP(GPHUP, uart_pin[dev][PIN_RX].pin, GPIO_PULLUP_DISABLE);
    S3C2440_GPIO_PULLUP(GPHUP, uart_pin[dev][PIN_TX].pin, GPIO_PULLUP_DISABLE);

    for(int i = 0; i < 2; i++)
    {
        semaphore_init(&uart[dev][i].sema, 1, 0);
        uart[dev][i].buf = uart[dev][i].p = NULL;
        uart[dev][i].rem_size = uart[dev][i].size = 0;
        uart[dev][i].cb = NULL;
        uart[dev][i].busy = uart[dev][i].wait = false;
    }

    bitset32(&CLKCON, uart_clkcon[dev]);
    SRCPND = uart_mask[dev];
    INTPND = uart_mask[dev];
    for(int i = 0; i < 3; i++)
        SUBSRCPND = uart_submask[dev][i];
    bitclr32(&INTMSK, uart_mask[dev]);
    for(int i = 0; i < 3; i++)
        bitclr32(&INTSUBMSK, uart_submask[dev][i]);
}

void uart_stop_device(unsigned dev)
{
    logf("uart_stop_dev(%d)", dev);

    for(int i = 0; i < 2; i++)
    {
        if(uart[dev][i].wait)
            semaphore_release(&uart[dev][i].sema);
    }

    bitset32(&INTMSK, uart_mask[dev]);
    bitclr32(&CLKCON, uart_clkcon[dev]);
}

void uart_config(unsigned dev, unsigned speed, unsigned num_bits, unsigned parity,
    unsigned stop_bits, unsigned afc)
{
    logf("uart_config(%d,%d,%d,%d,%d,%d)", dev, speed, num_bits, parity, stop_bits, afc);
    ULCON(dev)  = (parity << 3) + (stop_bits << 2) + (num_bits-5);
    UCON(dev)   = (1 << 2) | (1 << 0);   /* enable TX, RX, use PCLK */
    UBRDIV(dev) = PCLK / (speed*16);
    UFCON(dev) = 7; /* reset & enable fifo */
    
    if(afc)
    {
        UMCON(dev) |= 1 << 4;
        S3C2440_GPIO_CONFIG(GPHCON, uart_pin[dev][PIN_CTS].pin, uart_pin[dev][PIN_CTS].function);
        S3C2440_GPIO_CONFIG(GPHCON, uart_pin[dev][PIN_RTS].pin, uart_pin[dev][PIN_RTS].function);
    }
    else
        UMCON(dev) &= ~(1 << 4);
}

static void uart_wait_tx_cb(unsigned dev, int rc, void *buf, int size)
{
    (void) buf;
    (void) size;
    uart[dev][TX].rc = rc;
    if(uart[dev][TX].wait)
        semaphore_release(&uart[dev][TX].sema);
}

static void uart_wait_rx_cb(unsigned dev, int rc, void *buf, int size)
{
    (void) buf;
    (void) size;
    uart[dev][RX].rc = rc;
    if(uart[dev][RX].wait)
        semaphore_release(&uart[dev][RX].sema);
}

static void continue_send(unsigned dev)
{
    if(uart[dev][TX].rem_size == 0)
    {
        uart[dev][TX].cb(dev, 0, uart[dev][TX].buf, uart[dev][TX].size);
        uart[dev][TX].busy = false;
        return; 
    }
    /* while buffer not full and data to send */
    while(uart[dev][TX].rem_size > 0 && !(UFSTAT(dev) & (1 << 14)))
    {
        uart[dev][TX].rem_size--;
        UTXH(dev) = *uart[dev][TX].p++;
    }
    logf("uart: rem_size=%d UFSTAT=%lx", uart[dev][TX].rem_size, UFSTAT(dev));
}

static void continue_recv(unsigned dev)
{
    (void) dev;
    return;
}

int uart_tx(unsigned dev, void *buf, int size, completion_cb_t cb, bool wait)
{
    if(uart[dev][TX].busy)
        return -1;
    logf("uart_send_nb(%d,%d)", dev, size);
    uart[dev][TX].busy = true;
    uart[dev][TX].cb = cb;
    uart[dev][TX].buf = buf;
    uart[dev][TX].size = size;
    uart[dev][TX].rem_size = size;
    uart[dev][TX].wait = wait;
    continue_send(dev);
    if(wait)
        semaphore_wait(&uart[dev][TX].sema, TIMEOUT_BLOCK);
    else
        uart[dev][TX].rc = 0;
    return uart[dev][TX].rc;
}

int uart_rx(unsigned dev, void *buf, int size, completion_cb_t cb, bool wait)
{
    if(uart[dev][RX].busy)
        return -1;
    uart[dev][RX].busy = true;
    uart[dev][RX].cb = cb;
    uart[dev][RX].buf = buf;
    uart[dev][RX].size = size;
    uart[dev][RX].rem_size = size;
    uart[dev][TX].wait = wait;
    continue_recv(dev);
    semaphore_wait(&uart[dev][RX].sema, TIMEOUT_BLOCK);
    if(wait)
        semaphore_wait(&uart[dev][RX].sema, TIMEOUT_BLOCK);
    else
        uart[dev][RX].rc = 0;
    return uart[dev][RX].rc;
}

int uart_recv_nonblocking(unsigned dev, void *buf, int size, completion_cb_t cb)
{
    return uart_rx(dev, buf, size, cb, 0);
}

int uart_send_nonblocking(unsigned dev, void *buf, int size, completion_cb_t cb)
{
    return uart_tx(dev, buf, size, cb, 0);
}

int uart_recv(unsigned dev, void *buf, int size)
{
    return uart_rx(dev, buf, size, &uart_wait_rx_cb, 1);
}

int uart_send(unsigned dev, void *buf, int size)
{
    return uart_tx(dev, buf, size, &uart_wait_tx_cb, 1);
}

static void UART(unsigned dev)
{
    logf("uart %d int", dev);
    logf("tx: %d rx: %d err: %d", !!(SUBSRCPND & uart_submask[dev][SUB_TX]),
        !!(SUBSRCPND & uart_submask[dev][SUB_RX]),
        !!(SUBSRCPND & uart_submask[dev][SUB_ERR]));

    SRCPND = uart_mask[dev];
    INTPND = uart_mask[dev];
    for(int i = 0; i < 3; i++)
        SUBSRCPND = uart_submask[dev][i];
}

void UART0(void)
{
    UART(0);
}

void UART1(void)
{
    UART(1);
}

void UART2(void)
{
    UART(2);
}
