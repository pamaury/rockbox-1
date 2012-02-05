/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 *
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

#ifndef __UART_MIOC510_H__
#define __UART_MIOC510_H__

#define BLUETOOTH_UART_PORT 0
#define GPS_UART_PORT       1

#define UART_NO_PARITY      0
#define UART_ODD_PARITY     4
#define UART_EVEN_PARITY    5
#define UART_MARK_PARITY    6
#define UART_SPACE_PARITY   7

#define UART_1_STOP_BIT     0
#define UART_2_STOP_BIT     1


void uart_start_device(unsigned dev);
void uart_stop_device(unsigned dev);
void uart_config(unsigned dev, unsigned speed, unsigned num_bits, unsigned parity, unsigned stop_bits, unsigned afc);

typedef void (*completion_cb_t)(unsigned dev, int rc, void *buf, int size);

int uart_send(unsigned dev, void *buf, int size);
int uart_send_nonblocking(unsigned dev, void *buf, int size, completion_cb_t cb);
int uart_recv(unsigned dev, void *buf, int size);
int uart_recv_nonblocking(unsigned dev, void *buf, int size, completion_cb_t cb);

#endif /* __UART_MIOC510_H__ */
