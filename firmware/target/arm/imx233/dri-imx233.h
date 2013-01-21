/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 * $Id$
 *
 * Copyright (C) 2012 by Amaury Pouly
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
#ifndef __dri_imx233__
#define __dri_imx233__

#include "config.h"
#include "cpu.h"
#include "system.h"

#define HW_DRI_BASE             0x80074000

#define HW_DRI_CTRL             (*(volatile uint32_t *)(HW_DRI_BASE + 0x0))
#define HW_DRI_CTRL__RUN            (1 << 0)
#define HW_DRI_CTRL__ATTENTION_IRQ  (1 << 1)
#define HW_DRI_CTRL__PILOT_SYNC_LOSS_IRQ    (1 << 2)
#define HW_DRI_CTRL__OVERFLOW_IRQ   (1 << 3)
#define HW_DRI_CTRL__REACQUIRE_PHASE    (1 << 15)
#define HW_DRI_CTRL__ENABLE_INPUTS  (1 << 29)

#define HW_DRI_TIMING           (*(volatile uint32_t *)(HW_DRI_BASE + 0x10))
#define HW_DRI_TIMING__GAP_DETECTION_INTERVAL_BP    0
#define HW_DRI_TIMING__GAP_DETECTION_INTERVAL_BM    0xff
#define HW_DRI_TIMING__PILOT_REP_RATE_BP    16
#define HW_DRI_TIMING__PILOT_REP_RATE_BM    (0xf << 16)

#define HW_DRI_STAT             (*(volatile uint32_t *)(HW_DRI_BASE + 0x20))
#define HW_DRI_STAT__PILOT_PHASE_BP 16
#define HW_DRI_STAT__PILOT_PHASE_BM (0xf << 16)

#define HW_DRI_DATA             (*(volatile uint32_t *)(HW_DRI_BASE + 0x30))

#define HW_DRI_DEBUG0           (*(volatile uint32_t *)(HW_DRI_BASE + 0x40))

#define HW_DRI_DEBUG1           (*(volatile uint32_t *)(HW_DRI_BASE + 0x50))

struct imx233_dri_info_t
{
    bool running;
    bool inputs_enabled;
    bool attention;
    bool pilot_sync_loss;
    bool overflow;
    int pilot_phase;
};

void imx233_dri_init(void);
void imx233_dri_enable(bool en);
struct imx233_dri_info_t imx233_dri_get_info(void);

#endif /* __dri_imx233__ */
