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
#include "stddef.h"
#include "target.h"
#include "system.h"
#include "logf.h"

#define NVIC_ISER0  (*(volatile uint32_t *)0xe000e100)
#define NVIC_ICER0  (*(volatile uint32_t *)0xe000e180)
#define NVIC_ICPR0  (*(volatile uint32_t *)0xe000e280)

#define implement_extint(i) \
    void __attribute__((interrupt,weak)) EXTINT##i(void) {}

implement_extint(0)
implement_extint(1)
implement_extint(2)
implement_extint(3)
implement_extint(4)
implement_extint(5)
implement_extint(6)
implement_extint(8)
implement_extint(9)
implement_extint(10)
implement_extint(11)
implement_extint(12)
implement_extint(13)
implement_extint(14)
implement_extint(15)
implement_extint(16)
implement_extint(17)
implement_extint(18)

/**
 *
 * Global
 *
 */

static int g_atexit = HWSTUB_ATEXIT_OFF;

/**
 *
 * Power
 *
 */

void power_off(void)
{
}

void reset(void)
{
}

void target_init(void)
{
    /* disable all interrupt sources */
    NVIC_ICER0 = 0xffffffff;
    NVIC_ICPR0 = 0xffffffff;
    /* enable interrupts */
    __asm volatile("cpsie i\n");
}

static struct usb_resp_info_target_t g_target =
{
    .id = HWSTUB_TARGET_RKNANO,
    .name = "RkNano-B/C"
};

int target_get_info(int info, void **buffer)
{
    if(info == HWSTUB_INFO_TARGET)
    {
        *buffer = &g_target;
        return sizeof(g_target);
    }
    else
        return -1;
}

int target_atexit(int method)
{
    g_atexit = method;
    return 0;
}

void target_exit(void)
{
    switch(g_atexit)
    {
        case HWSTUB_ATEXIT_OFF:
            power_off();
            // fallthrough in case of return
        case HWSTUB_ATEXIT_REBOOT:
            reset();
            // fallthrough in case of return
        case HWSTUB_ATEXIT_NOP:
        default:
            return;
    }
}

void target_udelay(int us)
{
    /* CPU runs at ~29Minstr/s in DFU mode
     * and the loop has 3 instr/op */
    us *= 10;
    asm volatile(
        "1: cmp %[x], #0\n"
        "   sub %[x], #1\n"
        "   bne 1b\n"
        :[x]"+l"(us));
}

void target_mdelay(int ms)
{
    return target_udelay(ms * 1000);
}

void target_enable_usb_clocks(void)
{
}

void target_enable_usb_irq(void)
{
    NVIC_ISER0 = 0x80;
}

void target_disable_usb_irq(void)
{
    NVIC_ICER0 = 0x80;
}

void target_clear_usb_irq(void)
{
    NVIC_ICPR0 = 0x80;
}

void invalidate_dcache(const void* addr, uint32_t len)
{
}

void clean_dcache(const void* addr, uint32_t len)
{
}

void __attribute__((interrupt)) EXTINT7(void)
{
    usb_drv_irq();
}
