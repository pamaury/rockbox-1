/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 * $Id$
 *
 * Copyright (c) 2011 by Amaury Pouly
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
#include "config.h"
#include "cpu.h"
#include <stdbool.h>
#include <stdio.h>
#include "kernel.h"
#include "system.h"
#include "power.h"
#include "led-mioc510.h"

/** Related pins
 * - GPG1: input (low when charger present)
 * - GPG8: input (usb related ?)
 * - GPG9: input (low when charging ?)
 * - GPG10: input (unknown)
 * - GPG11: output (charging led, green=high / orange=low, other effect ?)
 * - GPB9: output (unknown)
 */

unsigned int power_input_status(void)
{
    return !(GPGDAT & (1 << 1)) ? POWER_INPUT_USB_CHARGER : POWER_INPUT_NONE;
}

bool charging_state(void)
{
    return !!(GPGDAT & (1 << 9));
}

void power_init(void)
{
    S3C2440_GPIO_CONFIG(GPGCON, 1, GPIO_INPUT);
    S3C2440_GPIO_CONFIG(GPGCON, 8, GPIO_INPUT);
    S3C2440_GPIO_CONFIG(GPGCON, 9, GPIO_INPUT);
    S3C2440_GPIO_CONFIG(GPGCON, 10, GPIO_INPUT);
    S3C2440_GPIO_CONFIG(GPGCON, 11, GPIO_OUTPUT);
    S3C2440_GPIO_CONFIG(GPBCON, 9, GPIO_OUTPUT);
}

void power_off(void)
{
    /* we don't have any power control, user must do it */
    //led_flash (LED_NONE, LED_ALL);
    while(1);
}
