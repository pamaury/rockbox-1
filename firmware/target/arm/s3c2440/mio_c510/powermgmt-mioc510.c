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
#include "powermgmt.h"
#include "adc.h"

/* dummy values */
const unsigned short battery_level_dangerous[BATTERY_TYPES_COUNT] =
{
    810
};

const unsigned short battery_level_shutoff[BATTERY_TYPES_COUNT] =
{
    806
};

/* voltages (millivolt) of 0%, 10%, ... 100% when charging disabled */
const unsigned short percent_to_volt_discharge[BATTERY_TYPES_COUNT][11] =
{
    { 806, 819, 833, 847, 861, 875, 889, 903, 917, 931, 945 },
};

/* voltages (millivolt) of 0%, 10%, ... 100% when charging enabled */
const unsigned short percent_to_volt_charge[11] =
{
    836, 849, 863, 877, 891, 905, 919, 933, 947, 961, 975
};

/* Returns battery voltage from ADC [millivolts] */
unsigned int battery_adc_voltage(void)
{
    return adc_read(ADC_BATTERY);
}

