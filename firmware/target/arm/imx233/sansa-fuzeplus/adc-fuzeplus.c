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
#include "adc-target.h"
#include "adc-imx233.h"

int imx233_adc_mapping[] =
{
    [ADC_BATTERY] = IMX233_ADC_BATTERY,
    [ADC_DIE_TEMP] = IMX233_ADC_DIE_TEMP,
    [ADC_VDDIO] = IMX233_ADC_VDDIO,
    [ADC_5V] = IMX233_ADC_VDD5V,
    [ADC_BATT_TEMP] = IMX233_ADC_BATT_TEMP,
    [ADC_CH2] = HW_LRADC_CHANNEL(2),
};

const char *imx233_adc_channel_name[] =
{
    [ADC_BATTERY] = "Battery(raw)",
    [ADC_DIE_TEMP] = "Die temperature(°C)",
    [ADC_VDDIO] = "VddIO(mV)",
    [ADC_5V] = "Vdd5V(mV)",
    [ADC_BATT_TEMP] = "Battery temperature(raw)",
    [ADC_CH2] = "Channel 2",
};
