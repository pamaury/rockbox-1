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
#include "button-target.h"
#include "system.h"
#include "system-target.h"
#include "pinctrl-imx233.h"
#include "power-imx233.h"
#include "string.h"
#include "usb.h"
#include "button-lradc-imx233.h"

struct imx233_button_lradc_mapping_t imx233_button_lradc_mapping[] =
{
    {450, BUTTON_LEFT},
    {800, BUTTON_MENU},
    {1255, BUTTON_RIGHT},
    {1760, BUTTON_PLAY},
    {3430, 0},
    {0, IMX233_BUTTON_LRADC_END},
};

void button_init_device(void)
{
    imx233_button_lradc_init();
    imx233_pinctrl_acquire(1, 10, "button_select");
    imx233_pinctrl_set_function(1, 10, PINCTRL_FUNCTION_GPIO);
    imx233_pinctrl_enable_gpio(1, 10, false);
    imx233_pinctrl_acquire(1, 14, "button_volup");
    imx233_pinctrl_set_function(1, 14, PINCTRL_FUNCTION_GPIO);
    imx233_pinctrl_enable_gpio(1, 14, false);
}

#ifdef HAS_BUTTON_HOLD
bool button_hold(void)
{
    return imx233_button_lradc_hold();
}
#endif

int button_read_device(void)
{
    int btn = 0;
    switch(imx233_power_read_pswitch())
    {
        case 1: btn |= BUTTON_POWER; break;
        case 3: btn |= BUTTON_VOL_DOWN; break;
    }
    if(imx233_pinctrl_get_gpio(1, 10) == 0)
        btn |= BUTTON_SELECT;
    if(imx233_pinctrl_get_gpio(1, 14) == 1)
        btn |= BUTTON_VOL_UP;
    return imx233_button_lradc_read(btn);
}
