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

#include "config.h"
#include "system.h"
#include "lcd.h"
#include "backlight.h"
#include "backlight-target.h"
#include "pwm-imx233.h"
#include "pinctrl-imx233.h"

void _backlight_set_brightness(int brightness)
{
    /*
    imx233_pinctrl_set_gpio(3, 13, false);
    for(int i = 0; i < brightness; i++)
    {
        udelay(1);
        imx233_pinctrl_set_gpio(3, 13, false);
        udelay(1);
        imx233_pinctrl_set_gpio(3, 13, true);
    }
    */
    imx233_pinctrl_set_gpio(3, 13, true);
}

bool _backlight_init(void)
{
    imx233_pinctrl_acquire(3, 13, "backlight");
    imx233_pinctrl_set_function(3, 13, PINCTRL_FUNCTION_GPIO);
    imx233_pinctrl_enable_gpio(3, 13, true);
    imx233_pinctrl_set_gpio(3, 13, false);
    _backlight_set_brightness(DEFAULT_BRIGHTNESS_SETTING);
    return true;
}

void _backlight_on(void)
{
#ifdef HAVE_LCD_ENABLE
    lcd_enable(true); /* power on lcd + visible display */
#endif
    /* don't do anything special, the core will set the brightness */
}

void _backlight_off(void)
{
    /* there is no real on/off but we can set to 0 brightness */
    _backlight_set_brightness(0);
#ifdef HAVE_LCD_ENABLE
    lcd_enable(false); /* power off visible display */
#endif
}

/* ELE8 is the button light GPIO */
void _buttonlight_on(void)
{
    imx233_pwm_enable(0, true);
}

void _buttonlight_off(void)
{
    imx233_pwm_enable(0, false);
}

void _buttonlight_set_brightness(int brightness)
{
    imx233_pwm_setup_simple(0, 24000, 100 - brightness);
}
