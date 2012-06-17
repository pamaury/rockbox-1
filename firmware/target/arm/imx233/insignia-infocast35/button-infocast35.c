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
#include "button-target.h"
#include "system.h"
#include "system-target.h"
#include "pinctrl-imx233.h"
#include "power-imx233.h"
#include "string.h"
#include "usb.h"
#include "touchscreen.h"
#include "touchscreen-imx233.h"

struct touch_calibration_point
{
    short px_x; /* known pixel value */
    short px_y;
    short val_x; /* touchscreen value at the known pixel */
    short val_y;
};

static struct touch_calibration_point topleft, bottomright;

/* Amaury Pouly: values on my device
 * Landscape:
 *   (0,0) = 200, 300
 *   (320,240) = 3910, 3860
*/
void button_init_device(void)
{
    topleft.val_x = 200;
    topleft.val_y = 300;

    bottomright.val_x = 3910;
    bottomright.val_y = 3860;

    topleft.px_x = 0;
    topleft.px_y = 0;

    bottomright.px_x = LCD_WIDTH;
    bottomright.px_y = LCD_HEIGHT;

    imx233_touchscreen_init();
    imx233_touchscreen_enable(true);

    /* bend button */
    imx233_pinctrl_acquire_pin(1, 30, "bend");
    imx233_set_pin_function(1, 30, PINCTRL_FUNCTION_GPIO);
    imx233_enable_gpio_output(1, 30, false);
}

static int touch_to_pixels(int *val_x, int *val_y)
{
    short x,y;

    x = *val_x;
    y = *val_y;

    x = (x - topleft.val_x) * (bottomright.px_x - topleft.px_x) / (bottomright.val_x - topleft.val_x) + topleft.px_x;
    y = (y - topleft.val_y) * (bottomright.px_y - topleft.px_y) / (bottomright.val_y - topleft.val_y) + topleft.px_y;

    x = MAX(0, MIN(x, LCD_WIDTH - 1));
    y = MAX(0, MIN(y, LCD_HEIGHT - 1));

    *val_x = x;
    *val_y = y;

    return (x<<16)|y;
}

static int touchscreen_read_device(int *data)
{
    int x, y;
    if(!imx233_touchscreen_get_touch(&x, &y))
        return 0;
    if(data)
        *data = touch_to_pixels(&x, &y);
    return /*touchscreen_to_pixels(x, y, data)*/0;
}

int button_read_device(int *data)
{
    int res = 0;
    if(!imx233_get_gpio_input_mask(1, 0x40000000))
        res |= BUTTON_MENU;
    switch(__XTRACT(HW_POWER_STS, PSWITCH))
    {
        case 1:
            res |= BUTTON_POWER;
            break;
        default:
            break;
    }
    return res | touchscreen_read_device(data);
}
