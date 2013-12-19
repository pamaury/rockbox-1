/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 * $Id$
 *
 * Copyright (c) 2013 by Amaury Pouly
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
#include <sys/types.h> /* off_t */
#include <string.h>
#include "cpu.h"
#include "system.h"
#include "backlight-target.h"
#include "lcd.h"
#include "lcdif-imx233.h"
#include "clkctrl-imx233.h"
#include "pinctrl-imx233.h"
#include "logf.h"

#ifdef HAVE_LCD_ENABLE
static bool lcd_on;
#endif

/* lcd offset */
#define OFFSET  2

static inline void lcd_write(bool is_data, void *data, int data_len)
{
    imx233_lcdif_pio_send(is_data, data_len, data);
}

static inline void lcd_write_cmd(uint8_t b)
{
    lcd_write(false, &b, 1);
}

static inline void lcd_write_addr(int page, int col)
{
    lcd_write_cmd(0xb0 + page); // set page address
    lcd_write_cmd(col & 0xf); // set low column address
    lcd_write_cmd(0x10 + (col >> 4)); // set high column address
}

static void lcd_init_seq(void)
{
    static uint8_t init_seq[] =
    {
        0x0a, // set lower column address: 10
        0xa1, // set segment ramp: 131 -> 0
        0xda, // set COM configuration:
        0x12, // -> use alternative
        0xc0, // set COM scan dir: normal
        0xa8, // set multiplex ratio:
        0x3f, // -> 63MUX
        0xd5, // set display clock freq:
        0x50, // -> divide ratio = 1, osc freq = 5
        0xdb, // set vdcom deselect level
        0x08, // -> 8
        0x81, // set contrast register
        0x25, // -> 0x25
        0xad, // set DCDC on/off
        0x8a, // -> off
        0xc8, // set com output scan: reverse
    };
    // blank frame, will be useful to clear the screen
    memset(FRAME, 0, IMX233_FRAMEBUFFER_SIZE);
    lcd_write(false, init_seq, ARRAYLEN(init_seq));
    for(int page = 0; page < 8; page++)
    {
        lcd_write_addr(page, 0);
        /* sends 132 bytes to fill 132x8 pixels (ie the whole page)
         * NOTE we even fill invisible pixels */
        lcd_write(true, FRAME, 132);
    }
    static uint8_t init_seq2[] =
    {
        0xaf, // turn on panel
        0x40, // set display start line: 0
        0xa4, // set entire display: normal
        0xa6, // set normal display: normal
        0xd3, // set vertical scroll:
        0x00, // -> 0
        0xd9, // set precharge period
        0x1f, // -> 31
    };
    lcd_write(false, init_seq2, ARRAYLEN(init_seq2));
    for(int page = 0; page < 8; page++)
    {
        lcd_write_addr(page, OFFSET);
        lcd_write(true, FRAME, LCD_WIDTH);
    }
}

void lcd_init_device(void)
{
    imx233_lcdif_init();
    imx233_lcdif_setup_system_pins(8);
    imx233_lcdif_set_word_length(8);
    imx233_lcdif_set_timings(4, 4, 1, 1);

    // reset device
    imx233_lcdif_reset_lcd(true);
    udelay(10);
    imx233_lcdif_reset_lcd(false);
    udelay(10);
    imx233_lcdif_reset_lcd(true);
    udelay(10);

    lcd_init_seq();
#ifdef HAVE_LCD_ENABLE
    lcd_on = true;
#endif
}

#ifdef HAVE_LCD_ENABLE
bool lcd_active(void)
{
    return lcd_on;
}

static void lcd_enable_seq(bool enable)
{
    if(enable)
    {
        lcd_write_cmd(0xaf); // turn on panel
    }
    else
    {
        lcd_write_cmd(0xae); // turn off panel
    }
}

void lcd_enable(bool enable)
{
    if(lcd_on == enable)
        return;

    lcd_on = enable;

    if(enable)
        imx233_lcdif_enable(true);
    lcd_enable_seq(enable);
    if(!enable)
        imx233_lcdif_enable(false);
    else
        send_event(LCD_EVENT_ACTIVATION, NULL);
}
#endif

void lcd_update(void)
{
    lcd_update_rect(0, 0, LCD_WIDTH, LCD_HEIGHT);
}

void lcd_update_rect(int x, int y, int w, int h)
{
#ifdef HAVE_LCD_ENABLE
    if(!lcd_on)
        return;
#endif
    /* y and h needs to be a multiple of 8 since the lcd is vertically packed */
    int ymax = (y + h - 1) / 8;
    y /= 8;

    for(; y <= ymax; y++)
    {
        lcd_write_addr(y, x + OFFSET);
        lcd_write(true, FBADDR(x, y), w);
    }
}

