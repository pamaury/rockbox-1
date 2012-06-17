/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 * $Id$
 *
 * Copyright (c) 2012 by Amaury Pouly
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
static unsigned lcd_yuv_options = 0;

void lcd_init_device(void)
{
    imx233_pinctrl_acquire_pin(1, 18, "lcd reset");
    imx233_pinctrl_acquire_pin(1, 19, "lcd rs");
    imx233_pinctrl_acquire_pin(1, 20, "lcd wr");
    imx233_pinctrl_acquire_pin(1, 21, "lcd cs");
    imx233_pinctrl_acquire_pin(1, 23, "lcd enable");
    imx233_pinctrl_acquire_pin(1, 25, "lcd vsync");
    imx233_pinctrl_acquire_pin(1, 24, "lcd hsync");
    imx233_pinctrl_acquire_pin(1, 22, "lcd dotclk");
    imx233_pinctrl_acquire_pin_mask(1, 0x3ffff, "lcd data");

    imx233_set_pin_function(1, 24, PINCTRL_FUNCTION_MAIN); /* lcd_hsync */
    imx233_set_pin_function(1, 25, PINCTRL_FUNCTION_MAIN); /* lcd_vsync */
    imx233_set_pin_function(1, 21, PINCTRL_FUNCTION_MAIN); /* lcd_cs */
    imx233_set_pin_function(1, 22, PINCTRL_FUNCTION_MAIN); /* lcd_dotclk */
    imx233_set_pin_function(1, 23, PINCTRL_FUNCTION_MAIN); /* lcd_enable */
    imx233_set_pin_function(1, 18, PINCTRL_FUNCTION_MAIN); /* lcd_reset */
    imx233_set_pin_function(1, 19, PINCTRL_FUNCTION_MAIN); /* lcd_rs */
    imx233_set_pin_function(1, 16, PINCTRL_FUNCTION_MAIN); /* lcd_d16 */
    imx233_set_pin_function(1, 17, PINCTRL_FUNCTION_MAIN); /* lcd_d17 */
    imx233_set_pin_function(1, 20, PINCTRL_FUNCTION_MAIN); /* lcd_wr */
    __REG_CLR(HW_PINCTRL_MUXSEL(2)) = 0xffffffff; /* lcd_d{0-15} */

    /* the LCD seems to work fine at 12Mhz, so use the xtal clock with a small divider */
    imx233_clkctrl_enable_clock(CLK_PIX, false);
    imx233_clkctrl_set_clock_divisor(CLK_PIX, 2);
    imx233_clkctrl_set_bypass_pll(CLK_PIX, true); /* use XTAL */
    imx233_clkctrl_enable_clock(CLK_PIX, true);

    imx233_lcdif_reset();

    __REG_SET(HW_LCDIF_CTRL1) = HW_LCDIF_CTRL1__RESET;
    udelay(100);
    __REG_CLR(HW_LCDIF_CTRL1) = HW_LCDIF_CTRL1__RESET;
    udelay(100);
    __REG_SET(HW_LCDIF_CTRL1) = HW_LCDIF_CTRL1__RESET;

    imx233_lcdif_set_lcd_databus_width(HW_LCDIF_CTRL__LCD_DATABUS_WIDTH_18_BIT);
    imx233_lcdif_set_word_length(HW_LCDIF_CTRL__WORD_LENGTH_16_BIT);
    imx233_lcdif_set_timings(1, 1, 1, 1);
    imx233_lcdif_enable_bus_master(true);
    imx233_lcdif_set_byte_packing_format(0xf);
    imx233_lcdif_enable_dotclk(true);
    
    HW_LCDIF_VDCTRL0 = 0x1c300003;
    HW_LCDIF_VDCTRL1 = 0x000000f9;
    HW_LCDIF_VDCTRL2 = 0x28000179;
    HW_LCDIF_VDCTRL3 = 0x00340006;
    HW_LCDIF_VDCTRL4 = 0x00040140;

    imx233_lcdif_dma_send(FRAME_PHYS_ADDR, LCD_WIDTH, LCD_HEIGHT);
}

#ifdef HAVE_LCD_ENABLE
bool lcd_active(void)
{
    return /*lcd_on*/true;
}

static void lcd_enable_seq(bool enable)
{
    if(!enable)
    {
        _begin_seq()
        _end_seq()
    }
    else
    {
        _begin_seq()
        _end_seq()
    }
}

void lcd_enable(bool enable)
{
    if(lcd_on == enable)
        return;

    lcd_on = enable;

    if(enable)
        common_lcd_enable(true);
    lcd_enable_seq(enable);
    if(!enable)
        common_lcd_enable(false);
}
#endif

void lcd_update(void)
{
    lcd_update_rect(0, 0, LCD_WIDTH, LCD_HEIGHT);
}

void lcd_update_rect(int x, int y, int w, int h)
{
    if(w == LCD_WIDTH)
    {
        memcpy((void *)FRAME, FBADDR(x,y), w * h * sizeof(fb_data));
    }
    else
    {
        for(int i = 0; i < h; i++)
            memcpy((fb_data *)FRAME + i * w, FBADDR(x,y + i), w * sizeof(fb_data));
    }
}

void lcd_yuv_set_options(unsigned options)
{
    lcd_yuv_options = options;
}

#define YFAC    (74)
#define RVFAC   (101)
#define GUFAC   (-24)
#define GVFAC   (-51)
#define BUFAC   (128)

static inline int clamp(int val, int min, int max)
{
    if (val < min)
        val = min;
    else if (val > max)
        val = max;
    return val;
}

void lcd_blit_yuv(unsigned char * const src[3],
                  int src_x, int src_y, int stride,
                  int x, int y, int width, int height)
{
    const unsigned char *ysrc, *usrc, *vsrc;
    int linecounter;
    fb_data *dst, *row_end;
    long z;

    /* width and height must be >= 2 and an even number */
    width &= ~1;
    linecounter = height >> 1;

    #if LCD_WIDTH >= LCD_HEIGHT
    dst     = FBADDR(x,y);
    row_end = dst + width;
    #else
    dst     = FBADDR(LCD_WIDTH - y - 1,x);
    row_end = dst + LCD_WIDTH * width;
    #endif

    z    = stride * src_y;
    ysrc = src[0] + z + src_x;
    usrc = src[1] + (z >> 2) + (src_x >> 1);
    vsrc = src[2] + (usrc - src[1]);

    /* stride => amount to jump from end of last row to start of next */
    stride -= width;

    /* upsampling, YUV->RGB conversion and reduction to RGB565 in one go */

    do
    {
        do
        {
            int y, cb, cr, rv, guv, bu, r, g, b;

            y  = YFAC*(*ysrc++ - 16);
            cb = *usrc++ - 128;
            cr = *vsrc++ - 128;

            rv  =            RVFAC*cr;
            guv = GUFAC*cb + GVFAC*cr;
            bu  = BUFAC*cb;

            r = y + rv;
            g = y + guv;
            b = y + bu;

            if ((unsigned)(r | g | b) > 64*256-1)
            {
                r = clamp(r, 0, 64*256-1);
                g = clamp(g, 0, 64*256-1);
                b = clamp(b, 0, 64*256-1);
            }

            *dst = LCD_RGBPACK_LCD(r >> 9, g >> 8, b >> 9);

            #if LCD_WIDTH >= LCD_HEIGHT
            dst++;
            #else
            dst += LCD_WIDTH;
            #endif

            y = YFAC*(*ysrc++ - 16);
            r = y + rv;
            g = y + guv;
            b = y + bu;

            if ((unsigned)(r | g | b) > 64*256-1)
            {
                r = clamp(r, 0, 64*256-1);
                g = clamp(g, 0, 64*256-1);
                b = clamp(b, 0, 64*256-1);
            }

            *dst = LCD_RGBPACK_LCD(r >> 9, g >> 8, b >> 9);

            #if LCD_WIDTH >= LCD_HEIGHT
            dst++;
            #else
            dst += LCD_WIDTH;
            #endif
        }
        while (dst < row_end);

        ysrc    += stride;
        usrc    -= width >> 1;
        vsrc    -= width >> 1;

        #if LCD_WIDTH >= LCD_HEIGHT
        row_end += LCD_WIDTH;
        dst     += LCD_WIDTH - width;
        #else
        row_end -= 1;
        dst     -= LCD_WIDTH*width + 1;
        #endif

        do
        {
            int y, cb, cr, rv, guv, bu, r, g, b;

            y  = YFAC*(*ysrc++ - 16);
            cb = *usrc++ - 128;
            cr = *vsrc++ - 128;

            rv  =            RVFAC*cr;
            guv = GUFAC*cb + GVFAC*cr;
            bu  = BUFAC*cb;

            r = y + rv;
            g = y + guv;
            b = y + bu;

            if ((unsigned)(r | g | b) > 64*256-1)
            {
                r = clamp(r, 0, 64*256-1);
                g = clamp(g, 0, 64*256-1);
                b = clamp(b, 0, 64*256-1);
            }

            *dst = LCD_RGBPACK_LCD(r >> 9, g >> 8, b >> 9);

            #if LCD_WIDTH >= LCD_HEIGHT
            dst++;
            #else
            dst += LCD_WIDTH;
            #endif

            y = YFAC*(*ysrc++ - 16);
            r = y + rv;
            g = y + guv;
            b = y + bu;

            if ((unsigned)(r | g | b) > 64*256-1)
            {
                r = clamp(r, 0, 64*256-1);
                g = clamp(g, 0, 64*256-1);
                b = clamp(b, 0, 64*256-1);
            }

            *dst = LCD_RGBPACK_LCD(r >> 9, g >> 8, b >> 9);

            #if LCD_WIDTH >= LCD_HEIGHT
            dst++;
            #else
            dst += LCD_WIDTH;
            #endif
        }
        while (dst < row_end);

        ysrc    += stride;
        usrc    += stride >> 1;
        vsrc    += stride >> 1;

        #if LCD_WIDTH >= LCD_HEIGHT
        row_end += LCD_WIDTH;
        dst     += LCD_WIDTH - width;
        #else
        row_end -= 1;
        dst     -= LCD_WIDTH*width + 1;
        #endif
    }
    while (--linecounter > 0);

    #if LCD_WIDTH >= LCD_HEIGHT
    lcd_update_rect(x, y, width, height);
    #else
    lcd_update_rect(LCD_WIDTH - y - height, x, height, width);
    #endif
}
