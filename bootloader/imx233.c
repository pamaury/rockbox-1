/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 * $Id$
 *
 * Copyright (C) 2011 by amaury Pouly
 *
 * Based on Rockbox iriver bootloader by Linus Nielsen Feltzing
 * and the ipodlinux bootloader by Daniel Palffy and Bernard Leach
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

#include <stdio.h>
#include <system.h>
#include <inttypes.h>
#include "config.h"
#include "gcc_extensions.h"
#include "lcd.h"
#include "backlight.h"
#include "button-target.h"
#include "common.h"
#include "storage.h"
#include "disk.h"
#include "panic.h"
#include "power.h"
#include "power-imx233.h"
#include "system-target.h"
#include "fmradio_i2c.h"
#include "version.h"
#include "powermgmt.h"
#include "partitions-imx233.h"
#include "adc-imx233.h"
#include "dir.h"
#include "usb.h"

extern char loadaddress[];
extern char loadaddressend[];

#ifdef HAVE_BOOTLOADER_USB_MODE
static void usb_mode(int connect_timeout)
{
    int button;
    
    usb_init();
    usb_start_monitoring();

    /* Wait for threads to connect or cable is pulled */
    printf("USB: Connecting");

    long end_tick = current_tick + connect_timeout;

    while(1)
    {
        button = button_get_w_tmo(HZ/10);

        if(button == SYS_USB_CONNECTED)
            break; /* Hit */

        if(TIME_AFTER(current_tick, end_tick))
        {
            /* Timed out waiting for the connect - will happen when connected
             * to a charger through the USB port */
            printf("USB: Timed out");
            break;
        }

        if(usb_detect() == USB_EXTRACTED)
            break; /* Cable pulled */
    }

    if(button == SYS_USB_CONNECTED)
    {
        /* Got the message - wait for disconnect */
        printf("Bootloader USB mode");
        /* Enable power management to charge */
        powermgmt_init();
        adc_init();

        usb_acknowledge(SYS_USB_CONNECTED_ACK);

        while(1)
        {
            button = button_get_w_tmo(HZ/2);
            if(button == SYS_USB_DISCONNECTED)
                break;
            struct imx233_powermgmt_info_t info = imx233_powermgmt_get_info();
            lcd_putsf(0, 7, "Charging status: %s",
                info.state == CHARGE_STATE_DISABLED ? "disabled" :
                info.state == CHARGE_STATE_ERROR ? "error" :
                info.state == DISCHARGING ? "discharging" :
                info.state == TRICKLE ? "trickle" :
                info.state == TOPOFF ? "topoff" :
                info.state == CHARGING ? "charging" : "<unknown>");
            lcd_putsf(0, 8, "Battery: %d%% (%d mV)", battery_level(), battery_voltage());
            lcd_putsf(0, 9, "Die temp: %d°C [%d, %d]",
                adc_read(ADC_DIE_TEMP), IMX233_DIE_TEMP_HIGH,
                IMX233_DIE_TEMP_LOW);
            #ifdef ADC_BATT_TEMP
            lcd_putsf(0, 10, "Batt temp: %d [%d, %d]",
                adc_read(ADC_BATT_TEMP), IMX233_BATT_TEMP_HIGH,
                IMX233_BATT_TEMP_LOW);
            #endif
            lcd_update();
        }
    }

    /* Put drivers initialized for USB connection into a known state */
    usb_close();
}
#else /* !HAVE_BOOTLOADER_USB_MODE */
static void usb_mode(int connect_timeout)
{
    (void) connect_timeout;
}
#endif /* HAVE_BOOTLOADER_USB_MODE */

int do_bootmenu(void)
{
    /* Wait until the power button it is released.
     * This allows to stop boot and read the screen for example.
     * If the button was hold for 1 second, trigger the boot menu. */
    bool boot_menu = false;
    unsigned timeout = current_tick + HZ;
    while(button_read_device() & BUTTON_POWER)
        if(TIME_AFTER(current_tick, timeout))
            boot_menu = true;

    if(!boot_menu)
        return 0; // boot from first device

#ifdef SANSA_FUZEPLUS
    unsigned bm_next = BUTTON_VOL_DOWN;
    const char str_next[] = "Vol-";
    unsigned bm_prev = BUTTON_VOL_UP;
    const char str_prev[] = "Vol+";
    unsigned bm_select = BUTTON_POWER;
    const char str_select[] = "Power";
#else
#error Please define bootmenu keymap!
#endif

    bool last_next = false;
    bool last_prev = false;
    bool last_select = false;
    int cur_choice = 0;
    
    while(1)
    {
#define COLOR_BOOTMENU  LCD_RGBPACK(255, 0, 0)
#define COLOR_ACTION    LCD_RGBPACK(255, 0, 255)
#define COLOR_BUTTON    LCD_RGBPACK(255, 255, 0)
        lcd_clear_display();
        int line = 0;
        lcd_set_background(LCD_BLACK);
        lcd_set_foreground(COLOR_BOOTMENU);
        lcd_putsf(0, line++, "Boot menu");
        lcd_set_foreground(COLOR_ACTION); lcd_putsf(0, line, "Next  : ");
        lcd_set_foreground(COLOR_BUTTON); lcd_putsf(8, line++, "%s", str_next);
        if(bm_prev)
        {
            lcd_set_foreground(COLOR_ACTION); lcd_putsf(0, line, "Prev  : ");
            lcd_set_foreground(COLOR_BUTTON); lcd_putsf(8, line++, "%s", str_prev);
        }
        lcd_set_foreground(COLOR_ACTION); lcd_putsf(0, line, "Select: ");
        lcd_set_foreground(COLOR_BUTTON); lcd_putsf(8, line++, "%s", str_select);
        
        for(int i = 0; i < storage_num_drives(); i++)
        {
            lcd_set_foreground(LCD_WHITE);
            lcd_set_background(LCD_BLACK);
            lcd_putsf(0, line, "%d) ", i);
            if(cur_choice == i)
            {
                lcd_set_foreground(LCD_BLACK);
                lcd_set_background(LCD_WHITE);
            }
            struct storage_info info;
            storage_get_info(i, &info);
            bool not_present = storage_removable(i) && !storage_present(i);
            lcd_putsf(4, line++, "%s %s", info.product, not_present ? "(not present)" : "");
        }
        lcd_update();
        lcd_set_foreground(LCD_WHITE);
        lcd_set_background(LCD_BLACK);

        unsigned button = button_read_device();

        if(last_next && !(button & bm_next))
            cur_choice = (cur_choice + 1) % storage_num_drives();
        if(last_prev && !(button & bm_prev))
            cur_choice = (cur_choice + storage_num_drives() - 1) % storage_num_drives();
        if(last_select && !(button & bm_select))
        {
            lcd_clear_display();
            lcd_update();
            return cur_choice;
        }
        last_next = button & bm_next;
        last_prev = button & bm_prev;
        last_select = button & bm_select;
    }
}

void main(uint32_t arg, uint32_t addr) NORETURN_ATTR;
void main(uint32_t arg, uint32_t addr)
{
    unsigned char* loadbuffer;
    int buffer_size;
    void(*kernel_entry)(void);
    int ret;

    system_init();
    kernel_init();

    power_init();
    enable_irq();

    lcd_init();
    lcd_clear_display();
    lcd_update();

    backlight_init();

    button_init();

    printf("Boot version: %s", RBVERSION);
    printf("arg=%x addr=%x", arg, addr);
    printf("power up source: %x", __XTRACT(HW_POWER_STS, PWRUP_SOURCE));

    if(arg == 0xfee1dead)
    {
        printf("Disable partitions window.");
        imx233_partitions_enable_window(false);
    }

    ret = storage_init();
    if(ret < 0)
        error(EATA, ret, true);

    int boot_dev = do_bootmenu();

    /* NOTE: allow disk_init and disk_mount_all to fail since we can do USB after.
     * We need this order to determine the correct logical sector size */
    while(!disk_init(IF_MV(0)))
        printf("disk_init failed!");

    if((ret = disk_mount_all()) <= 0)
        error(EDISK, ret, false);

    if(usb_detect() == USB_INSERTED)
        usb_mode(HZ);

    char bootfile[128];
    if(boot_dev == 0)
        snprintf(bootfile, sizeof(bootfile), "%s", BOOTFILE);
    else
    {
        /* look for a volume on the boot device */
        int boot_vol = -1;
        for(int i = 0; i < NUM_VOLUMES; i++)
            if(volume_get_drive(i) == boot_dev)
            {
                boot_vol = i;
                break;
            }
        if(boot_vol == -1)
        {
            printf("There is no valid volume on this drive");
            boot_vol = 0;
        }
        snprintf(bootfile, sizeof(bootfile), VOL_NAMES BOOTDIR "/%s", boot_vol, BOOTFILE);
    }
    printf("Loading firmware %s", bootfile);

    loadbuffer = (unsigned char*)loadaddress;
    buffer_size = (int)(loadaddressend - loadaddress);

    while((ret = load_firmware(loadbuffer, bootfile, buffer_size)) < 0)
    {
        error(EBOOTFILE, ret, true);
    }

    kernel_entry = (void*) loadbuffer;
    printf("Executing");
    disable_interrupt(IRQ_FIQ_STATUS);
    commit_discard_idcache();
    kernel_entry();
    printf("ERR: Failed to boot");

    /* never returns */
    while(1) ;
}
