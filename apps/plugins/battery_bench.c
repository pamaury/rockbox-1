/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 * $Id$
 *
 *
 * Copyright (C) 2006 Alexander Spyridakis, Hristo Kovachev
 *
 * All files in this archive are subject to the GNU General Public License.
 * See the file COPYING in the source tree root for full license agreement.
 *
 * This software is distributed on an "AS IS" basis, WITHOUT WARRANTY OF ANY
 * KIND, either express or implied.
 *
 ****************************************************************************/
#ifndef SIMULATOR /* not for the simulator */

#include "plugin.h"
PLUGIN_HEADER

#define BATTERY_LOG "/battery_bench.txt"
#define BUF_SIZE 16000
#define DISK_SPINDOWN_TIMEOUT 3600

#define EV_EXIT 1337

#if CONFIG_KEYPAD == RECORDER_PAD
#define BATTERY_ON BUTTON_ON
#define BATTERY_OFF BUTTON_OFF

#elif CONFIG_KEYPAD == ONDIO_PAD
#define BATTERY_ON BUTTON_RIGHT
#define BATTERY_OFF BUTTON_OFF

#elif CONFIG_KEYPAD == PLAYER_PAD
#define BATTERY_ON BUTTON_ON
#define BATTERY_OFF BUTTON_STOP

#elif (CONFIG_KEYPAD == IRIVER_H100_PAD) || \
      (CONFIG_KEYPAD == IRIVER_H300_PAD)
      
#define BATTERY_ON BUTTON_ON
#define BATTERY_RC_ON BUTTON_RC_ON

#define BATTERY_OFF BUTTON_OFF
#define BATTERY_RC_OFF BUTTON_RC_STOP

#elif CONFIG_KEYPAD == IPOD_4G_PAD

#define BATTERY_ON  BUTTON_PLAY
#define BATTERY_OFF BUTTON_MENU

#endif


/***************** Plugin Entry Point *****************/
static struct plugin_api* rb;
int main(void);
void exit_tsr(void);
void thread(void);

enum plugin_status plugin_start(struct plugin_api* api, void* parameter)
{
    (void)parameter;
    rb = api;
    
    return main();
}

struct 
{
    int id;
    bool ended;
} s_thread;

/* Struct for battery information */
struct batt_info
{
    int ticks, level, eta;
    unsigned int voltage;
};

struct event_queue thread_q;

void exit_tsr(void)
{
    rb->queue_post(&thread_q, EV_EXIT, NULL);
    while (!s_thread.ended)
        rb->yield();
    /* remove the thread's queue from the broadcast list */
    rb->queue_delete(&thread_q);
}

#define HMS(x) (x)/3600,((x)%3600)/60,((x)%3600)%60 
void thread(void)
{
    bool got_info = false, timeflag = false, in_usb_mode = false;
    int fd, buffelements, tick = 1, i = 0, skipped = 0, exit = 0;
    int fst = 0, lst = 0; /* first and last skipped tick */
    unsigned int last_voltage = 0;
    long sleep_time;
    
    struct event ev;

    struct batt_info bat[buffelements = (BUF_SIZE / sizeof(struct batt_info))];

    sleep_time = (rb->global_settings->disk_spindown > 1) ?
        (rb->global_settings->disk_spindown - 1) * HZ : 5 * HZ;
    
    do
    {
        if(!in_usb_mode && got_info && 
            (exit || timeflag || rb->ata_disk_is_active()) )
        {
            int last, secs, j, temp = skipped;
            
            fd = rb->open(BATTERY_LOG, O_RDWR | O_CREAT | O_APPEND);
            if(fd < 0)
                exit = 1;
            else
            {
               do
                {
                    if(skipped)
                    {
                        last = buffelements;
                        fst /= HZ;
                        lst /= HZ;
                        rb->fdprintf(fd,"-Skipped %d measurements from "
                            "%02d:%02d:%02d to %02d:%02d:%02d-\n",skipped,
                            HMS(fst),HMS(lst));
                        skipped = 0;
                    }
                    else
                    {
                        last = i;
                        i = 0;
                    }

                    for(j = i; j < last; j++)
                    {
                         secs = bat[j].ticks/HZ;
                         rb->fdprintf(fd,
                                "%02d:%02d:%02d,  %05d,     %03d%%,     "
                                "%02d:%02d,           %04d,     %04d\n",
                                HMS(secs), secs, bat[j].level,
                                bat[j].eta / 60, bat[j].eta % 60, 
                                bat[j].voltage * 10, temp + 1 + (j-i));
                         if(!j % 100 && !j) /* yield() at every 100 writes */
                            rb->yield();
                    }
                    temp += j - i;
 
                }while(i != 0);
           
                rb->close(fd);
                tick = *rb->current_tick;
                got_info = false;
                timeflag = false;
            }
        }
        else
        {            
            if(
#if CONFIG_CODEC == SWCODEC                
                !rb->pcm_is_playing()
#else
                !rb->mp3_is_playing()
#endif                
                && (*rb->current_tick - tick) > DISK_SPINDOWN_TIMEOUT * HZ)
                timeflag = true;
            
            if(last_voltage != rb->battery_voltage())
            {
                if(i == buffelements)
                {
                    if(!skipped++)
                        fst = bat[0].ticks;
                    i = 0;
                } 
                else if(skipped)
                {
                        skipped++;
                        lst = bat[i].ticks;
                }
                bat[i].ticks = *rb->current_tick;
                bat[i].level = rb->battery_level();
                bat[i].eta = rb->battery_time();
                last_voltage = bat[i++].voltage = rb->battery_voltage();
                got_info = true;
            }            
 
       }
        
        if(exit)
        {
            if(exit == 2)
                    rb->splash(HZ,true,"Exiting battery_bench...");
            s_thread.ended = true;
            rb->remove_thread(s_thread.id);
            rb->yield(); /* exit the thread, this yield() won't return */
        }
        
        rb->queue_wait_w_tmo(&thread_q, &ev, sleep_time);
        switch (ev.id)
        {
            case SYS_USB_CONNECTED: 
                in_usb_mode = true;
                rb->usb_acknowledge(SYS_USB_CONNECTED_ACK);
                break;
            case SYS_USB_DISCONNECTED:
                in_usb_mode = false;
                rb->usb_acknowledge(SYS_USB_DISCONNECTED_ACK);
                break;
            case SYS_POWEROFF:
                exit = 1;
                break;
            case EV_EXIT:
                exit = 2;
                break;
        }
    } while (1);
    
}

int main(void)
{
    int stacksize, button, fd;
    bool on = false;
    void* stack;
    
    
    rb->lcd_clear_display();

#ifdef HAVE_LCD_BITMAP
    int strwdt, strhgt;
    
    rb->lcd_setfont(FONT_SYSFIXED);
    
    rb->lcd_getstringsize("Battery Benchmark", &strwdt, &strhgt);
    rb->lcd_putsxy((LCD_WIDTH - strwdt)/2, strhgt, "Battery Benchmark");
    
    rb->lcd_getstringsize("Check /battery_bench.txt", &strwdt, &strhgt);
    rb->lcd_putsxy((LCD_WIDTH - strwdt)/2,strhgt * 3,
        "Check /battery_bench.txt");
    rb->lcd_getstringsize("file for more info.", &strwdt, &strhgt);
    rb->lcd_putsxy((LCD_WIDTH - strwdt)/2, strhgt * 4, "file for more info.");
    rb->lcd_getstringsize("Play to start, OFF to quit", &strwdt, &strhgt);
    rb->lcd_putsxy((LCD_WIDTH - strwdt)/2, strhgt * 5,
        "PLAY to start, OFF to quit");
    
    rb->lcd_update();

#else
    rb->lcd_puts_scroll(0, 1, "Battery Benchmark");
    rb->lcd_puts_scroll(0, 2, "PLAY to start, OFF to quit");
#endif
    
    do
    {
        button = rb->button_get(true);
        switch(button)
        {
            case BATTERY_ON:
#ifdef BATTERY_RC_ON
            case BATTERY_RC_ON:
#endif                         
                on = true;
                break;        
            case BATTERY_OFF: 
#ifdef BATTERY_RC_OFF
            case BATTERY_RC_OFF:
#endif                        
                return PLUGIN_OK;
                
            default: if(rb->default_event_handler(button) == SYS_USB_CONNECTED)
                    return PLUGIN_USB_CONNECTED;
        }
    }while(!on);
    
    stack = rb->plugin_get_buffer(&stacksize);
    /* long align it and leave some space (200bytes) for vars */
    stack = (void*)(((unsigned int)stack + 200) & ~3);
    
    stacksize = (stacksize - 200) & ~3;
    if (stacksize < BUF_SIZE)
    {
        rb->splash(HZ*2, true, "Out of memory");
        return PLUGIN_ERROR;
    }
    
    fd = rb->open(BATTERY_LOG, O_RDONLY);
    if(fd < 0)
    {
        fd = rb->open(BATTERY_LOG, O_RDWR | O_CREAT);
        if(fd >= 0)
        {
            rb->fdprintf(fd,
                "This plugin will log your battery performance in a\n"
                "file (%s) every time the disk is accessed (or every hour).\n"
                "To properly test your battery:\n"
                "1) Select and playback an album. "
                "(Be sure to be more than the player's buffer)\n"
                "2) Set to repeat.\n"
                "3) Let the player run completely out of battery.\n"
                "4) Recharge and copy (or whatever you want) the txt file to "
                "your computer.\n"
                "Now you can make graphs with the data of the battery log.\n"
                "Do not enter another plugin during the test or else the "
                "logging activity will end.\n\n"
                "P.S: You can decide how you will make your tests.\n"
                "Just don't open another  plugin to be sure that your log "
                "will continue.\n"
                "M/DA (Measurements per Disk Activity) shows how many times\n"
                "data was logged in the buffer between Disk Activity.\n\n"
                "Battery type: %d mAh      Buffer Entries: %d\n"
                "  Time:,  Seconds:,  Level:,  Time Left:,  Voltage[mV]:,"
                "  M/DA:\n"
                ,BATTERY_LOG,rb->global_settings->battery_capacity,
                BUF_SIZE / sizeof(struct batt_info));
            rb->close(fd);
        }
        else
        {
            rb->splash(HZ / 2, true, "Cannot create file!");
            return PLUGIN_ERROR;
        }
    }
    else
    {
        rb->close(fd);
        fd = rb->open(BATTERY_LOG, O_RDWR | O_APPEND);
        rb->fdprintf(fd, "\nFile already present. Resuming Benchmark\n");
        rb->close(fd);
    }
    
    rb->queue_init(&thread_q); /* put the thread's queue in the bcast list */
    rb->memset(&s_thread, 0, sizeof(s_thread)); /* zero the struct */
    s_thread.id = rb->create_thread(thread, stack,
        stacksize, "Battery Benchmark");
    rb->plugin_tsr(exit_tsr);
    
    return PLUGIN_OK;
}

#endif
