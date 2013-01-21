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
#include "dri-imx233.h"
#include "clkctrl-imx233.h"
#include "system-target.h"
#include "dma-imx233.h"
#include "icoll-imx233.h"
#include "string.h"
#include "pcm.h"
#include "pcm_sampr.h"
#include "file.h"

struct dri_dma_command_t
{
    struct apb_dma_command_t dma;
    /* padded to next multiple of cache line size (32 bytes) */
    uint32_t pad[5];
} __attribute__((packed)) CACHEALIGN_ATTR;

__ENSURE_STRUCT_CACHE_FRIENDLY(struct dri_dma_command_t)

static struct dri_dma_command_t dri_dma[2];

#define DRI_BUFFER_SIZE 4000
#define DAC_BUFFER_SIZE 2000

static uint16_t dri_buffer[2][DRI_BUFFER_SIZE];
static uint16_t dac_buffer[2][DAC_BUFFER_SIZE];
static volatile int cur_buffer = 0; /* buffer filled by DRI */

#define RDS_BUFFER_SIZE 100000
static uint16_t rds_buffer[RDS_BUFFER_SIZE];
static volatile int rds_write_pos = 0;
static int rds_read_pos = 0;

void imx233_dri_init(void)
{
    imx233_reset_block(&HW_DRI_CTRL);
}

void dri_pcm_cb(const void **start, size_t *size)
{
    *start = dac_buffer[1 - cur_buffer];
    *size = DAC_BUFFER_SIZE * sizeof(uint16_t);
}

void INT_DRI_DMA(void)
{
    discard_dcache_range(dri_buffer[cur_buffer], sizeof(dri_buffer[cur_buffer]));
    for(size_t i = 0, j = 0; i< DRI_BUFFER_SIZE && j < DAC_BUFFER_SIZE; i += 4, j += 2)
    {
        dac_buffer[cur_buffer][j] = dri_buffer[cur_buffer][i];
        dac_buffer[cur_buffer][j + 1] = dri_buffer[cur_buffer][i + 1];
        rds_buffer[rds_write_pos++] = dri_buffer[cur_buffer][i + 3];
        if(rds_write_pos >= RDS_BUFFER_SIZE)
            rds_write_pos = 0;
    }
    cur_buffer = 1 - cur_buffer;

    imx233_dma_clear_channel_interrupt(APB_DRI);
}

void imx233_dri_enable(bool en)
{
    if(en)
    {
        imx233_reset_block(&HW_DRI_CTRL);
        imx233_clkctrl_enable_xtal(XTAL_DRI, true);
        pcm_play_stop();
        pcm_set_frequency(FREQ_44);
        pcm_apply_settings();
        pcm_play_data(dri_pcm_cb, NULL, dac_buffer, DAC_BUFFER_SIZE * sizeof(uint16_t));
        pcm_play_pause(true);
        
        imx233_dma_reset_channel(APB_DRI);
        imx233_icoll_enable_interrupt(INT_SRC_DRI_DMA, true);
        imx233_dma_enable_channel_interrupt(APB_DRI, true);
        
        dri_dma[0].dma.next = &dri_dma[1].dma;
        dri_dma[0].dma.buffer = (void *)dri_buffer[0];
        dri_dma[0].dma.cmd = HW_APB_CHx_CMD__COMMAND__WRITE |
            (DRI_BUFFER_SIZE * 2) << HW_APB_CHx_CMD__XFER_COUNT_BP |
            HW_APB_CHx_CMD__IRQONCMPLT |
            HW_APB_CHx_CMD__CHAIN;
        dri_dma[1].dma.next = &dri_dma[0].dma;
        dri_dma[1].dma.buffer = (void *)dri_buffer[1];
        dri_dma[1].dma.cmd = HW_APB_CHx_CMD__COMMAND__WRITE |
            (DRI_BUFFER_SIZE * 2) << HW_APB_CHx_CMD__XFER_COUNT_BP |
            HW_APB_CHx_CMD__IRQONCMPLT |
            HW_APB_CHx_CMD__CHAIN;
        cur_buffer = 0;
        /* dma subsystem will make sure cached stuff is written to memory */
        imx233_dma_start_command(APB_DRI, &dri_dma[0].dma);

        __REG_SET(HW_DRI_CTRL) = HW_DRI_CTRL__ENABLE_INPUTS | HW_DRI_CTRL__RUN |
            HW_DRI_CTRL__REACQUIRE_PHASE;
    }
    else
    {
        pcm_play_stop();
        imx233_dma_enable_channel_interrupt(APB_DRI, false);
        imx233_icoll_enable_interrupt(INT_SRC_DRI_DMA, false);
        __REG_CLR(HW_DRI_CTRL) = HW_DRI_CTRL__ENABLE_INPUTS | HW_DRI_CTRL__RUN;
        imx233_clkctrl_enable_xtal(XTAL_DRI, false);
    }
}

struct imx233_dri_info_t imx233_dri_get_info(void)
{
    struct imx233_dri_info_t info;
    memset(&info, 0, sizeof(info));

    info.running = !!(HW_DRI_CTRL & HW_DRI_CTRL__RUN);
    info.inputs_enabled = !!(HW_DRI_CTRL & HW_DRI_CTRL__ENABLE_INPUTS);
    info.attention = !!(HW_DRI_CTRL & HW_DRI_CTRL__ATTENTION_IRQ);
    info.pilot_sync_loss = !!(HW_DRI_CTRL & HW_DRI_CTRL__PILOT_SYNC_LOSS_IRQ);
    info.overflow = !!(HW_DRI_CTRL & HW_DRI_CTRL__OVERFLOW_IRQ);
    info.pilot_phase = __XTRACT(HW_DRI_STAT, PILOT_PHASE);

    if(rds_read_pos != rds_write_pos)
    {
        int fd = open("/rds.bin", O_CREAT | O_WRONLY | O_APPEND);
        if(fd >= 0)
        {
            int pos = rds_write_pos; /* it is volatile ! */
            if(rds_read_pos <= pos)
            {
                write(fd, rds_buffer + rds_read_pos, pos - rds_read_pos);
                rds_read_pos = pos;
            }
            else
            {
                write(fd, rds_buffer + rds_read_pos, RDS_BUFFER_SIZE - rds_read_pos);
                rds_read_pos = 0;
            }
            close(fd);
        }
    }
    
    return info;
}