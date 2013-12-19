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
#ifndef __SSP_IMX233_H__
#define __SSP_IMX233_H__

#include "cpu.h"
#include "system.h"
#include "system-target.h"
#include "pinctrl-imx233.h"
#include "dma-imx233.h"

#include "regs/regs-ssp.h"

#if IMX233_SUBTARGET < 3700
#define IMX233_NR_SSP  1
#else
#define IMX233_NR_SSP  2
#endif

/* ssp can value 1 or 2 */
#if IMX233_NR_SSP >= 2
#define __SSP_SELECT(ssp, ssp1, ssp2) ((ssp) == 1 ? (ssp1) : (ssp2))
#else
#define __SSP_SELECT(ssp, ssp1, ssp2) (ssp1)
#endif

#define INT_SRC_SSP_DMA(ssp)    __SSP_SELECT(ssp, INT_SRC_SSP1_DMA, INT_SRC_SSP2_DMA)
#define INT_SRC_SSP_ERROR(ssp)  __SSP_SELECT(ssp, INT_SRC_SSP1_ERROR, INT_SRC_SSP2_ERROR)

#define BP_SSP_CTRL1_ALL_IRQ    0
#if IMX233_SUBTARGET < 3700
#define BM_SSP_CTRL1_ALL_IRQ \
    BM_OR7(SSP_CTRL1, SDIO_IRQ, RESP_ERR_IRQ, RESP_TIMEOUT_IRQ, DATA_TIMEOUT_IRQ, \
        DATA_CRC_IRQ, RECV_TIMEOUT_IRQ, RECV_OVRFLW_IRQ)
#define BM_SSP_CTRL1_ALL_IRQ_EN \
    BM_OR7(SSP_CTRL1, SDIO_IRQ_EN, RESP_ERR_IRQ_EN, RESP_TIMEOUT_IRQ_EN, DATA_TIMEOUT_IRQ_EN, \
        DATA_CRC_IRQ_EN, RECV_TIMEOUT_IRQ_EN, RECV_OVRFLW_IRQ_EN)
#else
#define BM_SSP_CTRL1_ALL_IRQ \
    BM_OR8(SSP_CTRL1, SDIO_IRQ, RESP_ERR_IRQ, RESP_TIMEOUT_IRQ, DATA_TIMEOUT_IRQ, \
        DATA_CRC_IRQ, FIFO_UNDERRUN_IRQ, RECV_TIMEOUT_IRQ, FIFO_OVERRUN_IRQ)
#define BM_SSP_CTRL1_ALL_IRQ_EN \
    BM_OR8(SSP_CTRL1, SDIO_IRQ_EN, RESP_ERR_IRQ_EN, RESP_TIMEOUT_IRQ_EN, DATA_TIMEOUT_IRQ_EN, \
        DATA_CRC_IRQ_EN, FIFO_UNDERRUN_EN, RECV_TIMEOUT_IRQ_EN, FIFO_OVERRUN_IRQ_EN)
#endif

#define BM_SSP_CTRL1_TIMEOUT_IRQ \
    BM_OR3(SSP_CTRL1, RESP_TIMEOUT_IRQ, DATA_TIMEOUT_IRQ, RECV_TIMEOUT_IRQ)

#define IMX233_MAX_SSP_XFER_SIZE            IMX233_MAX_SINGLE_DMA_XFER_SIZE

enum imx233_ssp_error_t
{
    SSP_SUCCESS = 0,
    SSP_ERROR = -1,
    SSP_TIMEOUT = -2,
};

enum imx233_ssp_resp_t
{
    SSP_NO_RESP = 0,
    SSP_SHORT_RESP,
    SSP_LONG_RESP
};

typedef void (*ssp_detect_cb_t)(int ssp);

void imx233_ssp_init(void);
/* start the ssp block, possibly ungating the SSPCLK */
void imx233_ssp_start(int ssp);
/* stop the ssp block, possibly gating the SSPCLK */
void imx233_ssp_stop(int ssp);
/* only softreset between start and stop or it might hang ! */
void imx233_ssp_softreset(int ssp);
/* set ssp block timing, relative to SSPCLK which runs at 96MHz */
void imx233_ssp_set_timings(int ssp, int divide, int rate, int timeout);
/* set ssp block mode: spi, sd/mmc, ... */
void imx233_ssp_set_mode(int ssp, unsigned mode);
/* set the ssp block bus width: 1-bit, 4-bit, 8-bit */
void imx233_ssp_set_bus_width(int ssp, unsigned width);
/* block_size uses the SSP format so it's actually the log_2 of the block_size */
void imx233_ssp_set_block_size(int ssp, unsigned log_block_size);
/* SD/MMC facilities */
enum imx233_ssp_error_t imx233_ssp_sd_mmc_transfer(int ssp, uint8_t cmd,
    uint32_t cmd_arg, enum imx233_ssp_resp_t resp, void *buffer, unsigned block_count,
    bool wait4irq, bool read, uint32_t *resp_ptr);
/* pullups/alternative are ignored on targets which don't support it */
void imx233_ssp_setup_ssp1_sd_mmc_pins(bool enable_pullups, unsigned bus_width, bool use_alt);
void imx233_ssp_setup_ssp2_sd_mmc_pins(bool enable_pullups, unsigned bus_width);
/* after callback is fired, imx233_ssp_sdmmc_setup_detect needs to be called
 * to enable detection again. If first_time is true, the callback will
 * be called if the sd card is inserted when the function is called, otherwise
 * it will be called on the next insertion change.
 * By default, sd_detect=1 means sd inserted; invert reverses this behaviour */
void imx233_ssp_sdmmc_setup_detect(int ssp, bool enable, ssp_detect_cb_t fn,
    bool first_time, bool invert);
/* needs prior setup with imx233_ssp_sdmmc_setup_detect */
bool imx233_ssp_sdmmc_is_detect_inverted(int spp);
/* raw value of the detect pin */
bool imx233_ssp_sdmmc_detect_raw(int ssp);
/* corrected value given the invert setting */
bool imx233_ssp_sdmmc_detect(int ssp);
/* SD/MMC requires that the card be provided the clock during an init sequence of
 * at least 1msec (or 74 clocks). Does NOT touch the clock so it has to be correct. */
void imx233_ssp_sd_mmc_power_up_sequence(int ssp);

#endif /* __SSP_IMX233_H__ */
