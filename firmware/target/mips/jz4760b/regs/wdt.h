/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 * This file was automatically generated by headergen, DO NOT EDIT it.
 * headergen version: 3.0.0
 * jz4760b version: 1.0
 * jz4760b authors: Amaury Pouly
 *
 * Copyright (C) 2015 by the authors
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
#ifndef __HEADERGEN_WDT_H__
#define __HEADERGEN_WDT_H__

#include "macro.h"

#define REG_WDT_DATA    jz_reg(WDT_DATA)
#define JA_WDT_DATA     (0xb0002000 + 0x0)
#define JT_WDT_DATA     JIO_16_RW
#define JN_WDT_DATA     WDT_DATA
#define JI_WDT_DATA     

#define REG_WDT_ENABLE              jz_reg(WDT_ENABLE)
#define JA_WDT_ENABLE               (0xb0002000 + 0x4)
#define JT_WDT_ENABLE               JIO_8_RW
#define JN_WDT_ENABLE               WDT_ENABLE
#define JI_WDT_ENABLE               
#define BP_WDT_ENABLE_TCEN          0
#define BM_WDT_ENABLE_TCEN          0x1
#define BF_WDT_ENABLE_TCEN(v)       (((v) & 0x1) << 0)
#define BFM_WDT_ENABLE_TCEN(v)      BM_WDT_ENABLE_TCEN
#define BF_WDT_ENABLE_TCEN_V(e)     BF_WDT_ENABLE_TCEN(BV_WDT_ENABLE_TCEN__##e)
#define BFM_WDT_ENABLE_TCEN_V(v)    BM_WDT_ENABLE_TCEN

#define REG_WDT_COUNT   jz_reg(WDT_COUNT)
#define JA_WDT_COUNT    (0xb0002000 + 0x8)
#define JT_WDT_COUNT    JIO_16_RW
#define JN_WDT_COUNT    WDT_COUNT
#define JI_WDT_COUNT    

#define REG_WDT_CTRL                jz_reg(WDT_CTRL)
#define JA_WDT_CTRL                 (0xb0002000 + 0xc)
#define JT_WDT_CTRL                 JIO_16_RW
#define JN_WDT_CTRL                 WDT_CTRL
#define JI_WDT_CTRL                 
#define BP_WDT_CTRL_PRESCALE        3
#define BM_WDT_CTRL_PRESCALE        0x38
#define BV_WDT_CTRL_PRESCALE__1     0x0
#define BV_WDT_CTRL_PRESCALE__4     0x1
#define BV_WDT_CTRL_PRESCALE__16    0x2
#define BV_WDT_CTRL_PRESCALE__64    0x3
#define BV_WDT_CTRL_PRESCALE__256   0x4
#define BV_WDT_CTRL_PRESCALE__1024  0x5
#define BF_WDT_CTRL_PRESCALE(v)     (((v) & 0x7) << 3)
#define BFM_WDT_CTRL_PRESCALE(v)    BM_WDT_CTRL_PRESCALE
#define BF_WDT_CTRL_PRESCALE_V(e)   BF_WDT_CTRL_PRESCALE(BV_WDT_CTRL_PRESCALE__##e)
#define BFM_WDT_CTRL_PRESCALE_V(v)  BM_WDT_CTRL_PRESCALE
#define BP_WDT_CTRL_CLKIN           0
#define BM_WDT_CTRL_CLKIN           0x7
#define BV_WDT_CTRL_CLKIN__PCK      0x1
#define BV_WDT_CTRL_CLKIN__RTC      0x2
#define BV_WDT_CTRL_CLKIN__EXT      0x4
#define BF_WDT_CTRL_CLKIN(v)        (((v) & 0x7) << 0)
#define BFM_WDT_CTRL_CLKIN(v)       BM_WDT_CTRL_CLKIN
#define BF_WDT_CTRL_CLKIN_V(e)      BF_WDT_CTRL_CLKIN(BV_WDT_CTRL_CLKIN__##e)
#define BFM_WDT_CTRL_CLKIN_V(v)     BM_WDT_CTRL_CLKIN

#endif /* __HEADERGEN_WDT_H__*/
