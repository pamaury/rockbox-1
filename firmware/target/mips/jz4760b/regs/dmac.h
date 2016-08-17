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
#ifndef __HEADERGEN_DMAC_H__
#define __HEADERGEN_DMAC_H__

#include "macro.h"

#define REG_DMAC_DSAR(_n1)  jz_reg(DMAC_DSAR(_n1))
#define JA_DMAC_DSAR(_n1)   (0xb3420000 + (((_n1))/6*0x100 + 0x00 + (((_n1))-((_n1))/6*6) * 0x20))
#define JT_DMAC_DSAR(_n1)   JIO_32_RW
#define JN_DMAC_DSAR(_n1)   DMAC_DSAR
#define JI_DMAC_DSAR(_n1)   (_n1)

#define REG_DMAC_DTAR(_n1)  jz_reg(DMAC_DTAR(_n1))
#define JA_DMAC_DTAR(_n1)   (0xb3420000 + (((_n1))/6*0x100 + 0x04 + (((_n1))-((_n1))/6*6) * 0x20))
#define JT_DMAC_DTAR(_n1)   JIO_32_RW
#define JN_DMAC_DTAR(_n1)   DMAC_DTAR
#define JI_DMAC_DTAR(_n1)   (_n1)

#define REG_DMAC_DTCR(_n1)  jz_reg(DMAC_DTCR(_n1))
#define JA_DMAC_DTCR(_n1)   (0xb3420000 + (((_n1))/6*0x100 + 0x08 + (((_n1))-((_n1))/6*6) * 0x20))
#define JT_DMAC_DTCR(_n1)   JIO_32_RW
#define JN_DMAC_DTCR(_n1)   DMAC_DTCR
#define JI_DMAC_DTCR(_n1)   (_n1)

#define REG_DMAC_DRSR(_n1)          jz_reg(DMAC_DRSR(_n1))
#define JA_DMAC_DRSR(_n1)           (0xb3420000 + (((_n1))/6*0x100 + 0x0c + (((_n1))-((_n1))/6*6) * 0x20))
#define JT_DMAC_DRSR(_n1)           JIO_32_RW
#define JN_DMAC_DRSR(_n1)           DMAC_DRSR
#define JI_DMAC_DRSR(_n1)           (_n1)
#define BP_DMAC_DRSR_RS             0
#define BM_DMAC_DRSR_RS             0x3f
#define BV_DMAC_DRSR_RS__AUTO       0x8
#define BV_DMAC_DRSR_RS__TSSIIN     0x9
#define BV_DMAC_DRSR_RS__EXTERN     0xc
#define BV_DMAC_DRSR_RS__UART3OUT   0xe
#define BV_DMAC_DRSR_RS__UART3IN    0xf
#define BV_DMAC_DRSR_RS__UART2OUT   0x10
#define BV_DMAC_DRSR_RS__UART2IN    0x11
#define BV_DMAC_DRSR_RS__UART1OUT   0x12
#define BV_DMAC_DRSR_RS__UART1IN    0x13
#define BV_DMAC_DRSR_RS__UART0OUT   0x14
#define BV_DMAC_DRSR_RS__UART0IN    0x15
#define BV_DMAC_DRSR_RS__SSI0OUT    0x16
#define BV_DMAC_DRSR_RS__SSI0IN     0x17
#define BV_DMAC_DRSR_RS__AICOUT     0x18
#define BV_DMAC_DRSR_RS__AICIN      0x19
#define BV_DMAC_DRSR_RS__MSC0OUT    0x1a
#define BV_DMAC_DRSR_RS__MSC0IN     0x1b
#define BV_DMAC_DRSR_RS__TCU        0x1c
#define BV_DMAC_DRSR_RS__SADC       0x1d
#define BV_DMAC_DRSR_RS__MSC1OUT    0x1e
#define BV_DMAC_DRSR_RS__MSC1IN     0x1f
#define BV_DMAC_DRSR_RS__SSI1OUT    0x20
#define BV_DMAC_DRSR_RS__SSI1IN     0x21
#define BV_DMAC_DRSR_RS__PMOUT      0x22
#define BV_DMAC_DRSR_RS__PMIN       0x23
#define BV_DMAC_DRSR_RS__MSC2OUT    0x24
#define BV_DMAC_DRSR_RS__MSC2IN     0x25
#define BF_DMAC_DRSR_RS(v)          (((v) & 0x3f) << 0)
#define BFM_DMAC_DRSR_RS(v)         BM_DMAC_DRSR_RS
#define BF_DMAC_DRSR_RS_V(e)        BF_DMAC_DRSR_RS(BV_DMAC_DRSR_RS__##e)
#define BFM_DMAC_DRSR_RS_V(v)       BM_DMAC_DRSR_RS

#define REG_DMAC_DCCSR(_n1)         jz_reg(DMAC_DCCSR(_n1))
#define JA_DMAC_DCCSR(_n1)          (0xb3420000 + (((_n1))/6*0x100 + 0x10 + (((_n1))-((_n1))/6*6) * 0x20))
#define JT_DMAC_DCCSR(_n1)          JIO_32_RW
#define JN_DMAC_DCCSR(_n1)          DMAC_DCCSR
#define JI_DMAC_DCCSR(_n1)          (_n1)
#define BP_DMAC_DCCSR_NDES          31
#define BM_DMAC_DCCSR_NDES          0x80000000
#define BF_DMAC_DCCSR_NDES(v)       (((v) & 0x1) << 31)
#define BFM_DMAC_DCCSR_NDES(v)      BM_DMAC_DCCSR_NDES
#define BF_DMAC_DCCSR_NDES_V(e)     BF_DMAC_DCCSR_NDES(BV_DMAC_DCCSR_NDES__##e)
#define BFM_DMAC_DCCSR_NDES_V(v)    BM_DMAC_DCCSR_NDES
#define BP_DMAC_DCCSR_DES8          30
#define BM_DMAC_DCCSR_DES8          0x40000000
#define BF_DMAC_DCCSR_DES8(v)       (((v) & 0x1) << 30)
#define BFM_DMAC_DCCSR_DES8(v)      BM_DMAC_DCCSR_DES8
#define BF_DMAC_DCCSR_DES8_V(e)     BF_DMAC_DCCSR_DES8(BV_DMAC_DCCSR_DES8__##e)
#define BFM_DMAC_DCCSR_DES8_V(v)    BM_DMAC_DCCSR_DES8
#define BP_DMAC_DCCSR_CDOA          16
#define BM_DMAC_DCCSR_CDOA          0xff0000
#define BF_DMAC_DCCSR_CDOA(v)       (((v) & 0xff) << 16)
#define BFM_DMAC_DCCSR_CDOA(v)      BM_DMAC_DCCSR_CDOA
#define BF_DMAC_DCCSR_CDOA_V(e)     BF_DMAC_DCCSR_CDOA(BV_DMAC_DCCSR_CDOA__##e)
#define BFM_DMAC_DCCSR_CDOA_V(v)    BM_DMAC_DCCSR_CDOA
#define BP_DMAC_DCCSR_AR            4
#define BM_DMAC_DCCSR_AR            0x10
#define BF_DMAC_DCCSR_AR(v)         (((v) & 0x1) << 4)
#define BFM_DMAC_DCCSR_AR(v)        BM_DMAC_DCCSR_AR
#define BF_DMAC_DCCSR_AR_V(e)       BF_DMAC_DCCSR_AR(BV_DMAC_DCCSR_AR__##e)
#define BFM_DMAC_DCCSR_AR_V(v)      BM_DMAC_DCCSR_AR
#define BP_DMAC_DCCSR_TT            3
#define BM_DMAC_DCCSR_TT            0x8
#define BF_DMAC_DCCSR_TT(v)         (((v) & 0x1) << 3)
#define BFM_DMAC_DCCSR_TT(v)        BM_DMAC_DCCSR_TT
#define BF_DMAC_DCCSR_TT_V(e)       BF_DMAC_DCCSR_TT(BV_DMAC_DCCSR_TT__##e)
#define BFM_DMAC_DCCSR_TT_V(v)      BM_DMAC_DCCSR_TT
#define BP_DMAC_DCCSR_HLT           2
#define BM_DMAC_DCCSR_HLT           0x4
#define BF_DMAC_DCCSR_HLT(v)        (((v) & 0x1) << 2)
#define BFM_DMAC_DCCSR_HLT(v)       BM_DMAC_DCCSR_HLT
#define BF_DMAC_DCCSR_HLT_V(e)      BF_DMAC_DCCSR_HLT(BV_DMAC_DCCSR_HLT__##e)
#define BFM_DMAC_DCCSR_HLT_V(v)     BM_DMAC_DCCSR_HLT
#define BP_DMAC_DCCSR_CT            1
#define BM_DMAC_DCCSR_CT            0x2
#define BF_DMAC_DCCSR_CT(v)         (((v) & 0x1) << 1)
#define BFM_DMAC_DCCSR_CT(v)        BM_DMAC_DCCSR_CT
#define BF_DMAC_DCCSR_CT_V(e)       BF_DMAC_DCCSR_CT(BV_DMAC_DCCSR_CT__##e)
#define BFM_DMAC_DCCSR_CT_V(v)      BM_DMAC_DCCSR_CT
#define BP_DMAC_DCCSR_EN            0
#define BM_DMAC_DCCSR_EN            0x1
#define BF_DMAC_DCCSR_EN(v)         (((v) & 0x1) << 0)
#define BFM_DMAC_DCCSR_EN(v)        BM_DMAC_DCCSR_EN
#define BF_DMAC_DCCSR_EN_V(e)       BF_DMAC_DCCSR_EN(BV_DMAC_DCCSR_EN__##e)
#define BFM_DMAC_DCCSR_EN_V(v)      BM_DMAC_DCCSR_EN

#define REG_DMAC_DCMD(_n1)              jz_reg(DMAC_DCMD(_n1))
#define JA_DMAC_DCMD(_n1)               (0xb3420000 + (((_n1))/6*0x100 + 0x14 + (((_n1))-((_n1))/6*6) * 0x20))
#define JT_DMAC_DCMD(_n1)               JIO_32_RW
#define JN_DMAC_DCMD(_n1)               DMAC_DCMD
#define JI_DMAC_DCMD(_n1)               (_n1)
#define BP_DMAC_DCMD_EACKS_LOW          31
#define BM_DMAC_DCMD_EACKS_LOW          0x80000000
#define BF_DMAC_DCMD_EACKS_LOW(v)       (((v) & 0x1) << 31)
#define BFM_DMAC_DCMD_EACKS_LOW(v)      BM_DMAC_DCMD_EACKS_LOW
#define BF_DMAC_DCMD_EACKS_LOW_V(e)     BF_DMAC_DCMD_EACKS_LOW(BV_DMAC_DCMD_EACKS_LOW__##e)
#define BFM_DMAC_DCMD_EACKS_LOW_V(v)    BM_DMAC_DCMD_EACKS_LOW
#define BP_DMAC_DCMD_EACKM_WRITE        30
#define BM_DMAC_DCMD_EACKM_WRITE        0x40000000
#define BF_DMAC_DCMD_EACKM_WRITE(v)     (((v) & 0x1) << 30)
#define BFM_DMAC_DCMD_EACKM_WRITE(v)    BM_DMAC_DCMD_EACKM_WRITE
#define BF_DMAC_DCMD_EACKM_WRITE_V(e)   BF_DMAC_DCMD_EACKM_WRITE(BV_DMAC_DCMD_EACKM_WRITE__##e)
#define BFM_DMAC_DCMD_EACKM_WRITE_V(v)  BM_DMAC_DCMD_EACKM_WRITE
#define BP_DMAC_DCMD_ERDM               28
#define BM_DMAC_DCMD_ERDM               0x30000000
#define BV_DMAC_DCMD_ERDM__LOW          0x0
#define BV_DMAC_DCMD_ERDM__FALL         0x1
#define BV_DMAC_DCMD_ERDM__HIGH         0x2
#define BV_DMAC_DCMD_ERDM__RISE         0x3
#define BF_DMAC_DCMD_ERDM(v)            (((v) & 0x3) << 28)
#define BFM_DMAC_DCMD_ERDM(v)           BM_DMAC_DCMD_ERDM
#define BF_DMAC_DCMD_ERDM_V(e)          BF_DMAC_DCMD_ERDM(BV_DMAC_DCMD_ERDM__##e)
#define BFM_DMAC_DCMD_ERDM_V(v)         BM_DMAC_DCMD_ERDM
#define BP_DMAC_DCMD_SAI                23
#define BM_DMAC_DCMD_SAI                0x800000
#define BF_DMAC_DCMD_SAI(v)             (((v) & 0x1) << 23)
#define BFM_DMAC_DCMD_SAI(v)            BM_DMAC_DCMD_SAI
#define BF_DMAC_DCMD_SAI_V(e)           BF_DMAC_DCMD_SAI(BV_DMAC_DCMD_SAI__##e)
#define BFM_DMAC_DCMD_SAI_V(v)          BM_DMAC_DCMD_SAI
#define BP_DMAC_DCMD_DAI                22
#define BM_DMAC_DCMD_DAI                0x400000
#define BF_DMAC_DCMD_DAI(v)             (((v) & 0x1) << 22)
#define BFM_DMAC_DCMD_DAI(v)            BM_DMAC_DCMD_DAI
#define BF_DMAC_DCMD_DAI_V(e)           BF_DMAC_DCMD_DAI(BV_DMAC_DCMD_DAI__##e)
#define BFM_DMAC_DCMD_DAI_V(v)          BM_DMAC_DCMD_DAI
#define BP_DMAC_DCMD_RDIL               16
#define BM_DMAC_DCMD_RDIL               0xf0000
#define BV_DMAC_DCMD_RDIL__IGN          0x0
#define BV_DMAC_DCMD_RDIL__2            0x1
#define BV_DMAC_DCMD_RDIL__4            0x2
#define BV_DMAC_DCMD_RDIL__8            0x3
#define BV_DMAC_DCMD_RDIL__12           0x4
#define BV_DMAC_DCMD_RDIL__16           0x5
#define BV_DMAC_DCMD_RDIL__20           0x6
#define BV_DMAC_DCMD_RDIL__24           0x7
#define BV_DMAC_DCMD_RDIL__28           0x8
#define BV_DMAC_DCMD_RDIL__32           0x9
#define BV_DMAC_DCMD_RDIL__48           0xa
#define BV_DMAC_DCMD_RDIL__60           0xb
#define BV_DMAC_DCMD_RDIL__64           0xc
#define BV_DMAC_DCMD_RDIL__124          0xd
#define BV_DMAC_DCMD_RDIL__128          0xe
#define BV_DMAC_DCMD_RDIL__200          0xf
#define BF_DMAC_DCMD_RDIL(v)            (((v) & 0xf) << 16)
#define BFM_DMAC_DCMD_RDIL(v)           BM_DMAC_DCMD_RDIL
#define BF_DMAC_DCMD_RDIL_V(e)          BF_DMAC_DCMD_RDIL(BV_DMAC_DCMD_RDIL__##e)
#define BFM_DMAC_DCMD_RDIL_V(v)         BM_DMAC_DCMD_RDIL
#define BP_DMAC_DCMD_SWDH               14
#define BM_DMAC_DCMD_SWDH               0xc000
#define BV_DMAC_DCMD_SWDH__32           0x0
#define BV_DMAC_DCMD_SWDH__8            0x1
#define BV_DMAC_DCMD_SWDH__16           0x2
#define BF_DMAC_DCMD_SWDH(v)            (((v) & 0x3) << 14)
#define BFM_DMAC_DCMD_SWDH(v)           BM_DMAC_DCMD_SWDH
#define BF_DMAC_DCMD_SWDH_V(e)          BF_DMAC_DCMD_SWDH(BV_DMAC_DCMD_SWDH__##e)
#define BFM_DMAC_DCMD_SWDH_V(v)         BM_DMAC_DCMD_SWDH
#define BP_DMAC_DCMD_DWDH               12
#define BM_DMAC_DCMD_DWDH               0x3000
#define BV_DMAC_DCMD_DWDH__32           0x0
#define BV_DMAC_DCMD_DWDH__8            0x1
#define BV_DMAC_DCMD_DWDH__16           0x2
#define BF_DMAC_DCMD_DWDH(v)            (((v) & 0x3) << 12)
#define BFM_DMAC_DCMD_DWDH(v)           BM_DMAC_DCMD_DWDH
#define BF_DMAC_DCMD_DWDH_V(e)          BF_DMAC_DCMD_DWDH(BV_DMAC_DCMD_DWDH__##e)
#define BFM_DMAC_DCMD_DWDH_V(v)         BM_DMAC_DCMD_DWDH
#define BP_DMAC_DCMD_DS                 8
#define BM_DMAC_DCMD_DS                 0x700
#define BV_DMAC_DCMD_DS__32BIT          0x0
#define BV_DMAC_DCMD_DS__8BIT           0x1
#define BV_DMAC_DCMD_DS__16BIT          0x2
#define BV_DMAC_DCMD_DS__16BYTE         0x3
#define BV_DMAC_DCMD_DS__32BYTE         0x4
#define BV_DMAC_DCMD_DS__64BYTE         0x5
#define BF_DMAC_DCMD_DS(v)              (((v) & 0x7) << 8)
#define BFM_DMAC_DCMD_DS(v)             BM_DMAC_DCMD_DS
#define BF_DMAC_DCMD_DS_V(e)            BF_DMAC_DCMD_DS(BV_DMAC_DCMD_DS__##e)
#define BFM_DMAC_DCMD_DS_V(v)           BM_DMAC_DCMD_DS
#define BP_DMAC_DCMD_STDE               2
#define BM_DMAC_DCMD_STDE               0x4
#define BF_DMAC_DCMD_STDE(v)            (((v) & 0x1) << 2)
#define BFM_DMAC_DCMD_STDE(v)           BM_DMAC_DCMD_STDE
#define BF_DMAC_DCMD_STDE_V(e)          BF_DMAC_DCMD_STDE(BV_DMAC_DCMD_STDE__##e)
#define BFM_DMAC_DCMD_STDE_V(v)         BM_DMAC_DCMD_STDE
#define BP_DMAC_DCMD_TIE                1
#define BM_DMAC_DCMD_TIE                0x2
#define BF_DMAC_DCMD_TIE(v)             (((v) & 0x1) << 1)
#define BFM_DMAC_DCMD_TIE(v)            BM_DMAC_DCMD_TIE
#define BF_DMAC_DCMD_TIE_V(e)           BF_DMAC_DCMD_TIE(BV_DMAC_DCMD_TIE__##e)
#define BFM_DMAC_DCMD_TIE_V(v)          BM_DMAC_DCMD_TIE
#define BP_DMAC_DCMD_LINK               0
#define BM_DMAC_DCMD_LINK               0x1
#define BF_DMAC_DCMD_LINK(v)            (((v) & 0x1) << 0)
#define BFM_DMAC_DCMD_LINK(v)           BM_DMAC_DCMD_LINK
#define BF_DMAC_DCMD_LINK_V(e)          BF_DMAC_DCMD_LINK(BV_DMAC_DCMD_LINK__##e)
#define BFM_DMAC_DCMD_LINK_V(v)         BM_DMAC_DCMD_LINK

#define REG_DMAC_DDA(_n1)           jz_reg(DMAC_DDA(_n1))
#define JA_DMAC_DDA(_n1)            (0xb3420000 + (((_n1))/6*0x100 + 0x18 + (((_n1))-((_n1))/6*6) * 0x20))
#define JT_DMAC_DDA(_n1)            JIO_32_RW
#define JN_DMAC_DDA(_n1)            DMAC_DDA
#define JI_DMAC_DDA(_n1)            (_n1)
#define BP_DMAC_DDA_BASE            12
#define BM_DMAC_DDA_BASE            0xfffff000
#define BF_DMAC_DDA_BASE(v)         (((v) & 0xfffff) << 12)
#define BFM_DMAC_DDA_BASE(v)        BM_DMAC_DDA_BASE
#define BF_DMAC_DDA_BASE_V(e)       BF_DMAC_DDA_BASE(BV_DMAC_DDA_BASE__##e)
#define BFM_DMAC_DDA_BASE_V(v)      BM_DMAC_DDA_BASE
#define BP_DMAC_DDA_OFFSET          4
#define BM_DMAC_DDA_OFFSET          0xff0
#define BF_DMAC_DDA_OFFSET(v)       (((v) & 0xff) << 4)
#define BFM_DMAC_DDA_OFFSET(v)      BM_DMAC_DDA_OFFSET
#define BF_DMAC_DDA_OFFSET_V(e)     BF_DMAC_DDA_OFFSET(BV_DMAC_DDA_OFFSET__##e)
#define BFM_DMAC_DDA_OFFSET_V(v)    BM_DMAC_DDA_OFFSET

#define REG_DMAC_DSD(_n1)       jz_reg(DMAC_DSD(_n1))
#define JA_DMAC_DSD(_n1)        (0xb3420000 + (((_n1))/6*0x100 + 0x1c + (((_n1))-((_n1))/6*6) * 0x04))
#define JT_DMAC_DSD(_n1)        JIO_32_RW
#define JN_DMAC_DSD(_n1)        DMAC_DSD
#define JI_DMAC_DSD(_n1)        (_n1)
#define BP_DMAC_DSD_TSD         16
#define BM_DMAC_DSD_TSD         0xffff0000
#define BF_DMAC_DSD_TSD(v)      (((v) & 0xffff) << 16)
#define BFM_DMAC_DSD_TSD(v)     BM_DMAC_DSD_TSD
#define BF_DMAC_DSD_TSD_V(e)    BF_DMAC_DSD_TSD(BV_DMAC_DSD_TSD__##e)
#define BFM_DMAC_DSD_TSD_V(v)   BM_DMAC_DSD_TSD
#define BP_DMAC_DSD_SSD         0
#define BM_DMAC_DSD_SSD         0xffff
#define BF_DMAC_DSD_SSD(v)      (((v) & 0xffff) << 0)
#define BFM_DMAC_DSD_SSD(v)     BM_DMAC_DSD_SSD
#define BF_DMAC_DSD_SSD_V(e)    BF_DMAC_DSD_SSD(BV_DMAC_DSD_SSD__##e)
#define BFM_DMAC_DSD_SSD_V(v)   BM_DMAC_DSD_SSD

#define REG_DMAC_DMACR(_n1)         jz_reg(DMAC_DMACR(_n1))
#define JA_DMAC_DMACR(_n1)          (0xb3420000 + 0x0300 + 0x100 * ((_n1)))
#define JT_DMAC_DMACR(_n1)          JIO_32_RW
#define JN_DMAC_DMACR(_n1)          DMAC_DMACR
#define JI_DMAC_DMACR(_n1)          (_n1)
#define BP_DMAC_DMACR_FMSC          31
#define BM_DMAC_DMACR_FMSC          0x80000000
#define BF_DMAC_DMACR_FMSC(v)       (((v) & 0x1) << 31)
#define BFM_DMAC_DMACR_FMSC(v)      BM_DMAC_DMACR_FMSC
#define BF_DMAC_DMACR_FMSC_V(e)     BF_DMAC_DMACR_FMSC(BV_DMAC_DMACR_FMSC__##e)
#define BFM_DMAC_DMACR_FMSC_V(v)    BM_DMAC_DMACR_FMSC
#define BP_DMAC_DMACR_FSSI          30
#define BM_DMAC_DMACR_FSSI          0x40000000
#define BF_DMAC_DMACR_FSSI(v)       (((v) & 0x1) << 30)
#define BFM_DMAC_DMACR_FSSI(v)      BM_DMAC_DMACR_FSSI
#define BF_DMAC_DMACR_FSSI_V(e)     BF_DMAC_DMACR_FSSI(BV_DMAC_DMACR_FSSI__##e)
#define BFM_DMAC_DMACR_FSSI_V(v)    BM_DMAC_DMACR_FSSI
#define BP_DMAC_DMACR_FTSSI         29
#define BM_DMAC_DMACR_FTSSI         0x20000000
#define BF_DMAC_DMACR_FTSSI(v)      (((v) & 0x1) << 29)
#define BFM_DMAC_DMACR_FTSSI(v)     BM_DMAC_DMACR_FTSSI
#define BF_DMAC_DMACR_FTSSI_V(e)    BF_DMAC_DMACR_FTSSI(BV_DMAC_DMACR_FTSSI__##e)
#define BFM_DMAC_DMACR_FTSSI_V(v)   BM_DMAC_DMACR_FTSSI
#define BP_DMAC_DMACR_FUART         28
#define BM_DMAC_DMACR_FUART         0x10000000
#define BF_DMAC_DMACR_FUART(v)      (((v) & 0x1) << 28)
#define BFM_DMAC_DMACR_FUART(v)     BM_DMAC_DMACR_FUART
#define BF_DMAC_DMACR_FUART_V(e)    BF_DMAC_DMACR_FUART(BV_DMAC_DMACR_FUART__##e)
#define BFM_DMAC_DMACR_FUART_V(v)   BM_DMAC_DMACR_FUART
#define BP_DMAC_DMACR_FAIC          27
#define BM_DMAC_DMACR_FAIC          0x8000000
#define BF_DMAC_DMACR_FAIC(v)       (((v) & 0x1) << 27)
#define BFM_DMAC_DMACR_FAIC(v)      BM_DMAC_DMACR_FAIC
#define BF_DMAC_DMACR_FAIC_V(e)     BF_DMAC_DMACR_FAIC(BV_DMAC_DMACR_FAIC__##e)
#define BFM_DMAC_DMACR_FAIC_V(v)    BM_DMAC_DMACR_FAIC
#define BP_DMAC_DMACR_PR            8
#define BM_DMAC_DMACR_PR            0x300
#define BV_DMAC_DMACR_PR__012345    0x0
#define BV_DMAC_DMACR_PR__120345    0x1
#define BV_DMAC_DMACR_PR__230145    0x2
#define BV_DMAC_DMACR_PR__340125    0x3
#define BF_DMAC_DMACR_PR(v)         (((v) & 0x3) << 8)
#define BFM_DMAC_DMACR_PR(v)        BM_DMAC_DMACR_PR
#define BF_DMAC_DMACR_PR_V(e)       BF_DMAC_DMACR_PR(BV_DMAC_DMACR_PR__##e)
#define BFM_DMAC_DMACR_PR_V(v)      BM_DMAC_DMACR_PR
#define BP_DMAC_DMACR_HLT           3
#define BM_DMAC_DMACR_HLT           0x8
#define BF_DMAC_DMACR_HLT(v)        (((v) & 0x1) << 3)
#define BFM_DMAC_DMACR_HLT(v)       BM_DMAC_DMACR_HLT
#define BF_DMAC_DMACR_HLT_V(e)      BF_DMAC_DMACR_HLT(BV_DMAC_DMACR_HLT__##e)
#define BFM_DMAC_DMACR_HLT_V(v)     BM_DMAC_DMACR_HLT
#define BP_DMAC_DMACR_AR            2
#define BM_DMAC_DMACR_AR            0x4
#define BF_DMAC_DMACR_AR(v)         (((v) & 0x1) << 2)
#define BFM_DMAC_DMACR_AR(v)        BM_DMAC_DMACR_AR
#define BF_DMAC_DMACR_AR_V(e)       BF_DMAC_DMACR_AR(BV_DMAC_DMACR_AR__##e)
#define BFM_DMAC_DMACR_AR_V(v)      BM_DMAC_DMACR_AR
#define BP_DMAC_DMACR_DMAE          0
#define BM_DMAC_DMACR_DMAE          0x1
#define BF_DMAC_DMACR_DMAE(v)       (((v) & 0x1) << 0)
#define BFM_DMAC_DMACR_DMAE(v)      BM_DMAC_DMACR_DMAE
#define BF_DMAC_DMACR_DMAE_V(e)     BF_DMAC_DMACR_DMAE(BV_DMAC_DMACR_DMAE__##e)
#define BFM_DMAC_DMACR_DMAE_V(v)    BM_DMAC_DMACR_DMAE

#define REG_DMAC_DMAIPR(_n1)        jz_reg(DMAC_DMAIPR(_n1))
#define JA_DMAC_DMAIPR(_n1)         (0xb3420000 + 0x0304 + 0x100 * ((_n1)))
#define JT_DMAC_DMAIPR(_n1)         JIO_32_RW
#define JN_DMAC_DMAIPR(_n1)         DMAC_DMAIPR
#define JI_DMAC_DMAIPR(_n1)         (_n1)
#define BP_DMAC_DMAIPR_CIRQ5        5
#define BM_DMAC_DMAIPR_CIRQ5        0x20
#define BF_DMAC_DMAIPR_CIRQ5(v)     (((v) & 0x1) << 5)
#define BFM_DMAC_DMAIPR_CIRQ5(v)    BM_DMAC_DMAIPR_CIRQ5
#define BF_DMAC_DMAIPR_CIRQ5_V(e)   BF_DMAC_DMAIPR_CIRQ5(BV_DMAC_DMAIPR_CIRQ5__##e)
#define BFM_DMAC_DMAIPR_CIRQ5_V(v)  BM_DMAC_DMAIPR_CIRQ5
#define BP_DMAC_DMAIPR_CIRQ4        4
#define BM_DMAC_DMAIPR_CIRQ4        0x10
#define BF_DMAC_DMAIPR_CIRQ4(v)     (((v) & 0x1) << 4)
#define BFM_DMAC_DMAIPR_CIRQ4(v)    BM_DMAC_DMAIPR_CIRQ4
#define BF_DMAC_DMAIPR_CIRQ4_V(e)   BF_DMAC_DMAIPR_CIRQ4(BV_DMAC_DMAIPR_CIRQ4__##e)
#define BFM_DMAC_DMAIPR_CIRQ4_V(v)  BM_DMAC_DMAIPR_CIRQ4
#define BP_DMAC_DMAIPR_CIRQ3        3
#define BM_DMAC_DMAIPR_CIRQ3        0x8
#define BF_DMAC_DMAIPR_CIRQ3(v)     (((v) & 0x1) << 3)
#define BFM_DMAC_DMAIPR_CIRQ3(v)    BM_DMAC_DMAIPR_CIRQ3
#define BF_DMAC_DMAIPR_CIRQ3_V(e)   BF_DMAC_DMAIPR_CIRQ3(BV_DMAC_DMAIPR_CIRQ3__##e)
#define BFM_DMAC_DMAIPR_CIRQ3_V(v)  BM_DMAC_DMAIPR_CIRQ3
#define BP_DMAC_DMAIPR_CIRQ2        2
#define BM_DMAC_DMAIPR_CIRQ2        0x4
#define BF_DMAC_DMAIPR_CIRQ2(v)     (((v) & 0x1) << 2)
#define BFM_DMAC_DMAIPR_CIRQ2(v)    BM_DMAC_DMAIPR_CIRQ2
#define BF_DMAC_DMAIPR_CIRQ2_V(e)   BF_DMAC_DMAIPR_CIRQ2(BV_DMAC_DMAIPR_CIRQ2__##e)
#define BFM_DMAC_DMAIPR_CIRQ2_V(v)  BM_DMAC_DMAIPR_CIRQ2
#define BP_DMAC_DMAIPR_CIRQ1        1
#define BM_DMAC_DMAIPR_CIRQ1        0x2
#define BF_DMAC_DMAIPR_CIRQ1(v)     (((v) & 0x1) << 1)
#define BFM_DMAC_DMAIPR_CIRQ1(v)    BM_DMAC_DMAIPR_CIRQ1
#define BF_DMAC_DMAIPR_CIRQ1_V(e)   BF_DMAC_DMAIPR_CIRQ1(BV_DMAC_DMAIPR_CIRQ1__##e)
#define BFM_DMAC_DMAIPR_CIRQ1_V(v)  BM_DMAC_DMAIPR_CIRQ1
#define BP_DMAC_DMAIPR_CIRQ0        0
#define BM_DMAC_DMAIPR_CIRQ0        0x1
#define BF_DMAC_DMAIPR_CIRQ0(v)     (((v) & 0x1) << 0)
#define BFM_DMAC_DMAIPR_CIRQ0(v)    BM_DMAC_DMAIPR_CIRQ0
#define BF_DMAC_DMAIPR_CIRQ0_V(e)   BF_DMAC_DMAIPR_CIRQ0(BV_DMAC_DMAIPR_CIRQ0__##e)
#define BFM_DMAC_DMAIPR_CIRQ0_V(v)  BM_DMAC_DMAIPR_CIRQ0

#define REG_DMAC_DMADBR(_n1)        jz_reg(DMAC_DMADBR(_n1))
#define JA_DMAC_DMADBR(_n1)         (0xb3420000 + 0x0308 + 0x100 * ((_n1)))
#define JT_DMAC_DMADBR(_n1)         JIO_32_RW
#define JN_DMAC_DMADBR(_n1)         DMAC_DMADBR
#define JI_DMAC_DMADBR(_n1)         (_n1)
#define BP_DMAC_DMADBR_DB5          5
#define BM_DMAC_DMADBR_DB5          0x20
#define BF_DMAC_DMADBR_DB5(v)       (((v) & 0x1) << 5)
#define BFM_DMAC_DMADBR_DB5(v)      BM_DMAC_DMADBR_DB5
#define BF_DMAC_DMADBR_DB5_V(e)     BF_DMAC_DMADBR_DB5(BV_DMAC_DMADBR_DB5__##e)
#define BFM_DMAC_DMADBR_DB5_V(v)    BM_DMAC_DMADBR_DB5
#define BP_DMAC_DMADBR_DB4          4
#define BM_DMAC_DMADBR_DB4          0x10
#define BF_DMAC_DMADBR_DB4(v)       (((v) & 0x1) << 4)
#define BFM_DMAC_DMADBR_DB4(v)      BM_DMAC_DMADBR_DB4
#define BF_DMAC_DMADBR_DB4_V(e)     BF_DMAC_DMADBR_DB4(BV_DMAC_DMADBR_DB4__##e)
#define BFM_DMAC_DMADBR_DB4_V(v)    BM_DMAC_DMADBR_DB4
#define BP_DMAC_DMADBR_DB3          3
#define BM_DMAC_DMADBR_DB3          0x8
#define BF_DMAC_DMADBR_DB3(v)       (((v) & 0x1) << 3)
#define BFM_DMAC_DMADBR_DB3(v)      BM_DMAC_DMADBR_DB3
#define BF_DMAC_DMADBR_DB3_V(e)     BF_DMAC_DMADBR_DB3(BV_DMAC_DMADBR_DB3__##e)
#define BFM_DMAC_DMADBR_DB3_V(v)    BM_DMAC_DMADBR_DB3
#define BP_DMAC_DMADBR_DB2          2
#define BM_DMAC_DMADBR_DB2          0x4
#define BF_DMAC_DMADBR_DB2(v)       (((v) & 0x1) << 2)
#define BFM_DMAC_DMADBR_DB2(v)      BM_DMAC_DMADBR_DB2
#define BF_DMAC_DMADBR_DB2_V(e)     BF_DMAC_DMADBR_DB2(BV_DMAC_DMADBR_DB2__##e)
#define BFM_DMAC_DMADBR_DB2_V(v)    BM_DMAC_DMADBR_DB2
#define BP_DMAC_DMADBR_DB1          1
#define BM_DMAC_DMADBR_DB1          0x2
#define BF_DMAC_DMADBR_DB1(v)       (((v) & 0x1) << 1)
#define BFM_DMAC_DMADBR_DB1(v)      BM_DMAC_DMADBR_DB1
#define BF_DMAC_DMADBR_DB1_V(e)     BF_DMAC_DMADBR_DB1(BV_DMAC_DMADBR_DB1__##e)
#define BFM_DMAC_DMADBR_DB1_V(v)    BM_DMAC_DMADBR_DB1
#define BP_DMAC_DMADBR_DB0          0
#define BM_DMAC_DMADBR_DB0          0x1
#define BF_DMAC_DMADBR_DB0(v)       (((v) & 0x1) << 0)
#define BFM_DMAC_DMADBR_DB0(v)      BM_DMAC_DMADBR_DB0
#define BF_DMAC_DMADBR_DB0_V(e)     BF_DMAC_DMADBR_DB0(BV_DMAC_DMADBR_DB0__##e)
#define BFM_DMAC_DMADBR_DB0_V(v)    BM_DMAC_DMADBR_DB0

#define REG_DMAC_DMADBSR(_n1)       jz_reg(DMAC_DMADBSR(_n1))
#define JA_DMAC_DMADBSR(_n1)        (0xb3420000 + 0x030C + 0x100 * ((_n1)))
#define JT_DMAC_DMADBSR(_n1)        JIO_32_RW
#define JN_DMAC_DMADBSR(_n1)        DMAC_DMADBSR
#define JI_DMAC_DMADBSR(_n1)        (_n1)
#define BP_DMAC_DMADBSR_DBS5        5
#define BM_DMAC_DMADBSR_DBS5        0x20
#define BF_DMAC_DMADBSR_DBS5(v)     (((v) & 0x1) << 5)
#define BFM_DMAC_DMADBSR_DBS5(v)    BM_DMAC_DMADBSR_DBS5
#define BF_DMAC_DMADBSR_DBS5_V(e)   BF_DMAC_DMADBSR_DBS5(BV_DMAC_DMADBSR_DBS5__##e)
#define BFM_DMAC_DMADBSR_DBS5_V(v)  BM_DMAC_DMADBSR_DBS5
#define BP_DMAC_DMADBSR_DBS4        4
#define BM_DMAC_DMADBSR_DBS4        0x10
#define BF_DMAC_DMADBSR_DBS4(v)     (((v) & 0x1) << 4)
#define BFM_DMAC_DMADBSR_DBS4(v)    BM_DMAC_DMADBSR_DBS4
#define BF_DMAC_DMADBSR_DBS4_V(e)   BF_DMAC_DMADBSR_DBS4(BV_DMAC_DMADBSR_DBS4__##e)
#define BFM_DMAC_DMADBSR_DBS4_V(v)  BM_DMAC_DMADBSR_DBS4
#define BP_DMAC_DMADBSR_DBS3        3
#define BM_DMAC_DMADBSR_DBS3        0x8
#define BF_DMAC_DMADBSR_DBS3(v)     (((v) & 0x1) << 3)
#define BFM_DMAC_DMADBSR_DBS3(v)    BM_DMAC_DMADBSR_DBS3
#define BF_DMAC_DMADBSR_DBS3_V(e)   BF_DMAC_DMADBSR_DBS3(BV_DMAC_DMADBSR_DBS3__##e)
#define BFM_DMAC_DMADBSR_DBS3_V(v)  BM_DMAC_DMADBSR_DBS3
#define BP_DMAC_DMADBSR_DBS2        2
#define BM_DMAC_DMADBSR_DBS2        0x4
#define BF_DMAC_DMADBSR_DBS2(v)     (((v) & 0x1) << 2)
#define BFM_DMAC_DMADBSR_DBS2(v)    BM_DMAC_DMADBSR_DBS2
#define BF_DMAC_DMADBSR_DBS2_V(e)   BF_DMAC_DMADBSR_DBS2(BV_DMAC_DMADBSR_DBS2__##e)
#define BFM_DMAC_DMADBSR_DBS2_V(v)  BM_DMAC_DMADBSR_DBS2
#define BP_DMAC_DMADBSR_DBS1        1
#define BM_DMAC_DMADBSR_DBS1        0x2
#define BF_DMAC_DMADBSR_DBS1(v)     (((v) & 0x1) << 1)
#define BFM_DMAC_DMADBSR_DBS1(v)    BM_DMAC_DMADBSR_DBS1
#define BF_DMAC_DMADBSR_DBS1_V(e)   BF_DMAC_DMADBSR_DBS1(BV_DMAC_DMADBSR_DBS1__##e)
#define BFM_DMAC_DMADBSR_DBS1_V(v)  BM_DMAC_DMADBSR_DBS1
#define BP_DMAC_DMADBSR_DBS0        0
#define BM_DMAC_DMADBSR_DBS0        0x1
#define BF_DMAC_DMADBSR_DBS0(v)     (((v) & 0x1) << 0)
#define BFM_DMAC_DMADBSR_DBS0(v)    BM_DMAC_DMADBSR_DBS0
#define BF_DMAC_DMADBSR_DBS0_V(e)   BF_DMAC_DMADBSR_DBS0(BV_DMAC_DMADBSR_DBS0__##e)
#define BFM_DMAC_DMADBSR_DBS0_V(v)  BM_DMAC_DMADBSR_DBS0

#define REG_DMAC_DMACK(_n1) jz_reg(DMAC_DMACK(_n1))
#define JA_DMAC_DMACK(_n1)  (0xb3420000 + 0x0310 + 0x100 * ((_n1)))
#define JT_DMAC_DMACK(_n1)  JIO_32_RW
#define JN_DMAC_DMACK(_n1)  DMAC_DMACK
#define JI_DMAC_DMACK(_n1)  (_n1)

#define REG_DMAC_DMACKS(_n1)    jz_reg(DMAC_DMACKS(_n1))
#define JA_DMAC_DMACKS(_n1)     (0xb3420000 + 0x0314 + 0x100 * ((_n1)))
#define JT_DMAC_DMACKS(_n1)     JIO_32_RW
#define JN_DMAC_DMACKS(_n1)     DMAC_DMACKS
#define JI_DMAC_DMACKS(_n1)     (_n1)

#define REG_DMAC_DMACKC(_n1)    jz_reg(DMAC_DMACKC(_n1))
#define JA_DMAC_DMACKC(_n1)     (0xb3420000 + 0x0318 + 0x100 * ((_n1)))
#define JT_DMAC_DMACKC(_n1)     JIO_32_RW
#define JN_DMAC_DMACKC(_n1)     DMAC_DMACKC
#define JI_DMAC_DMACKC(_n1)     (_n1)

#endif /* __HEADERGEN_DMAC_H__*/
