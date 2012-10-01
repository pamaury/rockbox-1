/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 * $Id$
 *
 * Copyright (C) 2011 by Marcin Bukat
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
#include "cpu.h"
#include "string.h"
#include "usb.h"
#include "usb_drv.h"
#include "usb_core.h"
#include "system.h"
#include "system-target.h"

void usb_init_device(void)
{
    _logf("usb_init_device");
    usb_drv_startup();
}

void usb_attach(void)
{
}

void usb_enable(bool on)
{
    _logf("usb_enable(%d)", on);
    if(on)
        usb_core_init();
    else
        usb_core_exit();
}
