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
#ifndef __ROCKPACK__
#define __ROCKPACK__

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

/*
 * On-disk format
 */

#define ROCKPACK_MAGIC  ('K' << 24 | 'P' << 16 | 'B' << 8 | 'R')
#define ROCKPACK_TARGET_LEN 16
#define ROCKPACK_VERSION    1
#define ROCKPACK_NAME_LEN   16

struct rockpack_header_t
{
    uint32_t magic; /* ROCKPACK_MAGIC */
    uint32_t checksum; /* of reset of header + entries */
    char target[ROCKPACK_TARGET_LEN];
    uint32_t size; /* total size */
    uint16_t version; /* ROCKPACK_VERSION */
    uint16_t nr_entries;
};

struct rockpack_entry_t
{
    char name[ROCKPACK_NAME_LEN];
    uint32_t offset; /* absolute offset */
    uint32_t size;
    uint32_t checksum;
};

/*
 * Library structure
 */
enum rockpack_error_t
{
    ROCKPACK_SUCCESS = 0,
    ROCKPACK_ERROR = -1,
    ROCKPACK_BAD_CHECKSUM = -2,
    ROCKPACK_BAD_SIZE = -3,
    ROCKPACK_BAD_ENTRY = -4,
    ROCKPACK_BAD_ALLOC = -5,
    ROCKPACK_BAD_NAME = -6,
    ROCKPACK_BAD_TARGET = -7,
    ROCKPACK_NULL_PTR = -8,
    ROCKPACK_NOT_FOUND = -9,
    ROCKPACK_BAD_MAGIC = -10,
    ROCKPACK_BAD_VERSION = -11,
};

struct rockpack_pack_t
{
    enum rockpack_error_t error;
    char *target;
    size_t nr_entries;
    struct rockpack_pack_entry_t *entries;
};

struct rockpack_pack_entry_t
{
    char *name;
    void *data;
    size_t size;
};

/*
 * Utility functions
 */

uint32_t rockpack_checksum(void *buffer, size_t size);

typedef void (*rockpack_debug_fn_t)(const char *fmt, ...);
void rockpack_debug(rockpack_debug_fn_t fn);

/*
 * Manipulation API
 */

/* get the error associated to a pack */
enum rockpack_error_t rockpack_error(struct rockpack_pack_t *p);
/* get the error message associated to the pack error */
const char *rockpack_error_msg(struct rockpack_pack_t *p);
/* create a new pack */
struct rockpack_pack_t *rockpack_new(void);
/* destroy an existing pack */
void rockpack_free(struct rockpack_pack_t *p);
/* read a pack from memory */
struct rockpack_pack_t *rockpack_read(void *buffer, size_t size);
/* write a pack to memory (returned buffer must be freed using free()) */
void *rockpack_write(struct rockpack_pack_t *p, size_t *out_size);
/* add an entry to a pack (data and name are copied) */
void rockpack_add(struct rockpack_pack_t *p, char *name, void *data, size_t sz);
/* remove an entry from a pack */
void rockpack_remove(struct rockpack_pack_t *p, size_t index);
/* look for an entry in a pack (use rockpack_error to check for success) */
size_t rockpack_search(struct rockpack_pack_t *p, const char *name);
/* get number of entries */
size_t rockpack_nr_entries(struct rockpack_pack_t *p);
/* get entry name */
const char *rockpack_entry_name(struct rockpack_pack_t *p, size_t idx);
/* get entry size */
size_t rockpack_entry_size(struct rockpack_pack_t *p, size_t idx);
/* get entry data */
void *rockpack_entry_data(struct rockpack_pack_t *p, size_t idx);
/* get target */
const char *rockpack_target(struct rockpack_pack_t *p);
/* set target (string is copied) */
void rockpack_retarget(struct rockpack_pack_t *p, const char *target);

#endif /* __ROCKPACK__ */