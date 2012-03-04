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
#include "rockpack.h"
#include <stdio.h>

#define ret_error(p, e) do {p->error = e; return; } while(0)
#define ret_error_val(p, e, v) do {p->error = e; return v; } while(0)

static rockpack_debug_fn_t rbpk_debug_fn = NULL;

#define rbpk_debug(...) do {if(rbpk_debug_fn) rbpk_debug_fn(__VA_ARGS__); } while(0)

static void *memdup(const void *buf, size_t s)
{
    void *data = malloc(s);
    if(data != NULL)
        memcpy(data, buf, s);
    return data;
}

static char *read_str(char arr[], size_t s)
{
    char *str = malloc(s + 1);
    str[s] = 0;
    memcpy(str, arr, s);
    return str;
}

uint32_t rockpack_checksum(void *buffer, size_t size)
{
    uint32_t sum = 0;
    uint8_t *p = buffer;
    while(size-- > 0)
        sum += *p++;
    return sum;
}

void rockpack_debug(rockpack_debug_fn_t fn)
{
    rbpk_debug_fn = fn;
}

enum rockpack_error_t rockpack_error(struct rockpack_pack_t *p)
{
    return p == NULL ? ROCKPACK_NULL_PTR : p->error;
}

const char *rockpack_error_msg(struct rockpack_pack_t *p)
{
    if(p == NULL) return "NULL pack";
    switch(p->error)
    {
        case ROCKPACK_SUCCESS: return "Success";
        case ROCKPACK_ERROR: return "Error";
        case ROCKPACK_BAD_CHECKSUM: return "Bad checksum";
        case ROCKPACK_BAD_SIZE: return "Bad size";
        case ROCKPACK_BAD_ENTRY: return "Bad entry";
        case ROCKPACK_BAD_ALLOC: return "Bad alloc";
        case ROCKPACK_BAD_NAME: return "Bad name";
        case ROCKPACK_BAD_TARGET: return "Bad target";
        case ROCKPACK_NULL_PTR: return "NULL pointer";
        case ROCKPACK_NOT_FOUND: return "Not found";
        case ROCKPACK_BAD_MAGIC: return "Bad magic";
        case ROCKPACK_BAD_VERSION: return "Bad version";
        default: return "Unknown error";
    }
}

struct rockpack_pack_t *rockpack_new(void)
{
    struct rockpack_pack_t *pack = malloc(sizeof(struct rockpack_pack_t));
    if(pack == NULL) return NULL;
    memset(pack, 0, sizeof(struct rockpack_pack_t));
    return pack;
}

void rockpack_free(struct rockpack_pack_t *p)
{
    if(p == NULL) return;
    for(size_t i = 0; i < p->nr_entries; i++)
    {
        free(p->entries[i].data);
        free(p->entries[i].name);
    }
    free(p->entries);
    free(p->target);
    free(p);
}

struct rockpack_pack_t *rockpack_read(void *buffer, size_t size)
{
    struct rockpack_pack_t *pack = rockpack_new();
    /* check size > size of header */
    rbpk_debug("[rockpack_read] size=%u hdr_size=%u\n", size,
        sizeof(struct rockpack_header_t));
    if(size < sizeof(struct rockpack_header_t))
        ret_error_val(pack, ROCKPACK_BAD_SIZE, pack);
    struct rockpack_header_t *hdr = buffer;
    /* check magic */
    rbpk_debug("[rockpack_read] magic=0x%x\n", hdr->magic);
    if(hdr->magic != ROCKPACK_MAGIC)
        ret_error_val(pack, ROCKPACK_BAD_MAGIC, pack);
    /* check version */
    rbpk_debug("[rockpack_read] version=%d\n", hdr->version);
    if(hdr->version != ROCKPACK_VERSION)
        ret_error_val(pack, ROCKPACK_BAD_VERSION, pack);
    /* check file size */
    rbpk_debug("[rockpack_read] size=%u expected_size=%u\n", size, hdr->size);
    if(hdr->size != size)
        ret_error_val(pack, ROCKPACK_BAD_SIZE, pack);
    rbpk_debug("[rockpack_write] nr_entries=%u\n", hdr->nr_entries);
    /* check headers size */
    size_t tot_hdr_size = sizeof(struct rockpack_header_t)
        + hdr->nr_entries * sizeof(struct rockpack_entry_t);
    rbpk_debug("[rockpack_read] size=%u tot_hdr_size=%u\n", size, tot_hdr_size);
    if(tot_hdr_size > size)
        ret_error_val(pack, ROCKPACK_BAD_SIZE, pack);
    /* checksum */
    tot_hdr_size -= offsetof(struct rockpack_header_t, target);
    uint32_t chksum = rockpack_checksum(&hdr->target, tot_hdr_size);
    rbpk_debug("[rockpack_read] checksum=0x%x expected=0x%x\n",
        chksum, hdr->checksum);
    if(hdr->checksum != chksum)
        ret_error_val(pack, ROCKPACK_BAD_CHECKSUM, pack);
    /* extract target */
    pack->target = read_str(hdr->target, ROCKPACK_TARGET_LEN);
    rbpk_debug("[rockpack_read] target=%s\n", pack->target);
    /* read entries */
    struct rockpack_entry_t *entry = (void *)(hdr + 1);
    for(size_t i = 0; i < hdr->nr_entries; i++, entry++)
    {
        rbpk_debug("[rockpack_read][entry %u] offset=%u size=%u\n",
            i, entry->offset, entry->size);
        /* check offset and size */
        if(entry->offset + entry->size > size)
            ret_error_val(pack, ROCKPACK_BAD_SIZE, pack);
        /* check checksum */
        void *data = (uint8_t *)buffer + entry->offset;
        chksum = rockpack_checksum(data, entry->size);
        rbpk_debug("[rockpack_read][entry %u] checksum=0x%x expected=0x%x\n",
            i, chksum, entry->checksum);
        if(chksum != entry->checksum)
            ret_error_val(pack, ROCKPACK_BAD_CHECKSUM, pack);
        /* add entry */
        char *name = read_str(entry->name, ROCKPACK_NAME_LEN);
        rbpk_debug("[rockpack_read][entry %u] name=%s\n", i, name);
        rockpack_add(pack, name, data, entry->size);
        free(name);
        if(rockpack_error(pack))
            return pack;
    }
    
    ret_error_val(pack, ROCKPACK_SUCCESS, pack);
}

void *rockpack_write(struct rockpack_pack_t *p, size_t *out_size)
{
    /* compute total size */
    size_t tot_hdr_size = sizeof(struct rockpack_header_t)
        + p->nr_entries * sizeof(struct rockpack_entry_t);
    size_t tot_size = tot_hdr_size;
    for(size_t i = 0; i < p->nr_entries; i++)
        tot_size += p->entries[i].size;
    rbpk_debug("[rockpack_write] nr_entries=%d\n", p->nr_entries);
    rbpk_debug("[rockpack_write] tot_size=%d tot_hdr_size=%d\n", tot_size,
        tot_hdr_size);
    /* alloc data */
    *out_size = tot_size;
    uint8_t *data = malloc(tot_size);
    if(data == NULL)
        ret_error_val(p, ROCKPACK_BAD_ALLOC, NULL);
    memset(data, 0, tot_size);
    /* write headers */
    struct rockpack_header_t *hdr = (void *)data;
    hdr->magic = ROCKPACK_MAGIC;
    hdr->version = ROCKPACK_VERSION;
    hdr->size = tot_size;
    hdr->nr_entries = p->nr_entries;
    rbpk_debug("[rockpack_write] magic=0x%x\n", ROCKPACK_MAGIC);
    rbpk_debug("[rockpack_write] version=%d\n", ROCKPACK_VERSION);
    rbpk_debug("[rockpack_write] target=%s\n", p->target);
    // we *WANT* strncpy, and *NOT* strlcpy !!
    if(p->target)
        strncpy(hdr->target, p->target, ROCKPACK_TARGET_LEN);
    /* write entries and data */
    struct rockpack_entry_t *entry = (void *)(hdr + 1);
    size_t offset = tot_hdr_size;
    for(size_t i = 0; i < p->nr_entries; i++, entry++)
    {
        // we *WANT* strncpy, and *NOT* strlcpy !!
        strncpy(entry->name, p->entries[i].name, ROCKPACK_NAME_LEN);
        entry->offset = offset;
        entry->size = p->entries[i].size;
        entry->checksum = rockpack_checksum(p->entries[i].data, entry->size);

        rbpk_debug("[rockpack_write][entry %u] name=%s offset=%u size=%u checksum=0x%x\n",
            i, p->entries[i].name, entry->offset, entry->size, entry->checksum);

        memcpy(data + entry->offset, p->entries[i].data, entry->size);
        
        offset += entry->size;
    }
    /* compute checksum */
    tot_hdr_size -= offsetof(struct rockpack_header_t, target);
    hdr->checksum = rockpack_checksum(&hdr->target, tot_hdr_size);
    rbpk_debug("[rockpack_write] checksum=0x%x\n", hdr->checksum);

    ret_error_val(p, ROCKPACK_SUCCESS, data);
}

void rockpack_add(struct rockpack_pack_t *p, char *name, void *data, size_t sz)
{
    if(strlen(name) > ROCKPACK_NAME_LEN)
        ret_error(p, ROCKPACK_BAD_NAME);
    p->entries = realloc(p->entries, (p->nr_entries + 1) * sizeof(struct rockpack_pack_entry_t));
    if(p->entries == NULL)
        ret_error(p, ROCKPACK_BAD_ALLOC);
    p->entries[p->nr_entries].name = memdup(name, strlen(name) + 1);
    p->entries[p->nr_entries].data = memdup(data, sz);
    p->entries[p->nr_entries].size = sz;
    
    if(p->entries[p->nr_entries].name == NULL || p->entries[p->nr_entries].data == NULL)
        ret_error(p, ROCKPACK_BAD_ALLOC);
    p->nr_entries++;
    ret_error(p, ROCKPACK_SUCCESS);
}

void rockpack_remove(struct rockpack_pack_t *p, size_t index)
{
    if(index >= p->nr_entries)
        ret_error(p, ROCKPACK_BAD_ENTRY);
    free(p->entries[index].data);
    free(p->entries[index].name);
    memmove(p->entries + index, p->entries + index + 1,
        (p->nr_entries - index - 1) * sizeof(struct rockpack_pack_entry_t));
    p->nr_entries--;
    ret_error(p, ROCKPACK_SUCCESS);
}

size_t rockpack_search(struct rockpack_pack_t *p, const char *name)
{
    for(size_t i = 0; i < p->nr_entries; i++)
        if(strcmp(name, p->entries[i].name) == 0)
            ret_error_val(p, ROCKPACK_SUCCESS, i);
    ret_error_val(p, ROCKPACK_NOT_FOUND, p->nr_entries);
}

size_t rockpack_nr_entries(struct rockpack_pack_t *p)
{
    ret_error_val(p, ROCKPACK_SUCCESS, p->nr_entries);
}


const char *rockpack_entry_name(struct rockpack_pack_t *p, size_t idx)
{
    if(idx >= p->nr_entries)
        ret_error_val(p, ROCKPACK_BAD_ENTRY, NULL);
    ret_error_val(p, ROCKPACK_SUCCESS, p->entries[idx].name);
}

size_t rockpack_entry_size(struct rockpack_pack_t *p, size_t idx)
{
    if(idx >= p->nr_entries)
        ret_error_val(p, ROCKPACK_BAD_ENTRY, 0);
    ret_error_val(p, ROCKPACK_SUCCESS, p->entries[idx].size);
}

void *rockpack_entry_data(struct rockpack_pack_t *p, size_t idx)
{
    if(idx >= p->nr_entries)
        ret_error_val(p, ROCKPACK_BAD_ENTRY, NULL);
    ret_error_val(p, ROCKPACK_SUCCESS, p->entries[idx].data);
}

const char *rockpack_target(struct rockpack_pack_t *p)
{
    ret_error_val(p, ROCKPACK_SUCCESS, p->target);
}

void rockpack_retarget(struct rockpack_pack_t *p, const char *target)
{
    if(strlen(target) > ROCKPACK_TARGET_LEN)
        ret_error(p, ROCKPACK_BAD_TARGET);
    free(p->target);
    p->target = memdup(target, strlen(target) + 1);
    if(p->target == NULL)
        ret_error(p, ROCKPACK_BAD_ALLOC);
    ret_error(p, ROCKPACK_SUCCESS);
}
