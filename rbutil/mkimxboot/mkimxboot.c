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
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include "mkimxboot.h"
#include "sb.h"
#include "rockpack.h"
#include "md5.h"

/* Supported models */
enum imx_model_t
{
    MODEL_UNKNOWN = -1,
    MODEL_FUZEPLUS = 0,
    /* new models go here */

    NUM_MODELS
};

struct imx_md5sum_t
{
    int model;
    char *md5sum;
};

struct imx_rockpack_entry_t
{
    /* Entry name */
    const char *name;
    /* Size (-1 if no constraint) */
    int size;
};

struct imx_rockpack_t
{
    /* Number of entries */
    int nr_entries;
    /* Entries */
    struct imx_rockpack_entry_t entries[];
};

struct imx_model_desc_t
{
    /* Descriptive name of this model */
    const char *model_name;
    /* Target name (as used in rockpack) */
    const char *target_name;
    /* Number of keys needed to decrypt/encrypt */
    int nr_keys;
    /* Array of keys */
    struct crypto_key_t *keys;
    /* Rockpack safety checks */
    struct imx_rockpack_t *rockpack_desc;
};

static const struct imx_md5sum_t imx_sums[] =
{
    { MODEL_FUZEPLUS, "c3e27620a877dc6b200b97dcb3e0ecc7" }, /* Version 2.38.6 */
};

static struct crypto_key_t zero_key =
{
    .method = CRYPTO_KEY,
    .u.key = {0}
};

static struct imx_rockpack_t imx_dualboot_bootloader_pack =
{
    4,
    {
        /* Dualboot loading address (32-bit) */
        { "dualboot.addr", 4 },
        /* Dualboot code */
        { "dualboot.code", -1 },
        /* Bootloader loading address (32-bit) */
        { "bootloader.addr", 4 },
        /* Bootloader code */
        { "bootloader.code", -1 },
    }
};

static const struct imx_model_desc_t imx_models[] =
{
    [MODEL_FUZEPLUS]  = { "Fuze+", "fuzeplus", 1, &zero_key, &imx_dualboot_bootloader_pack },
};

#define NR_IMX_SUMS     (sizeof(imx_sums) / sizeof(imx_sums[0]))
#define NR_IMX_MODELS   (sizeof(imx_models) / sizeof(imx_models[0]))

#define MAGIC_ROCK      0x726f636b /* 'rock' */
#define MAGIC_RECOVERY  0xfee1dead
#define MAGIC_NORMAL    0xcafebabe

static enum imx_error_t patch_std_zero_host_play(int jump_before, int model,
    enum imx_output_type_t type, struct sb_file_t *sb_file,
    void *dualboot, size_t dualboot_size, uint32_t dualboot_addr,
    void *bootloader, size_t bootloader_size, uint32_t bootloader_addr)
{
    printf("[INFO] Load %lu bytes dualboot at 0x%08x\n", dualboot_size, dualboot_addr);
    printf("[INFO] Load %lu bytes bootloader at 0x%08x\n", bootloader_size, bootloader_addr);
    /* We assume the file has three boot sections: ____, host, play and one
     * resource section rsrc.
     *
     * Dual Boot:
     * ----------
     * We patch the file by inserting the dualboot code before the <jump_before>th
     * call in the ____ section. We give it as argument the section name 'rock'
     * and add a section called 'rock' after rsrc which contains the bootloader.
     *
     * Single Boot & Recovery:
     * -----------------------
     * We patch the file by inserting the bootloader code after the <jump_before>th
     * call in the ____ section and get rid of everything else. In recovery mode,
     * we give 0xfee1dead as argument */

    /* Do not override real key and IV */
    sb_file->override_crypto_iv = false;
    sb_file->override_real_key = false;

    /* first locate the good instruction */
    struct sb_section_t *sec = &sb_file->sections[0];
    int jump_idx = 0;
    while(jump_idx < sec->nr_insts && jump_before > 0)
        if(sec->insts[jump_idx++].inst == SB_INST_CALL)
            jump_before--;
    if(jump_idx == sec->nr_insts)
    {
        printf("[ERR] Cannot locate call in section ____\n");
        return IMX_DONT_KNOW_HOW_TO_PATCH;
    }

    if(type == IMX_DUALBOOT)
    {
        /* create a new instruction array with a hole for two instructions */
        struct sb_inst_t *new_insts = xmalloc(sizeof(struct sb_inst_t) * (sec->nr_insts + 2));
        memcpy(new_insts, sec->insts, sizeof(struct sb_inst_t) * jump_idx);
        memcpy(new_insts + jump_idx + 2, sec->insts + jump_idx,
            sizeof(struct sb_inst_t) * (sec->nr_insts - jump_idx));
        /* first instruction is be a load */
        struct sb_inst_t *load = &new_insts[jump_idx];
        memset(load, 0, sizeof(struct sb_inst_t));
        load->inst = SB_INST_LOAD;
        load->size = dualboot_size;
        load->addr = dualboot_addr;
        /* duplicate memory because it will be free'd */
        load->data = memdup(dualboot, dualboot_size);
        /* second instruction is a call */
        struct sb_inst_t *call = &new_insts[jump_idx + 1];
        memset(call, 0, sizeof(struct sb_inst_t));
        call->inst = SB_INST_CALL;
        call->addr = dualboot_addr;
        call->argument = MAGIC_ROCK;
        /* free old instruction array */
        free(sec->insts);
        sec->insts = new_insts;
        sec->nr_insts += 2;

        /* create a new section */
        struct sb_section_t rock_sec;
        memset(&rock_sec, 0, sizeof(rock_sec));
        /* section has two instructions: load and call */
        rock_sec.identifier = MAGIC_ROCK;
        rock_sec.alignment = BLOCK_SIZE;
        rock_sec.nr_insts = 2;
        rock_sec.insts = xmalloc(2 * sizeof(struct sb_inst_t));
        memset(rock_sec.insts, 0, 2 * sizeof(struct sb_inst_t));
        rock_sec.insts[0].inst = SB_INST_LOAD;
        rock_sec.insts[0].size = bootloader_size;
        rock_sec.insts[0].data = memdup(bootloader, bootloader_size);
        rock_sec.insts[0].addr = bootloader_addr;
        rock_sec.insts[1].inst = SB_INST_JUMP;
        rock_sec.insts[1].addr = bootloader_addr;
        rock_sec.insts[1].argument = MAGIC_NORMAL;

        sb_file->sections = augment_array(sb_file->sections,
            sizeof(struct sb_section_t), sb_file->nr_sections,
            &rock_sec, 1);
        sb_file->nr_sections++;

        return IMX_SUCCESS;
    }
    else if(type == IMX_SINGLEBOOT || type == IMX_RECOVERY)
    {
        bool recovery = type == IMX_RECOVERY;
        /* remove everything after the call and add two instructions: load and call */
        struct sb_inst_t *new_insts = xmalloc(sizeof(struct sb_inst_t) * (jump_idx + 2));
        memcpy(new_insts, sec->insts, sizeof(struct sb_inst_t) * jump_idx);
        for(int i = jump_idx; i < sec->nr_insts; i++)
            sb_free_instruction(sec->insts[i]);
        memset(new_insts + jump_idx, 0, 2 * sizeof(struct sb_inst_t));
        new_insts[jump_idx + 0].inst = SB_INST_LOAD;
        new_insts[jump_idx + 0].size = bootloader_size;
        new_insts[jump_idx + 0].data = memdup(bootloader, bootloader_size);
        new_insts[jump_idx + 0].addr = bootloader_addr;
        new_insts[jump_idx + 1].inst = SB_INST_JUMP;
        new_insts[jump_idx + 1].addr = bootloader_addr;
        new_insts[jump_idx + 1].argument = recovery ? MAGIC_RECOVERY : MAGIC_NORMAL;
        
        free(sec->insts);
        sec->insts = new_insts;
        sec->nr_insts = jump_idx + 2;
        /* remove all other sections */
        for(int i = 1; i < sb_file->nr_sections; i++)
            sb_free_section(sb_file->sections[i]);
        struct sb_section_t *new_sec = xmalloc(sizeof(struct sb_section_t));
        memcpy(new_sec, &sb_file->sections[0], sizeof(struct sb_section_t));
        free(sb_file->sections);
        sb_file->sections = new_sec;
        sb_file->nr_sections = 1;

        return IMX_SUCCESS;
    }
    else
    {
        printf("[ERR] Bad output type !\n");
        return IMX_DONT_KNOW_HOW_TO_PATCH;
    }
}

static enum imx_error_t patch_firmware(int model, enum imx_output_type_t type,
    struct sb_file_t *sb_file, struct rockpack_pack_t *pack)
{
#define rbpk_data(name) rockpack_entry_data(pack, rockpack_search(pack, name))
#define rbpk_size(name) rockpack_entry_size(pack, rockpack_search(pack, name))
#define rbpk_uint32(name) *(uint32_t *)rbpk_data(name)
    switch(model)
    {
        case MODEL_FUZEPLUS:
            /* The Fuze+ uses the standard ____, host, play sections, patch after third
             * call in ____ section */
            return patch_std_zero_host_play(3, model, type, sb_file,
                rbpk_data("dualboot.code"), rbpk_size("dualboot.code"),
                rbpk_uint32("dualboot.addr"),
                rbpk_data("bootloader.code"), rbpk_size("bootloader.code"),
                rbpk_uint32("bootloader.addr"));
        default:
            return IMX_DONT_KNOW_HOW_TO_PATCH;
    }
}

static void imx_printf(void *user, bool error, color_t c, const char *fmt, ...)
{
    (void) user;
    (void) c;
    va_list args;
    va_start(args, fmt);
    /*
    if(error)
        printf("[ERR] ");
    else
        printf("[INFO] ");
    */
    vprintf(fmt, args);
    va_end(args);
}

enum imx_error_t mkimxboot(const char *infile, const char *bootfile,
    const char *outfile, struct imx_option_t opt)
{
    /* Dump tables */
    do
    {
        printf("[INFO] mkimxboot models:\n");
        for(int i = 0; i < NR_IMX_MODELS; i++)
        {
            printf("[INFO]   %s: idx=%d target_name=%s\n",
                imx_models[i].model_name, i, imx_models[i].target_name);
        }
        printf("[INFO] mkimxboot mapping:\n");
        for(int i = 0; i < NR_IMX_SUMS; i++)
        {
            printf("[INFO]   md5sum=%s -> idx=%d\n", imx_sums[i].md5sum,
                imx_sums[i].model);
        }
    }while(0);
    /* compute MD5 sum of the file */
    uint8_t file_md5sum[16];
    do
    {
        FILE *f = fopen(infile, "rb");
        if(f == NULL)
        {
            printf("[ERR] Cannot open input file\n");
            return IMX_OPEN_ERROR;
        }
        fseek(f, 0, SEEK_END);
        size_t sz = ftell(f);
        fseek(f, 0, SEEK_SET);
        void *buf = xmalloc(sz);
        if(fread(buf, sz, 1, f) != 1)
        {
            fclose(f);
            free(buf);
            printf("[ERR] Cannot read file\n");
            return IMX_READ_ERROR;
        }
        fclose(f);
        md5_context ctx;
        md5_starts(&ctx);
        md5_update(&ctx, buf, sz);
        md5_finish(&ctx, file_md5sum);
        free(buf);
    }while(0);
    printf("[INFO] MD5 sum of the file: ");
    print_hex(file_md5sum, 16, true);
    /* find model */
    int model;
    do
    {
        int i = 0;
        while(i < NR_IMX_SUMS)
        {
            uint8_t md5[20];
            if(strlen(imx_sums[i].md5sum) != 32)
            {
                printf("[INFO] Invalid MD5 sum in imx_sums\n");
                return IMX_ERROR;
            }
            for(int j = 0; j < 16; j++)
            {
                byte a, b;
                if(convxdigit(imx_sums[i].md5sum[2 * j], &a) || convxdigit(imx_sums[i].md5sum[2 * j + 1], &b))
                    return false;
                md5[j] = (a << 4) | b;
            }
            if(memcmp(file_md5sum, md5, 16) == 0)
                break;
            i++;
        }
        if(i == NR_IMX_SUMS)
        {
            printf("[ERR] MD5 sum doesn't match any known file\n");
            return IMX_NO_MATCH;
        }
        model = imx_sums[i].model;
    }while(0);
    printf("[INFO] File is for model %d (%s)\n", model, imx_models[model].model_name);
    /* load rockbox file */
    uint8_t *boot;
    size_t boot_size;
    do
    {
        FILE *f = fopen(bootfile, "rb");
        if(f == NULL)
        {
            printf("[ERR] Cannot open boot file\n");
            return IMX_OPEN_ERROR;
        }
        fseek(f, 0, SEEK_END);
        boot_size = ftell(f);
        fseek(f, 0, SEEK_SET);
        boot = xmalloc(boot_size);
        if(fread(boot, boot_size, 1, f) != 1)
        {
            free(boot);
            fclose(f);
            printf("[ERR] Cannot read boot file\n");
            return IMX_READ_ERROR;
        }
        fclose(f);
    }while(0);
    /* load boot file */
    struct rockpack_pack_t *pack = rockpack_read(boot, boot_size);
    free(boot);
    if(rockpack_error(pack))
    {
        printf("[ERR] Bootloader is invalid (%s)\n", rockpack_error_msg(pack));
        return IMX_BOOT_INVALID;
    }
    /* check boot file */
    struct imx_rockpack_t *ref_pack = imx_models[model].rockpack_desc;
    
    printf("[INFO] Bootloader rockpack:\n");
    printf("[INFO] Target = '%s'\n", rockpack_target(pack));
    for(size_t i = 0; i < rockpack_nr_entries(pack); i++)
    {
        printf("[INFO] Entry %u: '%s' (%u bytes)\n", (unsigned)i,
            rockpack_entry_name(pack, i), (unsigned)rockpack_entry_size(pack, i));
    }
    printf("[INFO] Reference rockpack:\n");
    printf("[INFO] Target = '%s'\n", imx_models[model].target_name);
    for(int i = 0; i < ref_pack->nr_entries; i++)
    {
        printf("[INFO] Entry '%s': ", ref_pack->entries[i].name);
        if(ref_pack->entries[i].size == -1)
            printf("no size constraint\n");
        else
            printf("exactly %u bytes\n", ref_pack->entries[i].size);
    }

    if(strcmp(rockpack_target(pack), imx_models[model].target_name) != 0)
    {
        printf("[ERR] Bootloader target mismatch\n");
        return IMX_BOOT_MISMATCH;
    }
    for(int i = 0; i < ref_pack->nr_entries; i++)
    {
        size_t idx = rockpack_search(pack, ref_pack->entries[i].name);
        if(rockpack_error(pack))
        {
            printf("[ERR] Bootloader file has no '%s' entry\n",
                ref_pack->entries[i].name);
            return IMX_BOOT_MISMATCH;
        }
        if(ref_pack->entries[i].size != -1 &&
                rockpack_entry_size(pack, idx) != ref_pack->entries[i].size)
        {
            printf("[ERR] Bootloader file entry '%s' has wrong size\n",
                ref_pack->entries[i].name);
            return IMX_BOOT_MISMATCH;
        }
    }
    /* load OF file */
    struct sb_file_t *sb_file;
    do
    {
        enum sb_error_t err;
        g_debug = opt.debug;
        clear_keys();
        add_keys(imx_models[model].keys, imx_models[model].nr_keys);
        sb_file = sb_read_file(infile, false, NULL, &imx_printf, &err);
        if(sb_file == NULL)
        {
            clear_keys();
            free(boot);
            return IMX_FIRST_SB_ERROR + err;
        }
    }while(0);
    /* produce file */
    enum imx_error_t ret = patch_firmware(model, opt.output, sb_file, pack);
    if(ret == IMX_SUCCESS)
        ret = sb_write_file(sb_file, outfile);

    clear_keys();
    sb_free(sb_file);
    rockpack_free(pack);
    return ret;
}
