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
#include "dualboot/dualboot.h"

struct player_t
{
    const char *name;
    /* for ams */
    bool has_nrv2e_d8;
    void *dualboot;
    size_t dualboot_size;
    /* for imx */
    bool has_addr;
    uint32_t dualboot_addr;
    uint32_t bootloader_addr;
} players[] =
{
#define ams_entry(name) {#name, true, dualboot_##name, sizeof(dualboot_##name), false, -1, -1}
#define imx_entry(name) {#name, false, dualboot_##name, sizeof(dualboot_##name), true, 0, 0x40000000}
    ams_entry(e200v2),
    ams_entry(c200v2),
    ams_entry(m200v4),
    ams_entry(clip),
    ams_entry(clipv2),
    ams_entry(clipplus),
    ams_entry(clipzip),
    ams_entry(fuze),
    ams_entry(fuzev2),
    imx_entry(fuzeplus),
};

void usage(void)
{
    printf("Usage: dualboot_scramble <player> <bootloader> <output>\n");
    printf("Players: e200v2, c200v2, m200v4, clip, clipv2, clipzip\n");
    printf("         clipplus, fuze, fuzev2, fuzeplus\n");
    exit(1);
}

int find_entry(const char *name)
{
    for(size_t i = 0; i < sizeof(players) / sizeof(players[0]); i++)
    {
        if(strcmp(name, players[i].name) == 0)
            return i;
    }
    return -1;
}

int main(int argc, char **argv)
{
    if(argc != 4)
        usage();
    int idx = find_entry(argv[1]);
    if(idx < 0)
    {
        fprintf(stderr, "Unsupported player: %s\n", argv[1]);
        return 1;
    }
    struct rockpack_pack_t *pack = rockpack_new();
    rockpack_retarget(pack, argv[1]);
    
#define add_entry(name, content, size) \
    do { \
        rockpack_add(pack, name, content, size); \
        if(rockpack_error(pack)) \
        { \
            fprintf(stderr, "Couldn't add " name " to pack (%s)\n", rockpack_error_msg(pack)); \
            return 2; \
        } \
    }while(0)
    
    if(players[idx].has_nrv2e_d8)
        add_entry("nrv2e_d8", nrv2e_d8, sizeof(nrv2e_d8));
    add_entry("dualboot.code", players[idx].dualboot, players[idx].dualboot_size);

    FILE *f = fopen(argv[2], "rb");
    if(f == NULL)
    {
        perror("Cannot open bootloader file");
        rockpack_free(pack);
        return 3;
    }
    fseek(f, 0, SEEK_END);
    size_t size = ftell(f);
    fseek(f, 0, SEEK_SET);
    void *data = malloc(size);
    if(fread(data, size, 1, f) != 1)
    {
        perror("Cannot read bootloader file");
        return 4;
    }
    fclose(f);
    
    add_entry("bootloader.code", data, size);
    if(players[idx].has_addr)
    {
        add_entry("dualboot.addr", &players[idx].dualboot_addr, sizeof(players[idx].dualboot_addr));
        add_entry("bootloader.addr", &players[idx].bootloader_addr, sizeof(players[idx].bootloader_addr));
    }
    data = rockpack_write(pack, &size);
    if(rockpack_error(pack))
    {
        fprintf(stderr, "Couldn't build output pack image(%s)\n", rockpack_error_msg(pack));
        return 5;
    }
    rockpack_free(pack);

    f = fopen(argv[3], "wb");
    if(f == NULL)
    {
        perror("Cannot open output file");
        return 6;
    }
    fwrite(data, size, 1, f);
    fclose(f);

    return 0;
}
