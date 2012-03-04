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
#include <getopt.h>
#include <stdarg.h>
#include <string.h>

#define DEFAULT_SEP '%'

void usage(void)
{
    printf("Usage: rbpack [options/actions]\n");
    printf("Options:\n");
    printf("  --help/-?           Display this help\n");
    printf("  --debug/-d          Enable debug output\n");
    printf("  --add/-a <file>     Add a file to the pack\n");
    printf("  --rm/-r <name>      Remove entry named <name>\n");
    printf("  --info/-i           Display pack information\n");
    printf("  --list/-l           List all entries in the pack\n");
    printf("  --target/-t <t>     Set target to <t>\n");
    printf("  --write-/w <f>      Write pack to <f>\n");
    printf("  --read/-e <f>       Read pack from <f>\n");
    printf("  --sep/-s <s>        Set name separator to <s>\n");
    printf("  --extract/-x <file> Extract entry to file\n");
    printf("If the output pack already exists, it is overwritten\n");
    printf("Supported input/output filenames:\n");
    printf(" * filename\n");
    printf("   The basename is used as a name.\n");
    printf(" * nameXfilename where X is the separator (default is %c)\n",
        DEFAULT_SEP);
    printf("   The separator can be escaped in the name (default is %c%c)\n",
        DEFAULT_SEP, DEFAULT_SEP);
    exit(1);
}

void debug_printf(const char *fmt, ...)
{
    va_list vl;
    va_start(vl, fmt);
    vprintf(fmt, vl);
    va_end(vl);
}

bool parse_name(char sep, const char *str, char **name, char **filename)
{
    size_t len = strlen(str);
    *name = malloc(len + 1);
    *filename = malloc(len + 1);
    /* look for separator */
    size_t i = 0;
    size_t name_pos = 0;
    while(i < len)
    {
        if(str[i] != sep)
            (*name)[name_pos++] = str[i++];
        else if(i + 1 < len && str[i + 1] == sep)
        {
            (*name)[name_pos++] = sep;
            i += 2;
        }
        else
            goto Lfound;
    }
    /* no separator */
    strcpy(*filename, str);
    strcpy(*name, str);
    return true;
    /* separator */
    Lfound:
    (*name)[name_pos] = 0;
    strcpy(*filename, str + i + 1);
    return true;
}

int main(int argc, char **argv)
{
    struct rockpack_pack_t *pack = rockpack_new();
    char sep = DEFAULT_SEP;
    
    while(1)
    {
        static struct option long_options[] =
        {
            {"help", no_argument, 0, '?'},
            {"debug", no_argument, 0, 'd'},
            {"rm", required_argument, 0, 'r'},
            {"info", no_argument, 0, 'i'},
            {"list", no_argument, 0, 'l'},
            {"target", required_argument, 0, 't'},
            {"read", required_argument, 0, 'e'},
            {"write", required_argument, 0, 'w'},
            {"add", required_argument, 0, 'a'},
            {"sep", required_argument, 0, 's'},
            {"extract", required_argument, 0, 'x'},
            {0, 0, 0, 0}
        };

        int c = getopt_long(argc, argv, "?dr:ilt:e:w:a:s:x:", long_options, NULL);
        if(c == -1)
            break;

#define check_err(...) \
    if(rockpack_error(pack)) { fprintf(stderr, __VA_ARGS__); rockpack_free(pack); return 3; }
        
        switch(c)
        {
            case -1:
                break;
            case 'd':
                rockpack_debug(&debug_printf);
                break;
            case '?':
                usage();
                break;
            case 'r':
            {
                size_t idx = rockpack_search(pack, optarg);
                check_err("Error: cannot find '%s' in the pack (%s)\n",
                    optarg, rockpack_error_msg(pack));
                rockpack_remove(pack, idx);
                check_err("Error: cannot remove '%s' from the pack (%s)\n",
                    optarg, rockpack_error_msg(pack));
                break;
            }
            case 'a':
            {
                char *name, *filename;
                if(!parse_name(sep, optarg, &name, &filename))
                {
                    rockpack_free(pack);
                    return 8;
                }
                FILE *f = fopen(filename, "rb");
                if(f == NULL)
                {
                    fprintf(stderr, "Error: cannot open input file '%s': ", filename);
                    perror("");
                    rockpack_free(pack);
                    return 9;
                }
                fseek(f, 0, SEEK_END);
                size_t size = ftell(f);
                fseek(f, 0, SEEK_SET);
                void *data = malloc(size);
                if(fread(data, size, 1, f) != 1)
                {
                    fprintf(stderr, "Error: cannot read input pack '%s': ", filename);
                    perror("");
                    free(data);
                    fclose(f);
                    return 6;
                }
                fclose(f);
                rockpack_add(pack, name, data, size);
                free(name);
                free(data);
                free(filename);
                check_err("Error: cannot add entry (%s)\n",
                    rockpack_error_msg(pack));
                break;
            }
            case 'i':
            {
                printf("Target: %s\n", rockpack_target(pack));
                printf("Nr entries: %u\n", (unsigned)rockpack_nr_entries(pack));
                break;
            }
            case 'l':
            {
                for(size_t i = 0; i < rockpack_nr_entries(pack); i++)
                {
                    printf("Entry '%s', %u bytes\n", rockpack_entry_name(pack, i),
                        (unsigned)rockpack_entry_size(pack, i));
                }
                break;
            }
            case 't':
            {
                rockpack_retarget(pack, optarg);
                check_err("Error: cannot retarget pack to '%s' (%s)\n",
                    optarg, rockpack_error_msg(pack));
                break;
            }
            case 'e':
            {
                rockpack_free(pack);
                FILE *f = fopen(optarg, "rb");
                if(f == NULL)
                {
                    fprintf(stderr, "Error: cannot open input file '%s': ", optarg);
                    perror("");
                    return 5;
                }
                fseek(f, 0, SEEK_END);
                size_t size = ftell(f);
                fseek(f, 0, SEEK_SET);
                void *data = malloc(size);
                if(fread(data, size, 1, f) != 1)
                {
                    fprintf(stderr, "Error: cannot read input pack '%s': ", optarg);
                    perror("");
                    free(data);
                    fclose(f);
                    return 6;
                }
                fclose(f);
                pack = rockpack_read(data, size);
                free(data);
                check_err("Error: cannot read input pack (%s)\n",
                    rockpack_error_msg(pack));
                break;
            }
            case 'w':
            {
                size_t size;
                void *data = rockpack_write(pack, &size);
                check_err("Error: cannot build pack output (%s)\n",
                    rockpack_error_msg(pack));
                FILE *f = fopen(optarg, "wb");
                if(f == NULL)
                {
                    free(data);
                    fprintf(stderr, "Error: cannot open output pack '%s': ", optarg);
                    perror("");
                    rockpack_free(pack);
                    return 4;
                }
                fwrite(data, size, 1, f);
                free(data);
                fclose(f);
                break;
            }
            case 's':
            {
                if(strlen(optarg) != 1)
                {
                    fprintf(stderr, "Error: invalid separator '%s' (must be a single charactor)\n",
                        optarg);
                    rockpack_free(pack);
                    return 7;
                }
                sep = optarg[0];
                break;
            }
            case 'x':
            {
                char *name, *filename;
                if(!parse_name(sep, optarg, &name, &filename))
                {
                    rockpack_free(pack);
                    return 9;
                }
                size_t idx = rockpack_search(pack, name);
                check_err("Error: cannot find '%s' in the pack (%s)\n",
                    optarg, rockpack_error_msg(pack));
                FILE *f = fopen(filename, "wb");
                if(f == NULL)
                {
                    fprintf(stderr, "Error: cannot open output file '%s': ", filename);
                    perror("");
                    rockpack_free(pack);
                    return 10;
                }
                free(name);
                free(filename);
                fwrite(rockpack_entry_data(pack, idx), 1,
                    rockpack_entry_size(pack, idx), f);
                fclose(f);
                break;
            }
            default:
                abort();
        }
    }
    rockpack_free(pack);

    if(argc == 1)
        usage();
    if(argc - optind != 0)
    {
        fprintf(stderr, "Error: extra option at the end: %s\n", argv[optind]);
        return 2;
    }
    return 0;

}
