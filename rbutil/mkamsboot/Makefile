#             __________               __   ___.
#   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
#   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
#   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
#   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
#                     \/            \/     \/    \/            \/

# We use the UCL code available in the Rockbox tools/ directory
TOOLS_DIR=../../tools/
CFLAGS += -I$(TOOLS_DIR) -I$(TOOLS_DIR)/ucl/include -Wall
CFLAGS += -std=c99

OUTPUT = mkamsboot
LIBUCL = libucl$(RBARCH).a

# inputs
TOOLS_SOURCES = rockpack.c
LIBSOURCES := md5.c mkamsboot.c \
		$(addprefix $(TOOLS_DIR),$(TOOLS_SOURCES))
SOURCES := $(LIBSOURCES) main.c
# additional link dependencies for the standalone executable
EXTRADEPS := $(LIBUCL)

include ../libtools.make
# explicit dependencies on dualboot.{c,h} and mkamsboot.h
$(OBJDIR)mkamsboot.o: mkamsboot.c mkamsboot.h
$(OBJDIR)main.o: main.c mkamsboot.h


$(TARGET_DIR)$(OUTPUT).dll: EXTRALIBOBJS += $(TARGET_DIR)ucl.dll
$(TARGET_DIR)$(OUTPUT).dll: $(TARGET_DIR)ucl.dll
$(TARGET_DIR)ucl.dll:
	$(SILENT)$(MAKE) -C ../../tools/ucl/src/ \
	    BUILD_DIR=$(BUILD_DIR) CC=$(CC) CROSS=$(CROSS) \
	    TARGET_DIR=$(TARGET_DIR)/ ucl.dll

$(OBJDIR)%.o: $(TOOLS_DIR)%.c
	@echo CC $<
	$(SILENT)mkdir -p $(dir $@)
	$(SILENT)$(CROSS)$(CC) $(CFLAGS) -c -o $@ $<