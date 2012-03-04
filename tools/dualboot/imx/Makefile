BIN2C=../bin2c
# Edit the following variables (plus copy/paste another set of rules) when
# adding a new target.  mkimxboot.c also needs to be edited to refer to these
# new images.

BOOTOBJS = dualboot_fuzeplus.o
BOOTBINS = $(BOOTOBJS:%.o=%.bin)
DUALBOOT = dualboot.h dualboot.c

all: $(DUALBOOT)

# Dualboot bootloaders

dualboot_fuzeplus.o: dualboot.S
	$(CC) -mcpu=arm926ej-s -DSANSA_FUZEPLUS -c -o dualboot_fuzeplus.o dualboot.S

# Rules for the ARM code embedded in mkamsboot - assemble, link, then extract
# the binary code and finally convert to .h for building in mkamsboot

%.elf: %.o
	$(LD) -Tdualboot.lds -o $@ $<

%.bin: %.elf
	$(OC) -O binary $< $@

$(DUALBOOT): $(BOOTBINS)
	$(BIN2C) dualboot $(BOOTBINS)

clean:
	rm -f *~ $(BOOTBINS) $(BOOTOBJS) $(DUALBOOT)
