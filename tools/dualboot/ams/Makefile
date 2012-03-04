BIN2C=../bin2c
# Edit the following variables (plus copy/paste another set of rules) when
# adding a new target.  mkamsboot.c also needs to be edited to refer to these
# new images.

BOOTOBJS = nrv2e_d8.o dualboot_clip.o dualboot_e200v2.o dualboot_c200v2.o dualboot_m200v4.o dualboot_fuze.o dualboot_clipv2.o dualboot_clipplus.o dualboot_fuzev2.o dualboot_clipzip.o
BOOTBINS = $(BOOTOBJS:%.o=%.bin)
DUALBOOT = dualboot.h dualboot.c

all: $(DUALBOOT)

# Dualboot bootloaders

dualboot_clip.o: dualboot.S
	$(CC) -DSANSA_CLIP -c -o dualboot_clip.o dualboot.S

dualboot_fuze.o: dualboot.S
	$(CC) -DSANSA_FUZE -c -o dualboot_fuze.o dualboot.S

dualboot_e200v2.o: dualboot.S
	$(CC) -DSANSA_E200V2 -c -o dualboot_e200v2.o dualboot.S

dualboot_m200v4.o: dualboot.S
	$(CC) -DSANSA_M200V4 -c -o dualboot_m200v4.o dualboot.S

dualboot_c200v2.o: dualboot.S
	$(CC) -DSANSA_C200V2 -c -o dualboot_c200v2.o dualboot.S

dualboot_clipv2.o: dualboot.S
	$(CC) -DSANSA_CLIPV2 -c -o dualboot_clipv2.o dualboot.S

dualboot_clipplus.o: dualboot.S
	$(CC) -DSANSA_CLIPPLUS -c -o dualboot_clipplus.o dualboot.S

dualboot_fuzev2.o: dualboot.S
	$(CC) -DSANSA_FUZEV2 -c -o dualboot_fuzev2.o dualboot.S

dualboot_clipzip.o: dualboot.S
	$(CC) -DSANSA_CLIPZIP -c -o dualboot_clipzip.o dualboot.S

# Rules for the ucl unpack function
nrv2e_d8.o: nrv2e_d8.S
	$(CC) -DPURE_THUMB -c -o nrv2e_d8.o nrv2e_d8.S

# Rules for the ARM code embedded in mkamsboot - assemble, link, then extract
# the binary code and finally convert to .h for building in mkamsboot

%.elf: %.o
	$(LD) -e 0 -Ttext=0 -o $@ $<

%.bin: %.elf
	$(OC) -O binary $< $@


$(DUALBOOT): $(BOOTBINS)
	$(BIN2C) dualboot $(BOOTBINS)

clean:
	rm -f *~ $(BOOTBINS) $(BOOTOBJS) $(DUALBOOT)
