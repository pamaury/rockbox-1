SUBDIRS=imx ams
CLEAN_SUBDIRS=$(SUBDIRS:%=clean-%)
DUALBOOT=dualboot.c dualboot.h

ifndef HOSTCC
export HOSTCC:=$(CC)
CROSS_PREFIX?=arm-elf-eabi
export CC=$(CROSS_PREFIX)-gcc
export LD=$(CROSS_PREFIX)-ld
export OC=$(CROSS_PREFIX)-objcopy
endif

.PHONY: all $(SUBDIRS) $(CLEAN_SUBDIRS) clean

all: $(DUALBOOT)

$(SUBDIRS): bin2c
	$(MAKE) -C $@

$(CLEAN_SUBDIRS):
	$(MAKE) -C $(@:clean-%=%) clean

dualboot.c: $(SUBDIRS)
	cat $(^:%=%/dualboot.c) > $@

dualboot.h: $(SUBDIRS)
	cat $(^:%=%/dualboot.h) > $@

bin2c: bin2c.c
	$(HOSTCC) -O2 -Wall -o $@ $^
	

clean: $(CLEAN_SUBDIRS)
	rm -f $(DUALBOOT) $(DUALBOOTSH) $(DUALBOOTSC) bin2c