
# export PATH:=/usr/local/avr/bin:$(PATH)
# export LIBRARY_PATH:=/usr/local/avr/lib

CPU:=atmega32u4

CFLAGS:=-mmcu=$(CPU) -mtiny-stack -O3 -Os -DCPU=$(CPU) -DF_CPU=16 \
         -Wall -Wno-uninitialized -Werror \
         -foptimize-sibling-calls -fpeephole -fpeephole2  \
         --param max-unrolled-insns=3 --param max-unroll-times=3  \
         --param inline-unit-growth=1 -fno-tree-scev-cprop -fsplit-wide-types

# http://www.tty1.net/blog/2008/avr-gcc-optimisations_en.html
LDFLAGS:=$(CFLAGS) -Wl,--relax

ifneq "$(LIST)" ""
   _LIST=-g -Wa,-ahlmns
endif

V:=0
ifeq "$(V)" "0"
   .SILENT:
   DASHQ:=-q
else
   DASHV:=-v
endif

all:            main.hex main.avr

# --------------------------------------------------------------------------

o:
		mkdir -p o

o/%.o:%.S | o#
		@echo gcc $^
		avr-gcc -c $(CFLAGS) -o $@ $^ $(_LIST)

o/%.o:%.c | o
		@echo gcc $^
		avr-gcc -c $(CFLAGS) -std=c99 -o $@ $^ $(_LIST)

o/%.o:%.cc | o
		@echo g++ $^
		avr-g++ -c $(CFLAGS) -o $@ $^

#%.avr:%.o
#		avr-gcc -L/usr/local/avr/lib $(CFLAGS) -o $@ $^ -v
#		@chmod -x $@

# Convert to hex with objcopy. We can NOT use ld --oformat here.
%.hex:%.avr
		@echo objcopy $@
		avr-objcopy -j .text -j .data -O ihex $^ $@
		chmod -x $@

# --------------------------------------------------------------------------

# need something to pick up latest tag
# echo '"'$$(hg tags | sort | awk '/^v[0-9\.]+/||1 { V=$$1 } END { if(V!="") print " " V}')'"'

version.h:	$(wildcard *.c *.cc)
		ID=$$(hg id -i) ; \
		TAG=$$(hg tags | sort | awk '/^v[0-9\.]+/ { V=$$1 } END { if(V!="") print " " V}') ; \
		REV=$$(hg id -r $${TAG:-0}) ; \
		if [ "$$ID" != "$$REV" ] ; then \
		   [ -n "$$TAG" ] && echo '"'$$TAG' "' ; \
		   echo '"'$$ID'"' ; \
		fi  >$@

random.h:	$(wildcard *.c *.cc)
		echo "#define STR_SERIAL_NUMBER L\"$$(od -A n -t u -N 4 /dev/urandom | tr -d ' ')\"" >$@

o/usb_serial.o: usb_serial.c random.h | o
		@echo gcc $^
		avr-gcc -c $(CFLAGS) -o $@ $<

o/commands.o:   commands.cc version.h | o
		@echo g++ $^
		avr-g++ -c $(CFLAGS) -o $@ $<

main.avr:       o/main.o o/formatting.o o/commands.o o/gpib.o o/serial.o o/usb_serial.o o/timer.o
		@echo ld $^
		avr-g++ -L/usr/local/avr/lib $(LDFLAGS) -o $@ $^ $(DASHV)
		chmod -x $@
		avr-size $@

CLEAN+=main.avr version.h random.h

# --------------------------------------------------------------------------

.PHONY: disas fuses program term setspeed clean rebuild
disas:          main.avr
		avr-objdump -r -z -d $^ | avr-c++filt

LFUSE=0x62      # clkout=of sut=2 clksel=2 (rc)
LFUSE=0xb2      # clkout=on sut=3 clksel=2 (rc)
#HFUSE=0xc4      # extreset=off, debugwire=off, spi=off, wdt=on, eesave=on, bod=4=4.5V
# NEVER program HFUSE bit 7=0 or bit 5=1, because SPI can no longer be used
# HFUSE values should be 0b1X0XXXXX, or 0x8X, 0x9X, 0xCX, 0xDX
HFUSE=0xdf      # extreset=off, debugwire=off, spi=off, wdt=off, eesave=on, bod=7
EFUSE=0xff      # self-programming=on

fuses:
		false Dangerous
		avrdude -p t$(CPU) -u -U efuse:w:$$(($(EFUSE))):m -U hfuse:w:$$(($(HFUSE))):m -U lfuse:w:$$(($(LFUSE))):m

program:        main.hex
		./program $(DASHV) --program $(CPU) $^
		rm -f random.h

term:
		avrdude -p t$(CPU) -v -E noreset -t

setspeed:
		avrdude -p t$(CPU) -v -E noreset -B300

clean:
		-rm -r o
		-rm $(CLEAN)

rebuild:
		$(MAKE) clean ; $(MAKE)

# ----- EOF Makefile -----
