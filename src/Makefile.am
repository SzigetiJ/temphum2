include $(top_srcdir)/scripts/elf2bin.mk
AM_LDFLAGS = @E32B_LDFLAGS@ -Wl,--gc-sections \
 -T @E32BDIR@/share/esp32.ld \
 -T @E32BDIR@/share/esp32peripheral.ld \
 -T @E32BDIR@/share/esp32.rom.newlib-funcs.ld 

noinst_HEADERS = defines.h i16utils.h

AM_CFLAGS  = -std=c11 -flto

if WITH_BINARIES
AM_LDFLAGS += \
 -T @E32BDIR@/share/esp32.rom.ld \
 -T @E32BDIR@/share/esp32.rom.libgcc.ld \
 -T @E32BDIR@/share/esp32.rom.newlib-data.ld \
 -T @E32BDIR@/share/esp32.rom.newlib-locale.ld \
 -T @E32BDIR@/share/esp32.rom.newlib-nano.ld \
 -T @E32BDIR@/share/esp32.rom.newlib-time.ld \
 -T @E32BDIR@/share/esp32.rom.redefined.ld \
 -T @E32BDIR@/share/esp32.rom.syscalls.ld
else
AM_LDFLAGS += \
 -T @E32BDIR@/share/esp32.rom.ld \
 -T @E32BDIR@/share/esp32.rom.libgcc.ld \
 -T @E32BDIR@/share/esp32.rom.newlib-nano.ld \
 -T @E32BDIR@/share/esp32.rom.redefined.ld \
 -T @E32BDIR@/share/esp32.rom.syscalls.ld
endif

AM_CPPFLAGS = @E32B_CPPFLAGS@
LDADD = -lesp32basic -lesp32modules

bin_PROGRAMS = \
 temphum.elf

temphum_elf_SOURCES = temphum.c i16utils.c


if WITH_BINARIES
CLEANFILES = \
 temphum.bin
endif

BUILT_SOURCES = $(CLEANFILES)
