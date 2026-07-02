# SPDX-License-Identifier: BSD-3-Clause-Clear
EXTRA_CFLAGS += -Werror -DCONFIG_MT76_LEDS
obj-m += mt7603e.o

mt7603e-y := \
	soc.o main.o init.o mcu.o \
	core.o dma.o mac.o eeprom.o \
	beacon.o debugfs.o

mt7603e-$(CONFIG_PCI) += pci.o
