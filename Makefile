EXTRA_CFLAGS += -Werror

obj-m := mt76.o mt76x2e.o mt7603e.o

mt76-y := \
	mmio.o util.o trace.o dma.o

mt76x2e-y := \
	mt76x2_pci.o mt76x2_dma.o \
	mt76x2_main.o mt76x2_init.o mt76x2_debugfs.o mt76x2_tx.o \
	mt76x2_core.o mt76x2_mac.o mt76x2_eeprom.o mt76x2_mcu.o mt76x2_phy.o \
	mt76x2_trace.o

mt76x2e-$(CONFIG_OF) += mt76x2_of.o

mt7603e-y := \
	mt7603_pci.o mt7603_main.o mt7603_init.o mt7603_mcu.o
