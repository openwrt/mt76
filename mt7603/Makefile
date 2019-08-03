EXTRA_CFLAGS += -Werror -DCONFIG_MT76_LEDS

CHECK_CC_ENUM_FLAGS = $(NOSTDINC_FLAGS) $(LINUXINCLUDE) $(KBUILD_CPPFLAGS) $(KBUILD_CFLAGS)
CHECK_CC_ENUM_FLAGS += -DKBUILD_MODNAME='"mt7603e"'
check-cc-enum = $(call try-run, echo "int v = $(1);" | $(CC) -Werror $(CHECK_CC_ENUM_FLAGS) $(2) -c -x c - -o "$$TMP",y,n)

# backwards compatibility hack
ifeq ($(call check-cc-enum,IEEE80211_HW_TX_STATUS_NO_AMPDU_LEN,-include net/mac80211.h),n)
  EXTRA_CFLAGS += -DIEEE80211_HW_TX_STATUS_NO_AMPDU_LEN=IEEE80211_HW_REPORTS_TX_ACK_STATUS
endif

obj-m := mt7603e.o

mt7603e-y := \
	pci.o soc.o main.o init.o mcu.o \
	core.o dma.o mac.o eeprom.o \
	beacon.o debugfs.o
