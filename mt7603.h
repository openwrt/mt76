/*
 * Copyright (C) 2015 Felix Fietkau <nbd@openwrt.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __MT7603_H
#define __MT7603_H

#include <linux/device.h>
#include "mt76.h"
#include "mt7603_regs.h"

#define MT7603_FIRMWARE_E1	"mt7603_e1.bin"
#define MT7603_FIRMWARE_E2	"mt7603_e2.bin"

struct mt7603_dev {
	struct mt76_dev mt76;
	struct device *dev;

	u32 rev;
};

extern const struct ieee80211_ops mt7603_ops;
struct mt7603_dev *mt7603_alloc_device(struct device *pdev);
int mt7603_mcu_init(struct mt7603_dev *dev);

#endif
