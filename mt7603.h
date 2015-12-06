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
	u32 rev;

	u32 irqmask;
	spinlock_t irq_lock;
};

extern const struct ieee80211_ops mt7603_ops;
struct mt7603_dev *mt7603_alloc_device(struct device *pdev);
irqreturn_t mt7603_irq_handler(int irq, void *dev_instance);
int mt7603_mcu_init(struct mt7603_dev *dev);

void mt7603_set_irq_mask(struct mt7603_dev *dev, u32 clear, u32 set);

static inline void mt7603_irq_enable(struct mt7603_dev *dev, u32 mask)
{
	mt7603_set_irq_mask(dev, 0, mask);
}

static inline void mt7603_irq_disable(struct mt7603_dev *dev, u32 mask)
{
	mt7603_set_irq_mask(dev, mask, 0);
}

#endif
