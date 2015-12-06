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

#ifndef __MT76_H
#define __MT76_H

#include <linux/kernel.h>
#include <linux/io.h>
#include <net/mac80211.h>
#include "util.h"

struct mt76_dev;

struct mt76_bus_ops {
	u32 (*rr)(struct mt76_dev *dev, u32 offset);
	void (*wr)(struct mt76_dev *dev, u32 offset, u32 val);
	u32 (*rmw)(struct mt76_dev *dev, u32 offset, u32 mask, u32 val);
	void (*copy)(struct mt76_dev *dev, u32 offset, const void *data, int len);
};

struct mt76_dev {
	const struct mt76_bus_ops *bus;
	void __iomem *regs;

	struct ieee80211_hw *hw;
};

#define mt76_rr(dev, ...)	(dev)->mt76.bus->rr(&((dev)->mt76), __VA_ARGS__)
#define mt76_wr(dev, ...)	(dev)->mt76.bus->wr(&((dev)->mt76), __VA_ARGS__)
#define mt76_rmw(dev, ...)	(dev)->mt76.bus->rmw(&((dev)->mt76), __VA_ARGS__)
#define mt76_wr_copy(dev, ...)	(dev)->mt76.bus->copy(&((dev)->mt76), __VA_ARGS__)

#define mt76_hw(dev) (dev)->mt76.hw

void mt76_mmio_init(struct mt76_dev *dev, void __iomem *regs);

#endif
