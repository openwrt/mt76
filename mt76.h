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
	struct device *dev;

	struct ieee80211_hw *hw;
};

#define mt76_rr(dev, ...)	(dev)->mt76.bus->rr(&((dev)->mt76), __VA_ARGS__)
#define mt76_wr(dev, ...)	(dev)->mt76.bus->wr(&((dev)->mt76), __VA_ARGS__)
#define mt76_rmw(dev, ...)	(dev)->mt76.bus->rmw(&((dev)->mt76), __VA_ARGS__)
#define mt76_wr_copy(dev, ...)	(dev)->mt76.bus->copy(&((dev)->mt76), __VA_ARGS__)

#define mt76_set(dev, offset, val)	mt76_rmw(dev, offset, 0, val)
#define mt76_clear(dev, offset, val)	mt76_rmw(dev, offset, val, 0)

#define mt76_get_field(_dev, _reg, _field)		\
	MT76_GET(_field, mt76_rr(dev, _reg))

#define mt76_rmw_field(_dev, _reg, _field, _val)	\
	mt76_rmw(_dev, _reg, _field, MT76_SET(_field, _val))

#define mt76_hw(dev) (dev)->mt76.hw

bool __mt76_poll(struct mt76_dev *dev, u32 offset, u32 mask, u32 val,
	         int timeout);

#define mt76_poll(dev, ...) __mt76_poll(&((dev)->mt76), __VA_ARGS__)

bool __mt76_poll_msec(struct mt76_dev *dev, u32 offset, u32 mask, u32 val,
		      int timeout);

#define mt76_poll_msec(dev, ...) __mt76_poll_msec(&((dev)->mt76), __VA_ARGS__)

void mt76_mmio_init(struct mt76_dev *dev, void __iomem *regs);

#endif
