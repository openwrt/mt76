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
#include "mt76.h"

static int
mt76_reg_set(void *data, u64 val)
{
	struct mt76_dev *dev = data;

	dev->bus->wr(dev, dev->debugfs_reg, val);
	return 0;
}

static int
mt76_reg_get(void *data, u64 *val)
{
	struct mt76_dev *dev = data;

	*val = dev->bus->rr(dev, dev->debugfs_reg);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(fops_regval, mt76_reg_get, mt76_reg_set, "0x%08llx\n");


struct dentry *mt76_register_debugfs(struct mt76_dev *dev)
{
	struct dentry *dir;

	dir = debugfs_create_dir("mt76", dev->hw->wiphy->debugfsdir);
	if (!dir)
		return NULL;

	debugfs_create_u32("regidx", S_IRUSR | S_IWUSR, dir, &dev->debugfs_reg);
	debugfs_create_file("regval", S_IRUSR | S_IWUSR, dir, dev, &fops_regval);
	debugfs_create_blob("eeprom", S_IRUSR, dir, &dev->eeprom);

	return dir;
}
EXPORT_SYMBOL_GPL(mt76_register_debugfs);
