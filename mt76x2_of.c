/*
 * Copyright (C) 2014 Felix Fietkau <nbd@openwrt.org>
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

#include <linux/of.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include "mt76x2.h"
#include "mt76x2_eeprom.h"
#include "mt76x2_of.h"

static int mt76x2_check_eeprom(struct mt76x2_dev *dev, const char *type)
{
	u16 val = get_unaligned_le16(dev->eeprom.data);
	switch (val) {
	case 0x7662:
	case 0x7612:
		return 0;
	default:
		printk("%s EEPROM data check failed: %04x\n", type, val);
		return -EINVAL;
	}
}

int
mt76x2_get_of_eeprom(struct mt76x2_dev *dev, int len)
{
	struct device_node *np = dev->mt76.dev->of_node;
	struct mtd_info *mtd;
	const __be32 *list;
	const char *part;
	phandle phandle;
	int offset = 0;
	int size;
	size_t retlen;
	int ret;

	if (!np)
		return -ENOENT;

	list = of_get_property(np, "mediatek,mtd-eeprom", &size);
	if (!list)
		return -ENOENT;

	phandle = be32_to_cpup(list++);
	if (!phandle)
		return -ENOENT;

	np = of_find_node_by_phandle(phandle);
	if (!np)
		return -EINVAL;

	part = of_get_property(np, "label", NULL);
	if (!part)
		part = np->name;

	mtd = get_mtd_device_nm(part);
	if (IS_ERR(mtd))
		return PTR_ERR(mtd);

	if (size <= sizeof(*list))
		return -EINVAL;

	offset = be32_to_cpup(list);
	ret = mtd_read(mtd, offset, len, &retlen, dev->eeprom.data);
	put_mtd_device(mtd);
	if (ret)
		return ret;

	if (retlen < len)
		return -EINVAL;

	return mt76x2_check_eeprom(dev, "Flash");
}

void
mt76x2_get_of_overrides(struct mt76x2_dev *dev)
{
	struct device_node *np = dev->mt76.dev->of_node;
	const __be32 *val;
	int size;

	if (!np)
		return;

	val = of_get_property(np, "mediatek,2ghz", &size);
	if (val)
		dev->cap.has_2ghz = be32_to_cpup(val);

	val = of_get_property(np, "mediatek,5ghz", &size);
	if (val)
		dev->cap.has_5ghz = be32_to_cpup(val);
}
