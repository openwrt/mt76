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
#include <linux/of.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include "mt76.h"

static int
mt76_get_of_eeprom(struct mt76_dev *dev, int len)
{
#ifdef CONFIG_OF
	struct device_node *np = dev->dev->of_node;
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

	return 0;
#else
	return -ENOENT;
#endif
}

#ifdef CONFIG_OF
void
mt76_eeprom_override(struct mt76_dev *dev)
{
	struct device_node *np = dev->dev->of_node;
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
EXPORT_SYMBOL_GPL(mt76_eeprom_override);
#endif

int
mt76_eeprom_init(struct mt76_dev *dev, int len)
{
	dev->eeprom.size = len;
	dev->eeprom.data = devm_kzalloc(dev->dev, len, GFP_KERNEL);
	if (!dev->eeprom.data)
		return -ENOMEM;

	return !mt76_get_of_eeprom(dev, len);
}
EXPORT_SYMBOL_GPL(mt76_eeprom_init);
