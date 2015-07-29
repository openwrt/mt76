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

#ifndef __MT76_OF_H
#define __MT76_OF_H

#include "mt76.h"

#ifdef CONFIG_OF

int mt76_get_of_eeprom(struct mt76_dev *dev, int len);
void mt76_get_of_overrides(struct mt76_dev *dev);

#else

static inline int
mt76_get_of_eeprom(struct mt76_dev *dev, int len)
{
	return -ENOENT;
}

static inline void
mt76_get_of_overrides(struct mt76_dev *dev)
{
}

#endif /* CONFIG_OF */

#endif
