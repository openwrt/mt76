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

#include <linux/module.h>
#include "mt76.h"

void mt76_remove_hdr_pad(struct sk_buff *skb)
{
	int len = ieee80211_get_hdrlen_from_skb(skb);
	memmove(skb->data + 2, skb->data, len);
	skb_pull(skb, 2);
}
EXPORT_SYMBOL_GPL(mt76_remove_hdr_pad);

int mt76_insert_hdr_pad(struct sk_buff *skb)
{
	int len = ieee80211_get_hdrlen_from_skb(skb);
	int ret;

	if (len % 4 == 0)
		return 0;

	if (skb_headroom(skb) < 2 &&
	    (ret = pskb_expand_head(skb, 2, 0, GFP_ATOMIC)) != 0)
		return ret;

	skb_push(skb, 2);
	memmove(skb->data, skb->data + 2, len);

	skb->data[len] = 0;
	skb->data[len + 1] = 0;
	return 2;
}
EXPORT_SYMBOL_GPL(mt76_insert_hdr_pad);

bool __mt76_poll(struct mt76_dev *dev, u32 offset, u32 mask, u32 val,
		 int timeout)
{
	u32 cur;

	timeout /= 10;
	do {
		cur = dev->bus->rr(dev, offset) & mask;
		if (cur == val)
			return true;

		udelay(10);
	} while (timeout-- > 0);

	return false;
}
EXPORT_SYMBOL_GPL(__mt76_poll);

bool __mt76_poll_msec(struct mt76_dev *dev, u32 offset, u32 mask, u32 val,
		      int timeout)
{
	u32 cur;

	timeout /= 10;
	do {
		cur = dev->bus->rr(dev, offset) & mask;
		if (cur == val)
			return true;

		msleep(10);
	} while (timeout-- > 0);

	return false;
}
EXPORT_SYMBOL_GPL(__mt76_poll_msec);

int mt76_wcid_alloc(unsigned long *mask, int size)
{
	int i, idx = 0, cur;

	for (i = 0; i < size / BITS_PER_LONG; i++) {
		idx = ffs(~mask[i]);
		if (!idx)
			continue;

		idx--;
		cur = i * BITS_PER_LONG + idx;
		if (cur >= size)
			break;

		mask[i] |= BIT(idx);
		return cur;
	}

	return -1;
}
EXPORT_SYMBOL_GPL(mt76_wcid_alloc);

MODULE_LICENSE("GPL");
