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

#include <linux/delay.h>
#include "mt76.h"
#include "trace.h"

bool mt76_poll(struct mt76_dev *dev, u32 offset, u32 mask, u32 val,
			   int timeout)
{
	u32 cur;

	timeout /= 10;
	do {
		cur = mt76_rr(dev, offset) & mask;
		if (cur == val)
			return true;

		udelay(10);
	} while (timeout-- > 0);

	return false;
}

bool mt76_poll_msec(struct mt76_dev *dev, u32 offset, u32 mask, u32 val,
				    int timeout)
{
	u32 cur;

	timeout /= 10;
	do {
		cur = mt76_rr(dev, offset) & mask;
		if (cur == val)
			return true;

		msleep(10);
	} while (timeout-- > 0);

	return false;
}

void mt76_write_reg_pairs(struct mt76_dev *dev,
			  const struct mt76_reg_pair *data, int len)
{
	while (len > 0) {
		mt76_wr(dev, data->reg, data->value);
		len--;
		data++;
	}
}

void mt76_set_irq_mask(struct mt76_dev *dev, u32 clear, u32 set)
{
	unsigned long flags;

	spin_lock_irqsave(&dev->irq_lock, flags);
	dev->irqmask &= ~clear;
	dev->irqmask |= set;
	mt76_wr(dev, MT_INT_MASK_CSR, dev->irqmask);
	spin_unlock_irqrestore(&dev->irq_lock, flags);
}

irqreturn_t mt76_irq_handler(int irq, void *dev_instance)
{
	struct mt76_dev *dev = dev_instance;
	u32 intr;

	intr = mt76_rr(dev, MT_INT_SOURCE_CSR);
	mt76_wr(dev, MT_INT_SOURCE_CSR, intr);

	if (!test_bit(MT76_STATE_INITAILIZED, &dev->state))
		return IRQ_NONE;

	trace_dev_irq(dev, intr, dev->irqmask);

	intr &= dev->irqmask;

	if (intr & MT_INT_TX_DONE_ALL) {
		mt76_irq_disable(dev, MT_INT_TX_DONE_ALL);
		tasklet_schedule(&dev->tx_tasklet);
	}

	if (intr & MT_INT_RX_DONE(0)) {
		mt76_irq_disable(dev, MT_INT_RX_DONE(0));
		napi_schedule(&dev->napi);
	}

	if (intr & MT_INT_RX_DONE(1)) {
		mt76_irq_disable(dev, MT_INT_RX_DONE(1));
		tasklet_schedule(&dev->rx_tasklet);
	}

	if (intr & MT_INT_PRE_TBTT)
		tasklet_schedule(&dev->pre_tbtt_tasklet);

	/* send buffered multicast frames now */
	if (intr & MT_INT_TBTT)
		mt76_kick_queue(dev, &dev->q_tx[MT_TXQ_PSD]);

	if (intr & MT_INT_TX_STAT) {
		mt76_mac_poll_tx_status(dev, true);
		tasklet_schedule(&dev->tx_tasklet);
	}

	return IRQ_HANDLED;
}

int mt76_set_channel(struct mt76_dev *dev, struct cfg80211_chan_def *chandef)
{
	int ret;

	tasklet_disable(&dev->pre_tbtt_tasklet);
	cancel_delayed_work_sync(&dev->cal_work);

	mt76_mac_stop(dev, true);
	ret = mt76_phy_set_channel(dev, chandef);
	mt76_mac_resume(dev);
	tasklet_enable(&dev->pre_tbtt_tasklet);

	return ret;
}

