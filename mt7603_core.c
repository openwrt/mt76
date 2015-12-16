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

#include "mt7603.h"
#include "mt7603_eeprom.h"

void mt7603_set_irq_mask(struct mt7603_dev *dev, u32 clear, u32 set)
{
	unsigned long flags;

	spin_lock_irqsave(&dev->irq_lock, flags);
	dev->irqmask &= ~clear;
	dev->irqmask |= set;
	mt76_wr(dev, MT_INT_MASK_CSR, dev->irqmask);
	spin_unlock_irqrestore(&dev->irq_lock, flags);
}

irqreturn_t mt7603_irq_handler(int irq, void *dev_instance)
{
	struct mt7603_dev *dev = dev_instance;
	u32 intr;

	intr = mt76_rr(dev, MT_INT_SOURCE_CSR);
	mt76_wr(dev, MT_INT_SOURCE_CSR, intr);

	if (!test_bit(MT76_STATE_INITIALIZED, &dev->mt76.state))
		return IRQ_NONE;

	intr &= dev->irqmask;

	if (intr & MT_INT_TX_DONE_ALL) {
		mt7603_irq_disable(dev, MT_INT_TX_DONE_ALL);
		tasklet_schedule(&dev->tx_tasklet);
	}

	if (intr & MT_INT_RX_DONE(0)) {
		mt7603_irq_disable(dev, MT_INT_RX_DONE(0));
		napi_schedule(&dev->napi);
	}

	if (intr & MT_INT_RX_DONE(1)) {
		mt7603_irq_disable(dev, MT_INT_RX_DONE(1));
		tasklet_schedule(&dev->rx_tasklet);
	}

#if 0
	if (intr & MT_INT_PRE_TBTT)
		tasklet_schedule(&dev->pre_tbtt_tasklet);

	/* send buffered multicast frames now */
	if (intr & MT_INT_TBTT)
		mt76_queue_kick(dev, &dev->q_tx[MT_TXQ_PSD]);
#endif

	return IRQ_HANDLED;
}

u32 mt7603_reg_map(struct mt7603_dev *dev, u32 addr)
{
	u32 base = addr & GENMASK(31, 19);
	u32 offset = addr & GENMASK(18, 0);

	mt76_wr(dev, MT_MCU_PCIE_REMAP_2, base);

	return MT_PCIE_REMAP_BASE_2 + offset;
}

int mt7603_set_channel(struct mt7603_dev *dev, struct cfg80211_chan_def *def)
{
	u8 *rssi_data = (u8 *) dev->mt76.eeprom.data;
	int idx, ret;

	u8 bw = MT_BW_20;

	mt7603_mac_stop(dev);

	dev->chandef = *def;
	mt76_rmw_field(dev, MT_AGG_BWCR, MT_AGG_BWCR_BW, bw);
	ret = mt7603_mcu_set_channel(dev);
	if (ret)
		return ret;

	if (def->chan->band == IEEE80211_BAND_5GHZ) {
		idx = 1;
		rssi_data += MT_EE_RSSI_OFFSET_5G;
	} else {
		idx = 0;
		rssi_data += MT_EE_RSSI_OFFSET_2G;
	}

	memcpy(dev->rssi_offset, rssi_data, sizeof(dev->rssi_offset));

	idx |= (def->chan - mt76_hw(dev)->wiphy->bands[def->chan->band]->channels) << 1;
	mt76_set(dev, MT_WF_RMAC_CH_FREQ, idx);
	mt7603_mac_start(dev);

	return 0;
}
