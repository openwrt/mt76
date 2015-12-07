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

struct mt7603_dev *mt7603_alloc_device(struct device *pdev)
{
	struct ieee80211_hw *hw;
	struct mt7603_dev *dev;

	hw = ieee80211_alloc_hw(sizeof(*dev), &mt7603_ops);
	if (!hw)
		return NULL;

	dev = hw->priv;
	dev->mt76.dev = pdev;
	dev->mt76.hw = hw;

	return dev;
}

static bool
wait_for_wpdma(struct mt7603_dev *dev)
{
	return mt76_poll(dev, MT_WPDMA_GLO_CFG,
			 MT_WPDMA_GLO_CFG_TX_DMA_BUSY |
			 MT_WPDMA_GLO_CFG_RX_DMA_BUSY,
			 0, 1000);
}

static int
mt7603_mac_reset(struct mt7603_dev *dev)
{
	u32 val;

	mt76_wr(dev, MT_WPDMA_GLO_CFG, 0x52000850);

	mt76_wr(dev, MT_DMA_DCR0, 0x1000);
	mt76_wr(dev, MT_DMA_DCR1, GENMASK(13, 11));

	val = mt76_rr(dev, MT_DMA_VCFR0);
	val &= ~BIT(0); /* To HIF */
	val |= BIT(13); /* Rx Ring 1 */
	mt76_wr(dev, MT_DMA_VCFR0, val);

	return 0;
}

int mt7603_mac_start(struct mt7603_dev *dev)
{
	wait_for_wpdma(dev);
	udelay(50);

	mt76_set(dev, MT_WPDMA_GLO_CFG,
		 MT_WPDMA_GLO_CFG_TX_DMA_EN |
		 MT_WPDMA_GLO_CFG_RX_DMA_EN);

	mt76_clear(dev, MT_WPDMA_GLO_CFG, MT_WPDMA_GLO_CFG_TX_WRITEBACK_DONE);

	mt7603_irq_enable(dev, MT_INT_RX_DONE_ALL | MT_INT_TX_DONE_ALL);
	return 0;
}

int mt7603_init_hardware(struct mt7603_dev *dev)
{
	int ret;

	ret = mt7603_mac_reset(dev);
	if (ret)
		return ret;

	ret = mt7603_dma_init(dev);
	if (ret)
		return ret;

	set_bit(MT76_STATE_INITIALIZED, &dev->mt76.state);

	ret = mt7603_mac_start(dev);
	if (ret)
		return ret;

	ret = mt7603_mcu_init(dev);
	if (ret)
		return ret;

	return 0;
}
