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
	mt76_wr(dev, MT_WPDMA_GLO_CFG, 0x52000850);

	/* Disable MAC */
	mt76_set(dev, MT_WF_ARB_SCR, MT_WF_ARB_TX_DISABLE | MT_WF_ARB_RX_DISABLE);
	mt76_wr(dev, MT_WF_ARB_TQCR0, 0);
	mt76_clear(dev, MT_WF_ARB_RQCR, MT_WF_ARB_RQCR_RX_START);

	mt76_rmw(dev, MT_DMA_DCR0, 0xffff, 0xc0211000);
	mt76_wr(dev, MT_DMA_DCR1, GENMASK(13, 11));

	mt76_rmw(dev, MT_DMA_VCFR0, BIT(0), BIT(13));
	mt76_rmw(dev, MT_DMA_TMCFR0, BIT(0) | BIT(1), BIT(13));

	mt76_clear(dev, MT_WF_RMAC_TMR_PA, BIT(31));

	mt76_set(dev, MT_WF_RMAC_RMACDR, BIT(30));
	mt76_rmw(dev, MT_WF_RMAC_MAXMINLEN, 0xffffff, 0x19000);

	mt76_wr(dev, MT_WF_RFCR1, 0);

	return 0;
}

int mt7603_mac_start(struct mt7603_dev *dev)
{
	wait_for_wpdma(dev);
	udelay(50);

	mt76_clear(dev, MT_WF_ARB_SCR, MT_WF_ARB_TX_DISABLE | MT_WF_ARB_RX_DISABLE);
	mt76_wr(dev, MT_WF_ARB_TQCR0, ~0);
	mt76_set(dev, MT_WF_ARB_RQCR, MT_WF_ARB_RQCR_RX_START);

	mt76_set(dev, MT_WPDMA_GLO_CFG,
		 MT_WPDMA_GLO_CFG_TX_DMA_EN |
		 MT_WPDMA_GLO_CFG_RX_DMA_EN);

	mt7603_irq_enable(dev, MT_INT_RX_DONE_ALL | MT_INT_TX_DONE_ALL);
	return 0;
}

static int
mt7603_init_hardware(struct mt7603_dev *dev)
{
	int ret;

	mt76_wr(dev, MT_INT_SOURCE_CSR, ~0);
	mt76_clear(dev, MT_WPDMA_GLO_CFG,
		   MT_WPDMA_GLO_CFG_TX_DMA_EN |
		   MT_WPDMA_GLO_CFG_RX_DMA_EN);

	ret = mt7603_dma_init(dev);
	if (ret)
		return ret;

	ret = mt7603_mac_reset(dev);
	if (ret)
		return ret;

	dev->rxfilter = mt76_rr(dev, MT_WF_RFCR);
	set_bit(MT76_STATE_INITIALIZED, &dev->mt76.state);

	ret = mt7603_mac_start(dev);
	if (ret)
		return ret;

	ret = mt7603_mcu_init(dev);
	if (ret)
		return ret;

	return 0;
}

#define CCK_RATE(_idx, _rate) {					\
	.bitrate = _rate,					\
	.flags = IEEE80211_RATE_SHORT_PREAMBLE,			\
	.hw_value = (MT_PHY_TYPE_CCK << 8) | _idx,		\
	.hw_value_short = (MT_PHY_TYPE_CCK << 8) | (4 + _idx),	\
}

#define OFDM_RATE(_idx, _rate) {				\
	.bitrate = _rate,					\
	.hw_value = (MT_PHY_TYPE_OFDM << 8) | _idx,		\
	.hw_value_short = (MT_PHY_TYPE_OFDM << 8) | _idx,	\
}

static struct ieee80211_rate mt76x2_rates[] = {
	CCK_RATE(0, 10),
	CCK_RATE(1, 20),
	CCK_RATE(2, 55),
	CCK_RATE(3, 110),
	OFDM_RATE(11, 60),
	OFDM_RATE(15, 90),
	OFDM_RATE(10, 120),
	OFDM_RATE(14, 180),
	OFDM_RATE(9,  240),
	OFDM_RATE(13, 360),
	OFDM_RATE(8,  480),
	OFDM_RATE(12, 540),
};


int mt7603_register_device(struct mt7603_dev *dev)
{
	static u8 macaddr[] = { 0x00, 0x11, 0x22, 0x33, 0x44, 0x55 };
	struct ieee80211_hw *hw = mt76_hw(dev);
	struct ieee80211_supported_band *sband;
	struct wiphy *wiphy = hw->wiphy;
	int ret;

	ret = mt7603_init_hardware(dev);
	if (ret)
		return ret;

	hw->queues = 4;
	hw->max_rates = 1;
	hw->max_report_rates = 7;
	hw->max_rate_tries = 1;
	hw->extra_tx_headroom = 2;

	SET_IEEE80211_PERM_ADDR(hw, macaddr);

	ret = mt76_register_device(&dev->mt76, BIT(IEEE80211_BAND_2GHZ), true,
				   mt76x2_rates, ARRAY_SIZE(mt76x2_rates));
	if (ret)
		return ret;

	sband = wiphy->bands[IEEE80211_BAND_2GHZ];
	dev->chandef.chan = &sband->channels[0];

	mt76_register_debugfs(&dev->mt76);

	return 0;
}
