/*
 * Copyright (C) 2016 Felix Fietkau <nbd@openwrt.org>
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

#include <linux/etherdevice.h>
#include "mt7603.h"
#include "mt7603_mac.h"
#include "mt7603_eeprom.h"

struct mt7603_dev *mt7603_alloc_device(struct device *pdev)
{
	static const struct mt76_driver_ops drv_ops = {
		.txwi_size = MT_TXD_SIZE,
		.tx_prepare_skb = mt7603_tx_prepare_skb,
		.tx_complete_skb = mt7603_tx_complete_skb,
		.rx_skb = mt7603_queue_rx_skb,
		.rx_poll_complete = mt7603_rx_poll_complete,
	};
	struct ieee80211_hw *hw;
	struct mt7603_dev *dev;

	hw = ieee80211_alloc_hw(sizeof(*dev), &mt7603_ops);
	if (!hw)
		return NULL;

	dev = hw->priv;
	dev->mt76.dev = pdev;
	dev->mt76.hw = hw;
	dev->mt76.drv = &drv_ops;

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

static void
mt7603_set_tmac_template(struct mt7603_dev *dev)
{
	u32 desc[5] = {
		[1] = MT76_SET(MT_TXD3_REM_TX_COUNT, 0xf),
		[3] = MT_TXD5_SW_POWER_MGMT
	};
	u32 addr;
	int i;

	addr = mt7603_reg_map(dev, MT_CLIENT_BASE_PHYS_ADDR);
	addr += MT_CLIENT_TMAC_INFO_TEMPLATE;
	for (i = 0; i < ARRAY_SIZE(desc); i++)
		mt76_wr(dev, addr + 4 * i, desc[i]);
}

static int
mt7603_mac_early_init(struct mt7603_dev *dev)
{
	mt76_wr(dev, MT_WPDMA_GLO_CFG, 0x52000850);

	mt76_clear(dev, MT_ARB_SCR, MT_ARB_SCR_TX_DISABLE | MT_ARB_SCR_RX_DISABLE);
	mt76_wr(dev, MT_WF_ARB_TX_START_0, ~0);
	mt76_clear(dev, MT_WF_ARB_RQCR, MT_WF_ARB_RQCR_RX_START);

	wait_for_wpdma(dev);
	udelay(50);

	mt76_set(dev, MT_WPDMA_GLO_CFG,
		 (MT_WPDMA_GLO_CFG_TX_DMA_EN |
		  MT_WPDMA_GLO_CFG_RX_DMA_EN |
		  MT76_SET(MT_WPDMA_GLO_CFG_DMA_BURST_SIZE, 3) |
		  MT_WPDMA_GLO_CFG_TX_WRITEBACK_DONE));

	mt7603_irq_enable(dev, MT_INT_RX_DONE_ALL | MT_INT_TX_DONE_ALL);

	return 0;
}

static void
mt7603_dma_sched_init(struct mt7603_dev *dev)
{
	int page_size, page_count;
	int max_len = 1792;
	int max_amsdu_len = 4096;
	int max_mcu_len = 4096;
	int max_beacon_len = 512 * 8 + max_len;
	int max_mcast_count = 3;
	int beacon_pages;
	int mcu_pages;
	int i;

	page_size = mt76_get_field(dev, MT_PSE_FC_P0,
				   MT_PSE_FC_P0_MAX_QUOTA);
	page_count = 0x1ae;
	beacon_pages = max_beacon_len / page_size;
	mcu_pages = max_mcu_len / page_size;

	mt76_wr(dev, MT_PSE_FRP,
		MT76_SET(MT_PSE_FRP_P0, 7) |
		MT76_SET(MT_PSE_FRP_P1, 6) |
		MT76_SET(MT_PSE_FRP_P2_RQ2, 4));

	mt76_wr(dev, MT_HIGH_PRIORITY_1, 0x55555553);
	mt76_wr(dev, MT_HIGH_PRIORITY_2, 0x78555555);

	mt76_wr(dev, MT_QUEUE_PRIORITY_1, 0x2b1a096e);
	mt76_wr(dev, MT_QUEUE_PRIORITY_2, 0x785f4d3c);

	mt76_wr(dev, MT_PRIORITY_MASK, 0xffffffff);

	mt76_wr(dev, MT_SCH_1, page_count | (2 << 28));
	mt76_wr(dev, MT_SCH_2, max_len / page_size);

	for (i = 0; i <= 4; i++)
		mt76_wr(dev, MT_PAGE_COUNT(i), max_amsdu_len / page_size);

	mt76_wr(dev, MT_PAGE_COUNT(5), mcu_pages);
	mt76_wr(dev, MT_PAGE_COUNT(7), beacon_pages);

	mt76_wr(dev, MT_PAGE_COUNT(8),
		(max_mcast_count + 1) * max_len / page_size);

	mt76_wr(dev, MT_RSV_MAX_THRESH, page_count);

	if (mt76xx_rev(dev) < MT7603_REV_E2) {
		mt76_wr(dev, MT_GROUP_THRESH(0), page_count);
		mt76_wr(dev, MT_BMAP_0, 0xffff);
	} else {
		mt76_wr(dev, MT_GROUP_THRESH(0),
			page_count - beacon_pages - mcu_pages);
		mt76_wr(dev, MT_GROUP_THRESH(1), beacon_pages);
		mt76_wr(dev, MT_BMAP_0, 0x0080ff5f);
		mt76_wr(dev, MT_GROUP_THRESH(2), mcu_pages);
		mt76_wr(dev, MT_BMAP_1, 0x00000020);
	}

	mt76_wr(dev, MT_SCH_4, 0);

	for (i = 0; i <= 15; i++)
		mt76_wr(dev, MT_TXTIME_THRESH(i), 0xfffff);

	mt76_set(dev, MT_SCH_4, BIT(6));
}

static void
mt7603_phy_init(struct mt7603_dev *dev)
{
	int rx_chains = BIT(dev->rx_chains) - 1;
	int tx_chains = dev->tx_chains - 1;

	mt76_rmw(dev, MT_WF_RMAC_RMCR,
		 (MT_WF_RMAC_RMCR_SMPS_MODE |
		  MT_WF_RMAC_RMCR_RX_STREAMS),
		 (MT76_SET(MT_WF_RMAC_RMCR_SMPS_MODE, 3) |
		  MT76_SET(MT_WF_RMAC_RMCR_RX_STREAMS, rx_chains)));

	mt76_rmw_field(dev, MT_TMAC_TCR, MT_TMAC_TCR_TX_STREAMS,
		       tx_chains);
}

static void
mt7603_mac_init(struct mt7603_dev *dev)
{
	u8 bc_addr[ETH_ALEN];
	u32 addr;
	int i;

	mt76_wr(dev, MT_AGG_BA_SIZE_LIMIT_0,
		(MT_AGG_SIZE_LIMIT(0) << 0 * MT_AGG_BA_SIZE_LIMIT_SHIFT) |
		(MT_AGG_SIZE_LIMIT(1) << 1 * MT_AGG_BA_SIZE_LIMIT_SHIFT) |
		(MT_AGG_SIZE_LIMIT(2) << 2 * MT_AGG_BA_SIZE_LIMIT_SHIFT) |
		(MT_AGG_SIZE_LIMIT(3) << 3 * MT_AGG_BA_SIZE_LIMIT_SHIFT));

	mt76_wr(dev, MT_AGG_BA_SIZE_LIMIT_1,
		(MT_AGG_SIZE_LIMIT(4) << 0 * MT_AGG_BA_SIZE_LIMIT_SHIFT) |
		(MT_AGG_SIZE_LIMIT(5) << 1 * MT_AGG_BA_SIZE_LIMIT_SHIFT) |
		(MT_AGG_SIZE_LIMIT(6) << 2 * MT_AGG_BA_SIZE_LIMIT_SHIFT) |
		(MT_AGG_SIZE_LIMIT(7) << 3 * MT_AGG_BA_SIZE_LIMIT_SHIFT));

	mt76_wr(dev, MT_AGG_LIMIT,
		MT76_SET(MT_AGG_LIMIT_AC(0), 21) |
		MT76_SET(MT_AGG_LIMIT_AC(1), 21) |
		MT76_SET(MT_AGG_LIMIT_AC(2), 21) |
		MT76_SET(MT_AGG_LIMIT_AC(3), 21));

	mt76_wr(dev, MT_AGG_LIMIT_1,
		MT76_SET(MT_AGG_LIMIT_AC(0), 21) |
		MT76_SET(MT_AGG_LIMIT_AC(1), 21) |
		MT76_SET(MT_AGG_LIMIT_AC(2), 21) |
		MT76_SET(MT_AGG_LIMIT_AC(3), 21));

	mt76_wr(dev, MT_AGG_CONTROL,
		MT76_SET(MT_AGG_CONTROL_BAR_RATE, 0x80) |
		MT76_SET(MT_AGG_CONTROL_CFEND_RATE, 0x69) |
		MT_AGG_CONTROL_NO_BA_AR_RULE);

	mt76_wr(dev, MT_AGG_RETRY_CONTROL,
		MT76_SET(MT_AGG_RETRY_CONTROL_BAR_LIMIT, 1) |
		MT76_SET(MT_AGG_RETRY_CONTROL_RTS_LIMIT, 15));

	mt76_rmw(dev, MT_DMA_DCR0, ~0xfffc, MT_RX_BUF_SIZE);

	mt76_rmw(dev, MT_DMA_VCFR0, BIT(0), BIT(13));
	mt76_rmw(dev, MT_DMA_TMCFR0, BIT(0) | BIT(1), BIT(13));

	mt76_clear(dev, MT_WF_RMAC_TMR_PA, BIT(31));

	mt76_set(dev, MT_WF_RMACDR, MT_WF_RMACDR_MAXLEN_20BIT);
	mt76_rmw(dev, MT_WF_RMAC_MAXMINLEN, 0xffffff, 0x19000);

	mt76_wr(dev, MT_WF_RFCR1, 0);

	mt76_set(dev, MT_TMAC_TCR, MT_TMAC_TCR_RX_RIFS_MODE);

	mt7603_set_tmac_template(dev);

	/* Enable RX group to HIF */
	addr = mt7603_reg_map(dev, MT_CLIENT_BASE_PHYS_ADDR);
	mt76_set(dev, addr + MT_CLIENT_RXINF, MT_CLIENT_RXINF_RXSH_GROUPS);

	/* Enable RX group to MCU */
	mt76_set(dev, MT_DMA_DCR1, GENMASK(13, 11));

	mt76_rmw_field(dev, MT_AGG_PCR, MT_AGG_PCR_RTS_PKT_THR, 3);
	mt76_set(dev, MT_TMAC_PCR, MT_TMAC_PCR_SPE_EN);
	mt76_wr(dev, MT_RXREQ, 4);

	/* Configure all rx packets to HIF */
	mt76_wr(dev, MT_DMA_RCFR0, 0xc0200000);

	/* Configure txs selection with aggregation */
	mt76_wr(dev, MT_DMA_TCFR1,
		MT76_SET(MT_DMA_TCFR1_TXS_AGGR_TIMEOUT, 1) | /* 32 us */
		MT_DMA_TCFR1_TXS_AGGR_COUNT | /* Maximum count */
		MT_DMA_TCFR1_TXS_QUEUE | /* Queue 1 */
		MT_DMA_TCFR1_TXS_BIT_MAP);

	mt76_wr(dev, MT_MCU_PCIE_REMAP_1, MT_PSE_WTBL_2_PHYS_ADDR);

	for (i = 0; i < MT7603_WTBL_SIZE; i++)
		mt7603_wtbl_clear(dev, i);

	eth_broadcast_addr(bc_addr);
	mt7603_wtbl_init(dev, MT7603_WTBL_RESERVED, bc_addr);
	dev->global_sta.wcid.idx = MT7603_WTBL_RESERVED;
	rcu_assign_pointer(dev->wcid[MT7603_WTBL_RESERVED],
			   &dev->global_sta.wcid);

	mt76_rmw_field(dev, MT_LPON_BTEIR, MT_LPON_BTEIR_MBSS_MODE, 2);
	mt76_rmw_field(dev, MT_WF_RMACDR, MT_WF_RMACDR_MBSSID_MASK, 2);

	mt76_wr(dev, MT_AGG_ARUCR, 0);
	mt76_wr(dev, MT_AGG_ARDCR,
		MT76_SET(MT_AGG_ARxCR_LIMIT(0), MT7603_RATE_RETRY - 1) |
		MT76_SET(MT_AGG_ARxCR_LIMIT(1), MT7603_RATE_RETRY - 1) |
		MT76_SET(MT_AGG_ARxCR_LIMIT(2), MT7603_RATE_RETRY - 1) |
		MT76_SET(MT_AGG_ARxCR_LIMIT(3), MT7603_RATE_RETRY - 1) |
		MT76_SET(MT_AGG_ARxCR_LIMIT(4), MT7603_RATE_RETRY - 1) |
		MT76_SET(MT_AGG_ARxCR_LIMIT(5), MT7603_RATE_RETRY - 1) |
		MT76_SET(MT_AGG_ARxCR_LIMIT(6), MT7603_RATE_RETRY - 1) |
		MT76_SET(MT_AGG_ARxCR_LIMIT(7), MT7603_RATE_RETRY - 1));

	mt76_wr(dev, MT_AGG_ARCR, (MT_AGG_ARCR_INIT_RATE1 |
				   MT76_SET(MT_AGG_ARCR_RTS_RATE_THR, 2)));
}

static int
mt7603_init_hardware(struct mt7603_dev *dev)
{
	int ret;

	mt76_wr(dev, MT_INT_SOURCE_CSR, ~0);

	ret = mt7603_eeprom_init(dev);
	if (ret < 0)
		return ret;

	ret = mt7603_dma_init(dev);
	if (ret)
		return ret;

	ret = mt7603_mac_early_init(dev);
	if (ret)
		return ret;

	dev->rxfilter = mt76_rr(dev, MT_WF_RFCR);
	set_bit(MT76_STATE_INITIALIZED, &dev->mt76.state);

	ret = mt7603_mcu_init(dev);
	if (ret)
		return ret;

	mt7603_dma_sched_init(dev);
	mt7603_mcu_set_eeprom(dev);
	mt7603_phy_init(dev);
	mt7603_mac_init(dev);

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

static struct ieee80211_rate mt7603_rates[] = {
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

static const struct ieee80211_iface_limit if_limits[] = {
	{
		.max = 1,
		.types = BIT(NL80211_IFTYPE_ADHOC)
	}, {
		.max = MT7603_MAX_INTERFACES,
		.types = BIT(NL80211_IFTYPE_STATION) |
#ifdef CONFIG_MAC80211_MESH
			 BIT(NL80211_IFTYPE_MESH_POINT) |
#endif
			 BIT(NL80211_IFTYPE_AP)
	 },
};

static const struct ieee80211_iface_combination if_comb[] = {
	{
		.limits = if_limits,
		.n_limits = ARRAY_SIZE(if_limits),
		.max_interfaces = 4,
		.num_different_channels = 1,
		.beacon_int_infra_match = true,
	}
};

int mt7603_register_device(struct mt7603_dev *dev)
{
	struct ieee80211_hw *hw = mt76_hw(dev);
	struct ieee80211_supported_band *sband;
	struct wiphy *wiphy = hw->wiphy;
	int ret;

	mutex_init(&dev->mutex);
	spin_lock_init(&dev->status_lock);
	INIT_LIST_HEAD(&dev->status_list);

	tasklet_init(&dev->pre_tbtt_tasklet, mt7603_pre_tbtt_tasklet, (unsigned long) dev);

	dev->rx_chains = 2;
	dev->tx_chains = 2;
	dev->slottime = 9;

	ret = mt7603_init_hardware(dev);
	if (ret)
		return ret;

	hw->queues = 4;
	hw->max_rates = 4;
	hw->max_report_rates = 7;
	hw->max_rate_tries = 11;

	hw->sta_data_size = sizeof(struct mt7603_sta);
	hw->vif_data_size = sizeof(struct mt7603_vif);

	wiphy->iface_combinations = if_comb;
	wiphy->n_iface_combinations = ARRAY_SIZE(if_comb);

	ieee80211_hw_set(hw, REPORTS_TX_ACK_STATUS);

	ret = mt76_register_device(&dev->mt76, true, mt7603_rates,
				   ARRAY_SIZE(mt7603_rates));
	if (ret)
		return ret;

	sband = wiphy->bands[IEEE80211_BAND_2GHZ];
	dev->chandef.chan = &sband->channels[0];

	mt76_register_debugfs(&dev->mt76);

	return 0;
}

void mt7603_unregister_device(struct mt7603_dev *dev)
{
	mt7603_mac_status_skb(dev, NULL, -1);
	mt76_unregister_device(&dev->mt76);
	mt7603_mcu_exit(dev);
	mt7603_dma_cleanup(dev);
	ieee80211_free_hw(mt76_hw(dev));
}
