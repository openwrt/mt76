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
#include "mt7603_mac.h"

#define MT_PSE_PAGE_SIZE	128

static u32
mt7603_ac_queue_mask0(u32 mask)
{
	u32 ret = 0;

	ret |= GENMASK(3, 0) * !!(mask & BIT(0));
	ret |= GENMASK(8, 5) * !!(mask & BIT(1));
	ret |= GENMASK(13, 10) * !!(mask & BIT(2));
	ret |= GENMASK(19, 16) * !!(mask & BIT(3));
	return ret;
}

static void
mt76_stop_tx_ac(struct mt7603_dev *dev, u32 mask)
{
	mt76_set(dev, MT_WF_ARB_TX_STOP_0, mt7603_ac_queue_mask0(mask));
}

static void
mt76_start_tx_ac(struct mt7603_dev *dev, u32 mask)
{
	mt76_set(dev, MT_WF_ARB_TX_START_0, mt7603_ac_queue_mask0(mask));
}

static void
mt7603_wtbl_update(struct mt7603_dev *dev, int idx, u32 mask)
{
	mt76_rmw(dev, MT_WTBL_UPDATE, MT_WTBL_UPDATE_WLAN_IDX,
		 MT76_SET(MT_WTBL_UPDATE_WLAN_IDX, idx) | mask);

	mt76_poll(dev, MT_WTBL_UPDATE, MT_WTBL_UPDATE_BUSY, 0, 5000);
}

void mt7603_wtbl_init(struct mt7603_dev *dev, int idx, const u8 *mac_addr)
{
	const void *_mac = mac_addr;
	u32 addr = MT_WTBL1_BASE + idx * MT_WTBL1_SIZE;

	mt76_set(dev, addr + 0 * 4,
		 MT76_SET(MT_WTBL1_W0_ADDR_HI, get_unaligned_le16(_mac + 4)));
	mt76_set(dev, addr + 1 * 4,
		 MT76_SET(MT_WTBL1_W1_ADDR_LO, get_unaligned_le32(_mac)));
	mt76_set(dev, addr + 2 * 4, MT_WTBL1_W2_ADMISSION_CONTROL);
}

void mt7603_wtbl_clear(struct mt7603_dev *dev, int idx)
{
	int wtbl2_frame = (idx * MT_WTBL2_SIZE) / MT_PSE_PAGE_SIZE;
	int wtbl2_entry = (idx * MT_WTBL2_SIZE) % MT_PSE_PAGE_SIZE;
	int wtbl3_frame = (idx * MT_WTBL3_SIZE) / MT_PSE_PAGE_SIZE;
	int wtbl3_entry = ((idx * MT_WTBL3_SIZE) % MT_PSE_PAGE_SIZE) * 2;
	int wtbl4_frame = (idx * MT_WTBL4_SIZE) / MT_PSE_PAGE_SIZE;
	int wtbl4_entry = (idx * MT_WTBL4_SIZE) % MT_PSE_PAGE_SIZE;
	u32 addr = MT_WTBL1_BASE + idx * MT_WTBL1_SIZE;
	int i;

	mt76_wr(dev, addr + 0 * 4,
		MT_WTBL1_W0_RX_CHECK_A2 |
		MT_WTBL1_W0_RX_VALID);
	mt76_wr(dev, addr + 1 * 4, 0);
	mt76_wr(dev, addr + 2 * 4, 0);

	mt76_set(dev, MT_WTBL1_OR, MT_WTBL1_OR_PSM_WRITE);

	mt76_wr(dev, addr + 3 * 4,
		MT76_SET(MT_WTBL1_W3_WTBL2_FRAME_ID, wtbl2_frame) |
		MT76_SET(MT_WTBL1_W3_WTBL2_ENTRY_ID, wtbl2_entry) |
		MT76_SET(MT_WTBL1_W3_WTBL4_FRAME_ID, wtbl4_frame));
	mt76_wr(dev, addr + 4 * 4,
		MT76_SET(MT_WTBL1_W4_WTBL3_FRAME_ID, wtbl3_frame) |
		MT76_SET(MT_WTBL1_W4_WTBL3_ENTRY_ID, wtbl3_entry) |
		MT76_SET(MT_WTBL1_W4_WTBL4_ENTRY_ID, wtbl4_entry));

	mt76_clear(dev, MT_WTBL1_OR, MT_WTBL1_OR_PSM_WRITE);

	/* Mapped to WTBL2 */
	addr = MT_PCIE_REMAP_BASE_1 + idx * MT_WTBL2_SIZE;

	/* Clear BA information */
	mt76_wr(dev, addr + (15 * 4), 0);

	mt76_stop_tx_ac(dev, GENMASK(3, 0));
	for (i = 2; i <= 4; i++)
		mt76_wr(dev, addr + (i * 4), 0);
	mt7603_wtbl_update(dev, idx, MT_WTBL_UPDATE_WTBL2);
	mt76_start_tx_ac(dev, GENMASK(3, 0));

	mt7603_wtbl_update(dev, idx, MT_WTBL_UPDATE_RX_COUNT_CLEAR);
	mt7603_wtbl_update(dev, idx, MT_WTBL_UPDATE_TX_COUNT_CLEAR);
	mt7603_wtbl_update(dev, idx, MT_WTBL_UPDATE_ADM_COUNT_CLEAR);
}

static int
mt7603_get_rate(struct mt7603_dev *dev, struct ieee80211_supported_band *sband,
		int idx, bool cck)
{
	int offset = 0;
	int len = sband->n_bitrates;
	int i;

	if (cck) {
		if (WARN_ON_ONCE(sband == &dev->mt76.sband_5g))
			return 0;

		idx &= ~BIT(2); /* short preamble */
	} else if (sband == &dev->mt76.sband_2g) {
		offset = 4;
	}

	for (i = offset; i < len; i++) {
		if ((sband->bitrates[i].hw_value & GENMASK(7, 0)) == idx)
			return i;
	}

	WARN_ON_ONCE(1);
	return 0;
}

int
mt7603_mac_fill_rx(struct mt7603_dev *dev, struct sk_buff *skb)
{
	struct ieee80211_rx_status *status = IEEE80211_SKB_RXCB(skb);
	struct ieee80211_supported_band *sband;
	__le32 *rxd = (__le32 *) skb->data;
	u32 rxd0 = le32_to_cpu(rxd[0]);
	bool remove_pad;
	int i;

	memset(status, 0, sizeof(*status));

	i = MT76_GET(MT_RXD1_NORMAL_CH_FREQ, rxd[1]);
	sband = (i & 1) ? &dev->mt76.sband_5g : &dev->mt76.sband_2g;
	i >>= 1;
	if (i < sband->n_channels)
		status->freq = sband->channels[i].center_freq;

	if (rxd[2] & MT_RXD2_NORMAL_FCS_ERR)
		status->rx_flags |= RX_FLAG_FAILED_FCS_CRC;

	if (rxd[2] & MT_RXD2_NORMAL_TKIP_MIC_ERR)
		status->rx_flags |= RX_FLAG_MMIC_ERROR;

	if (MT76_GET(MT_RXD2_NORMAL_SEC_MODE, rxd[2]) != 0 &&
	    !(rxd[2] & (MT_RXD2_NORMAL_CLM | MT_RXD2_NORMAL_CM)))
		status->rx_flags |= RX_FLAG_DECRYPTED;

	remove_pad = rxd[1] & MT_RXD1_NORMAL_HDR_OFFSET;

	if (rxd[2] & MT_RXD2_NORMAL_MAX_LEN_ERROR)
		return -EINVAL;

	if (WARN_ON_ONCE(!sband->channels))
		return -EINVAL;

	rxd += 4;
	if (rxd0 & MT_RXD0_NORMAL_GROUP_4) {
		rxd += 4;
		if ((u8 *) rxd - skb->data >= skb->len)
			return -EINVAL;
	}
	if (rxd0 & MT_RXD0_NORMAL_GROUP_1) {
		rxd += 4;
		if ((u8 *) rxd - skb->data >= skb->len)
			return -EINVAL;
	}
	if (rxd0 & MT_RXD0_NORMAL_GROUP_2) {
		rxd += 2;
		if ((u8 *) rxd - skb->data >= skb->len)
			return -EINVAL;
	}
	if (rxd0 & MT_RXD0_NORMAL_GROUP_3) {
		bool cck = false;

		i = MT76_GET(MT_RXV1_TX_RATE, rxd[0]);
		switch (MT76_GET(MT_RXV1_TX_MODE, rxd[0])) {
		case MT_PHY_TYPE_CCK:
			cck = true;
			/* fall through */
		case MT_PHY_TYPE_OFDM:
			i = mt7603_get_rate(dev, sband, i, cck);
			break;
		case MT_PHY_TYPE_HT_GF:
		case MT_PHY_TYPE_HT:
			status->rx_flags |= RX_FLAG_HT;
			break;
		case MT_PHY_TYPE_VHT:
			status->rx_flags |= RX_FLAG_VHT;
			break;
		default:
			WARN_ON(1);
		}

		if (rxd[0] & MT_RXV1_HT_SHORT_GI)
			status->rx_flags |= RX_FLAG_SHORT_GI;

		status->rx_flags |= RX_FLAG_STBC_MASK *
				    MT76_GET(MT_RXV1_HT_STBC, rxd[0]);

		status->rate_idx = i;

		status->chains = BIT(0) | BIT(1);
		status->chain_signal[0] = MT76_GET(MT_RXV4_IB_RSSI0, rxd[3]) +
					  dev->rssi_offset[0];
		status->chain_signal[1] = MT76_GET(MT_RXV4_IB_RSSI1, rxd[3]) +
					  dev->rssi_offset[1];
		status->signal = max(status->chain_signal[0], status->chain_signal[1]);

		rxd += 6;
		if ((u8 *) rxd - skb->data >= skb->len)
			return -EINVAL;
	}

	skb_pull(skb, (u8 *) rxd - skb->data + 2 * remove_pad);

	return 0;
}

void mt7603_mac_start(struct mt7603_dev *dev)
{
	mt76_clear(dev, MT_WF_ARB_SCR, MT_WF_ARB_TX_DISABLE | MT_WF_ARB_RX_DISABLE);
	mt76_wr(dev, MT_WF_ARB_TX_START_0, ~0);
	mt76_set(dev, MT_WF_ARB_RQCR, MT_WF_ARB_RQCR_RX_START);
}

void mt7603_mac_stop(struct mt7603_dev *dev)
{
	mt76_set(dev, MT_WF_ARB_SCR, MT_WF_ARB_TX_DISABLE | MT_WF_ARB_RX_DISABLE);
	mt76_wr(dev, MT_WF_ARB_TX_START_0, 0);
	mt76_clear(dev, MT_WF_ARB_RQCR, MT_WF_ARB_RQCR_RX_START);
}
