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

#include "mt76x2.h"
#include "mt76x2_dma.h"

struct beacon_bc_data {
	struct mt76x2_dev *dev;
	struct sk_buff_head q;
	struct sk_buff *tail[8];
};

void mt76x2_tx(struct ieee80211_hw *hw, struct ieee80211_tx_control *control,
	     struct sk_buff *skb)
{
	struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);
	struct mt76x2_dev *dev = hw->priv;
	struct ieee80211_vif *vif = info->control.vif;
	struct mt76x2_vif *mvif = (struct mt76x2_vif *) vif->drv_priv;
	struct mt76x2_sta *msta = NULL;
	struct mt76_wcid *wcid = &mvif->group_wcid;

	if (control->sta) {
		msta = (struct mt76x2_sta *) control->sta->drv_priv;
		wcid = &msta->wcid;
	}

	mt76_tx(&dev->mt76, control->sta, wcid, skb);
}

void mt76x2_tx_complete(struct mt76x2_dev *dev, struct sk_buff *skb)
{
	struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);
	struct mt76_queue *q;
	int qid = skb_get_queue_mapping(skb);

	if (info->flags & IEEE80211_TX_CTL_AMPDU) {
		ieee80211_free_txskb(mt76_hw(dev), skb);
	} else {
		ieee80211_tx_info_clear_status(info);
		info->status.rates[0].idx = -1;
		info->flags |= IEEE80211_TX_STAT_ACK;
		ieee80211_tx_status(mt76_hw(dev), skb);
	}

	q = &dev->mt76.q_tx[qid];
	if (q->queued < q->ndesc - 8)
		ieee80211_wake_queue(mt76_hw(dev), qid);
}

int mt76x2_tx_prepare_skb(struct mt76_dev *mdev, void *txwi,
			  struct sk_buff *skb, struct mt76_queue *q,
			  struct mt76_wcid *wcid, struct ieee80211_sta *sta,
			  u32 *tx_info)
{
	struct mt76x2_dev *dev = container_of(mdev, struct mt76x2_dev, mt76);
	struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);
	int qsel = MT_QSEL_EDCA;
	int ret;

	mt76x2_mac_write_txwi(dev, txwi, skb, wcid, sta);

	ret = mt76_insert_hdr_pad(skb);
	if (ret < 0)
		return ret;

	if (info->flags & IEEE80211_TX_CTL_RATE_CTRL_PROBE)
		qsel = 0;

	*tx_info = MT76_SET(MT_TXD_INFO_QSEL, qsel) |
		   MT_TXD_INFO_80211;

	if (!wcid || wcid->hw_key_idx == 0xff)
		*tx_info |= MT_TXD_INFO_WIV;

	return 0;
}

static void
mt76x2_update_beacon_iter(void *priv, u8 *mac, struct ieee80211_vif *vif)
{
	struct mt76x2_dev *dev = (struct mt76x2_dev *) priv;
	struct mt76x2_vif *mvif = (struct mt76x2_vif *) vif->drv_priv;
	struct sk_buff *skb = NULL;

	if (!(dev->beacon_mask & BIT(mvif->idx)))
		return;

	skb = ieee80211_beacon_get(mt76_hw(dev), vif);
	if (!skb)
		return;

	mt76x2_mac_set_beacon(dev, mvif->idx, skb);
}

static void
mt76x2_add_buffered_bc(void *priv, u8 *mac, struct ieee80211_vif *vif)
{
	struct beacon_bc_data *data = priv;
	struct mt76x2_dev *dev = data->dev;
	struct mt76x2_vif *mvif = (struct mt76x2_vif *) vif->drv_priv;
	struct ieee80211_tx_info *info;
	struct sk_buff *skb;

	if (!(dev->beacon_mask & BIT(mvif->idx)))
		return;

	skb = ieee80211_get_buffered_bc(mt76_hw(dev), vif);
	if (!skb)
		return;

	info = IEEE80211_SKB_CB(skb);
	info->control.vif = vif;
	info->flags |= IEEE80211_TX_CTL_ASSIGN_SEQ;
	mt76_skb_set_moredata(skb, true);
	__skb_queue_tail(&data->q, skb);
	data->tail[mvif->idx] = skb;
}

void mt76x2_pre_tbtt_tasklet(unsigned long arg)
{
	struct mt76x2_dev *dev = (struct mt76x2_dev *) arg;
	struct mt76_queue *q = &dev->mt76.q_tx[MT_TXQ_PSD];
	struct beacon_bc_data data = {};
	struct sk_buff *skb;
	int i, nframes;

	data.dev = dev;
	__skb_queue_head_init(&data.q);

	ieee80211_iterate_active_interfaces_atomic(mt76_hw(dev),
		IEEE80211_IFACE_ITER_RESUME_ALL,
		mt76x2_update_beacon_iter, dev);

	do {
		nframes = skb_queue_len(&data.q);
		ieee80211_iterate_active_interfaces_atomic(mt76_hw(dev),
			IEEE80211_IFACE_ITER_RESUME_ALL,
			mt76x2_add_buffered_bc, &data);
	} while (nframes != skb_queue_len(&data.q));

	if (!nframes)
		return;

	for (i = 0; i < ARRAY_SIZE(data.tail); i++) {
		if (!data.tail[i])
			continue;

		mt76_skb_set_moredata(data.tail[i], false);
	}

	spin_lock_bh(&q->lock);
	while ((skb = __skb_dequeue(&data.q)) != NULL) {
		struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);
		struct ieee80211_vif *vif = info->control.vif;
		struct mt76x2_vif *mvif = (struct mt76x2_vif *) vif->drv_priv;

		mt76_tx_queue_skb(&dev->mt76, q, skb, &mvif->group_wcid, NULL);
	}
	spin_unlock_bh(&q->lock);
}

