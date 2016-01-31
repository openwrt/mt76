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

#include "mt7603.h"

static int
mt7603_start(struct ieee80211_hw *hw)
{
	struct mt7603_dev *dev = hw->priv;

	mt7603_mac_start(dev);

	return 0;
}

static void
mt7603_stop(struct ieee80211_hw *hw)
{
	struct mt7603_dev *dev = hw->priv;

	mt7603_mac_stop(dev);
}

static void
mt7603_txq_init(struct mt7603_dev *dev, struct ieee80211_txq *txq)
{
	struct mt76_txq *mtxq;

	if (!txq)
		return;

	mtxq = (struct mt76_txq *) txq->drv_priv;
	if (txq->sta) {
		struct mt7603_sta *sta = (struct mt7603_sta *) txq->sta->drv_priv;
		mtxq->wcid = &sta->wcid;
	} else {
		struct mt7603_vif *mvif = (struct mt7603_vif *) txq->vif->drv_priv;
		mtxq->wcid = &mvif->group_wcid;
	}

	mt76_txq_init(&dev->mt76, txq);
}

static int
mt7603_add_interface(struct ieee80211_hw *hw, struct ieee80211_vif *vif)
{
	struct mt7603_vif *mvif = (struct mt7603_vif *) vif->drv_priv;
	struct mt7603_dev *dev = hw->priv;
	int ret = 0;

	mutex_lock(&dev->mutex);

	mvif->idx = ffs(~dev->vif_mask) - 1;
	if (mvif->idx >= MT7603_MAX_INTERFACES) {
		ret = -ENOSPC;
		goto out;
	}

	dev->vif_mask |= BIT(mvif->idx);
	mvif->group_wcid.idx = MT7603_WTBL_RESERVED - 1 - mvif->idx;
	mvif->group_wcid.hw_key_idx = -1;
	mt7603_txq_init(dev, vif->txq);

out:
	mutex_unlock(&dev->mutex);

	return ret;
}

static void
mt7603_remove_interface(struct ieee80211_hw *hw, struct ieee80211_vif *vif)
{
	struct mt7603_vif *mvif = (struct mt7603_vif *) vif->drv_priv;
	struct mt7603_dev *dev = hw->priv;

	mt76_txq_remove(&dev->mt76, vif->txq);

	mutex_lock(&dev->mutex);
	dev->vif_mask &= ~BIT(mvif->idx);
	mutex_unlock(&dev->mutex);
}

static int
mt7603_config(struct ieee80211_hw *hw, u32 changed)
{
	struct mt7603_dev *dev = hw->priv;
	int ret = 0;

	mutex_lock(&dev->mutex);

	if (changed & IEEE80211_CONF_CHANGE_CHANNEL) {
		ret = mt7603_set_channel(dev, &hw->conf.chandef);
	}

	mutex_unlock(&dev->mutex);

	return ret;
}

static void
mt7603_configure_filter(struct ieee80211_hw *hw, unsigned int changed_flags,
		      unsigned int *total_flags, u64 multicast)
{
	struct mt7603_dev *dev = hw->priv;
	u32 flags = 0;

#define MT76_FILTER(_flag, _hw) do { \
		flags |= *total_flags & FIF_##_flag;			\
		dev->rxfilter &= ~(_hw);				\
		dev->rxfilter |= !(flags & FIF_##_flag) * (_hw);	\
	} while (0)

	dev->rxfilter |= MT_WF_RFCR_DROP_STBC_MULTI;
	dev->rxfilter &= ~(MT_WF_RFCR_DROP_OTHER_BSS |
			   MT_WF_RFCR_DROP_OTHER_BEACON |
			   MT_WF_RFCR_DROP_FRAME_REPORT |
			   MT_WF_RFCR_DROP_PROBEREQ |
			   MT_WF_RFCR_DROP_MCAST_FILTERED |
			   MT_WF_RFCR_DROP_MCAST |
			   MT_WF_RFCR_DROP_BCAST |
			   MT_WF_RFCR_DROP_DUPLICATE);

	MT76_FILTER(OTHER_BSS, MT_WF_RFCR_DROP_OTHER_UC |
			       MT_WF_RFCR_DROP_OTHER_TIM |
			       MT_WF_RFCR_DROP_A3_MAC |
			       MT_WF_RFCR_DROP_A3_BSSID |
			       MT_WF_RFCR_DROP_A2_BSSID);

	MT76_FILTER(FCSFAIL, MT_WF_RFCR_DROP_FCSFAIL);

	MT76_FILTER(CONTROL, MT_WF_RFCR_DROP_UNWANTED_CTL |
			     MT_WF_RFCR_DROP_CTS |
			     MT_WF_RFCR_DROP_RTS |
			     MT_WF_RFCR_DROP_CTL_RSV |
			     MT_WF_RFCR_DROP_NDPA);

	*total_flags = flags;
	mt76_wr(dev, MT_WF_RFCR, dev->rxfilter);
}

static void
mt7603_bss_info_changed(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
		      struct ieee80211_bss_conf *info, u32 changed)
{
}

static int
mt7603_sta_add(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
	     struct ieee80211_sta *sta)
{
	struct mt7603_dev *dev = hw->priv;
	struct mt7603_sta *msta = (struct mt7603_sta *) sta->drv_priv;
	int ret, idx;

	mutex_lock(&dev->mutex);

	idx = mt76_wcid_alloc(dev->wcid_mask, ARRAY_SIZE(dev->wcid) - 1);
	if (idx < 0) {
		ret = -ENOSPC;
		goto out;
	}

	msta->wcid.idx = idx;
	mt7603_wtbl_update_cap(dev, sta);

	mt7603_wtbl_init(dev, idx, sta->addr);
	rcu_assign_pointer(dev->wcid[idx], &msta->wcid);

out:
	mutex_unlock(&dev->mutex);

	return ret;
}

static int
mt7603_sta_remove(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
		struct ieee80211_sta *sta)
{
	struct mt7603_dev *dev = hw->priv;
	struct mt7603_sta *msta = (struct mt7603_sta *) sta->drv_priv;
	int idx = msta->wcid.idx;

	mutex_lock(&dev->mutex);
	rcu_assign_pointer(dev->wcid[idx], NULL);
	mt7603_wtbl_clear(dev, idx);
	mutex_unlock(&dev->mutex);

	return 0;
}

static void
mt7603_sta_notify(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
		enum sta_notify_cmd cmd, struct ieee80211_sta *sta)
{
}

static int
mt7603_set_key(struct ieee80211_hw *hw, enum set_key_cmd cmd,
	     struct ieee80211_vif *vif, struct ieee80211_sta *sta,
	     struct ieee80211_key_conf *key)
{
	return -EINVAL;
}

static int
mt7603_conf_tx(struct ieee80211_hw *hw, struct ieee80211_vif *vif, u16 queue,
	     const struct ieee80211_tx_queue_params *params)
{
	return -EINVAL;
}

static void
mt7603_sw_scan(struct ieee80211_hw *hw, struct ieee80211_vif *vif, const u8 *mac)
{
}

static void
mt7603_sw_scan_complete(struct ieee80211_hw *hw, struct ieee80211_vif *vif)
{
}

static void
mt7603_flush(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
	   u32 queues, bool drop)
{
}

static int
mt7603_get_txpower(struct ieee80211_hw *hw, struct ieee80211_vif *vif, int *dbm)
{
	return -EINVAL;
}

static int
mt7603_ampdu_action(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
		    struct ieee80211_ampdu_params *params)
{
	return -EINVAL;
}

static void
mt7603_sta_rate_tbl_update(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
			 struct ieee80211_sta *sta)
{
	struct mt7603_dev *dev = hw->priv;
	struct mt7603_sta *msta = (struct mt7603_sta *) sta->drv_priv;
	struct ieee80211_sta_rates *sta_rates = rcu_dereference(sta->rates);
	struct ieee80211_tx_rate rates[IEEE80211_TX_MAX_RATES] = {};
	int i;

	for (i = 0; i < ARRAY_SIZE(rates); i++) {
		rates[i].idx = sta_rates->rate[i].idx;
		rates[i].count = sta_rates->rate[i].count;
		rates[i].flags = sta_rates->rate[i].flags;
		if (rates[i].idx < 0 || !rates[i].count)
			break;
	}

	mt7603_wtbl_set_rates(dev, msta->wcid.idx, rates, i);
}

static void mt7603_set_coverage_class(struct ieee80211_hw *hw,
				    s16 coverage_class)
{
}

static void mt7603_tx(struct ieee80211_hw *hw, struct ieee80211_tx_control *control,
		      struct sk_buff *skb)
{
	struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);
	struct ieee80211_vif *vif = info->control.vif;
	struct mt7603_vif *mvif = (struct mt7603_vif *) vif->drv_priv;
	struct mt76_wcid *wcid = &mvif->group_wcid;
	struct mt7603_dev *dev = hw->priv;

	mt76_tx(&dev->mt76, control->sta, wcid, skb);
}

const struct ieee80211_ops mt7603_ops = {
	.tx = mt7603_tx,
	.start = mt7603_start,
	.stop = mt7603_stop,
	.add_interface = mt7603_add_interface,
	.remove_interface = mt7603_remove_interface,
	.config = mt7603_config,
	.configure_filter = mt7603_configure_filter,
	.bss_info_changed = mt7603_bss_info_changed,
	.sta_add = mt7603_sta_add,
	.sta_remove = mt7603_sta_remove,
	.sta_notify = mt7603_sta_notify,
	.set_key = mt7603_set_key,
	.conf_tx = mt7603_conf_tx,
	.sw_scan_start = mt7603_sw_scan,
	.sw_scan_complete = mt7603_sw_scan_complete,
	.flush = mt7603_flush,
	.ampdu_action = mt7603_ampdu_action,
	.get_txpower = mt7603_get_txpower,
	.wake_tx_queue = mt76_wake_tx_queue,
	.sta_rate_tbl_update = mt7603_sta_rate_tbl_update,
	.release_buffered_frames = mt76_release_buffered_frames,
	.set_coverage_class = mt7603_set_coverage_class,
};
