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
	return -EINVAL;
}

static void
mt7603_stop(struct ieee80211_hw *hw)
{
}

static int
mt7603_add_interface(struct ieee80211_hw *hw, struct ieee80211_vif *vif)
{
	return -EINVAL;
}

static void
mt7603_remove_interface(struct ieee80211_hw *hw, struct ieee80211_vif *vif)
{
}

static int
mt7603_config(struct ieee80211_hw *hw, u32 changed)
{
	return -EINVAL;
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
	return -EINVAL;
}

static int
mt7603_sta_remove(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
		struct ieee80211_sta *sta)
{
	return -EINVAL;
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
		  enum ieee80211_ampdu_mlme_action action,
		  struct ieee80211_sta *sta,u16 tid, u16 *ssn, u8 buf_size,
		  bool amsdu)
{
	return -EINVAL;
}

static void
mt7603_sta_rate_tbl_update(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
			 struct ieee80211_sta *sta)
{
}

static void mt7603_set_coverage_class(struct ieee80211_hw *hw,
				    s16 coverage_class)
{
}

static void mt7603_tx(struct ieee80211_hw *hw, struct ieee80211_tx_control *control,
		      struct sk_buff *skb)
{
}

static void mt7603_wake_tx_queue(struct ieee80211_hw *hw, struct ieee80211_txq *txq)
{
}

static void
mt7603_release_buffered_frames(struct ieee80211_hw *hw, struct ieee80211_sta *sta,
			     u16 tids, int nframes,
			     enum ieee80211_frame_release_type reason,
			     bool more_data)
{
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
	.wake_tx_queue = mt7603_wake_tx_queue,
	.sta_rate_tbl_update = mt7603_sta_rate_tbl_update,
	.release_buffered_frames = mt7603_release_buffered_frames,
	.set_coverage_class = mt7603_set_coverage_class,
};
