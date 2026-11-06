/* SPDX-License-Identifier: BSD-3-Clause-Clear */
/* Copyright (C) 2026 MediaTek Inc. */

#ifndef __MT7925_P2P_H
#define __MT7925_P2P_H

#include <net/mac80211.h>

#include "../mt76.h"
#include "../mt792x.h"

#define MT7925_P2P_PROBERESP_LIFE_TIME_MS	100
#define MT7925_P2P_PROBERESP_RETRY_LIMIT	2

/* Leaves room for the TX status before the ROC ends */
#define MT7925_P2P_OFFCHAN_TXS_MARGIN_MS	40

static inline bool mt7925_is_p2p_go_probe_resp(struct sk_buff *skb)
{
	struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);
	struct ieee80211_vif *vif = info->control.vif;
	struct ieee80211_mgmt *mgmt = (struct ieee80211_mgmt *)skb->data;

	if (!vif || ieee80211_vif_type_p2p(vif) != NL80211_IFTYPE_P2P_GO)
		return false;

	if (info->flags & IEEE80211_TX_CTL_HW_80211_ENCAP)
		return false;

	if (skb->len < sizeof(mgmt->frame_control))
		return false;

	return ieee80211_is_probe_resp(mgmt->frame_control);
}

bool mt7925_p2p_rx_drop_chan_mismatch(struct mt792x_dev *dev,
				      struct sk_buff *skb);
void mt7925_p2p_txwi_override(struct mt76_dev *dev, __le32 *txwi,
			      struct sk_buff *skb);

int mt7925_p2p_bmc_add(struct mt792x_dev *dev,
		       struct mt792x_bss_conf *mconf);
void mt7925_p2p_bmc_remove(struct mt792x_dev *dev,
			   struct mt792x_bss_conf *mconf);
bool mt7925_p2p_chanctx_bss_programmed(struct ieee80211_vif *vif,
				       struct ieee80211_bss_conf *link_conf);

#endif /* __MT7925_P2P_H */
