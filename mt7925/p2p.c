// SPDX-License-Identifier: BSD-3-Clause-Clear
/* Copyright (C) 2026 MediaTek Inc. */

#include <linux/etherdevice.h>
#include <linux/units.h>
#include <net/mac80211.h>

#include "../mt76.h"
#include "../mt76_connac.h"
#include "../mt792x.h"
#include "mt7925.h"
#include "mac.h"
#include "mcu.h"
#include "p2p.h"

static u8 mt7925_rx_ie_chan(const u8 *ies, int ies_len)
{
	const struct ieee80211_ht_operation *ht_oper;
	const struct element *elem;

	elem = cfg80211_find_elem(WLAN_EID_DS_PARAMS, ies, ies_len);
	if (elem && elem->datalen == 1)
		return elem->data[0];

	elem = cfg80211_find_elem(WLAN_EID_HT_OPERATION, ies, ies_len);
	if (elem && elem->datalen >= sizeof(*ht_oper)) {
		ht_oper = (const void *)elem->data;
		return ht_oper->primary_chan;
	}

	return 0;
}

/* A P2P Probe Response can name a channel other than the one it arrived on */
static bool
mt7925_rx_drop_p2p_probe_resp(struct mt792x_dev *dev, struct sk_buff *skb,
			      struct mt76_rx_status *status)
{
	struct ieee80211_mgmt *mgmt = (struct ieee80211_mgmt *)skb->data;
	int hdr_len = offsetof(struct ieee80211_mgmt, u.probe_resp.variable);
	const u8 *ies;
	int ies_len;
	u8 ie_chan;

	if (!test_bit(MT76_HW_SCANNING, &dev->mphy.state) &&
	    !test_bit(MT76_STATE_ROC, &dev->mphy.state))
		return false;

	if (skb->len <= hdr_len)
		return false;

	ies = mgmt->u.probe_resp.variable;
	ies_len = skb->len - hdr_len;

	if (!cfg80211_find_vendor_ie(WLAN_OUI_WFA, WLAN_OUI_TYPE_WFA_P2P,
				     ies, ies_len))
		return false;

	ie_chan = mt7925_rx_ie_chan(ies, ies_len);
	if (!ie_chan)
		return false;

	if (ieee80211_channel_to_frequency(ie_chan, status->band) ==
	    status->freq)
		return false;

	return true;
}

struct mt7925_rx_chan_iter_data {
	u16 freq;
	u8 band;
	bool p2p_go_active;
	bool chan_match;
};

static void mt7925_rx_chan_iter(void *priv, u8 *mac, struct ieee80211_vif *vif)
{
	struct mt7925_rx_chan_iter_data *data = priv;
	unsigned int link_id;

	for (link_id = 0; link_id < ARRAY_SIZE(vif->link_conf); link_id++) {
		struct ieee80211_chanctx_conf *ctx;
		struct ieee80211_bss_conf *conf;

		conf = rcu_dereference(vif->link_conf[link_id]);
		if (!conf)
			continue;

		ctx = rcu_dereference(conf->chanctx_conf);
		if (!ctx || !ctx->def.chan)
			continue;

		if (vif->type == NL80211_IFTYPE_AP && vif->p2p)
			data->p2p_go_active = true;

		if (ctx->def.chan->band == data->band &&
		    ctx->def.chan->center_freq == data->freq)
			data->chan_match = true;
	}
}

static bool
mt7925_rx_drop_go_probe_req(struct mt792x_dev *dev, struct sk_buff *skb,
			    struct mt76_rx_status *status)
{
	struct ieee80211_mgmt *mgmt = (struct ieee80211_mgmt *)skb->data;
	struct mt7925_rx_chan_iter_data data = {
		.freq = status->freq,
		.band = status->band,
	};

	if (!is_broadcast_ether_addr(mgmt->da))
		return false;

	if (test_bit(MT76_STATE_ROC, &dev->mphy.state))
		return false;

	ieee80211_iterate_active_interfaces_atomic(dev->mt76.hw,
						   IEEE80211_IFACE_ITER_RESUME_ALL,
						   mt7925_rx_chan_iter, &data);

	if (!data.p2p_go_active || data.chan_match)
		return false;

	return true;
}

bool
mt7925_p2p_rx_drop_chan_mismatch(struct mt792x_dev *dev, struct sk_buff *skb)
{
	struct mt76_rx_status *status = (struct mt76_rx_status *)skb->cb;
	struct ieee80211_hdr *hdr = (struct ieee80211_hdr *)skb->data;

	if (skb->len < sizeof(struct ieee80211_hdr_3addr) || !status->freq)
		return false;

	if (ieee80211_is_probe_resp(hdr->frame_control))
		return mt7925_rx_drop_p2p_probe_resp(dev, skb, status);

	if (ieee80211_is_probe_req(hdr->frame_control))
		return mt7925_rx_drop_go_probe_req(dev, skb, status);

	return false;
}
