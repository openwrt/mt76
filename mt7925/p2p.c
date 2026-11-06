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

/* DW2 MAX_TX_TIME counts 64 TU units, and 0 means no limit. The host can only
 * program bits [6:0], as the HW replaces the field once the packet is queued.
 */
#define MT7925_TXD_MAX_TX_TIME_LIMIT	0x7f

static u32 mt7925_mac_txd_life_time(u32 ms)
{
	u32 unit;

	if (!ms)
		return 0;

	unit = (ms * USEC_PER_MSEC / 1024) >> 6;

	if (unit > MT7925_TXD_MAX_TX_TIME_LIMIT)
		return MT7925_TXD_MAX_TX_TIME_LIMIT;

	return unit ? unit : 1;
}

/* UNI_ROC_ABORT does not drop a frame from the PLE ALTX queue. A frame that goes
 * on air after its ROC ended gets a TX status with a cookie that wpa_supplicant
 * no longer knows, which stalls its radio work queue.
 */
static void
mt7925_mac_txwi_p2p_offchan_life_time(struct mt76_dev *mdev, __le32 *txwi,
				      struct sk_buff *skb)
{
	struct mt792x_dev *dev = container_of(mdev, struct mt792x_dev, mt76);
	struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);
	struct ieee80211_vif *vif = info->control.vif;
	struct mt792x_phy *phy = &dev->phy;
	struct ieee80211_hdr *hdr;
	unsigned long expires, now = jiffies;
	u32 life_time, dwell_ms = 0;

	if (!(info->flags & IEEE80211_TX_CTL_TX_OFFCHAN))
		return;

	if (!vif)
		return;

	switch (ieee80211_vif_type_p2p(vif)) {
	case NL80211_IFTYPE_P2P_DEVICE:
	case NL80211_IFTYPE_P2P_GO:
	case NL80211_IFTYPE_P2P_CLIENT:
		break;
	default:
		return;
	}

	if (info->flags & IEEE80211_TX_CTL_HW_80211_ENCAP)
		return;

	if (skb->len < sizeof(hdr->frame_control))
		return;

	hdr = (struct ieee80211_hdr *)skb->data;
	if (!ieee80211_is_mgmt(hdr->frame_control))
		return;

	if (test_bit(MT76_STATE_ROC, &phy->mt76->state) &&
	    timer_pending(&phy->roc_timer)) {
		expires = phy->roc_timer.expires;
		if (time_before(now, expires))
			dwell_ms = jiffies_to_msecs(expires - now);
	}

	if (dwell_ms > MT7925_P2P_OFFCHAN_TXS_MARGIN_MS)
		dwell_ms -= MT7925_P2P_OFFCHAN_TXS_MARGIN_MS;

	life_time = mt7925_mac_txd_life_time(max(dwell_ms, 1U));

	txwi[2] &= ~cpu_to_le32(MT_TXD2_MAX_TX_TIME);
	txwi[2] |= cpu_to_le32(FIELD_PREP(MT_TXD2_MAX_TX_TIME, life_time));
}

#define MT7925_TXD_PROBERESP_TIMESTAMP_IDX	0

static void
mt7925_mac_txwi_p2p_go_probe_resp(struct mt76_dev *dev, __le32 *txwi,
				  struct sk_buff *skb)
{
	u32 life_time, val;

	if (!mt7925_is_p2p_go_probe_resp(skb))
		return;

	life_time = mt7925_mac_txd_life_time(MT7925_P2P_PROBERESP_LIFE_TIME_MS);

	txwi[2] &= ~cpu_to_le32(MT_TXD2_MAX_TX_TIME);
	txwi[2] |= cpu_to_le32(FIELD_PREP(MT_TXD2_MAX_TX_TIME, life_time));

	txwi[3] &= ~cpu_to_le32(MT_TXD3_REM_TX_COUNT);
	txwi[3] |= cpu_to_le32(FIELD_PREP(MT_TXD3_REM_TX_COUNT,
					  MT7925_P2P_PROBERESP_RETRY_LIMIT));

	if (is_mt7928(dev)) {
		txwi[6] &= ~cpu_to_le32(MT_TXD6_MSDU_CNT_V2 |
					MT_TXD6_TIMESTAMP_OFS_EN_V2 |
					MT_TXD6_TIMESTAMP_OFS_IDX_V2);
		val = FIELD_PREP(MT_TXD6_TIMESTAMP_OFS_EN_V2, 1) |
		      FIELD_PREP(MT_TXD6_TIMESTAMP_OFS_IDX_V2,
				 MT7925_TXD_PROBERESP_TIMESTAMP_IDX);
	} else {
		txwi[6] &= ~cpu_to_le32(MT_TXD6_TIMESTAMP_OFS_EN |
					MT_TXD6_TIMESTAMP_OFS_IDX);
		val = MT_TXD6_TIMESTAMP_OFS_EN |
		      FIELD_PREP(MT_TXD6_TIMESTAMP_OFS_IDX,
				 MT7925_TXD_PROBERESP_TIMESTAMP_IDX);
	}
	txwi[6] |= cpu_to_le32(val | MT_TXD6_VTA);
}

void mt7925_p2p_txwi_override(struct mt76_dev *dev, __le32 *txwi,
			      struct sk_buff *skb)
{
	mt7925_mac_txwi_p2p_offchan_life_time(dev, txwi, skb);
	mt7925_mac_txwi_p2p_go_probe_resp(dev, txwi, skb);
}

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

/* The firmware sets up the BMC entry from bmc_tx_wlan_idx in BSS_INFO_BASIC */
int mt7925_p2p_bmc_add(struct mt792x_dev *dev,
		       struct mt792x_bss_conf *mconf)
{
	struct mt792x_link_sta *mbmc = &mconf->bmc;
	int idx;

	if (mconf->bmc_valid) {
		mbmc->wcid.link_id = mconf->link_id;
		return 0;
	}

	idx = mt76_wcid_alloc(dev->mt76.wcid_mask, MT792x_WTBL_STA - 1);
	if (idx < 0)
		return -ENOSPC;

	memset(mbmc, 0, sizeof(*mbmc));
	INIT_LIST_HEAD(&mbmc->wcid.poll_list);
	mbmc->wcid.idx = idx;
	mbmc->wcid.phy_idx = 0;
	mbmc->wcid.hw_key_idx = -1;
	mbmc->wcid.hw_key_idx2 = -1;
	mbmc->wcid.tx_info |= MT_WCID_TX_INFO_SET;
	mbmc->wcid.link_id = mconf->link_id;
	mt76_wcid_init(&mbmc->wcid, 0);

	rcu_assign_pointer(dev->mt76.wcid[idx], &mbmc->wcid);
	mt7925_mac_wtbl_update(dev, idx, MT_WTBL_UPDATE_ADM_COUNT_CLEAR);
	mconf->mt76.bmc_wcid = idx;
	mconf->bmc_valid = true;

	return 0;
}

void mt7925_p2p_bmc_remove(struct mt792x_dev *dev,
			   struct mt792x_bss_conf *mconf)
{
	struct mt76_wcid *wcid = &mconf->bmc.wcid;

	if (!mconf->bmc_valid)
		return;

	mconf->bmc_valid = false;
	mconf->mt76.bmc_wcid = 0;

	mt76_connac_free_pending_tx_skbs(&dev->pm, wcid);
	rcu_assign_pointer(dev->mt76.wcid[wcid->idx], NULL);

	spin_lock_bh(&dev->mt76.sta_poll_lock);
	if (!list_empty(&wcid->poll_list))
		list_del_init(&wcid->poll_list);
	spin_unlock_bh(&dev->mt76.sta_poll_lock);

	mt76_wcid_cleanup(&dev->mt76, wcid);
	mt76_wcid_mask_clear(dev->mt76.wcid_mask, wcid->idx);
}

/* Before the grant, an RLM-only update makes CNM guess the band. Both roles send
 * a full BASIC+RLM update after the grant.
 */
bool mt7925_p2p_chanctx_bss_programmed(struct ieee80211_vif *vif,
				       struct ieee80211_bss_conf *link_conf)
{
	if (!vif->p2p || !link_conf)
		return true;

	switch (vif->type) {
	case NL80211_IFTYPE_AP:
		return link_conf->enable_beacon;
	case NL80211_IFTYPE_STATION:
		return vif->cfg.assoc;
	default:
		return true;
	}
}

/* Userspace looks up the station by the address of a received frame, which is
 * the MLD address on an MLO link.
 */
static void
mt7925_mcu_sap_aging_timeout(struct mt792x_dev *dev, u16 wlan_idx)
{
	struct {
		struct ieee80211_hdr_3addr hdr;
		__le16 reason_code;
	} deauth = {
		.hdr.frame_control = cpu_to_le16(IEEE80211_FTYPE_MGMT |
						 IEEE80211_STYPE_DEAUTH),
		.reason_code = cpu_to_le16(WLAN_REASON_DISASSOC_DUE_TO_INACTIVITY),
	};
	struct ieee80211_chanctx_conf *ctx;
	struct mt792x_bss_conf *mconf;
	struct mt792x_link_sta *mlink;
	struct ieee80211_vif *vif;
	struct ieee80211_sta *sta;
	struct mt792x_vif *mvif;
	struct mt792x_sta *msta;
	struct mt76_wcid *wcid;
	struct wireless_dev *wdev;
	int freq = 0;
	u8 link_id;

	rcu_read_lock();

	wcid = mt76_wcid_ptr(dev, wlan_idx);
	if (!wcid || !wcid->sta)
		goto unlock;

	mlink = container_of(wcid, struct mt792x_link_sta, wcid);
	msta = mlink->sta;
	if (!msta || !msta->vif)
		goto unlock;

	vif = container_of((void *)msta->vif, struct ieee80211_vif, drv_priv);
	if (vif->type != NL80211_IFTYPE_AP)
		goto unlock;

	mvif = (struct mt792x_vif *)vif->drv_priv;
	sta = container_of((void *)msta, struct ieee80211_sta, drv_priv);
	link_id = wcid->link_id;

	ether_addr_copy(deauth.hdr.addr1, vif->addr);
	ether_addr_copy(deauth.hdr.addr2, sta->addr);
	ether_addr_copy(deauth.hdr.addr3, vif->addr);

	mconf = &mvif->bss_conf;

	if (ieee80211_vif_is_mld(vif) && link_id < IEEE80211_LINK_UNSPECIFIED) {
		struct mt792x_bss_conf *link_mconf;

		link_mconf = rcu_dereference(mvif->link_conf[link_id]);
		if (link_mconf)
			mconf = link_mconf;
	}

	ctx = mconf->mt76.ctx;
	if (ctx && ctx->def.chan)
		freq = ctx->def.chan->center_freq;
	else if (dev->mphy.chandef.chan)
		freq = dev->mphy.chandef.chan->center_freq;

	rcu_read_unlock();

	wdev = ieee80211_vif_to_wdev(vif);
	if (!wdev)
		return;

	cfg80211_rx_mgmt_khz(wdev, MHZ_TO_KHZ(freq), 0, (const u8 *)&deauth,
			     sizeof(deauth), 0);

	return;

unlock:
	rcu_read_unlock();
}

void mt7925_p2p_mcu_sap_event(struct mt792x_dev *dev, struct sk_buff *skb)
{
	struct mt7925_uni_sap_event *event;
	u32 tlv_len;
	u8 *tlv;

	skb_pull(skb, sizeof(struct mt7925_mcu_rxd));
	if (skb->len < sizeof(*event))
		return;

	event = (struct mt7925_uni_sap_event *)skb->data;
	tlv = event->tlv;
	tlv_len = skb->len - sizeof(*event);

	while (tlv_len >= sizeof(struct tlv)) {
		struct tlv *hdr = (struct tlv *)tlv;
		u16 tag = le16_to_cpu(hdr->tag);
		u16 len = le16_to_cpu(hdr->len);

		if (len < sizeof(*hdr) || len > tlv_len)
			break;

		if (tag == MT7925_SAP_TAG_AGING_TIMEOUT &&
		    len >= sizeof(struct mt7925_sap_aging_timeout_tlv)) {
			struct mt7925_sap_aging_timeout_tlv *aging =
				(struct mt7925_sap_aging_timeout_tlv *)tlv;

			mt7925_mcu_sap_aging_timeout(dev,
						     le16_to_cpu(aging->wlan_idx));
		}

		tlv += len;
		tlv_len -= len;
	}
}
