// SPDX-License-Identifier: ISC
/*
 * Copyright (C) 2022 MediaTek Inc.
 */

#include "mt7996.h"
#include "mcu.h"

static bool mt7996_dev_running(struct mt7996_dev *dev)
{
	struct mt7996_phy *phy;

	if (test_bit(MT76_STATE_RUNNING, &dev->mphy.state))
		return true;

	phy = mt7996_phy2(dev);
	if (phy && test_bit(MT76_STATE_RUNNING, &phy->mt76->state))
		return true;

	phy = mt7996_phy3(dev);

	return phy && test_bit(MT76_STATE_RUNNING, &phy->mt76->state);
}

static int mt7996_start(struct ieee80211_hw *hw)
{
	struct mt7996_dev *dev = mt7996_hw_dev(hw);
	struct mt7996_phy *phy = mt7996_hw_phy(hw);
	bool running;
	int ret;

	flush_work(&dev->init_work);

	mutex_lock(&dev->mt76.mutex);

	running = mt7996_dev_running(dev);
	if (!running) {
		ret = mt7996_mcu_set_hdr_trans(dev, true);
		if (ret)
			goto out;
	}

	mt7996_mac_enable_nf(dev, phy->mt76->band_idx);

	ret = mt7996_mcu_set_rts_thresh(phy, 0x92b);
	if (ret)
		goto out;

	ret = mt7996_mcu_set_chan_info(phy, UNI_CHANNEL_RX_PATH);
	if (ret)
		goto out;

	set_bit(MT76_STATE_RUNNING, &phy->mt76->state);

	ieee80211_iterate_interfaces(dev->mt76.hw,
				     IEEE80211_IFACE_ITER_RESUME_ALL,
				     mt7996_mcu_set_pm, dev->mt76.hw);

	ieee80211_queue_delayed_work(hw, &phy->mt76->mac_work,
				     MT7996_WATCHDOG_TIME);

	if (!running)
		mt7996_mac_reset_counters(phy);

out:
	mutex_unlock(&dev->mt76.mutex);

	return ret;
}

static void mt7996_stop(struct ieee80211_hw *hw)
{
	struct mt7996_dev *dev = mt7996_hw_dev(hw);
	struct mt7996_phy *phy = mt7996_hw_phy(hw);

	cancel_delayed_work_sync(&phy->mt76->mac_work);

	mutex_lock(&dev->mt76.mutex);

	clear_bit(MT76_STATE_RUNNING, &phy->mt76->state);

	ieee80211_iterate_interfaces(dev->mt76.hw,
				     IEEE80211_IFACE_ITER_RESUME_ALL,
				     mt7996_mcu_set_pm, dev->mt76.hw);

	mutex_unlock(&dev->mt76.mutex);
}

static inline int get_free_idx(u32 mask, u8 start, u8 end)
{
	return ffs(~mask & GENMASK(end, start));
}

static int get_omac_idx(enum nl80211_iftype type, u64 mask)
{
	int i;

	switch (type) {
	case NL80211_IFTYPE_MESH_POINT:
	case NL80211_IFTYPE_ADHOC:
	case NL80211_IFTYPE_STATION:
		/* prefer hw bssid slot 1-3 */
		i = get_free_idx(mask, HW_BSSID_1, HW_BSSID_3);
		if (i)
			return i - 1;

		if (type != NL80211_IFTYPE_STATION)
			break;

		i = get_free_idx(mask, EXT_BSSID_1, EXT_BSSID_MAX);
		if (i)
			return i - 1;

		if (~mask & BIT(HW_BSSID_0))
			return HW_BSSID_0;

		break;
	case NL80211_IFTYPE_MONITOR:
	case NL80211_IFTYPE_AP:
		/* ap uses hw bssid 0 and ext bssid */
		if (~mask & BIT(HW_BSSID_0))
			return HW_BSSID_0;

		i = get_free_idx(mask, EXT_BSSID_1, EXT_BSSID_MAX);
		if (i)
			return i - 1;

		break;
	default:
		WARN_ON(1);
		break;
	}

	return -1;
}

static void mt7996_init_bitrate_mask(struct ieee80211_vif *vif)
{
	struct mt7996_vif *mvif = (struct mt7996_vif *)vif->drv_priv;
	int i;

	for (i = 0; i < ARRAY_SIZE(mvif->bitrate_mask.control); i++) {
		mvif->bitrate_mask.control[i].gi = NL80211_TXRATE_DEFAULT_GI;
		mvif->bitrate_mask.control[i].he_gi = 0xff;
		mvif->bitrate_mask.control[i].he_ltf = 0xff;
		mvif->bitrate_mask.control[i].legacy = GENMASK(31, 0);
		memset(mvif->bitrate_mask.control[i].ht_mcs, 0xff,
		       sizeof(mvif->bitrate_mask.control[i].ht_mcs));
		memset(mvif->bitrate_mask.control[i].vht_mcs, 0xff,
		       sizeof(mvif->bitrate_mask.control[i].vht_mcs));
		memset(mvif->bitrate_mask.control[i].he_mcs, 0xff,
		       sizeof(mvif->bitrate_mask.control[i].he_mcs));
	}
}

static int mt7996_add_interface(struct ieee80211_hw *hw,
				struct ieee80211_vif *vif)
{
	struct mt7996_vif *mvif = (struct mt7996_vif *)vif->drv_priv;
	struct mt7996_dev *dev = mt7996_hw_dev(hw);
	struct mt7996_phy *phy = mt7996_hw_phy(hw);
	struct mt76_txq *mtxq;
	u8 band_idx = phy->mt76->band_idx;
	int idx, ret = 0;

	mutex_lock(&dev->mt76.mutex);

	if (vif->type == NL80211_IFTYPE_MONITOR &&
	    is_zero_ether_addr(vif->addr))
		phy->monitor_vif = vif;

	mvif->mt76.idx = __ffs64(~dev->mt76.vif_mask);
	if (mvif->mt76.idx >= mt7996_max_interface_num(dev)) {
		ret = -ENOSPC;
		goto out;
	}

	idx = get_omac_idx(vif->type, phy->omac_mask);
	if (idx < 0) {
		ret = -ENOSPC;
		goto out;
	}
	mvif->mt76.omac_idx = idx;
	mvif->phy = phy;
	mvif->mt76.band_idx = band_idx;
	mvif->mt76.wmm_idx = band_idx;

	ret = mt7996_mcu_add_dev_info(phy, vif, true);
	if (ret)
		goto out;

	ret = mt7996_mcu_set_radio_en(phy, true);
	if (ret)
		goto out;

	dev->mt76.vif_mask |= BIT_ULL(mvif->mt76.idx);
	phy->omac_mask |= BIT_ULL(mvif->mt76.omac_idx);

	idx = MT7996_WTBL_RESERVED - mvif->mt76.idx;

	INIT_LIST_HEAD(&mvif->sta.rc_list);
	INIT_LIST_HEAD(&mvif->sta.poll_list);
	mvif->sta.wcid.idx = idx;
	mvif->sta.wcid.phy_idx = band_idx;
	mvif->sta.wcid.hw_key_idx = -1;
	mvif->sta.wcid.tx_info |= MT_WCID_TX_INFO_SET;
	mt76_packet_id_init(&mvif->sta.wcid);

	mt7996_mac_wtbl_update(dev, idx,
			       MT_WTBL_UPDATE_ADM_COUNT_CLEAR);

	if (vif->txq) {
		mtxq = (struct mt76_txq *)vif->txq->drv_priv;
		mtxq->wcid = idx;
	}

	if (vif->type != NL80211_IFTYPE_AP &&
	    (!mvif->mt76.omac_idx || mvif->mt76.omac_idx > 3))
		vif->offload_flags = 0;
	vif->offload_flags |= IEEE80211_OFFLOAD_ENCAP_4ADDR;

	mt7996_init_bitrate_mask(vif);
	memset(&mvif->cap, -1, sizeof(mvif->cap));

	mt7996_mcu_add_bss_info(phy, vif, true);
	mt7996_mcu_add_sta(dev, vif, NULL, true);
	rcu_assign_pointer(dev->mt76.wcid[idx], &mvif->sta.wcid);

out:
	mutex_unlock(&dev->mt76.mutex);

	return ret;
}

static void mt7996_remove_interface(struct ieee80211_hw *hw,
				    struct ieee80211_vif *vif)
{
	struct mt7996_vif *mvif = (struct mt7996_vif *)vif->drv_priv;
	struct mt7996_sta *msta = &mvif->sta;
	struct mt7996_dev *dev = mt7996_hw_dev(hw);
	struct mt7996_phy *phy = mt7996_hw_phy(hw);
	int idx = msta->wcid.idx;

	mt7996_mcu_add_bss_info(phy, vif, false);
	mt7996_mcu_add_sta(dev, vif, NULL, false);

	if (vif == phy->monitor_vif)
		phy->monitor_vif = NULL;

	mt7996_mcu_add_dev_info(phy, vif, false);
	mt7996_mcu_set_radio_en(phy, false);

	rcu_assign_pointer(dev->mt76.wcid[idx], NULL);

	mutex_lock(&dev->mt76.mutex);
	dev->mt76.vif_mask &= ~BIT_ULL(mvif->mt76.idx);
	phy->omac_mask &= ~BIT_ULL(mvif->mt76.omac_idx);
	mutex_unlock(&dev->mt76.mutex);

	spin_lock_bh(&dev->sta_poll_lock);
	if (!list_empty(&msta->poll_list))
		list_del_init(&msta->poll_list);
	spin_unlock_bh(&dev->sta_poll_lock);

	mt76_packet_id_flush(&dev->mt76, &msta->wcid);
}

int mt7996_set_channel(struct mt7996_phy *phy)
{
	struct mt7996_dev *dev = phy->dev;
	int ret;

	cancel_delayed_work_sync(&phy->mt76->mac_work);

	mutex_lock(&dev->mt76.mutex);
	set_bit(MT76_RESET, &phy->mt76->state);

	mt76_set_channel(phy->mt76);

	ret = mt7996_mcu_set_chan_info(phy, UNI_CHANNEL_SWITCH);
	if (ret)
		goto out;

	mt7996_mac_set_timing(phy);
	ret = mt7996_dfs_init_radar_detector(phy);
	mt7996_mac_cca_stats_reset(phy);

	mt7996_mac_reset_counters(phy);
	phy->noise = 0;

out:
	clear_bit(MT76_RESET, &phy->mt76->state);
	mutex_unlock(&dev->mt76.mutex);

	mt76_txq_schedule_all(phy->mt76);

	ieee80211_queue_delayed_work(phy->mt76->hw,
				     &phy->mt76->mac_work,
				     MT7996_WATCHDOG_TIME);

	return ret;
}

static int mt7996_set_key(struct ieee80211_hw *hw, enum set_key_cmd cmd,
			  struct ieee80211_vif *vif, struct ieee80211_sta *sta,
			  struct ieee80211_key_conf *key)
{
	struct mt7996_dev *dev = mt7996_hw_dev(hw);
	struct mt7996_phy *phy = mt7996_hw_phy(hw);
	struct mt7996_vif *mvif = (struct mt7996_vif *)vif->drv_priv;
	struct mt7996_sta *msta = sta ? (struct mt7996_sta *)sta->drv_priv :
				  &mvif->sta;
	struct mt76_wcid *wcid = &msta->wcid;
	u8 *wcid_keyidx = &wcid->hw_key_idx;
	int idx = key->keyidx;
	int err = 0;

	/* The hardware does not support per-STA RX GTK, fallback
	 * to software mode for these.
	 */
	if ((vif->type == NL80211_IFTYPE_ADHOC ||
	     vif->type == NL80211_IFTYPE_MESH_POINT) &&
	    (key->cipher == WLAN_CIPHER_SUITE_TKIP ||
	     key->cipher == WLAN_CIPHER_SUITE_CCMP) &&
	    !(key->flags & IEEE80211_KEY_FLAG_PAIRWISE))
		return -EOPNOTSUPP;

	/* fall back to sw encryption for unsupported ciphers */
	switch (key->cipher) {
	case WLAN_CIPHER_SUITE_AES_CMAC:
		wcid_keyidx = &wcid->hw_key_idx2;
		key->flags |= IEEE80211_KEY_FLAG_GENERATE_MMIE;
		break;
	case WLAN_CIPHER_SUITE_TKIP:
	case WLAN_CIPHER_SUITE_CCMP:
	case WLAN_CIPHER_SUITE_CCMP_256:
	case WLAN_CIPHER_SUITE_GCMP:
	case WLAN_CIPHER_SUITE_GCMP_256:
	case WLAN_CIPHER_SUITE_SMS4:
		break;
	case WLAN_CIPHER_SUITE_WEP40:
	case WLAN_CIPHER_SUITE_WEP104:
	default:
		return -EOPNOTSUPP;
	}

	mutex_lock(&dev->mt76.mutex);

	if (cmd == SET_KEY && !sta && !mvif->mt76.cipher) {
		mvif->mt76.cipher = mt76_connac_mcu_get_cipher(key->cipher);
		mt7996_mcu_add_bss_info(phy, vif, true);
	}

	if (cmd == SET_KEY)
		*wcid_keyidx = idx;
	else if (idx == *wcid_keyidx)
		*wcid_keyidx = -1;
	else
		goto out;

	mt76_wcid_key_setup(&dev->mt76, wcid,
			    cmd == SET_KEY ? key : NULL);

	err = mt7996_mcu_add_key(&dev->mt76, vif, &msta->bip,
				 key, MCU_WMWA_UNI_CMD(STA_REC_UPDATE),
				 &msta->wcid, cmd);
out:
	mutex_unlock(&dev->mt76.mutex);

	return err;
}

static int mt7996_config(struct ieee80211_hw *hw, u32 changed)
{
	struct mt7996_dev *dev = mt7996_hw_dev(hw);
	struct mt7996_phy *phy = mt7996_hw_phy(hw);
	int ret;

	if (changed & IEEE80211_CONF_CHANGE_CHANNEL) {
		ieee80211_stop_queues(hw);
		ret = mt7996_set_channel(phy);
		if (ret)
			return ret;
		ieee80211_wake_queues(hw);
	}

	mutex_lock(&dev->mt76.mutex);

	if (changed & IEEE80211_CONF_CHANGE_MONITOR) {
		bool enabled = !!(hw->conf.flags & IEEE80211_CONF_MONITOR);

		if (!enabled)
			phy->rxfilter |= MT_WF_RFCR_DROP_OTHER_UC;
		else
			phy->rxfilter &= ~MT_WF_RFCR_DROP_OTHER_UC;

		mt76_rmw_field(dev, MT_DMA_DCR0(phy->mt76->band_idx),
			       MT_DMA_DCR0_RXD_G5_EN, enabled);
		mt76_wr(dev, MT_WF_RFCR(phy->mt76->band_idx), phy->rxfilter);
	}

	mutex_unlock(&dev->mt76.mutex);

	return 0;
}

static int
mt7996_conf_tx(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
	       unsigned int link_id, u16 queue,
	       const struct ieee80211_tx_queue_params *params)
{
	struct mt7996_vif *mvif = (struct mt7996_vif *)vif->drv_priv;

	/* no need to update right away, we'll get BSS_CHANGED_QOS */
	queue = mt76_connac_lmac_mapping(queue);
	mvif->queue_params[queue] = *params;

	return 0;
}

static void mt7996_configure_filter(struct ieee80211_hw *hw,
				    unsigned int changed_flags,
				    unsigned int *total_flags,
				    u64 multicast)
{
	struct mt7996_dev *dev = mt7996_hw_dev(hw);
	struct mt7996_phy *phy = mt7996_hw_phy(hw);
	u32 ctl_flags = MT_WF_RFCR1_DROP_ACK |
			MT_WF_RFCR1_DROP_BF_POLL |
			MT_WF_RFCR1_DROP_BA |
			MT_WF_RFCR1_DROP_CFEND |
			MT_WF_RFCR1_DROP_CFACK;
	u32 flags = 0;

#define MT76_FILTER(_flag, _hw) do {					\
		flags |= *total_flags & FIF_##_flag;			\
		phy->rxfilter &= ~(_hw);				\
		phy->rxfilter |= !(flags & FIF_##_flag) * (_hw);	\
	} while (0)

	mutex_lock(&dev->mt76.mutex);

	phy->rxfilter &= ~(MT_WF_RFCR_DROP_OTHER_BSS |
			   MT_WF_RFCR_DROP_OTHER_BEACON |
			   MT_WF_RFCR_DROP_FRAME_REPORT |
			   MT_WF_RFCR_DROP_PROBEREQ |
			   MT_WF_RFCR_DROP_MCAST_FILTERED |
			   MT_WF_RFCR_DROP_MCAST |
			   MT_WF_RFCR_DROP_BCAST |
			   MT_WF_RFCR_DROP_DUPLICATE |
			   MT_WF_RFCR_DROP_A2_BSSID |
			   MT_WF_RFCR_DROP_UNWANTED_CTL |
			   MT_WF_RFCR_DROP_STBC_MULTI);

	MT76_FILTER(OTHER_BSS, MT_WF_RFCR_DROP_OTHER_TIM |
			       MT_WF_RFCR_DROP_A3_MAC |
			       MT_WF_RFCR_DROP_A3_BSSID);

	MT76_FILTER(FCSFAIL, MT_WF_RFCR_DROP_FCSFAIL);

	MT76_FILTER(CONTROL, MT_WF_RFCR_DROP_CTS |
			     MT_WF_RFCR_DROP_RTS |
			     MT_WF_RFCR_DROP_CTL_RSV |
			     MT_WF_RFCR_DROP_NDPA);

	*total_flags = flags;
	mt76_wr(dev, MT_WF_RFCR(phy->mt76->band_idx), phy->rxfilter);

	if (*total_flags & FIF_CONTROL)
		mt76_clear(dev, MT_WF_RFCR1(phy->mt76->band_idx), ctl_flags);
	else
		mt76_set(dev, MT_WF_RFCR1(phy->mt76->band_idx), ctl_flags);

	mutex_unlock(&dev->mt76.mutex);
}

static void
mt7996_update_bss_color(struct ieee80211_hw *hw,
			struct ieee80211_vif *vif,
			struct cfg80211_he_bss_color *bss_color)
{
	struct mt7996_dev *dev = mt7996_hw_dev(hw);

	switch (vif->type) {
	case NL80211_IFTYPE_AP: {
		struct mt7996_vif *mvif = (struct mt7996_vif *)vif->drv_priv;

		if (mvif->mt76.omac_idx > HW_BSSID_MAX)
			return;
		fallthrough;
	}
	case NL80211_IFTYPE_STATION:
		mt7996_mcu_update_bss_color(dev, vif, bss_color);
		break;
	default:
		break;
	}
}

static void mt7996_bss_info_changed(struct ieee80211_hw *hw,
				    struct ieee80211_vif *vif,
				    struct ieee80211_bss_conf *info,
				    u64 changed)
{
	struct mt7996_phy *phy = mt7996_hw_phy(hw);
	struct mt7996_dev *dev = mt7996_hw_dev(hw);

	mutex_lock(&dev->mt76.mutex);

	/* station mode uses BSSID to map the wlan entry to a peer,
	 * and then peer references bss_info_rfch to set bandwidth cap.
	 */
	if (changed & BSS_CHANGED_BSSID &&
	    vif->type == NL80211_IFTYPE_STATION) {
		bool join = !is_zero_ether_addr(info->bssid);

		mt7996_mcu_add_bss_info(phy, vif, join);
		mt7996_mcu_add_sta(dev, vif, NULL, join);
	}

	if (changed & BSS_CHANGED_ASSOC)
		mt7996_mcu_add_bss_info(phy, vif, vif->cfg.assoc);

	if (changed & BSS_CHANGED_ERP_CTS_PROT)
		mt7996_mac_enable_rtscts(dev, vif, info->use_cts_prot);

	if (changed & BSS_CHANGED_ERP_SLOT) {
		int slottime = info->use_short_slot ? 9 : 20;

		if (slottime != phy->slottime) {
			phy->slottime = slottime;
			mt7996_mac_set_timing(phy);
		}
	}

	if (changed & BSS_CHANGED_BEACON_ENABLED && info->enable_beacon) {
		mt7996_mcu_add_bss_info(phy, vif, true);
		mt7996_mcu_add_sta(dev, vif, NULL, true);
	}

	/* ensure that enable txcmd_mode after bss_info */
	if (changed & (BSS_CHANGED_QOS | BSS_CHANGED_BEACON_ENABLED))
		mt7996_mcu_set_tx(dev, vif);

	if (changed & BSS_CHANGED_HE_OBSS_PD)
		mt7996_mcu_add_obss_spr(phy, vif, &info->he_obss_pd);

	if (changed & BSS_CHANGED_HE_BSS_COLOR)
		mt7996_update_bss_color(hw, vif, &info->he_bss_color);

	if (changed & (BSS_CHANGED_BEACON |
		       BSS_CHANGED_BEACON_ENABLED))
		mt7996_mcu_add_beacon(hw, vif, info->enable_beacon);

	if (changed & BSS_CHANGED_UNSOL_BCAST_PROBE_RESP ||
	    changed & BSS_CHANGED_FILS_DISCOVERY)
		mt7996_mcu_beacon_inband_discov(dev, vif, changed);

	mutex_unlock(&dev->mt76.mutex);
}

static void
mt7996_channel_switch_beacon(struct ieee80211_hw *hw,
			     struct ieee80211_vif *vif,
			     struct cfg80211_chan_def *chandef)
{
	struct mt7996_dev *dev = mt7996_hw_dev(hw);

	mutex_lock(&dev->mt76.mutex);
	mt7996_mcu_add_beacon(hw, vif, true);
	mutex_unlock(&dev->mt76.mutex);
}

int mt7996_mac_sta_add(struct mt76_dev *mdev, struct ieee80211_vif *vif,
		       struct ieee80211_sta *sta)
{
	struct mt7996_dev *dev = container_of(mdev, struct mt7996_dev, mt76);
	struct mt7996_sta *msta = (struct mt7996_sta *)sta->drv_priv;
	struct mt7996_vif *mvif = (struct mt7996_vif *)vif->drv_priv;
	u8 band_idx = mvif->phy->mt76->band_idx;
	int ret, idx;

	idx = mt76_wcid_alloc(dev->mt76.wcid_mask, MT7996_WTBL_STA);
	if (idx < 0)
		return -ENOSPC;

	INIT_LIST_HEAD(&msta->rc_list);
	INIT_LIST_HEAD(&msta->poll_list);
	msta->vif = mvif;
	msta->wcid.sta = 1;
	msta->wcid.idx = idx;
	msta->wcid.phy_idx = band_idx;
	msta->wcid.tx_info |= MT_WCID_TX_INFO_SET;
	msta->jiffies = jiffies;

	ewma_avg_signal_init(&msta->avg_ack_signal);

	mt7996_mac_wtbl_update(dev, idx,
			       MT_WTBL_UPDATE_ADM_COUNT_CLEAR);

	ret = mt7996_mcu_add_sta(dev, vif, sta, true);
	if (ret)
		return ret;

	return mt7996_mcu_add_rate_ctrl(dev, vif, sta, false);
}

void mt7996_mac_sta_remove(struct mt76_dev *mdev, struct ieee80211_vif *vif,
			   struct ieee80211_sta *sta)
{
	struct mt7996_dev *dev = container_of(mdev, struct mt7996_dev, mt76);
	struct mt7996_sta *msta = (struct mt7996_sta *)sta->drv_priv;
	int i;

	mt7996_mcu_add_sta(dev, vif, sta, false);

	mt7996_mac_wtbl_update(dev, msta->wcid.idx,
			       MT_WTBL_UPDATE_ADM_COUNT_CLEAR);

	for (i = 0; i < ARRAY_SIZE(msta->twt.flow); i++)
		mt7996_mac_twt_teardown_flow(dev, msta, i);

	spin_lock_bh(&dev->sta_poll_lock);
	if (!list_empty(&msta->poll_list))
		list_del_init(&msta->poll_list);
	if (!list_empty(&msta->rc_list))
		list_del_init(&msta->rc_list);
	spin_unlock_bh(&dev->sta_poll_lock);
}

static void mt7996_tx(struct ieee80211_hw *hw,
		      struct ieee80211_tx_control *control,
		      struct sk_buff *skb)
{
	struct mt7996_dev *dev = mt7996_hw_dev(hw);
	struct mt76_phy *mphy = hw->priv;
	struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);
	struct ieee80211_vif *vif = info->control.vif;
	struct mt76_wcid *wcid = &dev->mt76.global_wcid;

	if (control->sta) {
		struct mt7996_sta *sta;

		sta = (struct mt7996_sta *)control->sta->drv_priv;
		wcid = &sta->wcid;
	}

	if (vif && !control->sta) {
		struct mt7996_vif *mvif;

		mvif = (struct mt7996_vif *)vif->drv_priv;
		wcid = &mvif->sta.wcid;
	}

	mt76_tx(mphy, control->sta, wcid, skb);
}

static int mt7996_set_rts_threshold(struct ieee80211_hw *hw, u32 val)
{
	struct mt7996_phy *phy = mt7996_hw_phy(hw);
	int ret;

	mutex_lock(&phy->dev->mt76.mutex);
	ret = mt7996_mcu_set_rts_thresh(phy, val);
	mutex_unlock(&phy->dev->mt76.mutex);

	return ret;
}

static int
mt7996_ampdu_action(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
		    struct ieee80211_ampdu_params *params)
{
	enum ieee80211_ampdu_mlme_action action = params->action;
	struct mt7996_dev *dev = mt7996_hw_dev(hw);
	struct ieee80211_sta *sta = params->sta;
	struct ieee80211_txq *txq = sta->txq[params->tid];
	struct mt7996_sta *msta = (struct mt7996_sta *)sta->drv_priv;
	u16 tid = params->tid;
	u16 ssn = params->ssn;
	struct mt76_txq *mtxq;
	int ret = 0;

	if (!txq)
		return -EINVAL;

	mtxq = (struct mt76_txq *)txq->drv_priv;

	mutex_lock(&dev->mt76.mutex);
	switch (action) {
	case IEEE80211_AMPDU_RX_START:
		mt76_rx_aggr_start(&dev->mt76, &msta->wcid, tid, ssn,
				   params->buf_size);
		ret = mt7996_mcu_add_rx_ba(dev, params, true);
		break;
	case IEEE80211_AMPDU_RX_STOP:
		mt76_rx_aggr_stop(&dev->mt76, &msta->wcid, tid);
		ret = mt7996_mcu_add_rx_ba(dev, params, false);
		break;
	case IEEE80211_AMPDU_TX_OPERATIONAL:
		mtxq->aggr = true;
		mtxq->send_bar = false;
		ret = mt7996_mcu_add_tx_ba(dev, params, true);
		break;
	case IEEE80211_AMPDU_TX_STOP_FLUSH:
	case IEEE80211_AMPDU_TX_STOP_FLUSH_CONT:
		mtxq->aggr = false;
		clear_bit(tid, &msta->ampdu_state);
		ret = mt7996_mcu_add_tx_ba(dev, params, false);
		break;
	case IEEE80211_AMPDU_TX_START:
		set_bit(tid, &msta->ampdu_state);
		ret = IEEE80211_AMPDU_TX_START_IMMEDIATE;
		break;
	case IEEE80211_AMPDU_TX_STOP_CONT:
		mtxq->aggr = false;
		clear_bit(tid, &msta->ampdu_state);
		ret = mt7996_mcu_add_tx_ba(dev, params, false);
		ieee80211_stop_tx_ba_cb_irqsafe(vif, sta->addr, tid);
		break;
	}
	mutex_unlock(&dev->mt76.mutex);

	return ret;
}

static int
mt7996_sta_add(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
	       struct ieee80211_sta *sta)
{
	return mt76_sta_state(hw, vif, sta, IEEE80211_STA_NOTEXIST,
			      IEEE80211_STA_NONE);
}

static int
mt7996_sta_remove(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
		  struct ieee80211_sta *sta)
{
	return mt76_sta_state(hw, vif, sta, IEEE80211_STA_NONE,
			      IEEE80211_STA_NOTEXIST);
}

static int
mt7996_get_stats(struct ieee80211_hw *hw,
		 struct ieee80211_low_level_stats *stats)
{
	struct mt7996_phy *phy = mt7996_hw_phy(hw);
	struct mt7996_dev *dev = mt7996_hw_dev(hw);
	struct mib_stats *mib = &phy->mib;

	mutex_lock(&dev->mt76.mutex);

	stats->dot11RTSSuccessCount = mib->rts_cnt;
	stats->dot11RTSFailureCount = mib->rts_retries_cnt;
	stats->dot11FCSErrorCount = mib->fcs_err_cnt;
	stats->dot11ACKFailureCount = mib->ack_fail_cnt;

	mutex_unlock(&dev->mt76.mutex);

	return 0;
}

u64 __mt7996_get_tsf(struct ieee80211_hw *hw, struct mt7996_vif *mvif)
{
	struct mt7996_dev *dev = mt7996_hw_dev(hw);
	struct mt7996_phy *phy = mt7996_hw_phy(hw);
	union {
		u64 t64;
		u32 t32[2];
	} tsf;
	u16 n;

	lockdep_assert_held(&dev->mt76.mutex);

	n = mvif->mt76.omac_idx > HW_BSSID_MAX ? HW_BSSID_0
					       : mvif->mt76.omac_idx;
	/* TSF software read */
	mt76_rmw(dev, MT_LPON_TCR(phy->mt76->band_idx, n), MT_LPON_TCR_SW_MODE,
		 MT_LPON_TCR_SW_READ);
	tsf.t32[0] = mt76_rr(dev, MT_LPON_UTTR0(phy->mt76->band_idx));
	tsf.t32[1] = mt76_rr(dev, MT_LPON_UTTR1(phy->mt76->band_idx));

	return tsf.t64;
}

static u64
mt7996_get_tsf(struct ieee80211_hw *hw, struct ieee80211_vif *vif)
{
	struct mt7996_vif *mvif = (struct mt7996_vif *)vif->drv_priv;
	struct mt7996_dev *dev = mt7996_hw_dev(hw);
	u64 ret;

	mutex_lock(&dev->mt76.mutex);
	ret = __mt7996_get_tsf(hw, mvif);
	mutex_unlock(&dev->mt76.mutex);

	return ret;
}

static void
mt7996_set_tsf(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
	       u64 timestamp)
{
	struct mt7996_vif *mvif = (struct mt7996_vif *)vif->drv_priv;
	struct mt7996_dev *dev = mt7996_hw_dev(hw);
	struct mt7996_phy *phy = mt7996_hw_phy(hw);
	union {
		u64 t64;
		u32 t32[2];
	} tsf = { .t64 = timestamp, };
	u16 n;

	mutex_lock(&dev->mt76.mutex);

	n = mvif->mt76.omac_idx > HW_BSSID_MAX ? HW_BSSID_0
					       : mvif->mt76.omac_idx;
	mt76_wr(dev, MT_LPON_UTTR0(phy->mt76->band_idx), tsf.t32[0]);
	mt76_wr(dev, MT_LPON_UTTR1(phy->mt76->band_idx), tsf.t32[1]);
	/* TSF software overwrite */
	mt76_rmw(dev, MT_LPON_TCR(phy->mt76->band_idx, n), MT_LPON_TCR_SW_MODE,
		 MT_LPON_TCR_SW_WRITE);

	mutex_unlock(&dev->mt76.mutex);
}

static void
mt7996_offset_tsf(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
		  s64 timestamp)
{
	struct mt7996_vif *mvif = (struct mt7996_vif *)vif->drv_priv;
	struct mt7996_dev *dev = mt7996_hw_dev(hw);
	struct mt7996_phy *phy = mt7996_hw_phy(hw);
	union {
		u64 t64;
		u32 t32[2];
	} tsf = { .t64 = timestamp, };
	u16 n;

	mutex_lock(&dev->mt76.mutex);

	n = mvif->mt76.omac_idx > HW_BSSID_MAX ? HW_BSSID_0
					       : mvif->mt76.omac_idx;
	mt76_wr(dev, MT_LPON_UTTR0(phy->mt76->band_idx), tsf.t32[0]);
	mt76_wr(dev, MT_LPON_UTTR1(phy->mt76->band_idx), tsf.t32[1]);
	/* TSF software adjust*/
	mt76_rmw(dev, MT_LPON_TCR(phy->mt76->band_idx, n), MT_LPON_TCR_SW_MODE,
		 MT_LPON_TCR_SW_ADJUST);

	mutex_unlock(&dev->mt76.mutex);
}

static void
mt7996_set_coverage_class(struct ieee80211_hw *hw, s16 coverage_class)
{
	struct mt7996_phy *phy = mt7996_hw_phy(hw);
	struct mt7996_dev *dev = phy->dev;

	mutex_lock(&dev->mt76.mutex);
	phy->coverage_class = max_t(s16, coverage_class, 0);
	mt7996_mac_set_timing(phy);
	mutex_unlock(&dev->mt76.mutex);
}

static int
mt7996_set_antenna(struct ieee80211_hw *hw, u32 tx_ant, u32 rx_ant)
{
	struct mt7996_dev *dev = mt7996_hw_dev(hw);
	struct mt7996_phy *phy = mt7996_hw_phy(hw);
	int max_nss = hweight8(hw->wiphy->available_antennas_tx);
	u8 band_idx = phy->mt76->band_idx, shift = dev->chainshift[band_idx];

	if (!tx_ant || tx_ant != rx_ant || ffs(tx_ant) > max_nss)
		return -EINVAL;

	if ((BIT(hweight8(tx_ant)) - 1) != tx_ant)
		tx_ant = BIT(ffs(tx_ant) - 1) - 1;

	mutex_lock(&dev->mt76.mutex);

	phy->mt76->antenna_mask = tx_ant;

	/* restore to the origin chainmask which might have auxiliary path */
	if (hweight8(tx_ant) == max_nss && band_idx < MT_BAND2)
		phy->mt76->chainmask = ((dev->chainmask >> shift) &
					(BIT(dev->chainshift[band_idx + 1] - shift) - 1)) << shift;
	else if (hweight8(tx_ant) == max_nss)
		phy->mt76->chainmask = (dev->chainmask >> shift) << shift;
	else
		phy->mt76->chainmask = tx_ant << shift;

	mt76_set_stream_caps(phy->mt76, true);
	mt7996_set_stream_vht_txbf_caps(phy);
	mt7996_set_stream_he_eht_caps(phy);

	mutex_unlock(&dev->mt76.mutex);

	return 0;
}

static void mt7996_sta_statistics(struct ieee80211_hw *hw,
				  struct ieee80211_vif *vif,
				  struct ieee80211_sta *sta,
				  struct station_info *sinfo)
{
	struct mt7996_sta *msta = (struct mt7996_sta *)sta->drv_priv;
	struct rate_info *txrate = &msta->wcid.rate;

	if (!txrate->legacy && !txrate->flags)
		return;

	if (txrate->legacy) {
		sinfo->txrate.legacy = txrate->legacy;
	} else {
		sinfo->txrate.mcs = txrate->mcs;
		sinfo->txrate.nss = txrate->nss;
		sinfo->txrate.bw = txrate->bw;
		sinfo->txrate.he_gi = txrate->he_gi;
		sinfo->txrate.he_dcm = txrate->he_dcm;
		sinfo->txrate.he_ru_alloc = txrate->he_ru_alloc;
	}
	sinfo->txrate.flags = txrate->flags;
	sinfo->filled |= BIT_ULL(NL80211_STA_INFO_TX_BITRATE);

	sinfo->ack_signal = (s8)msta->ack_signal;
	sinfo->filled |= BIT_ULL(NL80211_STA_INFO_ACK_SIGNAL);

	sinfo->avg_ack_signal = -(s8)ewma_avg_signal_read(&msta->avg_ack_signal);
	sinfo->filled |= BIT_ULL(NL80211_STA_INFO_ACK_SIGNAL_AVG);
}

static void mt7996_sta_rc_work(void *data, struct ieee80211_sta *sta)
{
	struct mt7996_sta *msta = (struct mt7996_sta *)sta->drv_priv;
	struct mt7996_dev *dev = msta->vif->phy->dev;
	u32 *changed = data;

	spin_lock_bh(&dev->sta_poll_lock);
	msta->changed |= *changed;
	if (list_empty(&msta->rc_list))
		list_add_tail(&msta->rc_list, &dev->sta_rc_list);
	spin_unlock_bh(&dev->sta_poll_lock);
}

static void mt7996_sta_rc_update(struct ieee80211_hw *hw,
				 struct ieee80211_vif *vif,
				 struct ieee80211_sta *sta,
				 u32 changed)
{
	struct mt7996_phy *phy = mt7996_hw_phy(hw);
	struct mt7996_dev *dev = phy->dev;

	mt7996_sta_rc_work(&changed, sta);
	ieee80211_queue_work(hw, &dev->rc_work);
}

static int
mt7996_set_bitrate_mask(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
			const struct cfg80211_bitrate_mask *mask)
{
	struct mt7996_vif *mvif = (struct mt7996_vif *)vif->drv_priv;
	struct mt7996_phy *phy = mt7996_hw_phy(hw);
	struct mt7996_dev *dev = phy->dev;
	u32 changed = IEEE80211_RC_SUPP_RATES_CHANGED;

	mvif->bitrate_mask = *mask;

	/* if multiple rates across different preambles are given we can
	 * reconfigure this info with all peers using sta_rec command with
	 * the below exception cases.
	 * - single rate : if a rate is passed along with different preambles,
	 * we select the highest one as fixed rate. i.e VHT MCS for VHT peers.
	 * - multiple rates: if it's not in range format i.e 0-{7,8,9} for VHT
	 * then multiple MCS setting (MCS 4,5,6) is not supported.
	 */
	ieee80211_iterate_stations_atomic(hw, mt7996_sta_rc_work, &changed);
	ieee80211_queue_work(hw, &dev->rc_work);

	return 0;
}

static void mt7996_sta_set_4addr(struct ieee80211_hw *hw,
				 struct ieee80211_vif *vif,
				 struct ieee80211_sta *sta,
				 bool enabled)
{
	struct mt7996_dev *dev = mt7996_hw_dev(hw);
	struct mt7996_sta *msta = (struct mt7996_sta *)sta->drv_priv;

	if (enabled)
		set_bit(MT_WCID_FLAG_4ADDR, &msta->wcid.flags);
	else
		clear_bit(MT_WCID_FLAG_4ADDR, &msta->wcid.flags);

	mt7996_mcu_wtbl_update_hdr_trans(dev, vif, sta);
}

static void mt7996_sta_set_decap_offload(struct ieee80211_hw *hw,
					 struct ieee80211_vif *vif,
					 struct ieee80211_sta *sta,
					 bool enabled)
{
	struct mt7996_dev *dev = mt7996_hw_dev(hw);
	struct mt7996_sta *msta = (struct mt7996_sta *)sta->drv_priv;

	if (enabled)
		set_bit(MT_WCID_FLAG_HDR_TRANS, &msta->wcid.flags);
	else
		clear_bit(MT_WCID_FLAG_HDR_TRANS, &msta->wcid.flags);

	mt7996_mcu_wtbl_update_hdr_trans(dev, vif, sta);
}

static const char mt7996_gstrings_stats[][ETH_GSTRING_LEN] = {
	"tx_ampdu_cnt",
	"tx_stop_q_empty_cnt",
	"tx_mpdu_attempts",
	"tx_mpdu_success",
	"tx_rwp_fail_cnt",
	"tx_rwp_need_cnt",
	"tx_pkt_ebf_cnt",
	"tx_pkt_ibf_cnt",
	"tx_ampdu_len:0-1",
	"tx_ampdu_len:2-10",
	"tx_ampdu_len:11-19",
	"tx_ampdu_len:20-28",
	"tx_ampdu_len:29-37",
	"tx_ampdu_len:38-46",
	"tx_ampdu_len:47-55",
	"tx_ampdu_len:56-79",
	"tx_ampdu_len:80-103",
	"tx_ampdu_len:104-127",
	"tx_ampdu_len:128-151",
	"tx_ampdu_len:152-175",
	"tx_ampdu_len:176-199",
	"tx_ampdu_len:200-223",
	"tx_ampdu_len:224-247",
	"ba_miss_count",
	"tx_beamformer_ppdu_iBF",
	"tx_beamformer_ppdu_eBF",
	"tx_beamformer_rx_feedback_all",
	"tx_beamformer_rx_feedback_he",
	"tx_beamformer_rx_feedback_vht",
	"tx_beamformer_rx_feedback_ht",
	"tx_beamformer_rx_feedback_bw", /* zero based idx: 20, 40, 80, 160 */
	"tx_beamformer_rx_feedback_nc",
	"tx_beamformer_rx_feedback_nr",
	"tx_beamformee_ok_feedback_pkts",
	"tx_beamformee_feedback_trig",
	"tx_mu_beamforming",
	"tx_mu_mpdu",
	"tx_mu_successful_mpdu",
	"tx_su_successful_mpdu",
	"tx_msdu_pack_1",
	"tx_msdu_pack_2",
	"tx_msdu_pack_3",
	"tx_msdu_pack_4",
	"tx_msdu_pack_5",
	"tx_msdu_pack_6",
	"tx_msdu_pack_7",
	"tx_msdu_pack_8",

	/* rx counters */
	"rx_fifo_full_cnt",
	"rx_mpdu_cnt",
	"channel_idle_cnt",
	"rx_vector_mismatch_cnt",
	"rx_delimiter_fail_cnt",
	"rx_len_mismatch_cnt",
	"rx_ampdu_cnt",
	"rx_ampdu_bytes_cnt",
	"rx_ampdu_valid_subframe_cnt",
	"rx_ampdu_valid_subframe_b_cnt",
	"rx_pfdrop_cnt",
	"rx_vec_queue_overflow_drop_cnt",
	"rx_ba_cnt",

	/* per vif counters */
	"v_tx_mode_cck",
	"v_tx_mode_ofdm",
	"v_tx_mode_ht",
	"v_tx_mode_ht_gf",
	"v_tx_mode_vht",
	"v_tx_mode_he_su",
	"v_tx_mode_he_ext_su",
	"v_tx_mode_he_tb",
	"v_tx_mode_he_mu",
	"v_tx_mode_eht_su",
	"v_tx_mode_eht_trig",
	"v_tx_mode_eht_mu",
	"v_tx_bw_20",
	"v_tx_bw_40",
	"v_tx_bw_80",
	"v_tx_bw_160",
	"v_tx_bw_320",
	"v_tx_mcs_0",
	"v_tx_mcs_1",
	"v_tx_mcs_2",
	"v_tx_mcs_3",
	"v_tx_mcs_4",
	"v_tx_mcs_5",
	"v_tx_mcs_6",
	"v_tx_mcs_7",
	"v_tx_mcs_8",
	"v_tx_mcs_9",
	"v_tx_mcs_10",
	"v_tx_mcs_11",
	"v_tx_mcs_12",
	"v_tx_mcs_13",
};

#define MT7996_SSTATS_LEN ARRAY_SIZE(mt7996_gstrings_stats)

/* Ethtool related API */
static
void mt7996_get_et_strings(struct ieee80211_hw *hw,
			   struct ieee80211_vif *vif,
			   u32 sset, u8 *data)
{
	if (sset == ETH_SS_STATS)
		memcpy(data, *mt7996_gstrings_stats,
		       sizeof(mt7996_gstrings_stats));
}

static
int mt7996_get_et_sset_count(struct ieee80211_hw *hw,
			     struct ieee80211_vif *vif, int sset)
{
	if (sset == ETH_SS_STATS)
		return MT7996_SSTATS_LEN;

	return 0;
}

static void mt7996_ethtool_worker(void *wi_data, struct ieee80211_sta *sta)
{
	struct mt76_ethtool_worker_info *wi = wi_data;
	struct mt7996_sta *msta = (struct mt7996_sta *)sta->drv_priv;

	if (msta->vif->mt76.idx != wi->idx)
		return;

	mt76_ethtool_worker(wi, &msta->stats, true);
}

static
void mt7996_get_et_stats(struct ieee80211_hw *hw,
			 struct ieee80211_vif *vif,
			 struct ethtool_stats *stats, u64 *data)
{
	struct mt7996_dev *dev = mt7996_hw_dev(hw);
	struct mt7996_phy *phy = mt7996_hw_phy(hw);
	struct mt7996_vif *mvif = (struct mt7996_vif *)vif->drv_priv;
	struct mt76_ethtool_worker_info wi = {
		.data = data,
		.idx = mvif->mt76.idx,
	};
	struct mib_stats *mib = &phy->mib;
	/* See mt7996_ampdu_stat_read_phy, etc */
	int i, ei = 0;

	mutex_lock(&dev->mt76.mutex);

	mt7996_mac_update_stats(phy);

	data[ei++] = mib->tx_ampdu_cnt;
	data[ei++] = mib->tx_stop_q_empty_cnt;
	data[ei++] = mib->tx_mpdu_attempts_cnt;
	data[ei++] = mib->tx_mpdu_success_cnt;
	data[ei++] = mib->tx_rwp_fail_cnt;
	data[ei++] = mib->tx_rwp_need_cnt;
	data[ei++] = mib->tx_bf_ebf_ppdu_cnt;
	data[ei++] = mib->tx_bf_ibf_ppdu_cnt;

	/* Tx ampdu stat */
	for (i = 0; i < 15 /*ARRAY_SIZE(bound)*/; i++)
		data[ei++] = phy->mt76->aggr_stats[i];
	data[ei++] = phy->mib.ba_miss_cnt;

	/* Tx Beamformer monitor */
	data[ei++] = mib->tx_bf_ibf_ppdu_cnt;
	data[ei++] = mib->tx_bf_ebf_ppdu_cnt;

	/* Tx Beamformer Rx feedback monitor */
	data[ei++] = mib->tx_bf_rx_fb_all_cnt;
	data[ei++] = mib->tx_bf_rx_fb_he_cnt;
	data[ei++] = mib->tx_bf_rx_fb_vht_cnt;
	data[ei++] = mib->tx_bf_rx_fb_ht_cnt;

	data[ei++] = mib->tx_bf_rx_fb_bw;
	data[ei++] = mib->tx_bf_rx_fb_nc_cnt;
	data[ei++] = mib->tx_bf_rx_fb_nr_cnt;

	/* Tx Beamformee Rx NDPA & Tx feedback report */
	data[ei++] = mib->tx_bf_fb_cpl_cnt;
	data[ei++] = mib->tx_bf_fb_trig_cnt;

	/* Tx SU & MU counters */
	data[ei++] = mib->tx_mu_bf_cnt;
	data[ei++] = mib->tx_mu_mpdu_cnt;
	data[ei++] = mib->tx_mu_acked_mpdu_cnt;
	data[ei++] = mib->tx_su_acked_mpdu_cnt;

	/* Tx amsdu info (pack-count histogram) */
	for (i = 0; i < ARRAY_SIZE(mib->tx_amsdu); i++)
		data[ei++] = mib->tx_amsdu[i];

	/* rx counters */
	data[ei++] = mib->rx_fifo_full_cnt;
	data[ei++] = mib->rx_mpdu_cnt;
	data[ei++] = mib->channel_idle_cnt;
	data[ei++] = mib->rx_vector_mismatch_cnt;
	data[ei++] = mib->rx_delimiter_fail_cnt;
	data[ei++] = mib->rx_len_mismatch_cnt;
	data[ei++] = mib->rx_ampdu_cnt;
	data[ei++] = mib->rx_ampdu_bytes_cnt;
	data[ei++] = mib->rx_ampdu_valid_subframe_cnt;
	data[ei++] = mib->rx_ampdu_valid_subframe_bytes_cnt;
	data[ei++] = mib->rx_pfdrop_cnt;
	data[ei++] = mib->rx_vec_queue_overflow_drop_cnt;
	data[ei++] = mib->rx_ba_cnt;

	/* Add values for all stations owned by this vif */
	wi.initial_stat_idx = ei;
	ieee80211_iterate_stations_atomic(hw, mt7996_ethtool_worker, &wi);

	mutex_unlock(&dev->mt76.mutex);

	if (wi.sta_count == 0)
		return;

	ei += wi.worker_stat_count;
	if (ei != MT7996_SSTATS_LEN)
		dev_err(dev->mt76.dev, "ei: %d  MT7996_SSTATS_LEN: %d",
			ei, (int)MT7996_SSTATS_LEN);
}

static void
mt7996_twt_teardown_request(struct ieee80211_hw *hw,
			    struct ieee80211_sta *sta,
			    u8 flowid)
{
	struct mt7996_sta *msta = (struct mt7996_sta *)sta->drv_priv;
	struct mt7996_dev *dev = mt7996_hw_dev(hw);

	mutex_lock(&dev->mt76.mutex);
	mt7996_mac_twt_teardown_flow(dev, msta, flowid);
	mutex_unlock(&dev->mt76.mutex);
}

static int
mt7996_set_radar_background(struct ieee80211_hw *hw,
			    struct cfg80211_chan_def *chandef)
{
	struct mt7996_phy *phy = mt7996_hw_phy(hw);
	struct mt7996_dev *dev = phy->dev;
	int ret = -EINVAL;
	bool running;

	mutex_lock(&dev->mt76.mutex);

	if (dev->mt76.region == NL80211_DFS_UNSET)
		goto out;

	if (dev->rdd2_phy && dev->rdd2_phy != phy) {
		/* rdd2 is already locked */
		ret = -EBUSY;
		goto out;
	}

	/* rdd2 already configured on a radar channel */
	running = dev->rdd2_phy &&
		  cfg80211_chandef_valid(&dev->rdd2_chandef) &&
		  !!(dev->rdd2_chandef.chan->flags & IEEE80211_CHAN_RADAR);

	if (!chandef || running ||
	    !(chandef->chan->flags & IEEE80211_CHAN_RADAR)) {
		ret = mt7996_mcu_rdd_background_enable(phy, NULL);
		if (ret)
			goto out;

		if (!running)
			goto update_phy;
	}

	ret = mt7996_mcu_rdd_background_enable(phy, chandef);
	if (ret)
		goto out;

update_phy:
	dev->rdd2_phy = chandef ? phy : NULL;
	if (chandef)
		dev->rdd2_chandef = *chandef;
out:
	mutex_unlock(&dev->mt76.mutex);

	return ret;
}

const struct ieee80211_ops mt7996_ops = {
	.tx = mt7996_tx,
	.start = mt7996_start,
	.stop = mt7996_stop,
	.add_interface = mt7996_add_interface,
	.remove_interface = mt7996_remove_interface,
	.config = mt7996_config,
	.conf_tx = mt7996_conf_tx,
	.configure_filter = mt7996_configure_filter,
	.bss_info_changed = mt7996_bss_info_changed,
	.sta_add = mt7996_sta_add,
	.sta_remove = mt7996_sta_remove,
	.sta_pre_rcu_remove = mt76_sta_pre_rcu_remove,
	.sta_rc_update = mt7996_sta_rc_update,
	.set_key = mt7996_set_key,
	.ampdu_action = mt7996_ampdu_action,
	.set_rts_threshold = mt7996_set_rts_threshold,
	.wake_tx_queue = mt76_wake_tx_queue,
	.sw_scan_start = mt76_sw_scan,
	.sw_scan_complete = mt76_sw_scan_complete,
	.release_buffered_frames = mt76_release_buffered_frames,
	.get_txpower = mt76_get_txpower,
	.channel_switch_beacon = mt7996_channel_switch_beacon,
	.get_stats = mt7996_get_stats,
	.get_et_sset_count = mt7996_get_et_sset_count,
	.get_et_stats = mt7996_get_et_stats,
	.get_et_strings = mt7996_get_et_strings,
	.get_tsf = mt7996_get_tsf,
	.set_tsf = mt7996_set_tsf,
	.offset_tsf = mt7996_offset_tsf,
	.get_survey = mt76_get_survey,
	.get_antenna = mt76_get_antenna,
	.set_antenna = mt7996_set_antenna,
	.set_bitrate_mask = mt7996_set_bitrate_mask,
	.set_coverage_class = mt7996_set_coverage_class,
	.sta_statistics = mt7996_sta_statistics,
	.sta_set_4addr = mt7996_sta_set_4addr,
	.sta_set_decap_offload = mt7996_sta_set_decap_offload,
	.add_twt_setup = mt7996_mac_add_twt_setup,
	.twt_teardown_request = mt7996_twt_teardown_request,
#ifdef CONFIG_MAC80211_DEBUGFS
	.sta_add_debugfs = mt7996_sta_add_debugfs,
#endif
	.set_radar_background = mt7996_set_radar_background,
};
