// SPDX-License-Identifier: BSD-3-Clause-Clear
/*
 * Copyright (C) 2022 MediaTek Inc.
 */

#include <linux/firmware.h>
#include <linux/fs.h>
#include "mt7996.h"
#include "mcu.h"
#include "mac.h"
#include "eeprom.h"

#define fw_name(_dev, name, ...)	({			\
	char *_fw;						\
	switch (mt76_chip(&(_dev)->mt76)) {			\
	case MT7992_DEVICE_ID:						\
		switch ((_dev)->var.type) {			\
		case MT7992_VAR_TYPE_23:			\
			_fw = MT7992_##name##_23;		\
			break;					\
		case MT7992_VAR_TYPE_24:			\
			_fw = MT7992_##name##_24;		\
			break;					\
		default:					\
			_fw = MT7992_##name;			\
		}						\
		break;						\
	case MT7990_DEVICE_ID:					\
		_fw = MT7990_##name;				\
		break;						\
	case MT7996_DEVICE_ID:						\
	default:						\
		switch ((_dev)->var.type) {			\
		case MT7996_VAR_TYPE_233:			\
			_fw = MT7996_##name##_233;		\
			break;					\
		default:					\
			_fw = MT7996_##name;			\
		}						\
		break;						\
	}							\
	_fw;							\
})

struct mt7996_patch_hdr {
	char build_date[16];
	char platform[4];
	__be32 hw_sw_ver;
	__be32 patch_ver;
	__be16 checksum;
	u16 reserved;
	struct {
		__be32 patch_ver;
		__be32 subsys;
		__be32 feature;
		__be32 n_region;
		__be32 crc;
		u32 reserved[11];
	} desc;
} __packed;

struct mt7996_patch_sec {
	__be32 type;
	__be32 offs;
	__be32 size;
	union {
		__be32 spec[13];
		struct {
			__be32 addr;
			__be32 len;
			__be32 sec_key_idx;
			__be32 align_len;
			u32 reserved[9];
		} info;
	};
} __packed;

struct mt7996_fw_trailer {
	u8 chip_id;
	u8 eco_code;
	u8 n_region;
	u8 format_ver;
	u8 format_flag;
	u8 reserved[2];
	char fw_ver[10];
	char build_date[15];
	u32 crc;
} __packed;

struct mt7996_fw_region {
	__le32 decomp_crc;
	__le32 decomp_len;
	__le32 decomp_blk_sz;
	u8 reserved[4];
	__le32 addr;
	__le32 len;
	u8 feature_set;
	u8 reserved1[15];
} __packed;

#define MCU_PATCH_ADDRESS		0x200000

#define HE_PHY(p, c)			u8_get_bits(c, IEEE80211_HE_PHY_##p)
#define HE_MAC(m, c)			u8_get_bits(c, IEEE80211_HE_MAC_##m)
#define EHT_PHY(p, c)			u8_get_bits(c, IEEE80211_EHT_PHY_##p)

static bool sr_scene_detect = true;
module_param(sr_scene_detect, bool, 0644);
MODULE_PARM_DESC(sr_scene_detect, "Enable firmware scene detection algorithm");

static u8
mt7996_mcu_get_sta_nss(u16 mcs_map)
{
	u8 nss;

	for (nss = 8; nss > 0; nss--) {
		u8 nss_mcs = (mcs_map >> (2 * (nss - 1))) & 3;

		if (nss_mcs != IEEE80211_VHT_MCS_NOT_SUPPORTED)
			break;
	}

	return nss ? nss - 1 : 0;
}

static void
mt7996_mcu_set_sta_he_mcs(struct ieee80211_link_sta *link_sta,
			  struct mt7996_vif_link *link,
			  __le16 *he_mcs, u16 mcs_map)
{
	struct mt76_phy *mphy = mt76_vif_link_phy(&link->mt76);
	int nss, max_nss = link_sta->rx_nss > 3 ? 4 : link_sta->rx_nss;
	enum nl80211_band band;
	const u16 *mask;

	if (!mphy)
		return;

	band = mphy->chandef.chan->band;
	mask = link->bitrate_mask.control[band].he_mcs;

	for (nss = 0; nss < max_nss; nss++) {
		int mcs;

		switch ((mcs_map >> (2 * nss)) & 0x3) {
		case IEEE80211_HE_MCS_SUPPORT_0_11:
			mcs = GENMASK(11, 0);
			break;
		case IEEE80211_HE_MCS_SUPPORT_0_9:
			mcs = GENMASK(9, 0);
			break;
		case IEEE80211_HE_MCS_SUPPORT_0_7:
			mcs = GENMASK(7, 0);
			break;
		default:
			mcs = 0;
		}

		mcs = mcs ? fls(mcs & mask[nss]) - 1 : -1;

		switch (mcs) {
		case 0 ... 7:
			mcs = IEEE80211_HE_MCS_SUPPORT_0_7;
			break;
		case 8 ... 9:
			mcs = IEEE80211_HE_MCS_SUPPORT_0_9;
			break;
		case 10 ... 11:
			mcs = IEEE80211_HE_MCS_SUPPORT_0_11;
			break;
		default:
			mcs = IEEE80211_HE_MCS_NOT_SUPPORTED;
			break;
		}
		mcs_map &= ~(0x3 << (nss * 2));
		mcs_map |= mcs << (nss * 2);
	}

	*he_mcs = cpu_to_le16(mcs_map);
}

static void
mt7996_mcu_set_sta_vht_mcs(struct ieee80211_link_sta *link_sta,
			   __le16 *vht_mcs, const u16 *mask)
{
	u16 mcs, mcs_map = le16_to_cpu(link_sta->vht_cap.vht_mcs.rx_mcs_map);
	int nss, max_nss = link_sta->rx_nss > 3 ? 4 : link_sta->rx_nss;

	for (nss = 0; nss < max_nss; nss++, mcs_map >>= 2) {
		switch (mcs_map & 0x3) {
		case IEEE80211_VHT_MCS_SUPPORT_0_9:
			mcs = GENMASK(9, 0);
			break;
		case IEEE80211_VHT_MCS_SUPPORT_0_8:
			mcs = GENMASK(8, 0);
			break;
		case IEEE80211_VHT_MCS_SUPPORT_0_7:
			mcs = GENMASK(7, 0);
			break;
		default:
			mcs = 0;
		}

		vht_mcs[nss] = cpu_to_le16(mcs & mask[nss]);
	}
}

static void
mt7996_mcu_set_sta_ht_mcs(struct ieee80211_link_sta *link_sta,
			  u8 *ht_mcs, const u8 *mask)
{
	int nss, max_nss = link_sta->rx_nss > 3 ? 4 : link_sta->rx_nss;

	for (nss = 0; nss < max_nss; nss++)
		ht_mcs[nss] = link_sta->ht_cap.mcs.rx_mask[nss] & mask[nss];
}

static int
mt7996_mcu_parse_response(struct mt76_dev *mdev, int cmd,
			  struct sk_buff *skb, int seq)
{
	struct mt7996_dev *dev = container_of(mdev, struct mt7996_dev, mt76);
	struct mt7996_mcu_rxd *rxd;
	struct mt7996_mcu_uni_event *event;
	int mcu_cmd = FIELD_GET(__MCU_CMD_FIELD_ID, cmd);
	int ret = 0;
	u32 status;

	if (!skb) {
		dev_err(mdev->dev, "Message %08x (seq %d) timeout\n",
			cmd, seq);

		spin_lock_bh(&dev->mt76.lock);
		if (dev->recovery.hw_init_done &&
		    !test_and_set_bit(MT76_MCU_RESET, &dev->mphy.state)) {
			dev->recovery.restart = true;
			wake_up(&dev->mt76.mcu.wait);
			queue_work(dev->mt76.wq, &dev->reset_work);
			wake_up(&dev->reset_wait);
		}
		spin_unlock_bh(&dev->mt76.lock);

		return -ETIMEDOUT;
	}
	if (cmd == MCU_CMD(PATCH_SEM_CONTROL) &&
	    skb->len < offsetofend(struct mt7996_mcu_rxd, ext_eid))
		return -EINVAL;
	if (cmd != MCU_CMD(PATCH_SEM_CONTROL) && skb->len < sizeof(*rxd))
		return -EINVAL;

	rxd = (struct mt7996_mcu_rxd *)skb->data;
	if (seq != rxd->seq)
		return -EAGAIN;

	if (cmd == MCU_CMD(PATCH_SEM_CONTROL)) {
		skb_pull(skb, sizeof(*rxd) - 4);
		ret = *skb->data;
	} else if ((rxd->option & MCU_UNI_CMD_EVENT) &&
		    rxd->eid == MCU_UNI_EVENT_RESULT) {
		if (skb->len < sizeof(*rxd) + sizeof(*event))
			return -EINVAL;

		skb_pull(skb, sizeof(*rxd));
		event = (struct mt7996_mcu_uni_event *)skb->data;
		status = le32_to_cpu(event->status);
		/* skip invalid event */
		if (mcu_cmd != le16_to_cpu(event->cid)) {
			ret = -EAGAIN;
		} else if ((cmd & MT7996_MCU_CMD_STA_REC_EEXIST) &&
			   status == MT7996_MCU_STATUS_STA_REC_REFUSED) {
			ret = -EEXIST;
		} else if (status) {
			dev_warn_ratelimited(mdev->dev,
					     "Message %08x (seq %d) cid %04x failed: firmware status 0x%x\n",
					     cmd, seq, mcu_cmd, status);
			if (mcu_cmd == MCU_UNI_CMD_BSS_INFO_UPDATE &&
			    status == MT7996_MCU_STATUS_BCN_LOST)
				ret = -EBUSY;
			else
				ret = -EREMOTEIO;
		}
	} else {
		skb_pull(skb, sizeof(struct mt7996_mcu_rxd));
	}

	return ret;
}

static void
mt7996_mcu_set_timeout(struct mt76_dev *mdev, int cmd)
{
	mdev->mcu.timeout = 5 * HZ;

	if (!(cmd & __MCU_CMD_FIELD_UNI))
		return;

	switch (FIELD_GET(__MCU_CMD_FIELD_ID, cmd)) {
	case MCU_UNI_CMD_THERMAL:
	case MCU_UNI_CMD_TWT:
	case MCU_UNI_CMD_GET_MIB_INFO:
		mdev->mcu.timeout = 2 * HZ;
		return;
	case MCU_UNI_CMD_EFUSE_CTRL:
	case MCU_UNI_CMD_EXT_EEPROM_CTRL:
		mdev->mcu.timeout = 30 * HZ;
		return;
	default:
		break;
	}
}

static int
mt7996_mcu_send_message(struct mt76_dev *mdev, struct sk_buff *skb,
			int cmd, int *wait_seq)
{
	struct mt7996_dev *dev = container_of(mdev, struct mt7996_dev, mt76);
	int txd_len, mcu_cmd = FIELD_GET(__MCU_CMD_FIELD_ID, cmd);
	struct mt76_connac2_mcu_uni_txd *uni_txd;
	struct mt76_connac2_mcu_txd *mcu_txd;
	enum mt76_mcuq_id qid;
	__le32 *txd;
	u32 val;
	u8 seq;

	mt7996_mcu_set_timeout(mdev, cmd);

	seq = ++dev->mt76.mcu.msg_seq & 0xf;
	if (!seq)
		seq = ++dev->mt76.mcu.msg_seq & 0xf;

	if (cmd == MCU_CMD(FW_SCATTER)) {
		qid = MT_MCUQ_FWDL;
		goto exit;
	}

	txd_len = cmd & __MCU_CMD_FIELD_UNI ? sizeof(*uni_txd) : sizeof(*mcu_txd);
	txd = (__le32 *)skb_push(skb, txd_len);
	if (test_bit(MT76_STATE_MCU_RUNNING, &dev->mphy.state) && mt7996_has_wa(dev))
		qid = MT_MCUQ_WA;
	else
		qid = MT_MCUQ_WM;

	/* WA drops a longer command without an answer and loses its buffer */
	if (qid == MT_MCUQ_WA &&
	    skb->len - MT_TXD_SIZE > MT7996_MCU_WA_LEN_MAX) {
		dev_warn(mdev->dev, "Message %08x of %u octets above the WA limit\n",
			 cmd, skb->len - MT_TXD_SIZE);
		dev_kfree_skb(skb);
		return -EMSGSIZE;
	}

	val = FIELD_PREP(MT_TXD0_TX_BYTES, skb->len) |
	      FIELD_PREP(MT_TXD0_PKT_FMT, MT_TX_TYPE_CMD) |
	      FIELD_PREP(MT_TXD0_Q_IDX, MT_TX_MCU_PORT_RX_Q0);
	txd[0] = cpu_to_le32(val);

	val = FIELD_PREP(MT_TXD1_HDR_FORMAT, MT_HDR_FORMAT_CMD);
	txd[1] = cpu_to_le32(val);

	if (cmd & __MCU_CMD_FIELD_UNI) {
		uni_txd = (struct mt76_connac2_mcu_uni_txd *)txd;
		uni_txd->len = cpu_to_le16(skb->len - sizeof(uni_txd->txd));
		uni_txd->cid = cpu_to_le16(mcu_cmd);
		uni_txd->s2d_index = MCU_S2D_H2CN;
		uni_txd->pkt_type = MCU_PKT_ID;
		uni_txd->seq = seq;

		uni_txd->option = MCU_CMD_UNI;
		if (!(cmd & __MCU_CMD_FIELD_QUERY))
			uni_txd->option |= MCU_CMD_SET;

		if (wait_seq)
			uni_txd->option |= MCU_CMD_ACK;

		if ((cmd & __MCU_CMD_FIELD_WA) && (cmd & __MCU_CMD_FIELD_WM))
			uni_txd->s2d_index = MCU_S2D_H2CN;
		else if (cmd & __MCU_CMD_FIELD_WA)
			uni_txd->s2d_index = MCU_S2D_H2C;
		else if (cmd & __MCU_CMD_FIELD_WM)
			uni_txd->s2d_index = MCU_S2D_H2N;

		goto exit;
	}

	mcu_txd = (struct mt76_connac2_mcu_txd *)txd;
	mcu_txd->len = cpu_to_le16(skb->len - sizeof(mcu_txd->txd));
	mcu_txd->pq_id = cpu_to_le16(MCU_PQ_ID(MT_TX_PORT_IDX_MCU,
					       MT_TX_MCU_PORT_RX_Q0));
	mcu_txd->pkt_type = MCU_PKT_ID;
	mcu_txd->seq = seq;

	mcu_txd->cid = FIELD_GET(__MCU_CMD_FIELD_ID, cmd);
	mcu_txd->set_query = MCU_Q_NA;
	mcu_txd->ext_cid = FIELD_GET(__MCU_CMD_FIELD_EXT_ID, cmd);
	if (mcu_txd->ext_cid) {
		mcu_txd->ext_cid_ack = 1;

		if (cmd & __MCU_CMD_FIELD_QUERY)
			mcu_txd->set_query = MCU_Q_QUERY;
		else
			mcu_txd->set_query = MCU_Q_SET;
	}

	if (cmd & __MCU_CMD_FIELD_WA)
		mcu_txd->s2d_index = MCU_S2D_H2C;
	else
		mcu_txd->s2d_index = MCU_S2D_H2N;

exit:
	if (wait_seq)
		*wait_seq = seq;

	return mt76_tx_queue_skb_raw(dev, mdev->q_mcu[qid], skb, 0);
}

int mt7996_mcu_wa_cmd(struct mt7996_dev *dev, int cmd, u32 a1, u32 a2, u32 a3)
{
	struct {
		u8 _rsv[4];

		__le16 tag;
		__le16 len;
		__le32 args[3];
	} __packed req = {
		.args = {
			cpu_to_le32(a1),
			cpu_to_le32(a2),
			cpu_to_le32(a3),
		},
	};

	if (mt7996_has_wa(dev))
		return mt76_mcu_send_msg(&dev->mt76, cmd, &req.args,
					 sizeof(req.args), false);

	req.tag = cpu_to_le16(cmd == MCU_WA_PARAM_CMD(QUERY) ? UNI_CMD_SDO_QUERY :
							       UNI_CMD_SDO_SET);
	req.len = cpu_to_le16(sizeof(req) - 4);

	return mt76_mcu_send_msg(&dev->mt76, MCU_WA_UNI_CMD(SDO), &req,
				 sizeof(req), false);
}

int mt7996_mcu_set_qos_map(struct mt7996_dev *dev, struct mt76_vif_link *mlink)
{
	struct {
		u8 _rsv[4];

		__le16 tag;
		__le16 len;

		struct {
			u8 bss_idx;
			u8 enable;
			u8 _rsv[2];
			u8 up[64];
		} __packed data;
	} __packed req = {
		.tag = cpu_to_le16(UNI_CMD_SDO_SET_QOS_MAP),
		.len = cpu_to_le16(sizeof(req) - 4),
		.data = {
			.bss_idx = mlink->idx,
			.enable = true,
		},
	};
	int i;

	/* SDO takes the TID of an 802.3 frame from this table */
	for (i = 0; i < ARRAY_SIZE(req.data.up); i++)
		req.data.up[i] = cfg80211_dscp_to_up(i, NULL);

	if (mt7996_has_wa(dev))
		return mt76_mcu_send_msg(&dev->mt76, MCU_WA_EXT_CMD(SET_QOS_MAP),
					 &req.data, sizeof(req.data), false);

	return mt76_mcu_send_msg(&dev->mt76, MCU_WA_UNI_CMD(SDO), &req,
				 sizeof(req), false);
}

struct mt7996_mcu_countdown_data {
	struct mt76_phy *mphy;
	u8 omac_idx;
};

static void
mt7996_mcu_csa_finish(void *priv, u8 *mac, struct ieee80211_vif *vif)
{
	struct mt7996_mcu_countdown_data *cdata = (void *)priv;
	struct mt7996_vif *mvif = (struct mt7996_vif *)vif->drv_priv;
	struct ieee80211_bss_conf *link_conf = NULL;
	unsigned long valid_links = vif->valid_links ?: BIT(0);
	unsigned int link_id;

	if (vif->type == NL80211_IFTYPE_STATION)
		return;

	for_each_set_bit(link_id, &valid_links, IEEE80211_MLD_MAX_NUM_LINKS) {
		struct mt76_vif_link *mlink =
			rcu_dereference(mvif->mt76.link[link_id]);

		if (mlink && mlink->band_idx == cdata->mphy->band_idx &&
		    mlink->omac_idx == cdata->omac_idx) {
			link_conf = rcu_dereference(vif->link_conf[link_id]);
			break;
		}
	}

	if (!link_conf || !link_conf->csa_active)
		return;

	if (!mt76_cntdwn_finish_take(&mvif->link_info[link_id].cntdwn))
		return;

	ieee80211_csa_finish(vif, link_conf->link_id);
}

static void
mt7996_mcu_cca_finish(void *priv, u8 *mac, struct ieee80211_vif *vif)
{
	struct mt7996_mcu_countdown_data *cdata = (void *)priv;
	struct mt7996_vif *mvif = (struct mt7996_vif *)vif->drv_priv;
	struct ieee80211_bss_conf *link_conf = NULL;
	unsigned long valid_links = vif->valid_links ?: BIT(0);
	unsigned int link_id;

	if (vif->type == NL80211_IFTYPE_STATION)
		return;

	for_each_set_bit(link_id, &valid_links, IEEE80211_MLD_MAX_NUM_LINKS) {
		struct mt76_vif_link *mlink =
			rcu_dereference(mvif->mt76.link[link_id]);

		if (mlink && mlink->band_idx == cdata->mphy->band_idx &&
		    mlink->omac_idx == cdata->omac_idx) {
			link_conf = rcu_dereference(vif->link_conf[link_id]);
			break;
		}
	}

	if (!link_conf || !link_conf->color_change_active)
		return;

	if (!mt76_cntdwn_finish_take(&mvif->link_info[link_id].cntdwn))
		return;

	ieee80211_color_change_finish(vif, link_conf->link_id);
}

static void
mt7996_mcu_ie_countdown(struct mt7996_dev *dev, struct sk_buff *skb)
{
#define UNI_EVENT_IE_COUNTDOWN_CSA 0
#define UNI_EVENT_IE_COUNTDOWN_BCC 1
	struct header {
		u8 band;
		u8 rsv[3];
	};
	struct mt7996_mcu_rxd *rxd = (struct mt7996_mcu_rxd *)skb->data;
	const char *data = (char *)&rxd[1], *tail;
	struct header *hdr = (struct header *)data;
	struct tlv *tlv = (struct tlv *)(data + 4);
	struct mt7996_mcu_countdown_notify *event;
	struct mt7996_mcu_countdown_data cdata;

	if (skb->len < sizeof(*rxd) + sizeof(*hdr))
		return;

	if (hdr->band >= ARRAY_SIZE(dev->mt76.phys))
		return;

	cdata.mphy = dev->mt76.phys[hdr->band];
	if (!cdata.mphy)
		return;

	tail = skb->data + skb->len;
	data += sizeof(*hdr);
	while (data + sizeof(*tlv) <= tail) {
		u16 tag_len = le16_to_cpu(tlv->len);

		if (tag_len < sizeof(*tlv) + sizeof(*event) ||
		    data + tag_len > tail)
			break;

		event = (struct mt7996_mcu_countdown_notify *)tlv->data;
		cdata.omac_idx = event->omac_idx;

		switch (le16_to_cpu(tlv->tag)) {
		case UNI_EVENT_IE_COUNTDOWN_CSA:
			ieee80211_iterate_active_interfaces_atomic(mt76_hw(dev),
					IEEE80211_IFACE_ITER_RESUME_ALL,
					mt7996_mcu_csa_finish, &cdata);
			break;
		case UNI_EVENT_IE_COUNTDOWN_BCC:
			ieee80211_iterate_active_interfaces_atomic(mt76_hw(dev),
					IEEE80211_IFACE_ITER_RESUME_ALL,
					mt7996_mcu_cca_finish, &cdata);
			break;
		default:
			break;
		}

		data += tag_len;
		tlv = (struct tlv *)data;
	}
}

static void
mt7996_mcu_rx_radar_detected(struct mt7996_dev *dev, struct sk_buff *skb)
{
	struct mt76_phy *mphy = &dev->mt76.phy;
	struct mt7996_mcu_rdd_report *r;

	if (skb->len < offsetofend(struct mt7996_mcu_rdd_report, rdd_idx))
		return;

	r = (struct mt7996_mcu_rdd_report *)skb->data;

	switch (r->rdd_idx) {
	case MT_RDD_IDX_BAND2:
		mphy = dev->mt76.phys[MT_BAND2];
		break;
	case MT_RDD_IDX_BAND1:
		mphy = dev->mt76.phys[MT_BAND1];
		break;
	case MT_RDD_IDX_BACKGROUND:
		if (!dev->rdd2_phy)
			goto err;
		mphy = dev->rdd2_phy->mt76;
		break;
	default:
		goto err;
	}

	if (!mphy)
		goto err;

	if (r->rdd_idx == MT_RDD_IDX_BACKGROUND) {
		cfg80211_background_radar_event(mphy->hw->wiphy,
						&dev->rdd2_chandef,
						GFP_ATOMIC);
	} else {
		struct mt76_chanctx *ctx = READ_ONCE(mphy->chanctx);
		struct mt7996_phy *phy = mphy->priv;
		struct ieee80211_chanctx_conf *conf;

		phy->rdd_tx_paused = true;
		if (ctx) {
			conf = container_of((void *)ctx,
					    struct ieee80211_chanctx_conf, drv_priv);
			ieee80211_radar_detected(mphy->hw, conf);
		}
	}
	dev->hw_pattern++;

	return;

err:
	dev_err(dev->mt76.dev, "Invalid RDD idx %d\n", r->rdd_idx);
}

static void
mt7996_mcu_rx_log_message(struct mt7996_dev *dev, struct sk_buff *skb)
{
#define UNI_EVENT_FW_LOG_FORMAT 0
	struct mt7996_mcu_rxd *rxd = (struct mt7996_mcu_rxd *)skb->data;
	const char *data, *type;
	struct tlv *tlv;
	int len;

	if (skb->len < sizeof(*rxd))
		return;

	if (!(rxd->option & MCU_UNI_CMD_EVENT)) {
		data = (char *)&rxd[1];
		len = skb->len - sizeof(*rxd);
		goto out;
	}

	if (skb->len < sizeof(*rxd) + 4 + sizeof(*tlv))
		return;

	tlv = (struct tlv *)((char *)&rxd[1] + 4);
	if (le16_to_cpu(tlv->tag) != UNI_EVENT_FW_LOG_FORMAT)
		return;

	if (le16_to_cpu(tlv->len) < sizeof(*tlv) + 4)
		return;

	data = (char *)tlv + sizeof(*tlv) + 4;
	len = le16_to_cpu(tlv->len) - sizeof(*tlv) - 4;
	len = min_t(int, len, skb->len - (int)(data - (char *)skb->data));

out:
	switch (rxd->s2d_index) {
	case 0:
		if (mt7996_debugfs_rx_log(dev, data, len))
			return;

		type = "WM";
		break;
	case 2:
		type = "WA";
		break;
	default:
		type = "unknown";
		break;
	}

	wiphy_info(mt76_hw(dev)->wiphy, "%s: %.*s", type, len, data);
}

static int
mt7996_mcu_tx_rate_parse(struct rate_info *rate,
			 const struct ieee80211_supported_band *sband,
			 const struct all_sta_trx_rate *mcu_rate)
{
	u8 mcs = mcu_rate->tx_mcs, nss = mcu_rate->tx_nss;
	bool is_2g = sband->band == NL80211_BAND_2GHZ;
	struct rate_info tmp = {};

	if (mcu_rate->tx_stbc && nss > 1)
		nss /= 2;

	switch (mcu_rate->tx_mode) {
	case MT_PHY_TYPE_UNKNOWN:
		return 0;
	case MT_PHY_TYPE_CCK:
		/* the firmware reports a wlan_idx without a rate as all zero */
		if (!is_2g)
			return 0;

		/* the short preamble bit does not change the rate */
		mcs &= ~BIT(2);
		if (mcs > 3)
			return -EINVAL;

		tmp.legacy = sband->bitrates[mcs].bitrate;
		break;
	case MT_PHY_TYPE_OFDM:
		/* the OFDM rates follow the four CCK rates on 2.4 GHz */
		if (is_2g)
			mcs += 4;
		if (mcs >= sband->n_bitrates)
			return -EINVAL;

		tmp.legacy = sband->bitrates[mcs].bitrate;
		break;
	case MT_PHY_TYPE_HT:
	case MT_PHY_TYPE_HT_GF:
		if (nss < 1)
			return -EINVAL;

		tmp.mcs = mcs + ((nss - 1) << 3);
		if (tmp.mcs > 31)
			return -EINVAL;

		tmp.flags = RATE_INFO_FLAGS_MCS;
		if (mcu_rate->tx_gi)
			tmp.flags |= RATE_INFO_FLAGS_SHORT_GI;
		break;
	case MT_PHY_TYPE_VHT:
		if (mcs > 9 || nss < 1 || nss > 8)
			return -EINVAL;

		tmp.mcs = mcs;
		tmp.nss = nss;
		tmp.flags = RATE_INFO_FLAGS_VHT_MCS;
		if (mcu_rate->tx_gi)
			tmp.flags |= RATE_INFO_FLAGS_SHORT_GI;
		break;
	case MT_PHY_TYPE_HE_SU:
	case MT_PHY_TYPE_HE_EXT_SU:
	case MT_PHY_TYPE_HE_TB:
	case MT_PHY_TYPE_HE_MU:
		tmp.mcs = mcs & GENMASK(3, 0);
		if (tmp.mcs > 13 || nss < 1 || nss > 8 ||
		    mcu_rate->tx_gi > NL80211_RATE_INFO_HE_GI_3_2)
			return -EINVAL;

		tmp.nss = nss;
		tmp.flags = RATE_INFO_FLAGS_HE_MCS;
		tmp.he_gi = mcu_rate->tx_gi;
		tmp.he_dcm = !!(mcs & MT_PRXV_TX_DCM);
		break;
	case MT_PHY_TYPE_EHT_SU:
	case MT_PHY_TYPE_EHT_TRIG:
	case MT_PHY_TYPE_EHT_MU:
		tmp.mcs = mcs & GENMASK(3, 0);
		if (nss < 1 || nss > 8 ||
		    mcu_rate->tx_gi > NL80211_RATE_INFO_EHT_GI_3_2)
			return -EINVAL;

		/* cfg80211 takes EHT-MCS 14 on a wide channel alone */
		if (tmp.mcs == 14 && mcu_rate->tx_bw < IEEE80211_STA_RX_BW_80)
			return 0;

		tmp.nss = nss;
		tmp.flags = RATE_INFO_FLAGS_EHT_MCS;
		tmp.eht_gi = mcu_rate->tx_gi;
		break;
	default:
		return -EINVAL;
	}

	switch (mcu_rate->tx_bw) {
	case IEEE80211_STA_RX_BW_20:
		tmp.bw = RATE_INFO_BW_20;
		break;
	case IEEE80211_STA_RX_BW_40:
		tmp.bw = RATE_INFO_BW_40;
		break;
	case IEEE80211_STA_RX_BW_80:
		tmp.bw = RATE_INFO_BW_80;
		break;
	case IEEE80211_STA_RX_BW_160:
		tmp.bw = RATE_INFO_BW_160;
		break;
	case IEEE80211_STA_RX_BW_320:
		tmp.bw = RATE_INFO_BW_320;
		break;
	default:
		return -EINVAL;
	}

	if (mcu_rate->tx_mode == MT_PHY_TYPE_HE_EXT_SU &&
	    (mcs & MT_PRXV_TX_ER_SU_106T)) {
		tmp.bw = RATE_INFO_BW_HE_RU;
		tmp.he_ru_alloc = NL80211_RATE_INFO_HE_RU_ALLOC_106;
	}

	*rate = tmp;

	return 0;
}

static void
mt7996_mcu_update_tx_rate(struct mt7996_dev *dev, struct mt76_wcid *wcid,
			  const struct all_sta_trx_rate *mcu_rate)
{
	struct mt76_phy *mphy = mt76_dev_phy(&dev->mt76, wcid->phy_idx);
	struct ieee80211_supported_band *sband;

	/* nothing reads the rate of a wcid without a station */
	if (!wcid->sta || !mphy->chandef.chan)
		return;

	if (mphy->chandef.chan->band == NL80211_BAND_5GHZ)
		sband = &mphy->sband_5g.sband;
	else if (mphy->chandef.chan->band == NL80211_BAND_6GHZ)
		sband = &mphy->sband_6g.sband;
	else
		sband = &mphy->sband_2g.sband;

	if (mt7996_mcu_tx_rate_parse(&wcid->rate, sband, mcu_rate))
		dev_err(dev->mt76.dev, "Failed to update TX rate\n");
}

static struct ieee80211_sta *
mt7996_mcu_airtime_sta(struct mt7996_dev *dev,
		       struct mt7996_mcu_all_sta_info_event *res, u16 i)
{
	u16 wlan_idx = le16_to_cpu(res->airtime[i].wlan_idx);

	return wcid_to_sta(mt76_wcid_ptr(dev, wlan_idx));
}

static u32 mt7996_mcu_airtime_us(u64 val)
{
	/* the firmware counts in units of 1.024 us */
	return min_t(u64, div_u64(val * 1024, 1000), U32_MAX);
}

/* converted per AC like the station sum, so that the links of a station do
 * not add up to more than it registered, unless a value reaches U32_MAX
 */
static void
mt7996_mcu_airtime_link_add(struct mt7996_dev *dev,
			    struct mt7996_mcu_all_sta_info_event *res, u16 i)
{
	u16 wlan_idx = le16_to_cpu(res->airtime[i].wlan_idx);
	struct mt7996_sta_link *msta_link;
	struct mt76_wcid *wcid;
	u8 ac;

	wcid = mt76_wcid_ptr(dev, wlan_idx);
	if (!wcid_to_sta(wcid))
		return;

	msta_link = container_of(wcid, struct mt7996_sta_link, wcid);
	for (ac = 0; ac < IEEE80211_NUM_ACS; ac++) {
		u8 q = mt76_connac_lmac_mapping(ac);

		msta_link->airtime_tx +=
			mt7996_mcu_airtime_us(le32_to_cpu(res->airtime[i].tx[q]));
		msta_link->airtime_rx +=
			mt7996_mcu_airtime_us(le32_to_cpu(res->airtime[i].rx[q]));
	}
}

/* mac80211 drops the estimate of completed frames at each call, so the
 * links of a station are summed into one call per AC
 */
static void
mt7996_mcu_rx_airtime(struct mt7996_dev *dev,
		      struct mt7996_mcu_all_sta_info_event *res, u16 sta_num)
{
	struct ieee80211_sta *sta;
	struct mt7996_sta *msta;
	u32 seq;
	u16 i;
	u8 ac;

	atomic_set(&dev->airtime.missed, 0);

	if (!++dev->airtime.seq)
		dev->airtime.seq = 1;
	seq = dev->airtime.seq;

	rcu_read_lock();

	for (i = 0; i < sta_num; i++) {
		sta = mt7996_mcu_airtime_sta(dev, res, i);
		if (!sta)
			continue;

		msta = (struct mt7996_sta *)sta->drv_priv;
		if (msta->airtime_seq != seq) {
			msta->airtime_seq = seq;
			memset(msta->airtime_tx, 0, sizeof(msta->airtime_tx));
			memset(msta->airtime_rx, 0, sizeof(msta->airtime_rx));
		}

		for (ac = 0; ac < IEEE80211_NUM_ACS; ac++) {
			u8 q = mt76_connac_lmac_mapping(ac);

			msta->airtime_tx[ac] += le32_to_cpu(res->airtime[i].tx[q]);
			msta->airtime_rx[ac] += le32_to_cpu(res->airtime[i].rx[q]);
		}
	}

	for (i = 0; i < sta_num; i++) {
		sta = mt7996_mcu_airtime_sta(dev, res, i);
		if (!sta)
			continue;

		msta = (struct mt7996_sta *)sta->drv_priv;
		if (msta->airtime_seq != seq)
			continue;

		msta->airtime_seq = 0;
		for (ac = 0; ac < IEEE80211_NUM_ACS; ac++) {
			u32 tx = mt7996_mcu_airtime_us(msta->airtime_tx[ac]);
			u32 rx = mt7996_mcu_airtime_us(msta->airtime_rx[ac]);

			ieee80211_sta_register_airtime(sta, mt7996_ac_to_tid(ac),
						       tx, rx);
		}
	}

	for (i = 0; i < sta_num; i++)
		mt7996_mcu_airtime_link_add(dev, res, i);

	rcu_read_unlock();
}

static void
mt7996_mcu_rx_all_sta_info_event(struct mt7996_dev *dev, struct sk_buff *skb)
{
	struct mt7996_mcu_all_sta_info_event *res;
	u32 elem_size;
	u16 sta_num;
	u16 i;

	if (skb->len < sizeof(struct mt7996_mcu_rxd))
		return;

	skb_pull(skb, sizeof(struct mt7996_mcu_rxd));

	res = (struct mt7996_mcu_all_sta_info_event *)skb->data;
	if (skb->len < sizeof(*res))
		return;

	sta_num = le16_to_cpu(res->sta_num);
	switch (le16_to_cpu(res->tag)) {
	case UNI_ALL_STA_TXRX_RATE:
		elem_size = sizeof(res->rate[0]);
		break;
	case UNI_ALL_STA_TXRX_ADM_STAT:
		elem_size = sizeof(res->adm_stat[0]);
		break;
	case UNI_ALL_STA_TXRX_MSDU_COUNT:
		elem_size = sizeof(res->msdu_cnt[0]);
		break;
	case UNI_ALL_STA_TXRX_AIR_TIME:
		elem_size = sizeof(res->airtime[0]);
		break;
	default:
		return;
	}

	if (sta_num > (skb->len - sizeof(*res)) / elem_size)
		sta_num = (skb->len - sizeof(*res)) / elem_size;

	if (le16_to_cpu(res->tag) == UNI_ALL_STA_TXRX_AIR_TIME) {
		mt7996_mcu_rx_airtime(dev, res, sta_num);
		return;
	}

	for (i = 0; i < sta_num; i++) {
		u8 ac;
		u16 wlan_idx;
		struct mt76_wcid *wcid;

		switch (le16_to_cpu(res->tag)) {
		case UNI_ALL_STA_TXRX_RATE:
			wlan_idx = le16_to_cpu(res->rate[i].wlan_idx);
			wcid = mt76_wcid_ptr(dev, wlan_idx);

			if (!wcid)
				break;

			mt7996_mcu_update_tx_rate(dev, wcid, &res->rate[i]);
			break;
		case UNI_ALL_STA_TXRX_ADM_STAT:
			wlan_idx = le16_to_cpu(res->adm_stat[i].wlan_idx);
			wcid = mt76_wcid_ptr(dev, wlan_idx);

			if (!wcid)
				break;

			for (ac = 0; ac < IEEE80211_NUM_ACS; ac++) {
				wcid->stats.tx_bytes +=
					le32_to_cpu(res->adm_stat[i].tx_bytes[ac]);
				wcid->stats.rx_bytes +=
					le32_to_cpu(res->adm_stat[i].rx_bytes[ac]);
			}
			break;
		case UNI_ALL_STA_TXRX_MSDU_COUNT:
			wlan_idx = le16_to_cpu(res->msdu_cnt[i].wlan_idx);
			wcid = mt76_wcid_ptr(dev, wlan_idx);

			if (!wcid)
				break;

			wcid->stats.tx_packets +=
				le32_to_cpu(res->msdu_cnt[i].tx_msdu_cnt);
			wcid->stats.rx_packets +=
				le32_to_cpu(res->msdu_cnt[i].rx_msdu_cnt);
			break;
		default:
			break;
		}
	}
}

static void
mt7996_mcu_rx_thermal_notify(struct mt7996_dev *dev, struct sk_buff *skb)
{
#define THERMAL_NOTIFY_TAG 0x4
#define THERMAL_NOTIFY 0x2
	struct mt76_phy *mphy = &dev->mt76.phy;
	struct mt7996_mcu_thermal_notify *n;
	struct mt7996_phy *phy;

	if (skb->len < offsetofend(struct mt7996_mcu_thermal_notify,
				   duty_percent))
		return;

	n = (struct mt7996_mcu_thermal_notify *)skb->data;

	if (le16_to_cpu(n->tag) != THERMAL_NOTIFY_TAG)
		return;

	if (n->event_id != THERMAL_NOTIFY)
		return;

	if (n->band_idx > MT_BAND2)
		return;

	mphy = dev->mt76.phys[n->band_idx];
	if (!mphy)
		return;

	phy = (struct mt7996_phy *)mphy->priv;
	phy->throttle_state = n->duty_percent;
}

static void
mt7996_mcu_rx_ext_event(struct mt7996_dev *dev, struct sk_buff *skb)
{
	struct mt7996_mcu_rxd *rxd = (struct mt7996_mcu_rxd *)skb->data;

	switch (rxd->ext_eid) {
	case MCU_EXT_EVENT_FW_LOG_2_HOST:
		mt7996_mcu_rx_log_message(dev, skb);
		break;
	default:
		break;
	}
}

static void
mt7996_mcu_rx_unsolicited_event(struct mt7996_dev *dev, struct sk_buff *skb)
{
	struct mt7996_mcu_rxd *rxd = (struct mt7996_mcu_rxd *)skb->data;

	switch (rxd->eid) {
	case MCU_EVENT_EXT:
		mt7996_mcu_rx_ext_event(dev, skb);
		break;
	case MCU_UNI_EVENT_THERMAL:
		mt7996_mcu_rx_thermal_notify(dev, skb);
		break;
	default:
		break;
	}
	dev_kfree_skb(skb);
}

static void
mt7996_mcu_wed_rro_event(struct mt7996_dev *dev, struct sk_buff *skb)
{
	struct mt7996_mcu_wed_rro_event *event = (void *)skb->data;

	if (!mt7996_has_hwrro(dev) || skb->len < sizeof(*event))
		return;

	skb_pull(skb, sizeof(struct mt7996_mcu_rxd) + 4);

	switch (le16_to_cpu(event->tag)) {
	case UNI_WED_RRO_BA_SESSION_STATUS: {
		struct mt7996_mcu_wed_rro_ba_event *e;

		while (skb->len >= sizeof(*e)) {
			struct mt76_rx_tid *tid;
			struct mt76_wcid *wcid;
			u16 idx;

			e = (void *)skb->data;
			idx = le16_to_cpu(e->wlan_id);
			wcid = mt76_wcid_ptr(dev, idx);
			if (!wcid || !wcid->sta)
				break;

			if (e->tid >= ARRAY_SIZE(wcid->aggr))
				break;

			tid = rcu_dereference(wcid->aggr[e->tid]);
			if (!tid)
				break;

			tid->id = le16_to_cpu(e->id);
			skb_pull(skb, sizeof(*e));
		}
		break;
	}
	case UNI_WED_RRO_BA_SESSION_DELETE: {
		struct mt7996_mcu_wed_rro_ba_delete_event *e;

		while (skb->len >= sizeof(*e)) {
			struct mt7996_wed_rro_session_id *session;

			e = (void *)skb->data;
			session = kzalloc(sizeof(*session), GFP_ATOMIC);
			if (!session)
				break;

			session->id = le16_to_cpu(e->session_id);

			spin_lock_bh(&dev->wed_rro.lock);
			list_add_tail(&session->list, &dev->wed_rro.poll_list);
			spin_unlock_bh(&dev->wed_rro.lock);

			ieee80211_queue_work(mt76_hw(dev), &dev->wed_rro.work);
			skb_pull(skb, sizeof(*e));
		}
		break;
	}
	default:
		break;
	}
}

static void
mt7996_mcu_ps_transition(struct mt7996_dev *dev, u16 wcid_idx, bool ps)
{
	struct mt7996_sta_link *msta_link;
	struct mt76_wcid *wcid;

	wcid = mt76_wcid_ptr(dev, wcid_idx);
	if (!wcid || !wcid_to_sta(wcid))
		return;

	/* the firmware can deliver an enter that it stored before the
	 * first TWT agreement of the station
	 */
	msta_link = container_of(wcid, struct mt7996_sta_link, wcid);
	if (ps && READ_ONCE(msta_link->twt.flowid_mask))
		return;

	mt76_sta_ps_transition(&dev->mt76, wcid, ps);
}

static void
mt7996_mcu_rx_ps_sync(struct mt7996_dev *dev, struct sk_buff *skb)
{
	struct mt7996_mcu_ps_sync_event *event = (void *)skb->data;
	struct tlv *tlv;
	int len;

	if (skb->len < sizeof(*event))
		return;

	skb_pull(skb, sizeof(*event));

	len = skb->len;
	while (len >= sizeof(*tlv)) {
		u16 tag, tag_len;

		tlv = (struct tlv *)skb->data;
		tag = le16_to_cpu(tlv->tag);
		tag_len = le16_to_cpu(tlv->len);
		/* a tag_len below the header size would not advance the buffer */
		if (tag_len < sizeof(*tlv) || tag_len > len)
			break;

		switch (tag) {
		case UNI_PS_CLIENT_INFO: {
			struct mt7996_mcu_ps_client_info *info = (void *)tlv;

			/* the TWT state of a station, not its PS state: the
			 * firmware gates the air time of a station in TWT
			 * operation and reports no PS change of it
			 */
			if (tag_len < sizeof(*info) || !info->ps_bit)
				break;

			mt7996_mcu_ps_transition(dev,
						 le16_to_cpu(info->wlan_idx),
						 false);
			break;
		}
		case UNI_PS_MULTI_CLIENT_INFO: {
			struct mt7996_mcu_ps_multi_client_info *info = (void *)tlv;
			u16 cnt;
			int i;

			if (tag_len < sizeof(*info))
				break;

			cnt = min_t(u16, le16_to_cpu(info->sta_cnt),
				    (tag_len - sizeof(*info)) / sizeof(info->sta_ps_info[0]));
			for (i = 0; i < cnt; i++) {
				u16 entry = le16_to_cpu(info->sta_ps_info[i]);

				mt7996_mcu_ps_transition(dev,
					FIELD_GET(MT7996_PS_MULTI_WCID, entry),
					!!(entry & MT7996_PS_MULTI_PS_BIT));
			}
			break;
		}
		case UNI_PS_MULTI_CLIENT_INFO_BITMAP: {
			u8 *bitmap = tlv->data;
			int bitmap_len = tag_len - sizeof(*tlv);
			int i;

			for (i = 0; i < bitmap_len * 8; i++) {
				struct ieee80211_sta *sta;
				struct mt76_wcid *wcid;

				/* the firmware sets a bit only for the link
				 * records of an MLD
				 */
				wcid = mt76_wcid_ptr(dev, i);
				sta = wcid_to_sta(wcid);
				if (!sta || !sta->mlo)
					continue;

				mt76_sta_ps_transition(&dev->mt76, wcid,
					!!(bitmap[i / 8] & BIT(i % 8)));
			}
			break;
		}
		}

		skb_pull(skb, tag_len);
		len -= tag_len;
	}
}

struct mt7996_crit_update_data {
	struct mt7996_dev *dev;
	u8 bss_idx;
};

/* The event does not name the template, so it belongs to the last template
 * that the firmware accepted. An event while the firmware takes a template of
 * a new critical update can belong to either.
 */
static void
mt7996_mcu_link_crit_update_done(struct mt7996_dev *dev,
				 struct ieee80211_vif *vif,
				 struct mt7996_vif_link *link,
				 unsigned int link_id)
{
	bool pending;
	u32 id;

	spin_lock_bh(&dev->mt76.lock);
	pending = link->crit_update_pending;
	if (pending)
		link->crit_update_ambiguous = true;
	id = link->crit_update_id;
	spin_unlock_bh(&dev->mt76.lock);

	if (!pending)
		ieee80211_crit_update_done(vif, link_id, id);
}

static void
mt7996_mcu_crit_update_done(void *priv, u8 *mac, struct ieee80211_vif *vif)
{
	struct mt7996_vif *mvif = (struct mt7996_vif *)vif->drv_priv;
	unsigned long valid_links = vif->valid_links ?: BIT(0);
	struct mt7996_crit_update_data *data = priv;
	unsigned int link_id;

	if (vif->type != NL80211_IFTYPE_AP)
		return;

	for_each_set_bit(link_id, &valid_links, IEEE80211_MLD_MAX_NUM_LINKS) {
		struct mt76_vif_link *mlink =
			rcu_dereference(mvif->mt76.link[link_id]);
		struct mt7996_vif_link *link;

		if (!mlink || mlink->idx != data->bss_idx)
			continue;

		link = container_of(mlink, struct mt7996_vif_link, mt76);
		mt7996_mcu_link_crit_update_done(data->dev, vif, link, link_id);
		return;
	}
}

static void
mt7996_mcu_rx_bss_event(struct mt7996_dev *dev, struct sk_buff *skb)
{
	struct mt7996_mcu_bss_event *event = (void *)skb->data;
	struct tlv *tlv = (struct tlv *)event->buf;
	struct mt7996_crit_update_data data = {
		.dev = dev,
	};

	if (skb->len < sizeof(*event) + sizeof(*tlv))
		return;

	switch (le16_to_cpu(tlv->tag)) {
	case UNI_EVENT_BSS_BCN_CRIT_UPDATE:
		data.bss_idx = event->bss_idx;
		ieee80211_iterate_active_interfaces_atomic(mt76_hw(dev),
				IEEE80211_IFACE_ITER_RESUME_ALL,
				mt7996_mcu_crit_update_done, &data);
		break;
	default:
		break;
	}
}

struct mt7996_mld_attlm_data {
	struct mt7996_dev *dev;
	const u8 *mld_addr;
	enum nl80211_commands cmd;
	u8 idx;
};

static void mt7996_mcu_mld_attlm_cntdwn_arm(struct mt7996_vif *mvif)
{
	u64 now = ktime_get_boottime_ns(), next = U64_MAX;
	unsigned long links = mvif->attlm_cntdwn_links;
	unsigned int link_id;

	for_each_set_bit(link_id, &links, IEEE80211_MLD_MAX_NUM_LINKS) {
		u64 end = mvif->attlm_cntdwn_end[link_id];

		if (end <= now && !(mvif->attlm_stop_links & BIT(link_id)))
			continue;

		next = min(next, end);
	}

	if (next != U64_MAX)
		hrtimer_start(&mvif->attlm_cntdwn_timer, ns_to_ktime(next),
			      HRTIMER_MODE_ABS_SOFT);
}

static u16 mt7996_mcu_mld_attlm_cntdwn_clear(struct mt7996_vif *mvif, u16 links)
{
	u16 held = mvif->attlm_hold_links & links;

	mvif->attlm_cntdwn_links &= ~links;
	mvif->attlm_hold_links &= ~held;
	WRITE_ONCE(mvif->attlm_stop_links, mvif->attlm_stop_links & ~held);

	return held;
}

void mt7996_mcu_mld_attlm_cntdwn_reset(struct mt7996_dev *dev,
				       struct mt7996_vif *mvif, u16 links)
{
	spin_lock_bh(&dev->mt76.lock);
	mt7996_mcu_mld_attlm_cntdwn_clear(mvif, links);
	spin_unlock_bh(&dev->mt76.lock);
}

enum hrtimer_restart mt7996_mcu_mld_attlm_cntdwn_end(struct hrtimer *timer)
{
	struct mt7996_vif *mvif = container_of(timer, struct mt7996_vif,
					       attlm_cntdwn_timer);
	struct ieee80211_vif *vif = container_of((void *)mvif,
						 struct ieee80211_vif,
						 drv_priv);
	unsigned long ended = 0, released, upload = 0;
	struct mt7996_dev *dev = mvif->reconf_dev;
	u64 now = ktime_get_boottime_ns();
	unsigned int link_id;
	unsigned long links;

	spin_lock(&dev->mt76.lock);
	links = mvif->attlm_cntdwn_links & mvif->attlm_stop_links;
	for_each_set_bit(link_id, &links, IEEE80211_MLD_MAX_NUM_LINKS)
		if (mvif->attlm_cntdwn_end[link_id] <= now)
			ended |= BIT(link_id);
	released = mt7996_mcu_mld_attlm_cntdwn_clear(mvif, ended);
	mt7996_mcu_mld_attlm_cntdwn_arm(mvif);
	spin_unlock(&dev->mt76.lock);

	rcu_read_lock();
	for_each_set_bit(link_id, &ended, IEEE80211_MLD_MAX_NUM_LINKS) {
		struct ieee80211_bss_conf *link_conf;

		link_conf = rcu_dereference(vif->link_conf[link_id]);
		if (!link_conf)
			continue;

		if ((link_conf->csa_active || link_conf->color_change_active) &&
		    !mt76_cntdwn_finish_take(&mvif->link_info[link_id].cntdwn))
			continue;

		if (link_conf->csa_active)
			ieee80211_csa_finish(vif, link_id);
		else if (link_conf->color_change_active)
			ieee80211_color_change_finish(vif, link_id);
		else if (released & BIT(link_id))
			upload |= BIT(link_id);
	}
	rcu_read_unlock();

	if (!upload)
		return HRTIMER_NORESTART;

	spin_lock(&dev->mt76.lock);
	mvif->attlm_bcn_links |= upload;
	spin_unlock(&dev->mt76.lock);
	queue_work(system_highpri_wq, &dev->attlm_work);

	return HRTIMER_NORESTART;
}

static void mt7996_mcu_mld_attlm_stop_end(struct mt7996_vif *mvif)
{
	u16 hold = mvif->attlm_stop_links & mvif->attlm_cntdwn_links;

	mvif->attlm_hold_links |= hold;
	mvif->attlm_bcn_links |= mvif->attlm_stop_links & ~hold;
	mvif->attlm_disc_sync |= mvif->attlm_disc_off;
	WRITE_ONCE(mvif->attlm_stop_links, hold);
	mvif->attlm_disc_off = 0;
	mvif->attlm_disc_links = 0;
	mvif->attlm_links = 0;
}

/* the end of a mapping is reported once, by the first of the end event, the
 * end timer, the full reset and the interface add
 */
static u64 mt7996_mcu_mld_attlm_cookie_take(struct mt7996_vif *mvif)
{
	u64 cookie = mvif->attlm_cookie;

	mvif->attlm_cookie = 0;
	mvif->attlm_cancelled = false;

	return cookie;
}

static void
mt7996_mcu_mld_attlm_step(struct mt7996_dev *dev, struct ieee80211_vif *vif,
			  enum nl80211_commands cmd)
{
	struct mt7996_vif *mvif = (struct mt7996_vif *)vif->drv_priv;
	struct cfg80211_adv_ttlm_event event = {
		.cmd = cmd,
	};
	u16 bcn_links;

	/* the firmware stops the Beacons and the group frames of the BSS of
	 * the request alone, and the in-band discovery frames of no link
	 */
	spin_lock_bh(&dev->mt76.lock);
	if (cmd == NL80211_CMD_ADV_TTLM_ESTABLISHED) {
		if (!mvif->attlm_cancelled)
			event.cookie = mvif->attlm_cookie;
		event.tbtt_boottime = mvif->attlm_tbtt_boottime;
		mvif->attlm_bcn_links |= mvif->attlm_links;
		WRITE_ONCE(mvif->attlm_stop_links,
			   mvif->attlm_links | mvif->attlm_hold_links);
		mvif->attlm_hold_links &= ~mvif->attlm_links;
		mvif->attlm_disc_sync |= mvif->attlm_disc_links;
		mvif->attlm_disc_off = mvif->attlm_disc_links;
		mt7996_mcu_mld_attlm_cntdwn_arm(mvif);
	}
	if (cmd == NL80211_CMD_ADV_TTLM_ENDED) {
		event.cookie = mt7996_mcu_mld_attlm_cookie_take(mvif);
		mvif->attlm_end_boottime = 0;
		mvif->attlm_ttlm_resend = true;
		mt7996_mcu_mld_attlm_stop_end(mvif);
	}
	bcn_links = mvif->attlm_bcn_links | mvif->attlm_disc_sync;
	spin_unlock_bh(&dev->mt76.lock);

	/* the end of the mapping in the firmware also lifts the pause of a
	 * negotiated mapping on the links it disabled
	 */
	if (cmd == NL80211_CMD_ADV_TTLM_ENDED || bcn_links)
		queue_work(system_highpri_wq, &dev->attlm_work);

	/* a cancelled mapping runs on in the firmware until its end, which
	 * cfg80211 still waits for
	 */
	if (!event.cookie)
		return;

	ieee80211_adv_ttlm_notify(vif, &event, GFP_ATOMIC);
}

static void
mt7996_mcu_mld_attlm_notify(void *priv, u8 *mac, struct ieee80211_vif *vif)
{
	struct mt7996_vif *mvif = (struct mt7996_vif *)vif->drv_priv;
	struct mt7996_mld_attlm_data *data = priv;
	bool match, early, stale_end;

	if (vif->type != NL80211_IFTYPE_AP || !ieee80211_vif_is_mld(vif) ||
	    !ether_addr_equal(vif->addr, data->mld_addr))
		return;

	spin_lock_bh(&data->dev->mt76.lock);
	match = data->idx == mvif->attlm_idx;
	early = match && mvif->attlm_requesting;
	if (early && data->cmd == NL80211_CMD_ADV_TTLM_ESTABLISHED)
		mvif->attlm_early_switch = true;
	if (early && data->cmd == NL80211_CMD_ADV_TTLM_ENDED)
		mvif->attlm_early_end = true;
	/* the end timer can end a mapping before the firmware lifts the
	 * pause of its negotiated mappings
	 */
	stale_end = !match && data->cmd == NL80211_CMD_ADV_TTLM_ENDED;
	if (stale_end)
		mvif->attlm_ttlm_resend = true;
	spin_unlock_bh(&data->dev->mt76.lock);

	if (stale_end)
		queue_work(system_highpri_wq, &data->dev->attlm_work);

	if (match && !early)
		mt7996_mcu_mld_attlm_step(data->dev, vif, data->cmd);
}

static void
mt7996_mcu_mld_attlm_timeout(struct mt7996_dev *dev, const u8 *mld_addr,
			     struct tlv *tlv)
{
	struct mt7996_mcu_mld_attlm_timeout_event *timeout = (void *)tlv;
	struct mt7996_mld_attlm_data data = {
		.dev = dev,
		.mld_addr = mld_addr,
	};

	if (le16_to_cpu(tlv->len) < sizeof(*timeout))
		return;

	data.idx = timeout->attlm_idx;
	switch (timeout->event_type) {
	case MLD_ATTLM_TIMEOUT_SWITCH_TIME:
		data.cmd = NL80211_CMD_ADV_TTLM_ESTABLISHED;
		break;
	case MLD_ATTLM_TIMEOUT_DURATION:
		data.cmd = NL80211_CMD_ADV_TTLM_ENDED;
		break;
	default:
		return;
	}

	ieee80211_iterate_active_interfaces_atomic(mt76_hw(dev),
			IEEE80211_IFACE_ITER_RESUME_ALL,
			mt7996_mcu_mld_attlm_notify, &data);
}

struct mt7996_mld_attlm_end_data {
	struct mt7996_dev *dev;
	struct ieee80211_vif *vif;
};

static void
mt7996_mcu_mld_attlm_end_sta_iter(void *priv, struct ieee80211_sta *sta)
{
	struct mt7996_sta *msta = (struct mt7996_sta *)sta->drv_priv;
	struct mt7996_mld_attlm_end_data *data = priv;

	if (msta->vif != (struct mt7996_vif *)data->vif->drv_priv ||
	    !sta->mlo || !msta->neg_ttlm.valid)
		return;

	mt7996_mcu_peer_mld_ttlm_req(data->dev, data->vif, sta,
				     &msta->neg_ttlm);
}

static void
mt7996_mcu_mld_attlm_beacons(struct mt7996_dev *dev, struct ieee80211_vif *vif,
			     unsigned long links, unsigned long disc_links)
{
	struct ieee80211_hw *hw = mt76_hw(dev);
	unsigned int link_id;

	for_each_set_bit(link_id, &links, IEEE80211_MLD_MAX_NUM_LINKS) {
		struct ieee80211_bss_conf *link_conf;

		link_conf = link_conf_dereference_protected(vif, link_id);
		if (link_conf && link_conf->enable_beacon)
			mt7996_mcu_add_beacon(hw, vif, link_conf, true);
	}

	for_each_set_bit(link_id, &disc_links, IEEE80211_MLD_MAX_NUM_LINKS) {
		struct mt7996_vif_link *link = mt7996_vif_link(dev, vif, link_id);
		struct ieee80211_bss_conf *link_conf;

		link_conf = link_conf_dereference_protected(vif, link_id);
		if (link_conf && link)
			mt7996_mcu_beacon_inband_discov(dev, link_conf, link);
	}
}

/* the time to the end of the mapping in the firmware, in ns; a link with no
 * channel has no TSF to read, so its end falls back to the boot time
 */
static s64
mt7996_mcu_mld_attlm_end_rest(struct mt7996_dev *dev, struct ieee80211_vif *vif)
{
	struct mt7996_vif *mvif = (struct mt7996_vif *)vif->drv_priv;
	u64 end, end_bound, end_tsf, now;
	u8 link_id, band_idx, omac_idx;
	struct mt7996_vif_link *link;
	s64 rest;

	spin_lock_bh(&dev->mt76.lock);
	end = mvif->attlm_end_boottime;
	end_bound = mvif->attlm_end_bound;
	end_tsf = mvif->attlm_end_tsf;
	link_id = mvif->attlm_ref_link;
	band_idx = mvif->attlm_ref_band;
	omac_idx = mvif->attlm_ref_omac;
	spin_unlock_bh(&dev->mt76.lock);

	if (!end)
		return 0;

	/* the mapping belongs to the BSS of the request in the firmware */
	link = mt7996_vif_link(dev, vif, link_id);
	if (!link || link->mt76.band_idx != band_idx ||
	    link->mt76.omac_idx != omac_idx)
		return 0;

	now = ktime_get_boottime_ns();
	if (mt7996_vif_link_phy(link))
		rest = (s64)(end_tsf - __mt7996_get_tsf(mt76_hw(dev), link)) *
		       NSEC_PER_USEC;
	else
		rest = (s64)(end - now);

	return min_t(s64, rest, (s64)(end_bound - now));
}

/* the end event can get lost, so the driver stop of the disabled links and
 * the mapping for cfg80211 also end at the end time, and the negotiated
 * mappings go to the firmware again
 */
static void
mt7996_mcu_mld_attlm_end_check(struct mt7996_dev *dev,
			       struct ieee80211_vif *vif)
{
	struct mt7996_vif *mvif = (struct mt7996_vif *)vif->drv_priv;
	struct cfg80211_adv_ttlm_event event = {
		.cmd = NL80211_CMD_ADV_TTLM_ENDED,
	};
	bool check;
	s64 rest;

	spin_lock_bh(&dev->mt76.lock);
	check = mvif->attlm_end_check;
	mvif->attlm_end_check = false;
	spin_unlock_bh(&dev->mt76.lock);

	if (!check)
		return;

	rest = mt7996_mcu_mld_attlm_end_rest(dev, vif);
	if (rest > 0) {
		hrtimer_start(&mvif->attlm_end_timer,
			      ns_to_ktime(ktime_get_boottime_ns() + rest),
			      HRTIMER_MODE_ABS_SOFT);
		return;
	}

	spin_lock_bh(&dev->mt76.lock);
	if (mvif->attlm_end_boottime) {
		mt7996_mcu_mld_attlm_stop_end(mvif);
		event.cookie = mt7996_mcu_mld_attlm_cookie_take(mvif);
		mvif->attlm_end_boottime = 0;
		mvif->attlm_ttlm_resend = true;
	}
	spin_unlock_bh(&dev->mt76.lock);

	if (event.cookie)
		ieee80211_adv_ttlm_notify(vif, &event, GFP_KERNEL);
}

void mt7996_mcu_mld_attlm_work(struct work_struct *work)
{
	struct mt7996_dev *dev = container_of(work, struct mt7996_dev,
					      attlm_work);
	struct mt7996_mld_attlm_end_data data = {
		.dev = dev,
	};
	struct ieee80211_hw *hw = mt76_hw(dev);
	struct ieee80211_vif *vif;

	wiphy_lock(hw->wiphy);
	for_each_active_interface(vif, hw) {
		struct mt7996_vif *mvif = (struct mt7996_vif *)vif->drv_priv;
		u16 bcn_links, disc_links;
		bool resend;

		if (vif->type != NL80211_IFTYPE_AP || !ieee80211_vif_is_mld(vif))
			continue;

		mutex_lock(&dev->mt76.mutex);
		mt7996_mcu_mld_attlm_end_check(dev, vif);

		spin_lock_bh(&dev->mt76.lock);
		resend = mvif->attlm_ttlm_resend;
		mvif->attlm_ttlm_resend = false;
		bcn_links = mvif->attlm_bcn_links & vif->valid_links;
		mvif->attlm_bcn_links = 0;
		disc_links = mvif->attlm_disc_sync & vif->valid_links;
		mvif->attlm_disc_sync = 0;
		spin_unlock_bh(&dev->mt76.lock);

		data.vif = vif;
		if (resend)
			ieee80211_iterate_stations_mtx(hw,
					mt7996_mcu_mld_attlm_end_sta_iter,
					&data);
		mt7996_mcu_mld_attlm_beacons(dev, vif, bcn_links, disc_links);
		mutex_unlock(&dev->mt76.mutex);
	}
	wiphy_unlock(hw->wiphy);
}

/* the TSF read takes mt76_dev.mutex, so the work judges the end */
enum hrtimer_restart mt7996_mcu_mld_attlm_end(struct hrtimer *timer)
{
	struct mt7996_vif *mvif = container_of(timer, struct mt7996_vif,
					       attlm_end_timer);
	struct mt7996_dev *dev = mvif->reconf_dev;
	bool check;

	spin_lock(&dev->mt76.lock);
	check = !!mvif->attlm_end_boottime;
	if (check)
		mvif->attlm_end_check = true;
	spin_unlock(&dev->mt76.lock);

	if (check)
		queue_work(system_highpri_wq, &dev->attlm_work);

	return HRTIMER_NORESTART;
}

struct mt7996_mld_reconf_data {
	struct mt7996_dev *dev;
	const u8 *mld_addr;
	u16 links;
};

/* the firmware starts the count of a template at its pre-TBTT pass, this
 * long before a TBTT of the link
 */
#define MT7996_PRE_TBTT_US	8192

/* slack for the conversion of the removal TBTT to boot time */
#define MT7996_RECONF_EVENT_MARGIN_US	20000

/* The firmware raises the expiry event at the pre-TBTT pass one beacon
 * interval before the removal TBTT, so a longer lead marks a removal TBTT one
 * interval late. A removal TBTT one interval early gives no event, since the
 * TX stop ends the count first. A lead below the pass means an event more
 * than one interval after its raise, or a TX stop more than 11.8 ms late.
 */
static void
mt7996_mcu_mld_reconf_notify(void *priv, u8 *mac, struct ieee80211_vif *vif)
{
	struct mt7996_vif *mvif = (struct mt7996_vif *)vif->drv_priv;
	struct mt7996_mld_reconf_data *data = priv;
	struct ieee80211_bss_conf *link_conf;
	unsigned int link_id;
	s64 lead_us, max_us;
	u16 links;
	u64 tbtt;

	if (vif->type != NL80211_IFTYPE_AP || !ieee80211_vif_is_mld(vif) ||
	    !ether_addr_equal(vif->addr, data->mld_addr))
		return;

	spin_lock_bh(&data->dev->mt76.lock);
	links = data->links & mvif->reconf_tbtt_links;
	tbtt = mvif->reconf_tbtt_boottime;
	spin_unlock_bh(&data->dev->mt76.lock);

	if (!links)
		return;

	link_id = __ffs(links);
	link_conf = rcu_dereference(vif->link_conf[link_id]);
	if (!link_conf || !link_conf->beacon_int)
		return;

	/* The firmware's timer runs in seconds from the timer command, and its
	 * event names a link id of 0 to 7 alone, so the event only checks the
	 * removal TBTT that the driver derived.
	 */
	max_us = link_conf->beacon_int * 1024 + MT7996_PRE_TBTT_US +
		 MT7996_RECONF_EVENT_MARGIN_US;
	lead_us = div_s64((s64)(tbtt - ktime_get_boottime_ns()), NSEC_PER_USEC);
	if (lead_us < MT7996_PRE_TBTT_US || lead_us > max_us)
		dev_warn(data->dev->mt76.dev,
			 "expiry event of links 0x%x %lld us before the removal TBTT\n",
			 links, lead_us);
	else
		dev_dbg(data->dev->mt76.dev,
			"expiry event of links 0x%x %lld us before the removal TBTT\n",
			links, lead_us);
}

/* the TX stop takes a few ms; this lead leaves room for the work and the
 * wiphy lock before the removal TBTT
 */
#define MT7996_RECONF_STOP_LEAD_US	20000

enum hrtimer_restart mt7996_mcu_mld_reconf_tbtt(struct hrtimer *timer)
{
	struct mt7996_vif *mvif = container_of(timer, struct mt7996_vif,
					       reconf_timer);
	struct mt7996_dev *dev = mvif->reconf_dev;
	u16 links;

	spin_lock(&dev->mt76.lock);
	links = mvif->reconf_timer_links & mvif->reconf_links;
	mvif->reconf_links &= ~links;
	if (!mvif->reconf_links)
		mvif->reconf_bcn_links = 0;
	mvif->reconf_stop_links |= links;
	mvif->reconf_timer_links = 0;
	spin_unlock(&dev->mt76.lock);

	if (links)
		queue_work(system_highpri_wq, &dev->reconf_work);

	return HRTIMER_NORESTART;
}

static int
__mt7996_mcu_add_beacon(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
			struct ieee80211_bss_conf *link_conf,
			struct mt7996_vif_link *link, bool enabled,
			u16 strip_links);

struct mt7996_mld_reconf_sta_data {
	struct mt7996_dev *dev;
	struct mt7996_vif *mvif;
	u16 links;
};

static void
mt7996_mcu_mld_reconf_sta_iter(void *priv, struct ieee80211_sta *sta)
{
	struct mt7996_sta *msta = (struct mt7996_sta *)sta->drv_priv;
	struct mt7996_mld_reconf_sta_data *data = priv;

	if (msta->vif == data->mvif)
		mt7996_sta_tx_links_release(data->dev, sta, data->links);
}

/* The removed links stop their Beacon frames, and MLD stations move their TX
 * to the other links, just before the removal TBTT; user space disassociates
 * the non-MLD stations and stops the links after it. The other links drop
 * the profiles and the Reduced Neighbor Report entries of the removed links
 * from their templates here, since a new template from user space comes too
 * late for the removal TBTT.
 */
static bool
mt7996_mcu_mld_reconf_tx_stop(struct mt7996_dev *dev, struct ieee80211_vif *vif,
			      u16 links)
{
	struct mt7996_vif *mvif = (struct mt7996_vif *)vif->drv_priv;
	unsigned long valid_links = vif->valid_links;
	struct mt7996_mld_reconf_sta_data data = {
		.dev = dev,
		.mvif = mvif,
		.links = links,
	};
	struct ieee80211_hw *hw = mt76_hw(dev);
	unsigned int link_id;
	u16 attlm_stopped;

	mutex_lock(&dev->mt76.mutex);
	/* a full reset drops the links until the restart adds them again */
	if (links & ~mvif->mt76.valid_links) {
		mutex_unlock(&dev->mt76.mutex);
		return false;
	}

	spin_lock_bh(&dev->mt76.lock);
	mvif->reconf_done_links |= links;
	attlm_stopped = mvif->attlm_stop_links;
	spin_unlock_bh(&dev->mt76.lock);

	ieee80211_iterate_stations_mtx(hw, mt7996_mcu_mld_reconf_sta_iter,
				       &data);

	for_each_set_bit(link_id, &valid_links, IEEE80211_MLD_MAX_NUM_LINKS) {
		struct ieee80211_bss_conf *link_conf;

		link_conf = link_conf_dereference_protected(vif, link_id);
		if (link_conf && (links & BIT(link_id)))
			mt7996_mcu_add_beacon(hw, vif, link_conf, false);
	}
	for_each_set_bit(link_id, &valid_links, IEEE80211_MLD_MAX_NUM_LINKS) {
		struct ieee80211_bss_conf *link_conf;
		struct mt7996_vif_link *link;

		if ((links | attlm_stopped) & BIT(link_id))
			continue;

		link_conf = link_conf_dereference_protected(vif, link_id);
		link = mt7996_vif_link(dev, vif, link_id);
		if (link_conf && link && link_conf->enable_beacon &&
		    !link_conf->nontransmitted)
			__mt7996_mcu_add_beacon(hw, vif, link_conf, link, true,
						links);
	}
	mt7996_mcu_mld_reconf_stop_link(dev, vif, links);
	mutex_unlock(&dev->mt76.mutex);

	return true;
}

static void
mt7996_mcu_mld_reconf_links_stop(struct mt7996_dev *dev,
				 struct ieee80211_vif *vif)
{
	struct mt7996_vif *mvif = (struct mt7996_vif *)vif->drv_priv;
	u16 links, reported;

	spin_lock_bh(&dev->mt76.lock);
	links = mvif->reconf_stop_links & vif->valid_links;
	mvif->reconf_stop_links = 0;
	reported = mvif->reconf_done_links;
	spin_unlock_bh(&dev->mt76.lock);

	if (!links)
		return;

	if (!mt7996_mcu_mld_reconf_tx_stop(dev, vif, links)) {
		spin_lock_bh(&dev->mt76.lock);
		mvif->reconf_stop_links |= links;
		spin_unlock_bh(&dev->mt76.lock);
		return;
	}

	/* a full reset stops the stopped links again */
	links &= ~reported;
	if (links)
		ieee80211_links_removed(vif, links);
}

/* the template that started the wait goes up first, so that the templates of
 * the other links of the removal follow it in the same interval
 */
static void
mt7996_mcu_mld_reconf_links_start(struct mt7996_dev *dev,
				  struct ieee80211_vif *vif)
{
	struct mt7996_vif *mvif = (struct mt7996_vif *)vif->drv_priv;
	struct ieee80211_hw *hw = mt76_hw(dev);
	unsigned int first;
	u16 links;

	spin_lock_bh(&dev->mt76.lock);
	links = mvif->reconf_wait_links & vif->valid_links;
	first = mvif->reconf_wait_link;
	mvif->reconf_wait_links = 0;
	spin_unlock_bh(&dev->mt76.lock);

	mutex_lock(&dev->mt76.mutex);
	while (links) {
		unsigned int link_id = links & BIT(first) ? first : __ffs(links);
		struct ieee80211_bss_conf *link_conf;

		links &= ~BIT(link_id);
		link_conf = link_conf_dereference_protected(vif, link_id);
		if (link_conf && link_conf->enable_beacon)
			mt7996_mcu_add_beacon(hw, vif, link_conf, true);
	}
	mutex_unlock(&dev->mt76.mutex);
}

void mt7996_mcu_mld_reconf_work(struct work_struct *work)
{
	struct mt7996_dev *dev = container_of(work, struct mt7996_dev,
					      reconf_work);
	struct ieee80211_hw *hw = mt76_hw(dev);
	struct ieee80211_vif *vif;

	wiphy_lock(hw->wiphy);
	for_each_active_interface(vif, hw) {
		if (vif->type != NL80211_IFTYPE_AP || !ieee80211_vif_is_mld(vif))
			continue;

		mt7996_mcu_mld_reconf_links_stop(dev, vif);
		mt7996_mcu_mld_reconf_links_start(dev, vif);
	}
	wiphy_unlock(hw->wiphy);
}

enum hrtimer_restart mt7996_mcu_mld_reconf_window(struct hrtimer *timer)
{
	struct mt7996_vif *mvif = container_of(timer, struct mt7996_vif,
					       reconf_start_timer);

	queue_work(system_highpri_wq, &mvif->reconf_dev->reconf_work);

	return HRTIMER_NORESTART;
}

static void
mt7996_mcu_mld_reconf_expired(struct mt7996_dev *dev, const u8 *mld_addr,
			      struct tlv *tlv)
{
	struct mt7996_mcu_mld_reconf_event *reconf = (void *)tlv;
	struct mt7996_mld_reconf_data data = {
		.dev = dev,
		.mld_addr = mld_addr,
	};

	if (le16_to_cpu(tlv->len) < sizeof(*reconf))
		return;

	data.links = le16_to_cpu(reconf->link_bitmap);
	ieee80211_iterate_active_interfaces_atomic(mt76_hw(dev),
			IEEE80211_IFACE_ITER_RESUME_ALL,
			mt7996_mcu_mld_reconf_notify, &data);
}

static void
mt7996_mcu_rx_mld_event(struct mt7996_dev *dev, struct sk_buff *skb)
{
	struct mt7996_mcu_mld_event *event = (void *)skb->data;
	u8 *pos = event->buf;
	int len;

	if (skb->len < sizeof(*event))
		return;

	len = skb->len - sizeof(*event);
	while (len >= sizeof(struct tlv)) {
		struct tlv *tlv = (struct tlv *)pos;
		u16 tlv_len = le16_to_cpu(tlv->len);

		if (tlv_len < sizeof(*tlv) || tlv_len > len)
			break;

		switch (le16_to_cpu(tlv->tag)) {
		case UNI_EVENT_MLD_ATTLM_TIMEOUT:
			mt7996_mcu_mld_attlm_timeout(dev, event->mld_addr, tlv);
			break;
		case UNI_EVENT_MLD_RECONF_AP_REM_TIMER:
			mt7996_mcu_mld_reconf_expired(dev, event->mld_addr, tlv);
			break;
		default:
			break;
		}

		pos += tlv_len;
		len -= tlv_len;
	}
}

static void
mt7996_mcu_uni_rx_unsolicited_event(struct mt7996_dev *dev, struct sk_buff *skb)
{
	struct mt7996_mcu_rxd *rxd = (struct mt7996_mcu_rxd *)skb->data;

	switch (rxd->eid) {
	case MCU_UNI_EVENT_FW_LOG_2_HOST:
		mt7996_mcu_rx_log_message(dev, skb);
		break;
	case MCU_UNI_EVENT_IE_COUNTDOWN:
		mt7996_mcu_ie_countdown(dev, skb);
		break;
	case MCU_UNI_EVENT_RDD_REPORT:
		mt7996_mcu_rx_radar_detected(dev, skb);
		break;
	case MCU_UNI_EVENT_ALL_STA_INFO:
		mt7996_mcu_rx_all_sta_info_event(dev, skb);
		break;
	case MCU_UNI_EVENT_WED_RRO:
		mt7996_mcu_wed_rro_event(dev, skb);
		break;
	case MCU_UNI_EVENT_PS_SYNC:
		mt7996_mcu_rx_ps_sync(dev, skb);
		break;
	case MCU_UNI_EVENT_BSS_INFO:
		mt7996_mcu_rx_bss_event(dev, skb);
		break;
	case MCU_UNI_EVENT_MLD:
		mt7996_mcu_rx_mld_event(dev, skb);
		break;
	default:
		break;
	}
	dev_kfree_skb(skb);
}

void mt7996_mcu_rx_event(struct mt7996_dev *dev, struct sk_buff *skb)
{
	struct mt7996_mcu_rxd *rxd = (struct mt7996_mcu_rxd *)skb->data;

	if (skb->len < offsetofend(struct mt7996_mcu_rxd, ext_eid)) {
		dev_kfree_skb(skb);
		return;
	}

	if (rxd->option & MCU_UNI_CMD_UNSOLICITED_EVENT) {
		mt7996_mcu_uni_rx_unsolicited_event(dev, skb);
		return;
	}

	/* WA still uses legacy event*/
	if (rxd->ext_eid == MCU_EXT_EVENT_FW_LOG_2_HOST ||
	    !rxd->seq)
		mt7996_mcu_rx_unsolicited_event(dev, skb);
	else
		mt76_mcu_rx_event(&dev->mt76, skb);
}

static struct tlv *
mt7996_mcu_add_uni_tlv(struct sk_buff *skb, u16 tag, u16 len)
{
	struct tlv *ptlv = skb_put_zero(skb, len);

	ptlv->tag = cpu_to_le16(tag);
	ptlv->len = cpu_to_le16(len);

	return ptlv;
}

static void
mt7996_mcu_bss_rfch_tlv(struct sk_buff *skb, struct mt7996_phy *phy)
{
	static const u8 rlm_ch_band[] = {
		[NL80211_BAND_2GHZ] = 1,
		[NL80211_BAND_5GHZ] = 2,
		[NL80211_BAND_6GHZ] = 3,
	};
	struct cfg80211_chan_def *chandef = &phy->mt76->chandef;
	struct bss_rlm_tlv *ch;
	struct tlv *tlv;
	int freq1 = chandef->center_freq1;

	tlv = mt7996_mcu_add_uni_tlv(skb, UNI_BSS_INFO_RLM, sizeof(*ch));

	ch = (struct bss_rlm_tlv *)tlv;
	ch->control_channel = chandef->chan->hw_value;
	ch->center_chan = ieee80211_frequency_to_channel(freq1);
	ch->bw = mt76_connac_chan_bw(chandef);
	ch->tx_streams = hweight8(phy->mt76->antenna_mask);
	ch->rx_streams = hweight8(phy->mt76->antenna_mask);
	ch->band = rlm_ch_band[chandef->chan->band];

	if (chandef->width == NL80211_CHAN_WIDTH_80P80) {
		int freq2 = chandef->center_freq2;

		ch->center_chan2 = ieee80211_frequency_to_channel(freq2);
	}
}

static void
mt7996_mcu_bss_ra_tlv(struct sk_buff *skb, struct mt7996_phy *phy)
{
	struct bss_ra_tlv *ra;
	struct tlv *tlv;

	tlv = mt7996_mcu_add_uni_tlv(skb, UNI_BSS_INFO_RA, sizeof(*ra));

	ra = (struct bss_ra_tlv *)tlv;
	ra->short_preamble = true;
}

static void
mt7996_mcu_bss_he_tlv(struct sk_buff *skb, struct ieee80211_vif *vif,
		      struct ieee80211_bss_conf *link_conf,
		      struct mt7996_phy *phy)
{
#define DEFAULT_HE_PE_DURATION		4
#define DEFAULT_HE_DURATION_RTS_THRES	1023
	const struct ieee80211_sta_he_cap *cap;
	struct bss_info_uni_he *he;
	struct tlv *tlv;

	cap = mt76_connac_get_he_phy_cap(phy->mt76, vif);
	if (!cap)
		return;

	tlv = mt7996_mcu_add_uni_tlv(skb, UNI_BSS_INFO_HE_BASIC, sizeof(*he));

	he = (struct bss_info_uni_he *)tlv;
	he->he_pe_duration = link_conf->htc_trig_based_pkt_ext;
	if (!he->he_pe_duration)
		he->he_pe_duration = DEFAULT_HE_PE_DURATION;

	he->he_rts_thres = cpu_to_le16(link_conf->frame_time_rts_th);
	if (!he->he_rts_thres)
		he->he_rts_thres = cpu_to_le16(DEFAULT_HE_DURATION_RTS_THRES);

	he->max_nss_mcs[CMD_HE_MCS_BW80] = cap->he_mcs_nss_supp.tx_mcs_80;
	he->max_nss_mcs[CMD_HE_MCS_BW160] = cap->he_mcs_nss_supp.tx_mcs_160;
	he->max_nss_mcs[CMD_HE_MCS_BW8080] = cap->he_mcs_nss_supp.tx_mcs_80p80;
}

static void
mt7996_mcu_bss_mbssid_tlv(struct sk_buff *skb, struct ieee80211_bss_conf *link_conf,
			  bool enable)
{
	struct bss_info_uni_mbssid *mbssid;
	struct tlv *tlv;

	if (!link_conf->bssid_indicator && enable)
		return;

	tlv = mt7996_mcu_add_uni_tlv(skb, UNI_BSS_INFO_11V_MBSSID, sizeof(*mbssid));

	mbssid = (struct bss_info_uni_mbssid *)tlv;

	if (enable) {
		mbssid->max_indicator = link_conf->bssid_indicator;
		mbssid->mbss_idx = link_conf->bssid_index;
		mbssid->tx_bss_omac_idx = 0;
	}
}

static void
mt7996_mcu_bss_bmc_tlv(struct sk_buff *skb, struct mt76_vif_link *mlink,
		       struct mt7996_phy *phy)
{
	struct bss_rate_tlv *bmc;
	struct cfg80211_chan_def *chandef = &phy->mt76->chandef;
	enum nl80211_band band = chandef->chan->band;
	struct tlv *tlv;
	u8 idx = mlink->mcast_rates_idx ?
		 mlink->mcast_rates_idx : mlink->basic_rates_idx;

	tlv = mt7996_mcu_add_uni_tlv(skb, UNI_BSS_INFO_RATE, sizeof(*bmc));

	bmc = (struct bss_rate_tlv *)tlv;

	bmc->short_preamble = (band == NL80211_BAND_2GHZ);
	bmc->bc_fixed_rate = idx;
	bmc->mc_fixed_rate = idx;
}

static void
mt7996_mcu_bss_txcmd_tlv(struct sk_buff *skb, bool en)
{
	struct bss_txcmd_tlv *txcmd;
	struct tlv *tlv;

	tlv = mt7996_mcu_add_uni_tlv(skb, UNI_BSS_INFO_TXCMD, sizeof(*txcmd));

	txcmd = (struct bss_txcmd_tlv *)tlv;
	txcmd->txcmd_mode = en;
}

static void
mt7996_mcu_bss_mld_tlv(struct sk_buff *skb,
		       struct ieee80211_bss_conf *link_conf,
		       struct mt7996_vif_link *link)
{
	struct ieee80211_vif *vif = link_conf->vif;
	struct mt7996_vif *mvif = (struct mt7996_vif *)vif->drv_priv;
	struct bss_mld_tlv *mld;
	struct tlv *tlv;

	tlv = mt7996_mcu_add_uni_tlv(skb, UNI_BSS_INFO_MLD, sizeof(*mld));
	mld = (struct bss_mld_tlv *)tlv;
	mld->own_mld_id = link->mld_idx;
	mld->link_id = link_conf->link_id;

	if (ieee80211_vif_is_mld(vif) && !link->mt76.offchannel) {
		mld->group_mld_id = mvif->mld_group_idx;
		mld->remap_idx = mvif->mld_remap_idx;
		memcpy(mld->mac_addr, vif->addr, ETH_ALEN);
	} else {
		mld->group_mld_id = 0xff;
		mld->remap_idx = 0xff;
	}
}

static void
mt7996_mcu_bss_sec_tlv(struct sk_buff *skb, struct mt76_vif_link *mlink)
{
	struct bss_sec_tlv *sec;
	struct tlv *tlv;

	tlv = mt7996_mcu_add_uni_tlv(skb, UNI_BSS_INFO_SEC, sizeof(*sec));

	sec = (struct bss_sec_tlv *)tlv;
	sec->cipher = mlink->cipher;
}

static int
mt7996_mcu_muar_config(struct mt7996_dev *dev, struct mt76_vif_link *mlink,
		       const u8 *addr, bool bssid, bool enable)
{
#define UNI_MUAR_ENTRY 2
	u32 idx = mlink->omac_idx - REPEATER_BSSID_START;
	struct {
		struct {
			u8 band;
			u8 __rsv[3];
		} hdr;

		__le16 tag;
		__le16 len;

		bool smesh;
		u8 bssid;
		u8 index;
		u8 entry_add;
		u8 addr[ETH_ALEN];
		u8 __rsv[2];
	} __packed req = {
		.hdr.band = mlink->band_idx,
		.tag = cpu_to_le16(UNI_MUAR_ENTRY),
		.len = cpu_to_le16(sizeof(req) - sizeof(req.hdr)),
		.smesh = false,
		.index = idx * 2 + bssid,
		.entry_add = true,
	};

	if (enable)
		memcpy(req.addr, addr, ETH_ALEN);

	return mt76_mcu_send_msg(&dev->mt76, MCU_WM_UNI_CMD(REPT_MUAR), &req,
				 sizeof(req), true);
}

static void
mt7996_mcu_bss_ifs_timing_tlv(struct sk_buff *skb, struct mt7996_phy *phy)
{
	struct bss_ifs_time_tlv *ifs_time;
	struct tlv *tlv;
	bool is_2ghz = phy->mt76->chandef.chan->band == NL80211_BAND_2GHZ;

	tlv = mt7996_mcu_add_uni_tlv(skb, UNI_BSS_INFO_IFS_TIME, sizeof(*ifs_time));

	ifs_time = (struct bss_ifs_time_tlv *)tlv;
	ifs_time->slot_valid = true;
	ifs_time->sifs_valid = true;
	ifs_time->rifs_valid = true;
	ifs_time->eifs_valid = true;

	ifs_time->slot_time = cpu_to_le16(phy->slottime);
	ifs_time->sifs_time = cpu_to_le16(10);
	ifs_time->rifs_time = cpu_to_le16(2);
	ifs_time->eifs_time = cpu_to_le16(is_2ghz ? 78 : 84);

	if (is_2ghz) {
		ifs_time->eifs_cck_valid = true;
		ifs_time->eifs_cck_time = cpu_to_le16(314);
	}
}

static int
mt7996_mcu_bss_basic_tlv(struct sk_buff *skb,
			 struct ieee80211_vif *vif,
			 struct ieee80211_bss_conf *link_conf,
			 struct mt76_vif_link *mvif,
			 struct mt76_phy *phy, u16 wlan_idx,
			 bool enable)
{
	struct cfg80211_chan_def *chandef = &phy->chandef;
	struct mt76_connac_bss_basic_tlv *bss;
	u32 type = CONNECTION_INFRA_AP;
	u16 sta_wlan_idx = wlan_idx;
	struct tlv *tlv;
	int idx;

	switch (vif->type) {
	case NL80211_IFTYPE_MESH_POINT:
	case NL80211_IFTYPE_AP:
	case NL80211_IFTYPE_MONITOR:
		break;
	case NL80211_IFTYPE_STATION:
		if (enable) {
			struct ieee80211_sta *sta;

			rcu_read_lock();
			sta = ieee80211_find_sta(vif, link_conf->bssid);
			if (sta) {
				struct mt7996_sta *msta = (void *)sta->drv_priv;
				struct mt7996_sta_link *msta_link;
				int link_id = link_conf->link_id;

				msta_link = mt7996_sta_link(msta, link_id);
				if (msta_link)
					sta_wlan_idx = msta_link->wcid.idx;
			}
			rcu_read_unlock();
		}
		type = CONNECTION_INFRA_STA;
		break;
	case NL80211_IFTYPE_ADHOC:
		type = CONNECTION_IBSS_ADHOC;
		break;
	default:
		WARN_ON(1);
		break;
	}

	tlv = mt7996_mcu_add_uni_tlv(skb, UNI_BSS_INFO_BASIC, sizeof(*bss));

	bss = (struct mt76_connac_bss_basic_tlv *)tlv;
	bss->bmc_tx_wlan_idx = cpu_to_le16(wlan_idx);
	bss->sta_idx = cpu_to_le16(sta_wlan_idx);
	bss->conn_type = cpu_to_le32(type);
	bss->omac_idx = mvif->omac_idx;
	bss->band_idx = mvif->band_idx;
	bss->wmm_idx = mvif->wmm_idx;
	bss->conn_state = !enable;
	bss->active = enable;

	idx = mvif->omac_idx > EXT_BSSID_START ? HW_BSSID_0 : mvif->omac_idx;
	bss->hw_bss_idx = idx;

	if (vif->type == NL80211_IFTYPE_MONITOR) {
		memcpy(bss->bssid, phy->macaddr, ETH_ALEN);
		return 0;
	}

	memcpy(bss->bssid, link_conf->bssid, ETH_ALEN);
	bss->bcn_interval = cpu_to_le16(link_conf->beacon_int);
	bss->dtim_period = link_conf->dtim_period;
	bss->phymode = mt76_connac_get_phy_mode(phy, vif,
						chandef->chan->band, NULL);
	bss->phymode_ext = mt76_connac_get_phy_mode_ext(phy, link_conf,
							chandef->chan->band);

	return 0;
}

static struct sk_buff *
__mt7996_mcu_alloc_bss_req(struct mt76_dev *dev, struct mt76_vif_link *mvif, int len)
{
	struct bss_req_hdr hdr = {
		.bss_idx = mvif->idx,
	};
	struct sk_buff *skb;

	skb = mt76_mcu_msg_alloc(dev, NULL, len);
	if (!skb)
		return ERR_PTR(-ENOMEM);

	skb_put_data(skb, &hdr, sizeof(hdr));

	return skb;
}

int mt7996_mcu_add_bss_info(struct mt7996_phy *phy, struct ieee80211_vif *vif,
			    struct ieee80211_bss_conf *link_conf,
			    struct mt76_vif_link *mlink,
			    struct mt7996_sta_link *msta_link, int enable)
{
	struct mt7996_dev *dev = phy->dev;
	struct sk_buff *skb;

	if (mlink->omac_idx >= REPEATER_BSSID_START) {
		mt7996_mcu_muar_config(dev, mlink, link_conf->addr, false, enable);
		mt7996_mcu_muar_config(dev, mlink, link_conf->bssid, true, enable);
	}

	skb = __mt7996_mcu_alloc_bss_req(&dev->mt76, mlink,
					 MT7996_BSS_UPDATE_MAX_SIZE);
	if (IS_ERR(skb))
		return PTR_ERR(skb);

	/* bss_basic must be first */
	mt7996_mcu_bss_basic_tlv(skb, vif, link_conf, mlink, phy->mt76,
				 msta_link->wcid.idx, enable);
	mt7996_mcu_bss_sec_tlv(skb, mlink);

	if (vif->type == NL80211_IFTYPE_MONITOR)
		goto out;

	if (enable) {
		struct mt7996_vif_link *link;

		mt7996_mcu_bss_rfch_tlv(skb, phy);
		mt7996_mcu_bss_bmc_tlv(skb, mlink, phy);
		mt7996_mcu_bss_ra_tlv(skb, phy);
		mt7996_mcu_bss_txcmd_tlv(skb, true);
		mt7996_mcu_bss_ifs_timing_tlv(skb, phy);

		if (vif->bss_conf.he_support)
			mt7996_mcu_bss_he_tlv(skb, vif, link_conf, phy);

		/* this tag is necessary no matter if the vif is MLD */
		link = container_of(mlink, struct mt7996_vif_link, mt76);
		mt7996_mcu_bss_mld_tlv(skb, link_conf, link);
	}

	mt7996_mcu_bss_mbssid_tlv(skb, link_conf, enable);

out:
	return mt76_mcu_skb_send_msg(&dev->mt76, skb,
				     MCU_WMWA_UNI_CMD(BSS_INFO_UPDATE), true);
}

int mt7996_mcu_update_bss_rfch(struct mt7996_phy *phy, struct mt7996_vif_link *link)
{
	struct mt7996_dev *dev = phy->dev;
	struct sk_buff *skb;

	skb = __mt7996_mcu_alloc_bss_req(&dev->mt76, &link->mt76,
					 MT7996_BSS_UPDATE_MAX_SIZE);
	if (IS_ERR(skb))
		return PTR_ERR(skb);

	mt7996_mcu_bss_rfch_tlv(skb, phy);

	return mt76_mcu_skb_send_msg(&dev->mt76, skb,
				     MCU_WMWA_UNI_CMD(BSS_INFO_UPDATE), true);
}

int mt7996_mcu_set_protection(struct mt7996_phy *phy, struct mt7996_vif_link *link,
			      u8 ht_mode, bool use_cts_prot)
{
	struct mt7996_dev *dev = phy->dev;
	struct bss_prot_tlv *prot;
	struct sk_buff *skb;
	struct tlv *tlv;
	enum {
		PROT_NONMEMBER	 = BIT(1),
		PROT_20MHZ	 = BIT(2),
		PROT_NONHT_MIXED = BIT(3),
		PROT_LEGACY_ERP	 = BIT(5),
		PROT_NONGF_STA	 = BIT(7),
	};

	/* The current firmware causes TX failure. Need further investigation */
	if (is_mt7996(&dev->mt76))
		return 0;

	skb = __mt7996_mcu_alloc_bss_req(&dev->mt76, &link->mt76,
					 MT7996_BSS_UPDATE_MAX_SIZE);
	if (IS_ERR(skb))
		return PTR_ERR(skb);

	tlv = mt7996_mcu_add_uni_tlv(skb, UNI_BSS_INFO_PROTECT_INFO,
				     sizeof(*prot));
	prot = (struct bss_prot_tlv *)tlv;

	switch (ht_mode & IEEE80211_HT_OP_MODE_PROTECTION) {
	case IEEE80211_HT_OP_MODE_PROTECTION_NONMEMBER:
		prot->prot_mode = cpu_to_le32(PROT_NONMEMBER);
		break;
	case IEEE80211_HT_OP_MODE_PROTECTION_20MHZ:
		prot->prot_mode = cpu_to_le32(PROT_20MHZ);
		break;
	case IEEE80211_HT_OP_MODE_PROTECTION_NONHT_MIXED:
		prot->prot_mode = cpu_to_le32(PROT_NONHT_MIXED);
		break;
	}

	if (ht_mode & IEEE80211_HT_OP_MODE_NON_GF_STA_PRSNT)
		prot->prot_mode |= cpu_to_le32(PROT_NONGF_STA);

	if (use_cts_prot)
		prot->prot_mode |= cpu_to_le32(PROT_LEGACY_ERP);

	return mt76_mcu_skb_send_msg(&dev->mt76, skb,
				     MCU_WM_UNI_CMD(BSS_INFO_UPDATE), true);
}

int mt7996_mcu_set_emlsr_mode(struct mt7996_dev *dev,
			      struct ieee80211_vif *vif,
			      struct ieee80211_sta *sta,
			      struct ieee80211_eml_params *eml_params)
{
	struct mt7996_sta *msta = (struct mt7996_sta *)sta->drv_priv;
	struct mt7996_sta_link *msta_link;
	struct sta_rec_eml_op *eml_op;
	struct mt7996_vif_link *link;
	struct sk_buff *skb;
	struct tlv *tlv;

	msta_link = mt76_dereference(msta->link[eml_params->link_id],
				     &dev->mt76);
	if (!msta_link)
		return -EINVAL;

	link = mt7996_vif_link(dev, vif, eml_params->link_id);
	if (!link)
		return -EINVAL;

	skb = __mt76_connac_mcu_alloc_sta_req(&dev->mt76, &link->mt76,
					      &msta_link->wcid,
					      MT7996_STA_UPDATE_MAX_SIZE);
	if (IS_ERR(skb))
		return PTR_ERR(skb);

	tlv = mt76_connac_mcu_add_tlv(skb, STA_REC_EML_OP, sizeof(*eml_op));
	eml_op = (struct sta_rec_eml_op *)tlv;
	eml_op->link_bitmap = 0;

	if (eml_params->control & IEEE80211_EML_CTRL_EMLSR_MODE) {
		unsigned long link_bitmap = eml_params->link_bitmap;
		unsigned int link_id;

		for_each_set_bit(link_id, &link_bitmap,
				 IEEE80211_MLD_MAX_NUM_LINKS) {
			struct mt76_phy *mphy;

			link = mt7996_vif_link(dev, vif, link_id);
			if (!link)
				continue;

			mphy = mt76_vif_link_phy(&link->mt76);
			if (!mphy)
				continue;

			eml_op->link_bitmap |= BIT(mphy->band_idx);
		}
	}

	return mt76_mcu_skb_send_msg(&dev->mt76, skb,
				     MCU_WMWA_UNI_CMD(STA_REC_UPDATE), true);
}

int mt7996_mcu_set_timing(struct mt7996_phy *phy, struct ieee80211_vif *vif,
			  struct ieee80211_bss_conf *link_conf)
{
	struct mt7996_dev *dev = phy->dev;
	struct mt76_vif_link *mlink = mt76_vif_conf_link(&dev->mt76, vif, link_conf);
	struct sk_buff *skb;

	skb = __mt7996_mcu_alloc_bss_req(&dev->mt76, mlink,
					 MT7996_BSS_UPDATE_MAX_SIZE);
	if (IS_ERR(skb))
		return PTR_ERR(skb);

	mt7996_mcu_bss_ifs_timing_tlv(skb, phy);

	return mt76_mcu_skb_send_msg(&dev->mt76, skb,
				     MCU_WMWA_UNI_CMD(BSS_INFO_UPDATE), true);
}

static int
mt7996_mcu_sta_ba(struct mt7996_dev *dev, struct mt76_vif_link *mvif,
		  struct ieee80211_ampdu_params *params,
		  struct mt76_wcid *wcid, bool enable, bool tx)
{
	struct sta_rec_ba_uni *ba;
	struct sk_buff *skb;
	struct tlv *tlv;

	skb = __mt76_connac_mcu_alloc_sta_req(&dev->mt76, mvif, wcid,
					      MT7996_STA_UPDATE_MAX_SIZE);
	if (IS_ERR(skb))
		return PTR_ERR(skb);

	tlv = mt76_connac_mcu_add_tlv(skb, STA_REC_BA, sizeof(*ba));

	ba = (struct sta_rec_ba_uni *)tlv;
	ba->ba_type = tx ? MT_BA_TYPE_ORIGINATOR : MT_BA_TYPE_RECIPIENT;
	ba->winsize = cpu_to_le16(params->buf_size);
	ba->ssn = cpu_to_le16(params->ssn);
	ba->ba_en = enable << params->tid;
	ba->amsdu = params->amsdu;
	ba->tid = params->tid;
	ba->ba_rdd_rro = !tx && enable && mt7996_has_hwrro(dev);

	return mt76_mcu_skb_send_msg(&dev->mt76, skb,
				     MCU_WMWA_UNI_CMD(STA_REC_UPDATE), true);
}

/** starec & wtbl **/
int mt7996_mcu_add_tx_ba(struct mt7996_dev *dev,
			 struct ieee80211_ampdu_params *params,
			 struct ieee80211_vif *vif, unsigned long links,
			 bool enable)
{
	struct ieee80211_sta *sta = params->sta;
	struct mt7996_sta *msta = (struct mt7996_sta *)sta->drv_priv;
	unsigned int link_id;
	int ret = 0;

	for_each_set_bit(link_id, &links, IEEE80211_MLD_MAX_NUM_LINKS) {
		struct mt7996_sta_link *msta_link;
		struct mt7996_vif_link *link;

		msta_link = mt7996_sta_link_protected(dev, msta, link_id);
		if (!msta_link)
			continue;

		link = mt7996_vif_link(dev, vif, link_id);
		if (!link)
			continue;

		if (enable && !params->amsdu)
			msta_link->wcid.amsdu = false;

		ret = mt7996_mcu_sta_ba(dev, &link->mt76, params,
					&msta_link->wcid, enable, true);
		if (ret)
			break;
	}

	return ret;
}

int mt7996_mcu_add_rx_ba(struct mt7996_dev *dev,
			 struct ieee80211_ampdu_params *params,
			 struct ieee80211_vif *vif, unsigned long links,
			 bool enable)
{
	struct ieee80211_sta *sta = params->sta;
	struct mt7996_sta *msta = (struct mt7996_sta *)sta->drv_priv;
	unsigned int link_id;
	int ret = 0;

	for_each_set_bit(link_id, &links, IEEE80211_MLD_MAX_NUM_LINKS) {
		struct mt7996_sta_link *msta_link;
		struct mt7996_vif_link *link;

		msta_link = mt7996_sta_link_protected(dev, msta, link_id);
		if (!msta_link)
			continue;

		link = mt7996_vif_link(dev, vif, link_id);
		if (!link)
			continue;

		ret = mt7996_mcu_sta_ba(dev, &link->mt76, params,
					&msta_link->wcid, enable, false);
		if (ret)
			break;
	}

	return ret;
}

static void
mt7996_mcu_sta_he_tlv(struct sk_buff *skb,
		      struct ieee80211_link_sta *link_sta,
		      struct mt7996_vif_link *link)
{
	struct ieee80211_he_cap_elem *elem = &link_sta->he_cap.he_cap_elem;
	struct ieee80211_he_mcs_nss_supp mcs_map;
	struct sta_rec_he_v2 *he;
	struct tlv *tlv;
	int i = 0;

	if (!link_sta->he_cap.has_he)
		return;

	tlv = mt76_connac_mcu_add_tlv(skb, STA_REC_HE_V2, sizeof(*he));

	he = (struct sta_rec_he_v2 *)tlv;
	for (i = 0; i < 11; i++) {
		if (i < 6)
			he->he_mac_cap[i] = elem->mac_cap_info[i];
		he->he_phy_cap[i] = elem->phy_cap_info[i];
	}

	mcs_map = link_sta->he_cap.he_mcs_nss_supp;
	switch (link_sta->bandwidth) {
	case IEEE80211_STA_RX_BW_160:
		if (elem->phy_cap_info[0] &
		    IEEE80211_HE_PHY_CAP0_CHANNEL_WIDTH_SET_80PLUS80_MHZ_IN_5G)
			mt7996_mcu_set_sta_he_mcs(link_sta, link,
						  &he->max_nss_mcs[CMD_HE_MCS_BW8080],
						  le16_to_cpu(mcs_map.rx_mcs_80p80));

		mt7996_mcu_set_sta_he_mcs(link_sta, link,
					  &he->max_nss_mcs[CMD_HE_MCS_BW160],
					  le16_to_cpu(mcs_map.rx_mcs_160));
		fallthrough;
	default:
		mt7996_mcu_set_sta_he_mcs(link_sta, link,
					  &he->max_nss_mcs[CMD_HE_MCS_BW80],
					  le16_to_cpu(mcs_map.rx_mcs_80));
		break;
	}

	he->pkt_ext = 2;
}

static void
mt7996_mcu_sta_he_6g_tlv(struct sk_buff *skb,
			 struct ieee80211_link_sta *link_sta)
{
	struct sta_rec_he_6g_capa *he_6g;
	struct tlv *tlv;

	if (!link_sta->he_6ghz_capa.capa)
		return;

	tlv = mt76_connac_mcu_add_tlv(skb, STA_REC_HE_6G, sizeof(*he_6g));

	he_6g = (struct sta_rec_he_6g_capa *)tlv;
	he_6g->capa = link_sta->he_6ghz_capa.capa;
}

static void
mt7996_mcu_sta_eht_tlv(struct sk_buff *skb,
		       struct ieee80211_link_sta *link_sta)
{
	struct mt7996_sta *msta = (struct mt7996_sta *)link_sta->sta->drv_priv;
	struct ieee80211_vif *vif = container_of((void *)msta->vif,
						 struct ieee80211_vif, drv_priv);
	struct ieee80211_eht_mcs_nss_supp *mcs_map;
	struct ieee80211_eht_cap_elem_fixed *elem;
	struct sta_rec_eht *eht;
	struct tlv *tlv;

	if (!link_sta->eht_cap.has_eht)
		return;

	mcs_map = &link_sta->eht_cap.eht_mcs_nss_supp;
	elem = &link_sta->eht_cap.eht_cap_elem;

	tlv = mt76_connac_mcu_add_tlv(skb, STA_REC_EHT, sizeof(*eht));

	eht = (struct sta_rec_eht *)tlv;
	eht->tid_bitmap = 0xff;
	eht->mac_cap = cpu_to_le16(*(u16 *)elem->mac_cap_info);
	eht->phy_cap = cpu_to_le64(*(u64 *)elem->phy_cap_info);
	eht->phy_cap_ext = cpu_to_le64(elem->phy_cap_info[8]);

	if (vif->type != NL80211_IFTYPE_STATION &&
	    (link_sta->he_cap.he_cap_elem.phy_cap_info[0] &
	     (IEEE80211_HE_PHY_CAP0_CHANNEL_WIDTH_SET_40MHZ_IN_2G |
	      IEEE80211_HE_PHY_CAP0_CHANNEL_WIDTH_SET_40MHZ_80MHZ_IN_5G |
	      IEEE80211_HE_PHY_CAP0_CHANNEL_WIDTH_SET_160MHZ_IN_5G |
	      IEEE80211_HE_PHY_CAP0_CHANNEL_WIDTH_SET_80PLUS80_MHZ_IN_5G)) == 0) {
		memcpy(eht->mcs_map_bw20, &mcs_map->only_20mhz,
		       sizeof(eht->mcs_map_bw20));
		return;
	}

	memcpy(eht->mcs_map_bw80, &mcs_map->bw._80, sizeof(eht->mcs_map_bw80));
	memcpy(eht->mcs_map_bw160, &mcs_map->bw._160, sizeof(eht->mcs_map_bw160));
	memcpy(eht->mcs_map_bw320, &mcs_map->bw._320, sizeof(eht->mcs_map_bw320));
}

static void
mt7996_mcu_sta_ht_tlv(struct sk_buff *skb, struct ieee80211_link_sta *link_sta)
{
	struct sta_rec_ht_uni *ht;
	struct tlv *tlv;

	if (!link_sta->ht_cap.ht_supported)
		return;

	tlv = mt76_connac_mcu_add_tlv(skb, STA_REC_HT, sizeof(*ht));

	ht = (struct sta_rec_ht_uni *)tlv;
	ht->ht_cap = cpu_to_le16(link_sta->ht_cap.cap);
	ht->ampdu_param = u8_encode_bits(link_sta->ht_cap.ampdu_factor,
					 IEEE80211_HT_AMPDU_PARM_FACTOR) |
			  u8_encode_bits(link_sta->ht_cap.ampdu_density,
					 IEEE80211_HT_AMPDU_PARM_DENSITY);
}

static void
mt7996_mcu_sta_vht_tlv(struct sk_buff *skb, struct ieee80211_link_sta *link_sta)
{
	struct sta_rec_vht *vht;
	struct tlv *tlv;

	/* For 6G band, this tlv is necessary to let hw work normally */
	if (!link_sta->he_6ghz_capa.capa && !link_sta->vht_cap.vht_supported)
		return;

	tlv = mt76_connac_mcu_add_tlv(skb, STA_REC_VHT, sizeof(*vht));

	vht = (struct sta_rec_vht *)tlv;
	vht->vht_cap = cpu_to_le32(link_sta->vht_cap.cap);
	vht->vht_rx_mcs_map = link_sta->vht_cap.vht_mcs.rx_mcs_map;
	vht->vht_tx_mcs_map = link_sta->vht_cap.vht_mcs.tx_mcs_map;
}

static void
mt7996_mcu_sta_amsdu_tlv(struct mt7996_dev *dev, struct sk_buff *skb,
			 struct ieee80211_vif *vif,
			 struct ieee80211_link_sta *link_sta,
			 struct mt7996_sta_link *msta_link)
{
	struct sta_rec_amsdu *amsdu;
	struct tlv *tlv;

	if (vif->type != NL80211_IFTYPE_STATION &&
	    vif->type != NL80211_IFTYPE_MESH_POINT &&
	    vif->type != NL80211_IFTYPE_AP)
		return;

	if (!link_sta->agg.max_amsdu_len)
		return;

	tlv = mt76_connac_mcu_add_tlv(skb, STA_REC_HW_AMSDU, sizeof(*amsdu));
	amsdu = (struct sta_rec_amsdu *)tlv;
	amsdu->max_amsdu_num = 8;
	amsdu->amsdu_en = true;
	msta_link->wcid.amsdu = true;

	switch (link_sta->agg.max_amsdu_len) {
	case IEEE80211_MAX_MPDU_LEN_VHT_11454:
		amsdu->max_mpdu_size =
			IEEE80211_VHT_CAP_MAX_MPDU_LENGTH_11454;
		return;
	case IEEE80211_MAX_MPDU_LEN_HT_7935:
	case IEEE80211_MAX_MPDU_LEN_VHT_7991:
		amsdu->max_mpdu_size = IEEE80211_VHT_CAP_MAX_MPDU_LENGTH_7991;
		return;
	default:
		amsdu->max_mpdu_size = IEEE80211_VHT_CAP_MAX_MPDU_LENGTH_3895;
		return;
	}
}

static void
mt7996_mcu_sta_muru_tlv(struct mt7996_dev *dev, struct sk_buff *skb,
			struct ieee80211_bss_conf *link_conf,
			struct ieee80211_link_sta *link_sta)
{
	struct ieee80211_he_cap_elem *elem = &link_sta->he_cap.he_cap_elem;
	struct sta_rec_muru *muru;
	struct tlv *tlv;

	if (link_conf->vif->type != NL80211_IFTYPE_STATION &&
	    link_conf->vif->type != NL80211_IFTYPE_AP)
		return;

	tlv = mt76_connac_mcu_add_tlv(skb, STA_REC_MURU, sizeof(*muru));

	muru = (struct sta_rec_muru *)tlv;
	muru->cfg.mimo_dl_en = link_conf->eht_mu_beamformer ||
			       link_conf->he_mu_beamformer ||
			       link_conf->vht_mu_beamformer ||
			       link_conf->vht_mu_beamformee;
	muru->cfg.ofdma_dl_en = true;

	if (link_sta->vht_cap.vht_supported)
		muru->mimo_dl.vht_mu_bfee =
			!!(link_sta->vht_cap.cap & IEEE80211_VHT_CAP_MU_BEAMFORMEE_CAPABLE);

	if (!link_sta->he_cap.has_he)
		return;

	muru->mimo_dl.partial_bw_dl_mimo =
		HE_PHY(CAP6_PARTIAL_BANDWIDTH_DL_MUMIMO, elem->phy_cap_info[6]);

	muru->mimo_ul.full_ul_mimo =
		HE_PHY(CAP2_UL_MU_FULL_MU_MIMO, elem->phy_cap_info[2]);
	muru->mimo_ul.partial_ul_mimo =
		HE_PHY(CAP2_UL_MU_PARTIAL_MU_MIMO, elem->phy_cap_info[2]);

	muru->ofdma_dl.punc_pream_rx =
		HE_PHY(CAP1_PREAMBLE_PUNC_RX_MASK, elem->phy_cap_info[1]);
	muru->ofdma_dl.he_20m_in_40m_2g =
		HE_PHY(CAP8_20MHZ_IN_40MHZ_HE_PPDU_IN_2G, elem->phy_cap_info[8]);
	muru->ofdma_dl.he_20m_in_160m =
		HE_PHY(CAP8_20MHZ_IN_160MHZ_HE_PPDU, elem->phy_cap_info[8]);
	muru->ofdma_dl.he_80m_in_160m =
		HE_PHY(CAP8_80MHZ_IN_160MHZ_HE_PPDU, elem->phy_cap_info[8]);

	muru->ofdma_ul.t_frame_dur =
		HE_MAC(CAP1_TF_MAC_PAD_DUR_MASK, elem->mac_cap_info[1]);
	muru->ofdma_ul.mu_cascading =
		HE_MAC(CAP2_MU_CASCADING, elem->mac_cap_info[2]);
	muru->ofdma_ul.uo_ra =
		HE_MAC(CAP3_OFDMA_RA, elem->mac_cap_info[3]);
	muru->ofdma_ul.rx_ctrl_frame_to_mbss =
		HE_MAC(CAP3_RX_CTRL_FRAME_TO_MULTIBSS, elem->mac_cap_info[3]);
}

/* mac80211 fills the beamforming fields of link_conf for an AP alone, so a
 * station takes the capabilities that it advertises on the band
 */
static bool
mt7996_link_eht_su_bf(struct ieee80211_bss_conf *link_conf,
		      struct ieee80211_supported_band *sband, bool bfee)
{
	u8 bit = bfee ? IEEE80211_EHT_PHY_CAP0_SU_BEAMFORMEE :
			IEEE80211_EHT_PHY_CAP0_SU_BEAMFORMER;
	const struct ieee80211_sta_eht_cap *cap;

	if (link_conf->vif->type == NL80211_IFTYPE_AP)
		return bfee ? link_conf->eht_su_beamformee :
			      link_conf->eht_su_beamformer;

	cap = ieee80211_get_eht_iftype_cap(sband, link_conf->vif->type);

	return cap && (cap->eht_cap_elem.phy_cap_info[0] & bit);
}

static bool
mt7996_link_he_su_bf(struct ieee80211_bss_conf *link_conf,
		     struct ieee80211_supported_band *sband, bool bfee)
{
	const struct ieee80211_sta_he_cap *cap;

	if (link_conf->vif->type == NL80211_IFTYPE_AP)
		return bfee ? link_conf->he_su_beamformee :
			      link_conf->he_su_beamformer;

	cap = ieee80211_get_he_iftype_cap(sband, link_conf->vif->type);
	if (!cap)
		return false;

	if (bfee)
		return HE_PHY(CAP4_SU_BEAMFORMEE, cap->he_cap_elem.phy_cap_info[4]);

	return HE_PHY(CAP3_SU_BEAMFORMER, cap->he_cap_elem.phy_cap_info[3]);
}

static bool
mt7996_link_vht_su_bf(struct ieee80211_bss_conf *link_conf,
		      struct ieee80211_supported_band *sband, bool bfee)
{
	u32 bit = bfee ? IEEE80211_VHT_CAP_SU_BEAMFORMEE_CAPABLE :
			 IEEE80211_VHT_CAP_SU_BEAMFORMER_CAPABLE;

	if (link_conf->vif->type == NL80211_IFTYPE_AP)
		return bfee ? link_conf->vht_su_beamformee :
			      link_conf->vht_su_beamformer;

	return sband->vht_cap.cap & bit;
}

static inline bool
mt7996_is_ebf_supported(struct mt7996_phy *phy,
			struct ieee80211_bss_conf *link_conf,
			struct ieee80211_link_sta *link_sta, bool bfee)
{
	int sts = hweight16(phy->mt76->chainmask);
	struct ieee80211_supported_band *sband;

	if (link_conf->vif->type != NL80211_IFTYPE_STATION &&
	    link_conf->vif->type != NL80211_IFTYPE_AP)
		return false;

	if (!bfee && sts < 2)
		return false;

	sband = phy->mt76->hw->wiphy->bands[phy->mt76->chandef.chan->band];

	if (link_sta->eht_cap.has_eht) {
		struct ieee80211_sta_eht_cap *pc = &link_sta->eht_cap;
		struct ieee80211_eht_cap_elem_fixed *pe = &pc->eht_cap_elem;

		if (bfee)
			return mt7996_link_eht_su_bf(link_conf, sband, true) &&
			       EHT_PHY(CAP0_SU_BEAMFORMER, pe->phy_cap_info[0]);
		else
			return mt7996_link_eht_su_bf(link_conf, sband, false) &&
			       EHT_PHY(CAP0_SU_BEAMFORMEE, pe->phy_cap_info[0]);
	}

	if (link_sta->he_cap.has_he) {
		struct ieee80211_he_cap_elem *pe = &link_sta->he_cap.he_cap_elem;

		if (bfee)
			return mt7996_link_he_su_bf(link_conf, sband, true) &&
			       HE_PHY(CAP3_SU_BEAMFORMER, pe->phy_cap_info[3]);
		else
			return mt7996_link_he_su_bf(link_conf, sband, false) &&
			       HE_PHY(CAP4_SU_BEAMFORMEE, pe->phy_cap_info[4]);
	}

	if (link_sta->vht_cap.vht_supported) {
		u32 cap = link_sta->vht_cap.cap;

		if (bfee)
			return mt7996_link_vht_su_bf(link_conf, sband, true) &&
			       (cap & IEEE80211_VHT_CAP_SU_BEAMFORMER_CAPABLE);
		else
			return mt7996_link_vht_su_bf(link_conf, sband, false) &&
			       (cap & IEEE80211_VHT_CAP_SU_BEAMFORMEE_CAPABLE);
	}

	return false;
}

static void
mt7996_mcu_sta_sounding_rate(struct sta_rec_bf *bf, struct mt7996_phy *phy)
{
	bf->sounding_phy = MT_PHY_TYPE_OFDM;
	bf->ndp_rate = 0;				/* mcs0 */
	if (is_mt7996(phy->mt76->dev))
		bf->ndpa_rate = MT7996_CFEND_RATE_DEFAULT;	/* ofdm 24m */
	else
		bf->ndpa_rate = MT7992_CFEND_RATE_DEFAULT;	/* ofdm 6m */

	bf->rept_poll_rate = MT7996_CFEND_RATE_DEFAULT;	/* ofdm 24m */
}

static void
mt7996_mcu_sta_bfer_ht(struct ieee80211_link_sta *link_sta,
		       struct mt7996_phy *phy, struct sta_rec_bf *bf,
		       bool explicit)
{
	struct ieee80211_mcs_info *mcs = &link_sta->ht_cap.mcs;
	u8 n = 0;

	bf->tx_mode = MT_PHY_TYPE_HT;

	if ((mcs->tx_params & IEEE80211_HT_MCS_TX_RX_DIFF) &&
	    (mcs->tx_params & IEEE80211_HT_MCS_TX_DEFINED))
		n = FIELD_GET(IEEE80211_HT_MCS_TX_MAX_STREAMS_MASK,
			      mcs->tx_params);
	else if (mcs->rx_mask[3])
		n = 3;
	else if (mcs->rx_mask[2])
		n = 2;
	else if (mcs->rx_mask[1])
		n = 1;

	bf->nrow = hweight8(phy->mt76->antenna_mask) - 1;
	bf->ncol = min_t(u8, bf->nrow, n);
	bf->ibf_ncol = explicit ? min_t(u8, MT7996_IBF_MAX_NC, bf->ncol) :
				  min_t(u8, MT7996_IBF_MAX_NC, n);
}

static void
mt7996_mcu_sta_bfer_vht(struct ieee80211_link_sta *link_sta,
			struct mt7996_phy *phy, struct sta_rec_bf *bf,
			bool explicit)
{
	struct ieee80211_sta_vht_cap *pc = &link_sta->vht_cap;
	struct ieee80211_sta_vht_cap *vc = &phy->mt76->sband_5g.sband.vht_cap;
	u16 mcs_map = le16_to_cpu(pc->vht_mcs.rx_mcs_map);
	u8 nss_mcs = mt7996_mcu_get_sta_nss(mcs_map);
	u8 tx_ant = hweight8(phy->mt76->antenna_mask) - 1;

	bf->tx_mode = MT_PHY_TYPE_VHT;

	if (explicit) {
		u8 sts, snd_dim;

		mt7996_mcu_sta_sounding_rate(bf, phy);

		sts = FIELD_GET(IEEE80211_VHT_CAP_BEAMFORMEE_STS_MASK,
				pc->cap);
		snd_dim = FIELD_GET(IEEE80211_VHT_CAP_SOUNDING_DIMENSIONS_MASK,
				    vc->cap);
		bf->nrow = min_t(u8, min_t(u8, snd_dim, sts), tx_ant);
		bf->ncol = min_t(u8, nss_mcs, bf->nrow);
		bf->ibf_ncol = min_t(u8, MT7996_IBF_MAX_NC, bf->ncol);

		if (link_sta->bandwidth == IEEE80211_STA_RX_BW_160)
			bf->nrow = 1;
	} else {
		bf->nrow = tx_ant;
		bf->ncol = min_t(u8, nss_mcs, bf->nrow);
		bf->ibf_ncol = min_t(u8, MT7996_IBF_MAX_NC, nss_mcs);

		if (link_sta->bandwidth == IEEE80211_STA_RX_BW_160)
			bf->ibf_nrow = 1;
	}
}

static void
mt7996_mcu_sta_bfer_he(struct ieee80211_link_sta *link_sta,
		       struct ieee80211_vif *vif, struct mt7996_phy *phy,
		       struct sta_rec_bf *bf, bool explicit)
{
	struct ieee80211_sta_he_cap *pc = &link_sta->he_cap;
	struct ieee80211_he_cap_elem *pe = &pc->he_cap_elem;
	u16 mcs_map = le16_to_cpu(pc->he_mcs_nss_supp.rx_mcs_80);
	u8 nss_mcs = mt7996_mcu_get_sta_nss(mcs_map);
	const struct ieee80211_he_cap_elem *ve;
	const struct ieee80211_sta_he_cap *vc;
	u8 snd_dim, sts;

	vc = mt76_connac_get_he_phy_cap(phy->mt76, vif);
	if (!vc)
		return;

	bf->tx_mode = MT_PHY_TYPE_HE_SU;
	ve = &vc->he_cap_elem;

	mt7996_mcu_sta_sounding_rate(bf, phy);

	bf->trigger_su = HE_PHY(CAP6_TRIG_SU_BEAMFORMING_FB,
				pe->phy_cap_info[6]);
	bf->trigger_mu = HE_PHY(CAP6_TRIG_MU_BEAMFORMING_PARTIAL_BW_FB,
				pe->phy_cap_info[6]);
	snd_dim = HE_PHY(CAP5_BEAMFORMEE_NUM_SND_DIM_UNDER_80MHZ_MASK,
			 ve->phy_cap_info[5]);
	sts = HE_PHY(CAP4_BEAMFORMEE_MAX_STS_UNDER_80MHZ_MASK,
		     pe->phy_cap_info[4]);
	bf->nrow = min_t(u8, snd_dim, sts);
	bf->ncol = min_t(u8, nss_mcs, bf->nrow);
	bf->ibf_ncol = explicit ? min_t(u8, MT7996_IBF_MAX_NC, bf->ncol) :
				  min_t(u8, MT7996_IBF_MAX_NC, nss_mcs);

	if (link_sta->bandwidth != IEEE80211_STA_RX_BW_160)
		return;

	/* go over for 160MHz and 80p80 */
	if (pe->phy_cap_info[0] &
	    IEEE80211_HE_PHY_CAP0_CHANNEL_WIDTH_SET_160MHZ_IN_5G) {
		mcs_map = le16_to_cpu(pc->he_mcs_nss_supp.rx_mcs_160);
		nss_mcs = mt7996_mcu_get_sta_nss(mcs_map);

		bf->ncol_gt_bw80 = nss_mcs;
	}

	if (pe->phy_cap_info[0] &
	    IEEE80211_HE_PHY_CAP0_CHANNEL_WIDTH_SET_80PLUS80_MHZ_IN_5G) {
		mcs_map = le16_to_cpu(pc->he_mcs_nss_supp.rx_mcs_80p80);
		nss_mcs = mt7996_mcu_get_sta_nss(mcs_map);

		if (bf->ncol_gt_bw80)
			bf->ncol_gt_bw80 = min_t(u8, bf->ncol_gt_bw80, nss_mcs);
		else
			bf->ncol_gt_bw80 = nss_mcs;
	}

	snd_dim = HE_PHY(CAP5_BEAMFORMEE_NUM_SND_DIM_ABOVE_80MHZ_MASK,
			 ve->phy_cap_info[5]);
	sts = HE_PHY(CAP4_BEAMFORMEE_MAX_STS_ABOVE_80MHZ_MASK,
		     pe->phy_cap_info[4]);

	bf->nrow_gt_bw80 = min_t(int, snd_dim, sts);
}

static void
mt7996_mcu_sta_bfer_eht(struct ieee80211_link_sta *link_sta,
			struct ieee80211_vif *vif, struct mt7996_phy *phy,
			struct sta_rec_bf *bf, bool explicit)
{
	struct ieee80211_sta_eht_cap *pc = &link_sta->eht_cap;
	struct ieee80211_eht_cap_elem_fixed *pe = &pc->eht_cap_elem;
	struct ieee80211_eht_mcs_nss_supp *eht_nss = &pc->eht_mcs_nss_supp;
	u8 nss_mcs = u8_get_bits(eht_nss->bw._80.rx_tx_mcs9_max_nss,
				 IEEE80211_EHT_MCS_NSS_RX);
	const struct ieee80211_eht_cap_elem_fixed *ve;
	const struct ieee80211_sta_eht_cap *vc;
	u8 snd_dim, sts;

	vc = mt76_connac_get_eht_phy_cap(phy->mt76, vif);
	if (!vc)
		return;

	bf->tx_mode = MT_PHY_TYPE_EHT_MU;
	ve = &vc->eht_cap_elem;

	mt7996_mcu_sta_sounding_rate(bf, phy);

	bf->trigger_su = EHT_PHY(CAP3_TRIG_SU_BF_FDBK, pe->phy_cap_info[3]);
	bf->trigger_mu = EHT_PHY(CAP3_TRIG_MU_BF_PART_BW_FDBK, pe->phy_cap_info[3]);
	snd_dim = EHT_PHY(CAP2_SOUNDING_DIM_80MHZ_MASK, ve->phy_cap_info[2]);
	sts = EHT_PHY(CAP0_BEAMFORMEE_SS_80MHZ_MASK, pe->phy_cap_info[0]) +
	      (EHT_PHY(CAP1_BEAMFORMEE_SS_80MHZ_MASK, pe->phy_cap_info[1]) << 1);
	nss_mcs = nss_mcs ? nss_mcs - 1 : 0;
	bf->nrow = min_t(u8, snd_dim, sts);
	bf->ncol = min_t(u8, nss_mcs, bf->nrow);
	bf->ibf_ncol = explicit ? min_t(u8, MT7996_IBF_MAX_NC, bf->ncol) :
				  min_t(u8, MT7996_IBF_MAX_NC, nss_mcs);

	if (link_sta->bandwidth < IEEE80211_STA_RX_BW_160)
		return;

	switch (link_sta->bandwidth) {
	case IEEE80211_STA_RX_BW_160:
		snd_dim = EHT_PHY(CAP2_SOUNDING_DIM_160MHZ_MASK, ve->phy_cap_info[2]);
		sts = EHT_PHY(CAP1_BEAMFORMEE_SS_160MHZ_MASK, pe->phy_cap_info[1]);
		nss_mcs = u8_get_bits(eht_nss->bw._160.rx_tx_mcs9_max_nss,
				      IEEE80211_EHT_MCS_NSS_RX);
		nss_mcs = nss_mcs ? nss_mcs - 1 : 0;

		bf->nrow_gt_bw80 = min_t(u8, snd_dim, sts);
		bf->ncol_gt_bw80 = nss_mcs;
		break;
	case IEEE80211_STA_RX_BW_320:
		snd_dim = EHT_PHY(CAP2_SOUNDING_DIM_320MHZ_MASK, ve->phy_cap_info[2]) +
			  (EHT_PHY(CAP3_SOUNDING_DIM_320MHZ_MASK,
				   ve->phy_cap_info[3]) << 1);
		sts = EHT_PHY(CAP1_BEAMFORMEE_SS_320MHZ_MASK, pe->phy_cap_info[1]);
		nss_mcs = u8_get_bits(eht_nss->bw._320.rx_tx_mcs9_max_nss,
				      IEEE80211_EHT_MCS_NSS_RX);
		nss_mcs = nss_mcs ? nss_mcs - 1 : 0;

		bf->nrow_gt_bw80 = min_t(u8, snd_dim, sts) << 4;
		bf->ncol_gt_bw80 = nss_mcs << 4;
		break;
	default:
		break;
	}
}

static bool
mt7996_mcu_sta_bfer_tlv(struct mt7996_dev *dev, struct sk_buff *skb,
			struct ieee80211_bss_conf *link_conf,
			struct ieee80211_link_sta *link_sta,
			struct mt7996_vif_link *link)
{
#define EBF_MODE	BIT(0)
#define IBF_MODE	BIT(1)
#define BF_MAT_ORDER	4
	struct mt7996_phy *phy = mt7996_vif_link_phy(link);
	struct ieee80211_vif *vif = link_conf->vif;
	struct sta_rec_bf *bf;
	struct tlv *tlv;
	static const u8 matrix[BF_MAT_ORDER][BF_MAT_ORDER] = {
		{0, 0, 0, 0},
		{1, 1, 0, 0},	/* 2x1, 2x2, 2x3, 2x4 */
		{2, 4, 4, 0},	/* 3x1, 3x2, 3x3, 3x4 */
		{3, 5, 6, 0}	/* 4x1, 4x2, 4x3, 4x4 */
	};
	int tx_ant;
	bool ebf;

	if (!phy)
		return false;

	if (!(link_sta->ht_cap.ht_supported || link_sta->he_cap.has_he))
		return false;

	ebf = mt7996_is_ebf_supported(phy, link_conf, link_sta, false);
	if (!ebf && !dev->ibf)
		return false;

	tlv = mt76_connac_mcu_add_tlv(skb, STA_REC_BF, sizeof(*bf));
	bf = (struct sta_rec_bf *)tlv;
	/* the firmware keeps this id where it allocates no profile, and a
	 * release frees the id that the entry holds
	 */
	bf->pfmu = cpu_to_le16(MT7996_BF_PFMU_NONE);

	/* he/eht: eBF only, except mt7992 that has 5T on 5GHz also supports iBF
	 * vht: support eBF and iBF
	 * ht: iBF only, since mac80211 lacks of eBF support
	 */
	if (link_sta->eht_cap.has_eht)
		mt7996_mcu_sta_bfer_eht(link_sta, vif, phy, bf, ebf);
	else if (link_sta->he_cap.has_he)
		mt7996_mcu_sta_bfer_he(link_sta, vif, phy, bf, ebf);
	else if (link_sta->vht_cap.vht_supported)
		mt7996_mcu_sta_bfer_vht(link_sta, phy, bf, ebf);
	else if (link_sta->ht_cap.ht_supported)
		mt7996_mcu_sta_bfer_ht(link_sta, phy, bf, ebf);
	else
		return true;

	bf->bf_cap = ebf ? EBF_MODE : (dev->ibf ? IBF_MODE : 0);
	tx_ant = hweight16(phy->mt76->chainmask) - 1;
	if (is_mt7992(&dev->mt76) && tx_ant == 4)
		bf->bf_cap |= IBF_MODE;

	bf->bw = link_sta->bandwidth;
	bf->ibf_dbw = link_sta->bandwidth;
	bf->ibf_nrow = tx_ant;

	if (link_sta->eht_cap.has_eht || link_sta->he_cap.has_he)
		bf->ibf_timeout = is_mt7992(&dev->mt76) ? MT7992_IBF_TIMEOUT :
							  MT7996_IBF_TIMEOUT;
	else if (!ebf && link_sta->bandwidth <= IEEE80211_STA_RX_BW_40 && !bf->ncol)
		bf->ibf_timeout = MT7996_IBF_TIMEOUT_LEGACY;
	else
		bf->ibf_timeout = MT7996_IBF_TIMEOUT;

	if (bf->ncol < BF_MAT_ORDER) {
		if (ebf)
			bf->mem_20m = tx_ant < BF_MAT_ORDER ?
				      matrix[tx_ant][bf->ncol] : 0;
		else
			bf->mem_20m = bf->nrow < BF_MAT_ORDER ?
				      matrix[bf->nrow][bf->ncol] : 0;
	}

	return true;
}

static void
mt7996_mcu_sta_bfee_tlv(struct mt7996_dev *dev, struct sk_buff *skb,
			struct ieee80211_bss_conf *link_conf,
			struct ieee80211_link_sta *link_sta,
			struct mt7996_vif_link *link)
{
	struct mt7996_phy *phy = mt7996_vif_link_phy(link);
	struct sta_rec_bfee *bfee;
	struct tlv *tlv;
	u8 nrow = 0;
	int tx_ant;

	if (!phy)
		return;

	if (!(link_sta->vht_cap.vht_supported || link_sta->he_cap.has_he))
		return;

	if (!mt7996_is_ebf_supported(phy, link_conf, link_sta, true))
		return;

	tlv = mt76_connac_mcu_add_tlv(skb, STA_REC_BFEE, sizeof(*bfee));
	bfee = (struct sta_rec_bfee *)tlv;

	if (link_sta->he_cap.has_he) {
		struct ieee80211_he_cap_elem *pe = &link_sta->he_cap.he_cap_elem;

		nrow = HE_PHY(CAP5_BEAMFORMEE_NUM_SND_DIM_UNDER_80MHZ_MASK,
			      pe->phy_cap_info[5]);
	} else if (link_sta->vht_cap.vht_supported) {
		struct ieee80211_sta_vht_cap *pc = &link_sta->vht_cap;

		nrow = FIELD_GET(IEEE80211_VHT_CAP_SOUNDING_DIMENSIONS_MASK,
				 pc->cap);
	}

	/* reply with identity matrix to avoid 2x2 BF negative gain; a station
	 * feeds back the channel that it measured
	 */
	tx_ant = hweight8(phy->mt76->antenna_mask) - 1;
	bfee->fb_identity_matrix = link_conf->vif->type == NL80211_IFTYPE_AP &&
				   nrow == 1 && tx_ant == 2;
}

static void
mt7996_mcu_sta_tx_proc_tlv(struct sk_buff *skb)
{
	struct sta_rec_tx_proc *tx_proc;
	struct tlv *tlv;

	tlv = mt76_connac_mcu_add_tlv(skb, STA_REC_TX_PROC, sizeof(*tx_proc));

	tx_proc = (struct sta_rec_tx_proc *)tlv;
	tx_proc->flag = cpu_to_le32(0);
}

static void
mt7996_mcu_sta_hdrt_tlv(struct mt7996_dev *dev, struct sk_buff *skb,
			u8 hdrt_mode)
{
	struct sta_rec_hdrt *hdrt;
	struct tlv *tlv;

	tlv = mt76_connac_mcu_add_tlv(skb, STA_REC_HDRT, sizeof(*hdrt));

	hdrt = (struct sta_rec_hdrt *)tlv;
	hdrt->hdrt_mode = hdrt_mode;
}

static void
mt7996_mcu_sta_hdr_trans_tlv(struct mt7996_dev *dev, struct sk_buff *skb,
			     struct ieee80211_vif *vif, struct mt76_wcid *wcid)
{
	struct sta_rec_hdr_trans *hdr_trans;
	struct tlv *tlv;

	tlv = mt76_connac_mcu_add_tlv(skb, STA_REC_HDR_TRANS, sizeof(*hdr_trans));
	hdr_trans = (struct sta_rec_hdr_trans *)tlv;
	hdr_trans->dis_rx_hdr_tran = true;

	if (vif->type == NL80211_IFTYPE_STATION)
		hdr_trans->to_ds = true;
	else
		hdr_trans->from_ds = true;

	if (!wcid)
		return;

	hdr_trans->dis_rx_hdr_tran = !test_bit(MT_WCID_FLAG_HDR_TRANS, &wcid->flags);
	if (test_bit(MT_WCID_FLAG_4ADDR, &wcid->flags)) {
		hdr_trans->to_ds = true;
		hdr_trans->from_ds = true;
	}

	if (vif->type == NL80211_IFTYPE_MESH_POINT) {
		hdr_trans->to_ds = true;
		hdr_trans->from_ds = true;
		hdr_trans->mesh = true;
	}

	if (test_bit(MT_WCID_FLAG_TDLS_PEER, &wcid->flags)) {
		hdr_trans->to_ds = false;
		hdr_trans->from_ds = false;
	}
}

static enum mcu_mmps_mode
mt7996_mcu_get_mmps_mode(enum ieee80211_smps_mode smps)
{
	switch (smps) {
	case IEEE80211_SMPS_OFF:
		return MCU_MMPS_DISABLE;
	case IEEE80211_SMPS_STATIC:
		return MCU_MMPS_STATIC;
	case IEEE80211_SMPS_DYNAMIC:
		return MCU_MMPS_DYNAMIC;
	default:
		return MCU_MMPS_DISABLE;
	}
}

int mt7996_mcu_set_fixed_rate_ctrl(struct mt7996_dev *dev,
				   void *data, u16 version)
{
	struct uni_header hdr = {};
	struct ra_fixed_rate *req;
	struct sk_buff *skb;
	struct tlv *tlv;
	int len;

	len = sizeof(hdr) + sizeof(*req);

	skb = mt76_mcu_msg_alloc(&dev->mt76, NULL, len);
	if (!skb)
		return -ENOMEM;

	skb_put_data(skb, &hdr, sizeof(hdr));

	tlv = mt7996_mcu_add_uni_tlv(skb, UNI_RA_FIXED_RATE, sizeof(*req));
	req = (struct ra_fixed_rate *)tlv;
	req->version = cpu_to_le16(version);
	memcpy(&req->rate, data, sizeof(req->rate));

	return mt76_mcu_skb_send_msg(&dev->mt76, skb,
				     MCU_WM_UNI_CMD(RA), true);
}

int mt7996_mcu_set_fixed_field(struct mt7996_dev *dev, struct mt7996_sta *msta,
			       void *data, u8 link_id, u32 field)
{
	struct mt7996_vif *mvif = msta->vif;
	struct mt7996_sta_link *msta_link;
	struct sta_rec_ra_fixed_uni *ra;
	struct sta_phy_uni *phy = data;
	struct mt76_vif_link *mlink;
	struct sk_buff *skb;
	int err = -ENODEV;
	struct tlv *tlv;

	rcu_read_lock();

	mlink = rcu_dereference(mvif->mt76.link[link_id]);
	if (!mlink)
		goto error_unlock;

	msta_link = mt7996_sta_link(msta, link_id);
	if (!msta_link)
		goto error_unlock;

	skb = __mt76_connac_mcu_alloc_sta_req(&dev->mt76, mlink,
					      &msta_link->wcid,
					      MT7996_STA_UPDATE_MAX_SIZE);
	if (IS_ERR(skb)) {
		err = PTR_ERR(skb);
		goto error_unlock;
	}

	tlv = mt76_connac_mcu_add_tlv(skb, STA_REC_RA_UPDATE, sizeof(*ra));
	ra = (struct sta_rec_ra_fixed_uni *)tlv;

	switch (field) {
	case RATE_PARAM_AUTO:
		break;
	case RATE_PARAM_FIXED:
	case RATE_PARAM_FIXED_MCS:
	case RATE_PARAM_FIXED_GI:
	case RATE_PARAM_FIXED_HE_LTF:
		if (phy)
			ra->phy = *phy;
		break;
	case RATE_PARAM_MMPS_UPDATE: {
		struct ieee80211_sta *sta = wcid_to_sta(&msta_link->wcid);
		struct ieee80211_link_sta *link_sta;

		link_sta = rcu_dereference(sta->link[link_id]);
		if (!link_sta) {
			dev_kfree_skb(skb);
			goto error_unlock;
		}

		ra->mmps_mode = mt7996_mcu_get_mmps_mode(link_sta->smps_mode);
		break;
	}
	default:
		break;
	}
	ra->field = cpu_to_le32(field);

	rcu_read_unlock();

	return mt76_mcu_skb_send_msg(&dev->mt76, skb,
				     MCU_WMWA_UNI_CMD(STA_REC_UPDATE), true);
error_unlock:
	rcu_read_unlock();

	return err;
}

static int
mt7996_mcu_add_rate_ctrl_fixed(struct mt7996_dev *dev, struct mt7996_sta *msta,
			       struct ieee80211_vif *vif, u8 link_id)
{
	struct ieee80211_link_sta *link_sta;
	struct cfg80211_bitrate_mask mask;
	struct mt7996_sta_link *msta_link;
	struct mt7996_vif_link *link;
	struct sta_phy_uni phy = {};
	struct ieee80211_sta *sta;
	int ret, nrates = 0;
	enum nl80211_band band;
	struct mt76_phy *mphy;
	bool has_he;

#define __sta_phy_bitrate_mask_check(_mcs, _gi, _ht, _he)			\
	do {									\
		u8 i, gi = mask.control[band]._gi;				\
		gi = (_he) ? gi : gi == NL80211_TXRATE_FORCE_SGI;		\
		phy.sgi = gi;							\
		phy.he_ltf = mask.control[band].he_ltf;				\
		for (i = 0; i < ARRAY_SIZE(mask.control[band]._mcs); i++) {	\
			if (!mask.control[band]._mcs[i])			\
				continue;					\
			nrates += hweight16(mask.control[band]._mcs[i]);	\
			phy.mcs = ffs(mask.control[band]._mcs[i]) - 1;		\
			if (_ht)						\
				phy.mcs += 8 * i;				\
		}								\
	} while (0)

	rcu_read_lock();

	link = mt7996_vif_link(dev, vif, link_id);
	if (!link)
		goto error_unlock;

	msta_link = mt7996_sta_link(msta, link_id);
	if (!msta_link)
		goto error_unlock;

	sta = wcid_to_sta(&msta_link->wcid);
	link_sta = rcu_dereference(sta->link[link_id]);
	if (!link_sta)
		goto error_unlock;

	mphy = mt76_vif_link_phy(&link->mt76);
	if (!mphy)
		goto error_unlock;

	band = mphy->chandef.chan->band;
	has_he = link_sta->he_cap.has_he;
	mask = link->bitrate_mask;

	if (has_he) {
		__sta_phy_bitrate_mask_check(he_mcs, he_gi, 0, 1);
	} else if (link_sta->vht_cap.vht_supported) {
		__sta_phy_bitrate_mask_check(vht_mcs, gi, 0, 0);
	} else if (link_sta->ht_cap.ht_supported) {
		__sta_phy_bitrate_mask_check(ht_mcs, gi, 1, 0);
	} else {
		nrates = hweight32(mask.control[band].legacy);
		phy.mcs = ffs(mask.control[band].legacy) - 1;
	}

	rcu_read_unlock();

#undef __sta_phy_bitrate_mask_check

	/* fall back to auto rate control */
	if (mask.control[band].gi == NL80211_TXRATE_DEFAULT_GI &&
	    mask.control[band].he_gi == GENMASK(7, 0) &&
	    mask.control[band].he_ltf == GENMASK(7, 0) &&
	    nrates != 1)
		return 0;

	/* fixed single rate */
	if (nrates == 1) {
		ret = mt7996_mcu_set_fixed_field(dev, msta, &phy, link_id,
						 RATE_PARAM_FIXED_MCS);
		if (ret)
			return ret;
	}

	/* fixed GI */
	if (mask.control[band].gi != NL80211_TXRATE_DEFAULT_GI ||
	    mask.control[band].he_gi != GENMASK(7, 0)) {
		ret = mt7996_mcu_set_fixed_field(dev, msta, &phy, link_id,
						 RATE_PARAM_FIXED_GI);
		if (ret)
			return ret;
	}

	/* fixed HE_LTF */
	if (mask.control[band].he_ltf != GENMASK(7, 0)) {
		ret = mt7996_mcu_set_fixed_field(dev, msta, &phy, link_id,
						 RATE_PARAM_FIXED_HE_LTF);
		if (ret)
			return ret;
	}

	return 0;

error_unlock:
	rcu_read_unlock();

	return -ENODEV;
}

static void
mt7996_mcu_sta_rate_ctrl_tlv(struct sk_buff *skb, struct mt7996_dev *dev,
			     struct ieee80211_vif *vif,
			     struct ieee80211_bss_conf *link_conf,
			     struct ieee80211_link_sta *link_sta,
			     struct mt7996_vif_link *link)
{
#define INIT_RCPI 180
	struct mt76_phy *mphy = mt76_vif_link_phy(&link->mt76);
	struct cfg80211_bitrate_mask *mask = &link->bitrate_mask;
	u32 cap = link_sta->sta->wme ? STA_CAP_WMM : 0;
	struct cfg80211_chan_def *chandef;
	struct sta_rec_ra_uni *ra;
	enum nl80211_band band;
	struct tlv *tlv;
	u32 supp_rate;

	if (!mphy)
		return;

	tlv = mt76_connac_mcu_add_tlv(skb, STA_REC_RA, sizeof(*ra));
	ra = (struct sta_rec_ra_uni *)tlv;

	chandef = &mphy->chandef;
	band = chandef->chan->band;
	supp_rate = link_sta->supp_rates[band];

	ra->valid = true;
	ra->auto_rate = true;
	ra->phy_mode = mt76_connac_get_phy_mode(mphy, vif, band, link_sta);
	ra->channel = chandef->chan->hw_value;
	ra->bw = (link_sta->bandwidth == IEEE80211_STA_RX_BW_320) ?
		 CMD_CBW_320MHZ : link_sta->bandwidth;
	ra->phy.bw = ra->bw;
	ra->mmps_mode = mt7996_mcu_get_mmps_mode(link_sta->smps_mode);

	if (supp_rate) {
		supp_rate &= mask->control[band].legacy;
		ra->rate_len = hweight32(supp_rate);

		if (band == NL80211_BAND_2GHZ) {
			ra->supp_mode = MODE_CCK;
			ra->supp_cck_rate = supp_rate & GENMASK(3, 0);

			if (ra->rate_len > 4) {
				ra->supp_mode |= MODE_OFDM;
				ra->supp_ofdm_rate = supp_rate >> 4;
			}
		} else {
			ra->supp_mode = MODE_OFDM;
			ra->supp_ofdm_rate = supp_rate;
		}
	}

	if (link_sta->ht_cap.ht_supported) {
		ra->supp_mode |= MODE_HT;
		ra->af = link_sta->ht_cap.ampdu_factor;
		ra->ht_gf = !!(link_sta->ht_cap.cap & IEEE80211_HT_CAP_GRN_FLD);

		cap |= STA_CAP_HT;
		if (link_sta->ht_cap.cap & IEEE80211_HT_CAP_SGI_20)
			cap |= STA_CAP_SGI_20;
		if (link_sta->ht_cap.cap & IEEE80211_HT_CAP_SGI_40)
			cap |= STA_CAP_SGI_40;
		if (link_sta->ht_cap.cap & IEEE80211_HT_CAP_TX_STBC)
			cap |= STA_CAP_TX_STBC;
		if (link_sta->ht_cap.cap & IEEE80211_HT_CAP_RX_STBC)
			cap |= STA_CAP_RX_STBC;
		if ((vif->type != NL80211_IFTYPE_AP || link_conf->ht_ldpc) &&
		    (link_sta->ht_cap.cap & IEEE80211_HT_CAP_LDPC_CODING))
			cap |= STA_CAP_LDPC;

		mt7996_mcu_set_sta_ht_mcs(link_sta, ra->ht_mcs,
					  mask->control[band].ht_mcs);
		ra->supp_ht_mcs = *(__le32 *)ra->ht_mcs;
	}

	if (link_sta->vht_cap.vht_supported) {
		u8 af;

		ra->supp_mode |= MODE_VHT;
		af = FIELD_GET(IEEE80211_VHT_CAP_MAX_A_MPDU_LENGTH_EXPONENT_MASK,
			       link_sta->vht_cap.cap);
		ra->af = max_t(u8, ra->af, af);

		cap |= STA_CAP_VHT;
		if (link_sta->vht_cap.cap & IEEE80211_VHT_CAP_SHORT_GI_80)
			cap |= STA_CAP_VHT_SGI_80;
		if (link_sta->vht_cap.cap & IEEE80211_VHT_CAP_SHORT_GI_160)
			cap |= STA_CAP_VHT_SGI_160;
		if (link_sta->vht_cap.cap & IEEE80211_VHT_CAP_TXSTBC)
			cap |= STA_CAP_VHT_TX_STBC;
		if (link_sta->vht_cap.cap & IEEE80211_VHT_CAP_RXSTBC_MASK)
			cap |= STA_CAP_VHT_RX_STBC;
		if ((vif->type != NL80211_IFTYPE_AP || link_conf->vht_ldpc) &&
		    (link_sta->vht_cap.cap & IEEE80211_VHT_CAP_RXLDPC))
			cap |= STA_CAP_VHT_LDPC;

		mt7996_mcu_set_sta_vht_mcs(link_sta, ra->supp_vht_mcs,
					   mask->control[band].vht_mcs);
	}

	if (link_sta->he_cap.has_he) {
		ra->supp_mode |= MODE_HE;
		cap |= STA_CAP_HE;

		if (link_sta->he_6ghz_capa.capa)
			ra->af = le16_get_bits(link_sta->he_6ghz_capa.capa,
					       IEEE80211_HE_6GHZ_CAP_MAX_AMPDU_LEN_EXP);
	}
	ra->sta_cap = cpu_to_le32(cap);

	memset(ra->rx_rcpi, INIT_RCPI, sizeof(ra->rx_rcpi));
}

int mt7996_mcu_add_rate_ctrl(struct mt7996_dev *dev, struct mt7996_sta *msta,
			     struct ieee80211_vif *vif, u8 link_id,
			     bool changed)
{
	struct ieee80211_bss_conf *link_conf;
	struct ieee80211_link_sta *link_sta;
	struct mt7996_sta_link *msta_link;
	struct mt7996_vif_link *link;
	struct ieee80211_sta *sta;
	struct sk_buff *skb;
	int ret = -ENODEV;

	rcu_read_lock();

	link = mt7996_vif_link(dev, vif, link_id);
	if (!link)
		goto error_unlock;

	msta_link = mt7996_sta_link(msta, link_id);
	if (!msta_link)
		goto error_unlock;

	sta = wcid_to_sta(&msta_link->wcid);
	link_sta = rcu_dereference(sta->link[link_id]);
	if (!link_sta)
		goto error_unlock;

	link_conf = rcu_dereference(vif->link_conf[link_id]);
	if (!link_conf)
		goto error_unlock;

	skb = __mt76_connac_mcu_alloc_sta_req(&dev->mt76, &link->mt76,
					      &msta_link->wcid,
					      MT7996_STA_UPDATE_MAX_SIZE);
	if (IS_ERR(skb)) {
		ret = PTR_ERR(skb);
		goto error_unlock;
	}

	/* firmware rc algorithm refers to sta_rec_he for HE control.
	 * once dev->rc_work changes the settings driver should also
	 * update sta_rec_he here.
	 */
	if (changed)
		mt7996_mcu_sta_he_tlv(skb, link_sta, link);

	/* sta_rec_ra accommodates BW, NSS and only MCS range format
	 * i.e 0-{7,8,9} for VHT.
	 */
	mt7996_mcu_sta_rate_ctrl_tlv(skb, dev, vif, link_conf, link_sta, link);

	rcu_read_unlock();

	ret = mt76_mcu_skb_send_msg(&dev->mt76, skb,
				    MCU_WMWA_UNI_CMD(STA_REC_UPDATE), true);
	if (ret)
		return ret;

	return mt7996_mcu_add_rate_ctrl_fixed(dev, msta, vif, link_id);

error_unlock:
	rcu_read_unlock();

	return ret;
}

/* Assignment of the STA BSS group index aligns FW. Each band has its own BSS
 * group bitmap space.
 * 0..3:  BSS 0..3
 * 4..18: BSS 0x11..0x1f
 */
static u8 mt7996_vow_sta_bss_grp(struct mt76_vif_link *mvif)
{
	u8 omac_idx = mvif->omac_idx;

	if (omac_idx <= HW_BSSID_MAX)
		return omac_idx;

	/* Extended BSS */
	return HW_BSSID_MAX + omac_idx - EXT_BSSID_START;
}

int mt7996_mcu_set_vow_drr_ctrl(struct mt7996_dev *dev, u8 band_idx,
				struct mt76_wcid *wcid, struct mt76_vif_link *mvif,
				enum vow_drr_ctrl_id id, u16 weight)
{
	u32 val = 0;
	struct {
		u8 __rsv1[4];

		__le16 tag;
		__le16 len;
		__le16 wlan_idx;
		u8 band_idx;
		u8 wmm_idx;
		__le32 ctrl_id;
		union {
			__le32 val;
			u8 drr_quantum[VOW_DRR_QUANTUM_NUM];
		};
		u8 __rsv2[3];
		u8 omac_idx;
	} __packed req = {
		.tag = cpu_to_le16(UNI_VOW_DRR_CTRL),
		.len = cpu_to_le16(sizeof(req) - 4),
		.wlan_idx = cpu_to_le16(wcid ? wcid->idx : 0),
		.band_idx = band_idx,
		.wmm_idx = mvif ? mvif->wmm_idx : 0,
		.ctrl_id = cpu_to_le32(id),
		.omac_idx = mvif ? mvif->omac_idx : 0,
	};

	switch (id) {
	case VOW_DRR_CTRL_STA_ALL:
		val |= FIELD_PREP(VOW_DRR_STA_BSS_GRP_MASK,
				  mt7996_vow_sta_bss_grp(mvif));
		/* the firmware replaces the whole station entry */
		val |= FIELD_PREP(VOW_DRR_STA_WMM_SET_MASK, mvif->wmm_idx);
		val |= FIELD_PREP(VOW_DRR_STA_AC0_QNTM_MASK,
				  mt76_connac_vow_dwrr_quantum(weight, IEEE80211_AC_BK));
		val |= FIELD_PREP(VOW_DRR_STA_AC1_QNTM_MASK,
				  mt76_connac_vow_dwrr_quantum(weight, IEEE80211_AC_BE));
		val |= FIELD_PREP(VOW_DRR_STA_AC2_QNTM_MASK,
				  mt76_connac_vow_dwrr_quantum(weight, IEEE80211_AC_VI));
		val |= FIELD_PREP(VOW_DRR_STA_AC3_QNTM_MASK,
				  mt76_connac_vow_dwrr_quantum(weight, IEEE80211_AC_VO));
		req.val = cpu_to_le32(val);
		break;
	case VOW_DRR_CTRL_AIRTIME_DEFICIT_BOUND:
		req.val = cpu_to_le32(dev->vow_atf_en ? VOW_MAX_DEFICIT_ON :
							VOW_MAX_DEFICIT_OFF);
		break;
	case VOW_DRR_CTRL_AIRTIME_QUANTUM_ALL: {
		static const u8 quantum[VOW_DRR_QUANTUM_NUM] = VOW_DRR_QUANTUM_TABLE;

		memcpy(req.drr_quantum, quantum, sizeof(quantum));
		break;
	}
	case VOW_DRR_CTRL_STA_PAUSE:
		/* req.val 0 keeps the station unpaused */
		break;
	default:
		return -EINVAL;
	}

	return mt76_mcu_send_msg(&dev->mt76, MCU_WM_UNI_CMD(VOW), &req,
				 sizeof(req), true);
}

int mt7996_mcu_set_vow_feature_ctrl(struct mt7996_phy *phy)
{
	struct mt76_connac_vow_feature f =
		mt76_connac_vow_feature_get(phy->dev->vow_atf_en);
	struct {
		u8 __rsv1[4];

		__le16 tag;
		__le16 len;

		/* DW0 */
		__le16 apply_bwc_enable_per_grp;
		__le16 apply;

		/* DW1 */
		__le16 apply_bwc_check_time_token_per_grp;
		__le16 __rsv5;

		/* DW2 */
		__le16 apply_bwc_check_len_token_per_grp;
		__le16 __rsv6;

		/* DW3 */
		u8 band_idx;
		u8 __rsv7[3];

		/* DW4 */
		__le32 __rsv8;

		/* DW5 */
		__le16 bwc_enable_per_grp;
		__le16 ctrl;

		/* DW6 */
		__le16 bwc_check_time_token_per_grp;
		__le16 __rsv12;

		/* DW7 */
		__le16 bwc_check_len_token_per_grp;
		__le16 __rsv13;

		/* DW8 */
		__le32 atf_ctrl;

		/* DW9 */
		__le32 sch_ctrl;
	} __packed req = {
		.tag = cpu_to_le16(UNI_VOW_FEATURE_CTRL),
		.len = cpu_to_le16(sizeof(req) - 4),
		.apply_bwc_enable_per_grp = cpu_to_le16(0xffff),
		.apply_bwc_check_time_token_per_grp = cpu_to_le16(0xffff),
		.apply_bwc_check_len_token_per_grp = cpu_to_le16(0xffff),
		.band_idx = phy->mt76->band_idx,
		.bwc_enable_per_grp = cpu_to_le16(0xffff),
	};

	req.apply = cpu_to_le16(f.apply);
	req.ctrl = cpu_to_le16(f.ctrl);
	req.atf_ctrl = cpu_to_le32(f.atf_ctrl);
	req.sch_ctrl = cpu_to_le32(f.sch_ctrl);

	return mt76_mcu_send_msg(&phy->dev->mt76, MCU_WM_UNI_CMD(VOW), &req,
				 sizeof(req), true);
}

/* The firmware reads the Service Start Time LinkID as the band of the
 * link whose wcid takes the SCS record, and counts the Service Start Time
 * in the TSF of the link of the record. A stream on another link than the
 * named one goes without both.
 */
static void
mt7996_mcu_scs_link_set(struct ieee80211_qos_char_elem *qc,
			const struct mt7996_scs_stream *stream,
			struct mt7996_sta_link *msta_link)
{
	int offset = mt7996_scs_stream_link_offset(stream);
	int start = offset - MT7996_SCS_START_LEN;
	int len = MT7996_SCS_START_LEN + 1;
	u8 *data = (u8 *)qc;

	if (!offset)
		return;

	if ((stream->qos_char[offset] & MT7996_SCS_LINK_ID) ==
	    msta_link->wcid.link_id) {
		data[offset] = msta_link->wcid.phy_idx;
		return;
	}

	memmove(&data[start], &data[start + len], stream->len - start - len);
	memset(&data[stream->len - len], 0, len);
	qc->length -= len;
	qc->control &=
		~cpu_to_le32(u32_encode_bits(IEEE80211_QOS_CHAR_PRES_SERVICE_START_TIME |
					     IEEE80211_QOS_CHAR_PRES_SERVICE_START_LINK_ID,
					     IEEE80211_QOS_CHAR_CTRL_PRESENCE));
}

int mt7996_mcu_set_scs(struct mt7996_dev *dev,
		       struct mt7996_sta_link *msta_link,
		       const struct mt7996_scs_stream *stream, bool del)
{
	const struct ieee80211_qos_char_elem *qc = (void *)stream->qos_char;
#define MURU_QOS_FLAG_UPDATE	20
#define MURU_QOS_FLAG_DELETE	21
	struct {
		u8 __rsv1[4];

		__le16 tag;
		__le16 len;
		__le32 qos_flag;
		__le16 wlan_idx;
		u8 ac;
		u8 __rsv2[11];
		u8 dir;
		u8 __rsv3[4];
		u8 scs_id;
		u8 qos_ie[MT7996_SCS_QOS_CHAR_LEN];
	} __packed req = {
		.tag = cpu_to_le16(UNI_CMD_MURU_SET_QOS_CFG),
		.len = cpu_to_le16(sizeof(req) - 4),
		.wlan_idx = cpu_to_le16(msta_link->wcid.idx),
		/* the firmware stores the TID as the AC of a stream entry and
		 * finds the entry by this field
		 */
		.ac = mt7996_qos_char_tid(qc),
		.scs_id = stream->fw_id,
	};

	if (del && req.ac >= IEEE80211_NUM_ACS) {
		struct ieee80211_qos_char_elem *entry_qc = (void *)req.qos_ie;

		/* The firmware deletes no stream entry whose AC is above 3.
		 * An add under the key of the entry stores TID 0 in it, and
		 * the add of a downlink stream leaves the SCS record alone.
		 */
		req.qos_flag = cpu_to_le32(MURU_QOS_FLAG_UPDATE);
		entry_qc->element_id = WLAN_EID_EXTENSION;
		entry_qc->length = sizeof(*entry_qc) - 2;
		entry_qc->element_id_ext = WLAN_EID_EXT_QOS_CHARACTERISTICS;
		entry_qc->control =
			cpu_to_le32(u32_encode_bits(IEEE80211_QOS_CHAR_DIR_DOWNLINK,
						    IEEE80211_QOS_CHAR_CTRL_DIRECTION));
		mt76_mcu_send_msg(&dev->mt76, MCU_WMWA_UNI_CMD(MURU), &req,
				  sizeof(req), true);
		memset(req.qos_ie, 0, sizeof(req.qos_ie));
	}

	if (del) {
		req.qos_flag = cpu_to_le32(MURU_QOS_FLAG_DELETE);
	} else {
		req.qos_flag = cpu_to_le32(MURU_QOS_FLAG_UPDATE);
		req.dir = ieee80211_qos_char_direction(qc);
		memcpy(req.qos_ie, stream->qos_char, stream->len);
		mt7996_mcu_scs_link_set((void *)req.qos_ie, stream, msta_link);
	}

	return mt76_mcu_send_msg(&dev->mt76, MCU_WMWA_UNI_CMD(MURU), &req,
				 sizeof(req), true);
}

int mt7996_mcu_sta_init_vow(struct mt7996_dev *dev, struct mt76_vif_link *mvif,
			    struct mt76_wcid *wcid, u16 weight)
{
	int ret;

	ret = mt7996_mcu_set_vow_drr_ctrl(dev, mvif->band_idx, wcid, mvif,
					  VOW_DRR_CTRL_STA_PAUSE, 0);
	if (ret)
		return ret;

	return mt7996_mcu_set_vow_drr_ctrl(dev, mvif->band_idx, wcid, mvif,
					   VOW_DRR_CTRL_STA_ALL, weight);
}

int mt7996_mcu_mld_reconf_stop_link(struct mt7996_dev *dev,
				    struct ieee80211_vif *vif,
				    u16 removed_links)
{
	unsigned long rem_links = removed_links;
	struct mld_reconf_stop_link *sl;
	struct mld_req_hdr hdr = {};
	unsigned int link_id;
	struct sk_buff *skb;
	struct tlv *tlv;

	skb = mt76_mcu_msg_alloc(&dev->mt76, NULL, sizeof(hdr) + sizeof(*sl));
	if (!skb)
		return -ENOMEM;

	memcpy(hdr.mld_addr, vif->addr, ETH_ALEN);
	skb_put_data(skb, &hdr, sizeof(hdr));

	tlv = mt7996_mcu_add_uni_tlv(skb, UNI_CMD_MLD_RECONF_STOP_LINK,
				     sizeof(*sl));
	sl = (struct mld_reconf_stop_link *)tlv;
	sl->link_bitmap = cpu_to_le16(removed_links);

	for_each_set_bit(link_id, &rem_links, IEEE80211_MLD_MAX_NUM_LINKS) {
		struct mt7996_vif_link *link;

		link = mt7996_vif_link(dev, vif, link_id);
		if (!link)
			continue;

		sl->bss_idx[link_id] = link->mt76.idx;
	}

	return mt76_mcu_skb_send_msg(&dev->mt76, skb, MCU_WM_UNI_CMD(MLD),
				     true);
}

static u64 mt7996_tsf_to_boottime(u64 tsf, u64 ref_tsf, u64 ref_boottime)
{
	return ref_boottime + (s64)(tsf - ref_tsf) * NSEC_PER_USEC;
}

static int
mt7996_mcu_mld_attlm_started(struct mt7996_dev *dev, struct ieee80211_vif *vif,
			     struct sk_buff *skb,
			     struct cfg80211_adv_ttlm_params *params,
			     struct mt7996_vif_link *ref,
			     struct ieee80211_bss_conf *ref_conf,
			     u16 stop_links, u16 disc_links)
{
	unsigned int hdr_len = sizeof(struct mt7996_mcu_mld_event) -
			       sizeof(struct mt7996_mcu_rxd);
	struct mt7996_vif *mvif = (struct mt7996_vif *)vif->drv_priv;
	u64 ref_bi = (u64)ref_conf->beacon_int * 1024, rem;
	struct mt7996_mcu_mld_attlm_resp_event *resp;
	unsigned long valid_links = vif->valid_links;
	u64 switch_tsf, end_tsf, ref_tsf, ref_boottime;
	struct ieee80211_hw *hw = mt76_hw(dev);
	unsigned int link_id;
	u64 switch_boottime, end_boottime, end_bound, span_us;
	bool generic;

	/* where the firmware has no buffer for the response, the generic
	 * result of the command alone answers the request
	 */
	generic = skb->len == sizeof(struct mt7996_mcu_uni_event);
	if (!generic) {
		if (skb->len < hdr_len + sizeof(*resp))
			return -EIO;

		resp = (struct mt7996_mcu_mld_attlm_resp_event *)(skb->data +
								  hdr_len);
		if (le16_to_cpu(resp->tag) != UNI_EVENT_MLD_ATTLM_RES_RSP ||
		    le16_to_cpu(resp->len) < sizeof(*resp))
			return -EIO;

		if (resp->status) {
			dev_warn(dev->mt76.dev,
				 "A-TTLM request failed: status %u\n",
				 resp->status);
			return -EIO;
		}

		switch_tsf = le32_to_cpu(resp->switch_time_tsf[0]) |
			     (u64)le32_to_cpu(resp->switch_time_tsf[1]) << 32;
		end_tsf = le32_to_cpu(resp->end_tsf[0]) |
			  (u64)le32_to_cpu(resp->end_tsf[1]) << 32;
	}

	/* the events of the firmware carry no time, so the TSF of the switch
	 * gives the CLOCK_BOOTTIME of its TBTT
	 */
	ref_tsf = __mt7996_get_tsf(hw, ref);
	ref_boottime = ktime_get_boottime_ns();

	/* the firmware switches at the next TBTT of the BSS of the request
	 * plus the switch time, rounded up to a beacon interval
	 */
	if (generic) {
		u64 switch_us = (u64)params->switch_time * 1024;

		div64_u64_rem(ref_tsf, ref_bi, &rem);
		switch_tsf = ref_tsf - rem + ref_bi;
		div64_u64_rem(switch_us, ref_bi, &rem);
		switch_tsf += switch_us + (rem ? ref_bi - rem : 0);
		end_tsf = switch_tsf + (u64)params->duration * 1024;
	}
	switch_boottime = mt7996_tsf_to_boottime(switch_tsf, ref_tsf,
						 ref_boottime);
	end_boottime = mt7996_tsf_to_boottime(end_tsf, ref_tsf, ref_boottime);

	/* the TSF and CLOCK_BOOTTIME drift apart from the TSF read above to
	 * the end; 0.1 % of that span and a beacon interval exceed the
	 * tolerance of both clocks
	 */
	span_us = end_tsf > ref_tsf ? end_tsf - ref_tsf : 0;
	end_bound = end_boottime + div_u64(span_us * NSEC_PER_USEC, 1000) +
		    ref_bi * NSEC_PER_USEC;

	spin_lock_bh(&dev->mt76.lock);
	mvif->attlm_cookie = params->cookie;
	mvif->attlm_cancelled = false;
	mvif->attlm_tbtt_boottime = switch_boottime;
	mvif->attlm_end_boottime = end_boottime;
	mvif->attlm_end_bound = end_bound;
	mvif->attlm_end_tsf = end_tsf;
	mvif->attlm_ref_link = ref_conf->link_id;
	mvif->attlm_ref_band = ref->mt76.band_idx;
	mvif->attlm_ref_omac = ref->mt76.omac_idx;
	mvif->attlm_links = stop_links;
	mvif->attlm_disc_links = disc_links;
	spin_unlock_bh(&dev->mt76.lock);

	/* the firmware reports the switch time in the TSF of one link, and
	 * each affiliated AP advertises it in its own TSF
	 */
	for_each_set_bit(link_id, &valid_links, IEEE80211_MLD_MAX_NUM_LINKS) {
		struct mt7996_vif_link *link = mt7996_vif_link(dev, vif, link_id);
		struct cfg80211_adv_ttlm_event event = {
			.cmd = NL80211_CMD_ADV_TTLM_STARTED,
			.cookie = params->cookie,
			.link_id = link_id,
		};
		struct ieee80211_bss_conf *link_conf;
		u64 tsf, beacon_int, rem;

		link_conf = link_conf_dereference_protected(vif, link_id);
		if (!link || !link_conf)
			continue;

		tsf = switch_tsf + __mt7996_get_tsf(hw, link) -
		      __mt7996_get_tsf(hw, ref);
		event.switch_tsf = (tsf >> 10) & 0xffff;

		/* a TBTT is where the TSF is a multiple of the beacon
		 * interval
		 */
		beacon_int = (u64)link_conf->beacon_int * 1024;
		if (beacon_int) {
			div64_u64_rem(tsf, beacon_int, &rem);
			event.tbtt_boottime = switch_boottime +
				(rem ? beacon_int - rem : 0) * NSEC_PER_USEC;
		}

		ieee80211_adv_ttlm_notify(vif, &event, GFP_KERNEL);
	}

	return 0;
}

static u64
mt7996_mcu_mld_attlm_requested(struct mt7996_dev *dev, struct ieee80211_vif *vif)
{
	struct mt7996_vif *mvif = (struct mt7996_vif *)vif->drv_priv;
	bool switched, ended;
	u64 end;

	spin_lock_bh(&dev->mt76.lock);
	switched = mvif->attlm_early_switch;
	ended = mvif->attlm_early_end;
	mvif->attlm_requesting = false;
	mvif->attlm_early_switch = false;
	mvif->attlm_early_end = false;
	spin_unlock_bh(&dev->mt76.lock);

	if (switched)
		mt7996_mcu_mld_attlm_step(dev, vif,
					  NL80211_CMD_ADV_TTLM_ESTABLISHED);
	if (ended)
		mt7996_mcu_mld_attlm_step(dev, vif, NL80211_CMD_ADV_TTLM_ENDED);

	spin_lock_bh(&dev->mt76.lock);
	end = mvif->attlm_end_boottime;
	spin_unlock_bh(&dev->mt76.lock);

	return end;
}

int mt7996_mcu_mld_adv_ttlm(struct mt7996_dev *dev, struct ieee80211_vif *vif,
			    struct cfg80211_adv_ttlm_params *params)
{
	unsigned long disabled = vif->valid_links & ~params->links;
	struct mt7996_vif *mvif = (struct mt7996_vif *)vif->drv_priv;
	u64 duration_us = (u64)params->duration * 1024;
	u64 switch_us = (u64)params->switch_time * 1024;
	struct mld_req_hdr hdr = { .mld_idx = 0xff };
	struct ieee80211_bss_conf *ref_conf = NULL;
	struct mt7996_vif_link *ref = NULL;
	struct mld_attlm_req *req;
	struct sk_buff *skb, *rskb;
	u16 stop_links = 0, disc_links = 0, stopped;
	unsigned int link_id;
	struct tlv *tlv;
	bool busy;
	u64 end;
	int ret;

	if (!disabled)
		return -EINVAL;

	if (switch_us > U32_MAX || duration_us > U32_MAX)
		return -ERANGE;

	/* the firmware has no command that stops a mapping, so a cancelled
	 * one runs until its end, which the end timer can still have to report
	 */
	spin_lock_bh(&dev->mt76.lock);
	busy = mvif->attlm_end_check || hrtimer_active(&mvif->attlm_end_timer);
	stopped = mvif->attlm_stop_links | mvif->attlm_hold_links |
		  mvif->reconf_done_links | mvif->reconf_stop_links;
	spin_unlock_bh(&dev->mt76.lock);
	if (busy || mt7996_mcu_mld_attlm_end_rest(dev, vif) > 0)
		return -EBUSY;

	skb = mt76_mcu_msg_alloc(&dev->mt76, NULL, sizeof(hdr) + sizeof(*req));
	if (!skb)
		return -ENOMEM;

	memcpy(hdr.mld_addr, vif->addr, ETH_ALEN);
	skb_put_data(skb, &hdr, sizeof(hdr));

	tlv = mt7996_mcu_add_uni_tlv(skb, UNI_CMD_MLD_ATTLM_RES_REQ,
				     sizeof(*req));
	req = (struct mld_attlm_req *)tlv;
	req->mst_timer = 1;
	req->e_timer = 1;
	req->mst_duration = cpu_to_le32(switch_us);
	req->e_duration = cpu_to_le32(duration_us);
	req->bss_idx = 0xff;

	for_each_set_bit(link_id, &disabled, IEEE80211_MLD_MAX_NUM_LINKS) {
		struct mt7996_vif_link *link = mt7996_vif_link(dev, vif, link_id);
		struct ieee80211_bss_conf *link_conf;

		if (!link)
			continue;

		link_conf = link_conf_dereference_protected(vif, link_id);
		if (!ref && link_conf && link_conf->enable_beacon &&
		    !(stopped & BIT(link_id))) {
			ref = link;
			ref_conf = link_conf;
			req->bss_idx = link->mt76.idx;
			disc_links = BIT(link_id);
		} else {
			stop_links |= BIT(link_id);
		}
		req->disabled_link_bitmap |= cpu_to_le16(BIT(link_id));
		req->disabled_bss_idx[link_id] = link->mt76.idx;
	}

	if (!ref) {
		dev_kfree_skb(skb);
		return -ENOLINK;
	}

	spin_lock_bh(&dev->mt76.lock);
	req->attlm_idx = ++mvif->attlm_idx;
	mvif->attlm_requesting = true;
	spin_unlock_bh(&dev->mt76.lock);

	ret = mt76_mcu_skb_send_and_get_msg(&dev->mt76, skb,
					    MCU_WM_UNI_CMD(MLD), true, &rskb);
	if (!ret) {
		ret = mt7996_mcu_mld_attlm_started(dev, vif, rskb, params, ref,
						   ref_conf, stop_links,
						   disc_links);
		dev_kfree_skb(rskb);
	}

	end = mt7996_mcu_mld_attlm_requested(dev, vif);
	if (!ret && end)
		hrtimer_start(&mvif->attlm_end_timer, ns_to_ktime(end),
			      HRTIMER_MODE_ABS_SOFT);

	return ret;
}

void mt7996_mcu_mld_adv_ttlm_cancel(struct mt7996_dev *dev,
				    struct ieee80211_vif *vif, u64 cookie)
{
	struct mt7996_vif *mvif = (struct mt7996_vif *)vif->drv_priv;

	spin_lock_bh(&dev->mt76.lock);
	if (mvif->attlm_cookie == cookie)
		mvif->attlm_cancelled = true;
	spin_unlock_bh(&dev->mt76.lock);
}

void mt7996_mcu_mld_adv_ttlm_end(struct mt7996_dev *dev,
				 struct ieee80211_vif *vif, gfp_t gfp)
{
	struct mt7996_vif *mvif = (struct mt7996_vif *)vif->drv_priv;
	struct cfg80211_adv_ttlm_event event = {
		.cmd = NL80211_CMD_ADV_TTLM_ENDED,
	};

	spin_lock_bh(&dev->mt76.lock);
	event.cookie = mt7996_mcu_mld_attlm_cookie_take(mvif);
	mvif->attlm_end_boottime = 0;
	mvif->attlm_end_check = false;
	spin_unlock_bh(&dev->mt76.lock);

	if (event.cookie && vif->type == NL80211_IFTYPE_AP)
		ieee80211_adv_ttlm_notify(vif, &event, gfp);
}

void mt7996_mcu_mld_adv_ttlm_reset(struct mt7996_dev *dev,
				   struct ieee80211_vif *vif)
{
	struct mt7996_vif *mvif = (struct mt7996_vif *)vif->drv_priv;

	hrtimer_cancel(&mvif->attlm_end_timer);
	mt7996_mcu_mld_adv_ttlm_end(dev, vif, GFP_KERNEL);

	spin_lock_bh(&dev->mt76.lock);
	mvif->attlm_links = 0;
	WRITE_ONCE(mvif->attlm_stop_links, 0);
	mvif->attlm_hold_links = 0;
	mvif->attlm_bcn_links = 0;
	mvif->attlm_disc_links = 0;
	mvif->attlm_disc_off = 0;
	mvif->attlm_disc_sync = 0;
	spin_unlock_bh(&dev->mt76.lock);
}

int mt7996_mcu_mld_link_oper(struct mt7996_dev *dev,
			     struct ieee80211_bss_conf *link_conf,
			     struct mt7996_vif_link *link, bool add)
{
	struct ieee80211_vif *vif = link_conf->vif;
	struct mt7996_vif *mvif = (struct mt7996_vif *)vif->drv_priv;
	struct bss_mld_link_op_tlv *mld_op;
	struct sk_buff *skb;
	struct tlv *tlv;

	skb = __mt7996_mcu_alloc_bss_req(&dev->mt76, &link->mt76,
					 MT7996_BSS_UPDATE_MAX_SIZE);
	if (IS_ERR(skb))
		return PTR_ERR(skb);

	tlv = mt7996_mcu_add_uni_tlv(skb, UNI_BSS_INFO_MLD_LINK_OP,
				     sizeof(*mld_op));
	mld_op = (struct bss_mld_link_op_tlv *)tlv;
	mld_op->link_operation = add;
	mld_op->own_mld_id = link->mld_idx;
	mld_op->link_id = link_conf->link_id;
	mld_op->group_mld_id = add ? mvif->mld_group_idx : 0xff;
	mld_op->remap_idx = add ? mvif->mld_remap_idx : 0xff;
	memcpy(mld_op->mac_addr, vif->addr, ETH_ALEN);

	return mt76_mcu_skb_send_msg(&dev->mt76, skb,
				     MCU_WMWA_UNI_CMD(BSS_INFO_UPDATE), true);
}

static unsigned long
mt7996_mcu_sta_mld_setup_links(struct mt7996_dev *dev,
			       struct ieee80211_vif *vif,
			       struct ieee80211_sta *sta)
{
	struct mt7996_sta *msta = (struct mt7996_sta *)sta->drv_priv;
	unsigned long links = mt7996_mac_sta_links(dev, vif, sta);
	unsigned int link_id;

	if (!msta->peer_mld_links)
		return links;

	for (link_id = 0; link_id < ARRAY_SIZE(msta->link); link_id++) {
		struct mt7996_sta_link *msta_link;

		msta_link = mt7996_sta_link_protected(dev, msta, link_id);
		if (!msta_link || msta_link->wcid.idx != msta->fw_mld_idx)
			continue;

		if (mt7996_vif_link(dev, vif, link_id))
			links |= BIT(link_id);
		break;
	}

	return links;
}

static void
mt7996_mcu_sta_mld_setup_tlv(struct mt7996_dev *dev, struct sk_buff *skb,
			     struct ieee80211_vif *vif,
			     struct ieee80211_sta *sta)
{
	unsigned long links = mt7996_mcu_sta_mld_setup_links(dev, vif, sta);
	struct mt7996_sta *msta = (struct mt7996_sta *)sta->drv_priv;
	u8 seclink_id = mt7996_sta_seclink_get(dev, msta);
	unsigned int nlinks = hweight16(links);
	struct mld_setup_link *mld_setup_link;
	struct sta_rec_mld_setup *mld_setup;
	struct mt7996_sta_link *msta_link;
	unsigned int link_id;
	struct tlv *tlv;

	if (!(links & BIT(msta->deflink_id)))
		return;

	msta_link = mt7996_sta_link_protected(dev, msta, msta->deflink_id);

	tlv = mt76_connac_mcu_add_tlv(skb, STA_REC_MLD,
				      sizeof(struct sta_rec_mld_setup) +
				      sizeof(struct mld_setup_link) * nlinks);

	mld_setup = (struct sta_rec_mld_setup *)tlv;
	memcpy(mld_setup->mld_addr, sta->addr, ETH_ALEN);
	mld_setup->setup_wcid = cpu_to_le16(msta_link->wcid.idx);
	/* the firmware keeps the MLD record of PEER_MLD under fw_mld_idx */
	if (msta->peer_mld_links)
		mld_setup->setup_wcid = cpu_to_le16(msta->fw_mld_idx);
	mld_setup->primary_id = mld_setup->setup_wcid;

	if (!(links & BIT(seclink_id)))
		seclink_id = msta->deflink_id;
	msta_link = mt7996_sta_link_protected(dev, msta, seclink_id);
	mld_setup->seconed_id = cpu_to_le16(msta_link->wcid.idx);
	mld_setup->link_num = nlinks;
	/* PEER_MLD_INFO holds only the links of the association */
	if (links == msta->peer_mld_links)
		mld_setup->extra_info = EXTRA_INFO_MLD_SKIP;

	mld_setup_link = (struct mld_setup_link *)mld_setup->link_info;
	for_each_set_bit(link_id, &links, IEEE80211_MLD_MAX_NUM_LINKS) {
		struct mt7996_vif_link *link;

		msta_link = mt7996_sta_link_protected(dev, msta, link_id);
		link = mt7996_vif_link(dev, vif, link_id);

		mld_setup_link->wcid = cpu_to_le16(msta_link->wcid.idx);
		mld_setup_link->bss_idx = link->mt76.idx;
		mld_setup_link++;
	}
}

static u32 mt7996_mcu_peer_mld_amsdu_len(u16 max_amsdu_len)
{
	switch (max_amsdu_len) {
	case IEEE80211_MAX_MPDU_LEN_VHT_11454:
		return IEEE80211_VHT_CAP_MAX_MPDU_LENGTH_11454;
	case IEEE80211_MAX_MPDU_LEN_HT_7935:
	case IEEE80211_MAX_MPDU_LEN_VHT_7991:
		return IEEE80211_VHT_CAP_MAX_MPDU_LENGTH_7991;
	default:
		return IEEE80211_VHT_CAP_MAX_MPDU_LENGTH_3895;
	}
}

int mt7996_mcu_peer_mld_ttlm_req(struct mt7996_dev *dev,
				 struct ieee80211_vif *vif,
				 struct ieee80211_sta *sta,
				 const struct ieee80211_neg_ttlm *ttlm)
{
	struct mt7996_sta *msta = (struct mt7996_sta *)sta->drv_priv;
	unsigned long links = mt7996_mac_sta_links(dev, vif, sta);
	struct peer_mld_req_hdr hdr = { .fw_mld_idx = cpu_to_le16(0xff) };
	struct mt7996_sta_link *msta_link;
	struct peer_mld_ttlm_req *req;
	unsigned int link_id, tid;
	struct sk_buff *skb;
	struct tlv *tlv;

	memcpy(hdr.peer_mld_addr, sta->addr, ETH_ALEN);

	skb = mt76_mcu_msg_alloc(&dev->mt76, NULL, sizeof(hdr) + sizeof(*req));
	if (!skb)
		return -ENOMEM;

	skb_put_data(skb, &hdr, sizeof(hdr));
	tlv = mt7996_mcu_add_uni_tlv(skb, UNI_CMD_PEER_MLD_TTLM_REQ,
				     sizeof(*req));
	req = (struct peer_mld_ttlm_req *)tlv;
	memcpy(req->mld_addr, vif->addr, ETH_ALEN);

	for_each_set_bit(link_id, &links, IEEE80211_MLD_MAX_NUM_LINKS) {
		msta_link = mt7996_sta_link_protected(dev, msta, link_id);
		if (!msta_link)
			continue;

		req->enabled_link_bitmap |= cpu_to_le16(BIT(link_id));
		req->link_to_wcid[link_id] = cpu_to_le16(msta_link->wcid.idx);
		for (tid = 0; tid < IEEE80211_TTLM_NUM_TIDS; tid++) {
			if (ttlm->downlink[tid] & BIT(link_id))
				req->dl_tid_map[link_id] |= BIT(tid);
			if (ttlm->uplink[tid] & BIT(link_id))
				req->ul_tid_map[link_id] |= BIT(tid);
		}
	}

	return mt76_mcu_skb_send_msg(&dev->mt76, skb,
				     MCU_WM_UNI_CMD(PEER_MLD), true);
}

int mt7996_mcu_set_peer_mld(struct mt7996_dev *dev, struct ieee80211_vif *vif,
			    struct ieee80211_sta *sta)
{
	struct mt7996_sta *msta = (struct mt7996_sta *)sta->drv_priv;
	unsigned long links = mt7996_mac_sta_links(dev, vif, sta);
	unsigned int nlinks = hweight16(links);
	struct peer_mld_cap_entry *cap_entry;
	struct peer_mld_link_info *link_info;
	struct mt7996_sta_link *msta_link;
	u8 seclink_id = msta->deflink_id;
	struct peer_mld_req_hdr *hdr;
	u16 max_amsdu_len = U16_MAX;
	struct peer_mld_info *info;
	struct peer_mld_cap *cap;
	unsigned int link_id;
	struct sk_buff *skb;
	struct tlv *tlv;
	u16 fw_mld_idx;
	int len, ret;

	if (!(links & BIT(msta->deflink_id)))
		return 0;

	len = sizeof(*hdr) + sizeof(*info) + nlinks * sizeof(*link_info) +
	      sizeof(*cap) + 2 * sizeof(*cap_entry);
	skb = mt76_mcu_msg_alloc(&dev->mt76, NULL, len);
	if (!skb)
		return -ENOMEM;

	msta_link = mt7996_sta_link_protected(dev, msta, msta->deflink_id);

	fw_mld_idx = msta_link->wcid.idx;
	hdr = skb_put_zero(skb, sizeof(*hdr));
	hdr->fw_mld_idx = cpu_to_le16(fw_mld_idx);
	memcpy(hdr->peer_mld_addr, sta->addr, ETH_ALEN);

	tlv = mt7996_mcu_add_uni_tlv(skb, UNI_CMD_PEER_MLD_INFO,
				     sizeof(*info) +
				     nlinks * sizeof(*link_info));
	info = (struct peer_mld_info *)tlv;
	info->link_num = nlinks;
	info->primary_id = cpu_to_le16(msta_link->wcid.idx);
	info->setup_wcid = cpu_to_le16(msta_link->wcid.idx);

	if (links & BIT(msta->seclink_id))
		seclink_id = msta->seclink_id;
	msta_link = mt7996_sta_link_protected(dev, msta, seclink_id);
	info->second_id = cpu_to_le16(msta_link->wcid.idx);

	link_info = info->link_info;
	for_each_set_bit(link_id, &links, IEEE80211_MLD_MAX_NUM_LINKS) {
		struct ieee80211_link_sta *link_sta;
		struct mt7996_vif_link *link;

		msta_link = mt7996_sta_link_protected(dev, msta, link_id);
		link = mt7996_vif_link(dev, vif, link_id);
		link_sta = link_sta_dereference_protected(sta, link_id);

		max_amsdu_len = min(max_amsdu_len, link_sta->agg.max_amsdu_len);

		link_info->fw_link_id = cpu_to_le16(msta_link->wcid.idx);
		link_info->link_id = link_id;
		link_info->bss_idx = link->mt76.idx;
		link_info->wlan_idx = cpu_to_le16(msta_link->wcid.idx);
		link_info->band_idx = link->mt76.band_idx;
		link_info++;
	}

	tlv = mt7996_mcu_add_uni_tlv(skb, UNI_CMD_PEER_MLD_CAP,
				     sizeof(*cap) + 2 * sizeof(*cap_entry));
	cap = (struct peer_mld_cap *)tlv;

	cap_entry = cap->entry;
	cap_entry->type = cpu_to_le32(UNI_CMD_PEER_MLD_CAP_TYPE_AMSDU_MAX_LEN);
	cap_entry->val = cpu_to_le32(mt7996_mcu_peer_mld_amsdu_len(max_amsdu_len));
	cap_entry++;
	cap_entry->type = cpu_to_le32(UNI_CMD_PEER_MLD_CAP_TYPE_AMSDU_MAX_CNT);
	cap_entry->val = cpu_to_le32(31);

	ret = mt76_mcu_skb_send_msg(&dev->mt76, skb,
				    MCU_WMWA_UNI_CMD(PEER_MLD), true);
	if (!ret) {
		msta->peer_mld_links = links;
		msta->fw_mld_idx = fw_mld_idx;
		msta->fw_mld_link_id = msta->deflink_id;
	}

	return ret;
}

static void
mt7996_mcu_sta_eht_mld_tlv(struct mt7996_dev *dev, struct sk_buff *skb,
			   struct ieee80211_sta *sta)
{
	struct sta_rec_eht_mld *eht_mld;
	struct tlv *tlv;
	int i;

	tlv = mt76_connac_mcu_add_tlv(skb, STA_REC_EHT_MLD, sizeof(*eht_mld));
	eht_mld = (struct sta_rec_eht_mld *)tlv;

	for (i = 0; i < ARRAY_SIZE(eht_mld->str_cap); i++)
		eht_mld->str_cap[i] = 0x7;
}

int mt7996_mcu_add_sta(struct mt7996_dev *dev,
		       struct ieee80211_bss_conf *link_conf,
		       struct ieee80211_link_sta *link_sta,
		       struct mt7996_vif_link *link,
		       struct mt7996_sta_link *msta_link,
		       int conn_state, bool newly)
{
	struct mt76_wcid *wcid = msta_link ? &msta_link->wcid : link->mt76.wcid;
	struct ieee80211_sta *sta = link_sta ? link_sta->sta : NULL;
	u16 weight = IEEE80211_DEFAULT_AIRTIME_WEIGHT;
	struct sk_buff *skb;
	int ret;

	if (sta)
		weight = mt7996_sta_airtime_weight((struct mt7996_sta *)
						   sta->drv_priv);

	skb = __mt76_connac_mcu_alloc_sta_req(&dev->mt76, &link->mt76, wcid,
					      MT7996_STA_UPDATE_MAX_SIZE);
	if (IS_ERR(skb))
		return PTR_ERR(skb);

	/* starec basic */
	mt76_connac_mcu_sta_basic_tlv(&dev->mt76, skb, link_conf, link_sta,
				      conn_state, newly);

	if (conn_state == CONN_STATE_DISCONNECT)
		goto out;

	/* starec hdr trans */
	mt7996_mcu_sta_hdr_trans_tlv(dev, skb, link_conf->vif, wcid);
	/* starec tx proc */
	mt7996_mcu_sta_tx_proc_tlv(skb);

	/* tag order is in accordance with firmware dependency. */
	if (link_sta) {
		/* starec hdrt mode */
		mt7996_mcu_sta_hdrt_tlv(dev, skb, 1);
		if (conn_state == CONN_STATE_CONNECT) {
			/* starec bfer */
			if (mt7996_mcu_sta_bfer_tlv(dev, skb, link_conf,
						    link_sta, link))
				msta_link->bf_active = true;
			/* starec bfee */
			mt7996_mcu_sta_bfee_tlv(dev, skb, link_conf, link_sta,
						link);
		}
		/* starec ht */
		mt7996_mcu_sta_ht_tlv(skb, link_sta);
		/* starec vht */
		mt7996_mcu_sta_vht_tlv(skb, link_sta);
		/* starec uapsd */
		mt76_connac_mcu_sta_uapsd(skb, link_conf->vif, sta);
		/* starec amsdu */
		mt7996_mcu_sta_amsdu_tlv(dev, skb, link_conf->vif, link_sta,
					 msta_link);
		/* starec he */
		mt7996_mcu_sta_he_tlv(skb, link_sta, link);
		/* starec he 6g*/
		mt7996_mcu_sta_he_6g_tlv(skb, link_sta);
		/* starec eht */
		mt7996_mcu_sta_eht_tlv(skb, link_sta);
		/* starec muru */
		mt7996_mcu_sta_muru_tlv(dev, skb, link_conf, link_sta);

		if (sta->mlo) {
			mt7996_mcu_sta_mld_setup_tlv(dev, skb, link_conf->vif,
						     sta);
			mt7996_mcu_sta_eht_mld_tlv(dev, skb, sta);
		}
	}

out:
	ret = mt76_mcu_skb_send_msg(&dev->mt76, skb,
				    MCU_WMWA_UNI_CMD(STA_REC_UPDATE), true);
	if (ret || conn_state == CONN_STATE_DISCONNECT)
		return ret;

	mt7996_mcu_sta_init_vow(dev, &link->mt76, wcid, weight);

	return 0;
}

int mt7996_mcu_teardown_mld_sta(struct mt7996_dev *dev,
				struct mt7996_vif_link *link,
				struct mt7996_sta_link *msta_link)
{
	struct sk_buff *skb;

	skb = __mt76_connac_mcu_alloc_sta_req(&dev->mt76, &link->mt76,
					      &msta_link->wcid,
					      MT7996_STA_UPDATE_MAX_SIZE);
	if (IS_ERR(skb))
		return PTR_ERR(skb);

	mt76_connac_mcu_add_tlv(skb, STA_REC_MLD_OFF, sizeof(struct tlv));

	return mt76_mcu_skb_send_msg(&dev->mt76, skb,
				     MCU_WMWA_UNI_CMD(STA_REC_UPDATE), true);
}

int mt7996_mcu_sta_rec_check(struct mt7996_dev *dev,
			     struct mt7996_vif_link *link,
			     struct mt7996_sta_link *msta_link)
{
	struct mt76_wcid *wcid = msta_link ? &msta_link->wcid : link->mt76.wcid;
	struct sk_buff *skb;
	int ret;

	skb = __mt76_connac_mcu_alloc_sta_req(&dev->mt76, &link->mt76, wcid,
					      MT7996_STA_UPDATE_MAX_SIZE);
	if (IS_ERR(skb))
		return PTR_ERR(skb);

	/* the firmware refuses HDRT with 0x10003 only for a record not in use,
	 * and on a record in use HDRT writes the given mode, so the mode is
	 * the one the CONNECT of the record sets: 1 for a peer, none (0) for
	 * the BMC record
	 */
	mt7996_mcu_sta_hdrt_tlv(dev, skb, !!msta_link);

	ret = mt76_mcu_skb_send_msg(&dev->mt76, skb,
				    MCU_WMWA_UNI_CMD(STA_REC_UPDATE) |
				    MT7996_MCU_CMD_STA_REC_EEXIST, true);

	return ret == -EEXIST ? -ENOENT : ret;
}

int mt7996_mcu_sta_bf_release(struct mt7996_dev *dev,
			      struct mt7996_vif_link *link,
			      struct mt7996_sta_link *msta_link)
{
	struct sta_rec_bf *bf;
	struct sk_buff *skb;
	struct tlv *tlv;
	int ret;

	if (!msta_link->bf_active)
		return 0;

	/* a release after the record is freed can hit the profile that
	 * another station took, so a lost release is not sent again
	 */
	msta_link->bf_active = false;

	skb = __mt76_connac_mcu_alloc_sta_req(&dev->mt76, &link->mt76,
					      &msta_link->wcid,
					      MT7996_STA_UPDATE_MAX_SIZE);
	if (IS_ERR(skb)) {
		ret = PTR_ERR(skb);
		goto out;
	}

	tlv = mt76_connac_mcu_add_tlv(skb, STA_REC_BF, sizeof(*bf));
	bf = (struct sta_rec_bf *)tlv;
	bf->pfmu = cpu_to_le16(MT7996_BF_PFMU_NONE);
	bf->bf_cap = MT7996_BF_CAP_RELEASE;

	ret = mt76_mcu_skb_send_msg(&dev->mt76, skb,
				    MCU_WMWA_UNI_CMD(STA_REC_UPDATE), true);
out:
	if (ret)
		dev_warn(dev->mt76.dev, "wcid %d: BF release not sent: %d\n",
			 msta_link->wcid.idx, ret);

	return ret;
}

int mt7996_mcu_update_mld_sta(struct mt7996_dev *dev,
			      struct ieee80211_vif *vif,
			      struct ieee80211_sta *sta,
			      struct mt7996_vif_link *link,
			      struct mt7996_sta_link *msta_link)
{
	struct sk_buff *skb;

	skb = __mt76_connac_mcu_alloc_sta_req(&dev->mt76, &link->mt76,
					      &msta_link->wcid,
					      MT7996_STA_UPDATE_MAX_SIZE);
	if (IS_ERR(skb))
		return PTR_ERR(skb);

	mt7996_mcu_sta_mld_setup_tlv(dev, skb, vif, sta);
	mt7996_mcu_sta_eht_mld_tlv(dev, skb, sta);

	return mt76_mcu_skb_send_msg(&dev->mt76, skb,
				     MCU_WMWA_UNI_CMD(STA_REC_UPDATE), true);
}

void mt7996_mcu_update_sta_rec_bw(void *data, struct ieee80211_sta *sta)
{
	struct mt7996_vif_link *link = (struct mt7996_vif_link *)data;
	struct mt7996_sta *msta = (struct mt7996_sta *)sta->drv_priv;
	struct mt7996_phy *phy = mt7996_vif_link_phy(link);
	struct mt7996_sta_link *msta_link;
	struct mt7996_dev *dev;
	struct ieee80211_bss_conf *link_conf;
	struct ieee80211_link_sta *link_sta;
	struct ieee80211_vif *vif;
	struct sk_buff *skb;
	int link_id;

	if (!phy)
		return;

	if (link->mt76.mvif != &msta->vif->mt76)
		return;

	dev = phy->dev;
	link_id = link->msta_link.wcid.link_id;
	link_sta = link_sta_dereference_protected(sta, link_id);
	if (!link_sta)
		return;

	msta_link = mt7996_sta_link_protected(dev, msta, link_id);
	if (!msta_link)
		return;

	vif = container_of((void *)msta->vif, struct ieee80211_vif, drv_priv);
	link_conf = link_conf_dereference_protected(vif, link_id);
	if (!link_conf)
		return;

	skb = __mt76_connac_mcu_alloc_sta_req(&dev->mt76, &link->mt76,
					      &msta_link->wcid,
					      MT7996_STA_UPDATE_MAX_SIZE);
	if (IS_ERR(skb))
		return;

	/* each explicit STA_REC_BF takes one more sounding slot of the wcid,
	 * and the release frees those of the active list
	 */
	if (mt7996_mcu_sta_bfer_tlv(dev, skb, link_conf, link_sta, link)) {
		mt7996_mcu_sta_bf_release(dev, link, msta_link);
		msta_link->bf_active = true;
	}
	mt7996_mcu_sta_rate_ctrl_tlv(skb, dev, vif, link_conf, link_sta, link);

	mt76_mcu_skb_send_msg(&dev->mt76, skb,
			      MCU_WMWA_UNI_CMD(STA_REC_UPDATE), true);
}

static int
mt7996_mcu_sta_key_tlv(struct mt76_dev *dev, struct mt76_wcid *wcid,
		       struct sk_buff *skb,
		       struct ieee80211_key_conf *key,
		       enum set_key_cmd cmd)
{
	struct sta_rec_sec_uni *sec;
	struct sec_key_uni *sec_key;
	struct tlv *tlv;
	u8 cipher;

	tlv = mt76_connac_mcu_add_tlv(skb, STA_REC_KEY_V2, sizeof(*sec));
	sec = (struct sta_rec_sec_uni *)tlv;
	/* due to connac3 FW design, we only do remove key for BIGTK; even for
	 * removal, the field should be filled with SET_KEY
	 */
	sec->add = SET_KEY;
	sec->n_cipher = 1;
	sec_key = &sec->key[0];
	sec_key->wlan_idx = cpu_to_le16(wcid->idx);
	sec_key->key_id = key->keyidx;

	if (cmd != SET_KEY)
		return 0;

	cipher = mt76_connac_mcu_get_cipher(key->cipher);
	if (cipher == MCU_CIPHER_NONE)
		return -EOPNOTSUPP;

	sec_key->mgmt_prot = 0;
	sec_key->cipher_id = cipher;
	sec_key->cipher_len = sizeof(*sec_key);
	sec_key->key_len = key->keylen;
	sec_key->need_resp = 0;
	memcpy(sec_key->key, key->key, key->keylen);

	if (cipher == MCU_CIPHER_TKIP) {
		/* Rx/Tx MIC keys are swapped */
		memcpy(sec_key->key + 16, key->key + 24, 8);
		memcpy(sec_key->key + 24, key->key + 16, 8);
		return 0;
	}

	if (sec_key->key_id != 6 && sec_key->key_id != 7)
		return 0;

	switch (key->cipher) {
	case WLAN_CIPHER_SUITE_AES_CMAC:
		sec_key->cipher_id = MCU_CIPHER_BCN_PROT_CMAC_128;
		break;
	case WLAN_CIPHER_SUITE_BIP_GMAC_128:
		sec_key->cipher_id = MCU_CIPHER_BCN_PROT_GMAC_128;
		break;
	case WLAN_CIPHER_SUITE_BIP_GMAC_256:
		sec_key->cipher_id = MCU_CIPHER_BCN_PROT_GMAC_256;
		break;
	case WLAN_CIPHER_SUITE_BIP_CMAC_256:
		if (!is_mt7990(dev))
			return -EOPNOTSUPP;
		sec_key->cipher_id = MCU_CIPHER_BCN_PROT_CMAC_256;
		break;
	default:
		return -EOPNOTSUPP;
	}

	sec_key->bcn_mode = is_mt7990(dev) ? BP_HW_MODE : BP_SW_MODE;

	return 0;
}

int mt7996_mcu_add_key(struct mt76_dev *dev, struct mt7996_vif_link *link,
		       struct ieee80211_key_conf *key, int mcu_cmd,
		       struct mt76_wcid *wcid, enum set_key_cmd cmd)
{
	struct sk_buff *skb;
	int ret;

	skb = __mt76_connac_mcu_alloc_sta_req(dev, (struct mt76_vif_link *)link,
					      wcid, MT7996_STA_UPDATE_MAX_SIZE);
	if (IS_ERR(skb))
		return PTR_ERR(skb);

	ret = mt7996_mcu_sta_key_tlv(dev, wcid, skb, key, cmd);
	if (ret) {
		dev_kfree_skb(skb);
		return ret;
	}

	return mt76_mcu_skb_send_msg(dev, skb, mcu_cmd, true);
}

int mt7996_mcu_add_dev_info(struct mt7996_phy *phy, struct ieee80211_vif *vif,
			    struct ieee80211_bss_conf *link_conf,
			    struct mt76_vif_link *mlink, bool enable)
{
	struct mt7996_dev *dev = phy->dev;
	struct {
		struct req_hdr {
			u8 omac_idx;
			u8 band_idx;
			u8 __rsv[2];
		} __packed hdr;
		struct req_tlv {
			__le16 tag;
			__le16 len;
			u8 active;
			u8 __rsv;
			u8 omac_addr[ETH_ALEN];
		} __packed tlv;
	} data = {
		.hdr = {
			.omac_idx = mlink->omac_idx,
			.band_idx = mlink->band_idx,
		},
		.tlv = {
			.tag = cpu_to_le16(DEV_INFO_ACTIVE),
			.len = cpu_to_le16(sizeof(struct req_tlv)),
			.active = enable,
		},
	};

	if (mlink->omac_idx >= REPEATER_BSSID_START)
		return mt7996_mcu_muar_config(dev, mlink, link_conf->addr, false, enable);

	memcpy(data.tlv.omac_addr, link_conf->addr, ETH_ALEN);
	return mt76_mcu_send_msg(&dev->mt76, MCU_WMWA_UNI_CMD(DEV_INFO_UPDATE),
				 &data, sizeof(data), true);
}

static void
mt7996_mcu_beacon_cntdwn(struct sk_buff *rskb, struct sk_buff *skb,
			 struct ieee80211_mutable_offsets *offs,
			 bool csa, int cnt)
{
	struct bss_bcn_cntdwn_tlv *info;
	struct tlv *tlv;
	u16 tag;

	if (!offs->cntdwn_counter_offs[0] || cnt < 0)
		return;

	tag = csa ? UNI_BSS_INFO_BCN_CSA : UNI_BSS_INFO_BCN_BCC;

	tlv = mt7996_mcu_add_uni_tlv(rskb, tag, sizeof(*info));

	info = (struct bss_bcn_cntdwn_tlv *)tlv;
	info->cnt = cnt;

	/* a count of 0 leaves a countdown that the firmware still counts
	 * running; the abort ends it in every state with no event
	 */
	if (!cnt) {
		if (csa)
			info->csa.abort = true;
		else
			info->cca.abort = true;
	}

	/* abort the CCA countdown when starting CSA countdown */
	if (csa) {
		struct bss_bcn_cntdwn_tlv *cca_info;

		tlv = mt7996_mcu_add_uni_tlv(rskb, UNI_BSS_INFO_BCN_BCC,
					     sizeof(*cca_info));
		cca_info = (struct bss_bcn_cntdwn_tlv *)tlv;
		cca_info->cca.abort = true;
	}
}

static struct mt76_cntdwn *
mt7996_mcu_cntdwn_get(struct ieee80211_vif *vif,
		      struct ieee80211_bss_conf *link_conf)
{
	struct mt7996_vif *mvif = (struct mt7996_vif *)vif->drv_priv;

	return &mvif->link_info[link_conf->link_id].cntdwn;
}

static void
mt7996_mcu_beacon_cntdwn_end(struct ieee80211_vif *vif,
			     struct ieee80211_bss_conf *link_conf)
{
	if (!mt76_cntdwn_finish_take(mt7996_mcu_cntdwn_get(vif, link_conf)))
		return;

	if (link_conf->csa_active)
		ieee80211_csa_finish(vif, link_conf->link_id);
	else
		ieee80211_color_change_finish(vif, link_conf->link_id);
}

/* Returns false where the upload waits until no pass is near. The count of
 * a running countdown goes into mac80211 before it builds the template. A
 * CSA pass writes the stored count into the template before it counts down,
 * so the template of a CSA holds one more than the count left; a BCC pass
 * counts down first.
 */
static bool
mt7996_mcu_beacon_cntdwn_prepare(struct mt7996_dev *dev,
				 struct ieee80211_vif *vif,
				 struct ieee80211_bss_conf *link_conf,
				 struct mt7996_vif_link *link, bool enabled,
				 u64 now)
{
	struct mt76_cntdwn *cd = mt7996_mcu_cntdwn_get(vif, link_conf);
	u8 type = mt76_cntdwn_type(link_conf);
	unsigned int passes;
	u64 wait;

	if (!type) {
		mt76_cntdwn_clear(cd);
		return true;
	}

	if (!enabled) {
		cd->wait = false;
		return true;
	}

	if (mt76_cntdwn_begins(cd, type)) {
		mt76_cntdwn_clear(cd);
		return true;
	}

	if (test_bit(MT76_CNTDWN_PHASE_LOST, &cd->state)) {
		u64 tsf = __mt7996_get_tsf(mt76_hw(dev), link);

		now = ktime_get_boottime_ns();
		mt76_cntdwn_rephase(cd, MT7996_PRE_TBTT_US, tsf, now);
	}

	wait = mt76_cntdwn_wait(cd, now);
	if (wait) {
		mt76_cntdwn_defer(&dev->cntdwn_timer, cd, now + wait);
		return false;
	}

	passes = mt76_cntdwn_passes(cd, now);
	if (type == MT76_CNTDWN_CSA && passes)
		passes--;
	mt76_cntdwn_counter_sync(vif, link_conf->link_id, cd, passes);

	return true;
}

/* Returns false where a countdown that this template starts waits until no
 * pass is near; @cnt is the count of the countdown sub-tag, -1 for none.
 */
static bool
mt7996_mcu_beacon_cntdwn_count(struct mt7996_dev *dev,
			       struct ieee80211_vif *vif,
			       struct ieee80211_bss_conf *link_conf,
			       struct mt7996_vif_link *link, struct sk_buff *skb,
			       struct ieee80211_mutable_offsets *offs, u64 now,
			       int *cnt)
{
	struct mt76_cntdwn *cd = mt7996_mcu_cntdwn_get(vif, link_conf);
	u8 type = mt76_cntdwn_type(link_conf);

	*cnt = -1;
	if (!offs->cntdwn_counter_offs[0] || !type)
		return true;

	if (type != cd->type) {
		u64 tsf = __mt7996_get_tsf(mt76_hw(dev), link), wait;

		now = ktime_get_boottime_ns();
		mt76_cntdwn_start(cd, type, skb->data[offs->cntdwn_counter_offs[0]],
				  link_conf->beacon_int * 1024,
				  MT7996_PRE_TBTT_US, tsf, now);
		wait = mt76_cntdwn_wait(cd, now);
		if (wait) {
			mt76_cntdwn_clear(cd);
			mt76_cntdwn_defer(&dev->cntdwn_timer, cd, now + wait);
			return false;
		}
	}

	/* The firmware sends no end event for a countdown that it holds no
	 * longer, so the driver ends one whose last pass has gone.
	 */
	*cnt = max(mt76_cntdwn_left(cd, now), 0);
	if (!*cnt)
		mt7996_mcu_beacon_cntdwn_end(vif, link_conf);

	return true;
}

void mt7996_mcu_beacon_cntdwn_work(struct work_struct *work)
{
	struct mt7996_dev *dev = container_of(work, struct mt7996_dev,
					      cntdwn_work);
	struct ieee80211_hw *hw = mt76_hw(dev);
	struct ieee80211_vif *vif;

	wiphy_lock(hw->wiphy);
	mutex_lock(&dev->mt76.mutex);
	for_each_active_interface(vif, hw) {
		struct mt7996_vif *mvif = (struct mt7996_vif *)vif->drv_priv;
		unsigned long valid_links = vif->valid_links ?: BIT(0);
		unsigned int link_id;

		for_each_set_bit(link_id, &valid_links,
				 IEEE80211_MLD_MAX_NUM_LINKS) {
			struct mt76_cntdwn *cd = &mvif->link_info[link_id].cntdwn;
			struct ieee80211_bss_conf *link_conf;

			if (!cd->wait)
				continue;

			cd->wait = false;
			link_conf = link_conf_dereference_protected(vif,
								    link_id);
			if (link_conf)
				mt7996_mcu_add_beacon(hw, vif, link_conf,
						      link_conf->enable_beacon);
		}
	}
	mutex_unlock(&dev->mt76.mutex);
	wiphy_unlock(hw->wiphy);
}

enum hrtimer_restart mt7996_mcu_beacon_cntdwn_timer(struct hrtimer *timer)
{
	struct mt7996_dev *dev = container_of(timer, struct mt7996_dev,
					      cntdwn_timer);

	queue_work(system_highpri_wq, &dev->cntdwn_work);

	return HRTIMER_NORESTART;
}

static void
mt7996_mcu_beacon_mbss(struct sk_buff *rskb, struct sk_buff *skb,
		       struct bss_bcn_content_tlv *bcn,
		       struct ieee80211_mutable_offsets *offs)
{
	struct bss_bcn_mbss_tlv *mbss;
	const struct element *elem;
	struct tlv *tlv;

	tlv = mt7996_mcu_add_uni_tlv(rskb, UNI_BSS_INFO_BCN_MBSSID, sizeof(*mbss));

	mbss = (struct bss_bcn_mbss_tlv *)tlv;
	mbss->offset[0] = cpu_to_le16(offs->tim_offset);
	mbss->bitmap = cpu_to_le32(1);

	for_each_element_id(elem, WLAN_EID_MULTIPLE_BSSID,
			    &skb->data[offs->mbssid_off],
			    skb->len - offs->mbssid_off) {
		const struct element *sub_elem;

		if (elem->datalen < 2)
			continue;

		for_each_element(sub_elem, elem->data + 1, elem->datalen - 1) {
			const struct ieee80211_bssid_index *idx;
			const u8 *idx_ie;

			/* not a valid BSS profile */
			if (sub_elem->id || sub_elem->datalen < 4)
				continue;

			/* Find WLAN_EID_MULTI_BSSID_IDX
			 * in the merged nontransmitted profile
			 */
			idx_ie = cfg80211_find_ie(WLAN_EID_MULTI_BSSID_IDX,
						  sub_elem->data, sub_elem->datalen);
			if (!idx_ie || idx_ie[1] < sizeof(*idx))
				continue;

			idx = (void *)(idx_ie + 2);
			if (!idx->bssid_index || idx->bssid_index > 31)
				continue;

			mbss->offset[idx->bssid_index] = cpu_to_le16(idx_ie -
								     skb->data);
			mbss->bitmap |= cpu_to_le32(BIT(idx->bssid_index));
		}
	}
}

static bool
mt7996_mcu_beacon_crit_update(struct sk_buff *rskb, struct sk_buff *skb,
			      struct ieee80211_vif *vif,
			      struct ieee80211_mutable_offsets *offs)
{
	struct ieee80211_mgmt *mgmt = (struct ieee80211_mgmt *)skb->data;
	struct bss_bcn_crit_update_tlv *crit;
	struct tlv *tlv;

	if (!ieee80211_vif_is_mld(vif))
		return false;

	if (!(mgmt->u.beacon.capab_info &
	      cpu_to_le16(WLAN_CAPABILITY_CRITICAL_UPDATE)))
		return false;

	tlv = mt7996_mcu_add_uni_tlv(rskb, UNI_BSS_INFO_BCN_CRIT_UPDATE,
				     sizeof(*crit));
	crit = (struct bss_bcn_crit_update_tlv *)tlv;
	crit->bss_bitmap = cpu_to_le32(BIT(0));
	/* no link of the MLD waits for the Beacon frame of another band */
	crit->bypass_seq_bitmap = cpu_to_le32(BIT(0));
	crit->tim_ie_pos[0] = cpu_to_le16(offs->tim_offset);
	crit->cap_info_ie_pos[0] =
		cpu_to_le16(offsetof(struct ieee80211_mgmt,
				     u.beacon.capab_info));
	crit->require_event = true;

	return true;
}

static int
mt7996_mcu_beacon_send(struct mt7996_dev *dev, struct mt7996_vif_link *link,
		       struct sk_buff *rskb, bool crit_update,
		       u32 crit_update_id, bool wait, bool *ambiguous)
{
	bool pending;
	int ret;

	spin_lock_bh(&dev->mt76.lock);
	pending = crit_update && crit_update_id != link->crit_update_id;
	link->crit_update_pending = pending;
	spin_unlock_bh(&dev->mt76.lock);

	ret = mt76_mcu_skb_send_msg(&dev->mt76, rskb,
				    MCU_WMWA_UNI_CMD(BSS_INFO_UPDATE), wait);
	if (ret)
		dev_warn_ratelimited(dev->mt76.dev,
				     "beacon upload of link %u (bss_idx %u) failed: %d\n",
				     link->msta_link.wcid.link_id,
				     link->mt76.idx, ret);

	spin_lock_bh(&dev->mt76.lock);
	if (pending && !ret)
		link->crit_update_id = crit_update_id;
	*ambiguous = pending && !ret && link->crit_update_ambiguous;
	if (!ret) {
		link->crit_update_active = crit_update && !*ambiguous;
		link->crit_update_time = jiffies;
	}
	link->crit_update_pending = false;
	link->crit_update_ambiguous = false;
	spin_unlock_bh(&dev->mt76.lock);

	return ret;
}

/* the DTIM Count of a link can stall for several beacon intervals while
 * another link switches channel, so the wait for an event that can get lost
 * ends well after the next DTIM Beacon frame
 */
#define MT7996_CRIT_UPDATE_EVENT_MARGIN	HZ

/* Each template with the critical update TLV starts the critical update in
 * the firmware again, also between the DTIM Beacon frame that ends it and its
 * event. A template of the running update therefore waits for the event,
 * after which mac80211 updates the beacon without the flag. A template that
 * starts a countdown after one without, ends a countdown or removes links
 * goes up at once.
 */
static bool
mt7996_mcu_beacon_crit_update_held(struct mt7996_dev *dev,
				   struct ieee80211_bss_conf *link_conf,
				   struct mt7996_vif_link *link,
				   bool crit_update, u32 crit_update_id,
				   bool cntdwn, u16 links)
{
	unsigned long timeout;
	bool held;

	if (!crit_update || links || link->bcn_cntdwn != cntdwn)
		return false;

	timeout = TU_TO_JIFFIES((u32)(link_conf->dtim_period + 2) *
				link_conf->beacon_int) +
		  MT7996_CRIT_UPDATE_EVENT_MARGIN;

	spin_lock_bh(&dev->mt76.lock);
	held = link->crit_update_active &&
	       crit_update_id == link->crit_update_id &&
	       time_before(jiffies, link->crit_update_time + timeout);
	spin_unlock_bh(&dev->mt76.lock);

	return held;
}

static void
mt7996_mcu_beacon_attlm(struct sk_buff *rskb, struct sk_buff *skb,
			struct ieee80211_vif *vif,
			struct ieee80211_mutable_offsets *offs)
{
	u16 tail_offset = offs->tim_offset + offs->tim_length;
	struct bss_bcn_attlm_offset_tlv *attlm;
	const struct element *elem;
	struct tlv *tlv;

	if (!ieee80211_vif_is_mld(vif))
		return;

	/* the firmware counts the Expected Duration field down once the
	 * mapping is established, which is when the Mapping Switch Time
	 * field leaves the element
	 */
	for_each_element_extid(elem, WLAN_EID_EXT_TID_TO_LINK_MAPPING,
			       skb->data + tail_offset,
			       skb->len - tail_offset) {
		const struct ieee80211_ttlm_elem *ttlm = (const void *)(elem->data + 1);

		if (!ieee80211_tid_to_link_map_size_ok(elem->data + 1,
						       elem->datalen - 1))
			continue;

		if ((ttlm->control & IEEE80211_TTLM_CONTROL_SWITCH_TIME_PRESENT) ||
		    !(ttlm->control & IEEE80211_TTLM_CONTROL_EXPECTED_DUR_PRESENT))
			continue;

		tlv = mt7996_mcu_add_uni_tlv(rskb, UNI_BSS_INFO_BCN_ATTLM,
					     sizeof(*attlm));
		attlm = (struct bss_bcn_attlm_offset_tlv *)tlv;
		attlm->valid_id_bitmap = BIT(0);
		attlm->offset[0] = cpu_to_le16((const u8 *)elem - skb->data);
		return;
	}
}

struct mt7996_mcu_ml_reconf {
	u16 links;
	u16 to_sec;
	u16 offs[IEEE80211_MLD_MAX_NUM_LINKS];
	u16 count[IEEE80211_MLD_MAX_NUM_LINKS];
};

static const struct element *
mt7996_mcu_beacon_ml_reconf_elem(struct sk_buff *skb,
				 struct ieee80211_mutable_offsets *offs)
{
	u16 tail_offset = offs->tim_offset + offs->tim_length;
	const struct element *elem;

	for_each_element_extid(elem, WLAN_EID_EXT_EHT_MULTI_LINK,
			       skb->data + tail_offset,
			       skb->len - tail_offset)
		if (ieee80211_mle_type_ok(elem->data + 1,
					  IEEE80211_ML_CONTROL_TYPE_RECONF,
					  elem->datalen - 1))
			return elem;

	return NULL;
}

/* the firmware rounds the count per second down */
static u16
mt7996_mcu_mld_reconf_per_sec(const struct ieee80211_bss_conf *link_conf)
{
	if (!link_conf->beacon_int)
		return 0;

	return 1000 / link_conf->beacon_int;
}

/* At a beacon interval of 100 TU, a template that went up 63 ms or more before
 * a TBTT was on air at that TBTT in every run. Sends 41.8 and 50.5 ms before a
 * TBTT were on air at that TBTT, and one 48.7 ms before it at the next TBTT, as
 * were sends 32.9 and 13.4 ms before it.
 */
#define MT7996_RECONF_TAKE_US	63000
#define MT7996_RECONF_MISS_US	32900

/* The template that starts a removal goes up in a window where no TBTT of the
 * removed link lies between MT7996_RECONF_MISS_US and MT7996_RECONF_TAKE_US
 * ahead, so that the TBTT of the start is known. The delay of the upload lies
 * before the firmware handles it, so the window keeps that time to each TBTT,
 * whatever the beacon interval. The firmware also delays a template that
 * reaches it from the pre-TBTT pass to about 3 ms after the TBTT, and the
 * partner templates queue behind it, so the window keeps clear of that time.
 */
#define MT7996_RECONF_ALIGN_US		4000
#define MT7996_RECONF_WINDOW_US		10000
#define MT7996_RECONF_WINDOW_MIN_US	1000

/* the first time from a TBTT, at or after @from_us, that keeps clear of both,
 * and the length of the window from it
 */
static u32
mt7996_mcu_beacon_reconf_gap(u32 bi_us, u32 from_us, u32 *start_us)
{
	u32 t = from_us, defer, take;

	while (t < from_us + bi_us) {
		defer = ((t - MT7996_RECONF_ALIGN_US) / bi_us + 1) * bi_us;
		take = ((t + MT7996_RECONF_MISS_US) / bi_us + 1) * bi_us;
		if (t + MT7996_PRE_TBTT_US >= defer) {
			t = defer + MT7996_RECONF_ALIGN_US;
		} else if (t + MT7996_RECONF_TAKE_US > take) {
			t = take - MT7996_RECONF_MISS_US;
		} else {
			*start_us = t;
			return min(defer - MT7996_PRE_TBTT_US - 1,
				   take - MT7996_RECONF_TAKE_US) - t;
		}
	}

	*start_us = t;
	return 0;
}

/* The first window of MT7996_RECONF_WINDOW_US from MT7996_RECONF_ALIGN_US
 * after a TBTT on, or the first of the longest ones where none is that long.
 * Returns its length, with its start after a TBTT in @start_us, or 0 where
 * no window of MT7996_RECONF_WINDOW_MIN_US is left.
 */
static u32
mt7996_mcu_beacon_reconf_window(u32 bi_us, u32 *start_us)
{
	u32 from, start, len, best = 0;

	*start_us = 0;
	for (from = MT7996_RECONF_ALIGN_US;
	     from < MT7996_RECONF_ALIGN_US + bi_us; from = start + len + 1) {
		len = mt7996_mcu_beacon_reconf_gap(bi_us, from, &start);
		if (len >= MT7996_RECONF_WINDOW_US) {
			*start_us = start % bi_us;
			return MT7996_RECONF_WINDOW_US;
		}
		if (len > best) {
			best = len;
			*start_us = start % bi_us;
		}
	}

	return best < MT7996_RECONF_WINDOW_MIN_US ? 0 : best;
}

/* the firmware gives each removal entry of a template the count of the
 * template's link, from the last timer of the AP MLD, and a start needs a
 * window after a TBTT of the removed link. A link whose AP did not start has
 * no beacon interval and no count.
 */
static bool
mt7996_mcu_mld_reconf_countable(struct ieee80211_vif *vif)
{
	unsigned long valid_links = vif->valid_links;
	unsigned int link_id;

	for_each_set_bit(link_id, &valid_links, IEEE80211_MLD_MAX_NUM_LINKS) {
		struct ieee80211_bss_conf *link_conf;
		u32 start;

		link_conf = link_conf_dereference_protected(vif, link_id);
		if (!link_conf || !link_conf->beacon_int)
			continue;
		if (!mt7996_mcu_mld_reconf_per_sec(link_conf) ||
		    !mt7996_mcu_beacon_reconf_window(link_conf->beacon_int * 1024,
						     &start))
			return false;
	}

	return true;
}

static void
mt7996_mcu_beacon_ml_reconf(struct mt7996_dev *dev, struct ieee80211_vif *vif,
			    struct sk_buff *skb,
			    struct ieee80211_mutable_offsets *offs,
			    struct mt7996_mcu_ml_reconf *reconf)
{
	const struct element *elem, *sub;
	unsigned int link_id;

	if (!ieee80211_vif_is_mld(vif) || !mt7996_mcu_mld_reconf_countable(vif))
		return;

	elem = mt7996_mcu_beacon_ml_reconf_elem(skb, offs);
	if (!elem)
		return;

	for_each_mle_subelement(sub, elem->data + 1, elem->datalen - 1) {
		const struct ieee80211_mle_per_sta_profile *prof =
			(const void *)sub->data;
		const u8 *pos = prof->variable;
		u16 control;

		if (sub->id != IEEE80211_MLE_SUBELEM_PER_STA_PROFILE ||
		    !ieee80211_mle_reconf_sta_prof_size_ok(sub->data,
							   sub->datalen))
			continue;

		control = le16_to_cpu(prof->control);
		link_id = control & IEEE80211_MLE_STA_RECONF_CONTROL_LINK_ID;
		if (!(control &
		      IEEE80211_MLE_STA_RECONF_CONTROL_AP_REM_TIMER_PRESENT) ||
		    u16_get_bits(control,
				 IEEE80211_MLE_STA_RECONF_CONTROL_OPERATION_TYPE) !=
		    IEEE80211_MLE_STA_RECONF_CONTROL_OPERATION_TYPE_AP_REM ||
		    link_id >= IEEE80211_MLD_MAX_NUM_LINKS ||
		    !mt7996_vif_link(dev, vif, link_id) ||
		    hweight16(reconf->links) >= __MT_MAX_BAND)
			continue;

		if (control & IEEE80211_MLE_STA_RECONF_CONTROL_STA_MAC_ADDR_PRESENT)
			pos += ETH_ALEN;

		reconf->links |= BIT(link_id);
		reconf->offs[link_id] = pos - skb->data;
		reconf->count[link_id] = get_unaligned_le16(pos);
	}
}

/* The firmware counts the AP Removal Timer of each removed link down in the
 * Beacon frames. It writes the count of the BSS that an entry names, and in
 * a partner's Beacon frame that of the removed BSS is already one lower, so
 * each entry names the BSS of the template, whose count runs from the timer.
 */
static void
mt7996_mcu_beacon_ml_reconf_tlv(struct sk_buff *rskb,
				struct mt7996_vif_link *link,
				const struct mt7996_mcu_ml_reconf *reconf)
{
	struct bss_bcn_ml_reconf_offset *offset;
	struct bss_bcn_ml_reconf_tlv *ml_reconf;
	unsigned long links = reconf->links;
	unsigned int link_id;
	struct tlv *tlv;

	if (!links)
		return;

	tlv = mt7996_mcu_add_uni_tlv(rskb, UNI_BSS_INFO_BCN_ML_RECONF,
				     struct_size(ml_reconf, offset,
						 hweight16(reconf->links)));
	ml_reconf = (struct bss_bcn_ml_reconf_tlv *)tlv;
	ml_reconf->reconf_count = hweight16(reconf->links);

	offset = ml_reconf->offset;
	for_each_set_bit(link_id, &links, IEEE80211_MLD_MAX_NUM_LINKS) {
		offset->ap_removal_timer_offs = cpu_to_le16(reconf->offs[link_id]);
		offset->bss_idx = link->mt76.idx;
		offset++;
	}
}

/* The firmware times the removal in whole seconds, and a time rounded up
 * keeps each count that it puts on air at or above that of the template. On
 * air it changes only the low octet of the count, so the time keeps the
 * count of each link of the AP MLD within that octet.
 */
static u16
mt7996_mcu_mld_reconf_to_sec(struct ieee80211_vif *vif, u16 removed_links,
			     const struct mt7996_mcu_ml_reconf *reconf)
{
	unsigned long valid_links = vif->valid_links;
	u32 to_sec = 0, max_sec = U16_MAX;
	unsigned int link_id;

	for_each_set_bit(link_id, &valid_links, IEEE80211_MLD_MAX_NUM_LINKS) {
		struct ieee80211_bss_conf *link_conf;
		u16 per_sec;

		link_conf = link_conf_dereference_protected(vif, link_id);
		if (!link_conf)
			continue;

		per_sec = mt7996_mcu_mld_reconf_per_sec(link_conf);
		if (!per_sec)
			continue;

		max_sec = min_t(u32, max_sec, U8_MAX / per_sec);
		if (removed_links & BIT(link_id))
			to_sec = max_t(u32, to_sec,
				       DIV_ROUND_UP(reconf->count[link_id],
						    per_sec));
	}

	return min_t(u32, to_sec, max_sec);
}

/* the firmware puts the count of its timer into the Beacon frames, not the
 * count of the template
 */
static u16
mt7996_mcu_mld_reconf_count(const struct ieee80211_bss_conf *link_conf,
			    u16 to_sec)
{
	return mt7996_mcu_mld_reconf_per_sec(link_conf) * to_sec;
}

/* The TBTTs of the reference link from the first one that the firmware
 * counts on the link of the template to the removal TBTT. A TBTT of the link
 * counts from the reference TBTT at or before it, so an unequal beacon
 * interval gives a count at or above the true one.
 */
static u16
mt7996_mcu_mld_reconf_left(u64 tsf, u32 bi_us, u32 ref_bi_us, u64 tbtt_tsf)
{
	u64 next, end;

	if (!bi_us || !ref_bi_us)
		return 0;

	next = div_u64(tsf + MT7996_PRE_TBTT_US, bi_us) + 1;
	next = div_u64(next * bi_us, ref_bi_us);
	end = div_u64(tbtt_tsf, ref_bi_us);

	return end > next ? min_t(u64, end - next, U16_MAX) : 0;
}

static int
mt7996_mcu_mld_reconf_timer(struct mt7996_dev *dev, struct ieee80211_vif *vif,
			    u16 to_sec)
{
	struct mld_req_hdr hdr = { .mld_idx = 0xff };
	unsigned long valid_links = vif->valid_links;
	struct mld_reconf_timer *rt;
	unsigned int link_id;
	struct sk_buff *skb;
	struct tlv *tlv;

	skb = mt76_mcu_msg_alloc(&dev->mt76, NULL, sizeof(hdr) + sizeof(*rt));
	if (!skb)
		return -ENOMEM;

	memcpy(hdr.mld_addr, vif->addr, ETH_ALEN);
	skb_put_data(skb, &hdr, sizeof(hdr));

	/* the firmware starts no count for the removal entries of a template
	 * of a link outside the bitmap, and counts the template of a staying
	 * link by itself
	 */
	tlv = mt7996_mcu_add_uni_tlv(skb, UNI_CMD_MLD_RECONF_AP_REM_TIMER,
				     sizeof(*rt));
	rt = (struct mld_reconf_timer *)tlv;
	rt->link_bitmap = cpu_to_le16(vif->valid_links);
	/* the firmware skips a band whose index names no BSS */
	memset(rt->bss_idx, 0xff, sizeof(rt->bss_idx));

	for_each_set_bit(link_id, &valid_links, IEEE80211_MLD_MAX_NUM_LINKS) {
		struct mt7996_vif_link *link = mt7996_vif_link(dev, vif, link_id);
		u8 band_idx;

		if (!link)
			continue;

		band_idx = link->mt76.band_idx;
		rt->to_sec[band_idx] = cpu_to_le16(to_sec);
		rt->bss_idx[band_idx] = link->mt76.idx;
	}

	return mt76_mcu_skb_send_msg(&dev->mt76, skb, MCU_WM_UNI_CMD(MLD),
				     true);
}

/* a timer command names a BSS only for a link that the driver has added */
static u16
mt7996_mcu_mld_reconf_bss_links(struct ieee80211_vif *vif)
{
	struct mt7996_vif *mvif = (struct mt7996_vif *)vif->drv_priv;

	return vif->valid_links & mvif->mt76.valid_links;
}

/* a link whose TX stop is due or done, or whose end off air the driver
 * reported, has ended its removal
 */
static u16
mt7996_mcu_beacon_reconf_new(const struct mt7996_vif *mvif, u16 links)
{
	return links & ~(mvif->reconf_links | mvif->reconf_stop_links |
			 mvif->reconf_done_links | mvif->reconf_off_air_links);
}

/* The first template that names a removed link starts its timer, and each
 * template of the removal takes the time of that timer. A full reset drops
 * the links until the restart adds them again. The firmware counts each
 * removal entry of a template from the last timer of the AP MLD, so the
 * template keeps only the entries of the running removal and of the links
 * that it starts.
 */
static u16
mt7996_mcu_beacon_reconf_start(struct mt7996_dev *dev,
			       struct ieee80211_vif *vif,
			       struct mt7996_mcu_ml_reconf *reconf)
{
	struct mt7996_vif *mvif = (struct mt7996_vif *)vif->drv_priv;
	u16 links, running, to_sec;

	spin_lock_bh(&dev->mt76.lock);
	running = reconf->links & mvif->reconf_links;
	links = mt7996_mcu_beacon_reconf_new(mvif, reconf->links);
	if (mvif->reconf_links)
		reconf->to_sec = mvif->reconf_to_sec;
	spin_unlock_bh(&dev->mt76.lock);

	reconf->links = running;
	if (!links || (links & ~mvif->mt76.valid_links))
		return 0;

	to_sec = mt7996_mcu_mld_reconf_to_sec(vif, links, reconf);
	if (mt7996_mcu_mld_reconf_timer(dev, vif, to_sec))
		return 0;

	spin_lock_bh(&dev->mt76.lock);
	mvif->reconf_links |= links;
	mvif->reconf_wait_removed &= ~links;
	mvif->reconf_to_sec = to_sec;
	mvif->reconf_cmd_links = mt7996_mcu_mld_reconf_bss_links(vif);
	spin_unlock_bh(&dev->mt76.lock);

	reconf->links |= links;
	reconf->to_sec = to_sec;

	return links;
}

/* A beacon disable clears the count of the firmware, which then counts the
 * next template of the link from the time that the last timer gave. A
 * template of a running removal that the firmware does not count therefore
 * sends the time that is left first, and carries no removal entry where none
 * is left. The firmware starts no count for a link that the last timer did
 * not name with a BSS, so a change of those links also sends the timer again.
 * The firmware offsets the TSF of each band and starts the count at a pass
 * of the template's link, so the time left runs from the TSF of that link,
 * in TBTTs of the link whose TSF gave the removal TBTT.
 */
static void
mt7996_mcu_beacon_reconf_resume(struct mt7996_dev *dev,
				struct ieee80211_vif *vif,
				struct ieee80211_bss_conf *link_conf,
				struct mt7996_vif_link *link,
				struct mt7996_mcu_ml_reconf *reconf)
{
	struct mt7996_vif *mvif = (struct mt7996_vif *)vif->drv_priv;
	struct mt7996_mcu_ml_reconf left = {};
	unsigned int link_id = link_conf->link_id;
	struct ieee80211_bss_conf *ref_conf;
	bool counted, armed, named;
	unsigned int ref_id;
	u64 tbtt_tsf, tsf;
	u16 to_sec;

	spin_lock_bh(&dev->mt76.lock);
	counted = mvif->reconf_bcn_links & BIT(link_id);
	armed = mvif->reconf_tbtt_links & reconf->links;
	named = mvif->reconf_cmd_links == mt7996_mcu_mld_reconf_bss_links(vif);
	tbtt_tsf = mvif->reconf_tbtt_tsf;
	ref_id = mvif->reconf_tbtt_link;
	spin_unlock_bh(&dev->mt76.lock);

	if (counted || !reconf->links)
		return;

	to_sec = reconf->to_sec;
	if (armed) {
		ref_conf = link_conf_dereference_protected(vif, ref_id);
		if (ref_conf && mt7996_vif_link(dev, vif, ref_id)) {
			tsf = __mt7996_get_tsf(mt76_hw(dev), link);
			left.count[ref_id] =
				mt7996_mcu_mld_reconf_left(tsf,
							   link_conf->beacon_int * 1024,
							   ref_conf->beacon_int * 1024,
							   tbtt_tsf);
		}
		if (!left.count[ref_id]) {
			reconf->links = 0;
			return;
		}

		to_sec = mt7996_mcu_mld_reconf_to_sec(vif, BIT(ref_id), &left);
	}

	if ((to_sec != reconf->to_sec || !named) &&
	    mt7996_mcu_mld_reconf_timer(dev, vif, to_sec)) {
		reconf->links = 0;
		return;
	}

	spin_lock_bh(&dev->mt76.lock);
	mvif->reconf_to_sec = to_sec;
	mvif->reconf_cmd_links = mt7996_mcu_mld_reconf_bss_links(vif);
	spin_unlock_bh(&dev->mt76.lock);

	reconf->to_sec = to_sec;
}

/* On air the firmware changes only the low octet of the AP Removal Timer, so
 * each entry of a template carries the count that the firmware applies to the
 * link of the template.
 */
static void
mt7996_mcu_beacon_reconf_count_set(const struct ieee80211_bss_conf *link_conf,
				   const struct mt7996_mcu_ml_reconf *reconf,
				   u8 *frame)
{
	unsigned long links = reconf->links;
	unsigned int link_id;
	u16 count;

	if (!reconf->to_sec)
		return;

	count = mt7996_mcu_mld_reconf_count(link_conf, reconf->to_sec);
	for_each_set_bit(link_id, &links, IEEE80211_MLD_MAX_NUM_LINKS)
		put_unaligned_le16(count, frame + reconf->offs[link_id]);
}

/* the time from the last window start of the first removed link, with the
 * TSF of that link and its beacon interval; a beacon interval of 0 for no
 * link and for a link without a channel, which has no TSF
 */
static u32
mt7996_mcu_beacon_reconf_phase(struct mt7996_dev *dev,
			       struct ieee80211_vif *vif, u16 links, u64 *tsf,
			       u32 *bi_us)
{
	unsigned int link_id = ffs(links) - 1;
	struct ieee80211_bss_conf *link_conf;
	struct mt7996_vif_link *link;
	u64 pos;
	u32 start;

	*tsf = 0;
	*bi_us = 0;
	link_conf = link_conf_dereference_protected(vif, link_id);
	link = mt7996_vif_link(dev, vif, link_id);
	if (!link_conf || !link || !link_conf->beacon_int ||
	    !mt7996_vif_link_phy(link))
		return 0;

	*bi_us = link_conf->beacon_int * 1024;
	mt7996_mcu_beacon_reconf_window(*bi_us, &start);
	*tsf = __mt7996_get_tsf(mt76_hw(dev), link);
	pos = *tsf + *bi_us - start;

	return do_div(pos, *bi_us);
}

static bool
mt7996_mcu_beacon_reconf_outside(u32 phase, u32 bi_us)
{
	u32 start;

	return bi_us && phase > mt7996_mcu_beacon_reconf_window(bi_us, &start);
}

static void
mt7996_mcu_beacon_reconf_wait(struct mt7996_dev *dev,
			      struct ieee80211_vif *vif, unsigned int link_id,
			      u16 removed, u32 wait_us)
{
	struct mt7996_vif *mvif = (struct mt7996_vif *)vif->drv_priv;

	spin_lock_bh(&dev->mt76.lock);
	if (!mvif->reconf_wait_links)
		mvif->reconf_wait_link = link_id;
	mvif->reconf_wait_links |= BIT(link_id);
	mvif->reconf_wait_removed |= removed;
	spin_unlock_bh(&dev->mt76.lock);

	if (wait_us)
		hrtimer_start(&mvif->reconf_start_timer,
			      ns_to_ktime(ktime_get_boottime_ns() +
					  (u64)wait_us * NSEC_PER_USEC),
			      HRTIMER_MODE_ABS_SOFT);
}

/* A removed link without a channel sends no Beacon frames and has no TSF that
 * a removal TBTT could count in, so its removal ends at once, and the template
 * goes up with no removal entry for it. The templates of the other links name
 * it until user space drops it, so its end, or the expiry before it, is
 * reported once.
 */
static void
mt7996_mcu_beacon_reconf_off_air(struct mt7996_dev *dev,
				 struct ieee80211_vif *vif,
				 struct mt7996_mcu_ml_reconf *reconf)
{
	struct mt7996_vif *mvif = (struct mt7996_vif *)vif->drv_priv;
	unsigned long links = reconf->links;
	u16 ended = 0, reported;
	unsigned int link_id;

	for_each_set_bit(link_id, &links, IEEE80211_MLD_MAX_NUM_LINKS) {
		struct mt7996_vif_link *link = mt7996_vif_link(dev, vif, link_id);

		if (link && (!link_conf_dereference_protected(vif, link_id) ||
			     !mt7996_vif_link_phy(link)))
			ended |= BIT(link_id);
	}

	if (!ended)
		return;

	reconf->links &= ~ended;

	spin_lock_bh(&dev->mt76.lock);
	reported = mvif->reconf_off_air_links | mvif->reconf_done_links;
	mvif->reconf_off_air_links |= ended;
	spin_unlock_bh(&dev->mt76.lock);

	ended &= ~reported;
	if (ended)
		ieee80211_links_removed(vif, ended);
}

/* mac80211 takes an expired removal that a beacon still announces as no new
 * one until the link goes
 */
void mt7996_mcu_beacon_reconf_gone(struct mt7996_dev *dev,
				   struct ieee80211_vif *vif, u16 links)
{
	struct mt7996_vif *mvif = (struct mt7996_vif *)vif->drv_priv;

	spin_lock_bh(&dev->mt76.lock);
	mvif->reconf_done_links &= ~links;
	mvif->reconf_off_air_links &= ~links;
	spin_unlock_bh(&dev->mt76.lock);
}

/* Returns true for a template that waits. A template that would start a
 * removal outside the window waits for the next window, and so do the
 * templates of the other links that come before it.
 */
static bool
mt7996_mcu_beacon_reconf_waits(struct mt7996_dev *dev,
			       struct ieee80211_vif *vif, unsigned int link_id,
			       const struct mt7996_mcu_ml_reconf *reconf)
{
	struct mt7996_vif *mvif = (struct mt7996_vif *)vif->drv_priv;
	u32 phase, bi_us;
	u16 links, waiting;
	u64 tsf;

	spin_lock_bh(&dev->mt76.lock);
	links = mt7996_mcu_beacon_reconf_new(mvif, reconf->links);
	waiting = mvif->reconf_wait_links;
	spin_unlock_bh(&dev->mt76.lock);

	if (!links || (links & ~mvif->mt76.valid_links))
		return false;

	if (waiting) {
		mt7996_mcu_beacon_reconf_wait(dev, vif, link_id, links, 0);
		return true;
	}

	phase = mt7996_mcu_beacon_reconf_phase(dev, vif, links, &tsf, &bi_us);
	if (!mt7996_mcu_beacon_reconf_outside(phase, bi_us))
		return false;

	mt7996_mcu_beacon_reconf_wait(dev, vif, link_id, links, bi_us - phase);
	return true;
}

/* A start whose template did not go up, or whose timer command ended after
 * the window, waits for the next window. After the last try the removal counts
 * from the upload all the same, so that the TX of the removed links stops in
 * any case.
 */
#define MT7996_RECONF_START_TRIES	3

static bool
mt7996_mcu_beacon_reconf_retry(struct mt7996_dev *dev,
			       struct ieee80211_vif *vif, unsigned int link_id,
			       u16 started)
{
	struct mt7996_vif *mvif = (struct mt7996_vif *)vif->drv_priv;
	u32 phase, bi_us;
	u64 tsf;

	phase = mt7996_mcu_beacon_reconf_phase(dev, vif, started, &tsf, &bi_us);
	if (!bi_us)
		return false;

	spin_lock_bh(&dev->mt76.lock);
	if (++mvif->reconf_start_tries >= MT7996_RECONF_START_TRIES) {
		spin_unlock_bh(&dev->mt76.lock);
		return false;
	}
	mvif->reconf_links &= ~started;
	if (!mvif->reconf_links)
		mvif->reconf_bcn_links = 0;
	spin_unlock_bh(&dev->mt76.lock);

	mt7996_mcu_beacon_reconf_wait(dev, vif, link_id, started,
				      bi_us - phase);
	return true;
}

/* one timer stops the TX of all links that a template starts, at the
 * removal TBTT of the first one, so each link reports that time
 */
static void
mt7996_mcu_beacon_reconf_report(struct mt7996_dev *dev,
				struct ieee80211_vif *vif, u16 links,
				const struct mt7996_mcu_ml_reconf *reconf,
				struct mt7996_vif_link *ref, u64 tbtt_tsf,
				u64 tbtt)
{
	struct cfg80211_ap_removal_timer_event event = {
		.tbtt_boottime = tbtt,
	};
	struct ieee80211_hw *hw = mt76_hw(dev);
	unsigned long started = links;
	unsigned int link_id;

	for_each_set_bit(link_id, &started, IEEE80211_MLD_MAX_NUM_LINKS) {
		struct ieee80211_bss_conf *link_conf;
		struct mt7996_vif_link *link;

		link_conf = link_conf_dereference_protected(vif, link_id);
		link = mt7996_vif_link(dev, vif, link_id);
		if (!link_conf || !link)
			continue;

		event.link_id = link_id;
		event.count = mt7996_mcu_mld_reconf_count(link_conf,
							  reconf->to_sec);
		event.tsf = tbtt_tsf;
		if (link != ref)
			event.tsf += __mt7996_get_tsf(hw, link) -
				     __mt7996_get_tsf(hw, ref);
		ieee80211_ap_removal_timer_started(vif, &event, GFP_KERNEL);
	}
}

/* stops the TX of the removed links just before their removal TBTT */
static void
mt7996_mcu_beacon_reconf_arm(struct mt7996_dev *dev, struct ieee80211_vif *vif,
			     u16 links, const struct mt7996_mcu_ml_reconf *reconf,
			     u64 upload_tsf)
{
	struct mt7996_vif *mvif = (struct mt7996_vif *)vif->drv_priv;
	u64 t0, tbtt_tsf, ref_tsf, ref_boottime, tbtt;
	unsigned int link_id = ffs(links) - 1;
	struct ieee80211_bss_conf *link_conf;
	struct mt7996_vif_link *link;
	u32 bi_us;
	u16 count;

	link_conf = link_conf_dereference_protected(vif, link_id);
	link = mt7996_vif_link(dev, vif, link_id);
	if (!upload_tsf || !link_conf || !link || !link_conf->beacon_int)
		return;

	bi_us = link_conf->beacon_int * 1024;
	count = mt7996_mcu_mld_reconf_count(link_conf, reconf->to_sec);
	t0 = div_u64(upload_tsf + MT7996_RECONF_TAKE_US + bi_us - 1, bi_us);
	tbtt_tsf = (t0 + count) * bi_us;

	ref_tsf = __mt7996_get_tsf(mt76_hw(dev), link);
	ref_boottime = ktime_get_boottime_ns();
	tbtt = mt7996_tsf_to_boottime(tbtt_tsf, ref_tsf, ref_boottime);

	spin_lock_bh(&dev->mt76.lock);
	mvif->reconf_tbtt_links = links;
	mvif->reconf_timer_links = links;
	mvif->reconf_tbtt_boottime = tbtt;
	mvif->reconf_tbtt_tsf = tbtt_tsf;
	mvif->reconf_tbtt_link = link_id;
	mvif->reconf_start_tries = 0;
	spin_unlock_bh(&dev->mt76.lock);

	hrtimer_start(&mvif->reconf_timer,
		      ns_to_ktime(tbtt - MT7996_RECONF_STOP_LEAD_US *
				  NSEC_PER_USEC),
		      HRTIMER_MODE_ABS_SOFT);
	dev_dbg(dev->mt76.dev,
		"removal of links 0x%x at TSF %llu of link %u, count %u\n",
		links, tbtt_tsf, link_id, count);

	mt7996_mcu_beacon_reconf_report(dev, vif, links, reconf, link,
					tbtt_tsf, tbtt);
}

static void
mt7996_mcu_beacon_reconf_set(struct mt7996_dev *dev, struct ieee80211_vif *vif,
			     unsigned int link_id)
{
	struct mt7996_vif *mvif = (struct mt7996_vif *)vif->drv_priv;

	spin_lock_bh(&dev->mt76.lock);
	if (mvif->reconf_links)
		mvif->reconf_bcn_links |= BIT(link_id);
	spin_unlock_bh(&dev->mt76.lock);
}

/* The firmware places the count of a later template by an offset that it
 * corrects with the TIM length of the template before, so a link keeps the
 * template that it had at the start of the removal until the expiry.
 */
static bool
mt7996_mcu_beacon_reconf_held(struct mt7996_dev *dev,
			      struct ieee80211_vif *vif, unsigned int link_id)
{
	struct mt7996_vif *mvif = (struct mt7996_vif *)vif->drv_priv;
	bool held;

	spin_lock_bh(&dev->mt76.lock);
	held = mvif->reconf_links && (mvif->reconf_bcn_links & BIT(link_id));
	spin_unlock_bh(&dev->mt76.lock);

	return held;
}

void mt7996_mcu_beacon_reconf_stop(struct mt7996_dev *dev,
				   struct ieee80211_vif *vif,
				   unsigned int link_id, bool removed)
{
	struct mt7996_vif *mvif = (struct mt7996_vif *)vif->drv_priv;
	bool timer = false, ended = false, wait_ended = false;
	u16 waiting;

	spin_lock_bh(&dev->mt76.lock);
	/* A removed link that stops ends its removal before its expiry, also
	 * a removal whose start waits. The link sends no Beacon frames from
	 * now on, so the removal ends at once.
	 */
	if (removed) {
		ended = (mvif->reconf_links | mvif->reconf_stop_links |
			 mvif->reconf_wait_removed) & BIT(link_id);
		wait_ended = mvif->reconf_wait_removed == BIT(link_id);
		mvif->reconf_wait_removed &= ~BIT(link_id);
	}
	if (removed && (mvif->reconf_links & BIT(link_id))) {
		mvif->reconf_links &= ~BIT(link_id);
		if (!mvif->reconf_links)
			mvif->reconf_bcn_links = 0;
	}
	if (removed) {
		mvif->reconf_stop_links &= ~BIT(link_id);
		if (ended)
			mvif->reconf_off_air_links |= BIT(link_id);
		mvif->reconf_tbtt_links &= ~BIT(link_id);
		timer = mvif->reconf_timer_links & BIT(link_id);
		mvif->reconf_timer_links &= ~BIT(link_id);
		if (mvif->reconf_timer_links)
			timer = false;
	}
	mvif->reconf_bcn_links &= ~BIT(link_id);
	mvif->reconf_wait_links &= ~BIT(link_id);
	if (!mvif->reconf_wait_links)
		mvif->reconf_start_tries = 0;
	waiting = mvif->reconf_wait_links;
	spin_unlock_bh(&dev->mt76.lock);

	if (timer)
		hrtimer_cancel(&mvif->reconf_timer);

	if (!ended)
		return;

	/* the templates that waited for the start start nothing now */
	if (wait_ended) {
		hrtimer_cancel(&mvif->reconf_start_timer);
		if (waiting)
			queue_work(system_highpri_wq, &dev->reconf_work);
	}
	ieee80211_links_removed(vif, BIT(link_id));
}

/* the restart adds the driver link of a link with a channel again */
static u16
mt7996_mcu_beacon_reconf_chan_links(struct ieee80211_vif *vif, u16 links)
{
	unsigned long candidates = links;
	unsigned int link_id;
	u16 chan_links = 0;

	for_each_set_bit(link_id, &candidates, IEEE80211_MLD_MAX_NUM_LINKS) {
		struct ieee80211_bss_conf *link_conf;

		link_conf = link_conf_dereference_protected(vif, link_id);
		if (link_conf && rcu_access_pointer(link_conf->chanctx_conf))
			chan_links |= BIT(link_id);
	}

	return chan_links;
}

/* The reset firmware holds no removal timer, so the templates of the restart
 * start the removal again. A removal whose TX stop is due keeps it, and so
 * does a removal whose TX stop is done on a link that has a channel, since
 * the reset firmware sends on that link again.
 */
void mt7996_mcu_beacon_reconf_reset(struct mt7996_dev *dev,
				    struct ieee80211_vif *vif)
{
	struct mt7996_vif *mvif = (struct mt7996_vif *)vif->drv_priv;

	hrtimer_cancel(&mvif->reconf_timer);
	hrtimer_cancel(&mvif->reconf_start_timer);

	spin_lock_bh(&dev->mt76.lock);
	mvif->reconf_stop_links |=
		mt7996_mcu_beacon_reconf_chan_links(vif,
						    mvif->reconf_done_links);
	mvif->reconf_links = 0;
	mvif->reconf_bcn_links = 0;
	mvif->reconf_tbtt_links = 0;
	mvif->reconf_timer_links = 0;
	mvif->reconf_wait_links = 0;
	mvif->reconf_wait_removed = 0;
	mvif->reconf_start_tries = 0;
	spin_unlock_bh(&dev->mt76.lock);
}

static bool
mt7996_mcu_beacon_reconf_prof_strip(const struct element *sub, u16 strip_links)
{
	const struct ieee80211_mle_per_sta_profile *prof = (const void *)sub->data;

	return sub->id == IEEE80211_MLE_SUBELEM_PER_STA_PROFILE &&
	       sub->datalen >= sizeof(*prof) &&
	       (BIT(le16_to_cpu(prof->control) &
		    IEEE80211_MLE_STA_RECONF_CONTROL_LINK_ID) & strip_links);
}

static void
mt7996_mcu_beacon_cut(struct sk_buff *skb,
		      struct ieee80211_mutable_offsets *offs, u8 *start,
		      u16 len)
{
	u16 pos = start - skb->data;
	int i;

	memmove(start, start + len, skb->len - pos - len);
	skb_trim(skb, skb->len - len);

	for (i = 0; i < ARRAY_SIZE(offs->cntdwn_counter_offs); i++)
		if (offs->cntdwn_counter_offs[i] > pos)
			offs->cntdwn_counter_offs[i] -= len;
	if (offs->mbssid_off > pos)
		offs->mbssid_off -= len;
}

struct mt7996_mcu_rnr_strip {
	u16 links;
	struct ieee80211_neighbor_ap_info *info;
	u8 *tbtt;
};

static enum cfg80211_rnr_iter_ret
mt7996_mcu_beacon_rnr_strip_iter(void *_data, u8 type,
				 const struct ieee80211_neighbor_ap_info *info,
				 const u8 *tbtt_info, u8 tbtt_info_len)
{
	const struct ieee80211_tbtt_info_ge_11 *ti = (const void *)tbtt_info;
	const struct ieee80211_rnr_mld_params *mld;
	struct mt7996_mcu_rnr_strip *data = _data;

	if (type == IEEE80211_TBTT_INFO_TYPE_TBTT &&
	    tbtt_info_len >= offsetofend(struct ieee80211_tbtt_info_ge_11,
					 mld_params))
		mld = &ti->mld_params;
	else if (type == IEEE80211_TBTT_INFO_TYPE_MLD &&
		 tbtt_info_len >= sizeof(*mld))
		mld = (const void *)tbtt_info;
	else
		return RNR_ITER_CONTINUE;

	/* AP MLD ID 0 names the AP MLD of the reporting AP */
	if (mld->mld_id ||
	    !(BIT(le16_get_bits(mld->params, IEEE80211_RNR_MLD_PARAMS_LINK_ID)) &
	      data->links))
		return RNR_ITER_CONTINUE;

	data->info = (struct ieee80211_neighbor_ap_info *)info;
	data->tbtt = (u8 *)tbtt_info;

	return RNR_ITER_BREAK;
}

/* Takes the TBTT Information fields of @strip_links of the own AP MLD out of
 * the Reduced Neighbor Report elements of a template, and each Neighbor AP
 * Information field and element that is left empty.
 */
static void
mt7996_mcu_beacon_rnr_strip(struct sk_buff *skb,
			    struct ieee80211_mutable_offsets *offs,
			    u16 strip_links)
{
	u16 tail_offset = offs->tim_offset + offs->tim_length;

	/* a cut moves the rest of the template, so each cut starts over */
	for (;;) {
		struct mt7996_mcu_rnr_strip data = {
			.links = strip_links,
		};
		const struct element *elem;
		u8 *elem_start, *cut;
		u8 count;
		u16 len;

		for_each_element_id(elem, WLAN_EID_REDUCED_NEIGHBOR_REPORT,
				    skb->data + tail_offset,
				    skb->len - tail_offset) {
			cfg80211_iter_rnr((const u8 *)elem,
					  sizeof(*elem) + elem->datalen,
					  mt7996_mcu_beacon_rnr_strip_iter,
					  &data);
			if (data.tbtt)
				break;
		}
		if (!data.tbtt)
			return;

		elem_start = (u8 *)elem;
		cut = data.tbtt;
		len = data.info->tbtt_info_len;
		count = u8_get_bits(data.info->tbtt_info_hdr,
				    IEEE80211_AP_INFO_TBTT_HDR_COUNT);
		if (count) {
			u8p_replace_bits(&data.info->tbtt_info_hdr, count - 1,
					 IEEE80211_AP_INFO_TBTT_HDR_COUNT);
		} else {
			cut = (u8 *)data.info;
			len += sizeof(*data.info);
		}

		if (len == elem->datalen) {
			cut = elem_start;
			len += sizeof(*elem);
		} else {
			elem_start[1] -= len;
		}

		mt7996_mcu_beacon_cut(skb, offs, cut, len);
	}
}

/* Takes @strip_links out of a template: their TBTT Information fields out of
 * the Reduced Neighbor Report elements, and their per-STA profiles out of the
 * Reconfiguration Multi-Link element, with the element where none is left.
 */
static void
mt7996_mcu_beacon_reconf_strip(struct sk_buff *skb,
			       struct ieee80211_mutable_offsets *offs,
			       u16 strip_links)
{
	const struct element *elem, *sub;
	u8 *elem_start;
	bool kept = false;

	mt7996_mcu_beacon_rnr_strip(skb, offs, strip_links);

	elem = mt7996_mcu_beacon_ml_reconf_elem(skb, offs);
	if (!elem)
		return;

	for_each_mle_subelement(sub, elem->data + 1, elem->datalen - 1)
		if (!mt7996_mcu_beacon_reconf_prof_strip(sub, strip_links))
			kept = true;

	elem_start = (u8 *)elem;
	if (!kept) {
		mt7996_mcu_beacon_cut(skb, offs, elem_start,
				      sizeof(*elem) + elem->datalen);
		return;
	}

	/* a cut moves the rest of the element, so each cut starts over */
	for (;;) {
		const struct element *cut = NULL;

		elem = (const void *)elem_start;
		for_each_mle_subelement(sub, elem->data + 1, elem->datalen - 1) {
			if (mt7996_mcu_beacon_reconf_prof_strip(sub,
								strip_links)) {
				cut = sub;
				break;
			}
		}
		if (!cut)
			return;

		elem_start[1] -= sizeof(*cut) + cut->datalen;
		mt7996_mcu_beacon_cut(skb, offs, (u8 *)cut,
				      sizeof(*cut) + cut->datalen);
	}
}

static void
mt7996_mcu_beacon_cont(struct mt7996_dev *dev,
		       struct ieee80211_bss_conf *link_conf,
		       struct mt7996_vif_link *link,
		       struct sk_buff *rskb, struct sk_buff *skb,
		       struct bss_bcn_content_tlv *bcn,
		       struct ieee80211_mutable_offsets *offs)
{
	u8 *buf, keyidx = link->msta_link.wcid.hw_key_idx2;
	struct mt76_wcid *wcid;

	if (is_mt7990(&dev->mt76) && (keyidx == 6 || keyidx == 7))
		wcid = &link->msta_link.wcid;
	else
		wcid = &dev->mt76.global_wcid;

	bcn->pkt_len = cpu_to_le16(MT_TXD_SIZE + skb->len);
	bcn->tim_ie_pos = cpu_to_le16(offs->tim_offset);

	if (offs->cntdwn_counter_offs[0]) {
		u16 offset = offs->cntdwn_counter_offs[0];

		if (link_conf->csa_active)
			bcn->csa_ie_pos = cpu_to_le16(offset - 4);
		if (link_conf->color_change_active)
			bcn->bcc_ie_pos = cpu_to_le16(offset - 3);
	}

	buf = (u8 *)bcn + sizeof(*bcn);
	mt7996_mac_write_txwi(dev, (__le32 *)buf, skb, wcid, NULL, 0, 0,
			      BSS_CHANGED_BEACON, link_conf->link_id);

	memcpy(buf + MT_TXD_SIZE, skb->data, skb->len);
}

static int
__mt7996_mcu_add_beacon(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
			struct ieee80211_bss_conf *link_conf,
			struct mt7996_vif_link *link, bool enabled,
			u16 strip_links)
{
	struct mt76_phy *mphy = mt76_vif_link_phy(&link->mt76);
	struct mt7996_dev *dev = mt7996_hw_dev(hw);
	struct mt76_vif_link *mlink = &link->mt76;
	struct mt7996_mcu_ml_reconf reconf = {};
	struct ieee80211_mutable_offsets offs;
	struct ieee80211_tx_info *info;
	struct sk_buff *again = NULL;
	struct sk_buff *skb, *rskb;
	struct tlv *tlv;
	struct bss_bcn_content_tlv *bcn;
	bool crit_update = false, cntdwn = false, ambiguous;
	int len, extra_len = 0, ret, cnt;
	u64 upload_tsf = 0, now = ktime_get_boottime_ns();
	unsigned int reconf_len;
	u16 started = 0;
	u32 phase, bi_us;

	/* a later CSA needs a new template, also after a failed upload */
	if (!link_conf->csa_active)
		link->csa_bcn_uploaded = false;

	if (mphy && mphy->offchannel)
		enabled = false;

	if (!mt7996_mcu_beacon_cntdwn_prepare(dev, vif, link_conf, link,
					      enabled, now))
		return 0;

	rskb = __mt7996_mcu_alloc_bss_req(&dev->mt76, mlink,
					  MT7996_MAX_BSS_OFFLOAD_SIZE);
	if (IS_ERR(rskb))
		return PTR_ERR(rskb);

	skb = ieee80211_beacon_get_template(hw, vif, &offs, link_conf->link_id);
	if (enabled && !skb) {
		dev_kfree_skb(rskb);
		return -EINVAL;
	}

	if (skb) {
		if (skb->len > MT7996_MAX_BEACON_SIZE) {
			dev_err(dev->mt76.dev, "Bcn size limit exceed\n");
			dev_kfree_skb(rskb);
			dev_kfree_skb(skb);
			return -EINVAL;
		}

		if (strip_links)
			mt7996_mcu_beacon_reconf_strip(skb, &offs, strip_links);

		extra_len = skb->len;
	}

	len = ALIGN(sizeof(*bcn) + MT_TXD_SIZE + extra_len, 4);
	tlv = mt7996_mcu_add_uni_tlv(rskb, UNI_BSS_INFO_BCN_CONTENT, len);
	bcn = (struct bss_bcn_content_tlv *)tlv;
	bcn->enable = enabled;
	if (!bcn->enable)
		goto out;

	if (!mt7996_mcu_beacon_cntdwn_count(dev, vif, link_conf, link, skb,
					    &offs, now, &cnt)) {
		dev_kfree_skb(skb);
		dev_kfree_skb(rskb);
		return 0;
	}

	info = IEEE80211_SKB_CB(skb);
	info->hw_queue |= FIELD_PREP(MT_TX_HW_QUEUE_PHY, mlink->band_idx);

	mt7996_mcu_beacon_cont(dev, link_conf, link, rskb, skb, bcn, &offs);
	if (link_conf->bssid_indicator)
		mt7996_mcu_beacon_mbss(rskb, skb, bcn, &offs);
	mt7996_mcu_beacon_cntdwn(rskb, skb, &offs, link_conf->csa_active, cnt);
	cntdwn = !!offs.cntdwn_counter_offs[0];
	crit_update = mt7996_mcu_beacon_crit_update(rskb, skb, vif, &offs);
	mt7996_mcu_beacon_attlm(rskb, skb, vif, &offs);
	mt7996_mcu_beacon_ml_reconf(dev, vif, skb, &offs, &reconf);
	if (mt7996_mcu_beacon_crit_update_held(dev, link_conf, link, crit_update,
					       offs.crit_update_id, cntdwn,
					       strip_links | reconf.links)) {
		dev_kfree_skb(skb);
		dev_kfree_skb(rskb);
		return 0;
	}
	if (reconf.links)
		mt7996_mcu_beacon_reconf_off_air(dev, vif, &reconf);
	if (reconf.links &&
	    mt7996_mcu_beacon_reconf_waits(dev, vif, link_conf->link_id,
					   &reconf)) {
		dev_kfree_skb(skb);
		dev_kfree_skb(rskb);
		return 0;
	}
	if (reconf.links)
		started = mt7996_mcu_beacon_reconf_start(dev, vif, &reconf);
	/* the timer command takes time, so the TSF that the start counts from
	 * comes after it, and must still lie in the window
	 */
	if (started) {
		phase = mt7996_mcu_beacon_reconf_phase(dev, vif, started,
						       &upload_tsf, &bi_us);
		if (mt7996_mcu_beacon_reconf_outside(phase, bi_us) &&
		    mt7996_mcu_beacon_reconf_retry(dev, vif, link_conf->link_id,
						   started)) {
			dev_kfree_skb(skb);
			dev_kfree_skb(rskb);
			return 0;
		}
	}
	if (!started)
		mt7996_mcu_beacon_reconf_resume(dev, vif, link_conf, link,
						&reconf);
	reconf_len = rskb->len;
	mt7996_mcu_beacon_ml_reconf_tlv(rskb, link, &reconf);
	mt7996_mcu_beacon_reconf_count_set(link_conf, &reconf,
					   (u8 *)bcn + sizeof(*bcn) +
					   MT_TXD_SIZE);
	/* a deferred template and a refused tag end the command with no rerun,
	 * and the other tags go up with the lengths that the handlers test
	 */
	if (reconf.links && !started) {
		again = skb_copy(rskb, GFP_KERNEL);
		if (again)
			skb_trim(again, reconf_len);
	}
out:
	dev_kfree_skb(skb);
	/* the templates of the partner links must reach the firmware before
	 * the TBTT that takes this one, so the template that starts a removal
	 * goes up without a wait for its answer
	 */
	ret = mt7996_mcu_beacon_send(dev, link, rskb, crit_update,
				     crit_update ? offs.crit_update_id : 0,
				     !started, &ambiguous);
	if (ret == -EBUSY && again) {
		dev_warn(dev->mt76.dev,
			 "beacon template of link %u lost, sending it without removal entries\n",
			 link_conf->link_id);
		ret = mt7996_mcu_beacon_send(dev, link, again, crit_update,
					     crit_update ? offs.crit_update_id : 0,
					     true, &ambiguous);
		again = NULL;
		reconf.links = 0;
	}
	dev_kfree_skb(again);
	link->csa_bcn_uploaded = !ret && enabled && link_conf->csa_active;
	if (!ret)
		link->bcn_cntdwn = cntdwn;
	if (!ret && enabled != link->bcn_enabled) {
		link->bcn_enabled = enabled;
		mt7996_mcu_beacon_inband_discov(dev, link_conf, link);
	}
	if (ret && started &&
	    mt7996_mcu_beacon_reconf_retry(dev, vif, link_conf->link_id,
					   started))
		started = 0;
	if (started)
		mt7996_mcu_beacon_reconf_arm(dev, vif, started, &reconf,
					     upload_tsf);
	if (!ret && reconf.links)
		mt7996_mcu_beacon_reconf_set(dev, vif, link_conf->link_id);
	if (!ret && !enabled)
		mt7996_mcu_beacon_reconf_stop(dev, vif, link_conf->link_id,
					      false);
	if (ret || !ambiguous)
		return ret;

	/* the template again starts the critical update in the firmware, whose
	 * next event then belongs to it
	 */
	return __mt7996_mcu_add_beacon(hw, vif, link_conf, link, enabled,
				       strip_links);
}

/* The template keeps the first count of a countdown, and the firmware
 * counts from an upload, so the end is that many beacon intervals after
 * the first upload. A link that an advertised TTLM stops later still
 * needs the end of a countdown that began before the stop.
 */
static void
mt7996_mcu_beacon_cntdwn_record(struct ieee80211_hw *hw,
				struct ieee80211_vif *vif,
				struct ieee80211_bss_conf *link_conf)
{
	struct mt7996_vif *mvif = (struct mt7996_vif *)vif->drv_priv;
	struct mt7996_dev *dev = mt7996_hw_dev(hw);
	unsigned int link_id = link_conf->link_id;
	struct ieee80211_mutable_offsets offs;
	struct sk_buff *skb;
	bool running;
	u64 end;
	u8 cnt;

	if (!ieee80211_vif_is_mld(vif))
		return;

	if (!link_conf->csa_active && !link_conf->color_change_active) {
		mt7996_mcu_mld_attlm_cntdwn_reset(dev, mvif, BIT(link_id));
		return;
	}

	spin_lock_bh(&dev->mt76.lock);
	running = mvif->attlm_cntdwn_links & BIT(link_id);
	spin_unlock_bh(&dev->mt76.lock);
	if (running)
		return;

	skb = ieee80211_beacon_get_template(hw, vif, &offs, link_id);
	if (!skb)
		return;

	if (!offs.cntdwn_counter_offs[0]) {
		dev_kfree_skb(skb);
		return;
	}

	cnt = skb->data[offs.cntdwn_counter_offs[0]];
	dev_kfree_skb(skb);
	end = ktime_get_boottime_ns() +
	      (u64)cnt * link_conf->beacon_int * 1024 * NSEC_PER_USEC;

	spin_lock_bh(&dev->mt76.lock);
	mvif->attlm_cntdwn_links |= BIT(link_id);
	mvif->attlm_cntdwn_end[link_id] = end;
	mt7996_mcu_mld_attlm_cntdwn_arm(mvif);
	spin_unlock_bh(&dev->mt76.lock);
}

int mt7996_mcu_add_beacon(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
			  struct ieee80211_bss_conf *link_conf, bool enabled)
{
	struct mt7996_vif *mvif = (struct mt7996_vif *)vif->drv_priv;
	struct mt7996_dev *dev = mt7996_hw_dev(hw);
	struct mt7996_vif_link *link = mt7996_vif_conf_link(dev, vif, link_conf);
	u16 stopped, attlm_stopped;
	struct mt76_phy *mphy;

	if (link_conf->nontransmitted)
		return 0;

	if (!link)
		return -EINVAL;

	if (enabled)
		mt7996_mcu_beacon_cntdwn_record(hw, vif, link_conf);

	/* a link whose TX the removal stopped, or is due to stop, stays off
	 * until it goes. The other links leave its Reduced Neighbor Report
	 * entries out, and its profile, since the firmware after a full reset
	 * holds no timer for it.
	 */
	spin_lock_bh(&dev->mt76.lock);
	stopped = mvif->reconf_done_links | mvif->reconf_stop_links;
	attlm_stopped = mvif->attlm_stop_links;
	spin_unlock_bh(&dev->mt76.lock);

	if (!enabled || ((stopped | attlm_stopped) & BIT(link_conf->link_id)))
		return __mt7996_mcu_add_beacon(hw, vif, link_conf, link, false, 0);

	/* a phy off channel stops its beacons, which drops their hold */
	mphy = mt76_vif_link_phy(&link->mt76);
	if ((!mphy || !mphy->offchannel) &&
	    mt7996_mcu_beacon_reconf_held(dev, vif, link_conf->link_id))
		return 0;

	return __mt7996_mcu_add_beacon(hw, vif, link_conf, link, true, stopped);
}

struct mt7996_inband_iter_data {
	struct mt7996_dev *dev;
	struct mt7996_phy *phy;
	struct ieee80211_bss_conf *link_conf;
	struct mt7996_vif_link *link;
};

static bool
mt7996_mcu_beacon_inband_attlm_off(struct mt7996_dev *dev,
				   struct ieee80211_bss_conf *link_conf)
{
	struct mt7996_vif *mvif = (struct mt7996_vif *)link_conf->vif->drv_priv;
	bool off;

	spin_lock_bh(&dev->mt76.lock);
	off = mvif->attlm_disc_off & BIT(link_conf->link_id);
	spin_unlock_bh(&dev->mt76.lock);

	return off;
}

static bool
mt7996_mcu_beacon_inband_on(struct mt7996_dev *dev,
			    struct ieee80211_bss_conf *link_conf,
			    struct mt7996_vif_link *link)
{
	return link->bcn_enabled &&
	       (link_conf->fils_discovery.max_interval ||
		link_conf->unsol_bcast_probe_resp_interval) &&
	       !mt7996_mcu_beacon_inband_attlm_off(dev, link_conf);
}

static void
mt7996_mcu_beacon_inband_iter(void *priv, u8 *mac, struct ieee80211_vif *vif)
{
	struct mt7996_inband_iter_data *data = priv;
	struct ieee80211_bss_conf *link_conf;
	unsigned int link_id;

	if (data->link || vif->type != NL80211_IFTYPE_AP)
		return;

	for_each_vif_active_link(vif, link_conf, link_id) {
		struct mt7996_vif_link *link;

		link = mt7996_vif_link(data->dev, vif, link_id);
		if (!link || link_conf->nontransmitted ||
		    mt7996_vif_link_phy(link) != data->phy ||
		    !mt7996_mcu_beacon_inband_on(data->dev, link_conf, link))
			continue;

		data->link_conf = link_conf;
		data->link = link;
		return;
	}
}

static void
mt7996_mcu_beacon_inband_next(struct mt7996_dev *dev, struct mt7996_phy *phy)
{
	struct mt7996_inband_iter_data data = {
		.dev = dev,
		.phy = phy,
	};

	if (phy->mt76->offchannel)
		return;

	ieee80211_iterate_active_interfaces_atomic(phy->mt76->hw,
						   IEEE80211_IFACE_ITER_NORMAL,
						   mt7996_mcu_beacon_inband_iter,
						   &data);
	if (data.link)
		mt7996_mcu_beacon_inband_discov(dev, data.link_conf, data.link);
}

/* The firmware keeps one in-band discovery frame per band and sends it at
 * each TBTT of the band, also while the Beacon of its BSS is off. A stop
 * through any BSS of the band ends the frame of the last upload.
 */
static int
mt7996_mcu_beacon_inband_stop(struct mt7996_dev *dev,
			      struct mt7996_vif_link *link)
{
	struct mt7996_phy *phy = mt7996_vif_link_phy(link);
	struct bss_inband_discovery_tlv *discov;
	struct sk_buff *rskb;
	struct tlv *tlv;
	int ret;

	if (!phy || phy->inband_discov_link != link)
		return 0;

	rskb = __mt7996_mcu_alloc_bss_req(&dev->mt76, &link->mt76,
					  MT7996_BSS_UPDATE_MAX_SIZE);
	if (IS_ERR(rskb))
		return PTR_ERR(rskb);

	tlv = mt7996_mcu_add_uni_tlv(rskb, UNI_BSS_INFO_OFFLOAD,
				     sizeof(*discov));
	discov = (struct bss_inband_discovery_tlv *)tlv;
	discov->wcid = cpu_to_le16(MT7996_WTBL_RESERVED);

	ret = mt76_mcu_skb_send_msg(&dev->mt76, rskb,
				    MCU_WMWA_UNI_CMD(BSS_INFO_UPDATE), true);
	if (ret)
		return ret;

	phy->inband_discov_link = NULL;
	mt7996_mcu_beacon_inband_next(dev, phy);

	return 0;
}

int mt7996_mcu_beacon_inband_discov(struct mt7996_dev *dev,
				    struct ieee80211_bss_conf *link_conf,
				    struct mt7996_vif_link *link)
{
#define OFFLOAD_TX_MODE_SU	BIT(0)
#define OFFLOAD_TX_MODE_MU	BIT(1)
	struct mt76_phy *mphy = mt76_vif_link_phy(&link->mt76);
	bool fils = link_conf->fils_discovery.max_interval;
	struct ieee80211_vif *vif = link_conf->vif;
	struct ieee80211_hw *hw = mt76_hw(dev);
	struct mt76_wcid *wcid = &dev->mt76.global_wcid;
	struct bss_inband_discovery_tlv *discov;
	struct ieee80211_tx_info *info;
	struct sk_buff *rskb, *skb = NULL;
	struct cfg80211_chan_def *chandef;
	enum nl80211_band band;
	struct tlv *tlv;
	u8 *buf, interval;
	u32 changed;
	int len, ret;

	if (!mphy)
		return -EINVAL;

	if (link_conf->nontransmitted)
		return 0;

	if (!mt7996_mcu_beacon_inband_on(dev, link_conf, link))
		return mt7996_mcu_beacon_inband_stop(dev, link);

	chandef = &mphy->chandef;
	band = chandef->chan->band;

	rskb = __mt7996_mcu_alloc_bss_req(&dev->mt76, &link->mt76,
					  MT7996_MAX_BSS_OFFLOAD_SIZE);
	if (IS_ERR(rskb))
		return PTR_ERR(rskb);

	if (fils) {
		changed = BSS_CHANGED_FILS_DISCOVERY;
		interval = link_conf->fils_discovery.max_interval;
		skb = ieee80211_get_fils_discovery_tmpl(hw, vif,
							link_conf->link_id);
	} else {
		changed = BSS_CHANGED_UNSOL_BCAST_PROBE_RESP;
		interval = link_conf->unsol_bcast_probe_resp_interval;
		skb = ieee80211_get_unsol_bcast_probe_resp_tmpl(hw, vif,
								link_conf->link_id);
	}

	if (!skb) {
		dev_kfree_skb(rskb);
		return -EINVAL;
	}

	if (skb->len > MT7996_MAX_INBAND_FRAME_SIZE) {
		dev_err(dev->mt76.dev, "inband discovery size limit exceed\n");
		dev_kfree_skb(rskb);
		dev_kfree_skb(skb);
		return -EINVAL;
	}

	info = IEEE80211_SKB_CB(skb);
	info->control.vif = vif;
	info->band = band;
	info->hw_queue |= FIELD_PREP(MT_TX_HW_QUEUE_PHY, mphy->band_idx);

	len = ALIGN(sizeof(*discov) + MT_TXD_SIZE + skb->len, 4);
	tlv = mt7996_mcu_add_uni_tlv(rskb, UNI_BSS_INFO_OFFLOAD, len);

	discov = (struct bss_inband_discovery_tlv *)tlv;
	discov->tx_mode = OFFLOAD_TX_MODE_SU;
	/* 0: UNSOL PROBE RESP, 1: FILS DISCOV */
	discov->tx_type = !!(changed & BSS_CHANGED_FILS_DISCOVERY);
	discov->tx_interval = interval;
	discov->prob_rsp_len = cpu_to_le16(MT_TXD_SIZE + skb->len);
	discov->enable = true;
	discov->wcid = cpu_to_le16(MT7996_WTBL_RESERVED);

	buf = (u8 *)tlv + sizeof(*discov);

	mt7996_mac_write_txwi(dev, (__le32 *)buf, skb, wcid, NULL, 0, 0,
			      changed, link_conf->link_id);

	memcpy(buf + MT_TXD_SIZE, skb->data, skb->len);

	dev_kfree_skb(skb);

	ret = mt76_mcu_skb_send_msg(&dev->mt76, rskb,
				    MCU_WMWA_UNI_CMD(BSS_INFO_UPDATE), true);
	if (!ret)
		mt7996_vif_link_phy(link)->inband_discov_link = link;

	return ret;
}

static int mt7996_driver_own(struct mt7996_dev *dev, u8 band)
{
	mt76_wr(dev, MT_TOP_LPCR_HOST_BAND(band), MT_TOP_LPCR_HOST_DRV_OWN);
	if (!mt76_poll_msec(dev, MT_TOP_LPCR_HOST_BAND(band),
			    MT_TOP_LPCR_HOST_FW_OWN_STAT, 0, 500)) {
		dev_err(dev->mt76.dev, "Timeout for driver own\n");
		return -EIO;
	}

	/* clear irq when the driver own success */
	mt76_wr(dev, MT_TOP_LPCR_HOST_BAND_IRQ_STAT(band),
		MT_TOP_LPCR_HOST_BAND_STAT);

	return 0;
}

static u32 mt7996_patch_sec_mode(u32 key_info)
{
	u32 sec = u32_get_bits(key_info, MT7996_PATCH_SEC), key = 0;

	if (key_info == GENMASK(31, 0) || sec == MT7996_SEC_MODE_PLAIN)
		return 0;

	if (sec == MT7996_SEC_MODE_AES)
		key = u32_get_bits(key_info, MT7996_PATCH_AES_KEY);
	else
		key = u32_get_bits(key_info, MT7996_PATCH_SCRAMBLE_KEY);

	return MT7996_SEC_ENCRYPT | MT7996_SEC_IV |
	       u32_encode_bits(key, MT7996_SEC_KEY_IDX);
}

static int mt7996_load_patch(struct mt7996_dev *dev)
{
	const struct mt7996_patch_hdr *hdr;
	const struct firmware *fw = NULL;
	int i, ret, sem;

	sem = mt76_connac_mcu_patch_sem_ctrl(&dev->mt76, 1);
	switch (sem) {
	case PATCH_IS_DL:
		return 0;
	case PATCH_NOT_DL_SEM_SUCCESS:
		break;
	default:
		dev_err(dev->mt76.dev, "Failed to get patch semaphore\n");
		return -EAGAIN;
	}

	ret = request_firmware(&fw, fw_name(dev, ROM_PATCH), dev->mt76.dev);
	if (ret)
		goto out;

	if (mt76_connac2_patch_check(fw)) {
		dev_err(dev->mt76.dev, "Invalid firmware\n");
		ret = -EINVAL;
		goto out;
	}

	hdr = (const struct mt7996_patch_hdr *)(fw->data);

	dev_info(dev->mt76.dev, "HW/SW Version: 0x%x, Build Time: %.16s\n",
		 be32_to_cpu(hdr->hw_sw_ver), hdr->build_date);

	for (i = 0; i < be32_to_cpu(hdr->desc.n_region); i++) {
		struct mt7996_patch_sec *sec;
		const u8 *dl;
		u32 len, addr, sec_key_idx, mode = DL_MODE_NEED_RSP;

		sec = (struct mt7996_patch_sec *)(fw->data + sizeof(*hdr) +
						  i * sizeof(*sec));
		if ((be32_to_cpu(sec->type) & PATCH_SEC_TYPE_MASK) !=
		    PATCH_SEC_TYPE_INFO) {
			ret = -EINVAL;
			goto out;
		}

		addr = be32_to_cpu(sec->info.addr);
		len = be32_to_cpu(sec->info.len);
		sec_key_idx = be32_to_cpu(sec->info.sec_key_idx);
		dl = fw->data + be32_to_cpu(sec->offs);

		mode |= mt7996_patch_sec_mode(sec_key_idx);

		ret = mt76_connac_mcu_init_download(&dev->mt76, addr, len,
						    mode);
		if (ret) {
			dev_err(dev->mt76.dev, "Download request failed\n");
			goto out;
		}

		ret = __mt76_mcu_send_firmware(&dev->mt76, MCU_CMD(FW_SCATTER),
					       dl, len, 4096);
		if (ret) {
			dev_err(dev->mt76.dev, "Failed to send patch\n");
			goto out;
		}
	}

	ret = mt76_connac_mcu_start_patch(&dev->mt76);
	if (ret)
		dev_err(dev->mt76.dev, "Failed to start patch\n");

out:
	sem = mt76_connac_mcu_patch_sem_ctrl(&dev->mt76, 0);
	switch (sem) {
	case PATCH_REL_SEM_SUCCESS:
		break;
	default:
		ret = -EAGAIN;
		dev_err(dev->mt76.dev, "Failed to release patch semaphore\n");
		break;
	}
	release_firmware(fw);

	return ret;
}

static int
mt7996_mcu_send_ram_firmware(struct mt7996_dev *dev,
			     const struct mt7996_fw_trailer *hdr,
			     const u8 *data, enum mt7996_ram_type type)
{
	int i, offset = 0;
	u32 override = 0, option = 0;

	for (i = 0; i < hdr->n_region; i++) {
		const struct mt7996_fw_region *region;
		int err;
		u32 len, addr, mode;

		region = (const struct mt7996_fw_region *)((const u8 *)hdr -
			 (hdr->n_region - i) * sizeof(*region));
		/* DSP and WA use same mode */
		mode = mt76_connac_mcu_gen_dl_mode(&dev->mt76,
						   region->feature_set,
						   type != MT7996_RAM_TYPE_WM);
		len = le32_to_cpu(region->len);
		addr = le32_to_cpu(region->addr);

		if (region->feature_set & FW_FEATURE_OVERRIDE_ADDR)
			override = addr;

		err = mt76_connac_mcu_init_download(&dev->mt76, addr, len,
						    mode);
		if (err) {
			dev_err(dev->mt76.dev, "Download request failed\n");
			return err;
		}

		err = __mt76_mcu_send_firmware(&dev->mt76, MCU_CMD(FW_SCATTER),
					       data + offset, len, 4096);
		if (err) {
			dev_err(dev->mt76.dev, "Failed to send firmware.\n");
			return err;
		}

		offset += len;
	}

	if (override)
		option |= FW_START_OVERRIDE;

	if (type == MT7996_RAM_TYPE_WA)
		option |= FW_START_WORKING_PDA_CR4;
	else if (type == MT7996_RAM_TYPE_DSP)
		option |= FW_START_WORKING_PDA_DSP;

	return mt76_connac_mcu_start_firmware(&dev->mt76, override, option);
}

static int __mt7996_load_ram(struct mt7996_dev *dev, const char *fw_type,
			     const char *fw_file, enum mt7996_ram_type ram_type)
{
	const struct mt7996_fw_trailer *hdr;
	const struct firmware *fw;
	int ret;

	ret = request_firmware(&fw, fw_file, dev->mt76.dev);
	if (ret)
		return ret;

	if (mt76_connac2_fw_check(fw, NULL)) {
		dev_err(dev->mt76.dev, "Invalid firmware\n");
		ret = -EINVAL;
		goto out;
	}

	hdr = (const void *)(fw->data + fw->size - sizeof(*hdr));
	dev_info(dev->mt76.dev, "%s Firmware Version: %.10s, Build Time: %.15s\n",
		 fw_type, hdr->fw_ver, hdr->build_date);

	ret = mt7996_mcu_send_ram_firmware(dev, hdr, fw->data, ram_type);
	if (ret) {
		dev_err(dev->mt76.dev, "Failed to start %s firmware\n", fw_type);
		goto out;
	}

	snprintf(dev->mt76.hw->wiphy->fw_version,
		 sizeof(dev->mt76.hw->wiphy->fw_version),
		 "%.10s-%.15s", hdr->fw_ver, hdr->build_date);

out:
	release_firmware(fw);

	return ret;
}

static int mt7996_load_ram(struct mt7996_dev *dev)
{
	int ret;

	ret = __mt7996_load_ram(dev, "WM", fw_name(dev, FIRMWARE_WM),
				MT7996_RAM_TYPE_WM);
	if (ret)
		return ret;

	if (!mt7996_has_wa(dev))
		return 0;

	ret = __mt7996_load_ram(dev, "DSP", fw_name(dev, FIRMWARE_DSP),
				MT7996_RAM_TYPE_DSP);
	if (ret)
		return ret;

	return __mt7996_load_ram(dev, "WA", fw_name(dev, FIRMWARE_WA),
				 MT7996_RAM_TYPE_WA);
}

static int
mt7996_firmware_state(struct mt7996_dev *dev, u8 fw_state)
{
	u32 state = FIELD_PREP(MT_TOP_MISC_FW_STATE, fw_state);

	if (!mt76_poll_msec(dev, MT_TOP_MISC, MT_TOP_MISC_FW_STATE,
			    state, 1000)) {
		dev_err(dev->mt76.dev, "Timeout for initializing firmware\n");
		return -EIO;
	}
	return 0;
}

static int
mt7996_mcu_restart(struct mt76_dev *dev)
{
	struct {
		u8 __rsv1[4];

		__le16 tag;
		__le16 len;
		u8 power_mode;
		u8 __rsv2[3];
	} __packed req = {
		.tag = cpu_to_le16(UNI_POWER_OFF),
		.len = cpu_to_le16(sizeof(req) - 4),
		.power_mode = 1,
	};

	return mt76_mcu_send_msg(dev, MCU_WM_UNI_CMD(POWER_CTRL), &req,
				 sizeof(req), false);
}

static int mt7996_load_firmware(struct mt7996_dev *dev)
{
	u8 fw_state;
	int ret;

	/* make sure fw is download state */
	if (mt7996_firmware_state(dev, FW_STATE_FW_DOWNLOAD)) {
		/* restart firmware once */
		mt7996_mcu_restart(&dev->mt76);
		ret = mt7996_firmware_state(dev, FW_STATE_FW_DOWNLOAD);
		if (ret) {
			dev_err(dev->mt76.dev,
				"Firmware is not ready for download\n");
			return ret;
		}
	}

	ret = mt7996_load_patch(dev);
	if (ret)
		return ret;

	ret = mt7996_load_ram(dev);
	if (ret)
		return ret;

	fw_state = mt7996_has_wa(dev) ? FW_STATE_RDY : FW_STATE_NORMAL_TRX;
	ret = mt7996_firmware_state(dev, fw_state);
	if (ret)
		return ret;

	mt76_queue_tx_cleanup(dev, dev->mt76.q_mcu[MT_MCUQ_FWDL], false);

	dev_dbg(dev->mt76.dev, "Firmware init done\n");

	return 0;
}

int mt7996_mcu_fw_log_2_host(struct mt7996_dev *dev, u8 type, u8 ctrl)
{
	struct {
		u8 _rsv[4];

		__le16 tag;
		__le16 len;
		u8 ctrl;
		u8 interval;
		u8 _rsv2[2];
	} __packed data = {
		.tag = cpu_to_le16(UNI_WSYS_CONFIG_FW_LOG_CTRL),
		.len = cpu_to_le16(sizeof(data) - 4),
		.ctrl = ctrl,
	};

	if (type == MCU_FW_LOG_WA)
		return mt76_mcu_send_msg(&dev->mt76, MCU_WA_UNI_CMD(WSYS_CONFIG),
					 &data, sizeof(data), true);

	return mt76_mcu_send_msg(&dev->mt76, MCU_WM_UNI_CMD(WSYS_CONFIG), &data,
				 sizeof(data), true);
}

int mt7996_mcu_fw_dbg_ctrl(struct mt7996_dev *dev, u32 module, u8 level)
{
	struct {
		u8 _rsv[4];

		__le16 tag;
		__le16 len;
		__le32 module_idx;
		u8 level;
		u8 _rsv2[3];
	} data = {
		.tag = cpu_to_le16(UNI_WSYS_CONFIG_FW_DBG_CTRL),
		.len = cpu_to_le16(sizeof(data) - 4),
		.module_idx = cpu_to_le32(module),
		.level = level,
	};

	return mt76_mcu_send_msg(&dev->mt76, MCU_WM_UNI_CMD(WSYS_CONFIG), &data,
				 sizeof(data), false);
}

int mt7996_mcu_fw_debug_set(struct mt7996_dev *dev, u8 type, bool load)
{
	enum {
		DEBUG_TXCMD = 62,
		DEBUG_CMD_RPT_TX,
		DEBUG_CMD_RPT_TRIG,
		DEBUG_SPL,
		DEBUG_RPT_RX,
		DEBUG_RPT_RA = 68,
	} debug;
	bool tx, rx, en;
	u8 ctrl;
	int ret;

	if (type == MCU_FW_LOG_WA) {
		ret = mt7996_mcu_fw_log_2_host(dev, MCU_FW_LOG_WA,
					       dev->fw_debug_wa);
		if (ret || (load && !dev->fw_debug_wa))
			return ret;

		return mt7996_mcu_wa_cmd(dev, MCU_WA_PARAM_CMD(SET),
					 MCU_WA_PARAM_PDMA_RX,
					 !!dev->fw_debug_wa, 0);
	}

	if (dev->fw_debug_bin)
		ctrl = MCU_FW_LOG_RELAY;
	else
		ctrl = dev->fw_debug_wm;

	tx = dev->fw_debug_wm || (dev->fw_debug_bin & BIT(1));
	rx = dev->fw_debug_wm || (dev->fw_debug_bin & BIT(2));
	en = dev->fw_debug_wm || (dev->fw_debug_bin & BIT(0));

	ret = mt7996_mcu_fw_log_2_host(dev, MCU_FW_LOG_WM, ctrl);
	if (ret || (load && !en))
		return ret;

	for (debug = DEBUG_TXCMD; debug <= DEBUG_RPT_RA; debug++) {
		u8 level;

		if (debug == 67)
			continue;

		if (debug == DEBUG_RPT_RX)
			level = en && rx;
		else
			level = en && tx;

		ret = mt7996_mcu_fw_dbg_ctrl(dev, debug, level);
		if (ret)
			return ret;
	}

	return 0;
}

static int mt7996_mcu_set_mwds(struct mt7996_dev *dev, bool enabled)
{
	struct {
		u8 enable;
		u8 _rsv[3];
	} __packed req = {
		.enable = enabled
	};

	return mt76_mcu_send_msg(&dev->mt76, MCU_WA_EXT_CMD(MWDS_SUPPORT), &req,
				 sizeof(req), false);
}

static void mt7996_add_rx_airtime_tlv(struct sk_buff *skb, u8 band_idx)
{
	struct vow_rx_airtime *req;
	struct tlv *tlv;

	tlv = mt7996_mcu_add_uni_tlv(skb, UNI_VOW_RX_AT_AIRTIME_CLR_EN, sizeof(*req));
	req = (struct vow_rx_airtime *)tlv;
	req->enable = true;
	req->band = band_idx;

	tlv = mt7996_mcu_add_uni_tlv(skb, UNI_VOW_RX_AT_AIRTIME_EN, sizeof(*req));
	req = (struct vow_rx_airtime *)tlv;
	req->enable = true;
	req->band = band_idx;
}

static int
mt7996_mcu_init_rx_airtime(struct mt7996_dev *dev)
{
	struct uni_header hdr = {};
	struct sk_buff *skb;
	int len, num, i;

	num = 2 + 2 * (mt7996_band_valid(dev, MT_BAND1) +
		       mt7996_band_valid(dev, MT_BAND2));
	len = sizeof(hdr) + num * sizeof(struct vow_rx_airtime);
	skb = mt76_mcu_msg_alloc(&dev->mt76, NULL, len);
	if (!skb)
		return -ENOMEM;

	skb_put_data(skb, &hdr, sizeof(hdr));

	for (i = 0; i < __MT_MAX_BAND; i++) {
		if (mt7996_band_valid(dev, i))
			mt7996_add_rx_airtime_tlv(skb, i);
	}

	return mt76_mcu_skb_send_msg(&dev->mt76, skb,
				     MCU_WM_UNI_CMD(VOW), true);
}

int mt7996_mcu_init_firmware(struct mt7996_dev *dev)
{
	int ret;

	/* force firmware operation mode into normal state,
	 * which should be set before firmware download stage.
	 */
	mt76_wr(dev, MT_SWDEF_MODE, MT_SWDEF_NORMAL_MODE);

	ret = mt7996_driver_own(dev, 0);
	if (ret)
		return ret;
	/* set driver own for band1 when two hif exist */
	if (dev->hif2) {
		ret = mt7996_driver_own(dev, 1);
		if (ret)
			return ret;
	}

	ret = mt7996_load_firmware(dev);
	if (ret)
		return ret;

	set_bit(MT76_STATE_MCU_RUNNING, &dev->mphy.state);
	ret = mt7996_mcu_fw_debug_set(dev, MCU_FW_LOG_WM, true);
	if (ret)
		return ret;

	if (mt7996_has_wa(dev)) {
		ret = mt7996_mcu_fw_debug_set(dev, MCU_FW_LOG_WA, true);
		if (ret)
			return ret;

		ret = mt7996_mcu_set_mwds(dev, 1);
		if (ret)
			return ret;
	}

	ret = mt7996_mcu_init_rx_airtime(dev);
	if (ret)
		return ret;

	return mt7996_mcu_wa_cmd(dev, MCU_WA_PARAM_CMD(SET),
				 MCU_WA_PARAM_RED, 0, 0);
}

int mt7996_mcu_init(struct mt7996_dev *dev)
{
	static const struct mt76_mcu_ops mt7996_mcu_ops = {
		.headroom = sizeof(struct mt76_connac2_mcu_txd), /* reuse */
		.mcu_skb_send_msg = mt7996_mcu_send_message,
		.mcu_parse_response = mt7996_mcu_parse_response,
	};

	dev->mt76.mcu_ops = &mt7996_mcu_ops;

	return mt7996_mcu_init_firmware(dev);
}

void mt7996_mcu_exit(struct mt7996_dev *dev)
{
	mt7996_mcu_restart(&dev->mt76);
	if (mt7996_firmware_state(dev, FW_STATE_FW_DOWNLOAD)) {
		dev_err(dev->mt76.dev, "Failed to exit mcu\n");
		goto out;
	}

	mt76_wr(dev, MT_TOP_LPCR_HOST_BAND(0), MT_TOP_LPCR_HOST_FW_OWN);
	if (dev->hif2)
		mt76_wr(dev, MT_TOP_LPCR_HOST_BAND(1),
			MT_TOP_LPCR_HOST_FW_OWN);
out:
	skb_queue_purge(&dev->mt76.mcu.res_q);
}

int mt7996_mcu_set_hdr_trans(struct mt7996_dev *dev, bool hdr_trans)
{
	struct {
		u8 __rsv[4];
	} __packed hdr = {};
	struct hdr_trans_blacklist *req_blacklist;
	struct hdr_trans_en *req_en;
	struct sk_buff *skb;
	struct tlv *tlv;
	int len = MT7996_HDR_TRANS_MAX_SIZE + sizeof(hdr);

	skb = mt76_mcu_msg_alloc(&dev->mt76, NULL, len);
	if (!skb)
		return -ENOMEM;

	skb_put_data(skb, &hdr, sizeof(hdr));

	tlv = mt7996_mcu_add_uni_tlv(skb, UNI_HDR_TRANS_EN, sizeof(*req_en));
	req_en = (struct hdr_trans_en *)tlv;
	req_en->enable = hdr_trans;

	tlv = mt7996_mcu_add_uni_tlv(skb, UNI_HDR_TRANS_VLAN,
				     sizeof(struct hdr_trans_vlan));

	if (hdr_trans) {
		tlv = mt7996_mcu_add_uni_tlv(skb, UNI_HDR_TRANS_BLACKLIST,
					     sizeof(*req_blacklist));
		req_blacklist = (struct hdr_trans_blacklist *)tlv;
		req_blacklist->enable = 1;
		req_blacklist->type = cpu_to_le16(ETH_P_PAE);
	}

	return mt76_mcu_skb_send_msg(&dev->mt76, skb,
				     MCU_WM_UNI_CMD(RX_HDR_TRANS), true);
}

int mt7996_mcu_set_tx(struct mt7996_dev *dev, struct ieee80211_vif *vif,
		      struct ieee80211_bss_conf *link_conf)
{
#define MCU_EDCA_AC_PARAM	0
#define WMM_AIFS_SET		BIT(0)
#define WMM_CW_MIN_SET		BIT(1)
#define WMM_CW_MAX_SET		BIT(2)
#define WMM_TXOP_SET		BIT(3)
#define WMM_PARAM_SET		(WMM_AIFS_SET | WMM_CW_MIN_SET | \
				 WMM_CW_MAX_SET | WMM_TXOP_SET)
	struct mt7996_vif_link *link = mt7996_vif_conf_link(dev, vif, link_conf);
	struct mt7996_vif *mvif = (struct mt7996_vif *)vif->drv_priv;
	unsigned int link_id = link_conf->link_id;
	struct mt7996_vif_link_info *link_info = &mvif->link_info[link_id];
	struct {
		u8 bss_idx;
		u8 __rsv[3];
	} __packed hdr = {
		.bss_idx = link->mt76.idx,
	};
	struct sk_buff *skb;
	int len = sizeof(hdr) + IEEE80211_NUM_ACS * sizeof(struct edca);
	int ac;

	skb = mt76_mcu_msg_alloc(&dev->mt76, NULL, len);
	if (!skb)
		return -ENOMEM;

	skb_put_data(skb, &hdr, sizeof(hdr));

	for (ac = 0; ac < IEEE80211_NUM_ACS; ac++) {
		struct ieee80211_tx_queue_params *q = &link_info->queue_params[ac];
		struct edca *e;
		struct tlv *tlv;

		tlv = mt7996_mcu_add_uni_tlv(skb, MCU_EDCA_AC_PARAM, sizeof(*e));

		e = (struct edca *)tlv;
		e->set = WMM_PARAM_SET;
		e->queue = ac;
		e->aifs = q->aifs;
		e->txop = cpu_to_le16(q->txop);

		if (q->cw_min)
			e->cw_min = fls(q->cw_min);
		else
			e->cw_min = 5;

		if (q->cw_max)
			e->cw_max = fls(q->cw_max);
		else
			e->cw_max = 10;
	}

	return mt76_mcu_skb_send_msg(&dev->mt76, skb,
				     MCU_WM_UNI_CMD(EDCA_UPDATE), true);
}

int mt7996_mcu_set_fcc5_lpn(struct mt7996_dev *dev, int val)
{
	struct {
		u8 _rsv[4];

		__le16 tag;
		__le16 len;

		__le32 ctrl;
		__le16 min_lpn;
		u8 rsv[2];
	} __packed req = {
		.tag = cpu_to_le16(UNI_RDD_CTRL_SET_TH),
		.len = cpu_to_le16(sizeof(req) - 4),

		.ctrl = cpu_to_le32(0x1),
		.min_lpn = cpu_to_le16(val),
	};

	return mt76_mcu_send_msg(&dev->mt76, MCU_WM_UNI_CMD(RDD_CTRL),
				 &req, sizeof(req), true);
}

int mt7996_mcu_set_pulse_th(struct mt7996_dev *dev,
			    const struct mt7996_dfs_pulse *pulse)
{
	struct {
		u8 _rsv[4];

		__le16 tag;
		__le16 len;

		__le32 ctrl;

		__le32 max_width;		/* us */
		__le32 max_pwr;			/* dbm */
		__le32 min_pwr;			/* dbm */
		__le32 min_stgr_pri;		/* us */
		__le32 max_stgr_pri;		/* us */
		__le32 min_cr_pri;		/* us */
		__le32 max_cr_pri;		/* us */
	} __packed req = {
		.tag = cpu_to_le16(UNI_RDD_CTRL_SET_TH),
		.len = cpu_to_le16(sizeof(req) - 4),

		.ctrl = cpu_to_le32(0x3),

#define __req_field(field) .field = cpu_to_le32(pulse->field)
		__req_field(max_width),
		__req_field(max_pwr),
		__req_field(min_pwr),
		__req_field(min_stgr_pri),
		__req_field(max_stgr_pri),
		__req_field(min_cr_pri),
		__req_field(max_cr_pri),
#undef __req_field
	};

	return mt76_mcu_send_msg(&dev->mt76, MCU_WM_UNI_CMD(RDD_CTRL),
				 &req, sizeof(req), true);
}

int mt7996_mcu_set_radar_th(struct mt7996_dev *dev, int index,
			    const struct mt7996_dfs_pattern *pattern)
{
	struct {
		u8 _rsv[4];

		__le16 tag;
		__le16 len;

		__le32 ctrl;
		__le16 radar_type;

		u8 enb;
		u8 stgr;
		u8 min_crpn;
		u8 max_crpn;
		u8 min_crpr;
		u8 min_pw;
		__le32 min_pri;
		__le32 max_pri;
		u8 max_pw;
		u8 min_crbn;
		u8 max_crbn;
		u8 min_stgpn;
		u8 max_stgpn;
		u8 min_stgpr;
		u8 rsv[2];
		__le32 min_stgpr_diff;
	} __packed req = {
		.tag = cpu_to_le16(UNI_RDD_CTRL_SET_TH),
		.len = cpu_to_le16(sizeof(req) - 4),

		.ctrl = cpu_to_le32(0x2),
		.radar_type = cpu_to_le16(index),

#define __req_field_u8(field) .field = pattern->field
#define __req_field_u32(field) .field = cpu_to_le32(pattern->field)
		__req_field_u8(enb),
		__req_field_u8(stgr),
		__req_field_u8(min_crpn),
		__req_field_u8(max_crpn),
		__req_field_u8(min_crpr),
		__req_field_u8(min_pw),
		__req_field_u32(min_pri),
		__req_field_u32(max_pri),
		__req_field_u8(max_pw),
		__req_field_u8(min_crbn),
		__req_field_u8(max_crbn),
		__req_field_u8(min_stgpn),
		__req_field_u8(max_stgpn),
		__req_field_u8(min_stgpr),
		__req_field_u32(min_stgpr_diff),
#undef __req_field_u8
#undef __req_field_u32
	};

	return mt76_mcu_send_msg(&dev->mt76, MCU_WM_UNI_CMD(RDD_CTRL),
				 &req, sizeof(req), true);
}

static int
mt7996_mcu_background_chain_ctrl(struct mt7996_phy *phy,
				 struct cfg80211_chan_def *chandef,
				 int cmd)
{
	struct mt7996_dev *dev = phy->dev;
	struct mt76_phy *mphy = phy->mt76;
	struct ieee80211_channel *chan = mphy->chandef.chan;
	int freq = mphy->chandef.center_freq1;
	struct mt7996_mcu_background_chain_ctrl req = {
		.tag = cpu_to_le16(0),
		.len = cpu_to_le16(sizeof(req) - 4),
		.monitor_scan_type = 2, /* simple rx */
	};

	if (!chandef && cmd != CH_SWITCH_BACKGROUND_SCAN_STOP)
		return -EINVAL;

	if (!cfg80211_chandef_valid(&mphy->chandef))
		return -EINVAL;

	switch (cmd) {
	case CH_SWITCH_BACKGROUND_SCAN_START: {
		req.chan = chan->hw_value;
		req.central_chan = ieee80211_frequency_to_channel(freq);
		req.bw = mt76_connac_chan_bw(&mphy->chandef);
		req.monitor_chan = chandef->chan->hw_value;
		req.monitor_central_chan =
			ieee80211_frequency_to_channel(chandef->center_freq1);
		req.monitor_bw = mt76_connac_chan_bw(chandef);
		req.band_idx = phy->mt76->band_idx;
		req.scan_mode = 1;
		break;
	}
	case CH_SWITCH_BACKGROUND_SCAN_RUNNING:
		req.monitor_chan = chandef->chan->hw_value;
		req.monitor_central_chan =
			ieee80211_frequency_to_channel(chandef->center_freq1);
		req.band_idx = phy->mt76->band_idx;
		req.scan_mode = 2;
		break;
	case CH_SWITCH_BACKGROUND_SCAN_STOP:
		req.chan = chan->hw_value;
		req.central_chan = ieee80211_frequency_to_channel(freq);
		req.bw = mt76_connac_chan_bw(&mphy->chandef);
		req.tx_stream = hweight8(mphy->antenna_mask);
		req.rx_stream = mphy->antenna_mask;
		break;
	default:
		return -EINVAL;
	}
	req.band = chandef ? chandef->chan->band == NL80211_BAND_5GHZ : 1;

	return mt76_mcu_send_msg(&dev->mt76, MCU_WM_UNI_CMD(OFFCH_SCAN_CTRL),
				 &req, sizeof(req), false);
}

int mt7996_mcu_rdd_background_enable(struct mt7996_phy *phy,
				     struct cfg80211_chan_def *chandef)
{
	struct mt7996_dev *dev = phy->dev;
	int err, region, rdd_idx = mt7996_get_rdd_idx(phy, true);

	if (rdd_idx < 0)
		return -EINVAL;

	if (!chandef) { /* disable offchain */
		err = mt7996_mcu_rdd_cmd(dev, RDD_STOP, rdd_idx, 0);
		if (err)
			return err;

		return mt7996_mcu_background_chain_ctrl(phy, NULL,
				CH_SWITCH_BACKGROUND_SCAN_STOP);
	}

	err = mt7996_mcu_background_chain_ctrl(phy, chandef,
					       CH_SWITCH_BACKGROUND_SCAN_START);
	if (err)
		return err;

	switch (dev->mt76.region) {
	case NL80211_DFS_ETSI:
		region = 0;
		break;
	case NL80211_DFS_JP:
		region = 2;
		break;
	case NL80211_DFS_FCC:
	default:
		region = 1;
		break;
	}

	return mt7996_mcu_rdd_cmd(dev, RDD_START, rdd_idx, region);
}

int mt7996_mcu_set_chan_info(struct mt7996_phy *phy, u16 tag)
{
	static const u8 ch_band[] = {
		[NL80211_BAND_2GHZ] = 0,
		[NL80211_BAND_5GHZ] = 1,
		[NL80211_BAND_6GHZ] = 2,
	};
	struct mt7996_dev *dev = phy->dev;
	struct cfg80211_chan_def *chandef = &phy->mt76->chandef;
	int freq1 = chandef->center_freq1;
	u8 band_idx = phy->mt76->band_idx;
	bool scan;
	int ret;
	struct {
		/* fixed field */
		u8 __rsv[4];

		__le16 tag;
		__le16 len;
		u8 control_ch;
		u8 center_ch;
		u8 bw;
		u8 tx_path_num;
		u8 rx_path;	/* mask or num */
		u8 switch_reason;
		u8 band_idx;
		u8 center_ch2;	/* for 80+80 only */
		__le16 cac_case;
		u8 channel_band;
		u8 rsv0;
		__le32 outband_freq;
		u8 txpower_drop;
		u8 ap_bw;
		u8 ap_center_ch;
		u8 rsv1[53];
	} __packed req = {
		.tag = cpu_to_le16(tag),
		.len = cpu_to_le16(sizeof(req) - 4),
		.control_ch = chandef->chan->hw_value,
		.center_ch = ieee80211_frequency_to_channel(freq1),
		.bw = mt76_connac_chan_bw(chandef),
		.tx_path_num = hweight16(phy->mt76->chainmask),
		.rx_path = mt7996_rx_chainmask(phy) >> dev->chainshift[band_idx],
		.band_idx = band_idx,
		.channel_band = ch_band[chandef->chan->band],
	};

	if (phy->mt76->hw->conf.flags & IEEE80211_CONF_MONITOR &&
	    !phy->mt76->offchannel)
		req.switch_reason = CH_SWITCH_NORMAL;
	else if (phy->mt76->offchannel || !phy->mt76->chanctx)
		req.switch_reason = CH_SWITCH_SCAN_BYPASS_DPD;
	else if (!cfg80211_reg_can_beacon(phy->mt76->hw->wiphy, chandef,
					  NL80211_IFTYPE_AP))
		req.switch_reason = CH_SWITCH_DFS;
	else
		req.switch_reason = CH_SWITCH_NORMAL;

	if (tag == UNI_CHANNEL_SWITCH)
		req.rx_path = hweight8(req.rx_path);

	if (chandef->width == NL80211_CHAN_WIDTH_80P80) {
		int freq2 = chandef->center_freq2;

		req.center_ch2 = ieee80211_frequency_to_channel(freq2);
	}

	/* the firmware stops the station queues of the band at a scan switch
	 * and starts them again only at a normal or DFS switch
	 */
	scan = req.switch_reason == CH_SWITCH_SCAN_BYPASS_DPD;
	if (tag == UNI_CHANNEL_SWITCH && scan)
		phy->fw_offchannel = true;

	ret = mt76_mcu_send_msg(&dev->mt76, MCU_WMWA_UNI_CMD(CHANNEL_SWITCH),
				&req, sizeof(req), true);
	if (!ret && tag == UNI_CHANNEL_SWITCH && !scan)
		phy->fw_offchannel = false;

	return ret;
}

int mt7996_mcu_set_punct(struct mt7996_phy *phy)
{
	struct cfg80211_chan_def *chandef = &phy->mt76->chandef;
	struct mt7996_dev *dev = phy->dev;
	struct {
		u8 __rsv[4];

		__le16 tag;
		__le16 len;
		u8 mgmt_mode;
		u8 band_idx;
		u8 force_bitmap_ctrl;
		u8 auto_mode;
		__le16 bitmap;
		u8 csa_enable;
		u8 __rsv2;
	} __packed req = {
		.tag = cpu_to_le16(UNI_CMD_PP_EN_CTRL),
		.len = cpu_to_le16(sizeof(req) - 4),
		.mgmt_mode = 1,
		.band_idx = phy->mt76->band_idx,
		.force_bitmap_ctrl = 2,
		.bitmap = cpu_to_le16(chandef->punctured),
	};

	if (chandef->chan->band == NL80211_BAND_2GHZ)
		return 0;

	return mt76_mcu_send_msg(&dev->mt76, MCU_WM_UNI_CMD(PP),
				 &req, sizeof(req), false);
}

static int
mt7996_mcu_get_cal_free_data(struct mt7996_dev *dev)
{
#define MT_EE_7977BN_OFFSET	(0x1200 - 0x500)
	struct cal_free_data {
		u16 adie_offs;
		u16 eep_offs;
	};
	static const struct cal_free_data cal_7975[] = {
		{ 0x5cd, 0x451 }, { 0x5cf, 0x453 }, { 0x5d1, 0x455 },
		{ 0x5d3, 0x457 }, { 0x6c0, 0x44c }, { 0x6c1, 0x44d },
		{ 0x6c2, 0x44e }, { 0x6c3, 0x44f }, { 0x7a1, 0xba1 },
		{ 0x7a6, 0xba6 }, { 0x7a8, 0xba8 }, { 0x7aa, 0xbaa },
	};
	static const struct cal_free_data cal_7976[] = {
		{  0x4c, 0x44c }, {  0x4d, 0x44d }, {  0x4e, 0x44e },
		{  0x4f, 0x44f }, {  0x50, 0x450 }, {  0x51, 0x451 },
		{  0x53, 0x453 }, {  0x55, 0x455 }, {  0x57, 0x457 },
		{  0x59, 0x459 }, {  0x70, 0x470 }, {  0x71, 0x471 },
		{ 0x790, 0xb90 }, { 0x791, 0xb91 }, { 0x794, 0xb94 },
		{ 0x795, 0xb95 }, { 0x7a6, 0xba6 }, { 0x7a8, 0xba8 },
		{ 0x7aa, 0xbaa },
	};
	static const struct cal_free_data cal_7977[] = {
		{ 0x4c, 0x124c }, { 0x4d, 0x124d }, { 0x4e, 0x124e },
		{ 0x4f, 0x124f }, { 0x50, 0x1250 }, { 0x51, 0x1251 },
		{ 0x53, 0x1253 }, { 0x55, 0x1255 }, { 0x57, 0x1257 },
		{ 0x59, 0x1259 }, { 0x69, 0x1269 }, { 0x6a, 0x126a },
		{ 0x7a, 0x127a }, { 0x7b, 0x127b }, { 0x7c, 0x127c },
		{ 0x7d, 0x127d }, { 0x7e, 0x127e },
	};
	static const struct cal_free_data cal_7978[] = {
		{  0x91, 0xb91 }, {  0x95, 0xb95 }, { 0x100, 0x480 },
		{ 0x102, 0x482 }, { 0x104, 0x484 }, { 0x106, 0x486 },
		{ 0x107, 0x487 }, { 0x108, 0x488 }, { 0x109, 0x489 },
		{ 0x10a, 0x48a }, { 0x10b, 0x48b }, { 0x10c, 0x48c },
		{ 0x10e, 0x48e }, { 0x110, 0x490 },
	};
	static const struct cal_free_data cal_7979[] = {
		{ 0x4c, 0x124c }, { 0x4d, 0x124d }, { 0x4e, 0x124e },
		{ 0x4f, 0x124f }, { 0x50, 0x1250 }, { 0x51, 0x1251 },
		{ 0x53, 0x1253 }, { 0x55, 0x1255 }, { 0x57, 0x1257 },
		{ 0x59, 0x1259 }, { 0x69, 0x1269 }, { 0x6a, 0x126a },
		{ 0x7a, 0x127a }, { 0x7b, 0x127b }, { 0x7c, 0x127c },
		{ 0x7e, 0x127e }, { 0x80, 0x1280 },
	};
	const struct cal_free_data *cal_arr[__MT_MAX_BAND];
	u16 cal_arr_len[__MT_MAX_BAND] = {};
	u8 *eeprom = (u8 *)dev->mt76.eeprom.data;
	int band, i, ret;

#define CAL_ARR(_band, _adie) do {				\
		cal_arr[_band] = cal_##_adie;			\
		cal_arr_len[_band] = ARRAY_SIZE(cal_##_adie);	\
	} while (0)

	switch (mt76_chip(&dev->mt76)) {
	case MT7996_DEVICE_ID:
		/* adie 0 */
		if (dev->var.fem == MT7996_FEM_INT &&
		    dev->var.type != MT7996_VAR_TYPE_233)
			CAL_ARR(0, 7975);
		else
			CAL_ARR(0, 7976);

		/* adie 1 */
		if (dev->var.type == MT7996_VAR_TYPE_444)
			CAL_ARR(1, 7977);

		/* adie 2 */
		CAL_ARR(2, 7977);
		break;
	case MT7992_DEVICE_ID:
		/* adie 0 */
		if (dev->var.type == MT7992_VAR_TYPE_44 &&
		    dev->var.fem != MT7996_FEM_EXT)
			CAL_ARR(0, 7975);
		else if (dev->var.type == MT7992_VAR_TYPE_24)
			CAL_ARR(0, 7978);
		else
			CAL_ARR(0, 7976);

		/* adie 1 */
		if (dev->var.type == MT7992_VAR_TYPE_44 &&
		    dev->var.fem != MT7996_FEM_INT)
			CAL_ARR(1, 7977);
		else if (dev->var.type != MT7992_VAR_TYPE_23)
			CAL_ARR(1, 7979);
		break;
	case MT7990_DEVICE_ID:
		/* adie 0 */
		CAL_ARR(0, 7976);
		break;
	default:
		return -EINVAL;
	}

	for (band = 0; band < __MT_MAX_BAND; band++) {
		u8 buf[MT7996_EEPROM_BLOCK_SIZE];
		const struct cal_free_data *cal;
		u16 prev_block_idx = -1;
		u16 adie_base;

		if (!cal_arr_len[band])
			continue;

		if (band == MT_BAND0)
			adie_base = MT7996_EFUSE_BASE_OFFS_ADIE0;
		else if (band == MT_BAND1 && is_mt7992(&dev->mt76))
			adie_base = MT7992_EFUSE_BASE_OFFS_ADIE1;
		else if (band == MT_BAND1)
			adie_base = MT7996_EFUSE_BASE_OFFS_ADIE1;
		else
			adie_base = MT7996_EFUSE_BASE_OFFS_ADIE2;

		cal = cal_arr[band];
		for (i = 0; i < cal_arr_len[band]; i++) {
			u16 adie_offset = cal[i].adie_offs + adie_base;
			u16 eep_offset = cal[i].eep_offs;
			u16 block_idx = adie_offset / MT7996_EEPROM_BLOCK_SIZE;
			u16 offset = adie_offset % MT7996_EEPROM_BLOCK_SIZE;

			if (is_mt7996(&dev->mt76) && band == MT_BAND1 &&
			    dev->var.type == MT7996_VAR_TYPE_444)
				eep_offset -= MT_EE_7977BN_OFFSET;

			if (prev_block_idx != block_idx) {
				memset(buf, 0, sizeof(buf));
				ret = mt7996_mcu_get_eeprom(dev, adie_offset, buf,
							    MT7996_EEPROM_BLOCK_SIZE,
							    EEPROM_MODE_EFUSE);
				if (ret) {
					if (ret != -EINVAL)
						return ret;
					prev_block_idx = -1;
					continue;
				}
			}
			eeprom[eep_offset] = buf[offset];
			prev_block_idx = block_idx;
		}
	}

	return 0;
}

int mt7996_mcu_set_eeprom(struct mt7996_dev *dev)
{
#define MAX_PAGE_IDX_MASK	GENMASK(7, 5)
#define PAGE_IDX_MASK		GENMASK(4, 2)
#define PER_PAGE_SIZE		0x400
	struct mt7996_mcu_eeprom_update req = {
		.tag = cpu_to_le16(UNI_EFUSE_BUFFER_MODE),
		.buffer_mode = EE_MODE_BUFFER
	};
	u16 eeprom_size = MT7996_EEPROM_SIZE;
	u8 total = DIV_ROUND_UP(eeprom_size, PER_PAGE_SIZE);
	u8 *eep = (u8 *)dev->mt76.eeprom.data;
	int ret, eep_len, i;

	ret = mt7996_mcu_get_cal_free_data(dev);
	if (ret)
		return ret;

	for (i = 0; i < total; i++, eep += eep_len) {
		struct sk_buff *skb;
		int msg_len;

		if (i == total - 1 && !!(eeprom_size % PER_PAGE_SIZE))
			eep_len = eeprom_size % PER_PAGE_SIZE;
		else
			eep_len = PER_PAGE_SIZE;

		msg_len = sizeof(req) + eep_len;
		skb = mt76_mcu_msg_alloc(&dev->mt76, NULL, msg_len);
		if (!skb)
			return -ENOMEM;

		req.len = cpu_to_le16(msg_len - 4);
		req.format = FIELD_PREP(MAX_PAGE_IDX_MASK, total - 1) |
			     FIELD_PREP(PAGE_IDX_MASK, i) | EE_FORMAT_WHOLE;
		req.buf_len = cpu_to_le16(eep_len);

		skb_put_data(skb, &req, sizeof(req));
		skb_put_data(skb, eep, eep_len);

		ret = mt76_mcu_skb_send_msg(&dev->mt76, skb,
					    MCU_WM_UNI_CMD(EFUSE_CTRL), true);
		if (ret)
			return ret;
	}

	return 0;
}

int mt7996_mcu_get_eeprom(struct mt7996_dev *dev, u32 offset, u8 *buf, u32 buf_len,
			  enum mt7996_eeprom_mode mode)
{
	struct mt7996_mcu_eeprom_access req = {
		.info.len = cpu_to_le16(sizeof(req) - 4),
	};
	struct mt7996_mcu_eeprom_access_event *event;
	struct sk_buff *skb;
	size_t data_offset;
	int ret, cmd;
	u32 addr;

	switch (mode) {
	case EEPROM_MODE_EFUSE:
		addr = round_down(offset, MT7996_EEPROM_BLOCK_SIZE);
		cmd = MCU_WM_UNI_CMD_QUERY(EFUSE_CTRL);
		req.info.tag = cpu_to_le16(UNI_EFUSE_ACCESS);
		break;
	case EEPROM_MODE_EXT:
		addr = round_down(offset, MT7996_EXT_EEPROM_BLOCK_SIZE);
		cmd = MCU_WM_UNI_CMD_QUERY(EXT_EEPROM_CTRL);
		req.info.tag = cpu_to_le16(UNI_EXT_EEPROM_ACCESS);
		req.eeprom.ext_eeprom.data_len = cpu_to_le32(buf_len);
		break;
	default:
		return -EINVAL;
	}

	req.info.addr = cpu_to_le32(addr);
	ret = mt76_mcu_send_and_get_msg(&dev->mt76, cmd, &req, sizeof(req),
					true, &skb);
	if (ret)
		return ret;

	data_offset = offsetof(struct mt7996_mcu_eeprom_access_event, eeprom);
	if (skb->len < data_offset) {
		ret = -EIO;
		goto out;
	}

	event = (struct mt7996_mcu_eeprom_access_event *)skb->data;
	if (event->valid) {
		u32 block = mode == EEPROM_MODE_EXT ? MT7996_EXT_EEPROM_BLOCK_SIZE :
						      MT7996_EEPROM_BLOCK_SIZE;

		addr = le32_to_cpu(event->addr);

		if (!buf) {
			if (addr > dev->mt76.eeprom.size - block) {
				dev_kfree_skb(skb);
				return -EINVAL;
			}
			buf = (u8 *)dev->mt76.eeprom.data + addr;
		}

		switch (mode) {
		case EEPROM_MODE_EFUSE:
			if (!buf_len || buf_len > MT7996_EEPROM_BLOCK_SIZE)
				buf_len = MT7996_EEPROM_BLOCK_SIZE;
			if (skb->len - data_offset < buf_len) {
				ret = -EIO;
				break;
			}

			memcpy(buf, event->eeprom.efuse, buf_len);
			break;
		case EEPROM_MODE_EXT: {
			u32 ret_len;

			if (skb->len - data_offset < sizeof(__le32)) {
				ret = -EIO;
				break;
			}

			ret_len = le32_to_cpu(event->eeprom.ext_eeprom.data_len);
			data_offset += sizeof(__le32);
			if (ret_len > skb->len - data_offset) {
				ret = -EIO;
				break;
			}

			if (!buf_len || buf_len > MT7996_EXT_EEPROM_BLOCK_SIZE)
				buf_len = MT7996_EXT_EEPROM_BLOCK_SIZE;

			memcpy(buf, event->eeprom.ext_eeprom.data,
			       ret_len < buf_len ? ret_len : buf_len);
			break;
		}
		default:
			ret = -EINVAL;
			break;
		}
	} else {
		ret = -EINVAL;
	}

out:
	dev_kfree_skb(skb);

	return ret;
}

int mt7996_mcu_get_efuse_free_block(struct mt7996_dev *dev, u8 *block_num)
{
	struct {
		u8 _rsv[4];

		__le16 tag;
		__le16 len;
		u8 num;
		u8 version;
		u8 die_idx;
		u8 _rsv2;
	} __packed req = {
		.tag = cpu_to_le16(UNI_EFUSE_FREE_BLOCK),
		.len = cpu_to_le16(sizeof(req) - 4),
		.version = 2,
	};
	struct sk_buff *skb;
	int ret;

	ret = mt76_mcu_send_and_get_msg(&dev->mt76, MCU_WM_UNI_CMD_QUERY(EFUSE_CTRL), &req,
					sizeof(req), true, &skb);
	if (ret)
		return ret;
	if (skb->len < 8 + sizeof(*block_num)) {
		dev_kfree_skb(skb);
		return -EINVAL;
	}

	*block_num = *(u8 *)(skb->data + 8);
	dev_kfree_skb(skb);

	return 0;
}

int mt7996_mcu_get_chip_config(struct mt7996_dev *dev, u32 *cap)
{
#define UNI_EVENT_CHIP_CONFIG_EFUSE_VERSION	0x21
	struct {
		u8 _rsv[4];

		__le16 tag;
		__le16 len;
	} __packed req = {
		.tag = cpu_to_le16(UNI_CHIP_CONFIG_NIC_CAPA),
		.len = cpu_to_le16(sizeof(req) - 4),
	};
	struct sk_buff *skb;
	u8 *buf;
	int ret;

	ret = mt76_mcu_send_and_get_msg(&dev->mt76,
					MCU_WM_UNI_CMD_QUERY(CHIP_CONFIG), &req,
					sizeof(req), true, &skb);
	if (ret)
		return ret;

	/* fixed field */
	if (skb->len < 4) {
		dev_kfree_skb(skb);
		return -EINVAL;
	}
	skb_pull(skb, 4);

	buf = skb->data;
	while (buf - skb->data + sizeof(struct tlv) <= skb->len) {
		struct tlv *tlv = (struct tlv *)buf;
		u16 tlv_len = le16_to_cpu(tlv->len);

		if (tlv_len < sizeof(*tlv) ||
		    tlv_len > skb->len - (buf - skb->data))
			break;

		switch (le16_to_cpu(tlv->tag)) {
		case UNI_EVENT_CHIP_CONFIG_EFUSE_VERSION:
			if (tlv_len >= sizeof(*tlv) + sizeof(__le32))
				*cap = le32_to_cpu(*(__le32 *)(buf + sizeof(*tlv)));
			break;
		default:
			break;
		}

		buf += tlv_len;
	}

	dev_kfree_skb(skb);

	return 0;
}

int mt7996_mcu_get_chan_mib_info(struct mt7996_phy *phy, bool chan_switch)
{
	enum {
		IDX_TX_TIME,
		IDX_RX_TIME,
		IDX_OBSS_AIRTIME,
		IDX_NON_WIFI_TIME,
		IDX_NUM
	};
	struct {
		struct {
			u8 band;
			u8 __rsv[3];
		} hdr;
		struct {
			__le16 tag;
			__le16 len;
			__le32 offs;
		} data[IDX_NUM];
	} __packed req = {
		.hdr.band = phy->mt76->band_idx,
	};
	static const u32 offs[] = {
		[IDX_TX_TIME] = UNI_MIB_TX_TIME,
		[IDX_RX_TIME] = UNI_MIB_RX_TIME,
		[IDX_OBSS_AIRTIME] = UNI_MIB_OBSS_AIRTIME,
		[IDX_NON_WIFI_TIME] = UNI_MIB_NON_WIFI_TIME,
	};
	struct mt76_channel_state *state = phy->mt76->chan_state;
	struct mt76_channel_state *state_ts = &phy->state_ts;
	struct mt7996_dev *dev = phy->dev;
	struct mt7996_mcu_mib *res;
	struct sk_buff *skb;
	int i, ret;

	for (i = 0; i < IDX_NUM; i++) {
		req.data[i].tag = cpu_to_le16(UNI_CMD_MIB_DATA);
		req.data[i].len = cpu_to_le16(sizeof(req.data[i]));
		req.data[i].offs = cpu_to_le32(offs[i]);
	}

	ret = mt76_mcu_send_and_get_msg(&dev->mt76, MCU_WM_UNI_CMD_QUERY(GET_MIB_INFO),
					&req, sizeof(req), true, &skb);
	if (ret)
		return ret;
	if (skb->len < sizeof(req.hdr) + IDX_NUM * sizeof(*res)) {
		dev_kfree_skb(skb);
		return -EINVAL;
	}

	skb_pull(skb, sizeof(req.hdr));

	res = (struct mt7996_mcu_mib *)(skb->data);

	if (chan_switch)
		goto out;

#define __res_u64(s) le64_to_cpu(res[s].data)
	state->cc_tx += __res_u64(IDX_TX_TIME) - state_ts->cc_tx;
	state->cc_bss_rx += __res_u64(IDX_RX_TIME) - state_ts->cc_bss_rx;
	state->cc_rx += __res_u64(IDX_RX_TIME) +
			__res_u64(IDX_OBSS_AIRTIME) -
			state_ts->cc_rx;
	state->cc_busy += __res_u64(IDX_TX_TIME) +
			  __res_u64(IDX_RX_TIME) +
			  __res_u64(IDX_OBSS_AIRTIME) +
			  __res_u64(IDX_NON_WIFI_TIME) -
			  state_ts->cc_busy;
out:
	state_ts->cc_tx = __res_u64(IDX_TX_TIME);
	state_ts->cc_bss_rx = __res_u64(IDX_RX_TIME);
	state_ts->cc_rx = __res_u64(IDX_RX_TIME) + __res_u64(IDX_OBSS_AIRTIME);
	state_ts->cc_busy = __res_u64(IDX_TX_TIME) +
			    __res_u64(IDX_RX_TIME) +
			    __res_u64(IDX_OBSS_AIRTIME) +
			    __res_u64(IDX_NON_WIFI_TIME);
#undef __res_u64

	dev_kfree_skb(skb);

	return 0;
}

int mt7996_mcu_get_temperature(struct mt7996_phy *phy)
{
#define TEMPERATURE_QUERY 0
#define GET_TEMPERATURE 0
	struct {
		u8 _rsv[4];

		__le16 tag;
		__le16 len;

		u8 rsv1;
		u8 action;
		u8 band_idx;
		u8 rsv2;
	} req = {
		.tag = cpu_to_le16(TEMPERATURE_QUERY),
		.len = cpu_to_le16(sizeof(req) - 4),
		.action = GET_TEMPERATURE,
		.band_idx = phy->mt76->band_idx,
	};
	struct mt7996_mcu_thermal {
		u8 _rsv[4];

		__le16 tag;
		__le16 len;

		__le32 rsv;
		__le32 temperature;
	} __packed * res;
	struct sk_buff *skb;
	int ret;
	u32 temp;

	ret = mt76_mcu_send_and_get_msg(&phy->dev->mt76, MCU_WM_UNI_CMD(THERMAL),
					&req, sizeof(req), true, &skb);
	if (ret)
		return ret;
	if (skb->len < sizeof(*res)) {
		dev_kfree_skb(skb);
		return -EINVAL;
	}

	res = (void *)skb->data;
	temp = le32_to_cpu(res->temperature);
	dev_kfree_skb(skb);

	return temp;
}

int mt7996_mcu_set_thermal_throttling(struct mt7996_phy *phy, u8 state)
{
	struct {
		u8 _rsv[4];

		__le16 tag;
		__le16 len;

		struct mt7996_mcu_thermal_ctrl ctrl;
	} __packed req = {
		.tag = cpu_to_le16(UNI_CMD_THERMAL_PROTECT_DUTY_CONFIG),
		.len = cpu_to_le16(sizeof(req) - 4),
		.ctrl = {
			.band_idx = phy->mt76->band_idx,
		},
	};
	int level, ret;

	/* set duty cycle and level */
	for (level = 0; level < 4; level++) {
		req.ctrl.duty.duty_level = level;
		req.ctrl.duty.duty_cycle = state;
		state /= 2;

		ret = mt76_mcu_send_msg(&phy->dev->mt76, MCU_WM_UNI_CMD(THERMAL),
					&req, sizeof(req), false);
		if (ret)
			return ret;
	}

	return 0;
}

int mt7996_mcu_set_thermal_protect(struct mt7996_phy *phy, bool enable)
{
#define SUSTAIN_PERIOD		10
	struct {
		u8 _rsv[4];

		__le16 tag;
		__le16 len;

		struct mt7996_mcu_thermal_ctrl ctrl;
		struct mt7996_mcu_thermal_enable enable;
	} __packed req = {
		.len = cpu_to_le16(sizeof(req) - 4 - sizeof(req.enable)),
		.ctrl = {
			.band_idx = phy->mt76->band_idx,
			.type.protect_type = 1,
			.type.trigger_type = 1,
		},
	};
	int ret;

	req.tag = cpu_to_le16(UNI_CMD_THERMAL_PROTECT_DISABLE);

	ret = mt76_mcu_send_msg(&phy->dev->mt76, MCU_WM_UNI_CMD(THERMAL),
				&req, sizeof(req) - sizeof(req.enable), false);
	if (ret || !enable)
		return ret;

	/* set high-temperature trigger threshold */
	req.tag = cpu_to_le16(UNI_CMD_THERMAL_PROTECT_ENABLE);
	req.enable.restore_temp = cpu_to_le32(phy->throttle_temp[0]);
	req.enable.trigger_temp = cpu_to_le32(phy->throttle_temp[1]);
	req.enable.sustain_time = cpu_to_le16(SUSTAIN_PERIOD);

	req.len = cpu_to_le16(sizeof(req) - 4);

	return mt76_mcu_send_msg(&phy->dev->mt76, MCU_WM_UNI_CMD(THERMAL),
				 &req, sizeof(req), false);
}

int mt7996_mcu_set_ser(struct mt7996_dev *dev, u8 action, u8 val, u8 band)
{
	struct {
		u8 rsv[4];

		__le16 tag;
		__le16 len;

		union {
			struct {
				__le32 mask;
			} __packed set;

			struct {
				u8 method;
				u8 band;
				u8 rsv2[2];
			} __packed trigger;
		};
	} __packed req = {
		.tag = cpu_to_le16(action),
		.len = cpu_to_le16(sizeof(req) - 4),
	};

	switch (action) {
	case UNI_CMD_SER_SET:
		req.set.mask = cpu_to_le32(val);
		break;
	case UNI_CMD_SER_TRIGGER:
		req.trigger.method = val;
		req.trigger.band = band;
		break;
	default:
		return -EINVAL;
	}

	return mt76_mcu_send_msg(&dev->mt76, MCU_WM_UNI_CMD(SER),
				 &req, sizeof(req), false);
}

int mt7996_mcu_set_txbf(struct mt7996_dev *dev, u8 action)
{
#define MT7996_BF_MAX_SIZE	sizeof(union bf_tag_tlv)
#define BF_PROCESSING	4
	struct uni_header hdr;
	struct sk_buff *skb;
	struct tlv *tlv;
	int len = sizeof(hdr) + MT7996_BF_MAX_SIZE;

	memset(&hdr, 0, sizeof(hdr));

	skb = mt76_mcu_msg_alloc(&dev->mt76, NULL, len);
	if (!skb)
		return -ENOMEM;

	skb_put_data(skb, &hdr, sizeof(hdr));

	switch (action) {
	case BF_SOUNDING_ON: {
		struct bf_sounding_on *req_snd_on;

		tlv = mt7996_mcu_add_uni_tlv(skb, action, sizeof(*req_snd_on));
		req_snd_on = (struct bf_sounding_on *)tlv;
		req_snd_on->snd_mode = BF_PROCESSING;
		break;
	}
	case BF_HW_EN_UPDATE: {
		struct bf_hw_en_status_update *req_hw_en;

		tlv = mt7996_mcu_add_uni_tlv(skb, action, sizeof(*req_hw_en));
		req_hw_en = (struct bf_hw_en_status_update *)tlv;
		req_hw_en->ebf = true;
		req_hw_en->ibf = dev->ibf;
		break;
	}
	case BF_MOD_EN_CTRL: {
		struct bf_mod_en_ctrl *req_mod_en;

		tlv = mt7996_mcu_add_uni_tlv(skb, action, sizeof(*req_mod_en));
		req_mod_en = (struct bf_mod_en_ctrl *)tlv;
		req_mod_en->bf_num = mt7996_band_valid(dev, MT_BAND2) ? 3 : 2;
		req_mod_en->bf_bitmap = mt7996_band_valid(dev, MT_BAND2) ?
					GENMASK(2, 0) : GENMASK(1, 0);
		break;
	}
	default:
		return -EINVAL;
	}

	return mt76_mcu_skb_send_msg(&dev->mt76, skb, MCU_WM_UNI_CMD(BF), true);
}

static int
mt7996_mcu_enable_obss_spr(struct mt7996_phy *phy, u16 action, u8 val)
{
	struct mt7996_dev *dev = phy->dev;
	struct {
		u8 band_idx;
		u8 __rsv[3];

		__le16 tag;
		__le16 len;

		__le32 val;
	} __packed req = {
		.band_idx = phy->mt76->band_idx,
		.tag = cpu_to_le16(action),
		.len = cpu_to_le16(sizeof(req) - 4),
		.val = cpu_to_le32(val),
	};

	return mt76_mcu_send_msg(&dev->mt76, MCU_WM_UNI_CMD(SR),
				 &req, sizeof(req), true);
}

static int
mt7996_mcu_set_obss_spr_pd(struct mt7996_phy *phy,
			   struct ieee80211_he_obss_pd *he_obss_pd)
{
	struct mt7996_dev *dev = phy->dev;
	u8 max_th = 82, non_srg_max_th = 62;
	struct {
		u8 band_idx;
		u8 __rsv[3];

		__le16 tag;
		__le16 len;

		u8 pd_th_non_srg;
		u8 pd_th_srg;
		u8 period_offs;
		u8 rcpi_src;
		__le16 obss_pd_min;
		__le16 obss_pd_min_srg;
		u8 resp_txpwr_mode;
		u8 txpwr_restrict_mode;
		u8 txpwr_ref;
		u8 __rsv2[3];
	} __packed req = {
		.band_idx = phy->mt76->band_idx,
		.tag = cpu_to_le16(UNI_CMD_SR_SET_PARAM),
		.len = cpu_to_le16(sizeof(req) - 4),
		.obss_pd_min = cpu_to_le16(max_th),
		.obss_pd_min_srg = cpu_to_le16(max_th),
		.txpwr_restrict_mode = 2,
		.txpwr_ref = 21
	};
	int ret;

	/* disable firmware dynamical PD asjustment */
	ret = mt7996_mcu_enable_obss_spr(phy, UNI_CMD_SR_ENABLE_DPD, false);
	if (ret)
		return ret;

	if (he_obss_pd->sr_ctrl &
	    IEEE80211_HE_SPR_NON_SRG_OBSS_PD_SR_DISALLOWED)
		req.pd_th_non_srg = max_th;
	else if (he_obss_pd->sr_ctrl & IEEE80211_HE_SPR_NON_SRG_OFFSET_PRESENT)
		req.pd_th_non_srg  = max_th - he_obss_pd->non_srg_max_offset;
	else
		req.pd_th_non_srg  = non_srg_max_th;

	if (he_obss_pd->sr_ctrl & IEEE80211_HE_SPR_SRG_INFORMATION_PRESENT)
		req.pd_th_srg = max_th - he_obss_pd->max_offset;

	return mt76_mcu_send_msg(&dev->mt76, MCU_WM_UNI_CMD(SR),
				 &req, sizeof(req), true);
}

static int
mt7996_mcu_set_obss_spr_siga(struct mt7996_phy *phy,
			     struct mt7996_vif_link *link,
			     struct ieee80211_he_obss_pd *he_obss_pd)
{
	struct mt7996_dev *dev = phy->dev;
	u8 omac = link->mt76.omac_idx;
	struct {
		u8 band_idx;
		u8 __rsv[3];

		__le16 tag;
		__le16 len;

		u8 omac;
		u8 __rsv2[3];
		u8 flag[20];
	} __packed req = {
		.band_idx = phy->mt76->band_idx,
		.tag = cpu_to_le16(UNI_CMD_SR_SET_SIGA),
		.len = cpu_to_le16(sizeof(req) - 4),
		.omac = omac > HW_BSSID_MAX ? omac - 12 : omac,
	};
	int ret;

	if (he_obss_pd->sr_ctrl & IEEE80211_HE_SPR_HESIGA_SR_VAL15_ALLOWED)
		req.flag[req.omac] = 0xf;
	else
		return 0;

	/* switch to normal AP mode */
	ret = mt7996_mcu_enable_obss_spr(phy, UNI_CMD_SR_ENABLE_MODE, 0);
	if (ret)
		return ret;

	return mt76_mcu_send_msg(&dev->mt76, MCU_WM_UNI_CMD(SR),
				 &req, sizeof(req), true);
}

static int
mt7996_mcu_set_obss_spr_bitmap(struct mt7996_phy *phy,
			       struct ieee80211_he_obss_pd *he_obss_pd)
{
	struct mt7996_dev *dev = phy->dev;
	struct {
		u8 band_idx;
		u8 __rsv[3];

		__le16 tag;
		__le16 len;

		struct {
			__le32 color_l;
			__le32 color_h;
			__le32 bssid_l;
			__le32 bssid_h;
		} __packed spr_bitmap[__MT_MAX_BAND];
	} __packed req = {
		.band_idx = phy->mt76->band_idx,
		.tag = cpu_to_le16(UNI_CMD_SR_SET_SRG_BITMAP),
		.len = cpu_to_le16(sizeof(req) - 4),
	};
	u32 bitmap;

	memcpy(&bitmap, he_obss_pd->bss_color_bitmap, sizeof(bitmap));
	req.spr_bitmap[req.band_idx].color_l = cpu_to_le32(bitmap);

	memcpy(&bitmap, he_obss_pd->bss_color_bitmap + 4, sizeof(bitmap));
	req.spr_bitmap[req.band_idx].color_h = cpu_to_le32(bitmap);

	memcpy(&bitmap, he_obss_pd->partial_bssid_bitmap, sizeof(bitmap));
	req.spr_bitmap[req.band_idx].bssid_l = cpu_to_le32(bitmap);

	memcpy(&bitmap, he_obss_pd->partial_bssid_bitmap + 4, sizeof(bitmap));
	req.spr_bitmap[req.band_idx].bssid_h = cpu_to_le32(bitmap);

	return mt76_mcu_send_msg(&dev->mt76, MCU_WM_UNI_CMD(SR), &req,
				 sizeof(req), true);
}

int mt7996_mcu_add_obss_spr(struct mt7996_phy *phy,
			    struct mt7996_vif_link *link,
			    struct ieee80211_he_obss_pd *he_obss_pd)
{
	int ret;

	/* enable firmware scene detection algorithms */
	ret = mt7996_mcu_enable_obss_spr(phy, UNI_CMD_SR_ENABLE_SD,
					 sr_scene_detect);
	if (ret)
		return ret;

	/* firmware dynamically adjusts PD threshold so skip manual control */
	if (sr_scene_detect && !he_obss_pd->enable)
		return 0;

	/* enable spatial reuse */
	ret = mt7996_mcu_enable_obss_spr(phy, UNI_CMD_SR_ENABLE,
					 he_obss_pd->enable);
	if (ret)
		return ret;

	if (sr_scene_detect || !he_obss_pd->enable)
		return 0;

	ret = mt7996_mcu_enable_obss_spr(phy, UNI_CMD_SR_ENABLE_TX, true);
	if (ret)
		return ret;

	/* set SRG/non-SRG OBSS PD threshold */
	ret = mt7996_mcu_set_obss_spr_pd(phy, he_obss_pd);
	if (ret)
		return ret;

	/* Set SR prohibit */
	ret = mt7996_mcu_set_obss_spr_siga(phy, link, he_obss_pd);
	if (ret)
		return ret;

	/* set SRG BSS color/BSSID bitmap */
	return mt7996_mcu_set_obss_spr_bitmap(phy, he_obss_pd);
}

int mt7996_mcu_update_bss_color(struct mt7996_dev *dev,
				struct mt76_vif_link *mlink,
				struct cfg80211_he_bss_color *he_bss_color)
{
	int len = sizeof(struct bss_req_hdr) + sizeof(struct bss_color_tlv);
	struct bss_color_tlv *bss_color;
	struct sk_buff *skb;
	struct tlv *tlv;

	skb = __mt7996_mcu_alloc_bss_req(&dev->mt76, mlink, len);
	if (IS_ERR(skb))
		return PTR_ERR(skb);

	tlv = mt76_connac_mcu_add_tlv(skb, UNI_BSS_INFO_BSS_COLOR,
				      sizeof(*bss_color));
	bss_color = (struct bss_color_tlv *)tlv;
	bss_color->enable = he_bss_color->enabled;
	bss_color->color = he_bss_color->color;

	return mt76_mcu_skb_send_msg(&dev->mt76, skb,
				     MCU_WMWA_UNI_CMD(BSS_INFO_UPDATE), true);
}

#define TWT_AGRT_TRIGGER	BIT(0)
#define TWT_AGRT_ANNOUNCE	BIT(1)
#define TWT_AGRT_PROTECT	BIT(2)

int mt7996_mcu_twt_agrt_update(struct mt7996_dev *dev,
			       struct mt7996_vif_link *link,
			       struct mt7996_twt_flow *flow,
			       int cmd)
{
	struct {
		/* fixed field */
		u8 bss;
		u8 _rsv[3];

		__le16 tag;
		__le16 len;
		u8 tbl_idx;
		u8 cmd;
		u8 own_mac_idx;
		u8 flowid; /* 0xff for group id */
		__le16 peer_id; /* specify the peer_id (msb=0)
				 * or group_id (msb=1)
				 */
		u8 duration; /* 256 us */
		u8 bss_idx;
		__le64 start_tsf;
		__le16 mantissa;
		u8 exponent;
		u8 is_ap;
		u8 agrt_params;
		u8 __rsv2[23];
	} __packed req = {
		.tag = cpu_to_le16(UNI_CMD_TWT_ARGT_UPDATE),
		.len = cpu_to_le16(sizeof(req) - 4),
		.tbl_idx = flow->table_id,
		.cmd = cmd,
		.own_mac_idx = link->mt76.omac_idx,
		.flowid = flow->id,
		.peer_id = cpu_to_le16(flow->wcid),
		.duration = flow->duration,
		.bss = link->mt76.idx,
		.bss_idx = link->mt76.idx,
		.start_tsf = cpu_to_le64(flow->tsf),
		.mantissa = flow->mantissa,
		.exponent = flow->exp,
		.is_ap = true,
	};

	if (flow->protection)
		req.agrt_params |= TWT_AGRT_PROTECT;
	if (!flow->flowtype)
		req.agrt_params |= TWT_AGRT_ANNOUNCE;
	if (flow->trigger)
		req.agrt_params |= TWT_AGRT_TRIGGER;

	return mt76_mcu_send_msg(&dev->mt76, MCU_WM_UNI_CMD(TWT),
				 &req, sizeof(req), true);
}

int mt7996_mcu_set_bssid_mapping_addr(struct mt76_dev *dev, u8 band_idx)
{
	enum {
		BSSID_MAPPING_ADDR1,
		BSSID_MAPPING_ADDR2,
		BSSID_MAPPING_ADDR3,
	};
	struct {
		u8 band_idx;
		u8 _rsv1[3];

		__le16 tag;
		__le16 len;
		u8 addr;
		u8 _rsv2[3];
	} __packed req = {
		.band_idx = band_idx,
		.tag = cpu_to_le16(UNI_BAND_CONFIG_BSSID_MAPPING_ADDR),
		.len = cpu_to_le16(sizeof(req) - 4),
		.addr = BSSID_MAPPING_ADDR1,
	};

	return mt76_mcu_send_msg(dev, MCU_WM_UNI_CMD(BAND_CONFIG),
				 &req, sizeof(req), true);
}

int mt7996_mcu_set_rts_thresh(struct mt7996_phy *phy, u32 val)
{
	struct {
		u8 band_idx;
		u8 _rsv[3];

		__le16 tag;
		__le16 len;
		__le32 len_thresh;
		__le32 pkt_thresh;
	} __packed req = {
		.band_idx = phy->mt76->band_idx,
		.tag = cpu_to_le16(UNI_BAND_CONFIG_RTS_THRESHOLD),
		.len = cpu_to_le16(sizeof(req) - 4),
		.len_thresh = cpu_to_le32(val),
		.pkt_thresh = cpu_to_le32(0x2),
	};

	return mt76_mcu_send_msg(&phy->dev->mt76, MCU_WM_UNI_CMD(BAND_CONFIG),
				 &req, sizeof(req), true);
}

int mt7996_mcu_set_radio_en(struct mt7996_phy *phy, bool enable)
{
	struct {
		u8 band_idx;
		u8 _rsv[3];

		__le16 tag;
		__le16 len;
		u8 enable;
		u8 _rsv2[3];
	} __packed req = {
		.band_idx = phy->mt76->band_idx,
		.tag = cpu_to_le16(UNI_BAND_CONFIG_RADIO_ENABLE),
		.len = cpu_to_le16(sizeof(req) - 4),
		.enable = enable,
	};

	return mt76_mcu_send_msg(&phy->dev->mt76, MCU_WM_UNI_CMD(BAND_CONFIG),
				 &req, sizeof(req), true);
}

int mt7996_mcu_rdd_resume_tx(struct mt7996_phy *phy)
{
	struct {
		u8 band_idx;
		u8 _rsv[3];

		__le16 tag;
		__le16 len;
		u8 mac_enable;
		u8 _rsv2[3];
	} __packed req = {
		.band_idx = phy->mt76->band_idx,
		.tag = cpu_to_le16(UNI_BAND_CONFIG_MAC_ENABLE_CTRL),
		.len = cpu_to_le16(sizeof(req) - 4),
		.mac_enable = 2,
	};
	int ret;

	if (!phy->rdd_tx_paused)
		return 0;

	ret = mt76_mcu_send_msg(&phy->dev->mt76, MCU_WM_UNI_CMD(BAND_CONFIG),
				&req, sizeof(req), true);
	if (!ret)
		phy->rdd_tx_paused = false;

	return ret;
}

int mt7996_mcu_rdd_cmd(struct mt7996_dev *dev, int cmd, u8 rdd_idx, u8 val)
{
	struct {
		u8 _rsv[4];

		__le16 tag;
		__le16 len;

		u8 ctrl;
		u8 rdd_idx;
		u8 rdd_rx_sel;
		u8 val;
		u8 rsv[4];
	} __packed req = {
		.tag = cpu_to_le16(UNI_RDD_CTRL_PARM),
		.len = cpu_to_le16(sizeof(req) - 4),
		.ctrl = cmd,
		.rdd_idx = rdd_idx,
		.val = val,
	};

	return mt76_mcu_send_msg(&dev->mt76, MCU_WM_UNI_CMD(RDD_CTRL),
				 &req, sizeof(req), true);
}

int mt7996_mcu_wtbl_update_hdr_trans(struct mt7996_dev *dev,
				     struct ieee80211_vif *vif,
				     struct mt7996_vif_link *link,
				     struct mt7996_sta_link *msta_link)
{
	struct sk_buff *skb;

	skb = __mt76_connac_mcu_alloc_sta_req(&dev->mt76, &link->mt76,
					      &msta_link->wcid,
					      MT7996_STA_UPDATE_MAX_SIZE);
	if (IS_ERR(skb))
		return PTR_ERR(skb);

	/* starec hdr trans */
	mt7996_mcu_sta_hdr_trans_tlv(dev, skb, vif, &msta_link->wcid);
	return mt76_mcu_skb_send_msg(&dev->mt76, skb,
				     MCU_WMWA_UNI_CMD(STA_REC_UPDATE), true);
}

int mt7996_mcu_ps_leave(struct mt7996_dev *dev, struct mt7996_vif_link *link,
			struct mt7996_sta_link *msta_link)
{
	struct sk_buff *skb;

	skb = __mt76_connac_mcu_alloc_sta_req(&dev->mt76, &link->mt76,
					      &msta_link->wcid,
					      MT7996_STA_UPDATE_MAX_SIZE);
	if (IS_ERR(skb))
		return PTR_ERR(skb);

	mt76_connac_mcu_add_tlv(skb, STA_REC_PS_LEAVE,
				sizeof(struct sta_rec_ps_leave));

	return mt76_mcu_skb_send_msg(&dev->mt76, skb,
				     MCU_WMWA_UNI_CMD(STA_REC_UPDATE), true);
}

int mt7996_mcu_set_fixed_rate_table(struct mt7996_phy *phy, u8 table_idx,
				    u16 rate_idx, bool beacon)
{
#define UNI_FIXED_RATE_TABLE_SET	0
#define SPE_IXD_SELECT_TXD		0
#define SPE_IXD_SELECT_BMC_WTBL		1
	struct mt7996_dev *dev = phy->dev;
	struct fixed_rate_table_ctrl req = {
		.tag = cpu_to_le16(UNI_FIXED_RATE_TABLE_SET),
		.len = cpu_to_le16(sizeof(req) - 4),
		.table_idx = table_idx,
		.rate_idx = cpu_to_le16(rate_idx),
		.gi = 1,
		.he_ltf = 1,
	};
	u8 band_idx = phy->mt76->band_idx;

	if (beacon) {
		req.spe_idx_sel = SPE_IXD_SELECT_TXD;
		req.spe_idx = 24 + band_idx;
		phy->beacon_rate = rate_idx;
	} else {
		req.spe_idx_sel = SPE_IXD_SELECT_BMC_WTBL;
	}

	return mt76_mcu_send_msg(&dev->mt76, MCU_WM_UNI_CMD(FIXED_RATE_TABLE),
				 &req, sizeof(req), false);
}

int mt7996_mcu_rf_regval(struct mt7996_dev *dev, u32 regidx, u32 *val, bool set)
{
	struct {
		u8 __rsv1[4];

		__le16 tag;
		__le16 len;
		__le16 idx;
		u8 __rsv2[2];
		__le32 ofs;
		__le32 data;
	} __packed *res, req = {
		.tag = cpu_to_le16(UNI_CMD_ACCESS_RF_REG_BASIC),
		.len = cpu_to_le16(sizeof(req) - 4),

		.idx = cpu_to_le16(u32_get_bits(regidx, GENMASK(31, 24))),
		.ofs = cpu_to_le32(u32_get_bits(regidx, GENMASK(23, 0))),
		.data = set ? cpu_to_le32(*val) : 0,
	};
	struct sk_buff *skb;
	int ret;

	if (set)
		return mt76_mcu_send_msg(&dev->mt76, MCU_WM_UNI_CMD(REG_ACCESS),
					 &req, sizeof(req), true);

	ret = mt76_mcu_send_and_get_msg(&dev->mt76,
					MCU_WM_UNI_CMD_QUERY(REG_ACCESS),
					&req, sizeof(req), true, &skb);
	if (ret)
		return ret;
	if (skb->len < sizeof(*res)) {
		dev_kfree_skb(skb);
		return -EINVAL;
	}

	res = (void *)skb->data;
	*val = le32_to_cpu(res->data);
	dev_kfree_skb(skb);

	return 0;
}

int mt7996_mcu_trigger_assert(struct mt7996_dev *dev)
{
	struct {
		__le16 tag;
		__le16 len;
		u8 enable;
		u8 rsv[3];
	} __packed req = {
		.len = cpu_to_le16(sizeof(req) - 4),
		.enable = true,
	};

	return mt76_mcu_send_msg(&dev->mt76, MCU_WM_UNI_CMD(ASSERT_DUMP),
				 &req, sizeof(req), false);
}

int mt7996_mcu_set_rro(struct mt7996_dev *dev, u16 tag, u16 val)
{
	struct {
		u8 __rsv1[4];
		__le16 tag;
		__le16 len;
		union {
			struct {
				u8 type;
				u8 __rsv2[3];
			} __packed platform_type;
			struct {
				u8 type;
				u8 dest;
				u8 __rsv2[2];
			} __packed bypass_mode;
			struct {
				u8 path;
				u8 __rsv2[3];
			} __packed txfree_path;
			struct {
				__le16 flush_one;
				__le16 flush_all;
				u8 __rsv2[4];
			} __packed timeout;
		};
	} __packed req = {
		.tag = cpu_to_le16(tag),
		.len = cpu_to_le16(sizeof(req) - 4),
	};

	switch (tag) {
	case UNI_RRO_SET_PLATFORM_TYPE:
		req.platform_type.type = val;
		break;
	case UNI_RRO_SET_BYPASS_MODE:
		req.bypass_mode.type = val;
		break;
	case UNI_RRO_SET_TXFREE_PATH:
		req.txfree_path.path = val;
		break;
	case UNI_RRO_SET_FLUSH_TIMEOUT:
		req.timeout.flush_one = cpu_to_le16(val);
		req.timeout.flush_all = cpu_to_le16(2 * val);
		break;
	default:
		return -EINVAL;
	}

	return mt76_mcu_send_msg(&dev->mt76, MCU_WM_UNI_CMD(RRO), &req,
				 sizeof(req), true);
}

int mt7996_mcu_get_all_sta_info(struct mt7996_phy *phy, u16 tag)
{
	struct mt7996_dev *dev = phy->dev;
	struct {
		u8 _rsv[4];

		__le16 tag;
		__le16 len;
	} __packed req = {
		.tag = cpu_to_le16(tag),
		.len = cpu_to_le16(sizeof(req) - 4),
	};

	return mt76_mcu_send_msg(&dev->mt76, MCU_WM_UNI_CMD(ALL_STA_INFO),
				 &req, sizeof(req), false);
}

static void
mt7996_mcu_sta_ack_signal(struct mt7996_dev *dev, u16 wlan_idx, u8 *rcpi)
{
	struct mt7996_sta_link *msta_link;
	struct mt76_vif_link *mlink;
	struct mt76_wcid *wcid;
	s8 rssi[4];
	int i;

	wcid = mt76_wcid_ptr(dev, wlan_idx);
	if (!wcid || !wcid->sta)
		return;

	for (i = 0; i < ARRAY_SIZE(rssi); i++)
		rssi[i] = to_rssi(MT_PRXV_RCPI0, rcpi[i]);

	msta_link = container_of(wcid, struct mt7996_sta_link, wcid);
	mlink = rcu_dereference(msta_link->sta->vif->mt76.link[wcid->link_id]);
	if (mlink) {
		struct mt76_phy *mphy = mt76_vif_link_phy(mlink);

		if (mphy)
			msta_link->ack_signal =
				mt76_rx_signal(mphy->antenna_mask, rssi);
	}

	ewma_avg_signal_add(&msta_link->avg_ack_signal,
			    -msta_link->ack_signal);
}

int mt7996_mcu_get_per_sta_rssi(struct mt7996_dev *dev, u16 *sta_list,
				u16 sta_num)
{
	struct mt7996_mcu_per_sta_info_event *res;
	struct {
		u8 __rsv1;
		u8 unsolicit;
		u8 __rsv2[2];

		__le16 tag;
		__le16 len;
		__le16 sta_num;
		u8 __rsv3[2];
		__le16 sta_list[MT7996_PER_STA_INFO_MAX_NUM];
	} __packed req = {
		.tag = cpu_to_le16(UNI_PER_STA_RSSI),
		.len = cpu_to_le16(sizeof(req) - 4),
		.sta_num = cpu_to_le16(sta_num),
	};
	struct sk_buff *skb;
	u16 i, n;
	int ret;

	if (sta_num > ARRAY_SIZE(req.sta_list))
		return -EINVAL;

	for (i = 0; i < sta_num; i++)
		req.sta_list[i] = cpu_to_le16(sta_list[i]);

	ret = mt76_mcu_send_and_get_msg(&dev->mt76,
					MCU_WM_UNI_CMD(PER_STA_INFO),
					&req, sizeof(req), true, &skb);
	if (ret)
		return ret;

	res = (struct mt7996_mcu_per_sta_info_event *)skb->data;
	if (skb->len < sizeof(*res) ||
	    le16_to_cpu(res->tag) != UNI_PER_STA_RSSI) {
		ret = -EINVAL;
		goto out;
	}

	n = min_t(u16, sta_num,
		  (skb->len - sizeof(*res)) / sizeof(res->rssi[0]));

	rcu_read_lock();
	for (i = 0; i < n; i++)
		mt7996_mcu_sta_ack_signal(dev,
					  le16_to_cpu(res->rssi[i].wlan_idx),
					  res->rssi[i].rcpi);
	rcu_read_unlock();

out:
	dev_kfree_skb(skb);
	return ret;
}

int mt7996_mcu_wed_rro_reset_sessions(struct mt7996_dev *dev, u16 id)
{
	struct {
		u8 __rsv[4];

		__le16 tag;
		__le16 len;
		__le16 session_id;
		u8 pad[4];
	} __packed req = {
		.tag = cpu_to_le16(UNI_RRO_DEL_BA_SESSION),
		.len = cpu_to_le16(sizeof(req) - 4),
		.session_id = cpu_to_le16(id),
	};

	return mt76_mcu_send_msg(&dev->mt76, MCU_WM_UNI_CMD(RRO), &req,
				 sizeof(req), true);
}

int mt7996_mcu_set_sniffer_mode(struct mt7996_phy *phy, bool enabled)
{
	struct mt7996_dev *dev = phy->dev;
	struct {
		u8 band_idx;
		u8 _rsv[3];
		__le16 tag;
		__le16 len;
		u8 enable;
		u8 _pad[3];
	} __packed req = {
		.band_idx = phy->mt76->band_idx,
		.tag = 0,
		.len = cpu_to_le16(sizeof(req) - 4),
		.enable = enabled,
	};

	return mt76_mcu_send_msg(&dev->mt76, MCU_WM_UNI_CMD(SNIFFER), &req,
				 sizeof(req), true);
}

int mt7996_mcu_set_tx_power_ctrl(struct mt7996_phy *phy, u8 power_ctrl_id, u8 data)
{
	struct mt7996_dev *dev = phy->dev;
	struct tx_power_ctrl req = {
		.tag = cpu_to_le16(power_ctrl_id),
		.len = cpu_to_le16(sizeof(req) - 4),
		.power_ctrl_id = power_ctrl_id,
		.band_idx = phy->mt76->band_idx,
	};

	switch (power_ctrl_id) {
	case UNI_TXPOWER_SKU_POWER_LIMIT_CTRL:
		req.sku_enable = !!data;
		break;
	case UNI_TXPOWER_PERCENTAGE_CTRL:
		req.percentage_ctrl_enable = !!data;
		break;
	case UNI_TXPOWER_PERCENTAGE_DROP_CTRL:
		req.power_drop_level = data;
		break;
	case UNI_TXPOWER_BACKOFF_POWER_LIMIT_CTRL:
		req.bf_backoff_enable = !!data;
		break;
	case UNI_TXPOWER_ATE_MODE_CTRL:
		req.ate_mode_enable = !!data;
		break;
	default:
		return -EINVAL;
	}

	return mt76_mcu_send_msg(&dev->mt76, MCU_WM_UNI_CMD(TXPOWER),
				 &req, sizeof(req), false);
}

int mt7996_mcu_set_txpower_sku(struct mt7996_phy *phy)
{
#define TX_POWER_LIMIT_TABLE_RATE	0
	struct mt7996_dev *dev = phy->dev;
	struct mt76_phy *mphy = phy->mt76;
	struct tx_power_limit_table_ctrl {
		u8 __rsv1[4];

		__le16 tag;
		__le16 len;
		u8 power_ctrl_id;
		u8 power_limit_type;
		u8 band_idx;
	} __packed req = {
		.tag = cpu_to_le16(UNI_TXPOWER_POWER_LIMIT_TABLE_CTRL),
		.len = cpu_to_le16(sizeof(req) + MT7996_SKU_PATH_NUM - 4),
		.power_ctrl_id = UNI_TXPOWER_POWER_LIMIT_TABLE_CTRL,
		.power_limit_type = TX_POWER_LIMIT_TABLE_RATE,
		.band_idx = phy->mt76->band_idx,
	};
	struct mt76_power_limits la = {};
	struct sk_buff *skb;
	int i, tx_power;

	tx_power = mphy->chandef.chan->max_power;
	if (phy->txpower != INT_MIN)
		tx_power = min(tx_power, phy->txpower);
	tx_power = mt76_get_power_bound(mphy, tx_power);
	tx_power = mt76_get_rate_power_limits(mphy, mphy->chandef.chan,
					      &la, tx_power);
	mphy->txpower_cur = tx_power;

	skb = mt76_mcu_msg_alloc(&dev->mt76, NULL,
				 sizeof(req) + MT7996_SKU_PATH_NUM);
	if (!skb)
		return -ENOMEM;

	skb_put_data(skb, &req, sizeof(req));
	/* cck and ofdm */
	skb_put_data(skb, &la.cck, sizeof(la.cck));
	skb_put_data(skb, &la.ofdm, sizeof(la.ofdm));
	/* ht20 */
	skb_put_data(skb, &la.mcs[0], 8);
	/* ht40 */
	skb_put_data(skb, &la.mcs[1], 9);

	/* vht */
	for (i = 0; i < 4; i++) {
		skb_put_data(skb, &la.mcs[i], sizeof(la.mcs[i]));
		skb_put_zero(skb, 2);  /* padding */
	}

	/* he */
	skb_put_data(skb, &la.ru[0], sizeof(la.ru));
	/* eht */
	skb_put_data(skb, &la.eht[0], sizeof(la.eht));

	/* padding */
	skb_put_zero(skb, MT7996_SKU_PATH_NUM - MT7996_SKU_RATE_NUM);

	return mt76_mcu_skb_send_msg(&dev->mt76, skb,
				     MCU_WM_UNI_CMD(TXPOWER), true);
}

int mt7996_mcu_cp_support(struct mt7996_dev *dev, u8 mode)
{
	__le32 cp_mode;

	if (mode < mt76_connac_lmac_mapping(IEEE80211_AC_BE) ||
	    mode > mt76_connac_lmac_mapping(IEEE80211_AC_VO))
		return -EINVAL;

	if (!mt7996_has_wa(dev)) {
		struct {
			u8 _rsv[4];

			__le16 tag;
			__le16 len;
			u8 cp_mode;
			u8 rsv[3];
		} __packed req = {
			.tag = cpu_to_le16(UNI_CMD_SDO_CP_MODE),
			.len = cpu_to_le16(sizeof(req) - 4),
			.cp_mode = mode,
		};

		return mt76_mcu_send_msg(&dev->mt76, MCU_WA_UNI_CMD(SDO),
					 &req, sizeof(req), false);
	}

	cp_mode = cpu_to_le32(mode);

	return mt76_mcu_send_msg(&dev->mt76, MCU_WA_EXT_CMD(CP_SUPPORT),
				 &cp_mode, sizeof(cp_mode), true);
}

int mt7996_mcu_set_dup_wtbl(struct mt7996_dev *dev)
{
#define DUP_WTBL_NUM	80
	struct {
		u8 _rsv[4];

		__le16 tag;
		__le16 len;
		__le16 base;
		__le16 num;
		u8 _rsv2[4];
	} __packed req = {
		.tag = cpu_to_le16(UNI_CHIP_CONFIG_DUP_WTBL),
		.len = cpu_to_le16(sizeof(req) - 4),
		.base = cpu_to_le16(MT7996_WTBL_STA - DUP_WTBL_NUM + 1),
		.num = cpu_to_le16(DUP_WTBL_NUM),
	};

	return mt76_mcu_send_msg(&dev->mt76, MCU_WM_UNI_CMD(CHIP_CONFIG), &req,
				 sizeof(req), true);
}
