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

#include "mt76x2.h"

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
	struct mt76x2_wcid *wcid = &mvif->group_wcid;
	struct mt76x2_queue *q;
	int qid = skb_get_queue_mapping(skb);

	if (WARN_ON(qid >= MT_TXQ_PSD)) {
		qid = MT_TXQ_BE;
		skb_set_queue_mapping(skb, qid);
	}

	if (control->sta) {
		msta = (struct mt76x2_sta *) control->sta->drv_priv;
		wcid = &msta->wcid;
	}

	if (!wcid->tx_rate_set)
		ieee80211_get_tx_rates(info->control.vif, control->sta, skb,
				       info->control.rates, 1);

	q = &dev->q_tx[qid];

	spin_lock_bh(&q->lock);
	mt76x2_tx_queue_skb(dev, q, skb, wcid, control->sta);
	mt76x2_kick_queue(dev, q);

	if (q->queued > q->ndesc - 8)
		ieee80211_stop_queue(hw, skb_get_queue_mapping(skb));
	spin_unlock_bh(&q->lock);
}

void mt76x2_tx_complete(struct mt76x2_dev *dev, struct sk_buff *skb)
{
	struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);
	struct mt76x2_queue *q;
	int qid = skb_get_queue_mapping(skb);

	if (info->flags & IEEE80211_TX_CTL_AMPDU) {
		ieee80211_free_txskb(dev->hw, skb);
	} else {
		ieee80211_tx_info_clear_status(info);
		info->status.rates[0].idx = -1;
		info->flags |= IEEE80211_TX_STAT_ACK;
		ieee80211_tx_status(dev->hw, skb);
	}

	q = &dev->q_tx[qid];
	if (q->queued < q->ndesc - 8)
		ieee80211_wake_queue(dev->hw, qid);
}

static void
mt76x2_update_beacon_iter(void *priv, u8 *mac, struct ieee80211_vif *vif)
{
	struct mt76x2_dev *dev = (struct mt76x2_dev *) priv;
	struct mt76x2_vif *mvif = (struct mt76x2_vif *) vif->drv_priv;
	struct ieee80211_tx_info *info;
	struct sk_buff *skb = NULL;

	if (!(dev->beacon_mask & BIT(mvif->idx)))
		return;

	skb = ieee80211_beacon_get(dev->hw, vif);
	if (!skb)
		return;

	info = IEEE80211_SKB_CB(skb);
	info->flags |= IEEE80211_TX_CTL_ASSIGN_SEQ;
	mt76x2_mac_set_beacon(dev, mvif->idx, skb);
}

static void
mt76x2_skb_set_moredata(struct sk_buff *skb, bool enable)
{
	struct ieee80211_hdr *hdr = (struct ieee80211_hdr *) skb->data;

	if (enable)
		hdr->frame_control |= cpu_to_le16(IEEE80211_FCTL_MOREDATA);
	else
		hdr->frame_control &= ~cpu_to_le16(IEEE80211_FCTL_MOREDATA);
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

	skb = ieee80211_get_buffered_bc(dev->hw, vif);
	if (!skb)
		return;

	info = IEEE80211_SKB_CB(skb);
	info->control.vif = vif;
	info->flags |= IEEE80211_TX_CTL_ASSIGN_SEQ;
	mt76x2_skb_set_moredata(skb, true);
	__skb_queue_tail(&data->q, skb);
	data->tail[mvif->idx] = skb;
}

void mt76x2_pre_tbtt_tasklet(unsigned long arg)
{
	struct mt76x2_dev *dev = (struct mt76x2_dev *) arg;
	struct mt76x2_queue *q = &dev->q_tx[MT_TXQ_PSD];
	struct beacon_bc_data data = {};
	struct sk_buff *skb;
	int i, nframes;

	data.dev = dev;
	__skb_queue_head_init(&data.q);

	ieee80211_iterate_active_interfaces_atomic(dev->hw,
		IEEE80211_IFACE_ITER_RESUME_ALL,
		mt76x2_update_beacon_iter, dev);

	do {
		nframes = skb_queue_len(&data.q);
		ieee80211_iterate_active_interfaces_atomic(dev->hw,
			IEEE80211_IFACE_ITER_RESUME_ALL,
			mt76x2_add_buffered_bc, &data);
	} while (nframes != skb_queue_len(&data.q));

	if (!nframes)
		return;

	for (i = 0; i < ARRAY_SIZE(data.tail); i++) {
		if (!data.tail[i])
			continue;

		mt76x2_skb_set_moredata(data.tail[i], false);
	}

	spin_lock_bh(&q->lock);
	while ((skb = __skb_dequeue(&data.q)) != NULL) {
		struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);
		struct ieee80211_vif *vif = info->control.vif;
		struct mt76x2_vif *mvif = (struct mt76x2_vif *) vif->drv_priv;

		mt76x2_tx_queue_skb(dev, q, skb, &mvif->group_wcid, NULL);
	}
	spin_unlock_bh(&q->lock);
}

static int
mt76x2_txq_get_qid(struct ieee80211_txq *txq)
{
	if (!txq->sta)
		return MT_TXQ_BE;

	return txq->ac;
}

static struct sk_buff *
mt76x2_txq_dequeue(struct mt76x2_dev *dev, struct mt76x2_txq *mtxq, bool ps)
{
	struct ieee80211_txq *txq = mtxq_to_txq(mtxq);
	struct sk_buff *skb;

	skb = skb_dequeue(&mtxq->retry_q);
	if (skb) {
		u8 tid = skb->priority & IEEE80211_QOS_CTL_TID_MASK;

		if (ps && skb_queue_empty(&mtxq->retry_q));
			ieee80211_sta_set_buffered(txq->sta, tid, false);

		return skb;
	}

	skb = ieee80211_tx_dequeue(dev->hw, txq);
	if (!skb)
		return NULL;

	return skb;
}

static void
mt76x2_check_agg_ssn(struct mt76x2_txq *mtxq, struct sk_buff *skb)
{
	struct ieee80211_hdr *hdr = (struct ieee80211_hdr *) skb->data;

	if (!ieee80211_is_data_qos(hdr->frame_control))
		return;

	mtxq->agg_ssn = le16_to_cpu(hdr->seq_ctrl) + 0x10;
}

static void
mt76x2_queue_ps_skb(struct mt76x2_dev *dev, struct ieee80211_sta *sta,
		  struct sk_buff *skb, bool last)
{
	struct mt76x2_sta *msta = (struct mt76x2_sta *) sta->drv_priv;
	struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);
	struct mt76x2_queue *hwq = &dev->q_tx[MT_TXQ_PSD];
	struct mt76x2_wcid *wcid = &msta->wcid;

	info->control.flags |= IEEE80211_TX_CTRL_PS_RESPONSE;
	if (last)
		info->flags |= IEEE80211_TX_STATUS_EOSP;

	mt76x2_skb_set_moredata(skb, !last);
	mt76x2_tx_queue_skb(dev, hwq, skb, wcid, sta);
}

void
mt76x2_release_buffered_frames(struct ieee80211_hw *hw, struct ieee80211_sta *sta,
			     u16 tids, int nframes,
			     enum ieee80211_frame_release_type reason,
			     bool more_data)
{
	struct mt76x2_dev *dev = hw->priv;
	struct sk_buff *last_skb = NULL;
	struct mt76x2_queue *hwq = &dev->q_tx[MT_TXQ_PSD];
	int i;

	for (i = 0; tids && nframes; i++, tids >>= 1) {
		struct ieee80211_txq *txq = sta->txq[i];
		struct mt76x2_txq *mtxq = (struct mt76x2_txq *) txq->drv_priv;
		struct sk_buff *skb;

		if (!(tids & 1))
			continue;

		do {
			skb = mt76x2_txq_dequeue(dev, mtxq, true);
			if (!skb)
				break;

			if (mtxq->aggr)
				mt76x2_check_agg_ssn(mtxq, skb);

			nframes--;
			if (last_skb) {
				mt76x2_queue_ps_skb(dev, sta, last_skb, false);
				last_skb = skb;
			}
		} while (nframes);
	}

	if (!last_skb)
		return;

	mt76x2_queue_ps_skb(dev, sta, last_skb, true);
	mt76x2_kick_queue(dev, hwq);
}

static int
mt76x2_txq_send_burst(struct mt76x2_dev *dev, struct mt76x2_queue *hwq,
		    struct mt76x2_txq *mtxq, bool *empty)
{
	struct ieee80211_txq *txq = mtxq_to_txq(mtxq);
	struct ieee80211_tx_info *info;
	struct mt76x2_wcid *wcid;
	struct sk_buff *skb = NULL;
	int n_frames = 1, limit;
	struct ieee80211_tx_rate tx_rate;
	bool ampdu;
	bool probe;
	int idx;

	if (txq->sta) {
		struct mt76x2_sta *sta = (struct mt76x2_sta *) txq->sta->drv_priv;
		wcid = &sta->wcid;
	} else {
		struct mt76x2_vif *mvif = (struct mt76x2_vif *) txq->vif->drv_priv;
		wcid = &mvif->group_wcid;
	}

	skb = mt76x2_txq_dequeue(dev, mtxq, false);
	if (!skb) {
		*empty = true;
		return 0;
	}

	info = IEEE80211_SKB_CB(skb);
	if (!wcid->tx_rate_set)
		ieee80211_get_tx_rates(txq->vif, txq->sta, skb,
				       info->control.rates, 1);
	tx_rate = info->control.rates[0];

	probe = (info->flags & IEEE80211_TX_CTL_RATE_CTRL_PROBE);
	ampdu = IEEE80211_SKB_CB(skb)->flags & IEEE80211_TX_CTL_AMPDU;
	limit = ampdu ? 16 : 3;

	if (ampdu)
		mt76x2_check_agg_ssn(mtxq, skb);

	idx = mt76x2_tx_queue_skb(dev, hwq, skb, wcid, txq->sta);

	if (idx < 0)
		return idx;

	do {
		bool cur_ampdu;

		if (probe)
			break;

		skb = mt76x2_txq_dequeue(dev, mtxq, false);
		if (!skb) {
			*empty = true;
			break;
		}

		cur_ampdu = info->flags & IEEE80211_TX_CTL_AMPDU;

		if (ampdu != cur_ampdu ||
		    (info->flags & IEEE80211_TX_CTL_RATE_CTRL_PROBE)) {
			skb_queue_tail(&mtxq->retry_q, skb);
			break;
		}

		info = IEEE80211_SKB_CB(skb);
		info->control.rates[0] = tx_rate;

		if (cur_ampdu)
			mt76x2_check_agg_ssn(mtxq, skb);

		idx = mt76x2_tx_queue_skb(dev, hwq, skb, wcid, txq->sta);
		if (idx < 0)
			return idx;

		n_frames++;
	} while (n_frames < limit);

	if (!probe) {
		hwq->swq_queued++;
		hwq->entry[idx].schedule = true;
	}

	mt76x2_kick_queue(dev, hwq);

	return n_frames;
}

static int
mt76x2_txq_schedule_list(struct mt76x2_dev *dev, struct mt76x2_queue *hwq)
{
	struct mt76x2_txq *mtxq, *mtxq_last;
	int len = 0;

restart:
	mtxq_last = list_last_entry(&hwq->swq, struct mt76x2_txq, list);
	while (!list_empty(&hwq->swq)) {
		bool empty = false;
		int cur;

		mtxq = list_first_entry(&hwq->swq, struct mt76x2_txq, list);
		if (mtxq->send_bar && mtxq->aggr) {
			struct ieee80211_txq *txq = mtxq_to_txq(mtxq);
			struct ieee80211_sta *sta = txq->sta;
			struct ieee80211_vif *vif = txq->vif;
			u16 agg_ssn = mtxq->agg_ssn;
			u8 tid = txq->tid;

			mtxq->send_bar = false;
			spin_unlock_bh(&hwq->lock);
			ieee80211_send_bar(vif, sta->addr, tid, agg_ssn);
			spin_lock_bh(&hwq->lock);
			goto restart;
		}

		list_del_init(&mtxq->list);

		cur = mt76x2_txq_send_burst(dev, hwq, mtxq, &empty);
		if (!empty)
			list_add_tail(&mtxq->list, &hwq->swq);

		if (cur < 0)
			return cur;

		len += cur;

		if (mtxq == mtxq_last)
			break;
	}

	return len;
}

void mt76x2_txq_schedule(struct mt76x2_dev *dev, struct mt76x2_queue *hwq)
{
	int len;

	do {
		if (hwq->swq_queued >= 4 || list_empty(&hwq->swq))
			break;

		len = mt76x2_txq_schedule_list(dev, hwq);
	} while (len > 0);
}

void mt76x2_txq_init(struct mt76x2_dev *dev, struct ieee80211_txq *txq)
{
	struct mt76x2_txq *mtxq;

	if (!txq)
		return;

	mtxq = (struct mt76x2_txq *) txq->drv_priv;
	INIT_LIST_HEAD(&mtxq->list);
	skb_queue_head_init(&mtxq->retry_q);

	mtxq->hwq = &dev->q_tx[mt76x2_txq_get_qid(txq)];
}

void
mt76x2_stop_tx_queues(struct mt76x2_dev *dev, struct ieee80211_sta *sta)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(sta->txq); i++) {
		struct ieee80211_txq *txq = sta->txq[i];
		struct mt76x2_txq *mtxq = (struct mt76x2_txq *) txq->drv_priv;

		spin_lock_bh(&mtxq->hwq->lock);
		mtxq->send_bar = mtxq->aggr;
		if (!list_empty(&mtxq->list))
		    list_del_init(&mtxq->list);
		spin_unlock_bh(&mtxq->hwq->lock);
	}
}

void mt76x2_wake_tx_queue(struct ieee80211_hw *hw, struct ieee80211_txq *txq)
{
	struct mt76x2_dev *dev = hw->priv;
	struct mt76x2_txq *mtxq = (struct mt76x2_txq *) txq->drv_priv;
	struct mt76x2_queue *hwq = mtxq->hwq;

	spin_lock_bh(&hwq->lock);
	if (list_empty(&mtxq->list))
		list_add_tail(&mtxq->list, &hwq->swq);
	mt76x2_txq_schedule(dev, hwq);
	spin_unlock_bh(&hwq->lock);
}

void mt76x2_txq_remove(struct mt76x2_dev *dev, struct ieee80211_txq *txq)
{
	struct mt76x2_txq *mtxq;
	struct mt76x2_queue *hwq;
	struct sk_buff *skb;

	if (!txq)
		return;

	mtxq = (struct mt76x2_txq *) txq->drv_priv;
	hwq = mtxq->hwq;

	spin_lock_bh(&hwq->lock);
	if (!list_empty(&mtxq->list))
		list_del(&mtxq->list);
	spin_unlock_bh(&hwq->lock);

	while ((skb = skb_dequeue(&mtxq->retry_q)) != NULL)
		ieee80211_free_txskb(dev->hw, skb);
}
