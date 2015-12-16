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

#include "mt76.h"

static int
mt76_txq_get_qid(struct ieee80211_txq *txq)
{
	if (!txq->sta)
		return MT_TXQ_BE;

	return txq->ac;
}

static struct sk_buff *
mt76_txq_dequeue(struct mt76_dev *dev, struct mt76_txq *mtxq, bool ps)
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
mt76_check_agg_ssn(struct mt76_txq *mtxq, struct sk_buff *skb)
{
	struct ieee80211_hdr *hdr = (struct ieee80211_hdr *) skb->data;

	if (!ieee80211_is_data_qos(hdr->frame_control))
		return;

	mtxq->agg_ssn = le16_to_cpu(hdr->seq_ctrl) + 0x10;
}

static void
mt76_queue_ps_skb(struct mt76_dev *dev, struct ieee80211_sta *sta,
		  struct sk_buff *skb, bool last)
{
	struct mt76_wcid *wcid = (struct mt76_wcid *) sta->drv_priv;
	struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);
	struct mt76_queue *hwq = &dev->q_tx[MT_TXQ_PSD];

	info->control.flags |= IEEE80211_TX_CTRL_PS_RESPONSE;
	if (last)
		info->flags |= IEEE80211_TX_STATUS_EOSP;

	mt76_skb_set_moredata(skb, !last);
	mt76_tx_queue_skb(dev, hwq, skb, wcid, sta);
}

void
mt76_release_buffered_frames(struct ieee80211_hw *hw, struct ieee80211_sta *sta,
			     u16 tids, int nframes,
			     enum ieee80211_frame_release_type reason,
			     bool more_data)
{
	struct mt76_dev *dev = hw->priv;
	struct sk_buff *last_skb = NULL;
	struct mt76_queue *hwq = &dev->q_tx[MT_TXQ_PSD];
	int i;

	for (i = 0; tids && nframes; i++, tids >>= 1) {
		struct ieee80211_txq *txq = sta->txq[i];
		struct mt76_txq *mtxq = (struct mt76_txq *) txq->drv_priv;
		struct sk_buff *skb;

		if (!(tids & 1))
			continue;

		do {
			skb = mt76_txq_dequeue(dev, mtxq, true);
			if (!skb)
				break;

			if (mtxq->aggr)
				mt76_check_agg_ssn(mtxq, skb);

			nframes--;
			if (last_skb) {
				mt76_queue_ps_skb(dev, sta, last_skb, false);
				last_skb = skb;
			}
		} while (nframes);
	}

	if (!last_skb)
		return;

	mt76_queue_ps_skb(dev, sta, last_skb, true);
	dev->queue_ops->kick(dev, hwq);
}
EXPORT_SYMBOL_GPL(mt76_release_buffered_frames);

static int
mt76_txq_send_burst(struct mt76_dev *dev, struct mt76_queue *hwq,
		    struct mt76_txq *mtxq, bool *empty)
{
	struct ieee80211_txq *txq = mtxq_to_txq(mtxq);
	struct ieee80211_tx_info *info;
	struct mt76_wcid *wcid = mtxq->wcid;
	struct sk_buff *skb = NULL;
	int n_frames = 1, limit;
	struct ieee80211_tx_rate tx_rate;
	bool ampdu;
	bool probe;
	int idx;

	skb = mt76_txq_dequeue(dev, mtxq, false);
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
		mt76_check_agg_ssn(mtxq, skb);

	idx = mt76_tx_queue_skb(dev, hwq, skb, wcid, txq->sta);

	if (idx < 0)
		return idx;

	do {
		bool cur_ampdu;

		if (probe)
			break;

		skb = mt76_txq_dequeue(dev, mtxq, false);
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
			mt76_check_agg_ssn(mtxq, skb);

		idx = mt76_tx_queue_skb(dev, hwq, skb, wcid, txq->sta);
		if (idx < 0)
			return idx;

		n_frames++;
	} while (n_frames < limit);

	if (!probe) {
		hwq->swq_queued++;
		hwq->entry[idx].schedule = true;
	}

	dev->queue_ops->kick(dev, hwq);

	return n_frames;
}

static int
mt76_txq_schedule_list(struct mt76_dev *dev, struct mt76_queue *hwq)
{
	struct mt76_txq *mtxq, *mtxq_last;
	int len = 0;

restart:
	mtxq_last = list_last_entry(&hwq->swq, struct mt76_txq, list);
	while (!list_empty(&hwq->swq)) {
		bool empty = false;
		int cur;

		mtxq = list_first_entry(&hwq->swq, struct mt76_txq, list);
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

		cur = mt76_txq_send_burst(dev, hwq, mtxq, &empty);
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

void mt76_txq_schedule(struct mt76_dev *dev, struct mt76_queue *hwq)
{
	int len;

	do {
		if (hwq->swq_queued >= 4 || list_empty(&hwq->swq))
			break;

		len = mt76_txq_schedule_list(dev, hwq);
	} while (len > 0);
}
EXPORT_SYMBOL_GPL(mt76_txq_schedule);

void mt76_stop_tx_queues(struct mt76_dev *dev, struct ieee80211_sta *sta)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(sta->txq); i++) {
		struct ieee80211_txq *txq = sta->txq[i];
		struct mt76_txq *mtxq = (struct mt76_txq *) txq->drv_priv;

		spin_lock_bh(&mtxq->hwq->lock);
		mtxq->send_bar = mtxq->aggr;
		if (!list_empty(&mtxq->list))
		    list_del_init(&mtxq->list);
		spin_unlock_bh(&mtxq->hwq->lock);
	}
}
EXPORT_SYMBOL_GPL(mt76_stop_tx_queues);

void mt76_wake_tx_queue(struct ieee80211_hw *hw, struct ieee80211_txq *txq)
{
	struct mt76_dev *dev = hw->priv;
	struct mt76_txq *mtxq = (struct mt76_txq *) txq->drv_priv;
	struct mt76_queue *hwq = mtxq->hwq;

	spin_lock_bh(&hwq->lock);
	if (list_empty(&mtxq->list))
		list_add_tail(&mtxq->list, &hwq->swq);
	mt76_txq_schedule(dev, hwq);
	spin_unlock_bh(&hwq->lock);
}
EXPORT_SYMBOL_GPL(mt76_wake_tx_queue);

void mt76_txq_remove(struct mt76_dev *dev, struct ieee80211_txq *txq)
{
	struct mt76_txq *mtxq;
	struct mt76_queue *hwq;
	struct sk_buff *skb;

	if (!txq)
		return;

	mtxq = (struct mt76_txq *) txq->drv_priv;
	hwq = mtxq->hwq;

	spin_lock_bh(&hwq->lock);
	if (!list_empty(&mtxq->list))
		list_del(&mtxq->list);
	spin_unlock_bh(&hwq->lock);

	while ((skb = skb_dequeue(&mtxq->retry_q)) != NULL)
		ieee80211_free_txskb(dev->hw, skb);
}
EXPORT_SYMBOL_GPL(mt76_txq_remove);

void mt76_txq_init(struct mt76_dev *dev, struct ieee80211_txq *txq)
{
	struct mt76_txq *mtxq = (struct mt76_txq *) txq->drv_priv;

	INIT_LIST_HEAD(&mtxq->list);
	skb_queue_head_init(&mtxq->retry_q);

	mtxq->hwq = &dev->q_tx[mt76_txq_get_qid(txq)];
}
EXPORT_SYMBOL_GPL(mt76_txq_init);
