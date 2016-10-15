/*
 * Copyright (C) 2016 Felix Fietkau <nbd@nbd.name>
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include "mt7603.h"

struct beacon_bc_data {
	struct mt7603_dev *dev;
	struct sk_buff_head q;
	struct sk_buff *tail[MT7603_MAX_INTERFACES];
	int count[MT7603_MAX_INTERFACES];
};

static void
mt7603_update_beacon_iter(void *priv, u8 *mac, struct ieee80211_vif *vif)
{
	struct mt7603_dev *dev = (struct mt7603_dev *) priv;
	struct mt7603_vif *mvif = (struct mt7603_vif *) vif->drv_priv;
	struct sk_buff *skb = NULL;

	if (!(dev->beacon_mask & BIT(mvif->idx)))
		return;

	skb = ieee80211_beacon_get(mt76_hw(dev), vif);
	if (!skb)
		return;

	mt76_tx_queue_skb(&dev->mt76, &dev->mt76.q_tx[MT_TXQ_BEACON], skb,
			  &mvif->sta.wcid, NULL);
}

static void
mt7603_add_buffered_bc(void *priv, u8 *mac, struct ieee80211_vif *vif)
{
	struct beacon_bc_data *data = priv;
	struct mt7603_dev *dev = data->dev;
	struct mt7603_vif *mvif = (struct mt7603_vif *) vif->drv_priv;
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
	data->count[mvif->idx]++;
}

void mt7603_tbtt(struct mt7603_dev *dev)
{
	u32 mask = MT_WF_ARB_CAB_START_BSSn(0) |
		   (MT_WF_ARB_CAB_START_BSS0n(1) *
		    ((1 << (MT7603_MAX_INTERFACES - 1)) - 1));

	mt76_wr(dev, MT_WF_ARB_CAB_START, mask);
}

void mt7603_pre_tbtt_tasklet(unsigned long arg)
{
	struct mt7603_dev *dev = (struct mt7603_dev *) arg;
	struct mt76_queue *q;
	struct beacon_bc_data data = {};
	struct sk_buff *skb;
	int i, nframes;

	/* Flush all previous CAB queue packets */
	mt76_wr(dev, MT_WF_ARB_CAB_FLUSH, GENMASK(30, 0));

	data.dev = dev;
	__skb_queue_head_init(&data.q);

	q = &dev->mt76.q_tx[MT_TXQ_BEACON];
	spin_lock_bh(&q->lock);
	ieee80211_iterate_active_interfaces_atomic(mt76_hw(dev),
		IEEE80211_IFACE_ITER_RESUME_ALL,
		mt7603_update_beacon_iter, dev);
	mt76_queue_kick(dev, q);
	spin_unlock_bh(&q->lock);

	mt76_queue_tx_cleanup(dev, MT_TXQ_CAB, false);

	q = &dev->mt76.q_tx[MT_TXQ_CAB];
	do {
		nframes = skb_queue_len(&data.q);
		ieee80211_iterate_active_interfaces_atomic(mt76_hw(dev),
			IEEE80211_IFACE_ITER_RESUME_ALL,
			mt7603_add_buffered_bc, &data);
	} while (nframes != skb_queue_len(&data.q) && nframes < 8);

	if (!nframes)
		goto out;

	for (i = 0; i < ARRAY_SIZE(data.tail); i++) {
		if (!data.tail[i])
			continue;

		mt76_skb_set_moredata(data.tail[i], false);
	}

	spin_lock_bh(&q->lock);
	while ((skb = __skb_dequeue(&data.q)) != NULL) {
		struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);
		struct ieee80211_vif *vif = info->control.vif;
		struct mt7603_vif *mvif = (struct mt7603_vif *) vif->drv_priv;

		mt76_tx_queue_skb(&dev->mt76, q, skb, &mvif->sta.wcid, NULL);
	}
	mt76_queue_kick(dev, q);
	spin_unlock_bh(&q->lock);

	for (i = 0; i < ARRAY_SIZE(data.count); i++)
		mt76_wr(dev, MT_WF_ARB_CAB_COUNT(i),
			data.count[i] << MT_WF_ARB_CAB_COUNT_B0_SHIFT(i));

out:
	mt76_queue_tx_cleanup(dev, MT_TXQ_BEACON, false);
	if (dev->mt76.q_tx[MT_TXQ_BEACON].queued > MT7603_MAX_INTERFACES)
		dev->beacon_check++;
}

void mt7603_beacon_set_timer(struct mt7603_dev *dev, int idx, int intval)
{
	u32 pre_tbtt = MT7603_PRE_TBTT_TIME / 64;

	if (idx >= 0) {
		if (intval)
			dev->beacon_mask |= BIT(idx);
		else
			dev->beacon_mask &= BIT(idx);
	}

	if (!dev->beacon_mask || (!intval && idx < 0)) {
		mt7603_irq_disable(dev, MT_INT_MAC_IRQ3);
		mt76_clear(dev, MT_ARB_SCR, MT_ARB_SCR_BCNQ_OPMODE_MASK);
		mt76_wr(dev, MT_HW_INT_MASK(3), 0);
		return;
	}

	dev->beacon_int = intval;
	mt76_wr(dev, MT_TBTT,
		MT76_SET(MT_TBTT_PERIOD, intval) | MT_TBTT_CAL_ENABLE);

	mt76_wr(dev, MT_TBTT_TIMER_CFG, 0x99); /* start timer */

	mt76_rmw_field(dev, MT_ARB_SCR, MT_ARB_SCR_BCNQ_OPMODE_MASK,
		       MT_BCNQ_OPMODE_AP);
	mt76_clear(dev, MT_ARB_SCR,
		   MT_ARB_SCR_TBTT_BCN_PRIO | MT_ARB_SCR_TBTT_BCAST_PRIO);

	mt76_wr(dev, MT_PRE_TBTT, pre_tbtt);

	mt76_set(dev, MT_HW_INT_MASK(3),
		 MT_HW_INT3_PRE_TBTT0 | MT_HW_INT3_TBTT0);

	mt76_set(dev, MT_WF_ARB_BCN_START, MT_WF_ARB_BCN_START_BSSn(0));
	mt7603_irq_enable(dev, MT_INT_MAC_IRQ3);
}
