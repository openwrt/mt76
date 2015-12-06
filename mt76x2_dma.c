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
#include "mt76x2_dma.h"

struct mt76_txwi_cache {
	struct mt76_txwi txwi;
	dma_addr_t dma_addr;
	struct list_head list;
};

static inline int
mt76x2_rx_buf_offset(struct mt76x2_dev *dev)
{
	BUILD_BUG_ON(MT_RX_HEADROOM < sizeof(struct mt76x2_rxwi));

	return MT_RX_HEADROOM - sizeof(struct mt76x2_rxwi);
}

static struct mt76_txwi_cache *
mt76x2_alloc_txwi(struct mt76x2_dev *dev)
{
	struct mt76_txwi_cache *t;
	dma_addr_t addr;
	int size;

	size = (sizeof(*t) + L1_CACHE_BYTES - 1) & ~(L1_CACHE_BYTES - 1);
	t = devm_kzalloc(dev->mt76.dev, size, GFP_ATOMIC);
	if (!t)
		return NULL;

	addr = dma_map_single(dev->mt76.dev, &t->txwi, sizeof(t->txwi), DMA_TO_DEVICE);
	t->dma_addr = addr;

	return t;
}

static struct mt76_txwi_cache *
__mt76x2_get_txwi(struct mt76x2_dev *dev)
{
	struct mt76_txwi_cache *t = NULL;

	spin_lock_bh(&dev->lock);
	if (!list_empty(&dev->txwi_cache)) {
		t = list_first_entry(&dev->txwi_cache, struct mt76_txwi_cache,
				     list);
		list_del(&t->list);
	}
	spin_unlock_bh(&dev->lock);

	return t;
}

static struct mt76_txwi_cache *
mt76x2_get_txwi(struct mt76x2_dev *dev)
{
	struct mt76_txwi_cache *t = __mt76x2_get_txwi(dev);

	if (t)
		return t;

	return mt76x2_alloc_txwi(dev);
}

static void
mt76x2_put_txwi(struct mt76x2_dev *dev, struct mt76_txwi_cache *t)
{
	if (!t)
		return;

	spin_lock_bh(&dev->lock);
	list_add(&t->list, &dev->txwi_cache);
	spin_unlock_bh(&dev->lock);
}

static int
mt76x2_dma_add_rx_buf(struct mt76x2_dev *dev, struct mt76_queue *q,
		    dma_addr_t addr, int len)
{
	int offset = mt76x2_rx_buf_offset(dev);

	return mt76_queue_add_buf(dev, q, addr + offset, len - offset, 0, 0, 0);
}

static int
mt76x2_dma_rx_fill(struct mt76x2_dev *dev, struct mt76_queue *q)
{
	dma_addr_t addr;
	void *buf;
	int frames = 0;
	int len = SKB_WITH_OVERHEAD(q->buf_size);
	int idx;

	spin_lock_bh(&q->lock);

	while (q->queued < q->ndesc - 1) {
		buf = kzalloc(q->buf_size, GFP_ATOMIC);
		if (!buf)
			break;

		addr = dma_map_single(dev->mt76.dev, buf, len, DMA_FROM_DEVICE);
		if (dma_mapping_error(dev->mt76.dev, addr)) {
			kfree(buf);
			break;
		}

		idx = mt76x2_dma_add_rx_buf(dev, q, addr, len);
		q->entry[idx].buf = buf;
		frames++;
	}

	if (frames)
		mt76_queue_kick(dev, q);

	spin_unlock_bh(&q->lock);

	return frames;
}

int
mt76x2_tx_queue_mcu(struct mt76x2_dev *dev, enum mt76x2_txq_id qid,
		    struct sk_buff *skb, int cmd, int seq)
{
	struct mt76_queue *q = &dev->q_tx[qid];
	dma_addr_t addr;
	u32 tx_info;
	int idx;

	tx_info = MT_MCU_MSG_TYPE_CMD |
		  MT76_SET(MT_MCU_MSG_CMD_TYPE, cmd) |
		  MT76_SET(MT_MCU_MSG_CMD_SEQ, seq) |
		  MT76_SET(MT_MCU_MSG_PORT, CPU_TX_PORT) |
		  MT76_SET(MT_MCU_MSG_LEN, skb->len);

	addr = dma_map_single(dev->mt76.dev, skb->data, skb->len, DMA_TO_DEVICE);
	if (dma_mapping_error(dev->mt76.dev, addr))
		return -ENOMEM;

	spin_lock_bh(&q->lock);
	idx = mt76_queue_add_buf(dev, q, addr, skb->len, 0, 0, tx_info);
	q->entry[idx].skb = skb;
	mt76_queue_kick(dev, q);
	spin_unlock_bh(&q->lock);

	return 0;
}

int
mt76x2_tx_queue_skb(struct mt76x2_dev *dev, struct mt76_queue *q,
		    struct sk_buff *skb, struct mt76x2_wcid *wcid,
		    struct ieee80211_sta *sta)
{
	struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);
	struct mt76_txwi_cache *t;
	dma_addr_t addr;
	u32 tx_info = 0;
	int idx, ret, len;
	int qsel = MT_QSEL_EDCA;

	ret = -ENOMEM;
	t = mt76x2_get_txwi(dev);
	if (!t)
		goto free;

	dma_sync_single_for_cpu(dev->mt76.dev, t->dma_addr, sizeof(t->txwi),
				DMA_TO_DEVICE);
	mt76x2_mac_write_txwi(dev, &t->txwi, skb, wcid, sta);
	dma_sync_single_for_device(dev->mt76.dev, t->dma_addr, sizeof(t->txwi),
				   DMA_TO_DEVICE);

	ret = mt76_insert_hdr_pad(skb);
	if (ret)
		goto put_txwi;

	if (info->flags & IEEE80211_TX_CTL_RATE_CTRL_PROBE)
		qsel = 0;

	len = skb->len + sizeof(t->txwi);
	len += mt76x2_mac_skb_tx_overhead(dev, skb);
	tx_info = MT76_SET(MT_TXD_INFO_LEN, len) |
		  MT76_SET(MT_TXD_INFO_QSEL, qsel) |
		  MT_TXD_INFO_80211;

	if (!wcid || wcid->hw_key_idx == 0xff)
		tx_info |= MT_TXD_INFO_WIV;

	addr = dma_map_single(dev->mt76.dev, skb->data, skb->len, DMA_TO_DEVICE);
	if (dma_mapping_error(dev->mt76.dev, addr))
		goto put_txwi;

	idx = mt76_queue_add_buf(dev, q, t->dma_addr, sizeof(t->txwi),
				 addr, skb->len, tx_info);
	q->entry[idx].skb = skb;
	q->entry[idx].txwi = t;

	return idx;

put_txwi:
	mt76x2_put_txwi(dev, t);
free:
	ieee80211_free_txskb(mt76_hw(dev), skb);
	return ret;
}

static void
mt76x2_tx_cleanup_entry(struct mt76_dev *mdev, struct mt76_queue *q,
			struct mt76_queue_entry *e)
{
	struct mt76x2_dev *dev = container_of(mdev, struct mt76x2_dev, mt76);

	if (e->txwi) {
		mt76x2_mac_queue_txdone(dev, e->skb, &e->txwi->txwi);
		mt76x2_put_txwi(dev, e->txwi);
	} else {
		dev_kfree_skb_any(e->skb);
	}

	if (e->schedule)
		q->swq_queued--;
}

static void
mt76x2_rx_cleanup(struct mt76x2_dev *dev, struct mt76_queue *q)
{
	void *buf;

	spin_lock_bh(&q->lock);
	do {
		buf = mt76_queue_dequeue(dev, q, true, NULL, NULL);
		if (!buf)
			break;

		kfree(buf);
	} while (1);
	spin_unlock_bh(&q->lock);
}

static void
mt76_tx_cleanup(struct mt76x2_dev *dev, struct mt76_queue *q, bool flush)
{
	spin_lock_bh(&q->lock);
	mt76_queue_cleanup(dev, q, flush, mt76x2_tx_cleanup_entry);
	if (!flush)
		mt76x2_txq_schedule(dev, q);
	spin_unlock_bh(&q->lock);
}

static int
mt76x2_init_tx_queue(struct mt76x2_dev *dev, struct mt76_queue *q,
		   int idx, int n_desc, bool mcu)
{
	int ret;

	q->regs = dev->mt76.regs + MT_TX_RING_BASE + idx * MT_RING_SIZE;
	q->ndesc = n_desc;

	ret = mt76_queue_alloc(dev, q);
	if (ret)
		return ret;

	mt76x2_irq_enable(dev, MT_INT_TX_DONE(idx));

	return 0;
}

static void
mt76x2_process_rx_skb(struct mt76x2_dev *dev, struct mt76_queue *q,
		    struct sk_buff *skb, u32 info)
{
	void *rxwi = skb->data - sizeof(struct mt76x2_rxwi);

	if (q == &dev->mcu.q_rx) {
		u32 *rxfce;

		/* No RXWI header, data starts with payload */
		skb_push(skb, sizeof(struct mt76x2_rxwi));

		rxfce = (u32 *) skb->cb;
		*rxfce = info;

		skb_queue_tail(&dev->mcu.res_q, skb);
		wake_up(&dev->mcu.wait);
		return;
	}

	if (mt76x2_mac_process_rx(dev, skb, rxwi)) {
	    dev_kfree_skb(skb);
	    return;
	}

	mt76x2_rx(dev, skb);
}

static int
mt76x2_process_rx_queue(struct mt76x2_dev *dev, struct mt76_queue *q, int budget)
{
	struct sk_buff *skb;
	unsigned char *data;
	int len;
	int done = 0;

	while (done < budget) {
		u32 info;

		data = mt76_queue_dequeue(dev, q, false, &len, &info);
		if (!data)
			break;

		skb = build_skb(data, 0);
		if (!skb) {
			kfree(data);
			continue;
		}

		skb_reserve(skb, MT_RX_HEADROOM);
		if (skb->tail + len > skb->end) {
			dev_kfree_skb(skb);
			continue;
		}

		__skb_put(skb, len);
		mt76x2_process_rx_skb(dev, q, skb, info);
		done++;
	}

	mt76x2_dma_rx_fill(dev, q);
	return done;
}

static int
mt76x2_init_rx_queue(struct mt76x2_dev *dev, struct mt76_queue *q,
		   int idx, int n_desc, int bufsize)
{
	int ret;

	q->regs = dev->mt76.regs + MT_RX_RING_BASE + idx * MT_RING_SIZE;
	q->ndesc = n_desc;
	q->buf_size = bufsize;

	ret = mt76_queue_alloc(dev, q);
	if (ret)
		return ret;

	mt76x2_irq_enable(dev, MT_INT_RX_DONE(idx));

	return 0;
}

static void
mt76x2_tx_tasklet(unsigned long data)
{
	struct mt76x2_dev *dev = (struct mt76x2_dev *) data;
	int i;

	mt76x2_mac_process_tx_status_fifo(dev);

	for (i = ARRAY_SIZE(dev->q_tx) - 1; i >= 0; i--)
		mt76_tx_cleanup(dev, &dev->q_tx[i], false);

	mt76x2_mac_poll_tx_status(dev, false);
	mt76x2_irq_enable(dev, MT_INT_TX_DONE_ALL);
}

static int
mt76x2_dma_rx_poll(struct napi_struct *napi, int budget)
{
	struct mt76x2_dev *dev = container_of(napi, struct mt76x2_dev, napi);
	int done;

	done = mt76x2_process_rx_queue(dev, &dev->q_rx, budget);

	if (done < budget) {
		napi_complete(napi);
		mt76x2_irq_enable(dev, MT_INT_RX_DONE(0));
	}

	return done;
}

static void
mt76x2_rx_tasklet(unsigned long data)
{
	struct mt76x2_dev *dev = (struct mt76x2_dev *) data;

	mt76x2_process_rx_queue(dev, &dev->mcu.q_rx, dev->mcu.q_rx.ndesc);

	mt76x2_irq_enable(dev, MT_INT_RX_DONE(1));
}

int mt76x2_dma_init(struct mt76x2_dev *dev)
{
	static const u8 wmm_queue_map[] = {
		[IEEE80211_AC_BE] = 0,
		[IEEE80211_AC_BK] = 1,
		[IEEE80211_AC_VI] = 2,
		[IEEE80211_AC_VO] = 3,
	};
	int ret;
	int i;

	mt76_dma_init(&dev->mt76);

	init_waitqueue_head(&dev->mcu.wait);
	skb_queue_head_init(&dev->mcu.res_q);

	init_dummy_netdev(&dev->napi_dev);
	netif_napi_add(&dev->napi_dev, &dev->napi, mt76x2_dma_rx_poll, 64);

	tasklet_init(&dev->tx_tasklet, mt76x2_tx_tasklet, (unsigned long) dev);
	tasklet_init(&dev->rx_tasklet, mt76x2_rx_tasklet, (unsigned long) dev);

	mt76_wr(dev, MT_WPDMA_RST_IDX, ~0);

	for (i = 0; i < ARRAY_SIZE(wmm_queue_map); i++) {
		ret = mt76x2_init_tx_queue(dev, &dev->q_tx[i], wmm_queue_map[i],
					 MT_TX_RING_SIZE, false);
		if (ret)
			return ret;
	}

	ret = mt76x2_init_tx_queue(dev, &dev->q_tx[MT_TXQ_PSD],
				 MT_TX_HW_QUEUE_MGMT, MT_TX_RING_SIZE, false);
	if (ret)
		return ret;

	ret = mt76x2_init_tx_queue(dev, &dev->q_tx[MT_TXQ_MCU],
				 MT_TX_HW_QUEUE_MCU, MT_MCU_RING_SIZE, true);
	if (ret)
		return ret;

	ret = mt76x2_init_rx_queue(dev, &dev->mcu.q_rx, 1, MT_MCU_RING_SIZE,
				 MT_RX_BUF_SIZE);
	if (ret)
		return ret;

	ret = mt76x2_init_rx_queue(dev, &dev->q_rx, 0,
				 MT_RX_RING_SIZE, MT_RX_BUF_SIZE);
	if (ret)
		return ret;

	mt76x2_dma_rx_fill(dev, &dev->q_rx);
	mt76x2_dma_rx_fill(dev, &dev->mcu.q_rx);

	return 0;
}

void mt76x2_dma_cleanup(struct mt76x2_dev *dev)
{
	struct mt76_txwi_cache *t;
	int i;

	tasklet_kill(&dev->tx_tasklet);
	tasklet_kill(&dev->rx_tasklet);
	for (i = 0; i < ARRAY_SIZE(dev->q_tx); i++)
		mt76_tx_cleanup(dev, &dev->q_tx[i], true);
	mt76x2_rx_cleanup(dev, &dev->q_rx);
	mt76x2_rx_cleanup(dev, &dev->mcu.q_rx);

	while ((t = __mt76x2_get_txwi(dev)) != NULL)
		dma_unmap_single(dev->mt76.dev, t->dma_addr, sizeof(t->txwi),
				 DMA_TO_DEVICE);
}
