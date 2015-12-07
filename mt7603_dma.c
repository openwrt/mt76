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
#include "mt7603_dma.h"

static int
mt7603_dma_add_rx_buf(struct mt7603_dev *dev, struct mt76_queue *q,
		    dma_addr_t addr, int len)
{
	return mt76_queue_add_buf(dev, q, addr, len, 0, 0, 0);
}

static int
mt7603_dma_rx_fill(struct mt7603_dev *dev, struct mt76_queue *q)
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

		idx = mt7603_dma_add_rx_buf(dev, q, addr, len);
		q->entry[idx].buf = buf;
		frames++;
	}

	if (frames)
		mt76_queue_kick(dev, q);

	spin_unlock_bh(&q->lock);

	return frames;
}

int
mt7603_tx_queue_mcu(struct mt7603_dev *dev, enum mt7603_txq_id qid,
		    struct sk_buff *skb)
{
	struct mt76_queue *q = &dev->q_tx[qid];
	dma_addr_t addr;
	int idx;

	addr = dma_map_single(dev->mt76.dev, skb->data, skb->len, DMA_TO_DEVICE);
	if (dma_mapping_error(dev->mt76.dev, addr))
		return -ENOMEM;

	spin_lock_bh(&q->lock);
	idx = mt76_queue_add_buf(dev, q, addr, skb->len, 0, 0, 0);
	q->entry[idx].skb = skb;
	mt76_queue_kick(dev, q);
	spin_unlock_bh(&q->lock);

	return 0;
}

static void
mt7603_tx_cleanup_entry(struct mt76_dev *mdev, struct mt76_queue *q,
			struct mt76_queue_entry *e)
{
	struct mt7603_dev *dev = container_of(mdev, struct mt7603_dev, mt76);

	(void) dev;
	dev_kfree_skb_any(e->skb);

	if (e->schedule)
		q->swq_queued--;
}

static void
mt7603_rx_cleanup(struct mt7603_dev *dev, struct mt76_queue *q)
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
mt76_tx_cleanup(struct mt7603_dev *dev, struct mt76_queue *q, bool flush)
{
	spin_lock_bh(&q->lock);
	mt76_queue_cleanup(dev, q, flush, mt7603_tx_cleanup_entry);
	spin_unlock_bh(&q->lock);
}

static int
mt7603_init_tx_queue(struct mt7603_dev *dev, struct mt76_queue *q,
		   int idx, int n_desc, bool mcu)
{
	int ret;

	q->regs = dev->mt76.regs + MT_TX_RING_BASE + idx * MT_RING_SIZE;
	q->ndesc = n_desc;

	ret = mt76_queue_alloc(dev, q);
	if (ret)
		return ret;

	mt7603_irq_enable(dev, MT_INT_TX_DONE(idx));

	return 0;
}

static void
mt7603_process_rx_skb(struct mt7603_dev *dev, struct mt76_queue *q,
		    struct sk_buff *skb, u32 info)
{
	if (q == &dev->mcu.q_rx) {
		u32 *rxfce;

		rxfce = (u32 *) skb->cb;
		*rxfce = info;

		skb_queue_tail(&dev->mcu.res_q, skb);
		wake_up(&dev->mcu.wait);
		return;
	}

	dev_kfree_skb(skb);
}

static int
mt7603_process_rx_queue(struct mt7603_dev *dev, struct mt76_queue *q, int budget)
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

		if (skb->tail + len > skb->end) {
			dev_kfree_skb(skb);
			continue;
		}

		__skb_put(skb, len);
		mt7603_process_rx_skb(dev, q, skb, info);
		done++;
	}

	mt7603_dma_rx_fill(dev, q);
	return done;
}

static int
mt7603_init_rx_queue(struct mt7603_dev *dev, struct mt76_queue *q,
		   int idx, int n_desc, int bufsize)
{
	int ret;

	q->regs = dev->mt76.regs + MT_RX_RING_BASE + idx * MT_RING_SIZE;
	q->ndesc = n_desc;
	q->buf_size = bufsize;

	ret = mt76_queue_alloc(dev, q);
	if (ret)
		return ret;

	mt7603_irq_enable(dev, MT_INT_RX_DONE(idx));

	return 0;
}

static void
mt7603_tx_tasklet(unsigned long data)
{
	struct mt7603_dev *dev = (struct mt7603_dev *) data;
	int i;

	for (i = ARRAY_SIZE(dev->q_tx) - 1; i >= 0; i--)
		mt76_tx_cleanup(dev, &dev->q_tx[i], false);

	mt7603_irq_enable(dev, MT_INT_TX_DONE_ALL);
}

static int
mt7603_dma_rx_poll(struct napi_struct *napi, int budget)
{
	struct mt7603_dev *dev = container_of(napi, struct mt7603_dev, napi);
	int done;

	done = mt7603_process_rx_queue(dev, &dev->q_rx, budget);

	if (done < budget) {
		napi_complete(napi);
		mt7603_irq_enable(dev, MT_INT_RX_DONE(0));
	}

	return done;
}

static void
mt7603_rx_tasklet(unsigned long data)
{
	struct mt7603_dev *dev = (struct mt7603_dev *) data;

	mt7603_process_rx_queue(dev, &dev->mcu.q_rx, dev->mcu.q_rx.ndesc);
	mt7603_irq_enable(dev, MT_INT_RX_DONE(1));
}

int mt7603_dma_init(struct mt7603_dev *dev)
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
	netif_napi_add(&dev->napi_dev, &dev->napi, mt7603_dma_rx_poll, 64);

	tasklet_init(&dev->tx_tasklet, mt7603_tx_tasklet, (unsigned long) dev);
	tasklet_init(&dev->rx_tasklet, mt7603_rx_tasklet, (unsigned long) dev);

	mt76_wr(dev, MT_WPDMA_RST_IDX, ~0);

	for (i = 0; i < ARRAY_SIZE(wmm_queue_map); i++) {
		ret = mt7603_init_tx_queue(dev, &dev->q_tx[i], wmm_queue_map[i],
					 MT_TX_RING_SIZE, false);
		if (ret)
			return ret;
	}

	ret = mt7603_init_tx_queue(dev, &dev->q_tx[MT_TXQ_PSD],
				 MT_TX_HW_QUEUE_MGMT, MT_TX_RING_SIZE, false);
	if (ret)
		return ret;

	ret = mt7603_init_tx_queue(dev, &dev->q_tx[MT_TXQ_MCU],
				 MT_TX_HW_QUEUE_MCU, MT_MCU_RING_SIZE, true);
	if (ret)
		return ret;

	ret = mt7603_init_rx_queue(dev, &dev->mcu.q_rx, 1, MT_MCU_RING_SIZE,
				 MT_RX_BUF_SIZE);
	if (ret)
		return ret;

	ret = mt7603_init_rx_queue(dev, &dev->q_rx, 0,
				 MT_RX_RING_SIZE, MT_RX_BUF_SIZE);
	if (ret)
		return ret;

	mt7603_dma_rx_fill(dev, &dev->q_rx);
	mt7603_dma_rx_fill(dev, &dev->mcu.q_rx);

	return 0;
}

void mt7603_dma_cleanup(struct mt7603_dev *dev)
{
	int i;

	tasklet_kill(&dev->tx_tasklet);
	tasklet_kill(&dev->rx_tasklet);
	for (i = 0; i < ARRAY_SIZE(dev->q_tx); i++)
		mt76_tx_cleanup(dev, &dev->q_tx[i], true);
	mt7603_rx_cleanup(dev, &dev->q_rx);
	mt7603_rx_cleanup(dev, &dev->mcu.q_rx);
}
