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

#include <linux/dma-mapping.h>
#include "mt76.h"
#include "dma.h"

static int
mt76_dma_alloc_queue(struct mt76_dev *dev, struct mt76_queue *q)
{
	int size;
	int i;

	spin_lock_init(&q->lock);
	INIT_LIST_HEAD(&q->swq);

	size = q->ndesc * sizeof(struct mt76_desc);
	q->desc = dmam_alloc_coherent(dev->dev, size, &q->desc_dma, GFP_KERNEL);
	if (!q->desc)
		return -ENOMEM;

	size = q->ndesc * sizeof(*q->entry);
	q->entry = devm_kzalloc(dev->dev, size, GFP_KERNEL);
	if (!q->entry)
		return -ENOMEM;

	/* clear descriptors */
	for (i = 0; i < q->ndesc; i++)
		q->desc[i].ctrl = cpu_to_le32(MT_DMA_CTL_DMA_DONE);

	iowrite32(q->desc_dma, &q->regs->desc_base);
	iowrite32(0, &q->regs->cpu_idx);
	iowrite32(0, &q->regs->dma_idx);
	iowrite32(q->ndesc, &q->regs->ring_size);

	return 0;
}

static int
mt76_dma_add_buf(struct mt76_dev *dev, struct mt76_queue *q,
		 u32 buf0, int len0, u32 buf1, int len1, u32 info)
{
	struct mt76_desc *desc;
	u32 ctrl;
	int idx;

	ctrl = MT76_SET(MT_DMA_CTL_SD_LEN0, len0) |
	       MT76_SET(MT_DMA_CTL_SD_LEN1, len1) |
	       (len1 ? MT_DMA_CTL_LAST_SEC1 : MT_DMA_CTL_LAST_SEC0);

	idx = q->head;
	q->head = (q->head + 1) % q->ndesc;

	desc = &q->desc[idx];

	ACCESS_ONCE(desc->buf0) = cpu_to_le32(buf0);
	ACCESS_ONCE(desc->buf1) = cpu_to_le32(buf1);
	ACCESS_ONCE(desc->info) = cpu_to_le32(info);
	ACCESS_ONCE(desc->ctrl) = cpu_to_le32(ctrl);

	q->queued++;

	return idx;
}

static void
mt76_dma_tx_cleanup_idx(struct mt76_dev *dev, struct mt76_queue *q, int idx,
			struct mt76_queue_entry *prev_e)
{
	struct mt76_queue_entry *e = &q->entry[idx];
	struct sk_buff *skb = e->skb;
	dma_addr_t skb_addr;

	if (e->txwi)
		skb_addr = ACCESS_ONCE(q->desc[idx].buf1);
	else
		skb_addr = ACCESS_ONCE(q->desc[idx].buf0);

	dma_unmap_single(dev->dev, skb_addr, skb->len, DMA_TO_DEVICE);

	*prev_e = *e;
	memset(e, 0, sizeof(*e));
}

static void
mt76_dma_tx_cleanup(struct mt76_dev *dev, struct mt76_queue *q, bool flush,
		 void (*done)(struct mt76_dev *dev, struct mt76_queue *q,
			      struct mt76_queue_entry *e))
{
	struct mt76_queue_entry entry;
	int last;

	spin_lock_bh(&q->lock);
	if (flush)
		last = -1;
	else
		last = ioread32(&q->regs->dma_idx);

	while (q->queued && q->tail != last) {
		mt76_dma_tx_cleanup_idx(dev, q, q->tail, &entry);
		if (entry.schedule)
			q->swq_queued--;

		done(dev, q, &entry);
		if (entry.txwi)
			mt76_put_txwi(dev, entry.txwi);

		q->tail = (q->tail + 1) % q->ndesc;
		q->queued--;

		if (q->tail == last)
		    last = ioread32(&q->regs->dma_idx);
	}
	if (!flush)
		mt76_txq_schedule(dev, q);
	spin_unlock_bh(&q->lock);
}

static void *
mt76_dma_get_buf(struct mt76_dev *dev, struct mt76_queue *q, int idx,
		 int *len, u32 *info, bool *more)
{
	struct mt76_queue_entry *e = &q->entry[idx];
	struct mt76_desc *desc = &q->desc[idx];
	dma_addr_t buf_addr;
	void *buf = e->buf;
	int buf_len = SKB_WITH_OVERHEAD(q->buf_size);

	buf_addr = ACCESS_ONCE(desc->buf0);
	if (len) {
		u32 ctl = ACCESS_ONCE(desc->ctrl);
		*len = MT76_GET(MT_DMA_CTL_SD_LEN0, ctl);
		*more = !(ctl & MT_DMA_CTL_LAST_SEC0);
	}

	if (info)
		*info = le32_to_cpu(desc->info);

	dma_unmap_single(dev->dev, buf_addr, buf_len, DMA_FROM_DEVICE);
	e->buf = NULL;

	return buf;
}

static void *
mt76_dma_dequeue(struct mt76_dev *dev, struct mt76_queue *q, bool flush,
		 int *len, u32 *info, bool *more)
{
	int idx = q->tail;

	*more = false;
	if (!q->queued)
		return NULL;

	if (!flush && !(q->desc[idx].ctrl & cpu_to_le32(MT_DMA_CTL_DMA_DONE)))
		return NULL;

	q->tail = (q->tail + 1) % q->ndesc;
	q->queued--;

	return mt76_dma_get_buf(dev, q, idx, len, info, more);
}

static void
mt76_dma_kick_queue(struct mt76_dev *dev, struct mt76_queue *q)
{
	iowrite32(q->head, &q->regs->cpu_idx);
}

static int
mt76_dma_rx_fill(struct mt76_dev *dev, struct mt76_queue *q, bool napi)
{
	dma_addr_t addr;
	void *buf;
	int frames = 0;
	int len = SKB_WITH_OVERHEAD(q->buf_size);
	int offset = q->buf_offset;
	int idx;
	void *(*alloc)(unsigned int fragsz);

	if (napi)
		alloc = napi_alloc_frag;
	else
		alloc = netdev_alloc_frag;

	spin_lock_bh(&q->lock);

	while (q->queued < q->ndesc - 1) {
		buf = alloc(q->buf_size);
		if (!buf)
			break;

		addr = dma_map_single(dev->dev, buf, len, DMA_FROM_DEVICE);
		if (dma_mapping_error(dev->dev, addr)) {
			skb_free_frag(buf);
			break;
		}

		idx = mt76_dma_add_buf(dev, q, addr + offset, len - offset, 0, 0, 0);
		q->entry[idx].buf = buf;
		frames++;
	}

	if (frames)
		mt76_dma_kick_queue(dev, q);

	spin_unlock_bh(&q->lock);

	return frames;
}

static void
mt76_dma_rx_cleanup(struct mt76_dev *dev, struct mt76_queue *q)
{
	void *buf;
	bool more;

	spin_lock_bh(&q->lock);
	do {
		buf = mt76_dma_dequeue(dev, q, true, NULL, NULL, &more);
		if (!buf)
			break;

		skb_free_frag(buf);
	} while (1);
	spin_unlock_bh(&q->lock);
}

static int
mt76_dma_init(struct mt76_dev *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(dev->q_rx); i++)
		mt76_dma_rx_fill(dev, &dev->q_rx[i], false);

	return 0;
}

static void
mt76_add_fragment(struct mt76_dev *dev, struct mt76_queue *q, void *data,
		    int len, bool more)
{
	struct page *page = virt_to_head_page(data);
	int offset = data - page_address(page);
	struct sk_buff *skb = q->rx_head;

	skb_add_rx_frag(skb, skb_shinfo(skb)->nr_frags, page, offset, len,
			q->buf_size);

	if (more)
		return;

	q->rx_head = NULL;
	dev->drv->rx_skb(dev, q, skb);
}

static int
mt76_dma_rx_process(struct mt76_dev *dev, struct mt76_queue *q, int budget)
{
	struct sk_buff *skb;
	unsigned char *data;
	int len;
	int done = 0;
	bool napi = q == &dev->q_rx[MT_RXQ_MAIN];
	bool more;

	while (done < budget) {
		u32 info;

		data = mt76_dma_dequeue(dev, q, false, &len, &info, &more);
		if (!data)
			break;

		if (q->rx_head) {
			mt76_add_fragment(dev, q, data, len, more);
			continue;
		}

		skb = build_skb(data, q->buf_size);
		if (!skb) {
			skb_free_frag(data);
			continue;
		}

		skb_reserve(skb, q->buf_offset);
		if (skb->tail + len > skb->end) {
			dev_kfree_skb(skb);
			continue;
		}

		if (q == &dev->q_rx[MT_RXQ_MCU]) {
			u32 * rxfce = (u32 *) skb->cb;
			*rxfce = info;
		}

		__skb_put(skb, len);
		done++;

		if (more) {
			q->rx_head = skb;
			continue;
		}

		dev->drv->rx_skb(dev, q, skb);
	}

	mt76_dma_rx_fill(dev, q, napi);
	return done;
}

static const struct mt76_queue_ops mt76_dma_ops = {
	.init = mt76_dma_init,
	.alloc = mt76_dma_alloc_queue,
	.add_buf = mt76_dma_add_buf,
	.tx_cleanup = mt76_dma_tx_cleanup,
	.kick = mt76_dma_kick_queue,
	.rx_process = mt76_dma_rx_process,
};

int mt76_dma_attach(struct mt76_dev *dev)
{
	dev->queue_ops = &mt76_dma_ops;
	return 0;
}
EXPORT_SYMBOL_GPL(mt76_dma_attach);

static void
mt76_tx_cleanup_entry(struct mt76_dev *dev, struct mt76_queue *q,
			struct mt76_queue_entry *e)
{
	if (e->txwi)
		ieee80211_free_txskb(dev->hw, e->skb);
	else
		dev_kfree_skb_any(e->skb);
}

void mt76_dma_cleanup(struct mt76_dev *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(dev->q_tx); i++)
		mt76_dma_tx_cleanup(dev, &dev->q_tx[i], true,
				    mt76_tx_cleanup_entry);
	for (i = 0; i < ARRAY_SIZE(dev->q_rx); i++)
		mt76_dma_rx_cleanup(dev, &dev->q_rx[i]);
}
EXPORT_SYMBOL_GPL(mt76_dma_cleanup);
