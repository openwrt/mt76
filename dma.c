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

static int
mt76_dma_dequeue(struct mt76_dev *dev, struct mt76_queue *q, bool flush)
{
	int ret = -1;

	if (!q->queued)
		return -1;

	if (!flush && !(q->desc[q->tail].ctrl & cpu_to_le32(MT_DMA_CTL_DMA_DONE)))
		return -1;

	ret = q->tail;
	q->tail = (q->tail + 1) % q->ndesc;
	q->queued--;

	return ret;
}

static void
mt76_dma_kick_queue(struct mt76_dev *dev, struct mt76_queue *q)
{
	iowrite32(q->head, &q->regs->cpu_idx);
}

static const struct mt76_queue_ops mt76_dma_ops = {
	.alloc = mt76_dma_alloc_queue,
	.add_buf = mt76_dma_add_buf,
	.dequeue = mt76_dma_dequeue,
	.cleanup_idx = mt76_dma_tx_cleanup_idx,
	.kick = mt76_dma_kick_queue,
};

int mt76_dma_init(struct mt76_dev *dev)
{
	dev->queue_ops = &mt76_dma_ops;
	return 0;
}
EXPORT_SYMBOL_GPL(mt76_dma_init);
