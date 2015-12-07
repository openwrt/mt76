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

#ifndef __MT76_H
#define __MT76_H

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/skbuff.h>
#include <net/mac80211.h>
#include "util.h"

#define MT_RX_RING_SIZE     128
#define MT_TX_RING_SIZE     128
#define MT_MCU_RING_SIZE    32
#define MT_RX_BUF_SIZE      2048

struct mt76_dev;

struct mt76_bus_ops {
	u32 (*rr)(struct mt76_dev *dev, u32 offset);
	void (*wr)(struct mt76_dev *dev, u32 offset, u32 val);
	u32 (*rmw)(struct mt76_dev *dev, u32 offset, u32 mask, u32 val);
	void (*copy)(struct mt76_dev *dev, u32 offset, const void *data, int len);
};

struct mt76_queue_entry {
	struct sk_buff *skb;
	union {
		void *buf;
		struct mt76_txwi_cache *txwi;
	};
	bool schedule;
};

struct mt76_queue_regs {
	u32 desc_base;
	u32 ring_size;
	u32 cpu_idx;
	u32 dma_idx;
} __packed __aligned(4);

struct mt76_queue {
	struct mt76_queue_regs __iomem *regs;

	spinlock_t lock;
	struct mt76_queue_entry *entry;
	struct mt76_desc *desc;

	struct list_head swq;
	int swq_queued;

	u16 head;
	u16 tail;
	int ndesc;
	int queued;
	int buf_size;

	dma_addr_t desc_dma;
};

struct mt76_queue_ops {
	int (*alloc)(struct mt76_dev *dev, struct mt76_queue *q);

	int (*add_buf)(struct mt76_dev *dev, struct mt76_queue *q,
		       u32 buf0, int len0, u32 buf1, int len1, u32 info);

	void *(*dequeue)(struct mt76_dev *dev, struct mt76_queue *q, bool flush,
			 int *len, u32 *info);

	void (*cleanup)(struct mt76_dev *dev, struct mt76_queue *q, bool flush,
			void (*done)(struct mt76_dev *dev, struct mt76_queue *q,
				     struct mt76_queue_entry *e));

	void (*kick)(struct mt76_dev *dev, struct mt76_queue *q);
};

enum {
	MT76_STATE_INITIALIZED,
	MT76_STATE_RUNNING,
	MT76_SCANNING,
};

struct mt76_dev {
	struct ieee80211_hw *hw;

	const struct mt76_bus_ops *bus;
	void __iomem *regs;
	struct device *dev;

	const struct mt76_queue_ops *queue_ops;

	u32 rev;
	unsigned long state;

	struct ieee80211_supported_band sband_2g;
	struct ieee80211_supported_band sband_5g;
};

enum mt76_phy_type {
	MT_PHY_TYPE_CCK,
	MT_PHY_TYPE_OFDM,
	MT_PHY_TYPE_HT,
	MT_PHY_TYPE_HT_GF,
	MT_PHY_TYPE_VHT,
};

#define mt76_rr(dev, ...)	(dev)->mt76.bus->rr(&((dev)->mt76), __VA_ARGS__)
#define mt76_wr(dev, ...)	(dev)->mt76.bus->wr(&((dev)->mt76), __VA_ARGS__)
#define mt76_rmw(dev, ...)	(dev)->mt76.bus->rmw(&((dev)->mt76), __VA_ARGS__)
#define mt76_wr_copy(dev, ...)	(dev)->mt76.bus->copy(&((dev)->mt76), __VA_ARGS__)

#define mt76_set(dev, offset, val)	mt76_rmw(dev, offset, 0, val)
#define mt76_clear(dev, offset, val)	mt76_rmw(dev, offset, val, 0)

#define mt76_get_field(_dev, _reg, _field)		\
	MT76_GET(_field, mt76_rr(dev, _reg))

#define mt76_rmw_field(_dev, _reg, _field, _val)	\
	mt76_rmw(_dev, _reg, _field, MT76_SET(_field, _val))

#define mt76_hw(dev) (dev)->mt76.hw

bool __mt76_poll(struct mt76_dev *dev, u32 offset, u32 mask, u32 val,
	         int timeout);

#define mt76_poll(dev, ...) __mt76_poll(&((dev)->mt76), __VA_ARGS__)

bool __mt76_poll_msec(struct mt76_dev *dev, u32 offset, u32 mask, u32 val,
		      int timeout);

#define mt76_poll_msec(dev, ...) __mt76_poll_msec(&((dev)->mt76), __VA_ARGS__)

void mt76_mmio_init(struct mt76_dev *dev, void __iomem *regs);

static inline u16 mt76_rev(struct mt76_dev *dev)
{
	return dev->rev & 0xffff;
}

#define mt76xx_rev(dev) mt76_rev(&((dev)->mt76))

#define mt76_queue_alloc(dev, ...)	(dev)->mt76.queue_ops->alloc(&((dev)->mt76), __VA_ARGS__)
#define mt76_queue_add_buf(dev, ...)	(dev)->mt76.queue_ops->add_buf(&((dev)->mt76), __VA_ARGS__)
#define mt76_queue_dequeue(dev, ...)	(dev)->mt76.queue_ops->dequeue(&((dev)->mt76), __VA_ARGS__)
#define mt76_queue_cleanup(dev, ...)	(dev)->mt76.queue_ops->cleanup(&((dev)->mt76), __VA_ARGS__)
#define mt76_queue_kick(dev, ...)	(dev)->mt76.queue_ops->kick(&((dev)->mt76), __VA_ARGS__)

int mt76_register_device(struct mt76_dev *dev, int bands, bool vht,
			 struct ieee80211_rate *rates, int n_rates);

#endif
