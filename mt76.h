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

enum mt76_txq_id {
	MT_TXQ_VO = IEEE80211_AC_VO,
	MT_TXQ_VI = IEEE80211_AC_VI,
	MT_TXQ_BE = IEEE80211_AC_BE,
	MT_TXQ_BK = IEEE80211_AC_BK,
	MT_TXQ_PSD,
	MT_TXQ_MCU,
	MT_TXQ_BEACON,
	MT_TXQ_CAB,
	__MT_TXQ_MAX
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

struct mt76_wcid {
	u8 idx;
	u8 hw_key_idx;

	__le16 tx_rate;
	bool tx_rate_set;
	u8 tx_rate_nss;
};

struct mt76_txq {
	struct list_head list;
	struct mt76_queue *hwq;
	struct mt76_wcid *wcid;

	struct sk_buff_head retry_q;

	u16 agg_ssn;
	bool send_bar;
	bool aggr;
};

enum {
	MT76_STATE_INITIALIZED,
	MT76_STATE_RUNNING,
	MT76_SCANNING,
};

struct mt76_hw_cap {
	bool has_2ghz;
	bool has_5ghz;
};

struct mt76_driver_ops {
	int (*tx_queue_skb)(struct mt76_dev *dev, struct mt76_queue *q,
			    struct sk_buff *skb, struct mt76_wcid *wcid,
			    struct ieee80211_sta *sta);
};

struct mt76_dev {
	struct ieee80211_hw *hw;

	const struct mt76_bus_ops *bus;
	const struct mt76_driver_ops *drv;
	void __iomem *regs;
	struct device *dev;

	struct mt76_queue q_tx[__MT_TXQ_MAX];
	const struct mt76_queue_ops *queue_ops;

	u32 rev;
	unsigned long state;

	struct ieee80211_supported_band sband_2g;
	struct ieee80211_supported_band sband_5g;
	struct debugfs_blob_wrapper eeprom;
	struct mt76_hw_cap cap;

	u32 debugfs_reg;
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

#define mt76_tx_queue_skb(dev, ...) (dev)->drv->tx_queue_skb(dev, __VA_ARGS__)

int mt76_register_device(struct mt76_dev *dev, bool vht,
			 struct ieee80211_rate *rates, int n_rates);

struct dentry *mt76_register_debugfs(struct mt76_dev *dev);


#ifdef CONFIG_OF
void mt76_eeprom_override(struct mt76_dev *dev);
#else
static inline void mt76_eeprom_override(struct mt76_dev *dev)
{
}
#endif

int mt76_eeprom_init(struct mt76_dev *dev, int len);

static inline struct ieee80211_txq *
mtxq_to_txq(struct mt76_txq *mtxq)
{
	void *ptr = mtxq;
	return container_of(ptr, struct ieee80211_txq, drv_priv);
}

void mt76_tx(struct mt76_dev *dev, struct ieee80211_sta *sta,
	     struct mt76_wcid *wcid, struct sk_buff *skb);
void mt76_txq_init(struct mt76_dev *dev, struct ieee80211_txq *txq);
void mt76_txq_remove(struct mt76_dev *dev, struct ieee80211_txq *txq);
void mt76_wake_tx_queue(struct ieee80211_hw *hw, struct ieee80211_txq *txq);
void mt76_stop_tx_queues(struct mt76_dev *dev, struct ieee80211_sta *sta);
void mt76_txq_schedule(struct mt76_dev *dev, struct mt76_queue *hwq);
void mt76_release_buffered_frames(struct ieee80211_hw *hw,
				  struct ieee80211_sta *sta,
				  u16 tids, int nframes,
				  enum ieee80211_frame_release_type reason,
				  bool more_data);

#endif
