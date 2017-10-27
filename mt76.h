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

#ifndef __MT76_H
#define __MT76_H

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/skbuff.h>
#include <linux/leds.h>
#include <net/mac80211.h>
#include "util.h"

#define MT_RX_RING_SIZE     128
#define MT_TX_RING_SIZE     256
#define MT_MCU_RING_SIZE    32
#define MT_RX_BUF_SIZE      2048

struct mt76_dev;

struct mt76_bus_ops {
	u32 (*rr)(struct mt76_dev *dev, u32 offset);
	void (*wr)(struct mt76_dev *dev, u32 offset, u32 val);
	u32 (*rmw)(struct mt76_dev *dev, u32 offset, u32 mask, u32 val);
	void (*copy)(struct mt76_dev *dev, u32 offset, const void *data,
		     int len);
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

enum mt76_rxq_id {
	MT_RXQ_MAIN,
	MT_RXQ_MCU,
	__MT_RXQ_MAX
};

struct mt76_queue_buf {
	dma_addr_t addr;
	int len;
};

struct mt76_queue_entry {
	union {
		void *buf;
		struct sk_buff *skb;
	};
	struct mt76_txwi_cache *txwi;
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

	u8 buf_offset;
	u8 hw_idx;

	dma_addr_t desc_dma;
	struct sk_buff *rx_head;
};

struct mt76_queue_ops {
	int (*init)(struct mt76_dev *dev);

	int (*alloc)(struct mt76_dev *dev, struct mt76_queue *q);

	int (*add_buf)(struct mt76_dev *dev, struct mt76_queue *q,
		       struct mt76_queue_buf *buf, int nbufs, u32 info,
		       struct sk_buff *skb, void *txwi);

	void *(*dequeue)(struct mt76_dev *dev, struct mt76_queue *q, bool flush,
			 int *len, u32 *info, bool *more);

	void (*rx_reset)(struct mt76_dev *dev, enum mt76_rxq_id qid);

	void (*tx_cleanup)(struct mt76_dev *dev, enum mt76_txq_id qid,
			   bool flush);

	void (*kick)(struct mt76_dev *dev, struct mt76_queue *q);
};

struct mt76_wcid {
	u8 idx;
	u8 hw_key_idx;

	__le16 tx_rate;
	bool tx_rate_set;
	u8 tx_rate_nss;
	s8 max_txpwr_adj;
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

struct mt76_txwi_cache {
	u32 txwi[8];
	dma_addr_t dma_addr;
	struct list_head list;
};

enum {
	MT76_STATE_INITIALIZED,
	MT76_STATE_RUNNING,
	MT76_SCANNING,
	MT76_RESET,
};

struct mt76_hw_cap {
	bool has_2ghz;
	bool has_5ghz;
};

struct mt76_driver_ops {
	u16 txwi_size;

	void (*update_survey)(struct mt76_dev *dev);

	int (*tx_prepare_skb)(struct mt76_dev *dev, void *txwi_ptr,
			      struct sk_buff *skb, struct mt76_queue *q,
			      struct mt76_wcid *wcid,
			      struct ieee80211_sta *sta, u32 *tx_info);

	void (*tx_complete_skb)(struct mt76_dev *dev, struct mt76_queue *q,
				struct mt76_queue_entry *e, bool flush);

	void (*rx_skb)(struct mt76_dev *dev, enum mt76_rxq_id q,
		       struct sk_buff *skb);

	void (*rx_poll_complete)(struct mt76_dev *dev, enum mt76_rxq_id q);
};

struct mt76_channel_state {
	u64 cc_active;
	u64 cc_busy;
};

struct mt76_sband {
	struct ieee80211_supported_band sband;
	struct mt76_channel_state *chan;
};

struct mt76_dev {
	struct ieee80211_hw *hw;
	struct cfg80211_chan_def chandef;
	struct ieee80211_channel *main_chan;

	spinlock_t lock;
	spinlock_t cc_lock;
	const struct mt76_bus_ops *bus;
	const struct mt76_driver_ops *drv;
	void __iomem *regs;
	struct device *dev;

	struct net_device napi_dev;
	struct napi_struct napi[__MT_RXQ_MAX];
	struct sk_buff_head rx_skb[__MT_RXQ_MAX];

	struct list_head txwi_cache;
	struct mt76_queue q_tx[__MT_TXQ_MAX];
	struct mt76_queue q_rx[__MT_RXQ_MAX];
	const struct mt76_queue_ops *queue_ops;

	u8 macaddr[ETH_ALEN];
	u32 rev;
	unsigned long state;

	struct mt76_sband sband_2g;
	struct mt76_sband sband_5g;
	struct debugfs_blob_wrapper eeprom;
	struct debugfs_blob_wrapper otp;
	struct mt76_hw_cap cap;

	u32 debugfs_reg;

	struct led_classdev led_cdev;
	char led_name[32];
	bool led_al;
	u8 led_pin;
};

enum mt76_phy_type {
	MT_PHY_TYPE_CCK,
	MT_PHY_TYPE_OFDM,
	MT_PHY_TYPE_HT,
	MT_PHY_TYPE_HT_GF,
	MT_PHY_TYPE_VHT,
};

struct mt76_rate_power {
	union {
		struct {
			s8 cck[4];
			s8 ofdm[8];
			s8 ht[16];
			s8 vht[10];
		};
		s8 all[38];
	};
};

#define mt76_rr(dev, ...)	(dev)->mt76.bus->rr(&((dev)->mt76), __VA_ARGS__)
#define mt76_wr(dev, ...)	(dev)->mt76.bus->wr(&((dev)->mt76), __VA_ARGS__)
#define mt76_rmw(dev, ...)	(dev)->mt76.bus->rmw(&((dev)->mt76), __VA_ARGS__)
#define mt76_wr_copy(dev, ...)	(dev)->mt76.bus->copy(&((dev)->mt76), __VA_ARGS__)

#define mt76_set(dev, offset, val)	mt76_rmw(dev, offset, 0, val)
#define mt76_clear(dev, offset, val)	mt76_rmw(dev, offset, val, 0)

#define mt76_get_field(_dev, _reg, _field)		\
	FIELD_GET(_field, mt76_rr(dev, _reg))

#define mt76_rmw_field(_dev, _reg, _field, _val)	\
	mt76_rmw(_dev, _reg, _field, FIELD_PREP(_field, _val))

#define mt76_hw(dev) (dev)->mt76.hw

bool __mt76_poll(struct mt76_dev *dev, u32 offset, u32 mask, u32 val,
		 int timeout);

#define mt76_poll(dev, ...) __mt76_poll(&((dev)->mt76), __VA_ARGS__)

bool __mt76_poll_msec(struct mt76_dev *dev, u32 offset, u32 mask, u32 val,
		      int timeout);

#define mt76_poll_msec(dev, ...) __mt76_poll_msec(&((dev)->mt76), __VA_ARGS__)

void mt76_mmio_init(struct mt76_dev *dev, void __iomem *regs);

static inline u16 mt76_chip(struct mt76_dev *dev)
{
	return dev->rev >> 16;
}

static inline u16 mt76_rev(struct mt76_dev *dev)
{
	return dev->rev & 0xffff;
}

#define mt76xx_chip(dev) mt76_chip(&((dev)->mt76))
#define mt76xx_rev(dev) mt76_rev(&((dev)->mt76))

#define mt76_init_queues(dev)		(dev)->mt76.queue_ops->init(&((dev)->mt76))
#define mt76_queue_alloc(dev, ...)	(dev)->mt76.queue_ops->alloc(&((dev)->mt76), __VA_ARGS__)
#define mt76_queue_add_buf(dev, ...)	(dev)->mt76.queue_ops->add_buf(&((dev)->mt76), __VA_ARGS__)
#define mt76_queue_rx_reset(dev, ...)	(dev)->mt76.queue_ops->rx_reset(&((dev)->mt76), __VA_ARGS__)
#define mt76_queue_tx_cleanup(dev, ...)	(dev)->mt76.queue_ops->tx_cleanup(&((dev)->mt76), __VA_ARGS__)
#define mt76_queue_kick(dev, ...)	(dev)->mt76.queue_ops->kick(&((dev)->mt76), __VA_ARGS__)

static inline struct mt76_channel_state *
mt76_channel_state(struct mt76_dev *dev, struct ieee80211_channel *c)
{
	struct mt76_sband *msband;
	int idx;

	if (c->band == NL80211_BAND_2GHZ)
		msband = &dev->sband_2g;
	else
		msband = &dev->sband_5g;

	idx = c - &msband->sband.channels[0];
	return &msband->chan[idx];
}

int mt76_register_device(struct mt76_dev *dev, bool vht,
			 struct ieee80211_rate *rates, int n_rates);
void mt76_unregister_device(struct mt76_dev *dev);

struct dentry *mt76_register_debugfs(struct mt76_dev *dev);

int mt76_eeprom_init(struct mt76_dev *dev, int len);
void mt76_eeprom_override(struct mt76_dev *dev);

static inline struct ieee80211_txq *
mtxq_to_txq(struct mt76_txq *mtxq)
{
	void *ptr = mtxq;

	return container_of(ptr, struct ieee80211_txq, drv_priv);
}

int mt76_tx_queue_skb(struct mt76_dev *dev, struct mt76_queue *q,
		      struct sk_buff *skb, struct mt76_wcid *wcid,
		      struct ieee80211_sta *sta);

void mt76_rx(struct mt76_dev *dev, enum mt76_rxq_id q, struct sk_buff *skb);
void mt76_tx(struct mt76_dev *dev, struct ieee80211_sta *sta,
	     struct mt76_wcid *wcid, struct sk_buff *skb);
void mt76_txq_init(struct mt76_dev *dev, struct ieee80211_txq *txq);
void mt76_txq_remove(struct mt76_dev *dev, struct ieee80211_txq *txq);
void mt76_wake_tx_queue(struct ieee80211_hw *hw, struct ieee80211_txq *txq);
void mt76_stop_tx_queues(struct mt76_dev *dev, struct ieee80211_sta *sta,
			 bool send_bar);
void mt76_txq_schedule(struct mt76_dev *dev, struct mt76_queue *hwq);
void mt76_txq_schedule_all(struct mt76_dev *dev);
void mt76_release_buffered_frames(struct ieee80211_hw *hw,
				  struct ieee80211_sta *sta,
				  u16 tids, int nframes,
				  enum ieee80211_frame_release_type reason,
				  bool more_data);
void mt76_set_channel(struct mt76_dev *dev);
int mt76_get_survey(struct ieee80211_hw *hw, int idx,
		    struct survey_info *survey);

/* internal */
void mt76_tx_free(struct mt76_dev *dev);
void mt76_put_txwi(struct mt76_dev *dev, struct mt76_txwi_cache *t);
void mt76_rx_complete(struct mt76_dev *dev, enum mt76_rxq_id q);

#endif
