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

#ifndef __MT7603_H
#define __MT7603_H

#include "mt76.h"
#include "mt7603_regs.h"

#define MT7603_MAX_INTERFACES	4
#define MT7603_WTBL_SIZE	128
#define MT7603_WTBL_RESERVED	(MT7603_WTBL_SIZE - 1)
#define MT7603_WTBL_STA		(MT7603_WTBL_RESERVED - MT7603_MAX_INTERFACES)

#define MT7603_RATE_RETRY	2

#define MT7603_FIRMWARE_E1	"mt7603_e1.bin"
#define MT7603_FIRMWARE_E2	"mt7603_e2.bin"

#define MT7603_EEPROM_SIZE	1024

enum {
	MT7603_REV_E1 = 0x00,
	MT7603_REV_E2 = 0x10
};

enum mt7603_bw {
	MT_BW_20,
	MT_BW_40,
	MT_BW_80,
};

struct mt7603_mcu {
	struct mutex mutex;

	wait_queue_head_t wait;
	struct sk_buff_head res_q;

	struct mt76_queue q_rx;
	u32 msg_seq;

	bool running;
};

struct mt7603_sta {
	struct mt76_wcid wcid; /* must be first */

	struct ieee80211_tx_rate rates[4];
	int n_rates;

	int pid;

	int ampdu_count;
	int ampdu_acked;
};

struct mt7603_vif {
	u8 idx;

	struct mt7603_sta sta;
};

#define MT7603_CB_DMA_DONE	BIT(0)
#define MT7603_CB_TXS_DONE	BIT(1)

struct mt7603_cb {
	struct list_head list;
	u8 wcid;
	u8 pktid;
	u8 flags;
};

struct mt7603_dev {
	struct mt76_dev mt76; /* must be first */

	struct mutex mutex;
	struct cfg80211_chan_def chandef;

	u32 irqmask;
	spinlock_t irq_lock;

	u32 rxfilter;

	u8 vif_mask;
	unsigned long wcid_mask[MT7603_WTBL_SIZE / BITS_PER_LONG];
	struct mt76_wcid __rcu *wcid[MT7603_WTBL_SIZE];

	spinlock_t status_lock;
	struct list_head status_list;

	struct mt7603_sta global_sta;

	u8 rx_chains;
	u8 tx_chains;

	u8 rssi_offset[3];

	u8 slottime;
	s16 coverage_class;

	struct mt7603_mcu mcu;
	struct mt76_queue q_rx;

	struct tasklet_struct tx_tasklet;
	struct tasklet_struct pre_tbtt_tasklet;
};

extern const struct ieee80211_ops mt7603_ops;

static inline struct mt7603_cb *mt7603_skb_cb(struct sk_buff *skb)
{
	return (void *) IEEE80211_SKB_CB(skb)->rate_driver_data;
}

static inline struct sk_buff *mt7603_cb_skb(struct mt7603_cb *cb)
{
	struct ieee80211_tx_info *info;
	void *ptr = cb;

	BUILD_BUG_ON(sizeof(*cb) > sizeof(info->rate_driver_data));
	info = container_of(ptr, struct ieee80211_tx_info, rate_driver_data);
	ptr = info;
	return container_of(ptr, struct sk_buff, cb);
}

u32 mt7603_reg_map(struct mt7603_dev *dev, u32 addr);

struct mt7603_dev *mt7603_alloc_device(struct device *pdev);
irqreturn_t mt7603_irq_handler(int irq, void *dev_instance);

int mt7603_register_device(struct mt7603_dev *dev);
void mt7603_unregister_device(struct mt7603_dev *dev);
int mt7603_dma_init(struct mt7603_dev *dev);
void mt7603_dma_cleanup(struct mt7603_dev *dev);
int mt7603_mcu_init(struct mt7603_dev *dev);
int mt7603_tx_queue_mcu(struct mt7603_dev *dev, enum mt76_txq_id qid,
			struct sk_buff *skb);

void mt7603_set_irq_mask(struct mt7603_dev *dev, u32 clear, u32 set);

static inline void mt7603_irq_enable(struct mt7603_dev *dev, u32 mask)
{
	mt7603_set_irq_mask(dev, 0, mask);
}

static inline void mt7603_irq_disable(struct mt7603_dev *dev, u32 mask)
{
	mt7603_set_irq_mask(dev, mask, 0);
}

void mt7603_mac_start(struct mt7603_dev *dev);
void mt7603_mac_stop(struct mt7603_dev *dev);
void mt7603_mac_set_timing(struct mt7603_dev *dev);
int mt7603_mac_fill_rx(struct mt7603_dev *dev, struct sk_buff *skb);
void mt7603_mac_add_txs(struct mt7603_dev *dev, void *data);
struct sk_buff *mt7603_mac_status_skb(struct mt7603_dev *dev,
				      struct mt7603_sta *sta, int pktid);
void mt7603_mac_rx_ba_reset(struct mt7603_dev *dev, void *addr, u8 tid);

int mt7603_mcu_set_channel(struct mt7603_dev *dev);
int mt7603_mcu_reg_read(struct mt7603_dev *dev, u32 reg, u32 *val, bool rf);
int mt7603_mcu_set_eeprom(struct mt7603_dev *dev);
int mt7603_mcu_exit(struct mt7603_dev *dev);

void mt7603_wtbl_init(struct mt7603_dev *dev, int idx, const u8 *addr);
void mt7603_wtbl_clear(struct mt7603_dev *dev, int idx);
void mt7603_wtbl_update_cap(struct mt7603_dev *dev, struct ieee80211_sta *sta);
void mt7603_wtbl_set_rates(struct mt7603_dev *dev, struct mt7603_sta *sta);

int mt7603_tx_prepare_skb(struct mt76_dev *mdev, void *txwi_ptr,
			  struct sk_buff *skb, struct mt76_wcid *wcid,
			  struct ieee80211_sta *sta, u32 *tx_info);

void mt7603_tx_complete_skb(struct mt76_dev *mdev, struct mt76_queue *q,
			    struct mt76_queue_entry *e, bool flush);

void mt7603_queue_rx_skb(struct mt76_dev *dev, enum mt76_rxq_id q,
			 struct sk_buff *skb);
void mt7603_rx_poll_complete(struct mt76_dev *mdev, enum mt76_rxq_id q);

#endif
