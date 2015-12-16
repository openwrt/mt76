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

#define MT7603_WTBL_SIZE	128
#define MT7603_WTBL_RESERVED	(MT7603_WTBL_SIZE - 1)

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

enum mt7603_txq_id {
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

struct mt7603_mcu {
	struct mutex mutex;

	wait_queue_head_t wait;
	struct sk_buff_head res_q;

	struct mt76_queue q_rx;
	u32 msg_seq;

	bool running;
};

struct mt7603_dev {
	struct mt76_dev mt76;

	struct mutex mutex;
	struct cfg80211_chan_def chandef;

	u32 irqmask;
	spinlock_t irq_lock;

	u32 rxfilter;

	u8 rx_chains;
	u8 tx_chains;

	u8 rssi_offset[3];

	struct mt7603_mcu mcu;
	struct mt76_queue q_rx;
	struct mt76_queue q_tx[__MT_TXQ_MAX];

	struct net_device napi_dev;
	struct napi_struct napi;

	struct tasklet_struct tx_tasklet;
	struct tasklet_struct rx_tasklet;
	struct tasklet_struct pre_tbtt_tasklet;
};

extern const struct ieee80211_ops mt7603_ops;

u32 mt7603_reg_map(struct mt7603_dev *dev, u32 addr);

struct mt7603_dev *mt7603_alloc_device(struct device *pdev);
irqreturn_t mt7603_irq_handler(int irq, void *dev_instance);

int mt7603_register_device(struct mt7603_dev *dev);
int mt7603_dma_init(struct mt7603_dev *dev);
void mt7603_dma_cleanup(struct mt7603_dev *dev);
int mt7603_mcu_init(struct mt7603_dev *dev);
int mt7603_tx_queue_mcu(struct mt7603_dev *dev, enum mt7603_txq_id qid,
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
int mt7603_mac_fill_rx(struct mt7603_dev *dev, struct sk_buff *skb);

int mt7603_set_channel(struct mt7603_dev *dev, struct cfg80211_chan_def *def);
int mt7603_mcu_set_channel(struct mt7603_dev *dev);
int mt7603_mcu_reg_read(struct mt7603_dev *dev, u32 reg, u32 *val, bool rf);
int mt7603_mcu_set_eeprom(struct mt7603_dev *dev);
int mt7603_mcu_exit(struct mt7603_dev *dev);

void mt7603_wtbl_init(struct mt7603_dev *dev, int idx, const u8 *addr);
void mt7603_wtbl_clear(struct mt7603_dev *dev, int idx);

#endif
