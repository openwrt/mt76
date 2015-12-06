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

#ifndef __MT76x2_H
#define __MT76x2_H

#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/spinlock.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/bitops.h>
#include <linux/kfifo.h>

#define MT7662_FIRMWARE		"mt7662.bin"
#define MT7662_ROM_PATCH	"mt7662_rom_patch.bin"
#define MT7662_EEPROM_SIZE	512

#define MT_RX_BUF_SIZE		2048
#define MT_RX_HEADROOM		32
#define MT_RX_RING_SIZE		128

#define MT_TX_RING_SIZE		128

#define MT_MCU_RING_SIZE	32

#define MT_MAX_CHAINS		2

#define MT_CALIBRATE_INTERVAL	HZ

#include "mt76.h"
#include "mt76x2_regs.h"
#include "mt76x2_mac.h"

struct mt76x2_queue_entry {
	struct sk_buff *skb;
	union {
		void *buf;
		struct mt76x2_txwi_cache *txwi;
	};
	bool schedule;
};

enum {
	MT76_STATE_INITIALIZED,
	MT76_STATE_RUNNING,
	MT76_SCANNING,
};

enum mt76x2_txq_id {
	MT_TXQ_VO = IEEE80211_AC_VO,
	MT_TXQ_VI = IEEE80211_AC_VI,
	MT_TXQ_BE = IEEE80211_AC_BE,
	MT_TXQ_BK = IEEE80211_AC_BK,
	MT_TXQ_PSD,
	MT_TXQ_MCU,
	__MT_TXQ_MAX
};

struct mt76x2_queue {
	struct mt76x2_queue_regs __iomem *regs;

	spinlock_t lock;
	struct mt76x2_queue_entry *entry;
	struct mt76x2_desc *desc;

	struct list_head swq;
	int swq_queued;

	u16 head;
	u16 tail;
	int ndesc;
	int queued;
	int buf_size;

	dma_addr_t desc_dma;
};

struct mt76x2_mcu {
	struct mutex mutex;

	wait_queue_head_t wait;
	struct sk_buff_head res_q;

	struct mt76x2_queue q_rx;
	u32 msg_seq;
};

struct mt76x2_rx_freq_cal {
	s8 high_gain[MT_MAX_CHAINS];
	s8 rssi_offset[MT_MAX_CHAINS];
	s8 lna_gain;
	u32 mcu_gain;
};

struct mt76x2_calibration {
	struct mt76x2_rx_freq_cal rx;

	u8 agc_gain_init[MT_MAX_CHAINS];
	int avg_rssi[MT_MAX_CHAINS];
	int avg_rssi_all;

	s8 low_gain;

	u8 temp;

	bool init_cal_done;
	bool tssi_cal_done;
	bool tssi_comp_pending;
	bool dpd_cal_done;
	bool channel_cal_done;
};

struct mt76x2_wcid {
	u8 idx;
	u8 hw_key_idx;

	__le16 tx_rate;
	bool tx_rate_set;
	u8 tx_rate_nss;
};

struct mt76x2_hw_cap {
	bool has_2ghz;
	bool has_5ghz;
};

struct mt76x2_rate_power {
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

struct mt76x2_dma_ops {
	int (*queue_skb)(struct mt76x2_dev *dev, struct mt76x2_queue *q,
			 struct sk_buff *skb, struct mt76x2_wcid *wcid,
			 struct ieee80211_sta *sta);
	int (*queue_mcu)(struct mt76x2_dev *dev, enum mt76x2_txq_id qid,
			 struct sk_buff *skb, int cmd, int seq);

	void (*cleanup_idx)(struct mt76x2_dev *dev, struct mt76x2_queue *q, int idx,
			    bool flush, bool *schedule);

	int (*add_rx_buf)(struct mt76x2_dev *dev, struct mt76x2_queue *q,
	                  dma_addr_t addr, int len);
};

struct mt76x2_dev {
	struct mt76_dev mt76;

	struct device *dev;
	u8 macaddr[ETH_ALEN];
	struct mac_address macaddr_list[8];

	struct mutex mutex;

	const struct mt76x2_dma_ops *dma_ops;

	const u16 *beacon_offsets;
	unsigned long wcid_mask[256 / BITS_PER_LONG];

	struct cfg80211_chan_def chandef;
	struct ieee80211_supported_band sband_2g;
	struct ieee80211_supported_band sband_5g;
	int txpower_conf;
	int txpower_cur;

	u8 txdone_seq;
	DECLARE_KFIFO_PTR(txstatus_fifo, struct mt76x2_tx_status);

	struct list_head txwi_cache;
	struct mt76x2_mcu mcu;
	struct mt76x2_queue q_rx;
	struct mt76x2_queue q_tx[__MT_TXQ_MAX];

	struct net_device napi_dev;
	struct napi_struct napi;

	struct tasklet_struct tx_tasklet;
	struct tasklet_struct rx_tasklet;
	struct tasklet_struct pre_tbtt_tasklet;
	struct delayed_work cal_work;
	struct delayed_work mac_work;

	u32 aggr_stats[32];

	struct mt76x2_wcid __rcu *wcid[254 - 8];

	spinlock_t lock;
	spinlock_t irq_lock;
	u32 irqmask;
	unsigned long state;

	struct sk_buff *beacons[8];
	u8 beacon_mask;
	u8 beacon_data_mask;

	u32 rev;
	u32 rxfilter;

	u16 chainmask;

	struct mt76x2_calibration cal;
	struct debugfs_blob_wrapper eeprom;
	struct debugfs_blob_wrapper otp;
	struct mt76x2_hw_cap cap;

	u32 debugfs_reg;

	s8 target_power;
	s8 target_power_delta[2];
	struct mt76x2_rate_power rate_power;

	u8 coverage_class;
	u8 slottime;
};

struct mt76x2_vif {
	u8 idx;

	struct mt76x2_wcid group_wcid;
};

struct mt76x2_sta {
	struct mt76x2_wcid wcid;

	struct mt76x2_tx_status status;
	int n_frames;
	bool sleeping;
};

struct mt76x2_txq {
	struct list_head list;
	struct mt76x2_queue *hwq;

	struct sk_buff_head retry_q;

	u16 agg_ssn;
	bool send_bar;
	bool aggr;
};

struct mt76x2_reg_pair {
	u32 reg;
	u32 value;
};

bool mt76x2_poll(struct mt76x2_dev *dev, u32 offset, u32 mask, u32 val,
	       int timeout);
bool mt76x2_poll_msec(struct mt76x2_dev *dev, u32 offset, u32 mask, u32 val,
		    int timeout);
void mt76_write_reg_pairs(struct mt76x2_dev *dev,
			  const struct mt76x2_reg_pair *data, int len);

static inline bool is_mt7612(struct mt76x2_dev *dev)
{
	return (dev->rev >> 16) == 0x7612;
}

static inline u16 mt76xx_rev(struct mt76x2_dev *dev)
{
	return dev->rev & 0xffff;
}

void mt76x2_set_irq_mask(struct mt76x2_dev *dev, u32 clear, u32 set);

static inline void mt76x2_irq_enable(struct mt76x2_dev *dev, u32 mask)
{
	mt76x2_set_irq_mask(dev, 0, mask);
}

static inline void mt76x2_irq_disable(struct mt76x2_dev *dev, u32 mask)
{
	mt76x2_set_irq_mask(dev, mask, 0);
}

static inline struct ieee80211_txq *
mtxq_to_txq(struct mt76x2_txq *mtxq)
{
	void *ptr = mtxq;
	return container_of(ptr, struct ieee80211_txq, drv_priv);
}

extern const struct ieee80211_ops mt76x2_ops;

struct mt76x2_dev *mt76x2_alloc_device(struct device *pdev);
int mt76x2_register_device(struct mt76x2_dev *dev);
void mt76x2_init_debugfs(struct mt76x2_dev *dev);

irqreturn_t mt76x2_irq_handler(int irq, void *dev_instance);
void mt76x2_phy_power_on(struct mt76x2_dev *dev);
int mt76x2_init_hardware(struct mt76x2_dev *dev);
void mt76x2_stop_hardware(struct mt76x2_dev *dev);
int mt76x2_eeprom_init(struct mt76x2_dev *dev);
int mt76x2_apply_calibration_data(struct mt76x2_dev *dev, int channel);
void mt76x2_set_tx_ackto(struct mt76x2_dev *dev);

int mt76x2_phy_start(struct mt76x2_dev *dev);
int mt76x2_set_channel(struct mt76x2_dev *dev, struct cfg80211_chan_def *chandef);
int mt76x2_phy_set_channel(struct mt76x2_dev *dev,
			 struct cfg80211_chan_def *chandef);
int mt76x2_phy_get_rssi(struct mt76x2_dev *dev, s8 rssi, int chain);
void mt76x2_phy_calibrate(struct work_struct *work);
void mt76x2_phy_set_txpower(struct mt76x2_dev *dev);

int mt76x2_mcu_init(struct mt76x2_dev *dev);
int mt76x2_mcu_set_channel(struct mt76x2_dev *dev, u8 channel, u8 bw, u8 bw_index,
			 bool scan);
int mt76x2_mcu_set_radio_state(struct mt76x2_dev *dev, bool on);
int mt76x2_mcu_load_cr(struct mt76x2_dev *dev, u8 type, u8 temp_level, u8 channel);
int mt76x2_mcu_cleanup(struct mt76x2_dev *dev);

int mt76x2_dma_init(struct mt76x2_dev *dev);
void mt76x2_dma_cleanup(struct mt76x2_dev *dev);

void mt76x2_cleanup(struct mt76x2_dev *dev);
void mt76x2_rx(struct mt76x2_dev *dev, struct sk_buff *skb);

#define mt76x2_tx_queue_skb(dev, ...) dev->dma_ops->queue_skb(dev, __VA_ARGS__)
#define mt76x2_tx_queue_mcu(dev, ...) dev->dma_ops->queue_mcu(dev, __VA_ARGS__)

void mt76x2_tx(struct ieee80211_hw *hw, struct ieee80211_tx_control *control,
	     struct sk_buff *skb);
void mt76x2_tx_complete(struct mt76x2_dev *dev, struct sk_buff *skb);

void mt76x2_kick_queue(struct mt76x2_dev *dev, struct mt76x2_queue *q);

void mt76x2_pre_tbtt_tasklet(unsigned long data);

void mt76x2_txq_init(struct mt76x2_dev *dev, struct ieee80211_txq *txq);
void mt76x2_wake_tx_queue(struct ieee80211_hw *hw, struct ieee80211_txq *txq);
void mt76x2_txq_remove(struct mt76x2_dev *dev, struct ieee80211_txq *txq);
void mt76x2_txq_schedule(struct mt76x2_dev *dev, struct mt76x2_queue *hwq);

void mt76x2_release_buffered_frames(struct ieee80211_hw *hw,
				  struct ieee80211_sta *sta,
				  u16 tids, int nframes,
				  enum ieee80211_frame_release_type reason,
				  bool more_data);

void mt76x2_stop_tx_queues(struct mt76x2_dev *dev, struct ieee80211_sta *sta);

#endif
