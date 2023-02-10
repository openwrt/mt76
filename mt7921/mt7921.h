/* SPDX-License-Identifier: ISC */
/* Copyright (C) 2020 MediaTek Inc. */

#ifndef __MT7921_H
#define __MT7921_H

#include <linux/interrupt.h>
#include <linux/ktime.h>
#include "../mt76_connac_mcu.h"
#include "regs.h"
#include "acpi_sar.h"

#define MT7921_MAX_INTERFACES		4
#define MT7921_WTBL_SIZE		20
#define MT7921_WTBL_RESERVED		(MT7921_WTBL_SIZE - 1)
#define MT7921_WTBL_STA			(MT7921_WTBL_RESERVED - \
					 MT7921_MAX_INTERFACES)

#define MT7921_PM_TIMEOUT		(HZ / 12)
#define MT7921_HW_SCAN_TIMEOUT		(HZ / 10)
#define MT7921_WATCHDOG_TIME		(HZ / 4)
#define MT7921_RESET_TIMEOUT		(30 * HZ)

#define MT7921_TX_RING_SIZE		2048
#define MT7921_TX_MCU_RING_SIZE		256
#define MT7921_TX_FWDL_RING_SIZE	128

#define MT7921_RX_RING_SIZE		1536
#define MT7921_RX_MCU_RING_SIZE		512

#define MT7921_DRV_OWN_RETRY_COUNT	10
#define MT7921_MCU_INIT_RETRY_COUNT	10
#define MT7921_WFSYS_INIT_RETRY_COUNT	2

#define MT7921_FW_TAG_FEATURE		4
#define MT7921_FW_CAP_CNM		BIT(7)

#define MT7921_FIRMWARE_WM		"mediatek/WIFI_RAM_CODE_MT7961_1.bin"
#define MT7921_ROM_PATCH		"mediatek/WIFI_MT7961_patch_mcu_1_2_hdr.bin"

#define MT7922_FIRMWARE_WM		"mediatek/WIFI_RAM_CODE_MT7922_1.bin"
#define MT7922_ROM_PATCH		"mediatek/WIFI_MT7922_patch_mcu_1_1_hdr.bin"

#define MT7921_EEPROM_SIZE		3584
#define MT7921_TOKEN_SIZE		8192

#define MT7921_EEPROM_BLOCK_SIZE	16

#define MT7921_CFEND_RATE_DEFAULT	0x49	/* OFDM 24M */
#define MT7921_CFEND_RATE_11B		0x03	/* 11B LP, 11M */

#define MT7921_SKU_RATE_NUM		161
#define MT7921_SKU_MAX_DELTA_IDX	MT7921_SKU_RATE_NUM
#define MT7921_SKU_TABLE_SIZE		(MT7921_SKU_RATE_NUM + 1)

#define MT7921_SDIO_HDR_TX_BYTES	GENMASK(15, 0)
#define MT7921_SDIO_HDR_PKT_TYPE	GENMASK(17, 16)

#define MCU_UNI_EVENT_ROC  0x27

enum {
	UNI_ROC_ACQUIRE,
	UNI_ROC_ABORT,
	UNI_ROC_NUM
};

enum mt7921_roc_req {
	MT7921_ROC_REQ_JOIN,
	MT7921_ROC_REQ_ROC,
	MT7921_ROC_REQ_NUM
};

enum {
	UNI_EVENT_ROC_GRANT = 0,
	UNI_EVENT_ROC_TAG_NUM
};

struct mt7921_realease_info {
	__le16 len;
	u8 pad_len;
	u8 tag;
} __packed;

struct mt7921_fw_features {
	u8 segment;
	u8 data;
	u8 rsv[14];
} __packed;

struct mt7921_roc_grant_tlv {
	__le16 tag;
	__le16 len;
	u8 bss_idx;
	u8 tokenid;
	u8 status;
	u8 primarychannel;
	u8 rfsco;
	u8 rfband;
	u8 channelwidth;
	u8 centerfreqseg1;
	u8 centerfreqseg2;
	u8 reqtype;
	u8 dbdcband;
	u8 rsv[1];
	__le32 max_interval;
} __packed;

enum mt7921_sdio_pkt_type {
	MT7921_SDIO_TXD,
	MT7921_SDIO_DATA,
	MT7921_SDIO_CMD,
	MT7921_SDIO_FWDL,
};

struct mt7921_sdio_intr {
	u32 isr;
	struct {
		u32 wtqcr[16];
	} tx;
	struct {
		u16 num[2];
		u16 len0[16];
		u16 len1[128];
	} rx;
	u32 rec_mb[2];
} __packed;

#define to_rssi(field, rxv)		((FIELD_GET(field, rxv) - 220) / 2)
#define to_rcpi(rssi)			(2 * (rssi) + 220)

struct mt7921_vif;
struct mt7921_sta;

enum mt7921_txq_id {
	MT7921_TXQ_BAND0,
	MT7921_TXQ_BAND1,
	MT7921_TXQ_FWDL = 16,
	MT7921_TXQ_MCU_WM,
};

enum mt7921_rxq_id {
	MT7921_RXQ_BAND0 = 0,
	MT7921_RXQ_BAND1,
	MT7921_RXQ_MCU_WM = 0,
};

DECLARE_EWMA(avg_signal, 10, 8)

struct mt7921_sta {
	struct mt76_wcid wcid; /* must be first */

	struct mt7921_vif *vif;

	struct list_head poll_list;
	u32 airtime_ac[8];

	int ack_signal;
	struct ewma_avg_signal avg_ack_signal;

	unsigned long last_txs;
	unsigned long ampdu_state;

	struct mt76_connac_sta_key_conf bip;
};

DECLARE_EWMA(rssi, 10, 8);

struct mt7921_vif {
	struct mt76_vif mt76; /* must be first */

	struct mt7921_sta sta;
	struct mt7921_sta *wep_sta;

	struct mt7921_phy *phy;

	struct ewma_rssi rssi;

	struct ieee80211_tx_queue_params queue_params[IEEE80211_NUM_ACS];
	struct ieee80211_chanctx_conf *ctx;
};

struct mib_stats {
	u32 ack_fail_cnt;
	u32 fcs_err_cnt;
	u32 rts_cnt;
	u32 rts_retries_cnt;
	u32 ba_miss_cnt;

	u32 tx_bf_ibf_ppdu_cnt;
	u32 tx_bf_ebf_ppdu_cnt;
	u32 tx_bf_rx_fb_all_cnt;
	u32 tx_bf_rx_fb_he_cnt;
	u32 tx_bf_rx_fb_vht_cnt;
	u32 tx_bf_rx_fb_ht_cnt;

	u32 tx_ampdu_cnt;
	u32 tx_mpdu_attempts_cnt;
	u32 tx_mpdu_success_cnt;
	u32 tx_pkt_ebf_cnt;
	u32 tx_pkt_ibf_cnt;

	u32 rx_mpdu_cnt;
	u32 rx_ampdu_cnt;
	u32 rx_ampdu_bytes_cnt;
	u32 rx_ba_cnt;

	u32 tx_amsdu[8];
	u32 tx_amsdu_cnt;
};

enum {
	MT7921_CLC_POWER,
	MT7921_CLC_CHAN,
	MT7921_CLC_MAX_NUM,
};

struct mt7921_clc_rule {
	u8 alpha2[2];
	u8 type[2];
	__le16 len;
	u8 data[];
} __packed;

struct mt7921_clc {
	__le32 len;
	u8 idx;
	u8 ver;
	u8 nr_country;
	u8 type;
	u8 rsv[8];
	u8 data[];
} __packed;

struct mt7921_phy {
	struct mt76_phy *mt76;
	struct mt7921_dev *dev;

	struct ieee80211_sband_iftype_data iftype[NUM_NL80211_BANDS][NUM_NL80211_IFTYPES];

	u64 omac_mask;

	u16 noise;

	s16 coverage_class;
	u8 slottime;

	u32 rx_ampdu_ts;
	u32 ampdu_ref;

	struct mib_stats mib;

	u8 sta_work_count;

	struct sk_buff_head scan_event_list;
	struct delayed_work scan_work;
#ifdef CONFIG_ACPI
	struct mt7921_acpi_sar *acpisar;
#endif

	struct mt7921_clc *clc[MT7921_CLC_MAX_NUM];

	struct work_struct roc_work;
	struct timer_list roc_timer;
	wait_queue_head_t roc_wait;
	u8 roc_token_id;
	bool roc_grant;
};

#define mt7921_init_reset(dev)		((dev)->hif_ops->init_reset(dev))
#define mt7921_dev_reset(dev)		((dev)->hif_ops->reset(dev))
#define mt7921_mcu_init(dev)		((dev)->hif_ops->mcu_init(dev))
#define __mt7921_mcu_drv_pmctrl(dev)	((dev)->hif_ops->drv_own(dev))
#define	__mt7921_mcu_fw_pmctrl(dev)	((dev)->hif_ops->fw_own(dev))
struct mt7921_hif_ops {
	int (*init_reset)(struct mt7921_dev *dev);
	int (*reset)(struct mt7921_dev *dev);
	int (*mcu_init)(struct mt7921_dev *dev);
	int (*drv_own)(struct mt7921_dev *dev);
	int (*fw_own)(struct mt7921_dev *dev);
};

struct mt7921_dev {
	union { /* must be first */
		struct mt76_dev mt76;
		struct mt76_phy mphy;
	};

	const struct mt76_bus_ops *bus_ops;
	struct mt7921_phy phy;
	struct tasklet_struct irq_tasklet;

	struct work_struct reset_work;
	bool hw_full_reset:1;
	bool hw_init_done:1;
	bool fw_assert:1;

	struct list_head sta_poll_list;
	spinlock_t sta_poll_lock;

	struct work_struct init_work;

	u8 fw_debug;
	u8 fw_features;

	struct mt76_connac_pm pm;
	struct mt76_connac_coredump coredump;
	const struct mt7921_hif_ops *hif_ops;

	struct work_struct ipv6_ns_work;
	/* IPv6 addresses for WoWLAN */
	struct sk_buff_head ipv6_ns_list;

	enum environment_cap country_ie_env;
};

enum {
	TXPWR_USER,
	TXPWR_EEPROM,
	TXPWR_MAC,
	TXPWR_MAX_NUM,
};

struct mt7921_txpwr {
	u8 ch;
	u8 rsv[3];
	struct {
		u8 ch;
		u8 cck[4];
		u8 ofdm[8];
		u8 ht20[8];
		u8 ht40[9];
		u8 vht20[12];
		u8 vht40[12];
		u8 vht80[12];
		u8 vht160[12];
		u8 he26[12];
		u8 he52[12];
		u8 he106[12];
		u8 he242[12];
		u8 he484[12];
		u8 he996[12];
		u8 he996x2[12];
	} data[TXPWR_MAX_NUM];
};

static inline struct mt7921_phy *
mt7921_hw_phy(struct ieee80211_hw *hw)
{
	struct mt76_phy *phy = hw->priv;

	return phy->priv;
}

static inline struct mt7921_dev *
mt7921_hw_dev(struct ieee80211_hw *hw)
{
	struct mt76_phy *phy = hw->priv;

	return container_of(phy->dev, struct mt7921_dev, mt76);
}

#define mt7921_mutex_acquire(dev)	\
	mt76_connac_mutex_acquire(&(dev)->mt76, &(dev)->pm)
#define mt7921_mutex_release(dev)	\
	mt76_connac_mutex_release(&(dev)->mt76, &(dev)->pm)

extern const struct ieee80211_ops mt7921_ops;

u32 mt7921_reg_map(struct mt7921_dev *dev, u32 addr);

int __mt7921_start(struct mt7921_phy *phy);
int mt7921_register_device(struct mt7921_dev *dev);
void mt7921_unregister_device(struct mt7921_dev *dev);
int mt7921_dma_init(struct mt7921_dev *dev);
int mt7921_wpdma_reset(struct mt7921_dev *dev, bool force);
int mt7921_wpdma_reinit_cond(struct mt7921_dev *dev);
void mt7921_dma_cleanup(struct mt7921_dev *dev);
int mt7921_run_firmware(struct mt7921_dev *dev);
int mt7921_mcu_set_bss_pm(struct mt7921_dev *dev, struct ieee80211_vif *vif,
			  bool enable);
int mt7921_mcu_sta_update(struct mt7921_dev *dev, struct ieee80211_sta *sta,
			  struct ieee80211_vif *vif, bool enable,
			  enum mt76_sta_info_state state);
int mt7921_mcu_set_chan_info(struct mt7921_phy *phy, int cmd);
int mt7921_mcu_set_tx(struct mt7921_dev *dev, struct ieee80211_vif *vif);
int mt7921_mcu_set_eeprom(struct mt7921_dev *dev);
int mt7921_mcu_get_rx_rate(struct mt7921_phy *phy, struct ieee80211_vif *vif,
			   struct ieee80211_sta *sta, struct rate_info *rate);
int mt7921_mcu_fw_log_2_host(struct mt7921_dev *dev, u8 ctrl);
void mt7921_mcu_rx_event(struct mt7921_dev *dev, struct sk_buff *skb);

static inline void mt7921_irq_enable(struct mt7921_dev *dev, u32 mask)
{
	mt76_set_irq_mask(&dev->mt76, 0, 0, mask);

	tasklet_schedule(&dev->irq_tasklet);
}

static inline u32
mt7921_reg_map_l1(struct mt7921_dev *dev, u32 addr)
{
	u32 offset = FIELD_GET(MT_HIF_REMAP_L1_OFFSET, addr);
	u32 base = FIELD_GET(MT_HIF_REMAP_L1_BASE, addr);

	mt76_rmw_field(dev, MT_HIF_REMAP_L1, MT_HIF_REMAP_L1_MASK, base);
	/* use read to push write */
	mt76_rr(dev, MT_HIF_REMAP_L1);

	return MT_HIF_REMAP_BASE_L1 + offset;
}

static inline u32
mt7921_l1_rr(struct mt7921_dev *dev, u32 addr)
{
	return mt76_rr(dev, mt7921_reg_map_l1(dev, addr));
}

static inline void
mt7921_l1_wr(struct mt7921_dev *dev, u32 addr, u32 val)
{
	mt76_wr(dev, mt7921_reg_map_l1(dev, addr), val);
}

static inline u32
mt7921_l1_rmw(struct mt7921_dev *dev, u32 addr, u32 mask, u32 val)
{
	val |= mt7921_l1_rr(dev, addr) & ~mask;
	mt7921_l1_wr(dev, addr, val);

	return val;
}

#define mt7921_l1_set(dev, addr, val)	mt7921_l1_rmw(dev, addr, 0, val)
#define mt7921_l1_clear(dev, addr, val)	mt7921_l1_rmw(dev, addr, val, 0)

static inline bool mt7921_dma_need_reinit(struct mt7921_dev *dev)
{
	return !mt76_get_field(dev, MT_WFDMA_DUMMY_CR, MT_WFDMA_NEED_REINIT);
}

static inline void
mt7921_skb_add_usb_sdio_hdr(struct mt7921_dev *dev, struct sk_buff *skb,
			    int type)
{
	u32 hdr, len;

	len = mt76_is_usb(&dev->mt76) ? skb->len : skb->len + sizeof(hdr);
	hdr = FIELD_PREP(MT7921_SDIO_HDR_TX_BYTES, len) |
	      FIELD_PREP(MT7921_SDIO_HDR_PKT_TYPE, type);

	put_unaligned_le32(hdr, skb_push(skb, sizeof(hdr)));
}

void mt7921_stop(struct ieee80211_hw *hw);
int mt7921_mac_init(struct mt7921_dev *dev);
bool mt7921_mac_wtbl_update(struct mt7921_dev *dev, int idx, u32 mask);
void mt7921_mac_reset_counters(struct mt7921_phy *phy);
void mt7921_mac_set_timing(struct mt7921_phy *phy);
int mt7921_mac_sta_add(struct mt76_dev *mdev, struct ieee80211_vif *vif,
		       struct ieee80211_sta *sta);
void mt7921_mac_sta_assoc(struct mt76_dev *mdev, struct ieee80211_vif *vif,
			  struct ieee80211_sta *sta);
void mt7921_mac_sta_remove(struct mt76_dev *mdev, struct ieee80211_vif *vif,
			   struct ieee80211_sta *sta);
void mt7921_mac_work(struct work_struct *work);
void mt7921_mac_reset_work(struct work_struct *work);
void mt7921_mac_update_mib_stats(struct mt7921_phy *phy);
void mt7921_reset(struct mt76_dev *mdev);
int mt7921e_tx_prepare_skb(struct mt76_dev *mdev, void *txwi_ptr,
			   enum mt76_txq_id qid, struct mt76_wcid *wcid,
			   struct ieee80211_sta *sta,
			   struct mt76_tx_info *tx_info);

void mt7921_tx_worker(struct mt76_worker *w);
void mt7921_tx_token_put(struct mt7921_dev *dev);
bool mt7921_rx_check(struct mt76_dev *mdev, void *data, int len);
void mt7921_queue_rx_skb(struct mt76_dev *mdev, enum mt76_rxq_id q,
			 struct sk_buff *skb, u32 *info);
void mt7921_sta_ps(struct mt76_dev *mdev, struct ieee80211_sta *sta, bool ps);
void mt7921_stats_work(struct work_struct *work);
void mt7921_set_stream_he_caps(struct mt7921_phy *phy);
void mt7921_update_channel(struct mt76_phy *mphy);
int mt7921_init_debugfs(struct mt7921_dev *dev);

int mt7921_mcu_set_beacon_filter(struct mt7921_dev *dev,
				 struct ieee80211_vif *vif,
				 bool enable);
int mt7921_mcu_uni_tx_ba(struct mt7921_dev *dev,
			 struct ieee80211_ampdu_params *params,
			 bool enable);
int mt7921_mcu_uni_rx_ba(struct mt7921_dev *dev,
			 struct ieee80211_ampdu_params *params,
			 bool enable);
void mt7921_scan_work(struct work_struct *work);
void mt7921_roc_work(struct work_struct *work);
void mt7921_roc_timer(struct timer_list *timer);
int mt7921_mcu_uni_bss_ps(struct mt7921_dev *dev, struct ieee80211_vif *vif);
int mt7921_mcu_drv_pmctrl(struct mt7921_dev *dev);
int mt7921_mcu_fw_pmctrl(struct mt7921_dev *dev);
void mt7921_pm_wake_work(struct work_struct *work);
void mt7921_pm_power_save_work(struct work_struct *work);
void mt7921_coredump_work(struct work_struct *work);
int mt7921_wfsys_reset(struct mt7921_dev *dev);
int mt7921_get_txpwr_info(struct mt7921_dev *dev, struct mt7921_txpwr *txpwr);
int mt7921_testmode_cmd(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
			void *data, int len);
int mt7921_testmode_dump(struct ieee80211_hw *hw, struct sk_buff *msg,
			 struct netlink_callback *cb, void *data, int len);
void mt7921_txwi_free(struct mt7921_dev *dev, struct mt76_txwi_cache *t,
		      struct ieee80211_sta *sta, bool clear_status,
		      struct list_head *free_list);
void mt7921_mac_sta_poll(struct mt7921_dev *dev);
int mt7921_mcu_parse_response(struct mt76_dev *mdev, int cmd,
			      struct sk_buff *skb, int seq);

int mt7921e_driver_own(struct mt7921_dev *dev);
int mt7921e_mac_reset(struct mt7921_dev *dev);
int mt7921e_mcu_init(struct mt7921_dev *dev);
int mt7921s_wfsys_reset(struct mt7921_dev *dev);
int mt7921s_mac_reset(struct mt7921_dev *dev);
int mt7921s_init_reset(struct mt7921_dev *dev);
int __mt7921e_mcu_drv_pmctrl(struct mt7921_dev *dev);
int mt7921e_mcu_drv_pmctrl(struct mt7921_dev *dev);
int mt7921e_mcu_fw_pmctrl(struct mt7921_dev *dev);

int mt7921s_mcu_init(struct mt7921_dev *dev);
int mt7921s_mcu_drv_pmctrl(struct mt7921_dev *dev);
int mt7921s_mcu_fw_pmctrl(struct mt7921_dev *dev);
void mt7921_mac_add_txs(struct mt7921_dev *dev, void *data);
void mt7921_set_runtime_pm(struct mt7921_dev *dev);
void mt7921_mcu_set_suspend_iter(void *priv, u8 *mac,
				 struct ieee80211_vif *vif);
void mt7921_set_ipv6_ns_work(struct work_struct *work);

int mt7921_mcu_set_sniffer(struct mt7921_dev *dev, struct ieee80211_vif *vif,
			   bool enable);
int mt7921_mcu_config_sniffer(struct mt7921_vif *vif,
			      struct ieee80211_chanctx_conf *ctx);

int mt7921_usb_sdio_tx_prepare_skb(struct mt76_dev *mdev, void *txwi_ptr,
				   enum mt76_txq_id qid, struct mt76_wcid *wcid,
				   struct ieee80211_sta *sta,
				   struct mt76_tx_info *tx_info);
void mt7921_usb_sdio_tx_complete_skb(struct mt76_dev *mdev,
				     struct mt76_queue_entry *e);
bool mt7921_usb_sdio_tx_status_data(struct mt76_dev *mdev, u8 *update);

/* usb */
#define MT_USB_TYPE_VENDOR	(USB_TYPE_VENDOR | 0x1f)
#define MT_USB_TYPE_UHW_VENDOR	(USB_TYPE_VENDOR | 0x1e)

int mt7921u_mcu_power_on(struct mt7921_dev *dev);
int mt7921u_wfsys_reset(struct mt7921_dev *dev);
int mt7921u_dma_init(struct mt7921_dev *dev, bool resume);
int mt7921u_init_reset(struct mt7921_dev *dev);
int mt7921u_mac_reset(struct mt7921_dev *dev);
int mt7921_mcu_uni_add_beacon_offload(struct mt7921_dev *dev,
				      struct ieee80211_hw *hw,
				      struct ieee80211_vif *vif,
				      bool enable);
#ifdef CONFIG_ACPI
int mt7921_init_acpi_sar(struct mt7921_dev *dev);
int mt7921_init_acpi_sar_power(struct mt7921_phy *phy, bool set_default);
u8 mt7921_acpi_get_flags(struct mt7921_phy *phy);
#else
static inline int
mt7921_init_acpi_sar(struct mt7921_dev *dev)
{
	return 0;
}

static inline int
mt7921_init_acpi_sar_power(struct mt7921_phy *phy, bool set_default)
{
	return 0;
}

static inline u8
mt7921_acpi_get_flags(struct mt7921_phy *phy)
{
	return 0;
}
#endif
int mt7921_set_tx_sar_pwr(struct ieee80211_hw *hw,
			  const struct cfg80211_sar_specs *sar);

int mt7921_mcu_set_clc(struct mt7921_dev *dev, u8 *alpha2,
		       enum environment_cap env_cap);
int mt7921_mcu_set_roc(struct mt7921_phy *phy, struct mt7921_vif *vif,
		       struct ieee80211_channel *chan, int duration,
		       enum mt7921_roc_req type, u8 token_id);
int mt7921_mcu_abort_roc(struct mt7921_phy *phy, struct mt7921_vif *vif,
			 u8 token_id);
u8 mt7921_check_offload_capability(struct device *dev, const char *fw_wm);
#endif
