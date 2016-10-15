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

#include <linux/firmware.h>
#include "mt7603.h"
#include "mt7603_mcu.h"
#include "mt7603_eeprom.h"

#define MCU_SKB_RESERVE	8

struct mt7603_fw_trailer {
	char fw_ver[10];
	char build_date[15];
	__le32 dl_len;
} __packed;

static struct sk_buff *
mt7603_mcu_msg_alloc(struct mt7603_dev *dev, const void *data, int len)
{
	struct sk_buff *skb;

	skb = alloc_skb(len + sizeof(struct mt7603_mcu_txd),
			GFP_KERNEL);
	skb_reserve(skb, sizeof(struct mt7603_mcu_txd));
	if (data && len)
		memcpy(skb_put(skb, len), data, len);

	return skb;
}

static struct sk_buff *
mt7603_mcu_get_response(struct mt7603_dev *dev, unsigned long expires)
{
	unsigned long timeout;

	if (!time_is_after_jiffies(expires))
		return NULL;

	timeout = expires - jiffies;
	wait_event_timeout(dev->mcu.wait, !skb_queue_empty(&dev->mcu.res_q),
			   timeout);
	return skb_dequeue(&dev->mcu.res_q);
}

static int
__mt7603_mcu_msg_send(struct mt7603_dev *dev, struct sk_buff *skb, int cmd, int query, int *wait_seq)
{
	int hdrlen = dev->mcu.running ? sizeof(struct mt7603_mcu_txd) : 12;
	struct mt7603_mcu_txd *txd;
	u8 seq;

	if (!skb)
		return -EINVAL;

	seq = ++dev->mcu.msg_seq & 0xf;
	if (!seq)
		seq = ++dev->mcu.msg_seq & 0xf;

	txd = (struct mt7603_mcu_txd *) skb_push(skb, hdrlen);
	memset(txd, 0, hdrlen);

	txd->len = cpu_to_le16(skb->len);
	if (cmd == -MCU_CMD_FW_SCATTER)
		txd->pq_id = cpu_to_le16(MCU_PORT_QUEUE_FW);
	else
		txd->pq_id = cpu_to_le16(MCU_PORT_QUEUE);
	txd->pkt_type = MCU_PKT_ID;
	txd->seq = seq;

	if (cmd < 0) {
		txd->cid = -cmd;
	} else {
		txd->cid = MCU_CMD_EXT_CID;
		txd->ext_cid = cmd;
		if (query != MCU_Q_NA)
			txd->ext_cid_ack = 1;
	}

	txd->set_query = query;

	if (wait_seq)
		*wait_seq = seq;

	return mt7603_tx_queue_mcu(dev, MT_TXQ_MCU, skb);
}

static int
mt7603_mcu_msg_send(struct mt7603_dev *dev, struct sk_buff *skb, int cmd, int query,
		    struct sk_buff **skb_ret)
{
	unsigned long expires = jiffies + HZ;
	struct mt7603_mcu_rxd *rxd;
	int ret, seq;

	mutex_lock(&dev->mcu.mutex);

	ret = __mt7603_mcu_msg_send(dev, skb, cmd, query, &seq);
	if (ret)
		goto out;

	while (1) {
		skb = mt7603_mcu_get_response(dev, expires);
		if (!skb) {
			printk("MCU message %d (seq %d) timed out\n", cmd, seq);
			dev->tx_dma_check = MT7603_WATCHDOG_TIMEOUT;
			ret = -ETIMEDOUT;
			break;
		}

		rxd = (struct mt7603_mcu_rxd *) skb->data;
		skb_pull(skb, sizeof(*rxd));

		if (seq != rxd->seq)
			continue;

		if (skb_ret)
			*skb_ret = skb;
		else
			dev_kfree_skb(skb);

		break;
	}

out:
	mutex_unlock(&dev->mcu.mutex);

	return ret;
}

static int
mt7603_mcu_init_download(struct mt7603_dev *dev, u32 addr, u32 len)
{
	struct {
		__le32 addr;
		__le32 len;
		__le32 mode;
	} req = {
		.addr = cpu_to_le32(addr),
		.len = cpu_to_le32(len),
		.mode = cpu_to_le32(BIT(31)),
	};
	struct sk_buff *skb = mt7603_mcu_msg_alloc(dev, &req, sizeof(req));

	return mt7603_mcu_msg_send(dev, skb, -MCU_CMD_TARGET_ADDRESS_LEN_REQ, MCU_Q_NA, NULL);
}

static int
mt7603_mcu_send_firmware(struct mt7603_dev *dev, const void *data, int len)
{
	struct sk_buff *skb;
	int ret = 0;

	while (len > 0) {
		int cur_len = min_t(int, 4096 - sizeof(struct mt7603_mcu_txd), len);

		skb = mt7603_mcu_msg_alloc(dev, data, cur_len);
		if (!skb)
			return -ENOMEM;

		ret = __mt7603_mcu_msg_send(dev, skb, -MCU_CMD_FW_SCATTER, MCU_Q_NA, NULL);
		if (ret)
			break;

		data += cur_len;
		len -= cur_len;
	}

	return ret;
}

static int
mt7603_mcu_start_firmware(struct mt7603_dev *dev, u32 addr)
{
	struct {
		__le32 override;
		__le32 addr;
	} req = {
		.override = cpu_to_le32(addr ? 1 : 0),
		.addr = cpu_to_le32(addr),
	};
	struct sk_buff *skb = mt7603_mcu_msg_alloc(dev, &req, sizeof(req));

	return mt7603_mcu_msg_send(dev, skb, -MCU_CMD_FW_START_REQ, MCU_Q_NA, NULL);
}

static int
mt7603_mcu_restart(struct mt7603_dev *dev)
{
	struct sk_buff *skb = mt7603_mcu_msg_alloc(dev, NULL, 0);

	return mt7603_mcu_msg_send(dev, skb, -MCU_CMD_RESTART_DL_REQ, MCU_Q_NA, NULL);
}

static int
mt7603_load_firmware(struct mt7603_dev *dev)
{
	const struct firmware *fw;
	const struct mt7603_fw_trailer *hdr;
	const char *firmware;
	int dl_len;
	u32 addr, val;
	int ret;

	if (is_mt7628(dev)) {
		if (mt76xx_rev(dev) == MT7628_REV_E1)
			firmware = MT7628_FIRMWARE_E1;
		else
			firmware = MT7628_FIRMWARE_E2;
	} else {
		if (mt76xx_rev(dev) < MT7603_REV_E2)
			firmware = MT7603_FIRMWARE_E1;
		else
			firmware = MT7603_FIRMWARE_E2;
	}

	ret = request_firmware(&fw, firmware, dev->mt76.dev);
	if (ret)
		return ret;

	if (!fw || !fw->data || fw->size < sizeof(*hdr)) {
		dev_err(dev->mt76.dev, "Invalid firmware\n");
		ret = -EINVAL;
		goto out;
	}

	hdr = (const struct mt7603_fw_trailer *) (fw->data + fw->size - sizeof(*hdr));

	dev_info(dev->mt76.dev, "Firmware Version: %.10s\n", hdr->fw_ver);
	dev_info(dev->mt76.dev, "Build Time: %.15s\n", hdr->build_date);

	addr = mt7603_reg_map(dev, 0x50012498);
	mt76_wr(dev, addr, 0x5);
	mt76_wr(dev, addr, 0x5);
	udelay(1);

	/* switch to bypass mode */
	mt76_rmw(dev, MT_SCH_4, MT_SCH_4_FORCE_QID,
		 MT_SCH_4_BYPASS | MT76_SET(MT_SCH_4_FORCE_QID, 5));

	val = mt76_rr(dev, MT_TOP_MISC2);
	if (val & BIT(1)) {
		dev_info(dev->mt76.dev, "Firmware already running...\n");
		goto running;
	}

	if (!mt76_poll_msec(dev, MT_TOP_MISC2, BIT(0) | BIT(1), BIT(0), 500)) {
		dev_err(dev->mt76.dev, "Timeout waiting for ROM code to become ready\n");
		ret = -EIO;
		goto out;
	}

	dl_len = le32_to_cpu(hdr->dl_len) + 4;
	ret = mt7603_mcu_init_download(dev, MCU_FIRMWARE_ADDRESS, dl_len);
	if (ret) {
		dev_err(dev->mt76.dev, "Download request failed\n");
		goto out;
	}

	ret = mt7603_mcu_send_firmware(dev, fw->data, dl_len);
	if (ret) {
		dev_err(dev->mt76.dev, "Failed to send firmware to device\n");
		goto out;
	}

	ret = mt7603_mcu_start_firmware(dev, MCU_FIRMWARE_ADDRESS);
	if (ret) {
		dev_err(dev->mt76.dev, "Failed to start firmware\n");
		goto out;
	}

	if (!mt76_poll_msec(dev, MT_TOP_MISC2, BIT(1), BIT(1), 500)) {
		dev_err(dev->mt76.dev, "Timeout waiting for firmware to initialize\n");
		ret = -EIO;
		goto out;
	}

running:
	mt76_clear(dev, MT_SCH_4, MT_SCH_4_FORCE_QID | MT_SCH_4_BYPASS);

	mt76_set(dev, MT_SCH_4, BIT(8));
	mt76_clear(dev, MT_SCH_4, BIT(8));

	dev->mcu.running = true;
	printk("firmware init done\n");

out:
	release_firmware(fw);

	return ret;
}

int mt7603_mcu_init(struct mt7603_dev *dev)
{
	mutex_init(&dev->mcu.mutex);
	return mt7603_load_firmware(dev);
}

void mt7603_mcu_exit(struct mt7603_dev *dev)
{
	struct sk_buff *skb;

	mt7603_mcu_restart(dev);

	while ((skb = skb_dequeue(&dev->mcu.res_q)) != NULL)
		dev_kfree_skb(skb);
}

int mt7603_mcu_set_eeprom(struct mt7603_dev *dev)
{
	static const u16 req_fields[] = {
#define WORD(_start)			\
		_start,			\
		_start + 1
#define GROUP_2G(_start)		\
		WORD(_start),		\
		WORD(_start + 2),	\
		WORD(_start + 4)

		MT_EE_NIC_CONF_0 + 1,
		WORD(MT_EE_NIC_CONF_1),
		MT_EE_WIFI_RF_SETTING,
		MT_EE_TX_POWER_DELTA_BW40,
		MT_EE_TX_POWER_DELTA_BW80 + 1,
		MT_EE_TX_POWER_EXT_PA_5G,
		MT_EE_TEMP_SENSOR_CAL,
		GROUP_2G(MT_EE_TX_POWER_0_START_2G),
		GROUP_2G(MT_EE_TX_POWER_1_START_2G),
		WORD(MT_EE_TX_POWER_CCK),
		WORD(MT_EE_TX_POWER_OFDM_2G_6M),
		WORD(MT_EE_TX_POWER_OFDM_2G_24M),
		WORD(MT_EE_TX_POWER_OFDM_2G_54M),
		WORD(MT_EE_TX_POWER_HT_BPSK_QPSK),
		WORD(MT_EE_TX_POWER_HT_16_64_QAM),
		WORD(MT_EE_TX_POWER_HT_64_QAM),
		MT_EE_ELAN_RX_MODE_GAIN,
		MT_EE_ELAN_RX_MODE_NF,
		MT_EE_ELAN_RX_MODE_P1DB,
		MT_EE_ELAN_BYPASS_MODE_GAIN,
		MT_EE_ELAN_BYPASS_MODE_NF,
		MT_EE_ELAN_BYPASS_MODE_P1DB,
		WORD(MT_EE_STEP_NUM_NEG_6_7),
		WORD(MT_EE_STEP_NUM_NEG_4_5),
		WORD(MT_EE_STEP_NUM_NEG_2_3),
		WORD(MT_EE_STEP_NUM_NEG_0_1),
		WORD(MT_EE_REF_STEP_24G),
		WORD(MT_EE_STEP_NUM_PLUS_1_2),
		WORD(MT_EE_STEP_NUM_PLUS_3_4),
		WORD(MT_EE_STEP_NUM_PLUS_5_6),
		MT_EE_STEP_NUM_PLUS_7,
		MT_EE_XTAL_FREQ_OFFSET,
		MT_EE_XTAL_TRIM_2_COMP,
		MT_EE_XTAL_TRIM_3_COMP,
		MT_EE_XTAL_WF_RFCAL,

		/* unknown fields below */
		WORD(0x24),
		0x34,
		0x39,
		0x3b,
		WORD(0x42),
		WORD(0x9e),
		0xf2,
		WORD(0xf8),
		0xfa,
		0x12e,
		WORD(0x130), WORD(0x132), WORD(0x134), WORD(0x136),
		WORD(0x138), WORD(0x13a), WORD(0x13c), WORD(0x13e),

#undef GROUP_2G
#undef WORD

	};
	struct req_data {
		u16 addr;
		u8 val;
		u8 pad;
	} __packed;
	struct {
		u8 buffer_mode;
		u8 len;
		u8 pad[2];
	} req_hdr = {
		.buffer_mode = 1,
		.len = ARRAY_SIZE(req_fields) - 1,
	};
	struct sk_buff *skb;
	struct req_data *data;
	const int size = 0xff * sizeof(struct req_data);
	u8 *eep = (u8 *) dev->mt76.eeprom.data;
	int i;

	BUILD_BUG_ON(ARRAY_SIZE(req_fields) * sizeof(*data) > size);

	skb = mt7603_mcu_msg_alloc(dev, NULL, size + sizeof(req_hdr));
	memcpy(skb_put(skb, sizeof(req_hdr)), &req_hdr, sizeof(req_hdr));
	data = (struct req_data *) skb_put(skb, size);
	memset(data, 0, size);

	for (i = 0; i < ARRAY_SIZE(req_fields); i++) {
		data[i].addr = cpu_to_le16(req_fields[i]);
		data[i].val = eep[req_fields[i]];
		data[i].pad = 0;
	}

	return mt7603_mcu_msg_send(dev, skb, MCU_EXT_CMD_EFUSE_BUFFER_MODE, MCU_Q_SET, NULL);
}

static int mt7603_mcu_set_tx_power(struct mt7603_dev *dev)
{
	struct {
		u8 center_channel;
		u8 tssi;
		u8 temp_comp;
		u8 target_power[2];
		u8 rate_power_delta[14];
		u8 bw_power_delta;
		u8 ch_power_delta[6];
		u8 temp_comp_power[17];
		u8 reserved;
	} req = {
		.center_channel = dev->mt76.chandef.chan->hw_value,
#define EEP_VAL(n) ((u8 *) dev->mt76.eeprom.data)[n]
		.tssi = EEP_VAL(MT_EE_NIC_CONF_1 + 1),
		.temp_comp = EEP_VAL(MT_EE_NIC_CONF_1),
		.target_power = {
			EEP_VAL(MT_EE_TX_POWER_0_START_2G + 2),
			EEP_VAL(MT_EE_TX_POWER_1_START_2G + 2)
		},
		.bw_power_delta = EEP_VAL(MT_EE_TX_POWER_DELTA_BW40),
		.ch_power_delta = {
			EEP_VAL(MT_EE_TX_POWER_0_START_2G + 3),
			EEP_VAL(MT_EE_TX_POWER_0_START_2G + 4),
			EEP_VAL(MT_EE_TX_POWER_0_START_2G + 5),
			EEP_VAL(MT_EE_TX_POWER_1_START_2G + 3),
			EEP_VAL(MT_EE_TX_POWER_1_START_2G + 4),
			EEP_VAL(MT_EE_TX_POWER_1_START_2G + 5)
		},
#undef EEP_VAL
	};
	struct sk_buff *skb;
	u8 *eep = (u8 *) dev->mt76.eeprom.data;

	memcpy(req.rate_power_delta, eep + MT_EE_TX_POWER_CCK,
	       sizeof(req.rate_power_delta));

	memcpy(req.temp_comp_power, eep + MT_EE_STEP_NUM_NEG_6_7,
	       sizeof(req.temp_comp_power));

	skb = mt7603_mcu_msg_alloc(dev, &req, sizeof(req));
	return mt7603_mcu_msg_send(dev, skb, MCU_EXT_CMD_SET_TX_POWER_CTRL, MCU_Q_SET, NULL);
}

int mt7603_mcu_set_channel(struct mt7603_dev *dev)
{
	struct {
		u8 control_chan;
		u8 center_chan;
		u8 bw;
		u8 tx_streams;
		u8 rx_streams;
		u8 _res0[7];
		u8 txpower[21];
		u8 _res1[3];
	} req = {
		.control_chan = dev->mt76.chandef.chan->hw_value,
		.center_chan = dev->mt76.chandef.chan->hw_value,
		.bw = MT_BW_20,
		.tx_streams = dev->tx_chains,
		.rx_streams = dev->rx_chains,
	};
	struct sk_buff *skb;
	int ret;

	memset(req.txpower, 0xff, sizeof(req.txpower));
	skb = mt7603_mcu_msg_alloc(dev, &req, sizeof(req));
	ret = mt7603_mcu_msg_send(dev, skb, MCU_EXT_CMD_CHANNEL_SWITCH, MCU_Q_SET, NULL);
	if (ret)
		return ret;

	return mt7603_mcu_set_tx_power(dev);
}

int mt7603_mcu_set_timing(struct mt7603_dev *dev, int slot, int sifs, int rifs,
			  int eifs)
{
	struct {
		u8 slot_time;
		u8 sifs_time;
		u8 rifs_time;
		u8 __res0;
		u16 eifs_time;
		u16 __res1;
	} req = {
		.slot_time = slot,
		.sifs_time = sifs,
		.rifs_time = rifs,
		.eifs_time = eifs,
	};
	struct sk_buff *skb;

	skb = mt7603_mcu_msg_alloc(dev, &req, sizeof(req));
	return mt7603_mcu_msg_send(dev, skb, MCU_EXT_CMD_SLOT_TIME_SET, MCU_Q_SET, NULL);
}

int mt7603_mcu_reg_read(struct mt7603_dev *dev, u32 reg, u32 *val, bool rf)
{
	struct {
		__le32 type;
		__le32 addr;
		__le32 data;
	} req = {
		.type = rf ? cpu_to_le32(1) : 0,
		.addr = cpu_to_le32(reg),
	};
	struct sk_buff *skb;
	__le32 *res;
	int ret;

	skb = mt7603_mcu_msg_alloc(dev, &req, sizeof(req));
	ret = mt7603_mcu_msg_send(dev, skb, MCU_EXT_CMD_MULTIPLE_REG_ACCESS, MCU_Q_QUERY, &skb);
	if (ret)
		return ret;

	res = (__le32 *) skb_pull(skb, 20);
	if (skb->len != 12)
		return -EIO;

	*val = le32_to_cpu(res[2]);
	return 0;
}
