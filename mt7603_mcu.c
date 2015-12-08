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

#include <linux/firmware.h>
#include "mt7603.h"
#include "mt7603_mcu.h"

struct mt7603_fw_trailer {
	char fw_ver[10];
	char build_date[15];
	__le32 dl_len;
} __packed;

static struct sk_buff *
mt7603_mcu_msg_alloc(struct mt7603_dev *dev, const void *data, int len)
{
	struct sk_buff *skb;

	skb = alloc_skb(len + sizeof(struct mt7603_mcu_txd), GFP_KERNEL);
	skb_reserve(skb, sizeof(struct mt7603_mcu_txd));
	if (len)
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
mt7603_mcu_msg_send(struct mt7603_dev *dev, struct sk_buff *skb, int cmd, int query)
{
	unsigned long expires = jiffies + HZ;
	struct mt7603_mcu_rxd *rxd;
	int ret, seq;

	mutex_lock(&dev->mcu.mutex);

	ret = __mt7603_mcu_msg_send(dev, skb, cmd, query, &seq);
	if (ret)
		goto out;

	while (1) {
		bool check_seq = false;

		skb = mt7603_mcu_get_response(dev, expires);
		if (!skb) {
			printk("MCU message %d (seq %d) timed out\n", cmd, seq);
			ret = -ETIMEDOUT;
			break;
		}

		rxd = (struct mt7603_mcu_rxd *) skb->data;

		if (seq == rxd->seq)
			check_seq = true;

		dev_kfree_skb(skb);
		if (check_seq)
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

	return mt7603_mcu_msg_send(dev, skb, -MCU_CMD_TARGET_ADDRESS_LEN_REQ, MCU_Q_NA);
}

static int
mt7603_mcu_send_firmware(struct mt7603_dev *dev, const void *data, int len)
{
	struct sk_buff *skb;
	int ret = 0;

	while (len > 0) {
		int cur_len = min_t(int, 4096 - 12, len);

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

	return mt7603_mcu_msg_send(dev, skb, -MCU_CMD_FW_START_REQ, MCU_Q_NA);
}

static int
mt7603_mcu_restart(struct mt7603_dev *dev)
{
	struct sk_buff *skb = mt7603_mcu_msg_alloc(dev, NULL, 0);

	return mt7603_mcu_msg_send(dev, skb, -MCU_CMD_RESTART_DL_REQ, MCU_Q_NA);
}

static int
mt7603_load_firmware(struct mt7603_dev *dev)
{
	const struct firmware *fw;
	const struct mt7603_fw_trailer *hdr;
	const char *firmware;
	int dl_len;
	u32 val;
	int ret;

	if (mt76xx_rev(dev) < MT7603_REV_E2)
		firmware = MT7603_FIRMWARE_E1;
	else
		firmware = MT7603_FIRMWARE_E2;

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

	val = mt76_rr(dev, MT_MCU_PCIE_REMAP_1);

	mt76_wr(dev, MT_MCU_PCIE_REMAP_1,
		MT76_SET(MT_MCU_PCIE_REMAP_1_BASE, 0x1400));

	mt76_wr(dev, MT_PCIE_REMAP_BASE_1 + 0x12498, 0x5);
	mt76_wr(dev, MT_PCIE_REMAP_BASE_1 + 0x12498, 0x5);
	udelay(1);

	mt76_wr(dev, MT_MCU_PCIE_REMAP_1, val);

	/* switch to bypass mode */
	mt76_rmw(dev, MT_SCH_4, MT_SCH_4_FORCE_QID,
		 MT_SCH_4_BYPASS | MT76_SET(MT_SCH_4_FORCE_QID, 5));

	val = mt76_rr(dev, MT_TOP_MISC2);
	if (val & BIT(1)) {
		dev_info(dev->mt76.dev, "Firmware already running, restarting...\n");

		ret = mt7603_mcu_restart(dev);
		if (ret) {
			dev_err(dev->mt76.dev, "Failed to restart the MCU\n");
			goto out;
		}
	}

	if (!mt76_poll(dev, MT_TOP_MISC2, BIT(0), BIT(0), 500)) {
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

	mt76_clear(dev, MT_SCH_4, MT_SCH_4_FORCE_QID | MT_SCH_4_BYPASS);

	mt76_set(dev, MT_SCH_4, BIT(8));
	mt76_clear(dev, MT_SCH_4, BIT(8));

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
		.control_chan = dev->chandef.chan->hw_value,
		.center_chan = dev->chandef.chan->hw_value,
		.bw = MT_BW_20,
		.tx_streams = dev->tx_chains,
		.rx_streams = dev->rx_chains,
	};
	struct sk_buff *skb;

	memset(req.txpower, 0xff, sizeof(req.txpower));
	skb = mt7603_mcu_msg_alloc(dev, &req, sizeof(req));
	return mt7603_mcu_msg_send(dev, skb, MCU_EXT_CMD_CHANNEL_SWITCH, MCU_Q_SET);
}
