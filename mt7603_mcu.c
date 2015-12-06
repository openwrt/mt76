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

struct mt7603_fw_trailer {
	char fw_ver[10];
	char build_date[15];
	__le32 dl_len;
} __packed;

static int
mt7603_load_firmware(struct mt7603_dev *dev)
{
	const struct firmware *fw;
	const struct mt7603_fw_trailer *hdr;
	u32 val;
	int ret;

	ret = request_firmware(&fw, MT7603_FIRMWARE_E1, dev->dev);
	if (ret)
		return ret;

	if (!fw || !fw->data || fw->size < sizeof(*hdr))
		goto error;

	hdr = (const struct mt7603_fw_trailer *) (fw->data + fw->size - sizeof(*hdr));

	dev_info(dev->dev, "Firmware Version: %.10s\n", hdr->fw_ver);
	dev_info(dev->dev, "Build Time: %.15s\n", hdr->build_date);

	val = mt76_rr(dev, MT_MCU_PCIE_REMAP_1);
	mt76_wr(dev, MT_MCU_PCIE_REMAP_1,
		MT76_SET(MT_MCU_PCIE_REMAP_1_OFFSET, 0x1400));
	mt76_wr(dev, MT_PCIE_REMAP_BASE_1 + 0x12498, 0x5);
	mt76_wr(dev, MT_PCIE_REMAP_BASE_1 + 0x12498, 0x5);
	udelay(1);

	mt76_wr(dev, MT_MCU_PCIE_REMAP_1,
		MT76_SET(MT_MCU_PCIE_REMAP_1_BASE, 0x2940));

	mt76_wr(dev, MT_MCU_PCIE_REMAP_2,
		MT76_SET(MT_MCU_PCIE_REMAP_2_BASE, 0x1001));

	/* switch to bypass mode */
	mt76_rmw(dev, MT_SCH_4, MT_SCH_4_FORCE_QID,
		 MT_SCH_4_BYPASS | MT76_SET(MT_SCH_4_FORCE_QID, 5));

	val = mt76_rr(dev, MT_TOP_MISC2);
	if (val & BIT(1)) {
		dev_info(dev->dev, "Firmware already running!\n");
		goto out;
	}

	if (!mt76_poll(dev, MT_TOP_MISC2, BIT(0), BIT(0), 500)) {
		dev_err(dev->dev, "Timeout waiting for ROM code to become ready\n");
		ret = -EIO;
		goto out;
	}


out:
	release_firmware(fw);

	return ret;

error:
	printk("Invalid firmware\n");
	release_firmware(fw);
	return -ENOENT;
}

int mt7603_mcu_init(struct mt7603_dev *dev)
{
	return mt7603_load_firmware(dev);
}
