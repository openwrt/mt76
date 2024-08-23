// SPDX-License-Identifier: ISC
/* Copyright (C) 2020 MediaTek Inc. */

#include <linux/firmware.h>
#include "mt7915.h"
#include "eeprom.h"

static int mt7915_eeprom_load_precal(struct mt7915_dev *dev)
{
	struct mt76_dev *mdev = &dev->mt76;
	u8 *eeprom = mdev->eeprom.data;
	u32 offs = is_mt7915(&dev->mt76) ? MT_EE_DO_PRE_CAL : MT_EE_DO_PRE_CAL_V2;
	u32 size, val = eeprom[offs];
	int ret;

	if (!dev->flash_mode || !val)
		return 0;

	size = mt7915_get_cal_group_size(dev) + mt7915_get_cal_dpd_size(dev);

	dev->cal = devm_kzalloc(mdev->dev, size, GFP_KERNEL);
	if (!dev->cal)
		return -ENOMEM;

	offs = is_mt7915(&dev->mt76) ? MT_EE_PRECAL : MT_EE_PRECAL_V2;

	ret = mt76_get_of_data_from_mtd(mdev, dev->cal, offs, size);
	if (!ret)
		return ret;

	ret = mt76_get_of_data_from_nvmem(mdev, dev->cal, "precal", size);
	if (!ret)
		return ret;

	dev_warn(mdev->dev, "missing precal data, size=%d\n", size);
	devm_kfree(mdev->dev, dev->cal);
	dev->cal = NULL;

	return ret;
}

static int mt7915_check_eeprom(struct mt7915_dev *dev)
{
	u8 *eeprom = dev->mt76.eeprom.data;
	u16 val = get_unaligned_le16(eeprom);

#define CHECK_EEPROM_ERR(match)	(match ? 0 : -EINVAL)
	switch (val) {
	case 0x7915:
		return CHECK_EEPROM_ERR(is_mt7915(&dev->mt76));
	case 0x7916:
		return CHECK_EEPROM_ERR(is_mt7916(&dev->mt76));
	case 0x7981:
		return CHECK_EEPROM_ERR(is_mt7981(&dev->mt76));
	case 0x7986:
		return CHECK_EEPROM_ERR(is_mt7986(&dev->mt76));
	default:
		return -EINVAL;
	}
}

static char *mt7915_eeprom_name(struct mt7915_dev *dev)
{
	switch (mt76_chip(&dev->mt76)) {
	case 0x7915:
		return dev->dbdc_support ?
		       MT7915_EEPROM_DEFAULT_DBDC : MT7915_EEPROM_DEFAULT;
	case 0x7981:
		/* mt7981 only supports mt7976 and only in DBDC mode */
		return MT7981_EEPROM_MT7976_DEFAULT_DBDC;
	case 0x7986:
		switch (mt7915_check_adie(dev, true)) {
		case MT7976_ONE_ADIE_DBDC:
			return MT7986_EEPROM_MT7976_DEFAULT_DBDC;
		case MT7975_ONE_ADIE:
			return MT7986_EEPROM_MT7975_DEFAULT;
		case MT7976_ONE_ADIE:
			return MT7986_EEPROM_MT7976_DEFAULT;
		case MT7975_DUAL_ADIE:
			return MT7986_EEPROM_MT7975_DUAL_DEFAULT;
		case MT7976_DUAL_ADIE:
			return MT7986_EEPROM_MT7976_DUAL_DEFAULT;
		default:
			break;
		}
		return NULL;
	default:
		return MT7916_EEPROM_DEFAULT;
	}
}

static int
mt7915_eeprom_load_default(struct mt7915_dev *dev)
{
	u8 *eeprom = dev->mt76.eeprom.data;
	const struct firmware *fw = NULL;
	int ret;

	ret = request_firmware(&fw, mt7915_eeprom_name(dev), dev->mt76.dev);
	if (ret)
		return ret;

	if (!fw || !fw->data) {
		dev_err(dev->mt76.dev, "Invalid default bin\n");
		ret = -EINVAL;
		goto out;
	}

	memcpy(eeprom, fw->data, mt7915_eeprom_size(dev));
	dev->flash_mode = true;

out:
	release_firmware(fw);

	return ret;
}

static int mt7915_eeprom_load_efuse(struct mt7915_dev *dev)
{
	u32 eeprom_blk_size = MT7915_EEPROM_BLOCK_SIZE;
	u16 eeprom_size = mt7915_eeprom_size(dev);
	u8 free_block_num;
	u32 block_num, i;
	u8 *buf;
	int ret;

	buf = devm_kzalloc(dev->mt76.dev, eeprom_size, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	dev->mt76.otp.data = buf;
	dev->mt76.otp.size = eeprom_size;

	ret = mt7915_mcu_get_eeprom_free_block(dev, &free_block_num);
	if (ret < 0)
		return ret;

	/* read eeprom data from efuse */
	block_num = DIV_ROUND_UP(eeprom_size, eeprom_blk_size);
	for (i = 0; i < block_num; i++) {
		mt7915_mcu_get_eeprom(dev, buf, i * eeprom_blk_size);
		buf += eeprom_blk_size;
	}

	/* efuse info isn't enough */
	if (free_block_num >= 29)
		return -EINVAL;

	if (!dev->flash_mode)
		memcpy(dev->mt76.eeprom.data, dev->mt76.otp.data, eeprom_size);

	return 0;
}

static int mt7915_eeprom_load(struct mt7915_dev *dev)
{
	u16 eeprom_size = mt7915_eeprom_size(dev);
	int ret;

	ret = mt76_eeprom_init(&dev->mt76, eeprom_size);
	if (ret < 0)
		return ret;

	if (ret)
		dev->flash_mode = true;

	mt7915_eeprom_load_efuse(dev);

	return mt7915_check_eeprom(dev);
}

static void mt7915_eeprom_parse_band_config(struct mt7915_phy *phy)
{
	struct mt7915_dev *dev = phy->dev;
	u8 *eeprom = dev->mt76.eeprom.data;
	u8 band = phy->mt76->band_idx;
	u32 val;

	val = eeprom[MT_EE_WIFI_CONF + band];
	val = FIELD_GET(MT_EE_WIFI_CONF0_BAND_SEL, val);

	if (!is_mt7915(&dev->mt76)) {
		switch (val) {
		case MT_EE_V2_BAND_SEL_5GHZ:
			phy->mt76->cap.has_5ghz = true;
			return;
		case MT_EE_V2_BAND_SEL_6GHZ:
			phy->mt76->cap.has_6ghz = true;
			return;
		case MT_EE_V2_BAND_SEL_5GHZ_6GHZ:
			phy->mt76->cap.has_5ghz = true;
			phy->mt76->cap.has_6ghz = true;
			return;
		default:
			phy->mt76->cap.has_2ghz = true;
			return;
		}
	} else if (val == MT_EE_BAND_SEL_DEFAULT && dev->dbdc_support) {
		val = band ? MT_EE_BAND_SEL_5GHZ : MT_EE_BAND_SEL_2GHZ;
	}

	switch (val) {
	case MT_EE_BAND_SEL_5GHZ:
		phy->mt76->cap.has_5ghz = true;
		break;
	case MT_EE_BAND_SEL_2GHZ:
		phy->mt76->cap.has_2ghz = true;
		break;
	default:
		phy->mt76->cap.has_2ghz = true;
		phy->mt76->cap.has_5ghz = true;
		break;
	}
}

void mt7915_eeprom_parse_hw_cap(struct mt7915_dev *dev,
				struct mt7915_phy *phy)
{
	u8 path, nss, nss_max = 4, *eeprom = dev->mt76.eeprom.data;
	struct mt76_phy *mphy = phy->mt76;
	u8 band = phy->mt76->band_idx;

	mt7915_eeprom_parse_band_config(phy);

	/* read tx/rx path from eeprom */
	if (is_mt7915(&dev->mt76)) {
		path = FIELD_GET(MT_EE_WIFI_CONF0_TX_PATH,
				 eeprom[MT_EE_WIFI_CONF]);
	} else {
		path = FIELD_GET(MT_EE_WIFI_CONF0_TX_PATH,
				 eeprom[MT_EE_WIFI_CONF + band]);
	}

	if (!path || path > 4)
		path = 4;

	/* read tx/rx stream */
	nss = path;
	if (dev->dbdc_support) {
		if (is_mt7915(&dev->mt76)) {
			path = min_t(u8, path, 2);
			nss = FIELD_GET(MT_EE_WIFI_CONF3_TX_PATH_B0,
					eeprom[MT_EE_WIFI_CONF + 3]);
			if (band)
				nss = FIELD_GET(MT_EE_WIFI_CONF3_TX_PATH_B1,
						eeprom[MT_EE_WIFI_CONF + 3]);
		} else {
			nss = FIELD_GET(MT_EE_WIFI_CONF_STREAM_NUM,
					eeprom[MT_EE_WIFI_CONF + 2 + band]);
		}

		if (!is_mt798x(&dev->mt76))
			nss_max = 2;
	}

	if (!nss)
		nss = nss_max;
	nss = min_t(u8, min_t(u8, nss_max, nss), path);

	mphy->chainmask = BIT(path) - 1;
	if (band)
		mphy->chainmask <<= dev->chainshift;
	mphy->antenna_mask = BIT(nss) - 1;
	dev->chainmask |= mphy->chainmask;
	dev->chainshift = hweight8(dev->mphy.chainmask);
}

static int mt7915_apply_cal_free_data(struct mt7915_dev *dev)
{
#define MT_EE_CAL_FREE_MAX_SIZE		70
#define MT_EE_FREQ_OFFSET		0x77
#define MT_EE_ADIE1_MT7976C_OFFSET	0x270
#define MT_EE_ADIE1_E3_OFFSET		0x271
#define MT_EE_END_OFFSET		0xffff
	enum adie_type {
		ADIE_7975,
		ADIE_7976,
		ADIE_7981,
	};
	enum ddie_type {
		DDIE_7915,
		DDIE_7916,
	};
	static const u16 ddie_offs_list[][MT_EE_CAL_FREE_MAX_SIZE] = {
		[DDIE_7915] = {0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19,
			       0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x20, 0x21, 0x22, 0x23, 0x24,
			       0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e,
			       0x52, 0x70, 0x71, 0x72, 0x76, 0xa8, 0xa9, 0xaa, 0xab, 0xac,
			       0xad, 0xae, 0xaf, -1},
		[DDIE_7916] = {0x30, 0x31, 0x34, 0x35, 0x36, 0x38, 0x3c, 0x3a, 0x3d, 0x44,
			       0x46, 0xc8, 0xc9, 0xca, 0xcb, 0xcc, 0xcd, 0xce, 0xcf, 0xe0, -1},
	};
	static const u16 adie_offs_list[][MT_EE_CAL_FREE_MAX_SIZE] = {
		[ADIE_7975] = {0x7cd, 0x7cf, 0x7d1, 0x7d3, 0x802, 0x803, 0x804, 0x805, 0x806,
			       0x808, 0x80a, 0x80b, 0x80c, 0x80d, 0x80e, 0x810, 0x812, 0x813,
			       0x814, 0x815, 0x816, 0x818, 0x81a, 0x81b, 0x81c, 0x81d, 0x81e,
			       0x820, 0x822, 0x823, 0x824, 0x825, 0x826, 0x827, 0x828, 0x829,
			       0x82f, 0x8c0, 0x8c1, 0x8c2, 0x8c3, 0x9a0, 0x8d0, 0x8d1, 0x8d7,
			       0x8d8, 0x8fa, 0x9a1, 0x9a5, 0x9a6, 0x9a8, 0x9aa, 0x9b0, 0x9b1,
			       0x9b2, 0x9b3, 0x9b4, 0x9b5, 0x9b6, 0x9b7, -1},
		[ADIE_7976] = {0x24c, 0x24d, 0x24e, 0x24f, 0x250, 0x251, 0x253, 0x255, 0x257,
			       0x259, 0x990, 0x991, 0x994, 0x995, 0x9a6, 0x9a8, 0x9aa, -1},
		[ADIE_7981] = {0x7cd, 0x7cf, 0x7d1, 0x7d3, 0x802, 0x803, 0x804, 0x805, 0x806,
			       0x808, 0x80a, 0x80b, 0x80c, 0x80d, 0x80e, 0x810, 0x812, 0x813,
			       0x814, 0x815, 0x816, 0x818, 0x81a, 0x81b, 0x81c, 0x81d, 0x81e,
			       0x820, 0x822, 0x823, 0x824, 0x825, 0x826, 0x827, 0x828, 0x829,
			       0x82f, 0x8c0, 0x8c1, 0x8c2, 0x8c3, 0x8d0, 0x8d1, 0x8d7, 0x8d8,
			       0x8fa, 0x9a1, 0x9a5, 0x9a6, 0x9a8, 0x9aa, 0x9b0, 0x9b1, 0x9b2,
			       0x9b3, 0x9b4, 0x9b5, 0x9b6, 0x9b7, -1 },
	};
	static const u16 eep_offs_list[][MT_EE_CAL_FREE_MAX_SIZE] = {
		[ADIE_7975] = {0xe00, 0xe01, 0xe02, 0xe03, 0xe04, 0xe05, 0xe06, 0xe07, 0xe08,
			       0xe09, 0xe0a, 0xe0b, 0xe0c, 0xe0d, 0x80e, 0xe0f, 0xe10, 0xe11,
			       0xe12, 0xe13, 0xe14, 0xe15, 0xe16, 0xe17, 0xe18, 0xe19, 0xe1a,
			       0xe1b, 0xe1c, 0xe1d, 0xe1e, 0xe1f, 0xe20, 0xe21, 0xe22, 0xe23,
			       0xe24, 0xe25, 0xe26, 0xe27, 0xe28, 0xe29, 0xe2a, 0xe2b, 0xe2c,
			       0xe2d, 0xe2e, 0xe2f, 0xe33, 0xe34, 0xe36, 0xe38, 0xe39, 0xe3a,
			       0xe3b, 0xe3c, 0xe3d, 0xe3e, 0xe3f, 0xe40, -1},
		[ADIE_7976] = {0x33c, 0x33d, 0x33e, 0x33f, 0x340, 0x341, 0x343, 0x345, 0x347,
			       0x349, 0x359, 0x35a, 0x35d, 0x35e, 0x36a, 0x36c, 0x36e, -1},
	};
	static const u16 *ddie_offs;
	static const u16 *adie_offs[__MT_MAX_BAND];
	static const u16 *eep_offs[__MT_MAX_BAND];
	static u16 adie_base[__MT_MAX_BAND] = {0};
	u8 *eeprom = dev->mt76.eeprom.data;
	u8 buf[MT7915_EEPROM_BLOCK_SIZE];
	int adie_id, band, i, ret;

	switch (mt76_chip(&dev->mt76)) {
	case 0x7915:
		ddie_offs = ddie_offs_list[DDIE_7915];
		ret = mt7915_mcu_get_eeprom(dev, buf, MT_EE_ADIE_FT_VERSION);
		if (ret == -EINVAL)
			return 0;
		else if (ret)
			return ret;
		adie_id = buf[MT_EE_ADIE_FT_VERSION % MT7915_EEPROM_BLOCK_SIZE] - 1;
		adie_offs[0] = adie_offs_list[ADIE_7975];
		/* same as adie offset */
		eep_offs[0] = NULL;
		break;
	case 0x7981:
		adie_offs[0] = adie_offs_list[ADIE_7981];
		eep_offs[0] = NULL;
		break;
	case 0x7906:
		if (is_mt7916(&dev->mt76))
			ddie_offs = ddie_offs_list[DDIE_7916];
		adie_offs[0] = adie_offs_list[ADIE_7976];
		eep_offs[0] = NULL;
		break;
	case 0x7986:
		adie_id = mt7915_check_adie(dev, true);
		switch (adie_id) {
		case MT7975_ONE_ADIE:
		case MT7975_DUAL_ADIE:
			adie_offs[0] = adie_offs_list[ADIE_7975];
			eep_offs[0] = NULL;
			if (adie_id == MT7975_DUAL_ADIE) {
				adie_offs[1] = adie_offs_list[ADIE_7975];
				eep_offs[1] = eep_offs_list[ADIE_7975];
			}
			break;
		case MT7976_ONE_ADIE_DBDC:
		case MT7976_ONE_ADIE:
		case MT7976_DUAL_ADIE: {
			u16 base = 0, offset = MT_EE_ADIE1_MT7976C_OFFSET;

			adie_offs[0] = adie_offs_list[ADIE_7976];
			eep_offs[0] = NULL;
			if (adie_id == MT7976_DUAL_ADIE) {
				adie_offs[1] = adie_offs_list[ADIE_7976];
				eep_offs[1] = eep_offs_list[ADIE_7976];
				base = MT_EE_ADIE1_BASE_7986;
			}

			/* E3 re-bonding workaround */
			ret = mt7915_mcu_get_eeprom(dev, buf, offset + base);
			if (ret)
				break;
			offset = (offset + base) % MT7915_EEPROM_BLOCK_SIZE;
			eeprom[MT_EE_ADIE1_MT7976C_OFFSET] = buf[offset];
			offset = (MT_EE_ADIE1_E3_OFFSET + base) % MT7915_EEPROM_BLOCK_SIZE;
			eeprom[MT_EE_ADIE1_E3_OFFSET] = buf[offset];
			break;
		}
		default:
			return -EINVAL;
		}
		adie_base[1] = MT_EE_ADIE1_BASE_7986;
		break;
	default:
		return -EINVAL;
	}

	/* ddie */
	if (ddie_offs) {
		u16 ddie_offset;
		u32 block_num, prev_block_num = -1;

		for (i = 0; i < MT_EE_CAL_FREE_MAX_SIZE; i++) {
			ddie_offset = ddie_offs[i];
			block_num = ddie_offset / MT7915_EEPROM_BLOCK_SIZE;

			if (ddie_offset == MT_EE_END_OFFSET)
				break;

			if (prev_block_num != block_num) {
				ret = mt7915_mcu_get_eeprom(dev, buf, ddie_offset);
				if (ret) {
					prev_block_num = -1;
					continue;
				}
			}

			eeprom[ddie_offset] = buf[ddie_offset % MT7915_EEPROM_BLOCK_SIZE];
			prev_block_num = block_num;
		}
	}

	/* adie */
	for (band = 0; band < __MT_MAX_BAND; band++) {
		u16 adie_offset, eep_offset;
		u32 block_num, prev_block_num = -1;

		if (!adie_offs[band])
			continue;

		for (i = 0; i < MT_EE_CAL_FREE_MAX_SIZE; i++) {
			adie_offset = adie_offs[band][i] + adie_base[band];
			eep_offset = adie_offset;
			if (eep_offs[band])
				eep_offset = eep_offs[band][i];
			block_num = adie_offset / MT7915_EEPROM_BLOCK_SIZE;

			if (adie_offs[band][i] == MT_EE_END_OFFSET)
				break;

			if (is_mt7915(&dev->mt76) && !adie_id &&
			    adie_offset >= 0x8c0 && adie_offset <= 0x8c3)
				continue;

			if (prev_block_num != block_num) {
				ret = mt7915_mcu_get_eeprom(dev, buf, adie_offset);
				if (ret) {
					prev_block_num = -1;
					continue;
				}
			}

			eeprom[eep_offset] = buf[adie_offset % MT7915_EEPROM_BLOCK_SIZE];
			prev_block_num = block_num;

			/* workaround for Harrier */
			if (is_mt7915(&dev->mt76) && adie_offset == 0x9a1)
				eeprom[MT_EE_FREQ_OFFSET] = eeprom[adie_offset];
		}
	}

	return 0;
}

int mt7915_eeprom_init(struct mt7915_dev *dev)
{
	int ret;

	ret = mt7915_eeprom_load(dev);
	if (ret < 0) {
		if (ret != -EINVAL)
			return ret;

		dev_warn(dev->mt76.dev, "eeprom load fail, use default bin\n");
		ret = mt7915_eeprom_load_default(dev);
		if (ret)
			return ret;
	}

	mt7915_eeprom_load_precal(dev);
	ret = mt7915_apply_cal_free_data(dev);
	if (ret)
		return ret;

	mt7915_eeprom_parse_hw_cap(dev, &dev->phy);
	memcpy(dev->mphy.macaddr, dev->mt76.eeprom.data + MT_EE_MAC_ADDR,
	       ETH_ALEN);

	mt76_eeprom_override(&dev->mphy);

	return 0;
}

int mt7915_eeprom_get_target_power(struct mt7915_dev *dev,
				   struct ieee80211_channel *chan,
				   u8 chain_idx)
{
	u8 *eeprom = dev->mt76.eeprom.data;
	int index, target_power;
	bool tssi_on, is_7976;

	if (chain_idx > 3)
		return -EINVAL;

	tssi_on = mt7915_tssi_enabled(dev, chan->band);
	is_7976 = mt7915_check_adie(dev, false) || is_mt7916(&dev->mt76);

	if (chan->band == NL80211_BAND_2GHZ) {
		if (is_7976) {
			index = MT_EE_TX0_POWER_2G_V2 + chain_idx;
			target_power = eeprom[index];
		} else {
			index = MT_EE_TX0_POWER_2G + chain_idx * 3;
			target_power = eeprom[index];

			if (!tssi_on)
				target_power += eeprom[index + 1];
		}
	} else if (chan->band == NL80211_BAND_5GHZ) {
		int group = mt7915_get_channel_group_5g(chan->hw_value, is_7976);

		if (is_7976) {
			index = MT_EE_TX0_POWER_5G_V2 + chain_idx * 5;
			target_power = eeprom[index + group];
		} else {
			index = MT_EE_TX0_POWER_5G + chain_idx * 12;
			target_power = eeprom[index + group];

			if (!tssi_on)
				target_power += eeprom[index + 8];
		}
	} else {
		int group = mt7915_get_channel_group_6g(chan->hw_value);

		index = MT_EE_TX0_POWER_6G_V2 + chain_idx * 8;
		target_power = is_7976 ? eeprom[index + group] : 0;
	}

	return target_power;
}

s8 mt7915_eeprom_get_power_delta(struct mt7915_dev *dev, int band)
{
	u8 *eeprom = dev->mt76.eeprom.data;
	u32 val, offs;
	s8 delta;
	bool is_7976 = mt7915_check_adie(dev, false) || is_mt7916(&dev->mt76);

	if (band == NL80211_BAND_2GHZ)
		offs = is_7976 ? MT_EE_RATE_DELTA_2G_V2 : MT_EE_RATE_DELTA_2G;
	else if (band == NL80211_BAND_5GHZ)
		offs = is_7976 ? MT_EE_RATE_DELTA_5G_V2 : MT_EE_RATE_DELTA_5G;
	else
		offs = is_7976 ? MT_EE_RATE_DELTA_6G_V2 : 0;

	val = eeprom[offs];

	if (!offs || !(val & MT_EE_RATE_DELTA_EN))
		return 0;

	delta = FIELD_GET(MT_EE_RATE_DELTA_MASK, val);

	return val & MT_EE_RATE_DELTA_SIGN ? delta : -delta;
}

const u8 mt7915_sku_group_len[] = {
	[SKU_CCK] = 4,
	[SKU_OFDM] = 8,
	[SKU_HT_BW20] = 8,
	[SKU_HT_BW40] = 9,
	[SKU_VHT_BW20] = 12,
	[SKU_VHT_BW40] = 12,
	[SKU_VHT_BW80] = 12,
	[SKU_VHT_BW160] = 12,
	[SKU_HE_RU26] = 12,
	[SKU_HE_RU52] = 12,
	[SKU_HE_RU106] = 12,
	[SKU_HE_RU242] = 12,
	[SKU_HE_RU484] = 12,
	[SKU_HE_RU996] = 12,
	[SKU_HE_RU2x996] = 12
};
