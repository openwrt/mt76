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

#include <linux/delay.h>
#include "mt76.h"
#include "mcu.h"
#include "eeprom.h"

static bool
mt76_phy_rf_op(struct mt76_dev *dev, bool idx, u16 offset, bool write)
{
	u32 val = MT76_SET(MT_RF_CTRL_ADDR, offset);

	if (idx)
		val |= MT_RF_CTRL_IDX;

	if (write)
		val |= MT_RF_CTRL_WRITE;

	mt76_wr(dev, MT_RF_CTRL, val);

	return mt76_poll(dev, MT_RF_CTRL, MT_RF_CTRL_BUSY, 0, 2000);
}

static int __maybe_unused
mt76_phy_rf_read(struct mt76_dev *dev, bool idx, u16 offset, u32 *val)
{
	if (!mt76_phy_rf_op(dev, idx, offset, false))
		return -ETIMEDOUT;

	*val = mt76_rr(dev, MT_RF_DATA_READ);
	return 0;
}

static int __maybe_unused
mt76_phy_rf_write(struct mt76_dev *dev, bool idx, u16 offset, u32 val)
{
	mt76_wr(dev, MT_RF_DATA_WRITE, val);

	if (!mt76_phy_rf_op(dev, idx, offset, true))
		return -ETIMEDOUT;

	return 0;
}

static void
mt76_adjust_lna_gain(struct mt76_dev *dev, int reg, s8 offset)
{
	s8 gain;

	gain = MT76_GET(MT_BBP_AGC_LNA_GAIN, mt76_rr(dev, MT_BBP(AGC, reg)));
	gain -= offset / 2;
	mt76_rmw_field(dev, MT_BBP(AGC, reg), MT_BBP_AGC_LNA_GAIN, gain);
}

static void
mt76_adjust_agc_gain(struct mt76_dev *dev, int reg, s8 offset)
{
	s8 gain;

	gain = MT76_GET(MT_BBP_AGC_GAIN, mt76_rr(dev, MT_BBP(AGC, reg)));
	gain += offset;
	mt76_rmw_field(dev, MT_BBP(AGC, reg), MT_BBP_AGC_GAIN, gain);
}

static void
mt76_apply_gain_adj(struct mt76_dev *dev)
{
	s8 *gain_adj = dev->cal.rx.high_gain;

	mt76_adjust_lna_gain(dev, 4, gain_adj[0]);
	mt76_adjust_lna_gain(dev, 5, gain_adj[1]);

	mt76_adjust_agc_gain(dev, 8, gain_adj[0]);
	mt76_adjust_agc_gain(dev, 9, gain_adj[1]);
}

static u32
mt76_tx_power_mask(u8 v1, u8 v2, u8 v3, u8 v4)
{
	u32 val = 0;

	val |= (v1 & (BIT(6) - 1)) << 0;
	val |= (v2 & (BIT(6) - 1)) << 8;
	val |= (v3 & (BIT(6) - 1)) << 16;
	val |= (v4 & (BIT(6) - 1)) << 24;
	return val;
}

int mt76_phy_get_rssi(struct mt76_dev *dev, s8 rssi, int chain)
{
	struct mt76_rx_freq_cal *cal = &dev->cal.rx;

	rssi += cal->rssi_offset[chain];
	rssi -= cal->lna_gain;

	return rssi;
}

static u8
mt76_txpower_check(int value)
{
	if (value < 0)
		return 0;
	if (value > 0x2f)
		return 0x2f;
	return value;
}

static void
mt76_add_rate_power_offset(struct mt76_rate_power *r, int offset)
{
	int i;

	for (i = 0; i < sizeof(r->all); i++)
		r->all[i] += offset;
}

static void
mt76_limit_rate_power(struct mt76_rate_power *r, int limit)
{
	int i;

	for (i = 0; i < sizeof(r->all); i++)
		if (r->all[i] > limit)
			r->all[i] = limit;
}

static int
mt76_get_max_power(struct mt76_rate_power *r)
{
	int i;
	s8 ret = 0;

	for (i = 0; i < sizeof(r->all); i++)
		ret = max(ret, r->all[i]);

	return ret;
}

void mt76_phy_set_txpower(struct mt76_dev *dev)
{
	enum nl80211_chan_width width = dev->chandef.width;
	struct mt76_tx_power_info txp;
	int txp_0, txp_1, delta = 0;
	struct mt76_rate_power t = {};

	mt76_get_power_info(dev, &txp);

	if (width == NL80211_CHAN_WIDTH_40)
		delta = txp.delta_bw40;
	else if (width == NL80211_CHAN_WIDTH_80)
		delta = txp.delta_bw80;

	if (txp.target_power > dev->txpower_conf)
		delta -= txp.target_power - dev->txpower_conf;

	mt76_get_rate_power(dev, &t);
	mt76_add_rate_power_offset(&t, txp.chain[0].target_power +
				   txp.chain[0].delta);
	mt76_limit_rate_power(&t, dev->txpower_conf);
	dev->txpower_cur = mt76_get_max_power(&t);
	mt76_add_rate_power_offset(&t, -(txp.chain[0].target_power +
					 txp.chain[0].delta + delta));
	dev->target_power = txp.chain[0].target_power;
	dev->target_power_delta[0] = txp.chain[0].delta + delta;
	dev->target_power_delta[1] = txp.chain[1].delta + delta;
	dev->rate_power = t;

	txp_0 = mt76_txpower_check(txp.chain[0].target_power +
				   txp.chain[0].delta + delta);

	txp_1 = mt76_txpower_check(txp.chain[1].target_power +
				   txp.chain[1].delta + delta);

	mt76_rmw_field(dev, MT_TX_ALC_CFG_0, MT_TX_ALC_CFG_0_CH_INIT_0, txp_0);
	mt76_rmw_field(dev, MT_TX_ALC_CFG_0, MT_TX_ALC_CFG_0_CH_INIT_1, txp_1);

	mt76_wr(dev, MT_TX_PWR_CFG_0,
	        mt76_tx_power_mask(t.cck[0], t.cck[2], t.ofdm[0], t.ofdm[2]));
	mt76_wr(dev, MT_TX_PWR_CFG_1,
	        mt76_tx_power_mask(t.ofdm[4], t.ofdm[6], t.ht[0], t.ht[4]));
	mt76_wr(dev, MT_TX_PWR_CFG_2,
	        mt76_tx_power_mask(t.ht[4], t.ht[6], t.ht[8], t.ht[10]));
	mt76_wr(dev, MT_TX_PWR_CFG_3,
	        mt76_tx_power_mask(t.ht[12], t.ht[14], t.ht[0], t.ht[2]));
	mt76_wr(dev, MT_TX_PWR_CFG_4,
	        mt76_tx_power_mask(t.ht[4], t.ht[6], 0, 0));
	mt76_wr(dev, MT_TX_PWR_CFG_7,
	        mt76_tx_power_mask(t.ofdm[4], t.vht[8], t.ht[6], t.vht[8]));
	mt76_wr(dev, MT_TX_PWR_CFG_8,
	        mt76_tx_power_mask(t.ht[14], t.vht[8], t.vht[8], 0));
	mt76_wr(dev, MT_TX_PWR_CFG_9,
	        mt76_tx_power_mask(t.ht[6], t.vht[8], t.vht[8], 0));
}

static bool
mt76_channel_silent(struct mt76_dev *dev)
{
	struct ieee80211_channel *chan = dev->chandef.chan;

	return ((chan->flags & IEEE80211_CHAN_RADAR) &&
		chan->dfs_state != NL80211_DFS_AVAILABLE);
}

static bool
mt76_phy_tssi_init_cal(struct mt76_dev *dev)
{
	struct ieee80211_channel *chan = dev->chandef.chan;
	u32 flag = 0;

	if (!mt76_tssi_enabled(dev))
		return false;

	if (mt76_channel_silent(dev))
		return false;

	if (chan->band == IEEE80211_BAND_2GHZ)
		flag |= BIT(0);

	if (mt76_ext_pa_enabled(dev, chan->band))
		flag |= BIT(16);

	mt76_mcu_calibrate(dev, MCU_CAL_TSSI, flag);
	dev->cal.tssi_cal_done = true;
	return true;
}

static void
mt76_phy_channel_calibrate(struct mt76_dev *dev, bool mac_stopped)
{
	struct ieee80211_channel *chan = dev->chandef.chan;
	bool is_5ghz = chan->band == IEEE80211_BAND_5GHZ;

	if (dev->cal.channel_cal_done)
		return;

	if (mt76_channel_silent(dev))
		return;

	if (!dev->cal.tssi_cal_done)
		mt76_phy_tssi_init_cal(dev);

	if (!mac_stopped)
		mt76_mac_stop(dev, false);

	if (is_5ghz)
		mt76_mcu_calibrate(dev, MCU_CAL_LC, 0);

	mt76_mcu_calibrate(dev, MCU_CAL_TX_LOFT, is_5ghz);
	mt76_mcu_calibrate(dev, MCU_CAL_TXIQ, is_5ghz);
	mt76_mcu_calibrate(dev, MCU_CAL_RXIQC_FI, is_5ghz);
	mt76_mcu_calibrate(dev, MCU_CAL_TEMP_SENSOR, 0);
	mt76_mcu_calibrate(dev, MCU_CAL_TX_SHAPING, 0);

	if (!mac_stopped)
		mt76_mac_resume(dev);

	mt76_apply_gain_adj(dev);

	dev->cal.channel_cal_done = true;
}

static void
mt76_phy_set_txpower_regs(struct mt76_dev *dev, enum ieee80211_band band)
{
	u32 pa_mode[2];
	u32 pa_mode_adj;

	if (band == IEEE80211_BAND_2GHZ) {
		pa_mode[0] = 0x010055ff;
		pa_mode[1] = 0x00550055;

		mt76_wr(dev, MT_TX_ALC_CFG_2, 0x35160a00);
		mt76_wr(dev, MT_TX_ALC_CFG_3, 0x35160a06);

		if (mt76_ext_pa_enabled(dev, band)) {
			mt76_wr(dev, MT_RF_PA_MODE_ADJ0, 0x0000ec00);
			mt76_wr(dev, MT_RF_PA_MODE_ADJ1, 0x0000ec00);
		} else {
			mt76_wr(dev, MT_RF_PA_MODE_ADJ0, 0xf4000200);
			mt76_wr(dev, MT_RF_PA_MODE_ADJ1, 0xfa000200);
		}
	} else {
		pa_mode[0] = 0x0000ffff;
		pa_mode[1] = 0x00ff00ff;

		mt76_wr(dev, MT_TX_ALC_CFG_2, 0x1b0f0400);
		mt76_wr(dev, MT_TX_ALC_CFG_3, 0x1b0f0476);
		mt76_wr(dev, MT_TX_ALC_CFG_4, 0);

		if (mt76_ext_pa_enabled(dev, band))
			pa_mode_adj = 0x04000000;
		else
			pa_mode_adj = 0;

		mt76_wr(dev, MT_RF_PA_MODE_ADJ0, pa_mode_adj);
		mt76_wr(dev, MT_RF_PA_MODE_ADJ1, pa_mode_adj);
	}

	mt76_wr(dev, MT_BB_PA_MODE_CFG0, pa_mode[0]);
	mt76_wr(dev, MT_BB_PA_MODE_CFG1, pa_mode[1]);
	mt76_wr(dev, MT_RF_PA_MODE_CFG0, pa_mode[0]);
	mt76_wr(dev, MT_RF_PA_MODE_CFG1, pa_mode[1]);

	if (mt76_ext_pa_enabled(dev, band)) {
		u32 val = 0x3c3c023c;
		mt76_wr(dev, MT_TX0_RF_GAIN_CORR, val);
		mt76_wr(dev, MT_TX1_RF_GAIN_CORR, val);
		mt76_wr(dev, MT_TX_ALC_CFG_4, 0x00001818);
	} else {
		if (band == IEEE80211_BAND_2GHZ) {
			u32 val = 0x0f3c3c3c;
			mt76_wr(dev, MT_TX0_RF_GAIN_CORR, val);
			mt76_wr(dev, MT_TX1_RF_GAIN_CORR, val);
			mt76_wr(dev, MT_TX_ALC_CFG_4, 0x00000606);
		} else {
			mt76_wr(dev, MT_TX0_RF_GAIN_CORR, 0x383c023c);
			mt76_wr(dev, MT_TX1_RF_GAIN_CORR, 0x24282e28);
			mt76_wr(dev, MT_TX_ALC_CFG_4, 0);
		}
	}
}

static void
mt76_configure_tx_delay(struct mt76_dev *dev, enum ieee80211_band band, u8 bw)
{
	u32 cfg0, cfg1;

	if (mt76_ext_pa_enabled(dev, band)) {
		cfg0 = bw ? 0x000b0c01 : 0x00101101;
		cfg1 = 0x00010200;
	} else {
		cfg0 = bw ? 0x000b0b01 : 0x00101001;
		cfg1 = 0x00020000;
	}
	mt76_wr(dev, MT_TX_SW_CFG0, cfg0);
	mt76_wr(dev, MT_TX_SW_CFG1, cfg1);

	mt76_rmw_field(dev, MT_XIFS_TIME_CFG, MT_XIFS_TIME_CFG_CCK_SIFS,
		       13 + (bw ? 1 : 0));
}

static void
mt76_phy_set_bw(struct mt76_dev *dev, int width, u8 ctrl)
{
	int core_val, agc_val;

	switch (width) {
	case NL80211_CHAN_WIDTH_80:
		core_val = 3;
		agc_val = 7;
		break;
	case NL80211_CHAN_WIDTH_40:
		core_val = 2;
		agc_val = 3;
		break;
	default:
		core_val = 0;
		agc_val = 1;
		break;
	}

	mt76_rmw_field(dev, MT_BBP(CORE, 1), MT_BBP_CORE_R1_BW, core_val);
	mt76_rmw_field(dev, MT_BBP(AGC, 0), MT_BBP_AGC_R0_BW, agc_val);
	mt76_rmw_field(dev, MT_BBP(AGC, 0), MT_BBP_AGC_R0_CTRL_CHAN, ctrl);
	mt76_rmw_field(dev, MT_BBP(TXBE, 0), MT_BBP_TXBE_R0_CTRL_CHAN, ctrl);
}

static void
mt76_phy_set_band(struct mt76_dev *dev, int band, bool primary_upper)
{
	switch (band) {
	case IEEE80211_BAND_2GHZ:
		mt76_set(dev, MT_TX_BAND_CFG, MT_TX_BAND_CFG_2G);
		mt76_clear(dev, MT_TX_BAND_CFG, MT_TX_BAND_CFG_5G);
		break;
	case IEEE80211_BAND_5GHZ:
		mt76_clear(dev, MT_TX_BAND_CFG, MT_TX_BAND_CFG_2G);
		mt76_set(dev, MT_TX_BAND_CFG, MT_TX_BAND_CFG_5G);
		break;
	}

	mt76_rmw_field(dev, MT_TX_BAND_CFG, MT_TX_BAND_CFG_UPPER_40M,
		       primary_upper);
}

static void
mt76_set_rx_chains(struct mt76_dev *dev)
{
	u32 val;

	val = mt76_rr(dev, MT_BBP(AGC, 0));
	val &= ~(BIT(3) | BIT(4));

	if (dev->chainmask & BIT(1))
		val |= BIT(3);

	mt76_wr(dev, MT_BBP(AGC, 0), val);
}

static void
mt76_set_tx_dac(struct mt76_dev *dev)
{
	if (dev->chainmask & BIT(1))
		mt76_set(dev, MT_BBP(TXBE, 5), 3);
	else
		mt76_clear(dev, MT_BBP(TXBE, 5), 3);
}

static void
mt76_get_agc_gain(struct mt76_dev *dev, u8 *dest)
{
	dest[0] = mt76_get_field(dev, MT_BBP(AGC, 8), MT_BBP_AGC_GAIN);
	dest[1] = mt76_get_field(dev, MT_BBP(AGC, 9), MT_BBP_AGC_GAIN);
}

static int
mt76_get_rssi_gain_thresh(struct mt76_dev *dev)
{
	switch (dev->chandef.width) {
	case NL80211_CHAN_WIDTH_80:
		return -62;
	case NL80211_CHAN_WIDTH_40:
		return -65;
	default:
		return -68;
	}
}

static void
mt76_phy_update_channel_gain(struct mt76_dev *dev)
{
	u32 val = mt76_rr(dev, MT_BBP(AGC, 20));
	int rssi0 = (s8) MT76_GET(MT_BBP_AGC20_RSSI0, val);
	int rssi1 = (s8) MT76_GET(MT_BBP_AGC20_RSSI1, val);
	bool low_gain;
	u8 gain[2], gain_delta;

	dev->cal.avg_rssi[0] = (dev->cal.avg_rssi[0] * 15) / 16 + (rssi0 << 8);
	dev->cal.avg_rssi[1] = (dev->cal.avg_rssi[1] * 15) / 16 + (rssi1 << 8);
	dev->cal.avg_rssi_all = (dev->cal.avg_rssi[0] + dev->cal.avg_rssi[1]) / 512;

	low_gain = dev->cal.avg_rssi_all > mt76_get_rssi_gain_thresh(dev);
	if (dev->cal.low_gain == low_gain)
		return;

	dev->cal.low_gain = low_gain;

	if (dev->chandef.width >= NL80211_CHAN_WIDTH_40)
		val = 0x1e42 << 16;
	else
		val = 0x1836 << 16;

	mt76_get_agc_gain(dev, gain);
	val |= 0xf8;

	if (dev->chandef.width == NL80211_CHAN_WIDTH_80)
		mt76_wr(dev, MT_BBP(RXO, 14), 0x00560411);
	else
		mt76_wr(dev, MT_BBP(RXO, 14), 0x00560423);

	if (low_gain) {
		mt76_wr(dev, MT_BBP(AGC, 35), 0x08080808);
		mt76_wr(dev, MT_BBP(AGC, 37), 0x08080808);
		if (mt76_has_ext_lna(dev))
			gain_delta = 10;
		else
			gain_delta = 14;
	} else {
		mt76_wr(dev, MT_BBP(AGC, 35), 0x11111116);
		mt76_wr(dev, MT_BBP(AGC, 37), 0x1010161C);
		gain_delta = 0;
	}

	mt76_wr(dev, MT_BBP(AGC, 8),
		val | MT76_SET(MT_BBP_AGC_GAIN, gain[0] - gain_delta));
	mt76_wr(dev, MT_BBP(AGC, 9),
		val | MT76_SET(MT_BBP_AGC_GAIN, gain[1] - gain_delta));
}

int mt76_phy_set_channel(struct mt76_dev *dev,
			 struct cfg80211_chan_def *chandef)
{
	struct ieee80211_channel *chan = chandef->chan;
	bool scan = test_bit(MT76_SCANNING, &dev->state);
	enum ieee80211_band band = chan->band;
	u8 channel;

	u32 ext_cca_chan[4] = {
		[0] = MT76_SET(MT_EXT_CCA_CFG_CCA0, 0) |
		      MT76_SET(MT_EXT_CCA_CFG_CCA1, 1) |
		      MT76_SET(MT_EXT_CCA_CFG_CCA2, 2) |
		      MT76_SET(MT_EXT_CCA_CFG_CCA3, 3) |
		      MT76_SET(MT_EXT_CCA_CFG_CCA_MASK, BIT(0)),
		[1] = MT76_SET(MT_EXT_CCA_CFG_CCA0, 1) |
		      MT76_SET(MT_EXT_CCA_CFG_CCA1, 0) |
		      MT76_SET(MT_EXT_CCA_CFG_CCA2, 2) |
		      MT76_SET(MT_EXT_CCA_CFG_CCA3, 3) |
		      MT76_SET(MT_EXT_CCA_CFG_CCA_MASK, BIT(1)),
		[2] = MT76_SET(MT_EXT_CCA_CFG_CCA0, 2) |
		      MT76_SET(MT_EXT_CCA_CFG_CCA1, 3) |
		      MT76_SET(MT_EXT_CCA_CFG_CCA2, 1) |
		      MT76_SET(MT_EXT_CCA_CFG_CCA3, 0) |
		      MT76_SET(MT_EXT_CCA_CFG_CCA_MASK, BIT(2)),
		[3] = MT76_SET(MT_EXT_CCA_CFG_CCA0, 3) |
		      MT76_SET(MT_EXT_CCA_CFG_CCA1, 2) |
		      MT76_SET(MT_EXT_CCA_CFG_CCA2, 1) |
		      MT76_SET(MT_EXT_CCA_CFG_CCA3, 0) |
		      MT76_SET(MT_EXT_CCA_CFG_CCA_MASK, BIT(3)),
	};
	int ch_group_index;
	u8 bw, bw_index;
	int freq, freq1;
	int ret;
	u8 sifs = 13;

	dev->chandef = *chandef;
	dev->cal.channel_cal_done = false;
	freq = chandef->chan->center_freq;
	freq1 = chandef->center_freq1;
	channel = chan->hw_value;

	switch (chandef->width) {
	case NL80211_CHAN_WIDTH_40:
		bw = 1;
		if (freq1 > freq) {
			bw_index = 1;
			ch_group_index = 0;
		} else {
			bw_index = 3;
			ch_group_index = 1;
		}
		channel += 2 - ch_group_index * 4;
		break;
	case NL80211_CHAN_WIDTH_80:
		ch_group_index = (freq - freq1 + 30) / 20;
		if (WARN_ON(ch_group_index < 0 || ch_group_index > 3))
			ch_group_index = 0;
		bw = 2;
		bw_index = ch_group_index;
		channel += 6 - ch_group_index * 4;
		break;
	default:
		bw = 0;
		bw_index = 0;
		ch_group_index = 0;
		break;
	}

	mt76_read_rx_gain(dev);
	mt76_phy_set_txpower_regs(dev, band);
	mt76_configure_tx_delay(dev, band, bw);
	mt76_phy_set_txpower(dev);

	mt76_set_rx_chains(dev);
	mt76_phy_set_band(dev, chan->band, ch_group_index & 1);
	mt76_phy_set_bw(dev, chandef->width, ch_group_index);
	mt76_set_tx_dac(dev);

	mt76_rmw(dev, MT_EXT_CCA_CFG,
		 (MT_EXT_CCA_CFG_CCA0 |
		  MT_EXT_CCA_CFG_CCA1 |
		  MT_EXT_CCA_CFG_CCA2 |
		  MT_EXT_CCA_CFG_CCA3 |
		  MT_EXT_CCA_CFG_CCA_MASK),
		 ext_cca_chan[ch_group_index]);

	if (chandef->width >= NL80211_CHAN_WIDTH_40)
		sifs++;

	mt76_rmw_field(dev, MT_XIFS_TIME_CFG, MT_XIFS_TIME_CFG_OFDM_SIFS, sifs);

	ret = mt76_mcu_set_channel(dev, channel, bw, bw_index, scan);
	if (ret)
		return ret;

	mt76_mcu_init_gain(dev, channel, dev->cal.rx.mcu_gain, true);

	/* Enable LDPC Rx */
	if (mt76xx_rev(dev) >= MT76XX_REV_E3)
	    mt76_set(dev, MT_BBP(RXO, 13), BIT(10));

	if (!dev->cal.init_cal_done) {
		u8 val = mt76_eeprom_get(dev, MT_EE_BT_RCAL_RESULT);

		if (val != 0xff)
			mt76_mcu_calibrate(dev, MCU_CAL_R, 0);
	}

	mt76_mcu_calibrate(dev, MCU_CAL_RXDCOC, channel);

	/* Rx LPF calibration */
	if (!dev->cal.init_cal_done)
		mt76_mcu_calibrate(dev, MCU_CAL_RC, 0);

	dev->cal.init_cal_done = true;

	mt76_wr(dev, MT_BBP(AGC, 61), 0xFF64A4E2);
	mt76_wr(dev, MT_BBP(AGC, 7), 0x08081010);
	mt76_wr(dev, MT_BBP(AGC, 11), 0x00000404);
	mt76_wr(dev, MT_BBP(AGC, 2), 0x00007070);
	mt76_wr(dev, MT_TXOP_CTRL_CFG, 0x04101B3F);

	if (scan)
		return 0;

	dev->cal.low_gain = -1;
	mt76_phy_channel_calibrate(dev, true);
	mt76_get_agc_gain(dev, dev->cal.agc_gain_init);

	ieee80211_queue_delayed_work(dev->hw, &dev->cal_work,
				     MT_CALIBRATE_INTERVAL);

	return 0;
}

static void
mt76_phy_tssi_compensate(struct mt76_dev *dev)
{
	struct ieee80211_channel *chan = dev->chandef.chan;
	struct mt76_tx_power_info txp;
	struct mt76_tssi_comp t = {};

	if (!dev->cal.tssi_cal_done)
		return;

	if (dev->cal.tssi_comp_done) {
		/* TSSI trigger */
		t.cal_mode = BIT(0);
		mt76_mcu_tssi_comp(dev, &t);
	} else {
		if (!(mt76_rr(dev, MT_BBP(CORE, 34)) & BIT(4)))
			return;

		mt76_get_power_info(dev, &txp);

		if (mt76_ext_pa_enabled(dev, chan->band))
			t.pa_mode = 1;

		t.cal_mode = BIT(1);
		t.slope0 = txp.chain[0].tssi_slope;
		t.offset0 = txp.chain[0].tssi_offset;
		t.slope1 = txp.chain[1].tssi_slope;
		t.offset1 = txp.chain[1].tssi_offset;
		dev->cal.tssi_comp_done = true;
		mt76_mcu_tssi_comp(dev, &t);

		if (t.pa_mode || dev->cal.dpd_cal_done)
			return;

		msleep(10);
		mt76_mcu_calibrate(dev, MCU_CAL_DPD, chan->hw_value);
		dev->cal.dpd_cal_done = true;
	}
}

static void
mt76_phy_temp_compensate(struct mt76_dev *dev)
{
	struct mt76_temp_comp t;
	int temp, db_diff;

	if (mt76_get_temp_comp(dev, &t))
		return;

	temp = mt76_get_field(dev, MT_TEMP_SENSOR, MT_TEMP_SENSOR_VAL);
	temp -= t.temp_25_ref;
	temp = (temp * 1789) / 1000 + 25;
	dev->cal.temp = temp;

	if (temp > 25)
		db_diff = (temp - 25) / t.high_slope;
	else
		db_diff = (25 - temp) / t.low_slope;

	db_diff = min(db_diff, t.upper_bound);
	db_diff = max(db_diff, t.lower_bound);

	mt76_rmw_field(dev, MT_TX_ALC_CFG_1, MT_TX_ALC_CFG_1_TEMP_COMP,
		       db_diff * 2);
	mt76_rmw_field(dev, MT_TX_ALC_CFG_2, MT_TX_ALC_CFG_2_TEMP_COMP,
		       db_diff * 2);
}

void mt76_phy_calibrate(struct work_struct *work)
{
	struct mt76_dev *dev;

	dev = container_of(work, struct mt76_dev, cal_work.work);
	mt76_phy_channel_calibrate(dev, false);
	mt76_phy_tssi_compensate(dev);
	mt76_phy_temp_compensate(dev);
	mt76_phy_update_channel_gain(dev);
	ieee80211_queue_delayed_work(dev->hw, &dev->cal_work,
				     MT_CALIBRATE_INTERVAL);
}

int mt76_phy_start(struct mt76_dev *dev)
{
	int ret;

	ret = mt76_mcu_set_radio_state(dev, true);
	if (ret)
		return ret;

	mt76_mcu_load_cr(dev, MT_RF_BBP_CR, 0, 0);

	return ret;
}
