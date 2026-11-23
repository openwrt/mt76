// SPDX-License-Identifier: BSD-3-Clause-Clear
/*
 * Copyright (C) 2022 MediaTek Inc.
 */

#include <linux/relay.h>
#include "mt7996.h"
#include "eeprom.h"
#include "mcu.h"
#include "mac.h"

#define FW_BIN_LOG_MAGIC	0x44d9c99a

/** global debugfs **/

static int
mt7996_implicit_txbf_set(void *data, u64 val)
{
	struct mt7996_dev *dev = data;

	/* The existing connected stations shall reconnect to apply
	 * new implicit txbf configuration.
	 */
	dev->ibf = !!val;

	return mt7996_mcu_set_txbf(dev, BF_HW_EN_UPDATE);
}

static int
mt7996_implicit_txbf_get(void *data, u64 *val)
{
	struct mt7996_dev *dev = data;

	*val = dev->ibf;

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_implicit_txbf, mt7996_implicit_txbf_get,
			 mt7996_implicit_txbf_set, "%lld\n");

/* test knob of system error recovery */
static ssize_t
mt7996_sys_recovery_set(struct file *file, const char __user *user_buf,
			size_t count, loff_t *ppos)
{
	struct mt7996_dev *dev = file->private_data;
	char buf[16], *sep;
	int ret = 0;
	u16 band, val;

	if (count >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, user_buf, count))
		return -EFAULT;

	if (count && buf[count - 1] == '\n')
		buf[count - 1] = '\0';
	else
		buf[count] = '\0';

	sep = strchr(buf, ',');
	if (!sep)
		return -EINVAL;

	*sep = 0;
	if (kstrtou16(buf, 0, &band) || kstrtou16(sep + 1, 0, &val))
		return -EINVAL;

	switch (val) {
	case UNI_CMD_SER_QUERY:
		ret = mt7996_mcu_set_ser(dev, UNI_CMD_SER_QUERY, 0, band);
		break;
	case UNI_CMD_SER_SET_RECOVER_L1:
	case UNI_CMD_SER_SET_RECOVER_L2:
	case UNI_CMD_SER_SET_RECOVER_L3_RX_ABORT:
	case UNI_CMD_SER_SET_RECOVER_L3_TX_ABORT:
	case UNI_CMD_SER_SET_RECOVER_L3_TX_DISABLE:
	case UNI_CMD_SER_SET_RECOVER_L3_BF:
	case UNI_CMD_SER_SET_RECOVER_L4_MDP:
		ret = mt7996_mcu_set_ser(dev, UNI_CMD_SER_SET, BIT(val), band);
		if (ret)
			return ret;

		ret = mt7996_mcu_set_ser(dev, UNI_CMD_SER_TRIGGER, val, band);
		break;

	/* enable full chip reset */
	case UNI_CMD_SER_SET_RECOVER_FULL:
		mt76_set(dev, MT_WFDMA0_MCU_HOST_INT_ENA, MT_MCU_CMD_WDT_MASK);
		dev->recovery.state |= MT_MCU_CMD_WDT_MASK;
		mt7996_reset(dev);
		break;

	/* WARNING: trigger firmware crash */
	case UNI_CMD_SER_SET_SYSTEM_ASSERT:
		ret = mt7996_mcu_trigger_assert(dev);
		if (ret)
			return ret;
		break;
	default:
		break;
	}

	return ret ? ret : count;
}

static ssize_t
mt7996_sys_recovery_get(struct file *file, char __user *user_buf,
			size_t count, loff_t *ppos)
{
	struct mt7996_dev *dev = file->private_data;
	char *buff;
	int desc = 0;
	ssize_t ret;
	static const size_t bufsz = 1024;

	buff = kmalloc(bufsz, GFP_KERNEL);
	if (!buff)
		return -ENOMEM;

	/* HELP */
	desc += scnprintf(buff + desc, bufsz - desc,
			  "Please echo the correct value ...\n");
	desc += scnprintf(buff + desc, bufsz - desc,
			  "<band>,0: grab firmware transient SER state\n");
	desc += scnprintf(buff + desc, bufsz - desc,
			  "<band>,1 to 7: set the firmware SER enable mask to BIT(val) until the next firmware load, then request recovery val:\n");
	desc += scnprintf(buff + desc, bufsz - desc,
			  "  1: L1 recovery\n");
	desc += scnprintf(buff + desc, bufsz - desc,
			  "  2: L2 recovery\n");
	desc += scnprintf(buff + desc, bufsz - desc,
			  "  3: L3 rx abort\n");
	desc += scnprintf(buff + desc, bufsz - desc,
			  "  4: L3 tx abort\n");
	desc += scnprintf(buff + desc, bufsz - desc,
			  "  5: L3 tx disable\n");
	desc += scnprintf(buff + desc, bufsz - desc,
			  "  6: L3 bf recovery\n");
	desc += scnprintf(buff + desc, bufsz - desc,
			  "  7: L4 mdp recovery\n");
	desc += scnprintf(buff + desc, bufsz - desc,
			  "<band>,8: trigger system error full recovery\n");
	desc += scnprintf(buff + desc, bufsz - desc,
			  "<band>,9: trigger firmware crash\n");

	/* SER statistics */
	desc += scnprintf(buff + desc, bufsz - desc,
			  "\nlet's dump firmware SER statistics...\n");
	desc += scnprintf(buff + desc, bufsz - desc,
			  "::E  R , SER_STATUS        = 0x%08x\n",
			  mt76_rr(dev, MT_SWDEF_SER_STATS));
	desc += scnprintf(buff + desc, bufsz - desc,
			  "::E  R , SER_PLE_ERR       = 0x%08x\n",
			  mt76_rr(dev, MT_SWDEF_PLE_STATS));
	desc += scnprintf(buff + desc, bufsz - desc,
			  "::E  R , SER_PLE_ERR_1     = 0x%08x\n",
			  mt76_rr(dev, MT_SWDEF_PLE1_STATS));
	desc += scnprintf(buff + desc, bufsz - desc,
			  "::E  R , SER_PLE_ERR_AMSDU = 0x%08x\n",
			  mt76_rr(dev, MT_SWDEF_PLE_AMSDU_STATS));
	desc += scnprintf(buff + desc, bufsz - desc,
			  "::E  R , SER_PSE_ERR       = 0x%08x\n",
			  mt76_rr(dev, MT_SWDEF_PSE_STATS));
	desc += scnprintf(buff + desc, bufsz - desc,
			  "::E  R , SER_PSE_ERR_1     = 0x%08x\n",
			  mt76_rr(dev, MT_SWDEF_PSE1_STATS));
	desc += scnprintf(buff + desc, bufsz - desc,
			  "::E  R , SER_LMAC_WISR6_B0 = 0x%08x\n",
			  mt76_rr(dev, MT_SWDEF_LAMC_WISR6_BN0_STATS));
	desc += scnprintf(buff + desc, bufsz - desc,
			  "::E  R , SER_LMAC_WISR6_B1 = 0x%08x\n",
			  mt76_rr(dev, MT_SWDEF_LAMC_WISR6_BN1_STATS));
	desc += scnprintf(buff + desc, bufsz - desc,
			  "::E  R , SER_LMAC_WISR6_B2 = 0x%08x\n",
			  mt76_rr(dev, MT_SWDEF_LAMC_WISR6_BN2_STATS));
	desc += scnprintf(buff + desc, bufsz - desc,
			  "::E  R , SER_LMAC_WISR7_B0 = 0x%08x\n",
			  mt76_rr(dev, MT_SWDEF_LAMC_WISR7_BN0_STATS));
	desc += scnprintf(buff + desc, bufsz - desc,
			  "::E  R , SER_LMAC_WISR7_B1 = 0x%08x\n",
			  mt76_rr(dev, MT_SWDEF_LAMC_WISR7_BN1_STATS));
	desc += scnprintf(buff + desc, bufsz - desc,
			  "::E  R , SER_LMAC_WISR7_B2 = 0x%08x\n",
			  mt76_rr(dev, MT_SWDEF_LAMC_WISR7_BN2_STATS));
	desc += scnprintf(buff + desc, bufsz - desc,
			  "\nSYS_RESET_COUNT: WM %d, WA %d\n",
			  dev->recovery.wm_reset_count,
			  dev->recovery.wa_reset_count);

	ret = simple_read_from_buffer(user_buf, count, ppos, buff, desc);
	kfree(buff);
	return ret;
}

static const struct file_operations mt7996_sys_recovery_ops = {
	.write = mt7996_sys_recovery_set,
	.read = mt7996_sys_recovery_get,
	.open = simple_open,
	.llseek = default_llseek,
};

static int
mt7996_radar_trigger(void *data, u64 val)
{
#define RADAR_MAIN_CHAIN	1
#define RADAR_BACKGROUND	2
	struct mt7996_dev *dev = data;
	struct mt7996_phy *phy = mt7996_band_phy(dev, NL80211_BAND_5GHZ);
	struct cfg80211_chan_def *chandef;
	int rdd_idx, ret;

	if (!phy || !val || val > RADAR_BACKGROUND)
		return -EINVAL;

	if (test_bit(MT76_SCANNING, &phy->mt76->state))
		return -EBUSY;

	if (val == RADAR_BACKGROUND) {
		if (!dev->rdd2_phy || !cfg80211_chandef_valid(&dev->rdd2_chandef)) {
			dev_err(dev->mt76.dev, "Background radar is not enabled\n");
			return -EINVAL;
		}
		chandef = &dev->rdd2_chandef;
	} else {
		chandef = &phy->mt76->chandef;
	}

	rdd_idx = mt7996_get_rdd_idx(phy, val == RADAR_BACKGROUND);
	if (rdd_idx < 0) {
		dev_err(dev->mt76.dev, "No RDD found\n");
		return -EINVAL;
	}

	ret = cfg80211_chandef_dfs_required(dev->mt76.hw->wiphy, chandef,
					    NL80211_IFTYPE_AP);
	if (ret <= 0)
		return ret;

	return mt7996_mcu_rdd_cmd(dev, RDD_RADAR_EMULATE, rdd_idx, 0);
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_radar_trigger, NULL,
			 mt7996_radar_trigger, "%lld\n");

static int
mt7996_rdd_monitor(struct seq_file *s, void *data)
{
	struct mt7996_dev *dev = dev_get_drvdata(s->private);
	struct cfg80211_chan_def *chandef = &dev->rdd2_chandef;
	const char *bw;
	int ret = 0;

	mutex_lock(&dev->mt76.mutex);

	if (!cfg80211_chandef_valid(chandef)) {
		ret = -EINVAL;
		goto out;
	}

	if (!dev->rdd2_phy) {
		seq_puts(s, "not running\n");
		goto out;
	}

	switch (chandef->width) {
	case NL80211_CHAN_WIDTH_40:
		bw = "40";
		break;
	case NL80211_CHAN_WIDTH_80:
		bw = "80";
		break;
	case NL80211_CHAN_WIDTH_160:
		bw = "160";
		break;
	case NL80211_CHAN_WIDTH_80P80:
		bw = "80P80";
		break;
	default:
		bw = "20";
		break;
	}

	seq_printf(s, "channel %d (%d MHz) width %s MHz center1: %d MHz\n",
		   chandef->chan->hw_value, chandef->chan->center_freq,
		   bw, chandef->center_freq1);
out:
	mutex_unlock(&dev->mt76.mutex);

	return ret;
}

static int
mt7996_fw_debug_wm_set(void *data, u64 val)
{
	struct mt7996_dev *dev = data;

	dev->fw_debug_wm = val ? MCU_FW_LOG_TO_HOST : 0;

	return mt7996_mcu_fw_debug_set(dev, MCU_FW_LOG_WM, false);
}

static int
mt7996_fw_debug_wm_get(void *data, u64 *val)
{
	struct mt7996_dev *dev = data;

	*val = dev->fw_debug_wm;

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_fw_debug_wm, mt7996_fw_debug_wm_get,
			 mt7996_fw_debug_wm_set, "%lld\n");

static int
mt7996_fw_debug_wa_set(void *data, u64 val)
{
	struct mt7996_dev *dev = data;

	dev->fw_debug_wa = val ? MCU_FW_LOG_TO_HOST : 0;

	return mt7996_mcu_fw_debug_set(dev, MCU_FW_LOG_WA, false);
}

static int
mt7996_fw_debug_wa_get(void *data, u64 *val)
{
	struct mt7996_dev *dev = data;

	*val = dev->fw_debug_wa;

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_fw_debug_wa, mt7996_fw_debug_wa_get,
			 mt7996_fw_debug_wa_set, "%lld\n");

static struct dentry *
create_buf_file_cb(const char *filename, struct dentry *parent, umode_t mode,
		   struct rchan_buf *buf, int *is_global)
{
	struct dentry *f;

	f = debugfs_create_file("fwlog_data", mode, parent, buf,
				&relay_file_operations);
	if (IS_ERR(f))
		return NULL;

	*is_global = 1;

	return f;
}

static int
remove_buf_file_cb(struct dentry *f)
{
	debugfs_remove(f);

	return 0;
}

static int
mt7996_fw_debug_bin_set(void *data, u64 val)
{
	static struct rchan_callbacks relay_cb = {
		.create_buf_file = create_buf_file_cb,
		.remove_buf_file = remove_buf_file_cb,
	};
	struct mt7996_dev *dev = data;

	if (!dev->relay_fwlog)
		dev->relay_fwlog = relay_open("fwlog_data", dev->debugfs_dir,
					      1500, 512, &relay_cb, NULL);
	if (!dev->relay_fwlog)
		return -ENOMEM;

	dev->fw_debug_bin = val;

	relay_reset(dev->relay_fwlog);

	return mt7996_fw_debug_wm_set(dev, dev->fw_debug_wm);
}

static int
mt7996_fw_debug_bin_get(void *data, u64 *val)
{
	struct mt7996_dev *dev = data;

	*val = dev->fw_debug_bin;

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_fw_debug_bin, mt7996_fw_debug_bin_get,
			 mt7996_fw_debug_bin_set, "%lld\n");

static int
mt7996_fw_util_wa_show(struct seq_file *file, void *data)
{
	struct mt7996_dev *dev = file->private;

	if (dev->fw_debug_wa)
		return mt7996_mcu_wa_cmd(dev, MCU_WA_PARAM_CMD(QUERY),
					 MCU_WA_PARAM_CPU_UTIL, 0, 0);

	return 0;
}

DEFINE_SHOW_ATTRIBUTE(mt7996_fw_util_wa);

static int
mt7996_fw_debug_module_set(void *data, u64 val)
{
	struct mt7996_dev *dev = data;

	if (val > U32_MAX)
		return -EINVAL;

	dev->fw_debug_module = val;

	return 0;
}

static int
mt7996_fw_debug_module_get(void *data, u64 *val)
{
	struct mt7996_dev *dev = data;

	*val = dev->fw_debug_module;

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_fw_debug_module, mt7996_fw_debug_module_get,
			 mt7996_fw_debug_module_set, "%lld\n");

static int
mt7996_fw_debug_level_set(void *data, u64 val)
{
	struct mt7996_dev *dev = data;
	int ret;

	if (val > U8_MAX)
		return -EINVAL;

	ret = mt7996_mcu_fw_dbg_ctrl(dev, dev->fw_debug_module, val);
	if (ret)
		return ret;

	dev->fw_debug_level = val;

	return 0;
}

static int
mt7996_fw_debug_level_get(void *data, u64 *val)
{
	struct mt7996_dev *dev = data;

	*val = dev->fw_debug_level;

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_fw_debug_level, mt7996_fw_debug_level_get,
			 mt7996_fw_debug_level_set, "%lld\n");

#define MT7996_FW_LOG_LEN	32

static u32
mt7996_fw_wm_excp_log_read(struct mt7996_dev *dev, u32 ctrl, u32 log_base,
			   u32 *log)
{
	u32 val = mt76_rr(dev, ctrl);
	int i;

	for (i = 0; i < MT7996_FW_LOG_LEN; i++)
		log[i] = mt76_rr(dev, log_base + i * 4);

	return val;
}

static u32
mt7996_fw_wm_dbg_log_read(struct mt7996_dev *dev, u32 sel, u32 out, u32 *log)
{
	u32 val;
	int i;

	mt76_wr(dev, sel, MT7996_FW_LOG_LEN);
	val = mt76_rr(dev, out);

	for (i = 0; i < MT7996_FW_LOG_LEN; i++) {
		mt76_wr(dev, sel, i);
		log[i] = mt76_rr(dev, out);
	}

	return val;
}

static void
mt7996_fw_wm_log_show(struct seq_file *s, const char *name, u32 ctrl, u32 idx,
		      const u32 *log)
{
	int i;

	seq_printf(s, "%s log control 0x%08x, latest index %u\n",
		   name, ctrl, idx);
	for (i = 0; i < MT7996_FW_LOG_LEN; i++)
		seq_printf(s, "\t%s log %2d: 0x%08x\n", name, i, log[i]);
}

static int
mt7996_fw_wm_info_show(struct seq_file *s, void *data)
{
	struct mt7996_dev *dev = dev_get_drvdata(s->private);
	u32 pc_log[MT7996_FW_LOG_LEN], lr_log[MT7996_FW_LOG_LEN];
	u32 pc_cur[10], pc_ctrl, lr_ctrl, pc_idx, lr_idx;
	int i;

	mutex_lock(&dev->dump_mutex);

	mt76_wr(dev, MT_CONN_DBG_CTL_OUT_SEL, 0);
	mt76_wr(dev, MT_CONN_DBG_CTL_PC_LOG_SEL, 0x3f);
	for (i = 0; i < ARRAY_SIZE(pc_cur); i++)
		pc_cur[i] = mt76_rr(dev, MT_CONN_DBG_CTL_PC_LOG);

	if (is_mt7996(&dev->mt76)) {
		bool running = !mt76_rr(dev, MT_FW_ASSERT_CNT);

		if (running)
			mt7996_wm_excp_log_set(dev, false);
		pc_ctrl = mt7996_fw_wm_excp_log_read(dev, MT_MCU_WM_EXCP_PC_CTRL,
						     MT_MCU_WM_EXCP_PC_LOG,
						     pc_log);
		lr_ctrl = mt7996_fw_wm_excp_log_read(dev, MT_MCU_WM_EXCP_LR_CTRL,
						     MT_MCU_WM_EXCP_LR_LOG,
						     lr_log);
		if (running)
			mt7996_wm_excp_log_set(dev, true);
		pc_idx = u32_get_bits(pc_ctrl, MT_MCU_WM_EXCP_CTRL_IDX);
		lr_idx = u32_get_bits(lr_ctrl, MT_MCU_WM_EXCP_CTRL_IDX);
	} else {
		pc_ctrl = mt7996_fw_wm_dbg_log_read(dev,
						    MT_CONN_DBG_CTL_PC_LOG_SEL,
						    MT_CONN_DBG_CTL_PC_LOG,
						    pc_log);
		lr_ctrl = mt7996_fw_wm_dbg_log_read(dev,
						    MT_CONN_DBG_CTL_GPR_LOG_SEL,
						    MT_CONN_DBG_CTL_GPR_BUS_OUT_LOG,
						    lr_log);
		pc_idx = u32_get_bits(pc_ctrl, MT_CONN_DBG_CTL_LOG_IDX);
		lr_idx = u32_get_bits(lr_ctrl, MT_CONN_DBG_CTL_LOG_IDX);
	}

	mutex_unlock(&dev->dump_mutex);

	for (i = 0; i < ARRAY_SIZE(pc_cur); i++)
		seq_printf(s, "current PC: 0x%08x\n", pc_cur[i]);

	mt7996_fw_wm_log_show(s, "PC", pc_ctrl, pc_idx, pc_log);
	mt7996_fw_wm_log_show(s, "LR", lr_ctrl, lr_idx, lr_log);

	return 0;
}

static void
mt7996_ampdu_stat_read_phy(struct mt7996_phy *phy, struct seq_file *file)
{
	struct mt7996_dev *dev = phy->dev;
	int bound[15], range[8], i;
	u8 band_idx = phy->mt76->band_idx;

	/* Tx ampdu stat */
	for (i = 0; i < ARRAY_SIZE(range); i++)
		range[i] = mt76_rr(dev, MT_MIB_ARNG(band_idx, i));

	for (i = 0; i < ARRAY_SIZE(bound); i++)
		bound[i] = MT_MIB_ARNCR_RANGE(range[i / 2], i % 2) + 1;

	seq_printf(file, "\nPhy %s, Phy band %d\n",
		   wiphy_name(phy->mt76->hw->wiphy), band_idx);

	seq_printf(file, "Length: %8d | ", bound[0]);
	for (i = 0; i < ARRAY_SIZE(bound) - 1; i++)
		seq_printf(file, "%3d -%3d | ",
			   bound[i] + 1, bound[i + 1]);

	seq_puts(file, "\nCount:  ");
	for (i = 0; i < ARRAY_SIZE(bound); i++)
		seq_printf(file, "%8d | ", phy->mt76->aggr_stats[i]);
	seq_puts(file, "\n");

	seq_printf(file, "BA miss count: %d\n", phy->mib.ba_miss_cnt);
}

static void
mt7996_txbf_stat_read_phy(struct mt7996_phy *phy, struct seq_file *s)
{
	struct mt76_mib_stats *mib = &phy->mib;
	static const char * const bw[] = {
		"BW20", "BW40", "BW80", "BW160", "BW320"
	};

	/* Tx Beamformer monitor */
	seq_puts(s, "\nTx Beamformer applied PPDU counts: ");

	seq_printf(s, "iBF: %d, eBF: %d\n",
		   mib->tx_bf_ibf_ppdu_cnt,
		   mib->tx_bf_ebf_ppdu_cnt);

	/* Tx Beamformer Rx feedback monitor */
	seq_puts(s, "Tx Beamformer Rx feedback statistics: ");

	seq_printf(s, "All: %d, EHT: %d, HE: %d, VHT: %d, HT: %d, ",
		   mib->tx_bf_rx_fb_all_cnt,
		   mib->tx_bf_rx_fb_eht_cnt,
		   mib->tx_bf_rx_fb_he_cnt,
		   mib->tx_bf_rx_fb_vht_cnt,
		   mib->tx_bf_rx_fb_ht_cnt);

	seq_printf(s, "%s, NC: %d, NR: %d\n",
		   bw[mib->tx_bf_rx_fb_bw],
		   mib->tx_bf_rx_fb_nc_cnt,
		   mib->tx_bf_rx_fb_nr_cnt);

	/* Tx Beamformee Rx NDPA & Tx feedback report */
	seq_printf(s, "Tx Beamformee successful feedback frames: %d\n",
		   mib->tx_bf_fb_cpl_cnt);
	seq_printf(s, "Tx Beamformee feedback triggered counts: %d\n",
		   mib->tx_bf_fb_trig_cnt);

	/* Tx SU & MU counters */
	seq_printf(s, "Tx multi-user Beamforming counts: %d\n",
		   mib->tx_mu_bf_cnt);
	seq_printf(s, "Tx multi-user MPDU counts: %d\n", mib->tx_mu_mpdu_cnt);
	seq_printf(s, "Tx multi-user successful MPDU counts: %d\n",
		   mib->tx_mu_acked_mpdu_cnt);
	seq_printf(s, "Tx single-user successful MPDU counts: %d\n",
		   mib->tx_su_acked_mpdu_cnt);

	seq_puts(s, "\n");
}

static void
mt7996_tx_stats_show_phy(struct seq_file *file, struct mt7996_phy *phy)
{
	struct mt76_mib_stats *mib = &phy->mib;
	u32 attempts, success, per;
	int i;

	mt7996_mac_update_stats(phy);
	mt7996_ampdu_stat_read_phy(phy, file);

	attempts = mib->tx_mpdu_attempts_cnt;
	success = mib->tx_mpdu_success_cnt;
	per = attempts ? 100 - success * 100 / attempts : 100;
	seq_printf(file, "Tx attempts: %8u (MPDUs)\n", attempts);
	seq_printf(file, "Tx success: %8u (MPDUs)\n", success);
	seq_printf(file, "Tx PER: %u%%\n", per);

	mt7996_txbf_stat_read_phy(phy, file);

	/* Tx amsdu info */
	seq_puts(file, "Tx MSDU statistics:\n");
	for (i = 0; i < ARRAY_SIZE(mib->tx_amsdu); i++) {
		seq_printf(file, "AMSDU pack count of %d MSDU in TXD: %8d ",
			   i + 1, mib->tx_amsdu[i]);
		if (mib->tx_amsdu_cnt)
			seq_printf(file, "(%3d%%)\n",
				   mib->tx_amsdu[i] * 100 / mib->tx_amsdu_cnt);
		else
			seq_puts(file, "\n");
	}
}

static int
mt7996_tx_stats_show(struct seq_file *file, void *data)
{
	struct mt7996_dev *dev = file->private;
	struct mt7996_phy *phy = &dev->phy;

	mutex_lock(&dev->mt76.mutex);

	mt7996_tx_stats_show_phy(file, phy);
	phy = mt7996_phy2(dev);
	if (phy)
		mt7996_tx_stats_show_phy(file, phy);
	phy = mt7996_phy3(dev);
	if (phy)
		mt7996_tx_stats_show_phy(file, phy);

	mutex_unlock(&dev->mt76.mutex);

	return 0;
}

DEFINE_SHOW_ATTRIBUTE(mt7996_tx_stats);

enum {
	MT_QUEUE_PORT_HIF,
	MT_QUEUE_PORT_CPU,
	MT_QUEUE_PORT_LMAC,
	MT_QUEUE_PORT_PLE_CTRL,
};

struct mt7996_hw_queue {
	const char *name;
	u8 pid;
	u8 qid;
	u8 tgid;
};

#define MT7996_HW_QUEUE(_name, _pid, _qid, _tgid)	\
	{ .name = _name, .pid = MT_QUEUE_PORT_##_pid, .qid = _qid, .tgid = _tgid }

static const struct mt7996_hw_queue mt7996_ple_queues[32] = {
	[0] = MT7996_HW_QUEUE("CPU Q0", CPU, 0, 0),
	[1] = MT7996_HW_QUEUE("CPU Q1", CPU, 1, 0),
	[2] = MT7996_HW_QUEUE("CPU Q2", CPU, 2, 0),
	[3] = MT7996_HW_QUEUE("CPU Q3", CPU, 3, 0),
	[4] = MT7996_HW_QUEUE("ALTX Q0", LMAC, 0x10, 0),
	[5] = MT7996_HW_QUEUE("BMC Q0", LMAC, 0x11, 0),
	[6] = MT7996_HW_QUEUE("BCN Q0", LMAC, 0x12, 0),
	[7] = MT7996_HW_QUEUE("PSMP Q0", LMAC, 0x13, 0),
	[8] = MT7996_HW_QUEUE("ALTX Q1", LMAC, 0x10, 1),
	[9] = MT7996_HW_QUEUE("BMC Q1", LMAC, 0x11, 1),
	[10] = MT7996_HW_QUEUE("BCN Q1", LMAC, 0x12, 1),
	[11] = MT7996_HW_QUEUE("PSMP Q1", LMAC, 0x13, 1),
	[12] = MT7996_HW_QUEUE("ALTX Q2", LMAC, 0x1c, 2),
	[13] = MT7996_HW_QUEUE("BMC Q2", LMAC, 0x1d, 2),
	[14] = MT7996_HW_QUEUE("BCN Q2", LMAC, 0x1e, 2),
	[15] = MT7996_HW_QUEUE("PSMP Q2", LMAC, 0x1f, 2),
	[16] = MT7996_HW_QUEUE("NAF Q", LMAC, 0x18, 0),
	[17] = MT7996_HW_QUEUE("NBCN Q", LMAC, 0x19, 0),
	[20] = MT7996_HW_QUEUE("FIXFID Q", LMAC, 0x1a, 0),
	[28] = MT7996_HW_QUEUE("RLS4 Q", PLE_CTRL, 0x7c, 0),
	[29] = MT7996_HW_QUEUE("RLS3 Q", PLE_CTRL, 0x7d, 0),
	[30] = MT7996_HW_QUEUE("RLS2 Q", PLE_CTRL, 0x7e, 0),
	[31] = MT7996_HW_QUEUE("RLS Q", PLE_CTRL, 0x7f, 0),
};

static const struct mt7996_hw_queue mt7996_pse_queues[32] = {
	[0] = MT7996_HW_QUEUE("CPU Q0", CPU, 0, 0),
	[1] = MT7996_HW_QUEUE("CPU Q1", CPU, 1, 0),
	[2] = MT7996_HW_QUEUE("CPU Q2", CPU, 2, 0),
	[3] = MT7996_HW_QUEUE("CPU Q3", CPU, 3, 0),
	[16] = MT7996_HW_QUEUE("LMAC Q", LMAC, 0, 0),
	[17] = MT7996_HW_QUEUE("MDP TX Q0", LMAC, 1, 0),
	[18] = MT7996_HW_QUEUE("MDP RX Q", LMAC, 2, 0),
	[19] = MT7996_HW_QUEUE("SEC TX Q0", LMAC, 3, 0),
	[20] = MT7996_HW_QUEUE("SEC RX Q", LMAC, 4, 0),
	[21] = MT7996_HW_QUEUE("SFD PARK Q", LMAC, 5, 0),
	[22] = MT7996_HW_QUEUE("MDP TXIOC Q0", LMAC, 6, 0),
	[23] = MT7996_HW_QUEUE("MDP RXIOC Q0", LMAC, 7, 0),
	[24] = MT7996_HW_QUEUE("MDP TX Q1", LMAC, 0x11, 0),
	[25] = MT7996_HW_QUEUE("SEC TX Q1", LMAC, 0x13, 0),
	[26] = MT7996_HW_QUEUE("MDP TXIOC Q1", LMAC, 0x16, 0),
	[27] = MT7996_HW_QUEUE("MDP RXIOC Q1", LMAC, 0x17, 0),
	[28] = MT7996_HW_QUEUE("CPU Q4", CPU, 4, 0),
	[31] = MT7996_HW_QUEUE("RLS Q", PLE_CTRL, 0x1f, 0),
};

static const struct mt7996_hw_queue mt7996_pse_queues_1[32] = {
	[0] = MT7996_HW_QUEUE("MDP TDPIOC Q0", LMAC, 0x08, 0),
	[1] = MT7996_HW_QUEUE("MDP RDPIOC Q0", LMAC, 0x09, 0),
	[2] = MT7996_HW_QUEUE("MDP TDPIOC Q1", LMAC, 0x18, 0),
	[3] = MT7996_HW_QUEUE("MDP RDPIOC Q1", LMAC, 0x19, 0),
	[4] = MT7996_HW_QUEUE("MDP TDPIOC Q2", LMAC, 0x28, 0),
	[5] = MT7996_HW_QUEUE("MDP RDPIOC Q2", LMAC, 0x29, 0),
	[7] = MT7996_HW_QUEUE("MDP RDPIOC Q3", LMAC, 0x39, 0),
	[8] = MT7996_HW_QUEUE("MDP TX Q2", LMAC, 0x21, 0),
	[9] = MT7996_HW_QUEUE("SEC TX Q2", LMAC, 0x23, 0),
	[10] = MT7996_HW_QUEUE("MDP TXIOC Q2", LMAC, 0x26, 0),
	[11] = MT7996_HW_QUEUE("MDP RXIOC Q2", LMAC, 0x27, 0),
	[15] = MT7996_HW_QUEUE("MDP RXIOC Q3", LMAC, 0x37, 0),
	[16] = MT7996_HW_QUEUE("HIF Q0", HIF, 0, 0),
	[17] = MT7996_HW_QUEUE("HIF Q1", HIF, 1, 0),
	[18] = MT7996_HW_QUEUE("HIF Q2", HIF, 2, 0),
	[19] = MT7996_HW_QUEUE("HIF Q3", HIF, 3, 0),
	[20] = MT7996_HW_QUEUE("HIF Q4", HIF, 4, 0),
	[21] = MT7996_HW_QUEUE("HIF Q5", HIF, 5, 0),
	[22] = MT7996_HW_QUEUE("HIF Q6", HIF, 6, 0),
	[23] = MT7996_HW_QUEUE("HIF Q7", HIF, 7, 0),
	[24] = MT7996_HW_QUEUE("HIF Q8", HIF, 8, 0),
	[25] = MT7996_HW_QUEUE("HIF Q9", HIF, 9, 0),
	[26] = MT7996_HW_QUEUE("HIF Q10", HIF, 10, 0),
	[27] = MT7996_HW_QUEUE("HIF Q11", HIF, 11, 0),
	[28] = MT7996_HW_QUEUE("HIF Q12", HIF, 12, 0),
	[29] = MT7996_HW_QUEUE("HIF Q13", HIF, 13, 0),
};

static void
mt7996_fl_queue_show(struct seq_file *s, struct mt7996_dev *dev, bool pse,
		     const struct mt7996_hw_queue *q, u16 wlan_idx)
{
	static DEFINE_SPINLOCK(lock);
	u32 val, fid, num;

	val = MT_FL_Q0_CTRL_EXECUTE |
	      u32_encode_bits(q->qid, MT_FL_Q0_CTRL_QID) |
	      u32_encode_bits(q->tgid, MT_FL_Q0_CTRL_TGID) |
	      u32_encode_bits(q->pid, MT_FL_Q0_CTRL_PID) |
	      u32_encode_bits(wlan_idx, MT_FL_Q0_CTRL_WLAN_IDX);

	spin_lock(&lock);
	mt76_wr(dev, pse ? MT_PSE_FL_Q0_CTRL : MT_FL_Q0_CTRL, val);
	fid = mt76_rr(dev, pse ? MT_PSE_FL_Q2_CTRL : MT_FL_Q2_CTRL);
	num = mt76_rr(dev, pse ? MT_PSE_FL_Q3_CTRL : MT_FL_Q3_CTRL);
	spin_unlock(&lock);

	seq_printf(s, "tail/head fid 0x%04x/0x%04x, packets %u",
		   u32_get_bits(fid, MT_FL_Q2_CTRL_TAIL_FID),
		   u32_get_bits(fid, MT_FL_Q2_CTRL_HEAD_FID),
		   u32_get_bits(num, MT_FL_Q3_CTRL_PKT_NUM));
}

static void
mt7996_fl_queues_show(struct seq_file *s, struct mt7996_dev *dev, bool pse,
		      const struct mt7996_hw_queue *queues, unsigned long empty)
{
	unsigned int i;

	for_each_clear_bit(i, &empty, 32) {
		if (!queues[i].name)
			continue;

		seq_printf(s, "\t%s: ", queues[i].name);
		mt7996_fl_queue_show(s, dev, pse, &queues[i], 0);
		seq_puts(s, "\n");
	}
}

static void
mt7996_pbuf_show(struct seq_file *s, const char *name, u32 ctrl, bool pse)
{
	u32 page_size = pse ? 128 : 64;

	if (ctrl & MT_PBUF_CTRL_PAGE_SIZE)
		page_size *= 2;

	seq_printf(s, "%s packet buffer control 0x%08x\n", name, ctrl);
	seq_printf(s, "\tpage size %u bytes, offset %u (unit %u KB), %u pages\n",
		   page_size, u32_get_bits(ctrl, MT_PBUF_CTRL_OFFSET),
		   pse ? 64 : 2, u32_get_bits(ctrl, MT_PBUF_CTRL_PAGE_NUM));
}

static void
mt7996_freepg_show(struct seq_file *s, u32 cnt, u32 head_tail)
{
	seq_printf(s, "\tfree pages 0x%04x, free for all 0x%04x, head/tail 0x%04x/0x%04x\n",
		   u32_get_bits(cnt, MT_FREEPG_CNT_FREE),
		   u32_get_bits(cnt, MT_FREEPG_CNT_FFA),
		   u32_get_bits(head_tail, MT_FREEPG_HEAD_TAIL_HEAD),
		   u32_get_bits(head_tail, MT_FREEPG_HEAD_TAIL_TAIL));
}

static void
mt7996_pg_group_show(struct seq_file *s, const char *name, u32 group, u32 info)
{
	seq_printf(s, "\t%s group: max/min quota 0x%04x/0x%04x, used/reserved 0x%04x/0x%04x\n",
		   name, u32_get_bits(group, MT_PG_GROUP_MAX_QUOTA),
		   u32_get_bits(group, MT_PG_GROUP_MIN_QUOTA),
		   u32_get_bits(info, MT_PG_INFO_SRC_CNT),
		   u32_get_bits(info, MT_PG_INFO_RSV_CNT));
}

static unsigned int
mt7996_ple_sta_cr_num(struct mt7996_dev *dev)
{
	return is_mt7996(&dev->mt76) ? 34 : 17;
}

static unsigned int
mt7996_ple_sta_cr(struct mt7996_dev *dev, unsigned int n)
{
	/* MT7992 and MT7990 keep the last station group in the
	 * extension register that follows the first 32 groups
	 */
	if (!is_mt7996(&dev->mt76) && n == mt7996_ple_sta_cr_num(dev) - 1)
		return 32;

	return n;
}

static unsigned int
mt7996_drr_twt_cr(struct mt7996_dev *dev, unsigned int n)
{
	/* MT7990 has 18 TWT station registers, one more than it has
	 * station groups, so no station group uses its extension register
	 */
	if (is_mt7990(&dev->mt76))
		return n;

	return mt7996_ple_sta_cr(dev, n);
}

static void
mt7996_ple_sta_state_show(struct seq_file *s, struct mt7996_dev *dev, u8 ac,
			  unsigned int cr, u16 wlan_idx)
{
	unsigned int twt_cr = mt7996_drr_twt_cr(dev, wlan_idx / 32);
	u32 bit = BIT(wlan_idx % 32);
	struct mt76_wcid *wcid;
	int wmm_idx = -1;
	u8 band;

	rcu_read_lock();

	wcid = mt76_wcid_ptr(dev, wlan_idx);
	if (!wcid) {
		rcu_read_unlock();
		seq_puts(s, ", no station");
		return;
	}

	band = wcid->phy_idx;
	if (wcid->sta) {
		struct mt7996_sta_link *msta_link;
		struct mt76_vif_link *mlink;

		msta_link = container_of(wcid, struct mt7996_sta_link, wcid);
		mlink = rcu_dereference(msta_link->sta->vif->mt76.link[wcid->link_id]);
		if (mlink)
			wmm_idx = mlink->wmm_idx;
	}

	rcu_read_unlock();

	seq_printf(s, ", band %u", band);
	if (wmm_idx >= 0)
		seq_printf(s, ", WMM %d", wmm_idx);

	mt76_wr(dev, MT_DRR_SBRR, u32_encode_bits(band, MT_DRR_SBRR_TARGET_BAND));
	if (mt76_rr(dev, MT_DRR_STA_PAUSE(ac, cr)) & bit)
		seq_puts(s, ", paused");
	if (mt76_rr(dev, MT_DRR_TWT_STA_MAP(twt_cr)) & bit)
		seq_puts(s, ", TWT paused");
}

static void
mt7996_ple_sta_queues_show(struct seq_file *s, struct mt7996_dev *dev)
{
	unsigned int n, num = mt7996_ple_sta_cr_num(dev);

	for (n = 0; n < num * IEEE80211_NUM_ACS; n++) {
		struct mt7996_hw_queue q = {
			.pid = MT_QUEUE_PORT_LMAC,
			.qid = n / num,
		};
		unsigned int i, cr = mt7996_ple_sta_cr(dev, n % num);
		unsigned long empty;

		empty = mt76_rr(dev, MT_PLE_AC_QEMPTY(q.qid, cr));
		for_each_clear_bit(i, &empty, 32) {
			u16 wlan_idx = (n % num) * 32 + i;

			seq_printf(s, "\tSTA %u AC%u: ", wlan_idx, q.qid);
			mt7996_fl_queue_show(s, dev, false, &q, wlan_idx);
			mt7996_ple_sta_state_show(s, dev, q.qid, cr, wlan_idx);
			seq_puts(s, "\n");
		}
	}
}

static void
mt7996_ple_txcmd_queues_show(struct seq_file *s, struct mt7996_dev *dev)
{
	static const char * const names[] = {
		"ALTX Q0", "TF Q0", "TWT TSF-TF Q0", "TWT DL Q0", "TWT UL Q0"
	};
	u8 band;

	for (band = MT_BAND0; band < __MT_MAX_BAND; band++) {
		struct mt7996_hw_queue q = { .pid = MT_QUEUE_PORT_LMAC };
		unsigned long empty;
		unsigned int i;

		if (!mt7996_band_valid(dev, band))
			continue;

		empty = mt76_rr(dev, MT_PLE_NATIVE_TXCMD_Q_EMPTY(band));
		seq_printf(s, "band %u TXCMD queue empty 0x%08x, native 0x%08lx\n",
			   band, mt76_rr(dev, MT_PLE_TXCMD_Q_EMPTY(band)), empty);

		for_each_clear_bit(i, &empty, 16 + ARRAY_SIZE(names)) {
			if (i < 16) {
				q.qid = 0x40 + band * 16 + i;
				seq_printf(s, "\tAC%u%uQ: ", i / 4, i % 4);
			} else {
				q.qid = 0x70 + band * ARRAY_SIZE(names) + i - 16;
				seq_printf(s, "\t%s: ", names[i - 16]);
			}

			mt7996_fl_queue_show(s, dev, false, &q, 0);
			seq_puts(s, "\n");
		}
	}
}

static int
mt7996_ple_info_show(struct seq_file *s, void *data)
{
	static const struct {
		const char *name;
		u32 group;
		u32 info;
	} groups[] = {
		{ "HIF", MT_PLE_PG_HIF_GROUP, MT_PLE_HIF_PG_INFO },
		{ "HIF WM TXD", MT_PLE_PG_HIF_WMTXD_GROUP,
		  MT_PLE_HIF_WMTXD_PG_INFO },
		{ "HIF TXCMD", MT_PLE_PG_HIF_TXCMD_GROUP,
		  MT_PLE_HIF_TXCMD_PG_INFO },
		{ "CPU", MT_PLE_PG_CPU_GROUP, MT_PLE_CPU_PG_INFO },
	};
	struct mt7996_dev *dev = dev_get_drvdata(s->private);
	u32 empty;
	int i;

	mutex_lock(&dev->mt76.mutex);

	mt7996_pbuf_show(s, "PLE", mt76_rr(dev, MT_PLE_PBUF_CTRL), false);
	mt7996_freepg_show(s, mt76_rr(dev, MT_PLE_FREEPG_CNT),
			   mt76_rr(dev, MT_PLE_FREEPG_HEAD_TAIL));
	for (i = 0; i < ARRAY_SIZE(groups); i++)
		mt7996_pg_group_show(s, groups[i].name,
				     mt76_rr(dev, groups[i].group),
				     mt76_rr(dev, groups[i].info));

	empty = mt76_rr(dev, MT_FL_Q_EMPTY);
	seq_printf(s, "PLE queue empty 0x%08x\n", empty);
	mt7996_fl_queues_show(s, dev, false, mt7996_ple_queues, empty);
	if (!(empty & MT_FL_Q_EMPTY_ALL_AC))
		mt7996_ple_sta_queues_show(s, dev);

	mt7996_ple_txcmd_queues_show(s, dev);

	mutex_unlock(&dev->mt76.mutex);

	return 0;
}

static int
mt7996_pse_info_show(struct seq_file *s, void *data)
{
	static const struct {
		const char *name;
		u8 idx;
		bool band2;
	} groups[] = {
		{ "HIF0", 0 }, { "HIF1", 1 }, { "HIF2", 12 }, { "CPU", 2 },
		{ "LMAC0", 5 }, { "LMAC1", 6 }, { "LMAC2", 7 }, { "LMAC3", 8 },
		{ "PLE", 3 }, { "PLE1", 4 }, { "MDP", 9 }, { "MDP2", 11 },
		{ "MDP3", 13, true },
	};
	struct mt7996_dev *dev = dev_get_drvdata(s->private);
	u32 empty, empty_1;
	int i;

	mutex_lock(&dev->mt76.mutex);

	mt7996_pbuf_show(s, "PSE", mt76_rr(dev, MT_PSE_PBUF_CTRL), true);
	mt7996_freepg_show(s, mt76_rr(dev, MT_PSE_FREEPG_CNT),
			   mt76_rr(dev, MT_PSE_FREEPG_HEAD_TAIL));
	for (i = 0; i < ARRAY_SIZE(groups); i++) {
		if (groups[i].band2 && !mt7996_band_valid(dev, MT_BAND2))
			continue;

		mt7996_pg_group_show(s, groups[i].name,
				     mt76_rr(dev, MT_PSE_PG_GROUP(groups[i].idx)),
				     mt76_rr(dev, MT_PSE_PG_INFO(groups[i].idx)));
	}

	empty = mt76_rr(dev, MT_PSE_QUEUE_EMPTY);
	empty_1 = mt76_rr(dev, MT_PSE_QUEUE_EMPTY_1);
	seq_printf(s, "PSE queue empty 0x%08x 0x%08x\n", empty, empty_1);
	mt7996_fl_queues_show(s, dev, true, mt7996_pse_queues, empty);
	mt7996_fl_queues_show(s, dev, true, mt7996_pse_queues_1, empty_1);

	mutex_unlock(&dev->mt76.mutex);

	return 0;
}

static int
mt7996_xmit_queues_show(struct seq_file *file, void *data)
{
	struct mt7996_dev *dev = file->private;
	struct mt7996_phy *phy;
	struct {
		struct mt76_queue *q;
		char *queue;
	} queue_map[] = {
		{ dev->mphy.q_tx[MT_TXQ_BE],	 "  MAIN0"  },
		{ NULL,				 "  MAIN1"  },
		{ NULL,				 "  MAIN2"  },
		{ dev->mt76.q_mcu[MT_MCUQ_WM],	 "  MCUWM"  },
		{ dev->mt76.q_mcu[MT_MCUQ_WA],	 "  MCUWA"  },
		{ dev->mt76.q_mcu[MT_MCUQ_FWDL], "MCUFWDL" },
	};
	int i;

	phy = mt7996_phy2(dev);
	if (phy)
		queue_map[1].q = phy->mt76->q_tx[MT_TXQ_BE];

	phy = mt7996_phy3(dev);
	if (phy)
		queue_map[2].q = phy->mt76->q_tx[MT_TXQ_BE];

	seq_puts(file, "     queue | hw-queued |      head |      tail |\n");
	for (i = 0; i < ARRAY_SIZE(queue_map); i++) {
		struct mt76_queue *q = queue_map[i].q;

		if (!q)
			continue;

		seq_printf(file, "   %s | %9d | %9d | %9d |\n",
			   queue_map[i].queue, q->queued, q->head,
			   q->tail);
	}

	return 0;
}

DEFINE_SHOW_ATTRIBUTE(mt7996_xmit_queues);

static int
mt7996_twt_stats(struct seq_file *s, void *data)
{
	struct mt7996_dev *dev = dev_get_drvdata(s->private);
	struct mt7996_twt_flow *iter;

	mutex_lock(&dev->mt76.mutex);

	seq_puts(s, "     wcid |       id |    flags |      exp | mantissa");
	seq_puts(s, " | duration |            tsf |\n");
	list_for_each_entry(iter, &dev->twt_list, list)
		seq_printf(s,
			   "%9d | %8d | %5c%c%c%c | %8d | %8d | %8d | %14lld |\n",
			   iter->wcid, iter->id,
			   iter->sched ? 's' : 'u',
			   iter->protection ? 'p' : '-',
			   iter->trigger ? 't' : '-',
			   iter->flowtype ? '-' : 'a',
			   iter->exp, iter->mantissa,
			   iter->duration, iter->tsf);

	mutex_unlock(&dev->mt76.mutex);

	return 0;
}

/* The index of RF registers use the generic regidx, combined with two parts:
 * WF selection [31:24] and offset [23:0].
 */
static int
mt7996_rf_regval_get(void *data, u64 *val)
{
	struct mt7996_dev *dev = data;
	u32 regval;
	int ret;

	ret = mt7996_mcu_rf_regval(dev, dev->mt76.debugfs_reg, &regval, false);
	if (ret)
		return ret;

	*val = regval;

	return 0;
}

static int
mt7996_rf_regval_set(void *data, u64 val)
{
	struct mt7996_dev *dev = data;
	u32 val32 = val;

	return mt7996_mcu_rf_regval(dev, dev->mt76.debugfs_reg, &val32, true);
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_rf_regval, mt7996_rf_regval_get,
			 mt7996_rf_regval_set, "0x%08llx\n");

static int mt7996_vow_atf_set(void *data, u64 val)
{
	struct mt7996_dev *dev = data;
	struct mt7996_phy *phy;
	int ret;

	dev->vow_atf_en = !!val;

	mt7996_for_each_phy(dev, phy) {
		ret = mt7996_mcu_set_vow_drr_ctrl(dev, phy->mt76->band_idx, NULL,
						  NULL,
						  VOW_DRR_CTRL_AIRTIME_DEFICIT_BOUND, 0);
		if (ret)
			return ret;

		ret = mt7996_mcu_set_vow_feature_ctrl(phy);
		if (ret)
			return ret;
	}

	return 0;
}

static int mt7996_vow_atf_get(void *data, u64 *val)
{
	struct mt7996_dev *dev = data;

	*val = dev->vow_atf_en;

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_vow_atf, mt7996_vow_atf_get, mt7996_vow_atf_set,
			 "%lld\n");

int mt7996_init_debugfs(struct mt7996_dev *dev)
{
	struct dentry *dir;

	dir = mt76_register_debugfs_fops(&dev->mphy, NULL);

	debugfs_create_file("xmit-queues", 0400, dir, dev,
			    &mt7996_xmit_queues_fops);
	debugfs_create_file("tx_stats", 0400, dir, dev, &mt7996_tx_stats_fops);
	debugfs_create_file("sys_recovery", 0600, dir, dev,
			    &mt7996_sys_recovery_ops);
	debugfs_create_file("fw_debug_wm", 0600, dir, dev, &fops_fw_debug_wm);
	debugfs_create_file("fw_debug_wa", 0600, dir, dev, &fops_fw_debug_wa);
	debugfs_create_file("fw_debug_bin", 0600, dir, dev, &fops_fw_debug_bin);
	debugfs_create_file("fw_debug_module", 0600, dir, dev,
			    &fops_fw_debug_module);
	debugfs_create_file("fw_debug_level", 0600, dir, dev,
			    &fops_fw_debug_level);
	debugfs_create_devm_seqfile(dev->mt76.dev, "fw_wm_info", dir,
				    mt7996_fw_wm_info_show);
	debugfs_create_devm_seqfile(dev->mt76.dev, "ple_info", dir,
				    mt7996_ple_info_show);
	debugfs_create_devm_seqfile(dev->mt76.dev, "pse_info", dir,
				    mt7996_pse_info_show);
	/* TODO: wm fw cpu utilization */
	debugfs_create_file("fw_util_wa", 0400, dir, dev,
			    &mt7996_fw_util_wa_fops);
	debugfs_create_file("implicit_txbf", 0600, dir, dev,
			    &fops_implicit_txbf);
	debugfs_create_devm_seqfile(dev->mt76.dev, "twt_stats", dir,
				    mt7996_twt_stats);
	debugfs_create_file("rf_regval", 0600, dir, dev, &fops_rf_regval);
	debugfs_create_file("vow_atf", 0600, dir, dev, &fops_vow_atf);

	debugfs_create_u32("dfs_hw_pattern", 0400, dir, &dev->hw_pattern);
	debugfs_create_file("radar_trigger", 0200, dir, dev,
			    &fops_radar_trigger);
	debugfs_create_devm_seqfile(dev->mt76.dev, "rdd_monitor", dir,
				    mt7996_rdd_monitor);

	dev->debugfs_dir = dir;

	return 0;
}

static void
mt7996_debugfs_write_fwlog(struct mt7996_dev *dev, const void *hdr, int hdrlen,
			   const void *data, int len)
{
	static DEFINE_SPINLOCK(lock);
	unsigned long flags;
	void *dest;

	spin_lock_irqsave(&lock, flags);
	dest = relay_reserve(dev->relay_fwlog, hdrlen + len + 4);
	if (dest) {
		*(u32 *)dest = hdrlen + len;
		dest += 4;

		if (hdrlen) {
			memcpy(dest, hdr, hdrlen);
			dest += hdrlen;
		}

		memcpy(dest, data, len);
		relay_flush(dev->relay_fwlog);
	}
	spin_unlock_irqrestore(&lock, flags);
}

void mt7996_debugfs_rx_fw_monitor(struct mt7996_dev *dev, const void *data, int len)
{
	struct {
		__le32 magic;
		u8 version;
		u8 _rsv;
		__le16 serial_id;
		__le32 timestamp;
		__le16 msg_type;
		__le16 len;
	} hdr = {
		.version = 0x1,
		.magic = cpu_to_le32(FW_BIN_LOG_MAGIC),
		.msg_type = cpu_to_le16(PKT_TYPE_RX_FW_MONITOR),
	};

	if (!dev->relay_fwlog)
		return;

	hdr.serial_id = cpu_to_le16(dev->fw_debug_seq++);
	hdr.timestamp = cpu_to_le32(mt76_rr(dev, MT_LPON_FRCR(0)));
	hdr.len = *(__le16 *)data;
	mt7996_debugfs_write_fwlog(dev, &hdr, sizeof(hdr), data, len);
}

bool mt7996_debugfs_rx_log(struct mt7996_dev *dev, const void *data, int len)
{
	if (get_unaligned_le32(data) != FW_BIN_LOG_MAGIC)
		return false;

	if (dev->relay_fwlog)
		mt7996_debugfs_write_fwlog(dev, NULL, 0, data, len);

	return true;
}

#ifdef CONFIG_MAC80211_DEBUGFS
/** per-station debugfs **/

static void
mt7996_sta_hw_queue_read(void *data, struct ieee80211_sta *sta)
{
	struct mt7996_sta *msta = (struct mt7996_sta *)sta->drv_priv;
	struct mt7996_vif *mvif = msta->vif;
	struct ieee80211_link_sta *link_sta;
	struct seq_file *s = data;
	struct ieee80211_vif *vif;
	unsigned int link_id;

	vif = container_of((void *)mvif, struct ieee80211_vif, drv_priv);

	rcu_read_lock();

	for_each_sta_active_link(vif, sta, link_sta, link_id) {
		struct mt7996_sta_link *msta_link;
		struct mt76_vif_link *mlink;
		struct mt7996_phy *phy;
		unsigned int cr;
		u16 wlan_idx;
		u8 ac;

		mlink = rcu_dereference(mvif->mt76.link[link_id]);
		if (!mlink)
			continue;

		phy = mt7996_vif_link_phy(container_of(mlink,
						       struct mt7996_vif_link,
						       mt76));
		if (!phy)
			continue;

		msta_link = mt7996_sta_link(msta, link_id);
		if (!msta_link)
			continue;

		wlan_idx = msta_link->wcid.idx;
		cr = mt7996_ple_sta_cr(phy->dev, wlan_idx / 32);
		for (ac = 0; ac < IEEE80211_NUM_ACS; ac++) {
			struct mt7996_hw_queue q = {
				.pid = MT_QUEUE_PORT_LMAC,
				.qid = ac,
			};

			if (mt76_rr(phy->dev, MT_PLE_AC_QEMPTY(ac, cr)) &
			    BIT(wlan_idx % 32))
				continue;

			seq_printf(s, "\tSTA %pM wcid %u AC%u%u: ", sta->addr,
				   wlan_idx, mlink->wmm_idx, ac);
			mt7996_fl_queue_show(s, phy->dev, false, &q, wlan_idx);
			seq_puts(s, "\n");
		}
	}

	rcu_read_unlock();
}

static int
mt7996_queues_show(struct seq_file *s, void *data)
{
	struct ieee80211_sta *sta = s->private;

	mt7996_sta_hw_queue_read(s, sta);

	return 0;
}

DEFINE_SHOW_ATTRIBUTE(mt7996_queues);

void mt7996_sta_add_debugfs(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
			    struct ieee80211_sta *sta, struct dentry *dir)
{
	debugfs_create_file("hw-queues", 0400, dir, sta, &mt7996_queues_fops);
}

static ssize_t mt7996_link_sta_fixed_rate_set(struct file *file,
					      const char __user *user_buf,
					      size_t count, loff_t *ppos)
{
#define SHORT_PREAMBLE 0
#define LONG_PREAMBLE 1
	struct ieee80211_link_sta *link_sta = file->private_data;
	struct mt7996_sta *msta = (struct mt7996_sta *)link_sta->sta->drv_priv;
	struct mt7996_sta_link *msta_link;
	struct mt7996_phy *link_phy = NULL;
	struct mt76_vif_link *mlink;
	struct ra_rate phy = {};
	struct mt7996_dev *dev;
	char buf[100];
	int ret;
	u16 gi, ltf;

	rcu_read_lock();
	mlink = rcu_dereference(msta->vif->mt76.link[link_sta->link_id]);
	if (mlink)
		link_phy = mt7996_vif_link_phy(container_of(mlink,
							    struct mt7996_vif_link,
							    mt76));
	rcu_read_unlock();

	if (!link_phy)
		return -EINVAL;

	if (count >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, user_buf, count))
		return -EFAULT;

	if (count && buf[count - 1] == '\n')
		buf[count - 1] = '\0';
	else
		buf[count] = '\0';

	/* mode - cck: 0, ofdm: 1, ht: 2, gf: 3, vht: 4, he_su: 8, he_er: 9 EHT: 15
	 * bw - bw20: 0, bw40: 1, bw80: 2, bw160: 3, BW320: 4
	 * mcs - cck: 0~4, ofdm: 0~7, ht: 0~32, vht: 0~9, he_su: 0~11, he_er: 0~2, eht: 0~13
	 * nss - vht: 1~4, he: 1~4, eht: 1~4, others: ignore
	 * gi - (ht/vht) lgi: 0, sgi: 1; (he) 0.8us: 0, 1.6us: 1, 3.2us: 2
	 * preamble - short: 1, long: 0
	 * stbc - off: 0, on: 1
	 * ldpc - off: 0, on: 1
	 * spe - off: 0, on: 1
	 * ltf - 1xltf: 0, 2xltf: 1, 4xltf: 2
	 */
	dev = link_phy->dev;
	if (sscanf(buf, "%hhu %hhu %hhu %hhu %hu %hhu %hhu %hhu %hhu %hu",
		   &phy.mode, &phy.bw, &phy.mcs, &phy.nss, &gi,
		   &phy.preamble, &phy.stbc, &phy.ldpc, &phy.spe, &ltf) != 10) {
		dev_warn(dev->mt76.dev,
			 "format: Mode BW MCS NSS GI Preamble STBC LDPC SPE ltf\n");
		return -EINVAL;
	}

	mutex_lock(&dev->mt76.mutex);

	msta_link = mt7996_sta_link_protected(dev, msta, link_sta->link_id);
	if (!msta_link) {
		ret = -EINVAL;
		goto out;
	}
	phy.wlan_idx = cpu_to_le16(msta_link->wcid.idx);
	phy.gi = cpu_to_le16(gi);
	phy.ltf = cpu_to_le16(ltf);
	phy.ldpc = phy.ldpc ? 7 : 0;
	phy.preamble = phy.preamble ? SHORT_PREAMBLE : LONG_PREAMBLE;

	ret = mt7996_mcu_set_fixed_rate_ctrl(dev, &phy, 0);
	if (ret)
		goto out;

	ret = count;
out:
	mutex_unlock(&dev->mt76.mutex);
	return ret;
}

static const struct file_operations fops_fixed_rate = {
	.write = mt7996_link_sta_fixed_rate_set,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

void mt7996_link_sta_add_debugfs(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
				 struct ieee80211_link_sta *link_sta,
				 struct dentry *dir)
{
	debugfs_create_file("fixed_rate", 0600, dir, link_sta, &fops_fixed_rate);
}

#endif
