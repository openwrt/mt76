// SPDX-License-Identifier: BSD-3-Clause-Clear
/* Copyright (C) 2025 MediaTek Inc. */

#include <linux/of.h>
#include "mt7921.h"
#include "regd.h"
#include "mcu.h"

static void
mt7921_regd_channel_update(struct wiphy *wiphy, struct mt792x_dev *dev)
{
#define IS_UNII_INVALID(idx, sfreq, efreq) \
	(!(dev->phy.clc_chan_conf & BIT(idx)) && (cfreq) >= (sfreq) && (cfreq) <= (efreq))
	struct ieee80211_supported_band *sband;
	struct mt76_dev *mdev = &dev->mt76;
	struct device_node *np, *band_np;
	struct ieee80211_channel *ch;
	int i, cfreq;

	np = mt76_find_power_limits_node(mdev);

	sband = wiphy->bands[NL80211_BAND_5GHZ];
	band_np = np ? of_get_child_by_name(np, "txpower-5g") : NULL;
	for (i = 0; i < sband->n_channels; i++) {
		ch = &sband->channels[i];
		cfreq = ch->center_freq;

		if (np && (!band_np || !mt76_find_channel_node(band_np, ch))) {
			ch->flags |= IEEE80211_CHAN_DISABLED;
			continue;
		}

		/* UNII-4 */
		if (IS_UNII_INVALID(0, 5845, 5925))
			ch->flags |= IEEE80211_CHAN_DISABLED;
	}

	sband = wiphy->bands[NL80211_BAND_6GHZ];
	if (!sband)
		return;

	band_np = np ? of_get_child_by_name(np, "txpower-6g") : NULL;
	for (i = 0; i < sband->n_channels; i++) {
		ch = &sband->channels[i];
		cfreq = ch->center_freq;

		if (np && (!band_np || !mt76_find_channel_node(band_np, ch))) {
			ch->flags |= IEEE80211_CHAN_DISABLED;
			continue;
		}

		/* UNII-5/6/7/8 */
		if (IS_UNII_INVALID(1, 5925, 6425) ||
		    IS_UNII_INVALID(2, 6425, 6525) ||
		    IS_UNII_INVALID(3, 6525, 6875) ||
		    IS_UNII_INVALID(4, 6875, 7125))
			ch->flags |= IEEE80211_CHAN_DISABLED;
	}
}

void mt7921_regd_update(struct mt792x_dev *dev)
{
	struct mt76_dev *mdev = &dev->mt76;
	struct ieee80211_hw *hw = mdev->hw;
	struct wiphy *wiphy = hw->wiphy;

	mt7921_mcu_set_clc(dev, mdev->alpha2, dev->country_ie_env);
	mt7921_regd_channel_update(wiphy, dev);
	mt76_connac_mcu_set_channel_domain(hw->priv);
	mt7921_set_tx_sar_pwr(hw, NULL);
}
EXPORT_SYMBOL_GPL(mt7921_regd_update);

void mt7921_regd_notifier(struct wiphy *wiphy,
			  struct regulatory_request *request)
{
	struct ieee80211_hw *hw = wiphy_to_ieee80211_hw(wiphy);
	struct mt792x_dev *dev = mt792x_hw_dev(hw);
	struct mt76_connac_pm *pm = &dev->pm;

	memcpy(dev->mt76.alpha2, request->alpha2, sizeof(dev->mt76.alpha2));
	dev->mt76.region = request->dfs_region;
	dev->country_ie_env = request->country_ie_env;

	if (request->initiator == NL80211_REGDOM_SET_BY_USER) {
		if (dev->mt76.alpha2[0] == '0' && dev->mt76.alpha2[1] == '0')
			wiphy->regulatory_flags &= ~REGULATORY_COUNTRY_IE_IGNORE;
		else
			wiphy->regulatory_flags |= REGULATORY_COUNTRY_IE_IGNORE;
	}

	if (pm->suspended)
		return;

	dev->regd_in_progress = true;

	mt792x_mutex_acquire(dev);
	mt7921_regd_update(dev);
	mt792x_mutex_release(dev);

	dev->regd_in_progress = false;
	wake_up(&dev->wait);
}
