/* SPDX-License-Identifier: BSD-3-Clause-Clear */
/* Copyright (C) 2026 MediaTek Inc. */

#ifndef __MT7925_P2P_H
#define __MT7925_P2P_H

#include <net/mac80211.h>

#include "../mt76.h"
#include "../mt792x.h"

bool mt7925_p2p_rx_drop_chan_mismatch(struct mt792x_dev *dev,
				      struct sk_buff *skb);

#endif /* __MT7925_P2P_H */
