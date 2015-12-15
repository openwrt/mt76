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

#include "mt7603.h"

#define MT_PSE_PAGE_SIZE	128

void mt7603_wtbl_clear(struct mt7603_dev *dev, int idx)
{
	int wtbl2_frame = (idx * MT_WTBL2_SIZE) / MT_PSE_PAGE_SIZE;
	int wtbl2_entry = (idx * MT_WTBL2_SIZE) % MT_PSE_PAGE_SIZE;
	int wtbl3_frame = (idx * MT_WTBL3_SIZE) / MT_PSE_PAGE_SIZE;
	int wtbl3_entry = ((idx * MT_WTBL3_SIZE) % MT_PSE_PAGE_SIZE) * 2;
	int wtbl4_frame = (idx * MT_WTBL4_SIZE) / MT_PSE_PAGE_SIZE;
	int wtbl4_entry = (idx * MT_WTBL4_SIZE) % MT_PSE_PAGE_SIZE;
	u32 addr;

	addr = MT_WTBL1_BASE + idx * MT_WTBL1_SIZE;

	mt76_wr(dev, addr + 0 * 4,
		MT_WTBL1_W0_RX_CHECK_A2 |
		MT_WTBL1_W0_RX_VALID);
	mt76_wr(dev, addr + 1 * 4, 0);
	mt76_wr(dev, addr + 2 * 4, 0);
	mt76_wr(dev, addr + 3 * 4,
		MT76_SET(MT_WTBL1_W3_WTBL2_FRAME_ID, wtbl2_frame) |
		MT76_SET(MT_WTBL1_W3_WTBL2_ENTRY_ID, wtbl2_entry) |
		MT76_SET(MT_WTBL1_W3_WTBL4_FRAME_ID, wtbl4_frame));
	mt76_wr(dev, addr + 4 * 4,
		MT76_SET(MT_WTBL1_W4_WTBL3_FRAME_ID, wtbl3_frame) |
		MT76_SET(MT_WTBL1_W4_WTBL3_ENTRY_ID, wtbl3_entry) |
		MT76_SET(MT_WTBL1_W4_WTBL4_ENTRY_ID, wtbl4_entry));
}
