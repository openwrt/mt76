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

#if !defined(__MT76_TRACE_H) || defined(TRACE_HEADER_MULTI_READ)
#define __MT76_TRACE_H

#include <linux/tracepoint.h>
#include "mt76.h"

#undef TRACE_SYSTEM
#define TRACE_SYSTEM mt76

#define MAXNAME		32
#define DEV_ENTRY	__array(char, wiphy_name, 32)
#define DEV_ASSIGN	strlcpy(__entry->wiphy_name, wiphy_name(dev->hw->wiphy), MAXNAME)
#define DEV_PR_FMT	"%s"
#define DEV_PR_ARG	__entry->wiphy_name

#define TXID_ENTRY	__field(u8, wcid) __field(u8, pktid)
#define TXID_ASSIGN	__entry->wcid = wcid; __entry->pktid = pktid
#define TXID_PR_FMT	" [%d:%d]"
#define TXID_PR_ARG	__entry->wcid, __entry->pktid

#define REG_ENTRY	__field(u32, reg) __field(u32, val)
#define REG_ASSIGN	__entry->reg = reg; __entry->val = val
#define REG_PR_FMT	" %04x=%08x"
#define REG_PR_ARG	__entry->reg, __entry->val

DECLARE_EVENT_CLASS(dev_evt,
	TP_PROTO(struct mt76_dev *dev),
	TP_ARGS(dev),
	TP_STRUCT__entry(
		DEV_ENTRY
	),
	TP_fast_assign(
		DEV_ASSIGN;
	),
	TP_printk(DEV_PR_FMT, DEV_PR_ARG)
);

DECLARE_EVENT_CLASS(dev_txid_evt,
	TP_PROTO(struct mt76_dev *dev, u8 wcid, u8 pktid),
	TP_ARGS(dev, wcid, pktid),
	TP_STRUCT__entry(
		DEV_ENTRY
		TXID_ENTRY
	),
	TP_fast_assign(
		DEV_ASSIGN;
		TXID_ASSIGN;
	),
	TP_printk(
		DEV_PR_FMT TXID_PR_FMT,
		DEV_PR_ARG, TXID_PR_ARG
	)
);

DECLARE_EVENT_CLASS(dev_reg_evt,
	TP_PROTO(struct mt76_dev *dev, u32 reg, u32 val),
	TP_ARGS(dev, reg, val),
	TP_STRUCT__entry(
		DEV_ENTRY
		REG_ENTRY
	),
	TP_fast_assign(
		DEV_ASSIGN;
		REG_ASSIGN;
	),
	TP_printk(
		DEV_PR_FMT REG_PR_FMT,
		DEV_PR_ARG, REG_PR_ARG
	)
);

DEFINE_EVENT(dev_reg_evt, reg_read,
	TP_PROTO(struct mt76_dev *dev, u32 reg, u32 val),
	TP_ARGS(dev, reg, val)
);

DEFINE_EVENT(dev_reg_evt, reg_write,
	TP_PROTO(struct mt76_dev *dev, u32 reg, u32 val),
	TP_ARGS(dev, reg, val)
);

DEFINE_EVENT(dev_evt, mac_txstat_poll,
	TP_PROTO(struct mt76_dev *dev),
	TP_ARGS(dev)
);

DEFINE_EVENT(dev_txid_evt, mac_txdone_add,
	TP_PROTO(struct mt76_dev *dev, u8 wcid, u8 pktid),
	TP_ARGS(dev, wcid, pktid)
);

TRACE_EVENT(mac_txstat_fetch,
	TP_PROTO(struct mt76_dev *dev,
		 struct mt76_tx_status *stat),

	TP_ARGS(dev, stat),

	TP_STRUCT__entry(
		DEV_ENTRY
		TXID_ENTRY
		__field(bool, success)
		__field(bool, aggr)
		__field(bool, ack_req)
		__field(u16, rate)
		__field(u8, retry)
	),

	TP_fast_assign(
		DEV_ASSIGN;
		__entry->success = stat->success;
		__entry->aggr = stat->aggr;
		__entry->ack_req = stat->ack_req;
		__entry->wcid = stat->wcid;
		__entry->pktid = stat->pktid;
		__entry->rate = stat->rate;
		__entry->retry = stat->retry;
	),

	TP_printk(
		DEV_PR_FMT TXID_PR_FMT
		" success:%d aggr:%d ack_req:%d"
		" rate:%04x retry:%d",
		DEV_PR_ARG, TXID_PR_ARG,
		__entry->success, __entry->aggr, __entry->ack_req,
		__entry->rate, __entry->retry
	)
);


TRACE_EVENT(dev_irq,
	TP_PROTO(struct mt76_dev *dev, u32 val, u32 mask),

	TP_ARGS(dev, val, mask),

	TP_STRUCT__entry(
		DEV_ENTRY
		__field(u32, val)
		__field(u32, mask)
	),

	TP_fast_assign(
		DEV_ASSIGN;
		__entry->val = val;
		__entry->mask = mask;
	),

	TP_printk(
		DEV_PR_FMT " %08x & %08x",
		DEV_PR_ARG, __entry->val, __entry->mask
	)
);

#endif

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE trace

#include <trace/define_trace.h>
