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

#ifndef __MT7603_DMA_H
#define __MT7603_DMA_H

#include "dma.h"

#define MT_RXD0_LENGTH			GENMASK(15, 0)
#define MT_RXD0_PKT_TYPE		GENMASK(31, 29)

#define MT_RXD0_NORMAL_ETH_TYPE_OFS	GENMASK(22, 16)
#define MT_RXD0_NORMAL_IP_SUM		BIT(23)
#define MT_RXD0_NORMAL_UDP_TCP_SUM	BIT(24)
#define MT_RXD0_NORMAL_GROUP_1		BIT(25)
#define MT_RXD0_NORMAL_GROUP_2		BIT(26)
#define MT_RXD0_NORMAL_GROUP_3		BIT(27)
#define MT_RXD0_NORMAL_GROUP_4		BIT(28)

enum rx_pkt_type {
	PKT_TYPE_TXS		= 0,
	PKT_TYPE_TXRXV		= 1,
	PKT_TYPE_NORMAL		= 2,
	PKT_TYPE_RX_DUP_RFB	= 3,
	PKT_TYPE_RX_TMR		= 4,
	PKT_TYPE_RETRIEVE	= 5,
	PKT_TYPE_RX_EVENT	= 7,
};

#define MT_TXD0_P_IDX			BIT(31)
#define MT_TXD0_Q_IDX			GENMASK(30, 27)
#define MT_TXD0_UTXB			BIT(26)
#define MT_TXD0_UNXV			BIT(25)
#define MT_TXD0_UDP_TCP_SUM		BIT(24)
#define MT_TXD0_IP_SUM			BIT(23)
#define MT_TXD0_ETH_TYPE_OFFSET		GENMASK(22, 16)
#define MT_TXD0_TX_BYTES		GENMASK(15, 0)

#define MT_TXD1_OWN_MAC			GENMASK(31, 26)
#define MT_TXD1_PROTECTED		BIT(23)
#define MT_TXD1_TID			GENMASK(22, 20)
#define MT_TXD1_NO_ACK			BIT(19)
#define MT_TXD1_HDR_PAD			GENMASK(18, 16)
#define MT_TXD1_FT			BIT(15)
#define MT_TXD1_HDR_FORMAT		GENMASK(14, 13)
#define MT_TXD1_HDR_INFO		GENMASK(12, 8)
#define MT_TXD1_WLAN_IDX		GENMASK(7, 0)

#define MT_TXD2_FIX_RATE		BIT(31)
#define MT_TXD2_TIMING_MEASURE		BIT(30)
#define MT_TXD2_BA_DISABLE		BIT(29)
#define MT_TXD2_POWER_OFFSET		GENMASK(28, 24)
#define MT_TXD2_MAX_TX_TIME		GENMASK(23, 16)
#define MT_TXD2_FRAG			GENMASK(15, 14)
#define MT_TXD2_HTC_VLD			BIT(13)
#define MT_TXD2_DURATION		BIT(12)
#define MT_TXD2_BIP			BIT(11)
#define MT_TXD2_MULTICAST		BIT(10)
#define MT_TXD2_RTS			BIT(9)
#define MT_TXD2_SOUNDING		BIT(8)
#define MT_TXD2_NDPA			BIT(7)
#define MT_TXD2_NDP			BIT(6)
#define MT_TXD2_FRAME_TYPE		GENMASK(5, 4)
#define MT_TXD2_SUB_TYPE		GENMASK(3, 0)

#define MT_TXD3_SN_VALID		BIT(31)
#define MT_TXD3_PN_VALID		BIT(30)
#define MT_TXD3_SEQ			GENMASK(27, 16)
#define MT_TXD3_REM_TX_COUNT		GENMASK(15, 11)
#define MT_TXD3_TX_COUNT		GENMASK(10, 6)

#define MT_TXD4_PN_LOW			GENMASK(31, 0)

#define MT_TXD5_PN_HIGH			GENMASK(31, 16)
#define MT_TXD5_SW_POWER_MGMT		BIT(13)
#define MT_TXD5_BA_SEQ_CTRL		BIT(12)
#define MT_TXD5_DA_SELECT		BIT(11)
#define MT_TXD5_TX_STATUS_HOST		BIT(10)
#define MT_TXD5_TX_STATUS_MCU		BIT(9)
#define MT_TXD5_TX_STATUS_FMT		BIT(8)
#define MT_TXD5_PID			GENMASK(7, 0)

#define MT_TXD6_SGI			BIT(31)
#define MT_TXD6_LDPC			BIT(30)
#define MT_TXD6_TX_RATE			GENMASK(29, 18)
#define MT_TXD6_I_TXBF			BIT(17)
#define MT_TXD6_E_TXBF			BIT(16)
#define MT_TXD6_DYN_BW			BIT(15)
#define MT_TXD6_ANT_PRI			GENMASK(14, 12)
#define MT_TXD6_SPE_EN			BIT(11)
#define MT_TXD6_BW			GENMASK(10, 8)
#define MT_TXD6_ANT_ID			GENMASK(7, 2)
#define MT_TXD6_FIXED_RATE		BIT(0)

#endif
