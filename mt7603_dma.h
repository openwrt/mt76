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

#define MT_RXD0_PKT_TYPE		GENMASK(31, 29)

enum rx_pkt_type {
	PKT_TYPE_TXS		= 0,
	PKT_TYPE_TXRXV		= 1,
	PKT_TYPE_NORMAL		= 2,
	PKT_TYPE_RX_DUP_RFB	= 3,
	PKT_TYPE_RX_TMR		= 4,
	PKT_TYPE_RETRIEVE	= 5,
	PKT_TYPE_RX_EVENT	= 7,
};

#endif
