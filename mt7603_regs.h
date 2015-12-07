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

#ifndef __MT7603_REGS_H
#define __MT7603_REGS_H

#define MT_TOP_MISC2			0x1134

#define MT_MCU_BASE			0x2000
#define MT_MCU(ofs)			(MT_MCU_BASE + (ofs))

#define MT_MCU_PCIE_REMAP_1		MT_MCU(0x500)
#define MT_MCU_PCIE_REMAP_1_OFFSET	GENMASK(17, 0)
#define MT_MCU_PCIE_REMAP_1_BASE	GENMASK(31, 18)

#define MT_MCU_PCIE_REMAP_2		MT_MCU(0x504)
#define MT_MCU_PCIE_REMAP_2_OFFSET	GENMASK(18, 0)
#define MT_MCU_PCIE_REMAP_2_BASE	GENMASK(31, 19)

#define MT_HIF_BASE			0x4000
#define MT_HIF(ofs)			(MT_HIF_BASE + (ofs))

#define MT_ASIC_VERSION			MT_HIF(0x000)

#define MT_INT_SOURCE_CSR		MT_HIF(0x200)
#define MT_INT_MASK_CSR			MT_HIF(0x204)

#define MT_INT_RX_DONE(_n)		BIT(_n)
#define MT_INT_RX_DONE_ALL		GENMASK(1, 0)
#define MT_INT_TX_DONE_ALL		GENMASK(19, 4)
#define MT_INT_TX_DONE(_n)		BIT(_n + 4)

#define MT_INT_RX_COHERENT		BIT(20)
#define MT_INT_TX_COHERENT		BIT(21)
#define MT_INT_PRE_TBTT			BIT(27)

#define MT_INT_MCU_CMD			BIT(30)

#define MT_WPDMA_GLO_CFG		MT_HIF(0x208)
#define MT_WPDMA_GLO_CFG_TX_DMA_EN	BIT(0)
#define MT_WPDMA_GLO_CFG_TX_DMA_BUSY	BIT(1)
#define MT_WPDMA_GLO_CFG_RX_DMA_EN	BIT(2)
#define MT_WPDMA_GLO_CFG_RX_DMA_BUSY	BIT(3)
#define MT_WPDMA_GLO_CFG_DMA_BURST_SIZE	GENMASK(5, 4)
#define MT_WPDMA_GLO_CFG_TX_WRITEBACK_DONE	BIT(6)
#define MT_WPDMA_GLO_CFG_BIG_ENDIAN	BIT(7)
#define MT_WPDMA_GLO_CFG_HDR_SEG_LEN	GENMASK(15, 8)
#define MT_WPDMA_GLO_CFG_CLK_GATE_DIS	BIT(30)
#define MT_WPDMA_GLO_CFG_RX_2B_OFFSET	BIT(31)

#define MT_WPDMA_RST_IDX		MT_HIF(0x20c)

#define MT_TX_RING_BASE			MT_HIF(0x300)
#define MT_RX_RING_BASE			MT_HIF(0x400)

#define MT_SCH_1			MT_HIF(0x588)
#define MT_SCH_2			MT_HIF(0x58c)
#define MT_SCH_3			MT_HIF(0x590)

#define MT_SCH_4			MT_HIF(0x594)
#define MT_SCH_4_FORCE_QID		GENMASK(4, 0)
#define MT_SCH_4_BYPASS			BIT(5)

#define MT_PCIE_REMAP_BASE_1		0x40000
#define MT_PCIE_REMAP_BASE_2		0x80000

#define MT_TX_HW_QUEUE_MGMT		4
#define MT_TX_HW_QUEUE_MCU		5
#define MT_TX_HW_QUEUE_BCN		7
#define MT_TX_HW_QUEUE_BMC		8

#endif
