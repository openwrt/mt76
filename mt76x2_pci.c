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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>

#include "mt76x2.h"
#include "mt76x2_trace.h"

static const struct pci_device_id mt76pci_device_table[] = {
	{ PCI_DEVICE(0x14c3, 0x7662) },
	{ },
};

static int
mt76pci_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct mt76x2_dev *dev;
	int ret;

	ret = pcim_enable_device(pdev);
	if (ret)
		return ret;

	ret = pcim_iomap_regions(pdev, BIT(0), pci_name(pdev));
	if (ret)
		return ret;

	pci_set_master(pdev);

	ret = pci_set_dma_mask(pdev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	dev = mt76x2_alloc_device(&pdev->dev);
	if (!dev)
		return -ENOMEM;

	mt76_mmio_init(&dev->mt76, pcim_iomap_table(pdev)[0]);

	pci_set_drvdata(pdev, dev);

	dev->rev = mt76_rr(dev, MT_ASIC_VERSION);
	dev_printk(KERN_INFO, dev->dev, "ASIC revision: %08x\n", dev->rev);

	ret = devm_request_irq(dev->dev, pdev->irq, mt76x2_irq_handler,
			       IRQF_SHARED, KBUILD_MODNAME, dev);
	if (ret)
		goto error;

	ret = mt76x2_register_device(dev);
	if (ret)
		goto error;

	/* Fix up ASPM configuration */

	/* RG_SSUSB_G1_CDR_BIR_LTR = 0x9 */
	mt76_rmw_field(dev, 0x15a10, 0x1f << 16, 0x9);

	/* RG_SSUSB_G1_CDR_BIC_LTR = 0xf */
	mt76_rmw_field(dev, 0x15a0c, 0xf << 28, 0xf);

	/* RG_SSUSB_CDR_BR_PE1D = 0x3 */
	mt76_rmw_field(dev, 0x15c58, 0x3 << 6, 0x3);

	return 0;

error:
	ieee80211_free_hw(mt76_hw(dev));
	return ret;
}

static void
mt76pci_remove(struct pci_dev *pdev)
{
	struct mt76x2_dev *dev = pci_get_drvdata(pdev);

	ieee80211_unregister_hw(mt76_hw(dev));
	mt76x2_cleanup(dev);
	ieee80211_free_hw(mt76_hw(dev));
}

MODULE_DEVICE_TABLE(pci, mt76pci_device_table);
MODULE_FIRMWARE(MT7662_FIRMWARE);
MODULE_FIRMWARE(MT7662_ROM_PATCH);
MODULE_LICENSE("GPL");

static struct pci_driver mt76pci_driver = {
	.name		= KBUILD_MODNAME,
	.id_table	= mt76pci_device_table,
	.probe		= mt76pci_probe,
	.remove		= mt76pci_remove,
};

module_pci_driver(mt76pci_driver);
