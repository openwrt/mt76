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

#include "mt7603.h"

static const struct pci_device_id mt76pci_device_table[] = {
	{ PCI_DEVICE(0x14c3, 0x7603) },
	{ },
};

static int
mt76pci_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct mt7603_dev *dev;
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

	dev = mt7603_alloc_device(&pdev->dev);
	if (!dev)
		return -ENOMEM;

	mt76_mmio_init(&dev->mt76, pcim_iomap_table(pdev)[0]);

	dev->mt76.rev = (mt76_rr(dev, MT_HW_CHIPID) << 16) |
			(mt76_rr(dev, MT_HW_REV) & 0xff);
	dev_printk(KERN_INFO, dev->mt76.dev, "ASIC revision: %04x\n", dev->mt76.rev);

	ret = devm_request_irq(dev->mt76.dev, pdev->irq, mt7603_irq_handler,
			       IRQF_SHARED, KBUILD_MODNAME, dev);
	if (ret)
		goto error;

	ret = mt7603_register_device(dev);
	if (ret)
		goto error;

	return 0;
error:
	ieee80211_free_hw(mt76_hw(dev));
	return ret;
}

static void
mt76pci_remove(struct pci_dev *pdev)
{
	struct mt76_dev *mdev = pci_get_drvdata(pdev);
	struct mt7603_dev *dev = container_of(mdev, struct mt7603_dev, mt76);

	mt76_unregister_device(&dev->mt76);
	mt7603_mcu_exit(dev);
	mt7603_dma_cleanup(dev);
	ieee80211_free_hw(mt76_hw(dev));
}

MODULE_DEVICE_TABLE(pci, mt76pci_device_table);
MODULE_FIRMWARE(MT7603_FIRMWARE_E1);
MODULE_FIRMWARE(MT7603_FIRMWARE_E2);
MODULE_LICENSE("GPL");

static struct pci_driver mt76pci_driver = {
	.name		= KBUILD_MODNAME,
	.id_table	= mt76pci_device_table,
	.probe		= mt76pci_probe,
	.remove		= mt76pci_remove,
};

module_pci_driver(mt76pci_driver);
