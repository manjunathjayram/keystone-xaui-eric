/*
 * phy-keystone - USB PHY, talking to dwc3 controller in Keystone.
 *
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Author: WingMan Kwok <w-kwok2@ti.com>
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/usb/otg.h>
#include <linux/io.h>
#include <linux/of.h>

/* USB PHY control register offsets */
#define USB_PHY_CTL_UTMI		0x0000
#define USB_PHY_CTL_PIPE		0x0004
#define USB_PHY_CTL_PARAM_1		0x0008
#define USB_PHY_CTL_PARAM_2		0x000c
#define USB_PHY_CTL_CLOCK		0x0010
#define USB_PHY_CTL_PLL			0x0014

#define PHY_REF_SSP_EN			BIT(29)

struct keystone_usbphy {
	struct usb_phy	phy;
	struct device	*dev;
	void __iomem	*phy_ctrl;
};

static inline u32 k_usbphy_readl(void __iomem *base, u32 offset)
{
	return readl(base + offset);
}

static inline void k_usbphy_writel(void __iomem *base,
					  u32 offset, u32 value)
{
	writel(value, base + offset);
}

static int k_usbphy_set_suspend(struct usb_phy *x, int suspend)
{
	return 0;
}

static int k_usbphy_init(struct usb_phy *phy)
{
	struct keystone_usbphy *k_phy = dev_get_drvdata(phy->dev);
	u32 val;

	val  = k_usbphy_readl(k_phy->phy_ctrl, USB_PHY_CTL_CLOCK);
	k_usbphy_writel(k_phy->phy_ctrl, USB_PHY_CTL_CLOCK,
				val | PHY_REF_SSP_EN);
	return 0;
}

static void k_usbphy_shutdown(struct usb_phy *phy)
{
	struct keystone_usbphy *k_phy = dev_get_drvdata(phy->dev);
	u32 val;

	val  = k_usbphy_readl(k_phy->phy_ctrl, USB_PHY_CTL_CLOCK);
	k_usbphy_writel(k_phy->phy_ctrl, USB_PHY_CTL_CLOCK,
				val &= ~PHY_REF_SSP_EN);
}

static int k_usbphy_create_phy(struct device *dev, struct keystone_usbphy *kphy)
{
	enum usb_phy_type type = USB_PHY_TYPE_USB2;

	kphy->dev		= dev;
	kphy->phy.dev		= kphy->dev;
	kphy->phy.label		= "k_usbphy-xceiv";
	kphy->phy.set_suspend	= k_usbphy_set_suspend;
	kphy->phy.init		= k_usbphy_init;
	kphy->phy.shutdown	= k_usbphy_shutdown;

	kphy->phy.state		= OTG_STATE_UNDEFINED;
	kphy->phy.type		= type;

	return 0;
}

static int k_usbphy_probe(struct platform_device *pdev)
{
	struct device		*dev = &pdev->dev;
	struct keystone_usbphy	*k_phy;
	struct resource		*res;
	int ret;

	k_phy = devm_kzalloc(dev, sizeof(*k_phy), GFP_KERNEL);
	if (!k_phy)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	k_phy->phy_ctrl = devm_ioremap_resource(dev, res);
	if (IS_ERR(k_phy->phy_ctrl))
		return PTR_ERR(k_phy->phy_ctrl);

	ret = k_usbphy_create_phy(dev, k_phy);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, k_phy);

	ret = usb_add_phy_dev(&k_phy->phy);
	return ret;
}

static int k_usbphy_remove(struct platform_device *pdev)
{
	struct keystone_usbphy *k_phy = platform_get_drvdata(pdev);

	usb_remove_phy(&k_phy->phy);

	return 0;
}

static const struct of_device_id keystone_usbphy_ids[] = {
	{ .compatible = "ti,keystone-usbphy" },
	{ }
};
MODULE_DEVICE_TABLE(of, keystone_usbphy_ids);

static struct platform_driver keystone_usbphy_driver = {
	.probe          = k_usbphy_probe,
	.remove         = k_usbphy_remove,
	.driver         = {
		.name   = "keystone-usbphy",
		.owner  = THIS_MODULE,
		.of_match_table = keystone_usbphy_ids,
	},
};

module_platform_driver(keystone_usbphy_driver);

MODULE_ALIAS("platform:keystone-usbphy");
MODULE_AUTHOR("WingMan Kwok");
MODULE_DESCRIPTION("Keystone USB phy driver");
MODULE_LICENSE("GPL v2");
