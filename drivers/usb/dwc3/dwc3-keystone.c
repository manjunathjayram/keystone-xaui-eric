/**
 * dwc3-keystone.c - Keystone Specific Glue layer
 *
 * Copyright (C) 2010-2012 Texas Instruments Incorporated - http://www.ti.com
 *
 * Authors: WingMan Kwok <w-kwok2@ti.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions, and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The names of the above-listed copyright holders may not be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2, as published by the Free
 * Software Foundation.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/of_platform.h>

#include "core.h"
#include "io.h"

#define BITS(n)			(BIT(n) - 1)
#define BITFIELD(x, s, n)	(((x) & BITS(n)) << (s))
#define MASK			0xffffffff

struct kdwc3_phy_regs {
	u32	phy_utmi;		/* USB_PHY_CTL0 */
	u32	phy_pipe;		/* USB_PHY_CTL1 */
	u32	phy_param_ctrl_1;	/* USB_PHY_CTL2 */
	u32	phy_param_ctrl_2;	/* USB_PHY_CTL3 */
	u32	phy_clock;		/* USB_PHY_CTL4 */
#define PHY_REF_SSP_EN(x)		BITFIELD(x, 29, 1)
	u32	phy_pll;		/* USB_PHY_CTL5 */
};

/* IRQ register bits */
#define USBSS_IRQ_EOI_LINE(n)	BIT(n)
#define USBSS_IRQ_EVENT_ST	BIT(0)
#define USBSS_IRQ_COREIRQ_EN	BIT(0)
#define USBSS_IRQ_COREIRQ_CLR	BIT(0)

struct kdwc3_irq_regs {
	u32		revision;	/* 0x000 */
	u32		_rsvd0[3];
	u32		sysconfig;	/* 0x010 */
	u32		_rsvd1[1];
	u32		irq_eoi;
	u32		_rsvd2[1];
	struct {
		u32	raw_status;
		u32	status;
		u32	enable_set;
		u32	enable_clr;
	} irqs[16];
};

struct dwc3_keystone {
	struct platform_device	*dwc;
	struct device		*dev;
	struct clk		*clk;
	struct kdwc3_irq_regs __iomem	*usbss;
};

static u64 kdwc3_dma_mask;

static void kdwc3_enable_irqs(struct dwc3_keystone *kdwc)
{
	u32 val;

	val = readl(&kdwc->usbss->irqs[0].enable_set);
	val |= USBSS_IRQ_COREIRQ_EN;
	writel(val, &kdwc->usbss->irqs[0].enable_set);
}

static void kdwc3_disable_irqs(struct dwc3_keystone *kdwc)
{
	u32 val;

	val = readl(&kdwc->usbss->irqs[0].enable_set);
	val &= ~USBSS_IRQ_COREIRQ_EN;
	writel(val, &kdwc->usbss->irqs[0].enable_set);
}

static irqreturn_t dwc3_keystone_interrupt(int irq, void *_kdwc)
{
	struct dwc3_keystone	*kdwc = _kdwc;

	writel(USBSS_IRQ_COREIRQ_CLR,	&kdwc->usbss->irqs[0].enable_clr);
	writel(USBSS_IRQ_EVENT_ST,	&kdwc->usbss->irqs[0].status);
	writel(USBSS_IRQ_COREIRQ_EN,	&kdwc->usbss->irqs[0].enable_set);
	writel(USBSS_IRQ_EOI_LINE(0),	&kdwc->usbss->irq_eoi);
	return IRQ_HANDLED;
}

static int kdwc3_irq_init(struct dwc3_keystone *kdwc)
{
	struct device *dev = kdwc->dev;
	struct platform_device *pdev = to_platform_device(dev);
	int irq, error;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "missing irq\n");
		return -EINVAL;
	}

	error = devm_request_irq(dev, irq, dwc3_keystone_interrupt, IRQF_SHARED,
			dev_name(dev), kdwc);
	if (error) {
		dev_err(dev, "failed to request IRQ #%d --> %d\n",
				irq, error);
		return -EINVAL;
	}

	kdwc3_enable_irqs(kdwc);
	return 0;
}

static int kdwc3_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node	*node = pdev->dev.of_node;
	struct dwc3_keystone *kdwc;
	struct resource *res;
	int error;

	kdwc = devm_kzalloc(dev, sizeof(*kdwc), GFP_KERNEL);
	if (!kdwc)
		return -ENOMEM;

	platform_set_drvdata(pdev, kdwc);

	kdwc->dev = dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "missing usbss resource\n");
		return -ENODEV;
	}

	kdwc->usbss = devm_ioremap_resource(dev, res);
	if (IS_ERR(kdwc->usbss))
		return PTR_ERR(kdwc->usbss);

	dev_dbg(dev, "usbss control start=%08x size=%d mapped=%08x\n",
		(u32)(res->start), (int)resource_size(res),
		(u32)(kdwc->usbss));

	kdwc3_dma_mask = dma_get_mask(dev);
	dev->dma_mask = &kdwc3_dma_mask;

	kdwc->clk = devm_clk_get(dev, "usb");
	if (IS_ERR_OR_NULL(kdwc->clk)) {
		dev_err(dev, "unable to get kdwc usb clock\n");
		return -ENODEV;
	}

	error = clk_prepare_enable(kdwc->clk);
	if (error < 0) {
		dev_dbg(kdwc->dev, "unable to enable usb clock, err %d\n",
			error);
		return error;
	}

	error = kdwc3_irq_init(kdwc);
	if (error)
		goto err_irq;

	error = of_platform_populate(node, NULL, NULL, dev);
	if (error) {
		dev_err(&pdev->dev, "failed to create dwc3 core\n");
		goto err_core;
	}

	return 0;

err_core:
	kdwc3_disable_irqs(kdwc);
err_irq:
	clk_disable_unprepare(kdwc->clk);

	return error;
}

static int kdwc3_remove_core(struct device *dev, void *c)
{
	struct platform_device *pdev = to_platform_device(dev);

	platform_device_unregister(pdev);

	return 0;
}

static int kdwc3_remove(struct platform_device *pdev)
{
	struct dwc3_keystone *kdwc = platform_get_drvdata(pdev);

	kdwc3_disable_irqs(kdwc);
	device_for_each_child(&pdev->dev, NULL, kdwc3_remove_core);
	clk_disable_unprepare(kdwc->clk);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static const struct of_device_id kdwc3_of_match[] = {
	{ .compatible = "ti,keystone-dwc3", },
	{},
};
MODULE_DEVICE_TABLE(of, kdwc3_of_match);

static struct platform_driver kdwc3_driver = {
	.probe		= kdwc3_probe,
	.remove		= kdwc3_remove,
	.driver		= {
		.name	= "keystone-dwc3",
		.owner	        = THIS_MODULE,
		.of_match_table	= kdwc3_of_match,
	},
};

module_platform_driver(kdwc3_driver);

MODULE_ALIAS("platform:keystone-dwc3");
MODULE_AUTHOR("WingMan Kwok");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("DesignWare USB3 KEYSTONE Glue Layer");
