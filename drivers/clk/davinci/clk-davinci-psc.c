/*
 * PSC clk driver for DaVinci devices
 *
 * Copyright (C) 2006-2012 Texas Instruments.
 * Copyright (C) 2008-2009 Deep Root Systems, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/platform_data/clk-davinci-psc.h>

/* PSC register offsets */
#define EPCPR		0x070
#define PTCMD		0x120
#define PTSTAT		0x128
#define PDSTAT		0x200
#define PDCTL		0x300
#define MDSTAT		0x800
#define MDCTL		0xA00

/* PSC module states */
#define PSC_STATE_SWRSTDISABLE	0
#define PSC_STATE_SYNCRST	1
#define PSC_STATE_DISABLE	2
#define PSC_STATE_ENABLE	3

#define MDSTAT_STATE_MASK	0x3f
#define PDSTAT_STATE_MASK	0x1f
#define MDCTL_FORCE		BIT(31)
#define PDCTL_NEXT		BIT(0)
#define PDCTL_EPCGOOD		BIT(8)

/* PSC flags */
#define PSC_SWRSTDISABLE	BIT(0) /* Disable state is SwRstDisable */
#define PSC_FORCE		BIT(1) /* Force module state transtition */
#define PSC_HAS_EXT_POWER_CNTL	BIT(2) /* PSC has external power control
					* available (for DM6446 SoC) */
/**
 * struct clk_psc - DaVinci PSC clock
 * @hw: clk_hw for the psc
 * @psc_data: PSC driver specific data
 * @lock: Spinlock used by the driver
 */
struct clk_psc {
	struct clk_hw hw;
	struct clk_davinci_psc_data *psc_data;
	spinlock_t *lock;
};

#define to_clk_psc(_hw) container_of(_hw, struct clk_psc, hw)

/* Enable or disable a PSC domain */
static void clk_psc_config(void __iomem *base, unsigned int domain,
		unsigned int id, bool enable, u32 flags)
{
	u32 epcpr, ptcmd, ptstat, pdstat, pdctl, mdstat, mdctl;
	u32 next_state = PSC_STATE_ENABLE;
	void __iomem *psc_base = base;

	if (!enable) {
		if (flags & PSC_SWRSTDISABLE)
			next_state = PSC_STATE_SWRSTDISABLE;
		else
			next_state = PSC_STATE_DISABLE;
	}

	mdctl = __raw_readl(psc_base + MDCTL + 4 * id);
	mdctl &= ~MDSTAT_STATE_MASK;
	mdctl |= next_state;
	if (flags & PSC_FORCE)
		mdctl |= MDCTL_FORCE;
	__raw_writel(mdctl, psc_base + MDCTL + 4 * id);

	pdstat = __raw_readl(psc_base + PDSTAT + 4 * domain);
	if ((pdstat & PDSTAT_STATE_MASK) == 0) {
		pdctl = __raw_readl(psc_base + PDCTL + 4 * domain);
		pdctl |= PDCTL_NEXT;
		__raw_writel(pdctl, psc_base + PDCTL + 4 * domain);

		ptcmd = 1 << domain;
		__raw_writel(ptcmd, psc_base + PTCMD);

		if (flags & PSC_HAS_EXT_POWER_CNTL) {
			do {
				epcpr = __raw_readl(psc_base + EPCPR);
			} while ((((epcpr >> domain) & 1) == 0));
		}

		pdctl = __raw_readl(psc_base + PDCTL + 4 * domain);
		pdctl |= 0x100;
		__raw_writel(pdctl, psc_base + PDCTL + 4 * domain);

		pdctl = __raw_readl(psc_base + PDCTL + 4 * domain);
		pdctl |= PDCTL_EPCGOOD;
		__raw_writel(pdctl, psc_base + PDCTL + 4 * domain);
	} else {
		ptcmd = 1 << domain;
		__raw_writel(ptcmd, psc_base + PTCMD);
	}

	do {
		ptstat = __raw_readl(psc_base + PTSTAT);
	} while (!(((ptstat >> domain) & 1) == 0));

	do {
		mdstat = __raw_readl(psc_base + MDSTAT + 4 * id);
	} while (!((mdstat & MDSTAT_STATE_MASK) == next_state));
}

static int clk_psc_is_enabled(struct clk_hw *hw)
{
	struct clk_psc *psc = to_clk_psc(hw);
	struct clk_davinci_psc_data *psc_data = psc->psc_data;
	u32 mdstat;

	mdstat = __raw_readl(psc_data->base + MDSTAT + 4 * psc_data->lpsc);
	/* if clocked, state can be "Enable" or "SyncReset" */
	return (mdstat & BIT(12)) ? 1 : 0;
}

static int clk_psc_enable(struct clk_hw *hw)
{
	struct clk_psc *psc = to_clk_psc(hw);
	struct clk_davinci_psc_data *psc_data = psc->psc_data;
	unsigned long flags = 0;

	if (psc->lock)
		spin_lock_irqsave(psc->lock, flags);

	clk_psc_config(psc_data->base, psc_data->domain, psc_data->lpsc,
			1, psc_data->psc_flags);

	if (psc->lock)
		spin_unlock_irqrestore(psc->lock, flags);

	return 0;
}

static void clk_psc_disable(struct clk_hw *hw)
{
	struct clk_psc *psc = to_clk_psc(hw);
	struct clk_davinci_psc_data *psc_data = psc->psc_data;
	unsigned long flags = 0;

	if (psc->lock)
		spin_lock_irqsave(psc->lock, flags);

	clk_psc_config(psc_data->base, psc_data->domain, psc_data->lpsc,
			0, psc_data->psc_flags);

	if (psc->lock)
		spin_unlock_irqrestore(psc->lock, flags);
}

static const struct clk_ops clk_psc_ops = {
	.enable = clk_psc_enable,
	.disable = clk_psc_disable,
	.is_enabled = clk_psc_is_enabled,
};

struct clk *clk_register_davinci_psc(struct device *dev, const char *name,
			const char *parent_name,
			struct clk_davinci_psc_data *psc_data,
			spinlock_t *lock)
{
	struct clk_init_data init;
	struct clk_psc *psc;
	struct clk *clk;

	psc = kzalloc(sizeof(*psc), GFP_KERNEL);
	if (!psc)
		return ERR_PTR(-ENOMEM);

	init.name = name;
	init.ops = &clk_psc_ops;
	init.flags = psc_data->flags;
	init.parent_names = (parent_name ? &parent_name : NULL);
	init.num_parents = (parent_name ? 1 : 0);

	psc->psc_data = psc_data;
	psc->lock = lock;
	psc->hw.init = &init;

	clk = clk_register(NULL, &psc->hw);
	if (IS_ERR(clk))
		kfree(psc);

	return clk;
}
