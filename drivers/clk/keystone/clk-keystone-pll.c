/*
 * Main PLL clk driver for Keystone devices
 *
 * Copyright (C) 2012 Texas Instruments.
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
#include <linux/platform_data/clk-keystone-pll.h>

#include <mach/cputype.h>

/**
 * struct clk_pll - DaVinci Main pll clock
 * @hw: clk_hw for the pll
 * @pll_data: PLL driver specific data
 */
struct clk_pll {
	struct clk_hw hw;
	struct clk_keystone_pll_data *pll_data;
};

#define to_clk_pll(_hw) container_of(_hw, struct clk_pll, hw)

static unsigned long clk_pllclk_recalc(struct clk_hw *hw,
					unsigned long parent_rate)
{
	struct clk_pll *pll = to_clk_pll(hw);
	struct clk_keystone_pll_data *pll_data = pll->pll_data;
	unsigned long rate = parent_rate;
	u32  pllm, plld, postdiv, val;

	/* get bit0-5 of PLLM from PLLM PLL control register */
	val = __raw_readl(pll_data->pllm);
	pllm = (val & pll_data->pllm_lower_mask);

	/* bit6-12 of PLLM is in Main PLL control register */
	val = __raw_readl(pll_data->main_pll_ctl0);
	pllm |= ((val & pll_data->pllm_upper_mask)
			>> pll_data->pllm_upper_shift);
	plld = (val & pll_data->plld_mask);
	postdiv = pll_data->fixed_postdiv;

	rate /= (plld + 1);
	rate = (rate * (pllm + 1));
	rate /= postdiv;

	pr_notice("main_pll_clk rate is %ld, postdiv = %d, pllm = %d," \
		"plld = %d\n", rate, postdiv, pllm, plld);
	return rate;
}

static const struct clk_ops clk_pll_ops = {
	.recalc_rate = clk_pllclk_recalc,
};

struct clk *clk_register_keystone_pll(struct device *dev, const char *name,
			const char *parent_name,
			struct clk_keystone_pll_data *pll_data)
{
	struct clk_init_data init;
	struct clk_pll *pll;
	struct clk *clk;

	if (!pll_data)
		return ERR_PTR(-ENODEV);

	pll = kzalloc(sizeof(*pll), GFP_KERNEL);
	if (!pll)
		return ERR_PTR(-ENOMEM);

	init.name = name;
	init.ops = &clk_pll_ops;
	init.flags = 0;
	init.parent_names = (parent_name ? &parent_name : NULL);
	init.num_parents = (parent_name ? 1 : 0);

	pll->pll_data	= pll_data;
	pll->hw.init = &init;

	clk = clk_register(NULL, &pll->hw);
	if (IS_ERR(clk))
		kfree(pll);

	return clk;
}
