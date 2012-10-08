/*
 * TI Keyston clk-pll driver platform data definitions
 *
 * Copyright (C) 2012 Texas Instruments.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef __CLK_KEYSTONE_PLL_H
#define __CLK_KEYSTONE_PLL_H

struct clk_keystone_pll_data {
	/* register holds lower bits of PLLM */
	u32 phy_pllm;
	/* holds upper bits of PLLM */
	u32 phy_main_pll_ctl0;
	/* mapped addresses. should be initialized by  */
	void __iomem *pllm;
	void __iomem *main_pll_ctl0;
	u32 pllm_lower_mask;
	u32 pllm_upper_mask;
	u32 pllm_upper_shift;
	u32 plld_mask;
	/* use this value for postdiv */
	u32 fixed_postdiv;
};

extern struct clk *clk_register_keystone_pll(struct device *dev,
			const char *name, const char *parent_name,
			struct clk_keystone_pll_data *pll_data);

#ifdef CONFIG_OF
extern void __init of_keystone_pll_clk_init(struct device_node *node);
#endif
#endif /* CLK_KEYSTONE_PLL_H */
