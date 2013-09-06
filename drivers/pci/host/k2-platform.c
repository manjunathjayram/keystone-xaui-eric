/*
 * Copyright 2013 Texas Instruments, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <asm/setup.h>

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_address.h>

#include "pci-pdata.h"

#define reg_dump(addr, mask) \
		pr_debug("reg %p has value %x\n", (void *)addr, \
				(__raw_readl(addr) & ~mask))

#define PCIE_RC_MODE		(BIT(2))
#define PCIE_MODE_MASK		(BIT(1) | BIT(2))

static inline void reg_rmw(void __iomem *addr, u32 mask, u32 val)
{
	u32 read_data, data;
	read_data = __raw_readl(addr);
	data = (val & ~mask) | (read_data & mask);
	__raw_writel(data, addr);
}

struct serdes_config {
	u32 reg;
	u32 mask;
	u32 val;
};

static struct serdes_config k2_pcie_serdes_cfg[] = {
	{ 0x000, 0xffff00ff, 0x00000800 },
	{ 0x060, 0xff000000, 0x00041c5c },
	{ 0x064, 0x000000ff, 0x0343c700 },
	{ 0x06c, 0xffffff00, 0x00000012 },
	{ 0x068, 0xff00ffff, 0x00070000 },
	{ 0x078, 0xffff00ff, 0x0000c000 },
	{ 0x200, 0xffffff00, 0x00000000 },
	{ 0x204, 0x00ffff00, 0x5e000080 },
	{ 0x208, 0xffffff00, 0x00000006 },
	{ 0x210, 0xffffff00, 0x00000023 },
	{ 0x214, 0x00ff0000, 0x2e003060 },
	{ 0x218, 0x00ffffff, 0x76000000 },
	{ 0x22c, 0xff00ff00, 0x00100002 },
	{ 0x2a0, 0x0000ffff, 0xffee0000 },
	{ 0x2a4, 0xffffff00, 0x0000000f },
	{ 0x204, 0x00ffffff, 0x5e000000 },
	{ 0x208, 0xffffff00, 0x00000006 },
	{ 0x278, 0xffff00ff, 0x00002000 },
	{ 0x280, 0xff00ff00, 0x00280028 },
	{ 0x284, 0x00000000, 0x2d0f0385 },
	{ 0x250, 0x00ffffff, 0xd0000000 },
	{ 0x284, 0xffffff00, 0x00000085 },
	{ 0x294, 0x00ffffff, 0x20000000 },

	{ 0x400, 0xffffff00, 0x00000000 },
	{ 0x404, 0x00ffff00, 0x5e000080 },
	{ 0x408, 0xffffff00, 0x00000006 },
	{ 0x410, 0xffffff00, 0x00000023 },
	{ 0x414, 0x00ff0000, 0x2e003060 },
	{ 0x418, 0x00ffffff, 0x76000000 },
	{ 0x42c, 0xff00ff00, 0x00100002 },
	{ 0x4a0, 0x0000ffff, 0xffee0000 },
	{ 0x4a4, 0xffffff00, 0x0000000f },
	{ 0x404, 0x00ffffff, 0x5e000000 },
	{ 0x408, 0xffffff00, 0x00000006 },
	{ 0x478, 0xffff00ff, 0x00002000 },
	{ 0x480, 0xff00ff00, 0x00280028 },
	{ 0x484, 0x00000000, 0x2d0f0385 },
	{ 0x450, 0x00ffffff, 0xd0000000 },
	{ 0x494, 0x00ffffff, 0x20000000 },

	{ 0x604, 0xffffff00, 0x00000080 },
	{ 0x600, 0xffffff00, 0x00000000 },
	{ 0x604, 0x00ffffff, 0x5e000000 },
	{ 0x608, 0xffffff00, 0x00000006 },
	{ 0x610, 0xffffff00, 0x00000023 },
	{ 0x614, 0x00ff0000, 0x2e003060 },
	{ 0x618, 0x00ffffff, 0x76000000 },
	{ 0x62c, 0xff00ff00, 0x00100002 },
	{ 0x6a0, 0x0000ffff, 0xffee0000 },
	{ 0x6a4, 0xffffff00, 0x0000000f },
	{ 0x604, 0x00ffffff, 0x5e000000 },
	{ 0x608, 0xffffff00, 0x00000006 },
	{ 0x678, 0xffff00ff, 0x00002000 },
	{ 0x680, 0xff00ff00, 0x00280028 },
	{ 0x684, 0x00000000, 0x2d0f0385 },
	{ 0x650, 0x00ffffff, 0xd0000000 },
	{ 0x694, 0x00ffffff, 0x20000000 },

	{ 0x800, 0xffffff00, 0x00000000 },
	{ 0x804, 0x00ffff00, 0x5e000080 },
	{ 0x808, 0xffffff00, 0x00000006 },
	{ 0x810, 0xffffff00, 0x00000023 },
	{ 0x814, 0x00ff0000, 0x2e003060 },
	{ 0x818, 0x00ffffff, 0x76000000 },
	{ 0x82c, 0xff00ff00, 0x00100002 },
	{ 0x8a0, 0x0000ffff, 0xffee0000 },
	{ 0x8a4, 0xffffff00, 0x0000000f },
	{ 0x804, 0x00ffffff, 0x5e000000 },
	{ 0x808, 0xffffff00, 0x00000006 },
	{ 0x878, 0xffff00ff, 0x00002000 },
	{ 0x880, 0xff00ff00, 0x00280028 },
	{ 0x884, 0x00000000, 0x2d0f0385 },
	{ 0x850, 0x00ffffff, 0xd0000000 },
	{ 0x894, 0x00ffffff, 0x20000000 },

	{ 0xa00, 0xffff00ff, 0x00000100 },
	{ 0xa08, 0xff000000, 0x00e12c08 },
	{ 0xa0c, 0xffffff00, 0x00000081 },
	{ 0xa18, 0xff00ffff, 0x00e80000 },
	{ 0xa30, 0x00ffff00, 0x002f2f00 },
	{ 0xa48, 0xff0000ff, 0x00e3ce00 },
	{ 0xa4c, 0x0000ffff, 0xac820000 },
	{ 0xa54, 0x00ffffff, 0xc0000000 },
	{ 0xa58, 0xffff0000, 0x00001441 },
	{ 0xa84, 0xffff0000, 0x00000301 },

	{ 0xa8c, 0x0000ffff, 0x81030000 },
	{ 0xa90, 0xffff0000, 0x00006001 },
	{ 0xa94, 0x00ffffff, 0x01000000 },
	{ 0xaa0, 0x00ffffff, 0x81000000 },
	{ 0xabc, 0x00ffffff, 0xff000000 },
	{ 0xac0, 0xffffff00, 0x0000008b },

	{ 0x000, 0xffffff00, 0x00000003 },
	{ 0xa00, 0xffffff00, 0x0000009f },
};

int k2_pcie_platform_setup(void *pdata, struct device_node *np)
{
	struct keystone_pcie_pdata *p_data = pdata;
	void __iomem *reg_serdes_base, *devcfg;
	u32 val;
	int i;

	devcfg = of_iomap(np, 1);
	reg_serdes_base = of_iomap(np, 2);

	pr_info("keystone2_pcie_serdes_setup\n");

	if (!reg_serdes_base)
		pr_info("Assuming SERDES initialized by boot loader\n");

	if (!devcfg) {
		pr_warn("pcie device cfg bindings missing\n");
		return -EINVAL;
	}

	if (reg_serdes_base) {
		for (i = 0; i < ARRAY_SIZE(k2_pcie_serdes_cfg); i++) {
			reg_rmw((reg_serdes_base + k2_pcie_serdes_cfg[i].reg),
				k2_pcie_serdes_cfg[i].mask,
				k2_pcie_serdes_cfg[i].val);
			reg_dump((reg_serdes_base + k2_pcie_serdes_cfg[i].reg),
				k2_pcie_serdes_cfg[i].mask);
		}
	}

	udelay(2000);

	/* enable RC mode in devcfg */
	val = __raw_readl(devcfg);
	val &= ~PCIE_MODE_MASK;
	val |= PCIE_RC_MODE;
	__raw_writel(val, devcfg);

	/* check if we need to enable link training */
	p_data->en_link_train =
		(of_get_property(np, "enable-linktrain", NULL) != NULL);

	pr_info("keystone2_pcie_serdes_setup done, en_link_train = %d\n",
		p_data->en_link_train);
	return 0;
}
