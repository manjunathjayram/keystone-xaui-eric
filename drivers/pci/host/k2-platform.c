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

#define PCIE0_RC_MODE		(BIT(2))
#define PCIE0_MODE_MASK		(BIT(1) | BIT(2))
#define PCIE1_RC_MODE		(BIT(4))
#define PCIE1_MODE_MASK		(BIT(3) | BIT(4))

/* mask bits point to bits being modified */
#define reg_rmw(addr, value, mask) \
	__raw_writel(((__raw_readl(addr) & (~(mask))) | \
			(value & (mask))), (addr))

struct serdes_config {
	u32 reg;
	u32 val;
	u32 mask;
};

static struct serdes_config k2_100mhz_pcie_5gbps_serdes[] = {
	{0x0000, 0x00000800, 0x0000ff00},
	{0x0060, 0x00041c5c, 0x00ffffff},
	{0x0064, 0x0343c700, 0xffffff00},
	{0x006c, 0x00000012, 0x000000ff},
	{0x0068, 0x00070000, 0x00ff0000},
	{0x0078, 0x0000c000, 0x0000ff00},

	{0x0200, 0x00000000, 0x000000ff},
	{0x0204, 0x5e000080, 0xff0000ff},
	{0x0208, 0x00000006, 0x000000ff},
	{0x0210, 0x00000023, 0x000000ff},
	{0x0214, 0x2e003060, 0xff00ffff},
	{0x0218, 0x76000000, 0xff000000},
	{0x022c, 0x00200002, 0x00ff00ff},
	{0x02a0, 0xffee0000, 0xffff0000},
	{0x02a4, 0x0000000f, 0x000000ff},
	{0x0204, 0x5e000000, 0xff000000},
	{0x0208, 0x00000006, 0x000000ff},
	{0x0278, 0x00002000, 0x0000ff00},
	{0x0280, 0x00280028, 0x00ff00ff},
	{0x0284, 0x2d0f0385, 0xffffffff},
	{0x0250, 0xd0000000, 0xff000000},
	{0x0284, 0x00000085, 0x000000ff},
	{0x0294, 0x20000000, 0xff000000},

	{0x0400, 0x00000000, 0x000000ff},
	{0x0404, 0x5e000080, 0xff0000ff},
	{0x0408, 0x00000006, 0x000000ff},
	{0x0410, 0x00000023, 0x000000ff},
	{0x0414, 0x2e003060, 0xff00ffff},
	{0x0418, 0x76000000, 0xff000000},
	{0x042c, 0x00200002, 0x00ff00ff},
	{0x04a0, 0xffee0000, 0xffff0000},
	{0x04a4, 0x0000000f, 0x000000ff},
	{0x0404, 0x5e000000, 0xff000000},
	{0x0408, 0x00000006, 0x000000ff},
	{0x0478, 0x00002000, 0x0000ff00},
	{0x0480, 0x00280028, 0x00ff00ff},
	{0x0484, 0x2d0f0385, 0xffffffff},
	{0x0450, 0xd0000000, 0xff000000},
	{0x0494, 0x20000000, 0xff000000},

	{0x0604, 0x00000080, 0x000000ff},
	{0x0600, 0x00000000, 0x000000ff},
	{0x0604, 0x5e000000, 0xff000000},
	{0x0608, 0x00000006, 0x000000ff},
	{0x0610, 0x00000023, 0x000000ff},
	{0x0614, 0x2e003060, 0xff00ffff},
	{0x0618, 0x76000000, 0xff000000},
	{0x062c, 0x00200002, 0x00ff00ff},
	{0x06a0, 0xffee0000, 0xffff0000},
	{0x06a4, 0x0000000f, 0x000000ff},
	{0x0604, 0x5e000000, 0xff000000},
	{0x0608, 0x00000006, 0x000000ff},
	{0x0678, 0x00002000, 0x0000ff00},
	{0x0680, 0x00280028, 0x00ff00ff},
	{0x0684, 0x2d0f0385, 0xffffffff},
	{0x0650, 0xd0000000, 0xff000000},
	{0x0694, 0x20000000, 0xff000000},

	{0x0800, 0x00000000, 0x000000ff},
	{0x0804, 0x5e000080, 0xff0000ff},
	{0x0808, 0x00000006, 0x000000ff},
	{0x0810, 0x00000023, 0x000000ff},
	{0x0814, 0x2e003060, 0xff00ffff},
	{0x0818, 0x76000000, 0xff000000},
	{0x082c, 0x00200002, 0x00ff00ff},
	{0x08a0, 0xffee0000, 0xffff0000},
	{0x08a4, 0x0000000f, 0x000000ff},
	{0x0804, 0x5e000000, 0xff000000},
	{0x0808, 0x00000006, 0x000000ff},
	{0x0878, 0x00002000, 0x0000ff00},
	{0x0880, 0x00280028, 0x00ff00ff},
	{0x0884, 0x2d0f0385, 0xffffffff},
	{0x0850, 0xd0000000, 0xff000000},
	{0x0894, 0x20000000, 0xff000000},

	{0x0a00, 0x00000100, 0x0000ff00},
	{0x0a08, 0x00e12c08, 0x00ffffff},
	{0x0a0c, 0x00000081, 0x000000ff},
	{0x0a18, 0x00e80000, 0x00ff0000},
	{0x0a30, 0x002f2f00, 0x00ffff00},
	{0x0a4c, 0xac820000, 0xffff0000},
	{0x0a54, 0xc0000000, 0xff000000},
	{0x0a58, 0x00001441, 0x0000ffff},
	{0x0a84, 0x00000301, 0x0000ffff},

	{0x0a8c, 0x81030000, 0xffff0000},
	{0x0a90, 0x00006001, 0x0000ffff},
	{0x0a94, 0x01000000, 0xff000000},
	{0x0aa0, 0x81000000, 0xff000000},
	{0x0abc, 0xff000000, 0xff000000},
	{0x0ac0, 0x0000008b, 0x000000ff},

	{0x0000, 0x00000003, 0x000000ff},
	{0x0a00, 0x0000009f, 0x000000ff},

	{0x0a44, 0x5f733d00, 0xffffff00},
	{0x0a48, 0x00fdca00, 0x00ffff00},
	{0x0a5c, 0x00000000, 0xffff0000},
	{0x0a60, 0x00008000, 0xffffffff},
	{0x0a64, 0x0c581220, 0xffffffff},
	{0x0a68, 0xe13b0602, 0xffffffff},
	{0x0a6c, 0xb8074cc1, 0xffffffff},
	{0x0a70, 0x3f02e989, 0xffffffff},
	{0x0a74, 0x00000001, 0x000000ff},
	{0x0b14, 0x00370000, 0x00ff0000},
	{0x0b10, 0x37000000, 0xff000000},
	{0x0b14, 0x0000005d, 0x000000ff},
};

int k2_pcie_platform_setup(void *pdata, struct device_node *np, int domain)
{
	struct keystone_pcie_pdata *p_data = pdata;
	void __iomem *reg_serdes_base, *devcfg;
	struct serdes_config *p;
	u32 val;
	int i;

	devcfg = of_iomap(np, 1);
	reg_serdes_base = of_iomap(np, 2);

	pr_info("keystone2_pcie_serdes_setup for domain %d\n", domain);

	if (!reg_serdes_base)
		pr_info("Assuming SERDES initialized by boot loader\n");

	if (!devcfg) {
		pr_warn("pcie device cfg bindings missing\n");
		return -EINVAL;
	}

	if (reg_serdes_base) {
		for (i = 0, p = &k2_100mhz_pcie_5gbps_serdes[0];
			i < ARRAY_SIZE(k2_100mhz_pcie_5gbps_serdes);
			i++, p++) {
			reg_rmw((reg_serdes_base + p->reg), p->val, p->mask);
			reg_dump((reg_serdes_base + p->reg), p->mask);
		}
	}

	udelay(2000);

	/* enable RC mode in devcfg */
	val = __raw_readl(devcfg);
	if (domain) {
		val &= ~PCIE1_MODE_MASK;
		val |= PCIE1_RC_MODE;
	} else {
		val &= ~PCIE0_MODE_MASK;
		val |= PCIE0_RC_MODE;
	}
	__raw_writel(val, devcfg);

	/* check if we need to enable link training */
	p_data->en_link_train =
		(of_get_property(np, "enable-linktrain", NULL) != NULL);

	pr_info("keystone2_pcie_serdes_setup done domain %d, en_link_train = %d\n",
		domain, p_data->en_link_train);
	iounmap(devcfg);
	iounmap(reg_serdes_base);
	return 0;
}
