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

/* bit mask from bit-a to bit-b inclusive */
#define MASK(msb, lsb) \
	((((msb) - (lsb)) == 31) ? 0xffffffff :  \
		((((u32)1 << ((msb) - (lsb) + 1)) - 1) << (lsb)))

#define FINSR(base, offset, msb, lsb, val) \
	reg_rmw((base) + (offset), ((val) << (lsb)), MASK((msb), (lsb)))

struct serdes_config {
	u32 ofs;
	u32 msb;
	u32 lsb;
	u32 val;
};

static struct serdes_config k2_100mhz_pcie_5gbps_serdes[] = {
	{0x0000, 15,  8, 0x08},
	{0x0060,  7,  0, 0x5c},
	{0x0060, 15,  8, 0x1c},
	{0x0060, 23, 16, 0x04},
	{0x0064, 15,  8, 0xc7},
	{0x0064, 23, 16, 0x43},
	{0x0064, 31, 24, 0x03},
	{0x006c,  7,  0, 0x12},
	{0x0068, 23, 16, 0x07},
	{0x0078, 15,  8, 0xc0},
	{0x0200,  7,  0, 0x00},
	{0x0204,  7,  0, 0x80},
	{0x0204, 31, 24, 0x5e},
	{0x0208,  7,  0, 0x06},
	{0x0210,  7,  0, 0x23},
	{0x0214,  7,  0, 0x60},
	{0x0214, 15,  8, 0x30},
	{0x0214, 31, 24, 0x2e},
	{0x0218, 31, 24, 0x76},
	{0x022c,  7,  0, 0x02},
	{0x022c, 23, 16, 0x20},
	{0x02a0, 23, 16, 0xee},
	{0x02a0, 31, 24, 0xff},
	{0x02a4,  7,  0, 0x0f},
	{0x0204, 31, 24, 0x5e},
	{0x0208,  7,  0, 0x06},
	{0x0278, 15,  8, 0x20},
	{0x0280,  7,  0, 0x28},
	{0x0280, 23, 16, 0x28},
	{0x0284,  7,  0, 0x85},
	{0x0284, 15,  8, 0x03},
	{0x0284, 23, 16, 0x0f},
	{0x0284, 31, 24, 0x2d},
	{0x0250, 31, 24, 0xd0},
	{0x0284,  7,  0, 0x85},
	{0x0294, 31, 24, 0x20},
	{0x0400,  7,  0, 0x00},
	{0x0404,  7,  0, 0x80},
	{0x0404, 31, 24, 0x5e},
	{0x0408,  7,  0, 0x06},
	{0x0410,  7,  0, 0x23},
	{0x0414,  7,  0, 0x60},
	{0x0414, 15,  8, 0x30},
	{0x0414, 31, 24, 0x2e},
	{0x0418, 31, 24, 0x76},
	{0x042c,  7,  0, 0x02},
	{0x042c, 23, 16, 0x20},
	{0x04a0, 23, 16, 0xee},
	{0x04a0, 31, 24, 0xff},
	{0x04a4,  7,  0, 0x0f},
	{0x0404, 31, 24, 0x5e},
	{0x0408,  7,  0, 0x06},
	{0x0478, 15,  8, 0x20},
	{0x0480,  7,  0, 0x28},
	{0x0480, 23, 16, 0x28},
	{0x0484,  7,  0, 0x85},
	{0x0484, 15,  8, 0x03},
	{0x0484, 23, 16, 0x0f},
	{0x0484, 31, 24, 0x2d},
	{0x0450, 31, 24, 0xd0},
	{0x0494, 31, 24, 0x20},
	{0x0604,  7,  0, 0x80},
	{0x0600,  7,  0, 0x00},
	{0x0604, 31, 24, 0x5e},
	{0x0608,  7,  0, 0x06},
	{0x0610,  7,  0, 0x23},
	{0x0614,  7,  0, 0x60},
	{0x0614, 15,  8, 0x30},
	{0x0614, 31, 24, 0x2e},
	{0x0618, 31, 24, 0x76},
	{0x062c,  7,  0, 0x02},
	{0x062c, 23, 16, 0x20},
	{0x06a0, 23, 16, 0xee},
	{0x06a0, 31, 24, 0xff},
	{0x06a4,  7,  0, 0x0f},
	{0x0604, 31, 24, 0x5e},
	{0x0608,  7,  0, 0x06},
	{0x0678, 15,  8, 0x20},
	{0x0680,  7,  0, 0x28},
	{0x0680, 23, 16, 0x28},
	{0x0684,  7,  0, 0x85},
	{0x0684, 15,  8, 0x03},
	{0x0684, 23, 16, 0x0f},
	{0x0684, 31, 24, 0x2d},
	{0x0650, 31, 24, 0xd0},
	{0x0694, 31, 24, 0x20},
	{0x0800,  7,  0, 0x00},
	{0x0804,  7,  0, 0x80},
	{0x0804, 31, 24, 0x5e},
	{0x0808,  7,  0, 0x06},
	{0x0810,  7,  0, 0x23},
	{0x0814,  7,  0, 0x60},
	{0x0814, 15,  8, 0x30},
	{0x0814, 31, 24, 0x2e},
	{0x0818, 31, 24, 0x76},
	{0x082c,  7,  0, 0x02},
	{0x082c, 23, 16, 0x20},
	{0x08a0, 23, 16, 0xee},
	{0x08a0, 31, 24, 0xff},
	{0x08a4,  7,  0, 0x0f},
	{0x0804, 31, 24, 0x5e},
	{0x0808,  7,  0, 0x06},
	{0x0878, 15,  8, 0x20},
	{0x0880,  7,  0, 0x28},
	{0x0880, 23, 16, 0x28},
	{0x0884,  7,  0, 0x85},
	{0x0884, 15,  8, 0x03},
	{0x0884, 23, 16, 0x0f},
	{0x0884, 31, 24, 0x2d},
	{0x0850, 31, 24, 0xd0},
	{0x0894, 31, 24, 0x20},
	{0x0a00, 15,  8, 0x01},
	{0x0a08,  7,  0, 0x08},
	{0x0a08, 15,  8, 0x2c},
	{0x0a08, 23, 16, 0xe1},
	{0x0a0c,  7,  0, 0x81},
	{0x0a18, 23, 16, 0xe8},
	{0x0a30, 15,  8, 0x2f},
	{0x0a30, 23, 16, 0x2f},
	{0x0a4c, 23, 16, 0x82},
	{0x0a4c, 31, 24, 0xac},
	{0x0a54, 31, 24, 0xc0},
	{0x0a58,  7,  0, 0x41},
	{0x0a58, 15,  8, 0x14},
	{0x0a84,  7,  0, 0x01},
	{0x0a84, 15,  8, 0x03},
	{0x0a8c, 23, 16, 0x03},
	{0x0a8c, 31, 24, 0x81},
	{0x0a90,  7,  0, 0x01},
	{0x0a90, 15,  8, 0x60},
	{0x0a94, 31, 24, 0x01},
	{0x0aa0, 31, 24, 0x81},
	{0x0abc, 31, 24, 0xff},
	{0x0ac0,  7,  0, 0x8b},
	{0x0a44, 15,  8, 0x3d},
	{0x0a44, 23, 16, 0x73},
	{0x0a44, 31, 24, 0x5f},
	{0x0a48, 15,  8, 0xca},
	{0x0a48, 23, 16, 0xfd},
	{0x0a5c, 23, 16, 0x00},
	{0x0a5c, 31, 24, 0x00},
	{0x0a60,  7,  0, 0x00},
	{0x0a60, 15,  8, 0x80},
	{0x0a60, 23, 16, 0x00},
	{0x0a60, 31, 24, 0x00},
	{0x0a64,  7,  0, 0x20},
	{0x0a64, 15,  8, 0x12},
	{0x0a64, 23, 16, 0x58},
	{0x0a64, 31, 24, 0x0c},
	{0x0a68,  7,  0, 0x02},
	{0x0a68, 15,  8, 0x06},
	{0x0a68, 23, 16, 0x3b},
	{0x0a68, 31, 24, 0xe1},
	{0x0a6c,  7,  0, 0xc1},
	{0x0a6c, 15,  8, 0x4c},
	{0x0a6c, 23, 16, 0x07},
	{0x0a6c, 31, 24, 0xb8},
	{0x0a70,  7,  0, 0x89},
	{0x0a70, 15,  8, 0xe9},
	{0x0a70, 23, 16, 0x02},
	{0x0a70, 31, 24, 0x3f},
	{0x0a74,  7,  0, 0x01},
	{0x0b14, 23, 16, 0x37},
	{0x0b10, 31, 24, 0x37},
	{0x0b14,  7,  0, 0x5d},
	{0x0000,  7,  0, 0x03},
	{0x0a00,  7,  0, 0x9f},
};

int k2_pcie_platform_setup(void *pdata, struct device_node *np, int domain)
{
	struct serdes_config *p = &k2_100mhz_pcie_5gbps_serdes[0];
	struct keystone_pcie_pdata *p_data = pdata;
	void __iomem *reg_serdes_base, *devcfg;
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
		for (i = 0; i < ARRAY_SIZE(k2_100mhz_pcie_5gbps_serdes); i++) {
			FINSR(reg_serdes_base, p[i].ofs, p[i].msb, p[i].lsb,
				p[i].val);
			reg_dump((reg_serdes_base + p[i].ofs), MASK(p[i].msb,
				p[i].lsb));
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
