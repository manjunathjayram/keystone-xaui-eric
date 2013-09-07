/*
 * Copyright (C) 2012 Texas Instruments Incorporated
 * Author: WingMan Kwok <w-kwok2@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/delay.h>

#include "keystone_net.h"

#define XGE_SERDES_BASE		0x0231E000
#define XGE_SERDES_SIZE		0x2000

#define XGE_SGMII_1_STATUS	0x02f00114
#define XGE_SGMII_2_STATUS	0x02f00214
#define XGE_SW_BASE		0x02f00000
#define XGE_SW_SIZE		0x00001000

#define PCSR_OFFSET(x)	((x == 0) ? (0x600) : (0x680))

/*
 * PCS-R registers
 */
#define PCSR_TX_CTL(x)		(PCSR_OFFSET(x) + 0x00)
#define PCSR_TX_STATUS(x)	(PCSR_OFFSET(x) + 0x04)
#define PCSR_RX_CTL(x)		(PCSR_OFFSET(x) + 0x08)
#define PCSR_RX_STATUS(x)	(PCSR_OFFSET(x) + 0x0C)

#define PCSR_SEED_A_LO(x)	(PCSR_OFFSET(x) + 0x10)
#define PCSR_SEED_A_HI(x)	(PCSR_OFFSET(x) + 0x14)
#define PCSR_SEED_B_LO(x)	(PCSR_OFFSET(x) + 0x18)
#define PCSR_SEED_B_HI(x)	(PCSR_OFFSET(x) + 0x1C)

#define PCSR_FEC(x)		(PCSR_OFFSET(x) + 0x20)
#define PCSR_CTL(x)		(PCSR_OFFSET(x) + 0x24)
#define PCSR_RX_FEC_CNT(x)	(PCSR_OFFSET(x) + 0x28)
#define PCSR_RX_ERR_FIFO(x)	(PCSR_OFFSET(x) + 0x2C)

#define PCSR_SIGNAL_OK_EN	BIT(1)

#define reg_rmw(addr, value, mask) \
	__raw_writel(((__raw_readl(addr) & (~(mask))) | \
			(value & (mask))), (addr))

struct serdes_cfg {
	u32 ofs;
	u32 val;
	u32 mask;
};

struct serdes_cfg cfg_cmu0_156p25mhz_10p3125g[] = {
	{0x0000, 0x00800002, 0x00ff00ff},
	{0x0014, 0x00003838, 0x0000ffff},
	{0x0060, 0x1c44e438, 0xffffffff},
	{0x0064, 0x00c18400, 0x00ffffff},
	{0x0068, 0x17078200, 0xffffff00},
	{0x006c, 0x00000014, 0x000000ff},
	{0x0078, 0x0000c000, 0x0000ff00}
};

struct serdes_cfg cfg_cmu1_156p25mhz_10p3125g[] = {
	{0x0c00, 0x00030002, 0x00ff00ff},
	{0x0c14, 0x00005252, 0x0000ffff},
	{0x0c28, 0x80000000, 0xff000000},
	{0x0c2c, 0x000000f6, 0x000000ff},
	{0x0c3c, 0x04000405, 0xff00ffff},
	{0x0c40, 0xc0800000, 0xffff0000},
	{0x0c44, 0x5a202062, 0xffffffff},
	{0x0c48, 0x40040424, 0xffffffff},
	{0x0c4c, 0x00004002, 0x0000ffff},
	{0x0c50, 0x19001c00, 0xff00ff00},
	{0x0c54, 0x00002100, 0x0000ff00},
	{0x0c58, 0x00000060, 0x000000ff},
	{0x0c60, 0x80131e7c, 0xffffffff},
	{0x0c64, 0x00008b02, 0x0000ffff},
	{0x0c68, 0x17078200, 0xffffff00},
	{0x0c6c, 0x0000001a, 0x000000ff},
	{0x0c74, 0x00000400, 0x0000ff00},
	{0x0c78, 0x0000c000, 0x0000ff00}
};

struct serdes_cfg cfg_comlane_156p25mhz_10p3125g[] = {
	{0x0a00, 0x00000800, 0x0000ff00},
	{0x0a84, 0x00000000, 0x000000ff},
	{0x0a8c, 0x00130000, 0x00ff0000},
	{0x0a90, 0x77a00000, 0xffff0000},
	{0x0a94, 0x00007777, 0x0000ffff},
	{0x0b08, 0x000f0000, 0xffff0000},
	{0x0b0c, 0x000f0000, 0x00ffffff},
	{0x0b10, 0xbe000000, 0xff000000},
	{0x0b14, 0x000000ff, 0x000000ff},
	{0x0b18, 0x00000014, 0x000000ff},
	{0x0b5c, 0x981b0000, 0xffff0000},
	{0x0b64, 0x00001100, 0x0000ff00},
	{0x0b78, 0x00000c00, 0x0000ff00},
	{0x0abc, 0xff000000, 0xff000000},
	{0x0ac0, 0x0000008b, 0x000000ff}
};

struct serdes_cfg cfg_lane_156p25mhz_10p3125g[] = {
	{0x0204, 0xfc000080, 0xff0000ff},
	{0x0208, 0x00009341, 0x0000ffff},
	{0x0210, 0x1a000000, 0xff000000},
	/* Set dlpf_div2_ena. Orig = 0x58 */
	{0x0214, 0x00006b5c, 0x00ffffff},
	{0x0218, 0x75800084, 0xffff00ff},
	{0x022c, 0x00300000, 0x00ff0000},
	{0x0230, 0x00003800, 0x0000ff00},
	{0x024c, 0x008f0000, 0x00ff0000},
	{0x0250, 0x30000000, 0xff000000},
	{0x0260, 0x00000002, 0x000000ff},
	{0x0264, 0x00000057, 0x000000ff},
	{0x0268, 0x00575700, 0x00ffff00},
	{0x0278, 0xff000000, 0xff000000},
	{0x0280, 0x00500050, 0x00ff00ff},
	{0x0284, 0x00001f15, 0x0000ffff},
	{0x028c, 0x00006f00, 0x0000ff00},
	{0x0294, 0x00000000, 0xffffff00},
	{0x0298, 0x00002640, 0xff00ffff},
	{0x029c, 0x00000003, 0x000000ff},
	{0x02a4, 0x00000f13, 0x0000ffff},
	{0x02a8, 0x0001b600, 0x00ffff00},
	/* CJT, sb=0x30, disable aneg */
	{0x0380, 0x00000020, 0x000000ff},
	/* CJT, sb=0x02, disable training */
	{0x03c0, 0x00000000, 0x0000ff00},
	{0x03cc, 0x00000018, 0x000000ff},
	{0x03cc, 0x00000000, 0x000000ff},
};

static inline void keystone_xge_serdes_init(void __iomem *serdes_regs)
{
	int i;

	/* cmu0 setup */
	for (i = 0; i < ARRAY_SIZE(cfg_cmu0_156p25mhz_10p3125g); i++) {
		reg_rmw(serdes_regs + cfg_cmu0_156p25mhz_10p3125g[i].ofs,
			cfg_cmu0_156p25mhz_10p3125g[i].val,
			cfg_cmu0_156p25mhz_10p3125g[i].mask);
	}

	/* cmu1 setup */
	for (i = 0; i < ARRAY_SIZE(cfg_cmu1_156p25mhz_10p3125g); i++) {
		reg_rmw(serdes_regs + cfg_cmu1_156p25mhz_10p3125g[i].ofs,
			cfg_cmu1_156p25mhz_10p3125g[i].val,
			cfg_cmu1_156p25mhz_10p3125g[i].mask);
	}

	/* comlane setup */
	for (i = 0; i < ARRAY_SIZE(cfg_comlane_156p25mhz_10p3125g); i++) {
		reg_rmw(serdes_regs + cfg_comlane_156p25mhz_10p3125g[i].ofs,
			cfg_comlane_156p25mhz_10p3125g[i].val,
			cfg_comlane_156p25mhz_10p3125g[i].mask);
	}
}

static inline void keystone_xge_serdes_lane_config(
			void __iomem *serdes_regs, int lane)
{
	int i;

	/* lane setup */
	for (i = 0; i < ARRAY_SIZE(cfg_lane_156p25mhz_10p3125g); i++) {
		reg_rmw(serdes_regs +
				cfg_lane_156p25mhz_10p3125g[i].ofs +
				(0x200 * lane),
			cfg_lane_156p25mhz_10p3125g[i].val,
			cfg_lane_156p25mhz_10p3125g[i].mask);
	}
}

static inline void keystone_xge_serdes_com_enable(void __iomem *serdes_regs)
{
	/* Bring SerDes out of Reset if SerDes is Shutdown & is in Reset Mode*/
	reg_rmw(serdes_regs + 0x10, 0x0, 0x10000000);
	reg_rmw(serdes_regs + 0x0c10, 0x0, 0x10000000);

	/* Enable CMU and COMLANE */
	reg_rmw(serdes_regs + 0x0000, 0x03, 0x000000ff);
	reg_rmw(serdes_regs + 0x0c00, 0x03, 0x000000ff);
	reg_rmw(serdes_regs + 0x0a00, 0x5f, 0x000000ff);
}

static inline void keystone_xge_serdes_lane_enable(
			void __iomem *serdes_regs, int lane)
{
	/* Bit 28 Toggled. Bring it out of Reset TX PLL for all lanes */
	reg_rmw(serdes_regs + 0x0200 * (lane + 1) + 0x28, 0x0, 0x20000000);

	/* Set Lane Control Rate */
	__raw_writel(0xe0e9e038, serdes_regs + 0x1fe0 + (4 * lane));

	/* Set NES bit if Loopback Enabled */
}

static inline void keystone_xge_serdes_pll_enable(void __iomem *serdes_regs,
							int enable)
{
	/* Set PLL Enable Val */
	if (enable)
		__raw_writel(0xee000000, serdes_regs + 0x1ff4);
	else
		__raw_writel(0x08000000, serdes_regs + 0x1ff4);
}

static inline void keystone_xge_wait_pll_locked(
			void __iomem *sw_regs)
{
	u32 val;

	do {
		/* sgmii 1 status */
		val = __raw_readl(sw_regs + 0x0114);
	} while ((val & 0x10) != 0x10);

	do {
		/* sgmii 2 status */
		val = __raw_readl(sw_regs + 0x0214);
	} while ((val & 0x10) != 0x10);
}

static inline void keystone_xge_serdes_enable_xgmii_port(
			void __iomem *sw_regs)
{
	__raw_writel(0x03, sw_regs + 0x0c);
}

static inline void keystone_xge_serdes_fixup(void __iomem *serdes_regs)
{
	u32 i, val;

	/* Kick the PCS block...
	   This is temporary workaround for SERDES PCS not coming
	   up correctly. Not adding to CSL just yet.
	*/
	for (i = 0; i < 2; i++) {
		val = __raw_readl(serdes_regs + 0x204 + (i * 0x200));
		reg_rmw(serdes_regs + 0x204 + (i * 0x200),
				0x00000004, 0x00000006);
		reg_rmw(serdes_regs + 0x204 + (i * 0x200),
				val & 0x00000006, 0x00000006);
	}
	/* wait... 2uS */
	udelay(20);
}

static void keystone_xge_serdes_start(void)
{
	void __iomem *serdes_regs, *xge_sw_regs;
	u32 i;

	serdes_regs = ioremap(XGE_SERDES_BASE, XGE_SERDES_SIZE);
	xge_sw_regs = ioremap(XGE_SW_BASE, XGE_SW_SIZE);

	keystone_xge_serdes_pll_enable(serdes_regs, 0);

	keystone_xge_serdes_init(serdes_regs);

	for (i = 0; i < 2; i++)
		keystone_xge_serdes_lane_config(serdes_regs, i);

	keystone_xge_serdes_com_enable(serdes_regs);

	for (i = 0; i < 2; i++)
		keystone_xge_serdes_lane_enable(serdes_regs, i);

	keystone_xge_serdes_pll_enable(serdes_regs, 1);

	/* SB PLL Status Poll */
	keystone_xge_wait_pll_locked(xge_sw_regs);

	keystone_xge_serdes_enable_xgmii_port(xge_sw_regs);

	keystone_xge_serdes_fixup(serdes_regs);

	iounmap(serdes_regs);
	iounmap(xge_sw_regs);
}

static int keystone_xge_serdes_configured;  /* FIXME */

void xge_serdes_init_156p25Mhz(void)
{
	/* Serdes should only be configured once */
	if (keystone_xge_serdes_configured)
		return;

	keystone_xge_serdes_start();
	keystone_xge_serdes_configured = 1;
}

int keystone_pcsr_config(void __iomem *pcsr_ofs, int port, u32 interface)
{
	return 0;
}
