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

#define XGE_SW_BASE		0x02f00000
#define XGE_SW_SIZE		0x00001000

#define XGE_CTRL_OFFSET		0x0c
#define XGE_SGMII_1_OFFSET	0x0114
#define XGE_SGMII_2_OFFSET	0x0214

#define PCSR_OFFSET(x)	((x == 0) ? (0x600) : (0x680))

/*
 * PCS-R registers
 */
#define PCSR_CPU_CTRL_OFFSET	0x1fd0

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
#define POR_EN			BIT(29)
#define POR_EN_MASK		BIT(29)

#define reg_rmw(addr, value, mask) \
	__raw_writel(((__raw_readl(addr) & (~(mask))) | \
			(value & (mask))), (addr))

struct serdes_cfg {
	u32 ofs;
	u32 val;
	u32 mask;
};

struct serdes_cfg cfg_phyb_1p25g_156p25mhz_cmu0[] = {
	{0x0000, 0x00800002, 0x00ff00ff},
	{0x0014, 0x00003838, 0x0000ffff},
	{0x0060, 0x1c44e438, 0xffffffff},
	{0x0064, 0x00c18400, 0x00ffffff},
	{0x0068, 0x17078200, 0xffffff00},
	{0x006c, 0x00000014, 0x000000ff},
	{0x0078, 0x0000c000, 0x0000ff00},
	{0x0000, 0x00000003, 0x000000ff},
};

struct serdes_cfg cfg_phyb_10p3125g_156p25mhz_cmu1[] = {
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
	{0x0c64, 0x8400cb02, 0xff00ffff},
	{0x0c68, 0x17078200, 0xffffff00},
	{0x0c6c, 0x00000016, 0x000000ff},
	{0x0c74, 0x00000400, 0x0000ff00},
	{0x0c78, 0x0000c000, 0x0000ff00},
	{0x0c00, 0x00000003, 0x000000ff},
};

struct serdes_cfg cfg_phyb_10p3125g_16bit_lane[] = {
	{0x0204, 0x00000080, 0x000000ff},
	{0x0208, 0x0000920d, 0x0000ffff},
	{0x0204, 0xfc000000, 0xff000000},
	{0x0208, 0x00009104, 0x0000ffff},
	{0x0210, 0x1a000000, 0xff000000},
	{0x0214, 0x00006b58, 0x00ffffff},
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
	{0x0380, 0x00000030, 0x000000ff},
	{0x03c0, 0x00000200, 0x0000ff00},
	{0x03cc, 0x00000018, 0x000000ff},
	{0x03cc, 0x00000000, 0x000000ff},
};

struct serdes_cfg cfg_phyb_10p3125g_comlane[] = {
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
	{0x0ac0, 0x0000008b, 0x000000ff},
};

struct serdes_cfg cfg_cm_c1_c2[] = {
	{0x0208, 0x00000000, 0x00000f00},
	{0x0208, 0x00000000, 0x0000001f},
	{0x0204, 0x00000000, 0x00040000},
	{0x0208, 0x000000a0, 0x000000e0},
};

static inline void keystone_xge_serdes_init(void __iomem *serdes_regs)
{
	int i;

	/* cmu0 setup */
	for (i = 0; i < ARRAY_SIZE(cfg_phyb_1p25g_156p25mhz_cmu0); i++) {
		reg_rmw(serdes_regs + cfg_phyb_1p25g_156p25mhz_cmu0[i].ofs,
			cfg_phyb_1p25g_156p25mhz_cmu0[i].val,
			cfg_phyb_1p25g_156p25mhz_cmu0[i].mask);
	}

	/* cmu1 setup */
	for (i = 0; i < ARRAY_SIZE(cfg_phyb_10p3125g_156p25mhz_cmu1); i++) {
		reg_rmw(serdes_regs + cfg_phyb_10p3125g_156p25mhz_cmu1[i].ofs,
			cfg_phyb_10p3125g_156p25mhz_cmu1[i].val,
			cfg_phyb_10p3125g_156p25mhz_cmu1[i].mask);
	}
}

/* lane is 0 based */
static inline void keystone_xge_serdes_lane_config(
			void __iomem *serdes_regs, int lane)
{
	int i;

	/* lane setup */
	for (i = 0; i < ARRAY_SIZE(cfg_phyb_10p3125g_16bit_lane); i++) {
		reg_rmw(serdes_regs +
				cfg_phyb_10p3125g_16bit_lane[i].ofs +
				(0x200 * lane),
			cfg_phyb_10p3125g_16bit_lane[i].val,
			cfg_phyb_10p3125g_16bit_lane[i].mask);
	}

	/* disable auto negotiation*/
	reg_rmw(serdes_regs + (0x200 * lane) + 0x0380,
		0x00000000, 0x00000010);

	/* disable link training */
	reg_rmw(serdes_regs + (0x200 * lane) + 0x03c0,
		0x00000000, 0x00000200);
}

static inline void keystone_xge_serdes_com_enable(void __iomem *serdes_regs)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(cfg_phyb_10p3125g_comlane); i++) {
		reg_rmw(serdes_regs + cfg_phyb_10p3125g_comlane[i].ofs,
			cfg_phyb_10p3125g_comlane[i].val,
			cfg_phyb_10p3125g_comlane[i].mask);
	}
}

static inline void keystone_xge_serdes_lane_enable(
			void __iomem *serdes_regs, int lane)
{
	/* Set Lane Control Rate */
	__raw_writel(0xe0e9e038, serdes_regs + 0x1fe0 + (4 * lane));
}

static inline void keystone_xge_serdes_phyb_rst_clr(void __iomem *serdes_regs)
{
	reg_rmw(serdes_regs + 0x0a00, 0x0000001f, 0x000000ff);
}

static inline void keystone_xge_serdes_pll_disable(void __iomem *serdes_regs)
{
	__raw_writel(0x88000000, serdes_regs + 0x1ff4);
}

static inline void keystone_xge_serdes_pll_enable(void __iomem *serdes_regs)
{
	keystone_xge_serdes_phyb_rst_clr(serdes_regs);
	__raw_writel(0xee000000, serdes_regs + 0x1ff4);
}

static int keystone_xge_wait_pll_locked(void __iomem *sw_regs)
{
	unsigned long timeout;
	int ret = 0;
	u32 val_1, val_0;

	timeout = jiffies + msecs_to_jiffies(500);
	do {
		val_0 = (__raw_readl(sw_regs + XGE_SGMII_1_OFFSET) & BIT(4));
		val_1 = (__raw_readl(sw_regs + XGE_SGMII_2_OFFSET) & BIT(4));

		if (val_1 && val_0)
			return 0;

		if (time_after(jiffies, timeout)) {
			ret = -ETIMEDOUT;
			break;
		}

		cpu_relax();
	} while (true);

	pr_info("XGE serdes not locked: time out.\n");
	return ret;

}

static inline void keystone_xge_serdes_enable_xgmii_port(
			void __iomem *sw_regs)
{
	__raw_writel(0x03, sw_regs + XGE_CTRL_OFFSET);
}

static inline void keystone_xge_serdes_setup_cm_c1_c2(void __iomem *serdes_regs,
				int lane, int cm, int c1, int c2)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(cfg_cm_c1_c2); i++) {
		reg_rmw(serdes_regs +
				cfg_cm_c1_c2[i].ofs + (0x200 * lane),
			cfg_cm_c1_c2[i].val,
			cfg_cm_c1_c2[i].mask);
	}
}

static inline void keystone_xge_reset_serdes(void __iomem *serdes_regs)
{
	/* Toggle the POR_EN bit in CONFIG.CPU_CTRL */
	/* enable POR_EN bit */
	reg_rmw(serdes_regs + PCSR_CPU_CTRL_OFFSET, POR_EN, POR_EN_MASK);
	udelay(10);

	/* disable POR_EN bit */
	reg_rmw(serdes_regs + PCSR_CPU_CTRL_OFFSET, 0, POR_EN_MASK);
	udelay(10);
}

static int keystone_xge_serdes_config(void __iomem *serdes_regs,
				      void __iomem *sw_regs)
{
	u32 ret, i;

	keystone_xge_serdes_pll_disable(serdes_regs);

	keystone_xge_serdes_init(serdes_regs);

	for (i = 0; i < 2; i++)
		keystone_xge_serdes_lane_config(serdes_regs, i);

	keystone_xge_serdes_com_enable(serdes_regs);

	/* This is EVM + RTM-BOC specific */
	for (i = 0; i < 2; i++)
		keystone_xge_serdes_setup_cm_c1_c2(serdes_regs, i, 0, 0, 5);

	keystone_xge_serdes_pll_enable(serdes_regs);

	for (i = 0; i < 2; i++)
		keystone_xge_serdes_lane_enable(serdes_regs, i);

	/* SB PLL Status Poll */
	ret = keystone_xge_wait_pll_locked(sw_regs);
	if (ret)
		return ret;

	keystone_xge_serdes_enable_xgmii_port(sw_regs);

	return ret;
}

static int keystone_xge_serdes_start(void)
{
	void __iomem *serdes_regs, *xge_sw_regs;
	u32 val;
	int ret;

	serdes_regs = ioremap(XGE_SERDES_BASE, XGE_SERDES_SIZE);
	xge_sw_regs = ioremap(XGE_SW_BASE, XGE_SW_SIZE);

	/* read COMLANE bits 4:0 */
	val = __raw_readl(serdes_regs + 0xa00);
	if (val & 0x1f) {
		pr_info("XGE: serdes in operation - reset before config\n");
		keystone_xge_reset_serdes(serdes_regs);
	}

	ret = keystone_xge_serdes_config(serdes_regs, xge_sw_regs);

	iounmap(serdes_regs);
	iounmap(xge_sw_regs);
	return ret;
}

static int keystone_xge_serdes_configured;  /* FIXME */

int xge_serdes_init_156p25Mhz(void)
{
	int ret;

	/* Serdes should only be configured once */
	if (keystone_xge_serdes_configured)
		return 0;

	ret = keystone_xge_serdes_start();
	if (!ret)
		keystone_xge_serdes_configured = 1;

	return ret;
}

int keystone_pcsr_config(void __iomem *pcsr_ofs, int port, u32 interface)
{
	return 0;
}
