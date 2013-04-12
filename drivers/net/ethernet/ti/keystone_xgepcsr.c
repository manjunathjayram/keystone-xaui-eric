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
	__raw_writel(((__raw_readl(addr) & (~(mask))) | (value)), (addr))

struct serdes_cfg {
	u32 ofs;
	u32 val;
	u32 mask;
};

struct serdes_cfg cfg_cmu0_1g_slow[] = {
	{0x0000, 0x00800002, 0x00ff00ff},
	{0x0014, 0x00003838, 0x0000ffff},
	{0x0060, 0x1c44e438, 0xffffffff},
	{0x0064, 0x00c18400, 0x00ffffff},
	{0x0068, 0x17078200, 0xffffff00},
	{0x006c, 0x00000014, 0x000000ff},
	{0x0078, 0x0000c000, 0x0000ff00},
	{0x0000, 0x00000003, 0x000000ff}
};

struct serdes_cfg cfg_cmu1_10g_slow[] = {
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
	{0x0c78, 0x0000c000, 0x0000ff00},
	{0x0c00, 0x00000003, 0x000000ff}
};

struct serdes_cfg cfg_lane1_10g_slow[] = {
	{0x0204, 0x00000080, 0x000000ff},
	{0x0208, 0x0000920d, 0x0000ffff},
	{0x0204, 0xfc000000, 0xff000000},
	{0x0208, 0x00009104, 0x0000ffff},
	{0x0210, 0x1a000000, 0xff000000},
	/* 58 Lane address 0x14 pma_ln_ctrl[32] should be set to 1.
	   This is dlpf_div2_ena bit and controls the clock speed for
	   updating the DLPF integral path in the CDR.
	*/
	{0x0214, 0x0000b65c, 0x00ffffff},
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

struct serdes_cfg cfg_lane2_10g_slow[] = {
	{0x0404, 0x00000080, 0x000000ff},
	{0x0408, 0x0000920d, 0x0000ffff},
	{0x0404, 0xfc000000, 0xff000000},
	{0x0408, 0x00009104, 0x0000ffff},
	{0x0410, 0x1a000000, 0xff000000},
	/* 58 Lane address 0x14 pma_ln_ctrl[32] should be set to 1.
	   This is dlpf_div2_ena bit and controls the clock speed
	   for updating the DLPF integral path in the CDR.
	*/
	{0x0414, 0x0000b6b5c, 0x00ffffff},
	{0x0418, 0x75800084, 0xffff00ff},
	{0x042c, 0x00300000, 0x00ff0000},
	{0x0430, 0x00003800, 0x0000ff00},
	{0x044c, 0x008f0000, 0x00ff0000},
	{0x0450, 0x30000000, 0xff000000},
	{0x0460, 0x00000002, 0x000000ff},
	{0x0464, 0x00000057, 0x000000ff},
	{0x0468, 0x00575700, 0x00ffff00},
	{0x0478, 0xff000000, 0xff000000},
	{0x0480, 0x00500050, 0x00ff00ff},
	{0x0484, 0x00001f15, 0x0000ffff},
	{0x048c, 0x00006f00, 0x0000ff00},
	{0x0494, 0x00000000, 0xffffff00},
	{0x0498, 0x00002640, 0xff00ffff},
	{0x049c, 0x00000003, 0x000000ff},
	{0x04a4, 0x00000f13, 0x0000ffff},
	{0x04a8, 0x0001b600, 0x00ffff00},
	/* CJT, sb=0x30, disable aneg */
	{0x0580, 0x00000020, 0x000000ff},
	/* CJT, sb=0x30, disable training */
	{0x05c0, 0x00000000, 0x0000ff00},
	{0x05cc, 0x00000018, 0x000000ff},
	{0x05cc, 0x00000000, 0x000000ff}
};

struct serdes_cfg cfg_comlane_10g_slow[] = {
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

void xge_serdes_10gbps_setup_phy_b(void)
{
	void __iomem *regs, *sgmii_1_status, *sgmii_2_status;
	u32 i, val;

	regs = ioremap(XGE_SERDES_BASE, XGE_SERDES_SIZE);
	sgmii_1_status = ioremap(XGE_SGMII_1_STATUS, 4);
	sgmii_2_status = ioremap(XGE_SGMII_2_STATUS, 4);

	/* preamble */
	__raw_writel(37500, regs + 0x1ffc);

	/* cmu0_1g_slow */
	for (i = 0; i < ARRAY_SIZE(cfg_cmu0_1g_slow); i++) {
		reg_rmw(regs + cfg_cmu0_1g_slow[i].ofs,
			cfg_cmu0_1g_slow[i].val,
			cfg_cmu0_1g_slow[i].mask);
	}

	/* cmu1_10g_slow */
	for (i = 0; i < ARRAY_SIZE(cfg_cmu1_10g_slow); i++) {
		reg_rmw(regs + cfg_cmu1_10g_slow[i].ofs,
			cfg_cmu1_10g_slow[i].val,
			cfg_cmu1_10g_slow[i].mask);
	}

	/* lane1_10g_slow */
	for (i = 0; i < ARRAY_SIZE(cfg_lane1_10g_slow); i++) {
		reg_rmw(regs + cfg_lane1_10g_slow[i].ofs,
			cfg_lane1_10g_slow[i].val,
			cfg_lane1_10g_slow[i].mask);
	}

	/* lane2_10g_slow */
	for (i = 0; i < ARRAY_SIZE(cfg_lane2_10g_slow); i++) {
		reg_rmw(regs + cfg_lane2_10g_slow[i].ofs,
			cfg_lane2_10g_slow[i].val,
			cfg_lane2_10g_slow[i].mask);
	}

	/* comlane_10g_slow */
	for (i = 0; i < ARRAY_SIZE(cfg_comlane_10g_slow); i++) {
		reg_rmw(regs + cfg_comlane_10g_slow[i].ofs,
			cfg_comlane_10g_slow[i].val,
			cfg_comlane_10g_slow[i].mask);
	}

	/* reset_clr */
	reg_rmw(regs + 0x0a00, 0x0000005f, 0x000000ff);

	/*Enable pll via the pll_ctrl 0x0014*/
	__raw_writel(0xee000000, regs + 0x1ff4);

	/* Enable TX and RX via the LANExCTL_STS 0x0000 + x*4 */
	/* Full Rate mode, 16b width */
	__raw_writel(0xe0e9e038, regs + 0x1fe0);
	__raw_writel(0xe0e9e038, regs + 0x1fe4);
	udelay(200);

	/* Wait for SGMII Serdes PLL lock */
	do {
		val = __raw_readl(sgmii_1_status);
	} while ((val & 0x10) != 0x10);

	do {
		/* sgmii_2 status */
	} while ((val & 0x10) != 0x10);

	iounmap(regs);
}

static int keystone_xge_serdes_configured;  /* FIXME */

void xge_serdes_init_156p25Mhz(void)
{
	/* Serdes should only be configured once */
	if (keystone_xge_serdes_configured)
		return;

	xge_serdes_10gbps_setup_phy_b();
	keystone_xge_serdes_configured = 1;
}

int keystone_pcsr_config(void __iomem *pcsr_ofs, int port, u32 interface)
{
	return 0;
}
