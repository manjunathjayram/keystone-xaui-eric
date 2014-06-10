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
#include <linux/of.h>
#include <linux/err.h>
#include <linux/delay.h>

#include "keystone_net.h"

#define XGE_SERDES_BASE		0x0231e000
#define XGE_SERDES_SIZE		0x2000

#define XGE_SW_BASE		0x02f00000
#define XGE_SW_SIZE		0x00001000

#define XGE_CTRL_OFFSET		0x0c
#define XGE_SGMII_1_OFFSET	0x0114
#define XGE_SGMII_2_OFFSET	0x0214

/*
 * Keystone2 SERDES registers
 */
/* 0x1fc0 - 0x1fff */
#define K2SERDES_SS_OFFSET	0x1fc0
/* 0x1fc0 */
#define MOD_VER_REG		(K2SERDES_SS_OFFSET + 0x00)
/* 0x1fd0 */
#define CPU_CTRL_REG		(K2SERDES_SS_OFFSET + 0x10)
/* 0x1ff4 */
#define PLL_CTRL_REG		(K2SERDES_SS_OFFSET + 0x34)
/* 0x1fe0, 0x1fe4 */
#define LANE_CTRL_STS_REG(x)	(K2SERDES_SS_OFFSET + 0x20 + (x * 0x04))

/* CMU0 SS 0x0000 - 0x01ff */
#define CMU0_SS_OFFSET		0x0000
#define CMU0_REG(x)		(CMU0_SS_OFFSET + x)

/* LANE SS 0x0200 - 0x03ff, 0x0400 - 0x05ff, ... */
#define LANE0_SS_OFFSET		0x0200
#define LANEX_SS_OFFSET(x)	(LANE0_SS_OFFSET * (x + 1))
#define LANEX_REG(x, y)		(LANEX_SS_OFFSET(x) + y)

/* CML SS 0x0a00 - 0x0bff */
#define CML_SS_OFFSET		0x0a00
#define CML_REG(x)		(CML_SS_OFFSET + x)

/* CMU1 SS 0x0c00 - 0x0dff */
#define CMU1_SS_OFFSET		0x0c00
#define CMU1_REG(x)		(CMU1_SS_OFFSET + x)

/*
 * PCS-R registers
 */
#define PCSR_OFFSET(x)		(0x600 + (x * 0x80))

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

/* bit mask from bit-a to bit-b inclusive */
#define MASK(msb, lsb) \
	((((msb) - (lsb)) == 31) ? 0xffffffff :  \
		((((u32)1 << ((msb) - (lsb) + 1)) - 1) << (lsb)))

#define FINSR(base, offset, msb, lsb, val) \
	reg_rmw((base) + (offset), ((val) << (lsb)), MASK((msb), (lsb)))

#define PHY_A(serdes) \
	(0x4eba != ((k2serdes_readl(serdes, MOD_VER_REG) >> 16) & 0xffff))

#define MOD_VER(serdes) \
	((k2serdes_readl(serdes, MOD_VER_REG) >> 16) & 0xffff)

#define FOUR_LANE(serdes) \
	((0x4eb9 == MOD_VER(serdes)) || (0x4ebd == MOD_VER(serdes)))

#define K2SERDES_REF_CLOCK_156P25M	0

#define K2SERDES_LINK_RATE_10P3125G	0
#define K2SERDES_LINK_RATE_1P25G	1

#define K2SERDES_PHY_XGE		0
#define K2SERDES_PHY_SGMII		1
#define K2SERDES_PHY_PCIE		2

#define MAX_LANES			4
#define MAX_COMPARATORS			5

#define DFE_OFFSET_SAMPLES		100

struct k2serdes_comparator_tap_offsets {
	u32 cmp;
	u32 tap1;
	u32 tap2;
	u32 tap3;
	u32 tap4;
	u32 tap5;
};

struct k2serdes_lane_offsets {
	struct k2serdes_comparator_tap_offsets ct_ofs[MAX_COMPARATORS];
};

struct k2serdes_offsets {
	struct k2serdes_lane_offsets lane_ofs[MAX_LANES];
};

struct hw_specific {
	u32 ref_clock_rate;
	u32 link_rate;
	u32 lanes;
	u32 phy_type;
	u32 c1;
	u32 c2;
	u32 cm;
	u32 tx_att;
	u32 tx_vreg;
	u32 eq_vreg_enable;
	u32 eq_cdfe_enable;
	u32 eq_offset_enable;
};

struct serdes_cfg {
	u32 ofs;
	u32 msb;
	u32 lsb;
	u32 val;
};

/* offset w.r.t. CMU0 SS 0x0000 */
struct serdes_cfg cfg_phyb_1p25g_156p25mhz_cmu0[] = {
	{0x0000,	 7, 0,		0x02},
	{0x0000,	23, 16,		0x80},
	{0x0014,	 7, 0,		0x38},
	{0x0014,	15, 8,		0x38},
	{0x0060,	 7, 0,		0x38},
	{0x0060,	15, 8,		0xe4},
	{0x0060,	23, 16,		0x44},
	{0x0060,	31, 24,		0x1c},
	{0x0064,	 7, 0,		0x00},
	{0x0064,	15, 8,		0x84},
	{0x0064,	23, 16,		0xc1},
	{0x0068,	15, 8,		0x82},
	{0x0068,	23, 16,		0x07},
	{0x0068,	31, 24,		0x17},
	{0x006c,	 7, 0,		0x14},
	{0x0078,	15, 8,		0xc0},
	{0x0000,	 7, 0,		0x03},
};

/* offset w.r.t. CMU1 SS 0x0c00 */
struct serdes_cfg cfg_phyb_10p3125g_156p25mhz_cmu1[] = {
	{0x0000,	 7, 0,		0x02},
	{0x0000,	23, 16,		0x03},
	{0x0014,	 7, 0,		0x52},
	{0x0014,	15, 8,		0x52},
	{0x0028,	31, 24,		0x80},
	{0x002c,	 7, 0,		0xf6},
	{0x003c,	 7, 0,		0x05},
	{0x003c,	15, 8,		0x04},
	{0x003c,	31, 24,		0x04},
	{0x0040,	23, 16,		0x80},
	{0x0040,	31, 16,		0xc0},
	{0x0044,	 7, 0,		0x62},
	{0x0044,	15, 8,		0x20},
	{0x0044,	23, 16,		0x20},
	{0x0044,	31, 24,		0x5a},
	{0x0048,	 7, 0,		0x24},
	{0x0048,	15, 8,		0x04},
	{0x0048,	23, 16,		0x04},
	{0x0048,	31, 24,		0x40},
	{0x004c,	 7, 0,		0x02},
	{0x004c,	15, 8,		0x40},
	{0x0050,	15, 8,		0x1c},
	{0x0050,	31, 24,		0x19},
	{0x0054,	15, 8,		0x21},
	{0x0058,	 7, 0,		0x60},
	{0x0060,	 7, 0,		0x7c},
	{0x0060,	15, 8,		0x1e},
	{0x0060,	23, 16,		0x13},
	{0x0060,	31, 24,		0x80},
	{0x0064,	 7, 0,		0x02},
	{0x0064,	15, 8,		0xcb},
	{0x0064,	31, 24,		0x84},
	{0x0068,	15, 8,		0x82},
	{0x0068,	23, 16,		0x07},
	{0x0068,	31, 24,		0x17},
	{0x006c,	 7, 0,		0x16},
	{0x0074,	15, 8,		0x04},
	{0x0078,	15, 8,		0xc0},
	{0x0000,	 7, 0,		0x03},
};

/* offset w.r.t. LaneX SS 0x0200, 0x0400, ... */
struct serdes_cfg cfg_phyb_10p3125g_16bit_lane[] = {
	{0x0004,	 7, 0,		0x80},
	{0x0008,	 7, 0,		0x0d},
	{0x0008,	15, 8,		0x92},
	{0x0004,	31, 24,		0xfc},
	{0x0008,	 7, 0,		0x04},
	{0x0008,	15, 8,		0x91},
	{0x0010,	31, 24,		0x1a},
	{0x0014,	 7, 0,		0x58},
	{0x0014,	15, 8,		0x6b},
	{0x0014,	23, 16,		0x00},
	{0x0018,	 7, 0,		0x84},
	{0x0018,	23, 16,		0x80},
	{0x0018,	31, 24,		0x75},
	{0x002c,	23, 16,		0x30},
	{0x0030,	15, 8,		0x38},
	{0x004c,	23, 16,		0x8f},
	{0x0050,	31, 24,		0x30},
	{0x0060,	 7, 0,		0x02},
	{0x0064,	 7, 0,		0x57},
	{0x0068,	15, 8,		0x57},
	{0x0068,	23, 16,		0x57},
	{0x0078,	31, 24,		0xff},
	{0x0080,	 7, 0,		0x50},
	{0x0080,	23, 16,		0x50},
	{0x0084,	 7, 0,		0x15},
	{0x0084,	15, 8,		0x1f},
	{0x008c,	15, 8,		0x6f},
	{0x0094,	15, 8,		0x00},
	{0x0094,	23, 16,		0x00},
	{0x0094,	31, 24,		0x00},
	{0x0098,	 7, 0,		0x40},
	{0x0098,	15, 8,		0x26},
	{0x0098,	31, 24,		0x00},
	{0x009c,	 7, 0,		0x03},
	{0x00a4,	 7, 0,		0x13},
	{0x00a4,	15, 8,		0x0f},
	{0x00a8,	15, 8,		0xb6},
	{0x00a8,	23, 16,		0x01},
	{0x0180,	 7, 0,		0x30},
	{0x01c0,	15, 8,		0x02},
	{0x01cc,	 7, 0,		0x18},
	{0x01cc,	 7, 0,		0x00},
};

/* offset w.r.t. LaneX SS 0x0200, 0x0400, ... */
struct serdes_cfg cfg_phyb_aneg_lane[] = {
	{0x0004,	 7, 0,		0x80},
	{0x0004,	31, 24,		0x70},
	{0x0008,	 7, 0,		0x00},
	{0x0008,	15, 8,		0x00},
	{0x0010,	31, 24,		0x1b},
	{0x0014,	 7, 0,		0xf8},
	{0x0014,	15, 8,		0x6b},
	{0x0014,	23, 16,		0x00},
	{0x0018,	 7, 0,		0x74},
	{0x0018,	23, 16,		0x80},
	{0x0018,	31, 24,		0x75},
	{0x0030,	15, 8,		0x00},
	{0x004c,	23, 16,		0x8f},
	{0x0050,	31, 24,		0x00},
	{0x0060,	 7, 0,		0x00},
	{0x0064,	 7, 0,		0x57},
	{0x0068,	15, 8,		0x57},
	{0x0068,	23, 16,		0x57},
	{0x0078,	31, 24,		0xff},
	{0x0080,	 7, 0,		0x50},
	{0x0080,	23, 16,		0x50},
	{0x0084,	 7, 0,		0x05},
	{0x0094,	15, 8,		0x00},
	{0x0094,	23, 16,		0x00},
	{0x0094,	31, 24,		0x00},
	{0x0098,	 7, 0,		0x00},
	{0x0098,	15, 8,		0x00},
	{0x0098,	31, 24,		0x00},
	{0x009c,	 7, 0,		0x01},
	{0x00a4,	 7, 0,		0x0a},
	{0x00a4,	15, 8,		0x00},
	{0x00a8,	15, 8,		0xb6},
	{0x00a8,	23, 16,		0x01},
	{0x0180,	 7, 0,		0xb0},
	{0x0180,	 7, 0,		0x30},
	{0x01c0,	15, 8,		0x00},
	{0x01cc,	 7, 0,		0x18},
};

/* offset w.r.t. CML SS 0x0a00 */
struct serdes_cfg cfg_phyb_10p3125g_cml[] = {
	{0x0000,	15, 8,		0x08},
	{0x0084,	 7, 0,		0x00},
	{0x008c,	23, 16,		0x13},
	{0x0090,	23, 16,		0xa0},
	{0x0090,	31, 24,		0x77},
	{0x0094,	 7, 0,		0x77},
	{0x0094,	15, 8,		0x77},
	{0x0108,	23, 16,		0x0f},
	{0x0108,	31, 24,		0x00},
	{0x010c,	 7, 0,		0x00},
	{0x010c,	15, 8,		0x00},
	{0x010c,	23, 16,		0x0f},
	{0x0110,	31, 24,		0xbe},
	{0x0114,	 7, 0,		0xff},
	{0x0118,	 7, 0,		0x14},
	{0x015c,	23, 16,		0x1b},
	{0x015c,	31, 24,		0x98},
	{0x0164,	15, 8,		0x11},
	{0x0178,	15, 8,		0x0c},
	{0x00bc,	31, 24,		0xff},
	{0x00c0,	 7, 0,		0x8b},
};

/* offset w.r.t. CML SS 0x0a00 */
struct serdes_cfg cfg_phyb_1p25g_156p25mhz_cml[] = {
	{0x0000,	15, 8,		0x08},
	{0x0084,	 7, 0,		0x00},
	{0x0090,	23, 16,		0xa0},
	{0x0090,	31, 24,		0x77},
	{0x0094,	 7, 0,		0x77},
	{0x0094,	15, 8,		0x77},
	{0x0108,	23, 16,		0x00},
	{0x0108,	31, 24,		0x00},
	{0x010c,	 7, 0,		0x00},
	{0x010c,	15, 8,		0x00},
	{0x010c,	23, 16,		0x00},
	{0x0110,	31, 24,		0xbe},
	{0x0114,	 7, 0,		0xff},
	{0x0118,	 7, 0,		0x14},
	{0x015c,	23, 16,		0x1b},
	{0x015c,	31, 24,		0x98},
	{0x0164,	15, 8,		0x11},
	{0x0178,	15, 8,		0x0c},
	{0x00bc,	31, 24,		0xff},
	{0x00c0,	 7, 0,		0x8b},
	{0x0048,	15, 8,		0x8c},
	{0x0048,	23, 16,		0xfd},
	{0x0054,	 7, 0,		0x72},
	{0x0054,	15, 8,		0xec},
	{0x0054,	23, 16,		0x2f},
	{0x0058,	15, 8,		0x21},
	{0x0058,	23, 16,		0xf9},
	{0x0058,	31, 24,		0x00},
	{0x005c,	 7, 0,		0x60},
	{0x005c,	15, 8,		0x00},
	{0x005c,	23, 16,		0x04},
	{0x005c,	31, 24,		0x00},
	{0x0060,	 7, 0,		0x00},
	{0x0060,	15, 8,		0x80},
	{0x0060,	23, 16,		0x00},
	{0x0060,	31, 24,		0x00},
	{0x0064,	 7, 0,		0x20},
	{0x0064,	15, 8,		0x12},
	{0x0064,	23, 16,		0x58},
	{0x0064,	31, 24,		0x0c},
	{0x0068,	 7, 0,		0x02},
	{0x0068,	15, 8,		0x06},
	{0x0068,	23, 16,		0x3b},
	{0x0068,	31, 24,		0xe1},
	{0x006c,	 7, 0,		0xc1},
	{0x006c,	15, 8,		0x4c},
	{0x006c,	23, 16,		0x07},
	{0x006c,	31, 24,		0xb8},
	{0x0070,	 7, 0,		0x89},
	{0x0070,	15, 8,		0xe9},
	{0x0070,	23, 16,		0x02},
	{0x0070,	31, 24,		0x3f},
	{0x0074,	 7, 0,		0x01},
	{0x011c,	31, 24,		0x37},
	{0x0120,	 7, 0,		0x5d},
	{0x0120,	23, 16,		0x37},
};

static inline u32 k2serdes_readl(void __iomem *base, u32 offset)
{
	return readl(base + offset);
}

static inline void k2serdes_writel(void __iomem *base, u32 offset, u32 value)
{
	writel(value, base + offset);
}

static inline u32 k2serdes_read_tbus_val(void __iomem *serdes)
{
	u32 tmp;

	if (PHY_A(serdes)) {
		tmp  = ((k2serdes_readl(serdes, CMU0_REG(0xec))) >> 24) & 0x0ff;
		tmp |= ((k2serdes_readl(serdes, CMU0_REG(0xfc))) >> 16) & 0xf00;
	} else
		tmp  = ((k2serdes_readl(serdes, CMU0_REG(0xf8))) >> 16) & 0xfff;

	return tmp;
}

static void k2serdes_write_tbus_addr(void __iomem *serdes, int select, int ofs)
{
	if (select && !FOUR_LANE(serdes))
		++select;

	if (PHY_A(serdes))
		FINSR(serdes, CMU0_REG(0x8), 31, 24, ((select << 5) + ofs));
	else
		FINSR(serdes, CMU0_REG(0xfc), 26, 16, ((select << 8) + ofs));
}

u32 k2serdes_read_select_tbus(void __iomem *serdes,
				int select, int ofs)
{
	/* set tbus address */
	k2serdes_write_tbus_addr(serdes, select, ofs);
	/* get tbus value */
	return k2serdes_read_tbus_val(serdes);
}

static inline void k2serdes_config_ss(void __iomem *ss_base,
				struct serdes_cfg *serdes_cfg,
				int serdes_cfg_num)
{
	int i;

	for (i = 0; i < serdes_cfg_num; i++) {
		FINSR(ss_base, serdes_cfg[i].ofs,
			serdes_cfg[i].msb,
			serdes_cfg[i].lsb,
			serdes_cfg[i].val);
	}
}

static inline void k2serdes_phyb_rst_clr(void __iomem *serdes)
{
	FINSR(serdes, CML_REG(0), 5, 0, 0x1f);
}

static inline int k2serdes_phyb_in_rst(void __iomem *serdes)
{
	u32 val;

	/* read CML_00[4:0] */
	val = k2serdes_readl(serdes, CML_REG(0));
	return ((val & 0x1f) == 0x00);
}

static inline void k2serdes_pll_disable(void __iomem *serdes)
{
	FINSR(serdes, PLL_CTRL_REG, 31, 29, 0x4);
	FINSR(serdes, PLL_CTRL_REG, 27, 25, 0x4);
}

static inline void k2serdes_pll_enable_10p3125g(void __iomem *serdes)
{
	FINSR(serdes, PLL_CTRL_REG, 31, 29, 0x7);
	FINSR(serdes, PLL_CTRL_REG, 27, 25, 0x7);
}

static inline void k2serdes_pll_enable_1p25g(void __iomem *serdes)
{
	k2serdes_writel(serdes, PLL_CTRL_REG, 0xe0000000);
}

static inline void k2serdes_pll_enable(void __iomem *serdes,
					struct hw_specific *hw)
{
	k2serdes_phyb_rst_clr(serdes);

	if (hw->link_rate == K2SERDES_LINK_RATE_10P3125G)
		k2serdes_pll_enable_10p3125g(serdes);
	else if (hw->link_rate == K2SERDES_LINK_RATE_1P25G)
		k2serdes_pll_enable_1p25g(serdes);
}

static int k2serdes_init(void __iomem *serdes,
				struct hw_specific *hw)
{
	struct serdes_cfg *serdes_cfg;
	void __iomem *base;
	int a_size;

	if (hw->ref_clock_rate != K2SERDES_REF_CLOCK_156P25M)
		return -EINVAL;

	if ((hw->link_rate != K2SERDES_LINK_RATE_10P3125G) &&
		(hw->link_rate != K2SERDES_LINK_RATE_1P25G))
		return -EINVAL;

	k2serdes_pll_disable(serdes);

	/* cmu0 setup */
	a_size		= ARRAY_SIZE(cfg_phyb_1p25g_156p25mhz_cmu0);
	serdes_cfg	= &cfg_phyb_1p25g_156p25mhz_cmu0[0];
	base		= serdes + CMU0_SS_OFFSET;
	k2serdes_config_ss(base, serdes_cfg, a_size);

	/* cmu1 setup */
	a_size		= ARRAY_SIZE(cfg_phyb_10p3125g_156p25mhz_cmu1);
	serdes_cfg	= &cfg_phyb_10p3125g_156p25mhz_cmu1[0];
	base		= serdes + CMU1_SS_OFFSET;
	k2serdes_config_ss(base, serdes_cfg, a_size);

	return 0;
}

/* lane is 0 based */
static inline void k2serdes_lane_10p3125g_config(void __iomem *serdes,
						int lane)
{
	struct serdes_cfg *serdes_cfg;
	void __iomem *base;
	int a_size;

	/* lane setup */
	a_size		= ARRAY_SIZE(cfg_phyb_10p3125g_16bit_lane);
	serdes_cfg	= &cfg_phyb_10p3125g_16bit_lane[0];
	base		= serdes + LANEX_SS_OFFSET(lane);
	k2serdes_config_ss(base, serdes_cfg, a_size);

	/* disable auto negotiation*/
	FINSR(serdes, LANEX_REG(lane, 0x180), 4, 4, 0x0);

	/* disable link training */
	FINSR(serdes, LANEX_REG(lane, 0x1c0), 9, 9, 0x0);
}

static inline void k2serdes_lane_1p25g_config(void __iomem *serdes,
						int lane)
{
	struct serdes_cfg *serdes_cfg;
	void __iomem *base;
	int a_size;

	/* lane setup */
	a_size		= ARRAY_SIZE(cfg_phyb_aneg_lane);
	serdes_cfg	= &cfg_phyb_aneg_lane[0];
	base		= serdes + LANEX_SS_OFFSET(lane);
	k2serdes_config_ss(base, serdes_cfg, a_size);

	/* disable auto negotiation*/
	FINSR(serdes, LANEX_REG(lane, 0x180), 4, 4, 0x0);
}

static inline void k2serdes_lane_config(void __iomem *serdes,
				int lane, struct hw_specific *hw)
{
	if (hw->ref_clock_rate != K2SERDES_REF_CLOCK_156P25M)
		return;

	if (hw->link_rate == K2SERDES_LINK_RATE_10P3125G)
		k2serdes_lane_10p3125g_config(serdes, lane);
	else if (hw->link_rate == K2SERDES_LINK_RATE_1P25G)
		k2serdes_lane_1p25g_config(serdes, lane);
}


static inline void k2serdes_com_enable(void __iomem *serdes,
					struct hw_specific *hw)
{
	struct serdes_cfg *serdes_cfg;
	void __iomem *base;
	int a_size;

	if (hw->link_rate == K2SERDES_LINK_RATE_10P3125G) {
		a_size		= ARRAY_SIZE(cfg_phyb_10p3125g_cml);
		serdes_cfg	= &cfg_phyb_10p3125g_cml[0];
	} else if (hw->link_rate == K2SERDES_LINK_RATE_1P25G) {
		a_size		= ARRAY_SIZE(cfg_phyb_1p25g_156p25mhz_cml);
		serdes_cfg	= &cfg_phyb_1p25g_156p25mhz_cml[0];
	} else
		return;

	base = serdes + CML_SS_OFFSET;
	k2serdes_config_ss(base, serdes_cfg, a_size);
}


static inline void k2serdes_eq_vreg_enable(void __iomem *serdes,
					struct hw_specific *hw)
{
	u32 i;

	for (i = 0; i < hw->lanes; i++) {
		/* pma_ln_vreg */
		FINSR(serdes, LANEX_REG(i, 0x18), 25, 24, 0x2);
		/* pma_ln_vregh */
		FINSR(serdes, LANEX_REG(i, 0x18), 27, 26, 0x2);
	}
}

static inline void k2serdes_eq_cdfe_enable(void __iomem *serdes,
					struct hw_specific *hw)
{
	u32 i;

	if (hw->phy_type != K2SERDES_PHY_XGE)
		return;

	for (i = 0; i < hw->lanes; i++)
		FINSR(serdes, LANEX_REG(i, 0x94), 24, 24, 0x1);

	/* setting initial cdfe */
	FINSR(serdes, CML_REG(0x108), 23, 16, 0xff);

	/* turn on ei exit recal */
	FINSR(serdes, CML_REG(0x10c), 7, 0, 0xff);

	for (i = 0; i < hw->lanes; i++) {
		/* enable ei exit cal for cdfe */
		FINSR(serdes, LANEX_REG(i, 0x98), 2, 2, 0x0);
		/* enable cdfe_ln_force_cal for cdfe */
		FINSR(serdes, LANEX_REG(i, 0x98), 0, 0, 0x1);
	}

	/* setting rx tap */
	FINSR(serdes, CML_REG(0xbc), 28, 24, 0x0);
}

static void k2serdes_tx_rx_set_equalizer(void __iomem *serdes,
					struct hw_specific *hw)
{
	u32 i;

	/* set tx output swing voltage */
	if (hw->eq_vreg_enable)
		k2serdes_eq_vreg_enable(serdes, hw);

	/* enable cdfe */
	if (hw->eq_cdfe_enable)
		k2serdes_eq_cdfe_enable(serdes, hw);

	/* set att and boost start */
	for (i = 0; i < hw->lanes; i++) {
		/* att start -1 for short channel */
		FINSR(serdes, LANEX_REG(i, 0x8c), 11, 8, 0x4);
		/* boost start -3 for short channel */
		FINSR(serdes, LANEX_REG(i, 0x8c), 15, 12, 0xa);
	}
}

static inline void k2serdes_lane_enable(void __iomem *serdes,
				int lane, struct hw_specific *hw)
{
	/* Set Lane Control Rate */
	if (hw->link_rate == K2SERDES_LINK_RATE_10P3125G) {
		FINSR(serdes, LANE_CTRL_STS_REG(lane), 28, 26, 0x4);
		FINSR(serdes, LANE_CTRL_STS_REG(lane), 12, 10, 0x4);
	} else if (hw->link_rate == K2SERDES_LINK_RATE_1P25G)
		k2serdes_writel(serdes, LANE_CTRL_STS_REG(lane), 0xf800f8c0);

	/* set bus-width */
	FINSR(serdes, LANE_CTRL_STS_REG(lane), 23, 21, 0x7);
	FINSR(serdes, LANE_CTRL_STS_REG(lane),  5,  3, 0x7);

	/* enable PCS overlay and lane select 10GKR */
	FINSR(serdes, LANE_CTRL_STS_REG(lane), 16, 16, 0x1);
	FINSR(serdes, LANE_CTRL_STS_REG(lane), 19, 19, 0x1);

	/* enable lanes */
	FINSR(serdes, LANE_CTRL_STS_REG(lane), 31, 29, 0x7);
	FINSR(serdes, LANE_CTRL_STS_REG(lane), 15, 13, 0x7);
}

static inline int k2serdes_get_lane_status(void __iomem *serdes,
				struct hw_specific *hw, int lane)
{
	u32 val;

	if (hw->phy_type != K2SERDES_PHY_XGE)
		return 0;

	val = k2serdes_readl(serdes, CML_REG(0x1f8));
	return (val >> (29 + lane)) & 0x1;
}

static int k2serdes_get_status(void __iomem *serdes,
			      struct hw_specific *hw)
{
	u32 lanes_ok = 1;
	int i;

	for (i = 0; i < hw->lanes; i++) {
		lanes_ok &= k2serdes_get_lane_status(serdes, hw, i);
		cpu_relax();
	}

	return lanes_ok;
}

static int k2serdes_wait_pll_locked(void __iomem *serdes,
				      struct hw_specific *hw)
{
	unsigned long timeout;
	int ret = 0;
	u32 status;

	timeout = jiffies + msecs_to_jiffies(500);
	do {
		status = k2serdes_get_status(serdes, hw);

		if (status)
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

static void k2serdes_assert_reset(void __iomem *serdes, struct hw_specific *hw)
{
	u32 i;

	for (i = 0; i < MAX_LANES; i++)
		FINSR(serdes, LANEX_REG(i, 0x028), 29, 15, 0x4260);
}

static inline void k2serdes_deassert_reset_poll_xge(void __iomem *serdes,
						struct hw_specific *hw)
{
	u32 lanes_ok = MASK((hw->lanes - 1), 0);
	u32 val;

	do {
		val = k2serdes_readl(serdes, CML_REG(0x1f8));
		cpu_relax();
	} while (((val >> 29) & lanes_ok) != lanes_ok);
}

static inline void k2serdes_deassert_reset_poll_pcie(void __iomem *serdes,
						struct hw_specific *hw)
{
	u32 tmp, i;

	for (i = 0; i < 4; i++)
		do {
			tmp = k2serdes_read_select_tbus(serdes, i + 1, 0x02);
		} while ((tmp & BIT(4)) != 0);
}

static inline void k2serdes_deassert_reset_poll_others(void __iomem *serdes,
						struct hw_specific *hw)
{
	u32 lanes_ok = MASK((hw->lanes - 1), 0);
	u32 val;

	/* 4 lane PHY-A */
	do {
		val = k2serdes_readl(serdes, CML_REG(0x1f8));
		cpu_relax();
	} while (((val >> 28) & lanes_ok) != lanes_ok);
}

static int k2serdes_deassert_reset(void __iomem *serdes,
			struct hw_specific *hw, int poll)
{
	u32 i;

	for (i = 0; i < hw->lanes; i++) {
		if (hw->phy_type == K2SERDES_PHY_XGE)
			/* set pma_cmu_sel to 1 */
			FINSR(serdes, LANEX_REG(i, 0x60), 0, 0, 0x1);
		/* release reset */
		FINSR(serdes, LANEX_REG(i, 0x28), 29, 29, 0x0);
	}

	if (!poll)
		goto out;

	if (hw->phy_type == K2SERDES_PHY_XGE)
		k2serdes_deassert_reset_poll_xge(serdes, hw);
	else if (hw->phy_type == K2SERDES_PHY_PCIE)
		k2serdes_deassert_reset_poll_pcie(serdes, hw);
	else
		k2serdes_deassert_reset_poll_others(serdes, hw);

out:
	return 0;
}

/* lane is 0-based */
static void k2serdes_get_cmp_tap_offsets_xge(void __iomem *serdes,
	u32 lane, u32 cmp, struct k2serdes_comparator_tap_offsets *ofs)
{
	/* set comparator number */
	FINSR(serdes, CML_REG(0x8c), 23, 21, cmp);

	/* read offsets */
	FINSR(serdes, CMU0_REG(0xfc), 26, 16, ((lane + 2) << 8) + 0x11);
	ofs->cmp = (k2serdes_read_tbus_val(serdes) & 0x0ff0) >> 4;

	FINSR(serdes, CMU0_REG(0xfc), 26, 16, ((lane + 2) << 8) + 0x11);
	ofs->tap1 = (k2serdes_read_tbus_val(serdes) & 0x000f) << 3;

	FINSR(serdes, CMU0_REG(0xfc), 26, 16, ((lane + 2) << 8) + 0x12);
	ofs->tap1 |= (k2serdes_read_tbus_val(serdes) & 0x0e00) >> 9;
	ofs->tap2  = (k2serdes_read_tbus_val(serdes) & 0x01f8) >> 3;
	ofs->tap3  = (k2serdes_read_tbus_val(serdes) & 0x0007) << 3;

	FINSR(serdes, CMU0_REG(0xfc), 26, 16, ((lane + 2) << 8) + 0x13);
	ofs->tap3 |= (k2serdes_read_tbus_val(serdes) & 0x0e00) >> 9;
	ofs->tap4  = (k2serdes_read_tbus_val(serdes) & 0x01f8) >> 3;
	ofs->tap5  = (k2serdes_read_tbus_val(serdes) & 0x0007) << 3;

	FINSR(serdes, CMU0_REG(0xfc), 26, 16, ((lane + 2) << 8) + 0x14);
	ofs->tap5 |= (k2serdes_read_tbus_val(serdes) & 0x0e00) >> 9;
}

static void k2serdes_add_offsets_xge(void __iomem *serdes,
	struct hw_specific *hw, struct k2serdes_offsets *sofs)
{
	struct k2serdes_comparator_tap_offsets *ctofs;
	struct k2serdes_comparator_tap_offsets sample;
	struct k2serdes_lane_offsets *lofs;
	u32 lane, cmp;

	for (lane = 0; lane < hw->lanes; lane++) {
		lofs = &(sofs->lane_ofs[lane]);
		/* yes cmp starts from 1 */
		for (cmp = 1; cmp < MAX_COMPARATORS; cmp++) {
			ctofs = &(lofs->ct_ofs[cmp]);

			k2serdes_get_cmp_tap_offsets_xge(serdes,
						lane, cmp, &sample);

			ctofs->cmp  += sample.cmp;
			ctofs->tap1 += sample.tap1;
			ctofs->tap2 += sample.tap2;
			ctofs->tap3 += sample.tap3;
			ctofs->tap4 += sample.tap4;
			ctofs->tap5 += sample.tap5;
		}
	}
}

/* lane is 0-based */
static void k2serdes_get_cmp_tap_offsets_non_xge(void __iomem *serdes,
	u32 lane, u32 cmp, struct k2serdes_comparator_tap_offsets *ofs)
{
	/* set comparator number */
	FINSR(serdes, CML_REG(0x8c), 23, 21, cmp);

	/* read offsets */
	FINSR(serdes, CMU0_REG(0x8), 31, 24, ((lane + 1) << 5) + 0x12);
	ofs->cmp = (k2serdes_read_tbus_val(serdes) & 0x0ff0) >> 4;
}

static void k2serdes_add_offsets_non_xge(void __iomem *serdes,
	struct hw_specific *hw, struct k2serdes_offsets *sofs)
{
	struct k2serdes_comparator_tap_offsets *ctofs;
	struct k2serdes_comparator_tap_offsets sample;
	struct k2serdes_lane_offsets *lofs;
	u32 lane, cmp;

	for (lane = 0; lane < hw->lanes; lane++) {
		lofs = &(sofs->lane_ofs[lane]);
		/* yes cmp starts from 1 */
		for (cmp = 1; cmp < MAX_COMPARATORS; cmp++) {
			ctofs = &(lofs->ct_ofs[cmp]);

			k2serdes_get_cmp_tap_offsets_non_xge(serdes,
					lane, cmp, &sample);

			ctofs->cmp  += sample.cmp;
		}
	}
}

static void k2serdes_get_average_offsets(void __iomem *serdes, u32 samples,
		struct hw_specific *hw, struct k2serdes_offsets *sofs)
{
	struct k2serdes_comparator_tap_offsets *ctofs;
	struct k2serdes_lane_offsets *lofs;
	u32 i, lane, cmp;

	memset(sofs, 0, sizeof(*sofs));

	/* get the total of each offset for specified number of samples */
	for (i = 0; i < samples; i++) {
		k2serdes_assert_reset(serdes, hw);
		k2serdes_deassert_reset(serdes, hw, 1);

		if (hw->phy_type == K2SERDES_PHY_XGE)
			k2serdes_add_offsets_xge(serdes, hw, sofs);
		else
			k2serdes_add_offsets_non_xge(serdes, hw, sofs);
	}

	/* take the average */
	for (lane = 0; lane < hw->lanes; lane++) {
		lofs = &(sofs->lane_ofs[lane]);
		/* yes cmp starts from 1 */
		for (cmp = 1; cmp < MAX_COMPARATORS; cmp++) {
			ctofs = &(lofs->ct_ofs[cmp]);
			if (hw->phy_type == K2SERDES_PHY_XGE) {
				ctofs->cmp  /= samples;
				ctofs->tap1 /= samples;
				ctofs->tap2 /= samples;
				ctofs->tap3 /= samples;
				ctofs->tap4 /= samples;
				ctofs->tap5 /= samples;
			} else
				ctofs->cmp  /= samples;
		}
	}
}

static void k2serdes_override_cmp_tap_offsets(void __iomem *serdes,
	u32 lane, u32 cmp, struct k2serdes_comparator_tap_offsets *ofs)
{
	/* set dfe_shadow_lane_sel */
	FINSR(serdes, CML_REG(0xf0), 27, 26, (lane + 1));

	/* set cmp_offset_ovr_en to 1 */
	FINSR(serdes, CML_REG(0x98), 24, 24, 0x1);

	/* set rxeq_ovr_en to 0x1 */
	FINSR(serdes, LANEX_REG(lane, 0x2c), 2, 2, 0x1);

	/* set rxeq_dfe_cmp_sel_ovr to comp_no */
	FINSR(serdes, LANEX_REG(lane, 0x30), 7, 5, cmp);

	/* set dfe_tap_ovr_en to 1 */
	FINSR(serdes, LANEX_REG(lane, 0x5c), 31, 31, 0x1);

	/* set cmp offset override */
	FINSR(serdes, CML_REG(0x9c), 7, 0, ofs->cmp);
	/* set tap offset overrides */
	FINSR(serdes, LANEX_REG(lane, 0x58), 30, 24, ofs->tap1);
	FINSR(serdes, LANEX_REG(lane, 0x5c),  5,  0, ofs->tap2);
	FINSR(serdes, LANEX_REG(lane, 0x5c), 13,  8, ofs->tap3);
	FINSR(serdes, LANEX_REG(lane, 0x5c), 21, 16, ofs->tap4);
	FINSR(serdes, LANEX_REG(lane, 0x5c), 29, 24, ofs->tap5);

	/* set rxeq_ovr_latch_o = 0x1 */
	FINSR(serdes, LANEX_REG(lane, 0x2c), 10, 10, 0x1);
	/* set rxeq_ovr_latch_o = 0x0 */
	FINSR(serdes, LANEX_REG(lane, 0x2c), 10, 10, 0x0);

	/* set cmp_offset_ovr_en to 0 */
	FINSR(serdes, CML_REG(0x98), 24, 24, 0x0);
	/* set rxeq_ovr_en to 0x0 */
	FINSR(serdes, LANEX_REG(lane, 0x2c), 2, 2, 0x0);
	/* set dfe_tap_ovr_en to 0 */
	FINSR(serdes, LANEX_REG(lane, 0x5c), 31, 31, 0x0);
}

static inline void k2serdes_override_cmp_offset_cdfe(void __iomem *serdes,
					u32 lane, u32 cmp, u32 cmp_offset)
{
	/* enable comparator offset calibrate */
	FINSR(serdes, LANEX_REG(lane, 0x58), 18, 18, 0x1);

	/* set gcfsm sel override to comparator */
	FINSR(serdes, LANEX_REG(lane, 0x4c), 5, 2, (0x1 << (cmp - 1)));
	/* set comparator offset */
	FINSR(serdes, LANEX_REG(lane, 0x48), 24, 17, cmp_offset);
	/* latch in value */
	FINSR(serdes, LANEX_REG(lane, 0x48), 29, 29, 0x1);
	FINSR(serdes, LANEX_REG(lane, 0x48), 29, 29, 0x0);

	/* disable comparator offset calibrate */
	FINSR(serdes, LANEX_REG(lane, 0x58), 18, 18, 0x0);
}

static inline void k2serdes_override_tap_offset_cdfe(void __iomem *serdes,
	u32 lane, u32 tap, u32 width, u32 tap_offset)
{
	/* enable tap */
	FINSR(serdes, LANEX_REG(lane, 0x58), 23, 19, BIT(tap - 1));
	/* set tap offset */
	FINSR(serdes, LANEX_REG(lane, 0x48), 17 + (width - 1), 17, tap_offset);
	/* latch in value */
	FINSR(serdes, LANEX_REG(lane, 0x48), 29, 29, 0x1);
	FINSR(serdes, LANEX_REG(lane, 0x48), 29, 29, 0x0);
}

static void k2serdes_override_cmp_tap_offsets_cdfe(void __iomem *serdes,
	u32 lane, u32 cmp, struct k2serdes_comparator_tap_offsets *ofs)
{
	/* enable overrides */
	FINSR(serdes, LANEX_REG(lane, 0x58), 16, 16, 0x1);
	FINSR(serdes, LANEX_REG(lane, 0x48), 16, 16, 0x1);

	k2serdes_override_cmp_offset_cdfe(serdes, lane, cmp, ofs->cmp);

	/* enable tap offset calibrate */
	FINSR(serdes, LANEX_REG(lane, 0x58), 17, 17, 0x1);

	/* set tap offsets */
	k2serdes_override_tap_offset_cdfe(serdes, lane, 1, 7, ofs->tap1);
	k2serdes_override_tap_offset_cdfe(serdes, lane, 2, 6, ofs->tap2);
	k2serdes_override_tap_offset_cdfe(serdes, lane, 3, 6, ofs->tap3);
	k2serdes_override_tap_offset_cdfe(serdes, lane, 4, 6, ofs->tap4);
	k2serdes_override_tap_offset_cdfe(serdes, lane, 5, 6, ofs->tap5);

	/* disable overrides */
	FINSR(serdes, LANEX_REG(lane, 0x58), 16, 16, 0x0);
	FINSR(serdes, LANEX_REG(lane, 0x48), 16, 16, 0x0);
	FINSR(serdes, LANEX_REG(lane, 0x58), 18, 18, 0x0);
	FINSR(serdes, LANEX_REG(lane, 0x58), 17, 17, 0x0);
}

static void k2serdes_set_offsets_xge(void __iomem *serdes,
	struct hw_specific *hw, struct k2serdes_offsets *sofs)
{
	struct k2serdes_comparator_tap_offsets *ctofs;
	struct k2serdes_lane_offsets *lofs;
	u32 lane, cmp;

	for (lane = 0; lane < hw->lanes; lane++) {
		lofs = &(sofs->lane_ofs[lane]);
		for (cmp = 1; cmp < MAX_COMPARATORS; cmp++) {
			ctofs = &(lofs->ct_ofs[cmp]);
			k2serdes_override_cmp_tap_offsets(serdes,
						lane, cmp, ctofs);
			k2serdes_override_cmp_tap_offsets_cdfe(serdes,
						lane, cmp, ctofs);
		}
	}
}

static void k2serdes_set_offsets_non_xge(void __iomem *serdes,
	struct hw_specific *hw, struct k2serdes_offsets *sofs)
{
	struct k2serdes_comparator_tap_offsets *ctofs;
	struct k2serdes_lane_offsets *lofs;
	u32 lane, cmp;

	for (lane = 0; lane < hw->lanes; lane++) {
		lofs = &(sofs->lane_ofs[lane]);
		for (cmp = 1; cmp < MAX_COMPARATORS; cmp++) {
			ctofs = &(lofs->ct_ofs[cmp]);
			k2serdes_override_cmp_tap_offsets(serdes,
						lane, cmp, ctofs);
		}
	}
}

static void k2serdes_set_offsets(void __iomem *serdes,
	struct hw_specific *hw, struct k2serdes_offsets *sofs)
{
	if (hw->phy_type == K2SERDES_PHY_XGE)
		k2serdes_set_offsets_xge(serdes, hw, sofs);
	else
		k2serdes_set_offsets_non_xge(serdes, hw, sofs);
}

static inline void k2serdes_config_c1_c2_cm(void __iomem *serdes,
				int lane, struct hw_specific *hw)
{
	u32 c1, c2, cm;

	c1 = hw->c1;
	c2 = hw->c2;
	cm = hw->cm;

	if (hw->phy_type == K2SERDES_PHY_XGE) {
		/* TX Control override enable */
		FINSR(serdes, LANEX_REG(lane, 0x8), 4, 0, (c1 & 0x1f));

		/* TX Control override enable */
		FINSR(serdes, LANEX_REG(lane, 0x4), 18, 18, ((c2 >> 3) & 0x1));

		/* TX Control override enable */
		FINSR(serdes, LANEX_REG(lane, 0x8), 7, 5, (c2 & 0x7));

		/* TX Control override enable */
		FINSR(serdes, LANEX_REG(lane, 0x8), 11, 8, (cm & 0xf));
	} else {
		FINSR(serdes, LANEX_REG(lane, 0x8), 4, 0, (c1 & 0x1f));
		FINSR(serdes, LANEX_REG(lane, 0x8), 11, 8, (c2 & 0xf));
		FINSR(serdes, LANEX_REG(lane, 0x8), 15, 12, (cm & 0xf));
	}
}

static void k2serdes_dfe_offset_calibrate(void __iomem *serdes,
					struct hw_specific *hw)
{
	struct k2serdes_offsets sofs;
	u32 i;

	/* force serdes signal detect low (reset CDR, Att and Boost) */
	for (i = 0; i < hw->lanes; i++)
		FINSR(serdes, LANEX_REG(i, 0x004), 2, 1, 0x2);

	udelay(10);

	k2serdes_get_average_offsets(serdes, DFE_OFFSET_SAMPLES, hw, &sofs);

	/* assert reset to apply tx FIR coeff */
	k2serdes_assert_reset(serdes, hw);

	/* set tx swing */
	for (i = 0; i < hw->lanes; i++)
		if (hw->phy_type == K2SERDES_PHY_XGE)
			FINSR(serdes, LANEX_REG(i, 0x4),
				29, 26, (hw->tx_att & 0xf));
		else
			FINSR(serdes, LANEX_REG(i, 0x4),
				28, 25, (hw->tx_att & 0xf));

	/* set regulator setting for tx driver */
	for (i = 0; i < hw->lanes; i++)
		FINSR(serdes, LANEX_REG(i, 0x84), 7, 5, (hw->tx_vreg & 0x7));

	/* apply the tx FIR coeff to the lanes */
	for (i = 0; i < hw->lanes; i++)
		k2serdes_config_c1_c2_cm(serdes, i, hw);

	k2serdes_deassert_reset(serdes, hw, 1);

	/* offset compensation patch */
	k2serdes_set_offsets(serdes, hw, &sofs);

	udelay(10);

	/* re-acquire signal detect */
	for (i = 0; i < hw->lanes; i++)
		FINSR(serdes, LANEX_REG(i, 0x4), 2, 1, 0x0);

	udelay(10);
}

static inline void k2serdes_enable_xgmii_port(void __iomem *sw_regs)
{
	k2serdes_writel(sw_regs, XGE_CTRL_OFFSET, 0x03);
}

void k2serdes_reset_cdr(void __iomem *serdes, int lane)
{
	/* toggle signal detect */
	FINSR(serdes, LANEX_REG(lane, 0x04), 2, 1, 0x2);
	mdelay(1);
	FINSR(serdes, LANEX_REG(lane, 0x04), 2, 1, 0x0);
}

/* Call every 10 ms */
int k2serdes_check_link_status(void __iomem *serdes,
			      void __iomem *sw_regs,
			      u32 lanes,
			      u32 *current_state,
			      u32 *lane_down)
{
	u32 pcsr_rx_stat, blk_lock, blk_errs;
	int loss, i, status = 1;

	for (i = 0; i < lanes; i++) {
		/* Rx Signal Loss bit in serdes lane control and status reg*/
		loss = (k2serdes_readl(serdes, LANE_CTRL_STS_REG(i))) & 0x01;

		/* Block Errors and Block Lock bits in PCSR rx status reg */
		pcsr_rx_stat = k2serdes_readl(sw_regs, PCSR_RX_STATUS(i));
		blk_lock = (pcsr_rx_stat >> 30) & 0x1;
		blk_errs = (pcsr_rx_stat >> 16) & 0x0ff;
/*pr_info("++++++ lane %d pcsr_rx_stat=0x%08x\n", i, pcsr_rx_stat);*/

		/* If Block error, attempt recovery! */
		if (blk_errs)
			blk_lock = 0;

		switch (current_state[i]) {
		case 0:
			/* if good link lock the signal detect ON! */
			if (!loss && blk_lock) {
				pr_debug("XGE PCSR Linked Lane: %d\n", i);
				FINSR(serdes, LANEX_REG(i, 0x04), 2, 1, 0x3);
				current_state[i] = 1;
			} else {
				/* if no lock, then reset CDR */
				if (!blk_lock) {
					pr_debug("XGE PCSR Recover Lane: %d\n",
						i);

					k2serdes_reset_cdr(serdes, i);
				}
			}
			break;
		case 1:
			if (!blk_lock) {
				/* Link Lost? */
				lane_down[i] = 1;
				current_state[i] = 2;
			}
			break;
		case 2:
			if (blk_lock)
				/* Nope just noise */
				current_state[i] = 1;
			else {
				/* Lost the block lock, reset CDR if it is
				   not centered and go back to sync state
				 */
				k2serdes_reset_cdr(serdes, i);

				current_state[i] = 0;
			}
			break;
		default:
			pr_info("XGE: unknown current_state[%d] %d\n",
				i, current_state[i]);
			break;
		}

		if (blk_errs) {
			/* Reset the Error counts! */
			FINSR(sw_regs, PCSR_RX_CTL(i), 7, 0, 0x19);
			FINSR(sw_regs, PCSR_RX_CTL(i), 7, 0, 0x00);
		}

		status &= (current_state[i] == 1);
	}

	return status;
}

static int k2serdes_check_lane(void __iomem *serdes,
		  void __iomem *sw_regs, u32 lanes)
{
	u32 current_state[2] = {0, 0};
	int retries = 0, link_up;
	u32 lane_down[2];

	do {
		mdelay(10);
		lane_down[0] = 0;
		lane_down[1] = 0;

		link_up = k2serdes_check_link_status(serdes, sw_regs,
					   lanes, current_state, lane_down);

		/* if we did not get link up then wait 100ms
		   before calling it again
		 */
		if (link_up)
			break;

		if (lane_down[0])
			pr_debug("XGE: detected link down on lane 0\n");

		if (lanes > 1 && lane_down[1])
			pr_debug("XGE: detected link down on lane 1\n");

		if (++retries > 100) {
			pr_err("XGE: timeout waiting for serdes link up\n");
			return -ETIMEDOUT;
		}
	} while (!link_up);

	pr_info("XGE: serdes link up: retried %d times\n", retries);
	return 0;
}

static inline void k2serdes_reset(void __iomem *serdes)
{
	/* Toggle POR_EN bit */
	FINSR(serdes, CPU_CTRL_REG, 29, 29, 0x1);
	udelay(10);
	FINSR(serdes, CPU_CTRL_REG, 29, 29, 0x0);
	udelay(10);
}

static inline void k2serdes_txb_clk_mode(void __iomem *serdes,
					      struct hw_specific *hw)
{
	if (hw->link_rate != K2SERDES_LINK_RATE_1P25G)
		return;

	k2serdes_writel(serdes, CPU_CTRL_REG, 0x20000000);
	k2serdes_writel(serdes, PLL_CTRL_REG, 0x00380000);
	k2serdes_writel(serdes, CPU_CTRL_REG, 0x00000000);
}

static int k2serdes_config(void __iomem *serdes,
			      void __iomem *sw_regs,
			      struct hw_specific *hw)
{
	u32 ret, i, lanes = hw->lanes;

	k2serdes_txb_clk_mode(serdes, hw);

	ret = k2serdes_init(serdes, hw);
	if (ret)
		return ret;

	for (i = 0; i < lanes; i++)
		k2serdes_lane_config(serdes, i, hw);

	k2serdes_com_enable(serdes, hw);

	k2serdes_tx_rx_set_equalizer(serdes, hw);

	k2serdes_pll_enable(serdes, hw);

	for (i = 0; i < lanes; i++)
		k2serdes_lane_enable(serdes, i, hw);

	/* SB PLL Status Poll */
	ret = k2serdes_wait_pll_locked(serdes, hw);
	if (ret)
		return ret;

	k2serdes_dfe_offset_calibrate(serdes, hw);

	k2serdes_enable_xgmii_port(sw_regs);

	k2serdes_check_lane(serdes, sw_regs, lanes);

	return ret;
}

static int k2serdes_start(struct hw_specific *hw)
{
	void __iomem *serdes, *xge_sw_regs;
	int ret;

	serdes = ioremap(XGE_SERDES_BASE, XGE_SERDES_SIZE);
	xge_sw_regs = ioremap(XGE_SW_BASE, XGE_SW_SIZE);

	pr_info("XGE: serdes reset\n");
	k2serdes_reset(serdes);

	ret = k2serdes_config(serdes, xge_sw_regs, hw);

	iounmap(serdes);
	iounmap(xge_sw_regs);
	return ret;
}

static int k2_xge_serdes_configured;  /* FIXME */

int xge_serdes_init(struct device_node *node)
{
	int ret;
	struct hw_specific *hw = NULL;

	hw = kzalloc(sizeof(*hw), GFP_KERNEL);

	if (!hw) {
		pr_err("xge serdes mem alloc failed\n");
		return -ENOMEM;
	}

	if (of_property_read_u32(node, "ref_clock",
					&hw->ref_clock_rate))
		hw->ref_clock_rate = K2SERDES_REF_CLOCK_156P25M;

	if (of_property_read_u32(node, "link_rate",
					&hw->link_rate))
		hw->link_rate = K2SERDES_LINK_RATE_10P3125G;

	if (hw->link_rate != K2SERDES_LINK_RATE_10P3125G)
		hw->link_rate = K2SERDES_LINK_RATE_1P25G;

	if (of_property_read_u32_array(node, "tx_ctrl_override",
					&hw->c1, 5)) {
		hw->c1		= 2;
		hw->c2		= 0;
		hw->cm		= 2;
		hw->tx_att	= 12;
		hw->tx_vreg	= 4;
	}

	if (of_property_read_u32_array(node, "equalizer_flags",
					&hw->eq_vreg_enable, 3)) {
		hw->eq_vreg_enable	= 1;
		hw->eq_cdfe_enable	= 1;
		hw->eq_offset_enable	= 1;
	}

	hw->lanes = 2;
	hw->phy_type = K2SERDES_PHY_XGE;

	pr_info("XGE serdes config %d:\n",
		k2_xge_serdes_configured + 1);
	pr_info("  ref_clk=%s, link_rate=%s, lanes=%u\n",
		(hw->ref_clock_rate == K2SERDES_REF_CLOCK_156P25M) ?
			"156.25MHz" : "not 156.25MHz",
		(hw->link_rate == K2SERDES_LINK_RATE_10P3125G) ?
			"10.3125G" : "1.25G",
		hw->lanes);
	pr_info("  c1=%u, c2=%u, cm=%u, tx_att=%u, tx_vreg=%u\n",
		hw->c1, hw->c2, hw->cm, hw->tx_att, hw->tx_vreg);
	pr_info("  eq flags: vreg=%u, cdfe=%u, offset=%u\n",
		hw->eq_vreg_enable, hw->eq_cdfe_enable, hw->eq_offset_enable);

	ret = k2serdes_start(hw);
	if (!ret)
		++k2_xge_serdes_configured;

	kfree(hw);
	return ret;
}

int keystone_pcsr_config(void __iomem *pcsr_ofs, int port, u32 interface)
{
	return 0;
}
