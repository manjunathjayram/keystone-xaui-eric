/*
 * Texas Instruments Keystone SerDes driver
 * Authors: Hao Zhang <hzhang@ti.com>
 *	    WingMan Kwok <w-kwok2@ti.com>
 *	    Michael Scherban <m-scherban@ti.com>
 *
 * Copyright (C) 2015 Texas Instruments
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
#include <linux/device.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/random.h>

#include "keystone_serdes.h"
#include "keystone_xge_fw.h"

#define ks_debug	pr_debug
#define ks_info		pr_info
#define ks_err		pr_err
#define ks_dump		pr_info

/*
 * Keystone2 SERDES registers
 */
/* 0x1fc0 - 0x1fff */
#define KSERDES_SS_OFFSET	0x1fc0
/* 0x1fc0 */
#define MOD_VER_REG		(KSERDES_SS_OFFSET + 0x00)
/* 0x1fc4 */
#define MEM_ADR_REG		(KSERDES_SS_OFFSET + 0x04)
/* 0x1fc8 */
#define MEM_DAT_REG		(KSERDES_SS_OFFSET + 0x08)
/* 0x1fcc */
#define MEM_DATINC_REG		(KSERDES_SS_OFFSET + 0x0c)
/* 0x1fd0 */
#define CPU_CTRL_REG		(KSERDES_SS_OFFSET + 0x10)
/* 0x1fe0, 0x1fe4 */
#define LANE_CTRL_STS_REG(x)	(KSERDES_SS_OFFSET + 0x20 + (x * 0x04))
/* 0x1ff0 */
#define LINK_LOSS_WAIT_REG	(KSERDES_SS_OFFSET + 0x30)
/* 0x1ff4 */
#define PLL_CTRL_REG		(KSERDES_SS_OFFSET + 0x34)

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
 * XGE PCS-R registers
 */
#define PCSR_OFFSET(x)		(0x600 + (x * 0x80))

#define PCSR_TX_CTL(x)		(PCSR_OFFSET(x) + 0x00)
#define PCSR_TX_STATUS(x)	(PCSR_OFFSET(x) + 0x04)
#define PCSR_RX_CTL(x)		(PCSR_OFFSET(x) + 0x08)
#define PCSR_RX_STATUS(x)	(PCSR_OFFSET(x) + 0x0C)

#define XGE_CTRL_OFFSET		0x0c

#define reg_rmw(addr, value, mask) \
	__raw_writel(((__raw_readl(addr) & (~(mask))) | \
			(value & (mask))), (addr))

/* bit mask from bit-a to bit-b inclusive */
#define MASK(msb, lsb) \
	((((msb) - (lsb)) == 31) ? 0xffffffff :  \
		((((u32)1 << ((msb) - (lsb) + 1)) - 1) << (lsb)))

/* Replaces bit field [msb:lsb] in register located
 * at (base + offset) by val
 */
#define FINSR(base, offset, msb, lsb, val) \
	reg_rmw((base) + (offset), ((val) << (lsb)), MASK((msb), (lsb)))

/* This version of FEXTR is NOT safe for msb = 31, lsb = 0
 * but then why would we need FEXTR for that case.
 */
#define FEXTR(val, msb, lsb) \
	(((val) >> (lsb)) & ((1 << ((msb) - (lsb) + 1)) - 1))

#define PHY_A(serdes) \
	(0x4eba != ((kserdes_readl(serdes, MOD_VER_REG) >> 16) & 0xffff))

#define MOD_VER(serdes) \
	((kserdes_readl(serdes, MOD_VER_REG) >> 16) & 0xffff)

#define FOUR_LANE(serdes) \
	((0x4eb9 == MOD_VER(serdes)) || (0x4ebd == MOD_VER(serdes)))

#define LANE_ENABLE(sc, n) ((sc)->lane[n].enable)

#define for_each_enable_lane(func, sc)			\
	do {						\
		int i;					\
		for (i = 0; i < (sc)->lanes; i++) {	\
			if (!LANE_ENABLE((sc), i))	\
				continue;		\
							\
			(func)((sc), i);		\
		}					\
	} while (0)

#define for_each_lane(func, sc)				\
	do {						\
		int i;					\
		for (i = 0; i < (sc)->lanes; i++) {	\
			(func)((sc), i);		\
		}					\
	} while (0)

#define for_each_enable_lane_return(func, sc, r)	\
	do {						\
		int i;					\
		(r) = 0;				\
		for (i = 0; i < (sc)->lanes; i++) {	\
			if (!LANE_ENABLE((sc), i))	\
				continue;		\
							\
			(r) = (func)((sc), i);		\
			if ((r))			\
				break;			\
		}					\
	} while (0)

#define for_each_lane_return(func, sc, r)		\
	do {						\
		int i;					\
		(r) = 0;				\
		for (i = 0; i < (sc)->lanes; i++) {	\
			(r) = (func)((sc), i);		\
			if ((r))			\
				break;			\
		}					\
	} while (0)

#define MAX_COMPARATORS			5
#define DFE_OFFSET_SAMPLES		100

/* CPU CTRL bits */
#define CPU_EN			BIT(31)
#define CPU_GO			BIT(30)
#define POR_EN			BIT(29)
#define CPUREG_EN		BIT(28)
#define AUTONEG_CTL		BIT(27)
#define DATASPLIT		BIT(26)
#define LNKTRN_SIG_DET		BIT(8)

#define ANEG_LINK_CTL_10GKR_MASK	MASK(21, 20)
#define ANEG_LINK_CTL_1GKX_MASK		MASK(17, 16)
#define ANEG_LINK_CTL_1G10G_MASK \
	(ANEG_LINK_CTL_10GKR_MASK | ANEG_LINK_CTL_1GKX_MASK)

#define ANEG_1G_10G_OPT_MASK	MASK(7, 5)

struct kserdes_comparator_tap_offsets {
	u32 cmp;
	u32 tap1;
	u32 tap2;
	u32 tap3;
	u32 tap4;
	u32 tap5;
};

struct kserdes_lane_offsets {
	struct kserdes_comparator_tap_offsets ct_ofs[MAX_COMPARATORS];
};

struct kserdes_offsets {
	struct kserdes_lane_offsets lane_ofs[KSERDES_MAX_LANES];
};

struct serdes_cfg {
	u32 ofs;
	u32 msb;
	u32 lsb;
	u32 val;
};

/* offset w.r.t. serdes SS base */
static struct serdes_cfg cfg_156p25mhz_10bit_5gbps[] = {
	{0x0000,	23, 16,		0x80},
	{0x0000,	31, 24,		0x00},
	{0x0014,	 7,  0,		0x82},
	{0x0014,	15,  8,		0x82},
	{0x0060,	 7,  0,		0x38},
	{0x0060,	15,  8,		0x24},
	{0x0060,	23, 16,		0x14},
	{0x0064,	15,  8,		0xc7},
	{0x0064,	23, 16,		0xc3},
	{0x0078,	15,  8,		0xc0},
	{0x0204,	 7,  0,		0x80},
	{0x0204,	31, 24,		0x78},
	{0x0208,	 7,  0,		0x20},
	{0x0208,	23, 16,		0x01},
	{0x020c,	31, 24,		0x02},
	{0x0210,	31, 24,		0x1b},
	{0x0214,	 7,  0,		0xb8},
	{0x0214,	15,  8,		0x6f},
	{0x0218,	 7,  0,		0xe4},
	{0x0218,	23, 16,		0x80},
	{0x0218,	31, 24,		0x75},
	{0x02ac,	15,  8,		0x44},
	{0x022c,	15,  8,		0x08},
	{0x022c,	23, 16,		0x20},
	{0x0280,	 7,  0,		0x82},
	{0x0280,	23, 16,		0x82},
	{0x0284,	 7,  0,		0x85},
	{0x0284,	15,  8,		0x03},
	{0x0284,	23, 16,		0x0f},
	{0x0284,	31, 24,		0x1d},
	{0x0404,	 7,  0,		0x80},
	{0x0404,	31, 24,		0x78},
	{0x0408,	 7,  0,		0x20},
	{0x0408,	23, 16,		0x01},
	{0x040c,	31, 24,		0x02},
	{0x0410,	31, 24,		0x1b},
	{0x0414,	 7,  0,		0xb8},
	{0x0414,	15,  8,		0x6f},
	{0x0418,	 7,  0,		0xe4},
	{0x0418,	23, 16,		0x80},
	{0x0418,	31, 24,		0x75},
	{0x04ac,	15,  8,		0x44},
	{0x042c,	15,  8,		0x08},
	{0x042c,	23, 16,		0x20},
	{0x0480,	 7,  0,		0x82},
	{0x0480,	23, 16,		0x82},
	{0x0484,	 7,  0,		0x85},
	{0x0484,	15,  8,		0x03},
	{0x0484,	23, 16,		0x0f},
	{0x0484,	31, 24,		0x1d},
	{0x0604,	 7,  0,		0x80},
	{0x0604,	31, 24,		0x78},
	{0x0608,	 7,  0,		0x20},
	{0x0608,	23, 16,		0x01},
	{0x060c,	31, 24,		0x02},
	{0x0610,	31, 24,		0x1b},
	{0x0614,	 7,  0,		0xb8},
	{0x0614,	15,  8,		0x6f},
	{0x0618,	 7,  0,		0xe4},
	{0x0618,	23, 16,		0x80},
	{0x0618,	31, 24,		0x75},
	{0x06ac,	15,  8,		0x44},
	{0x062c,	15,  8,		0x08},
	{0x062c,	23, 16,		0x20},
	{0x0680,	 7,  0,		0x82},
	{0x0680,	23, 16,		0x82},
	{0x0684,	 7,  0,		0x85},
	{0x0684,	15,  8,		0x03},
	{0x0684,	23, 16,		0x0f},
	{0x0684,	31, 24,		0x1d},
	{0x0804,	 7,  0,		0x80},
	{0x0804,	31, 24,		0x78},
	{0x0808,	 7,  0,		0x20},
	{0x0808,	23, 16,		0x01},
	{0x080c,	31, 24,		0x02},
	{0x0810,	31, 24,		0x1b},
	{0x0814,	 7,  0,		0xb8},
	{0x0814,	15,  8,		0x6f},
	{0x0818,	 7,  0,		0xe4},
	{0x0818,	23, 16,		0x80},
	{0x0818,	31, 24,		0x75},
	{0x08ac,	15,  8,		0x44},
	{0x082c,	15,  8,		0x08},
	{0x082c,	23, 16,		0x20},
	{0x0880,	 7,  0,		0x82},
	{0x0880,	23, 16,		0x82},
	{0x0884,	 7,  0,		0x85},
	{0x0884,	15,  8,		0x03},
	{0x0884,	23, 16,		0x0f},
	{0x0884,	31, 24,		0x1d},
	{0x0a00,	15,  8,		0x08},
	{0x0a08,	23, 16,		0xa2},
	{0x0a08,	31, 24,		0x38},
	{0x0a30,	15,  8,		0x8a},
	{0x0a30,	23, 16,		0x8a},
	{0x0a84,	15,  8,		0x07},
	{0x0a94,	31, 24,		0x10},
	{0x0aa0,	31, 24,		0x81},
	{0x0abc,	31, 24,		0xff},
	{0x0ac0,	 7,  0,		0x8b},
	{0x0b08,	23, 16,		0x3f},
	{0x0b08,	31, 24,		0x58},
	{0x0b0c,	 7,  0,		0x4e},
	{0x0a48,	15,  8,		0x8c},
	{0x0a48,	23, 16,		0xfd},
	{0x0a54,	 7,  0,		0x72},
	{0x0a54,	15,  8,		0xec},
	{0x0a54,	23, 16,		0x2f},
	{0x0a58,	15,  8,		0x21},
	{0x0a58,	23, 16,		0xf9},
	{0x0a58,	31, 24,		0x00},
	{0x0a5c,	 7,  0,		0x60},
	{0x0a5c,	15,  8,		0x00},
	{0x0a5c,	23, 16,		0x04},
	{0x0a5c,	31, 24,		0x00},
	{0x0a60,	 7,  0,		0x00},
	{0x0a60,	15,  8,		0x80},
	{0x0a60,	23, 16,		0x00},
	{0x0a60,	31, 24,		0x00},
	{0x0a64,	 7,  0,		0x20},
	{0x0a64,	15,  8,		0x12},
	{0x0a64,	23, 16,		0x58},
	{0x0a64,	31, 24,		0x0c},
	{0x0a68,	 7,  0,		0x02},
	{0x0a68,	15,  8,		0x06},
	{0x0a68,	23, 16,		0x3b},
	{0x0a68,	31, 24,		0xe1},
	{0x0a6c,	 7,  0,		0xc1},
	{0x0a6c,	15,  8,		0x4c},
	{0x0a6c,	23, 16,		0x07},
	{0x0a6c,	31, 24,		0xb8},
	{0x0a70,	 7,  0,		0x89},
	{0x0a70,	15,  8,		0xe9},
	{0x0a70,	23, 16,		0x02},
	{0x0a70,	31, 24,		0x3f},
	{0x0a74,	 7,  0,		0x01},
	{0x0b20,	23, 16,		0x37},
	{0x0b1c,	31, 24,		0x37},
	{0x0b20,	 7,  0,		0x5d},
	{0x0000,	 7,  0,		0x03},
	{0x0a00,	 7,  0,		0x5f},
};

/* offset w.r.t. serdes SS base */
struct serdes_cfg cfg_phyb_1p25g_156p25mhz_cmu0[] = {
	{0x0000,	 7,  0,		0x02},
	{0x0000,	23, 16,		0x80},
	{0x0014,	 7,  0,		0x38},
	{0x0014,	15,  8,		0x38},
	{0x0060,	 7,  0,		0x38},
	{0x0060,	15,  8,		0xe4},
	{0x0060,	23, 16,		0x44},
	{0x0060,	31, 24,		0x1c},
	{0x0064,	 7,  0,		0x00},
	{0x0064,	15,  8,		0x84},
	{0x0064,	23, 16,		0xc1},
	{0x0068,	15,  8,		0x82},
	{0x0068,	23, 16,		0x07},
	{0x0068,	31, 24,		0x17},
	{0x006c,	 7,  0,		0x14},
	{0x0078,	15,  8,		0xc0},
	{0x0000,	 7,  0,		0x03},
};

/* offset w.r.t. serdes SS base */
struct serdes_cfg cfg_phyb_10p3125g_156p25mhz_cmu1[] = {
	{0x0c00,	 7,  0,		0x02},
	{0x0c00,	23, 16,		0x03},
	{0x0c14,	 7,  0,		0x52},
	{0x0c14,	15,  8,		0x52},
	{0x0c28,	31, 24,		0x80},
	{0x0c2c,	 7,  0,		0xf6},
	{0x0c3c,	 7,  0,		0x05},
	{0x0c3c,	15,  8,		0x04},
	{0x0c3c,	31, 24,		0x04},
	{0x0c40,	23, 16,		0x80},
	{0x0c40,	31, 24,		0xc0},
	{0x0c44,	 7,  0,		0x62},
	{0x0c44,	15,  8,		0x20},
	{0x0c44,	23, 16,		0x20},
	{0x0c44,	31, 24,		0x5a},
	{0x0c48,	 7,  0,		0x24},
	{0x0c48,	15,  8,		0x04},
	{0x0c48,	23, 16,		0x04},
	{0x0c48,	31, 24,		0x40},
	{0x0c4c,	 7,  0,		0x02},
	{0x0c4c,	15,  8,		0x40},
	{0x0c50,	15,  8,		0x1c},
	{0x0c50,	31, 24,		0x19},
	{0x0c54,	15,  8,		0x21},
	{0x0c58,	 7,  0,		0x60},
	{0x0c60,	 7,  0,		0x7c},
	{0x0c60,	15,  8,		0x1e},
	{0x0c60,	23, 16,		0x13},
	{0x0c60,	31, 24,		0x80},
	{0x0c64,	 7,  0,		0x02},
	{0x0c64,	15,  8,		0xcb},
	{0x0c64,	31, 24,		0x84},
	{0x0c68,	15,  8,		0x82},
	{0x0c68,	23, 16,		0x07},
	{0x0c68,	31, 24,		0x17},
	{0x0c6c,	 7,  0,		0x16},
	{0x0c74,	15,  8,		0x04},
	{0x0c78,	15,  8,		0xc0},
	{0x0c00,	 7,  0,		0x03},
};

/* offset w.r.t. LaneX SS 0x0200, 0x0400, ... */
struct serdes_cfg cfg_phyb_10p3125g_16bit_lane[] = {
	{0x0004,	 7,  0,		0x80},
	{0x0008,	 7,  0,		0x0d},
	{0x0008,	15,  8,		0x92},
	{0x0004,	31, 24,		0xfc},
	{0x0008,	 7,  0,		0x04},
	{0x0008,	15,  8,		0x91},
	{0x0010,	31, 24,		0x1a},
	{0x0014,	 7,  0,		0x58},
	{0x0014,	15,  8,		0x6b},
	{0x0014,	23, 16,		0x00},
	{0x0018,	 7,  0,		0x84},
	{0x0018,	23, 16,		0x80},
	{0x0018,	31, 24,		0x75},
	{0x002c,	23, 16,		0x30},
	{0x0030,	15,  8,		0x38},
	{0x004c,	23, 16,		0x8f},
	{0x0050,	31, 24,		0x30},
	{0x0060,	 7,  0,		0x02},
	{0x0064,	 7,  0,		0x57},
	{0x0068,	15,  8,		0x57},
	{0x0068,	23, 16,		0x57},
	{0x0078,	31, 24,		0xff},
	{0x0080,	 7,  0,		0x50},
	{0x0080,	23, 16,		0x50},
	{0x0084,	 7,  0,		0x15},
	{0x0084,	15,  8,		0x1f},
	{0x008c,	15,  8,		0x6f},
	{0x0094,	15,  8,		0x00},
	{0x0094,	23, 16,		0x00},
	{0x0094,	31, 24,		0x00},
	{0x0098,	 7,  0,		0x40},
	{0x0098,	15,  8,		0x26},
	{0x0098,	31, 24,		0x00},
	{0x009c,	 7,  0,		0x03},
	{0x00a4,	 7,  0,		0x13},
	{0x00a4,	15,  8,		0x0f},
	{0x00a8,	15,  8,		0xb6},
	{0x00a8,	23, 16,		0x01},
	{0x0180,	 7,  0,		0x30},
	{0x01c0,	15,  8,		0x02},
	{0x01cc,	 7,  0,		0x18},
	{0x01cc,	 7,  0,		0x00},
};

/* offset w.r.t. LaneX SS 0x0200, 0x0400, ... */
struct serdes_cfg cfg_phyb_aneg_lane[] = {
	{0x0004,	 7,  0,		0x80},
	{0x0004,	31, 24,		0x70},
	{0x0008,	 7,  0,		0x00},
	{0x0008,	15,  8,		0x00},
	{0x0010,	31, 24,		0x1b},
	{0x0014,	 7,  0,		0xf8},
	{0x0014,	15,  8,		0x6b},
	{0x0014,	23, 16,		0x00},
	{0x0018,	 7,  0,		0x74},
	{0x0018,	23, 16,		0x80},
	{0x0018,	31, 24,		0x75},
	{0x0030,	15,  8,		0x00},
	{0x004c,	23, 16,		0x8f},
	{0x0050,	31, 24,		0x00},
	{0x0060,	 7,  0,		0x00},
	{0x0064,	 7,  0,		0x57},
	{0x0068,	15,  8,		0x57},
	{0x0068,	23, 16,		0x57},
	{0x0078,	31, 24,		0xff},
	{0x0080,	 7,  0,		0x50},
	{0x0080,	23, 16,		0x50},
	{0x0084,	 7,  0,		0x05},
	{0x0094,	15,  8,		0x00},
	{0x0094,	23, 16,		0x00},
	{0x0094,	31, 24,		0x00},
	{0x0098,	 7,  0,		0x00},
	{0x0098,	15,  8,		0x00},
	{0x0098,	31, 24,		0x00},
	{0x009c,	 7,  0,		0x01},
	{0x00a4,	 7,  0,		0x0a},
	{0x00a4,	15,  8,		0x00},
	{0x00a8,	15,  8,		0xb6},
	{0x00a8,	23, 16,		0x01},
	{0x0180,	 7,  0,		0xb0},
	{0x0180,	 7,  0,		0x30},
	{0x01c0,	15,  8,		0x00},
	{0x01cc,	 7,  0,		0x18},
	{0x01cc,	 7,  0,		0x18},
};

/* offset w.r.t. serdes SS */
struct serdes_cfg cfg_phyb_10p3125g_comlane[] = {
	{0x0a00,	15,  8,		0x08},
	{0x0a84,	 7,  0,		0x00},
	{0x0a8c,	23, 16,		0x13},
	{0x0a90,	23, 16,		0xa0},
	{0x0a90,	31, 24,		0x77},
	{0x0a94,	 7,  0,		0x77},
	{0x0a94,	15,  8,		0x77},
	{0x0b08,	23, 16,		0x04},
	{0x0b08,	31, 24,		0x00},
	{0x0b0c,	 7,  0,		0x00},
	{0x0b0c,	15,  8,		0x00},
	{0x0b0c,	23, 16,		0x0f},
	{0x0b10,	31, 24,		0xbe},
	{0x0b14,	 7,  0,		0xff},
	{0x0b18,	 7,  0,		0x14},
	{0x0b5c,	23, 16,		0x1b},
	{0x0b5c,	31, 24,		0x98},
	{0x0b64,	15,  8,		0x11},
	{0x0b78,	15,  8,		0x0c},
	{0x0abc,	31, 24,		0xe0},
	{0x0ac0,	 7,  0,		0x8b},
};

/* offset w.r.t. serdes SS */
struct serdes_cfg cfg_phyb_1p25g_156p25mhz_comlane[] = {
	{0x0a00,	15,  8,		0x08},
	{0x0a84,	 7,  0,		0x00},
	{0x0a90,	23, 16,		0xa0},
	{0x0a90,	31, 24,		0x77},
	{0x0a94,	 7,  0,		0x77},
	{0x0a94,	15,  8,		0x77},
	{0x0b08,	23, 16,		0x00},
	{0x0b08,	31, 24,		0x00},
	{0x0b0c,	 7,  0,		0x00},
	{0x0b0c,	15,  8,		0x00},
	{0x0b0c,	23, 16,		0x00},
	{0x0b10,	31, 24,		0xbe},
	{0x0b14,	 7,  0,		0xff},
	{0x0b18,	 7,  0,		0x14},
	{0x0b5c,	23, 16,		0x1b},
	{0x0b5c,	31, 24,		0x98},
	{0x0b64,	15,  8,		0x11},
	{0x0b78,	15,  8,		0x0c},
	{0x0abc,	31, 24,		0xff},
	{0x0ac0,	 7,  0,		0x8b},
	{0x0a48,	15,  8,		0x8c},
	{0x0a48,	23, 16,		0xfd},
	{0x0a54,	 7,  0,		0x72},
	{0x0a54,	15,  8,		0xec},
	{0x0a54,	23, 16,		0x2f},
	{0x0a58,	15,  8,		0x21},
	{0x0a58,	23, 16,		0xf9},
	{0x0a58,	31, 24,		0x00},
	{0x0a5c,	 7,  0,		0x60},
	{0x0a5c,	15,  8,		0x00},
	{0x0a5c,	23, 16,		0x04},
	{0x0a5c,	31, 24,		0x00},
	{0x0a60,	 7,  0,		0x00},
	{0x0a60,	15,  8,		0x80},
	{0x0a60,	23, 16,		0x00},
	{0x0a60,	31, 24,		0x00},
	{0x0a64,	 7,  0,		0x20},
	{0x0a64,	15,  8,		0x12},
	{0x0a64,	23, 16,		0x58},
	{0x0a64,	31, 24,		0x0c},
	{0x0a68,	 7,  0,		0x02},
	{0x0a68,	15,  8,		0x06},
	{0x0a68,	23, 16,		0x3b},
	{0x0a68,	31, 24,		0xe1},
	{0x0a6c,	 7,  0,		0xc1},
	{0x0a6c,	15,  8,		0x4c},
	{0x0a6c,	23, 16,		0x07},
	{0x0a6c,	31, 24,		0xb8},
	{0x0a70,	 7,  0,		0x89},
	{0x0a70,	15,  8,		0xe9},
	{0x0a70,	23, 16,		0x02},
	{0x0a70,	31, 24,		0x3f},
	{0x0a74,	 7,  0,		0x01},
	{0x0ba4,	31, 24,		0x37},
	{0x0ba8,	 7,  0,		0x5d},
	{0x0ba8,	23, 16,		0x37},
};


struct serdes_cfg cfg_100mhz_pci_5gbps[] = {
	{0x0000,	15,	 8,	0x08},
	{0x0060,	 7,	 0,	0x5c},
	{0x0060,	15,	 8,	0x1c},
	{0x0060,	23,	16,	0x04},
	{0x0064,	15,	 8,	0xc7},
	{0x0064,	23,	16,	0x43},
	{0x0064,	31,	24,	0x03},
	{0x006c,	 7,	 0,	0x12},
	{0x0068,	23,	16,	0x07},
	{0x0078,	15,	 8,	0xc0},
	{0x0200,	 7,	 0,	0x00},
	{0x0204,	 7,	 0,	0x80},
	{0x0204,	31,	24,	0x5e},
	{0x0208,	 7,	 0,	0x06},
	{0x0208,	23,	16,	0x01},
	{0x0210,	 7,	 0,	0x23},
	{0x0214,	 7,	 0,	0x60},
	{0x0214,	15,	 8,	0x30},
	{0x0214,	31,	24,	0x2e},
	{0x0218,	31,	24,	0x76},
	{0x022c,	 7,	 0,	0x02},
	{0x022c,	23,	16,	0x20},
	{0x02a0,	23,	16,	0xee},
	{0x02a0,	31,	24,	0xff},
	{0x02a4,	 7,	 0,	0x0f},
	{0x0204,	31,	24,	0x5e},
	{0x0208,	 7,	 0,	0x06},
	{0x0278,	15,	 8,	0x20},
	{0x0280,	 7,	 0,	0x28},
	{0x0280,	23,	16,	0x28},
	{0x0284,	 7,	 0,	0x85},
	{0x0284,	15,	 8,	0x03},
	{0x0284,	23,	16,	0x0f},
	{0x0284,	31,	24,	0x2d},
	{0x0250,	31,	24,	0xd0},
	{0x0284,	 7,	 0,	0x85},
	{0x0294,	31,	24,	0x20},
	{0x0400,	 7,	 0,	0x00},
	{0x0404,	 7,	 0,	0x80},
	{0x0404,	31,	24,	0x5e},
	{0x0408,	 7,	 0,	0x06},
	{0x0408,	23,	16,	0x01},
	{0x0410,	 7,	 0,	0x23},
	{0x0414,	 7,	 0,	0x60},
	{0x0414,	15,	 8,	0x30},
	{0x0414,	31,	24,	0x2e},
	{0x0418,	31,	24,	0x76},
	{0x042c,	 7,	 0,	0x02},
	{0x042c,	23,	16,	0x20},
	{0x04a0,	23,	16,	0xee},
	{0x04a0,	31,	24,	0xff},
	{0x04a4,	 7,	 0,	0x0f},
	{0x0404,	31,	24,	0x5e},
	{0x0408,	 7,	 0,	0x06},
	{0x0478,	15,	 8,	0x20},
	{0x0480,	 7,	 0,	0x28},
	{0x0480,	23,	16,	0x28},
	{0x0484,	 7,	 0,	0x85},
	{0x0484,	15,	 8,	0x03},
	{0x0484,	23,	16,	0x0f},
	{0x0484,	31,	24,	0x2d},
	{0x0450,	31,	24,	0xd0},
	{0x0494,	31,	24,	0x20},
	{0x0604,	 7,	 0,	0x80},
	{0x0600,	 7,	 0,	0x00},
	{0x0604,	31,	24,	0x5e},
	{0x0608,	 7,	 0,	0x06},
	{0x0608,	23,	16,	0x01},
	{0x0610,	 7,	 0,	0x23},
	{0x0614,	 7,	 0,	0x60},
	{0x0614,	15,	 8,	0x30},
	{0x0614,	31,	24,	0x2e},
	{0x0618,	31,	24,	0x76},
	{0x062c,	 7,	 0,	0x02},
	{0x062c,	23,	16,	0x20},
	{0x06a0,	23,	16,	0xee},
	{0x06a0,	31,	24,	0xff},
	{0x06a4,	 7,	 0,	0x0f},
	{0x0604,	31,	24,	0x5e},
	{0x0608,	 7,	 0,	0x06},
	{0x0678,	15,	 8,	0x20},
	{0x0680,	 7,	 0,	0x28},
	{0x0680,	23,	16,	0x28},
	{0x0684,	 7,	 0,	0x85},
	{0x0684,	15,	 8,	0x03},
	{0x0684,	23,	16,	0x0f},
	{0x0684,	31,	24,	0x2d},
	{0x0650,	31,	24,	0xd0},
	{0x0694,	31,	24,	0x20},
	{0x0800,	 7,	 0,	0x00},
	{0x0804,	 7,	 0,	0x80},
	{0x0804,	31,	24,	0x5e},
	{0x0808,	 7,	 0,	0x06},
	{0x0810,	 7,	 0,	0x23},
	{0x0814,	 7,	 0,	0x60},
	{0x0814,	15,	 8,	0x30},
	{0x0814,	31,	24,	0x2e},
	{0x0818,	31,	24,	0x76},
	{0x082c,	 7,	 0,	0x02},
	{0x082c,	23,	16,	0x20},
	{0x08a0,	23,	16,	0xee},
	{0x08a0,	31,	24,	0xff},
	{0x08a4,	 7,	 0,	0x0f},
	{0x0804,	31,	24,	0x5e},
	{0x0808,	 7,	 0,	0x06},
	{0x0808,	23,	16,	0x01},
	{0x0878,	15,	 8,	0x20},
	{0x0880,	 7,	 0,	0x28},
	{0x0880,	23,	16,	0x28},
	{0x0884,	 7,	 0,	0x85},
	{0x0884,	15,	 8,	0x03},
	{0x0884,	23,	16,	0x0f},
	{0x0884,	31,	24,	0x2d},
	{0x0850,	31,	24,	0xd0},
	{0x0894,	31,	24,	0x20},
	{0x0a00,	15,	 8,	0x01},
	{0x0a08,	 7,	 0,	0x08},
	{0x0a08,	15,	 8,	0x2c},
	{0x0a08,	23,	16,	0xe1},
	{0x0a0c,	 7,	 0,	0x81},
	{0x0a18,	23,	16,	0xe8},
	{0x0a30,	15,	 8,	0x2f},
	{0x0a30,	23,	16,	0x2f},
	{0x0a4c,	23,	16,	0x82},
	{0x0a4c,	31,	24,	0xac},
	{0x0a54,	31,	24,	0xc0},
	{0x0a58,	 7,	 0,	0x41},
	{0x0a58,	15,	 8,	0x14},
	{0x0a84,	 7,	 0,	0x01},
	{0x0a84,	15,	 8,	0x03},
	{0x0a8c,	23,	16,	0x03},
	{0x0a8c,	31,	24,	0x81},
	{0x0a90,	 7,	 0,	0x01},
	{0x0a90,	15,	 8,	0x60},
	{0x0a94,	31,	24,	0x01},
	{0x0aa0,	31,	24,	0x81},
	{0x0aa4,	 7,	 0,	0x35},
	{0x0aa4,	15,	 8,	0xa8},
	{0x0abc,	31,	24,	0xff},
	{0x0ac0,	 7,	 0,	0x8b},
	{0x0a44,	15,	 8,	0x3d},
	{0x0a44,	23,	16,	0x73},
	{0x0a44,	31,	24,	0x5f},
	{0x0a48,	15,	 8,	0xca},
	{0x0a48,	23,	16,	0xfd},
	{0x0a5c,	23,	16,	0x00},
	{0x0a5c,	31,	24,	0x00},
	{0x0a60,	 7,	 0,	0x00},
	{0x0a60,	15,	 8,	0x80},
	{0x0a60,	23,	16,	0x00},
	{0x0a60,	31,	24,	0x00},
	{0x0a64,	 7,	 0,	0x20},
	{0x0a64,	15,	 8,	0x12},
	{0x0a64,	23,	16,	0x58},
	{0x0a64,	31,	24,	0x0c},
	{0x0a68,	 7,	 0,	0x02},
	{0x0a68,	15,	 8,	0x06},
	{0x0a68,	23,	16,	0x3b},
	{0x0a68,	31,	24,	0xe1},
	{0x0a6c,	 7,	 0,	0xc1},
	{0x0a6c,	15,	 8,	0x4c},
	{0x0a6c,	23,	16,	0x07},
	{0x0a6c,	31,	24,	0xb8},
	{0x0a70,	 7,	 0,	0x89},
	{0x0a70,	15,	 8,	0xe9},
	{0x0a70,	23,	16,	0x02},
	{0x0a70,	31,	24,	0x3f},
	{0x0a74,	 7,	 0,	0x01},
	{0x0b14,	23,	16,	0x37},
	{0x0b10,	31,	24,	0x37},
	{0x0b14,	 7,	 0,	0x5d},
	{0x0000,	 7,	 0,	0x03},
	{0x0a00,	 7,	 0,	0x9f},
};

static inline u32 kserdes_readl(void __iomem *base, u32 offset)
{
	return readl(base + offset);
}

static inline void kserdes_writel(void __iomem *base, u32 offset, u32 value)
{
	writel(value, base + offset);
}

static void kserdes_do_config(void __iomem *base,
		     struct serdes_cfg *cfg, u32 size)
{
	u32 i;

	for (i = 0; i < size; i++) {
		FINSR(base, cfg[i].ofs, cfg[i].msb, cfg[i].lsb, cfg[i].val);
	}
}

static inline u32 _kserdes_read_tbus_val(void __iomem *sregs)
{
	u32 tmp;

	if (PHY_A(sregs)) {
		tmp  = ((kserdes_readl(sregs, CMU0_REG(0xec))) >> 24) & 0x0ff;
		tmp |= ((kserdes_readl(sregs, CMU0_REG(0xfc))) >> 16) & 0xf00;
	} else
		tmp  = ((kserdes_readl(sregs, CMU0_REG(0xf8))) >> 16) & 0xfff;

	return tmp;
}

static void _kserdes_write_tbus_addr(void __iomem *sregs, int select, int ofs)
{
	if (select && !FOUR_LANE(sregs))
		++select;

	if (PHY_A(sregs))
		FINSR(sregs, CMU0_REG(0x8), 31, 24, ((select << 5) + ofs));
	else
		FINSR(sregs, CMU0_REG(0xfc), 26, 16, ((select << 8) + ofs));
}

static u32 _kserdes_read_select_tbus(void __iomem *sregs, int select, int ofs)
{
	/* set tbus address */
	_kserdes_write_tbus_addr(sregs, select, ofs);
	/* get tbus value */
	return _kserdes_read_tbus_val(sregs);
}

static inline void _kserdes_cfg_156p25mhz_10bit_5gbps(void __iomem *sregs)
{
	kserdes_do_config(sregs, cfg_156p25mhz_10bit_5gbps,
			ARRAY_SIZE(cfg_156p25mhz_10bit_5gbps));
}

static inline void _kserdes_cfg_phyb_1p25g_156p25mhz_cmu0(void __iomem *sregs)
{
	kserdes_do_config(sregs, cfg_phyb_1p25g_156p25mhz_cmu0,
			ARRAY_SIZE(cfg_phyb_1p25g_156p25mhz_cmu0));
}

static inline void _kserdes_cfg_phyb_10p3125g_156p25mhz_cmu1(
						void __iomem *sregs)
{
	kserdes_do_config(sregs, cfg_phyb_10p3125g_156p25mhz_cmu1,
			ARRAY_SIZE(cfg_phyb_10p3125g_156p25mhz_cmu1));
}

static inline void kserdes_tap1_patch(struct kserdes_config *sc)
{
	FINSR(sc->regs, CML_REG(0xbc), 28, 24, 0x1e);
}

static inline void kserdes_set_tx_idle(struct kserdes_config *sc, u32 lane)
{
	if (sc->phy_type != KSERDES_PHY_XGE)
		FINSR(sc->regs, LANEX_REG(lane, 0xb8),   17, 16, 3);

	FINSR(sc->regs, LANE_CTRL_STS_REG(lane), 25, 24, 3);
	FINSR(sc->regs, LANEX_REG(lane, 0x28),   21, 20, 0);
}

static inline void kserdes_clr_tx_idle(struct kserdes_config *sc, u32 lane)
{
	if (sc->phy_type != KSERDES_PHY_XGE)
		FINSR(sc->regs, LANEX_REG(lane, 0xb8),   17, 16, 0);

	FINSR(sc->regs, LANE_CTRL_STS_REG(lane), 25, 24, 0);
	FINSR(sc->regs, LANEX_REG(lane, 0x28),   21, 20, 0);
}

static inline void kserdes_cdfe_enable(struct kserdes_config *sc, u32 lane)
{
	FINSR(sc->regs,  LANEX_REG(lane, 0x94), 24, 24, 0x1);
}

static inline void kserdes_cdfe_force_calibration_enable(
			struct kserdes_config *sc, u32 lane)
{
	FINSR(sc->regs, LANEX_REG(lane, 0x98), 0, 0, 0x1);
}

static void kserdes_phya_lane_patch(struct kserdes_config *sc, u32 lane)
{
	/* pma_ln_vreg */
	FINSR(sc->regs, LANEX_REG(lane, 0x18), 25, 24, 0x2);
	/* pma_ln_vregh */
	FINSR(sc->regs, LANEX_REG(lane, 0x18), 27, 26, 0x2);
	/* pma_int_step */
	FINSR(sc->regs, LANEX_REG(lane, 0x14), 15, 13, 0x1);
	/* turn off att_boost */
	FINSR(sc->regs, LANEX_REG(lane, 0x4c), 19, 16, 0xf);
	/* set dfe_bias to 10 */
	FINSR(sc->regs, LANEX_REG(lane, 0x4c), 23, 20, 0xa);
	/* Set offset average num of samples to max value */
	FINSR(sc->regs, LANEX_REG(lane, 0x78), 30, 24, 0x7f);
}

static void kserdes_phyb_patch(struct kserdes_config *sc)
{
	/* Enables the Center DFE */
	for_each_enable_lane(kserdes_cdfe_enable, sc);

	/* setting initial cdfe */
	FINSR(sc->regs, CML_REG(0x108), 23, 16, 0x04);

	/* setting rx tap */
	FINSR(sc->regs, CML_REG(0xbc), 28, 24, 0x0);

	/* enable cdfe_ln_force_cal for cdfe */
	for_each_lane(kserdes_cdfe_force_calibration_enable, sc);
}

static inline void kserdes_set_lane_starts(struct kserdes_config *sc, u32 lane)
{
	/* att start -1 for short channel */
	FINSR(sc->regs, LANEX_REG(lane, 0x8c), 11, 8,
				sc->lane[lane].rx_start.att);
	/* boost start -3 for short channel */
	FINSR(sc->regs, LANEX_REG(lane, 0x8c), 15, 12,
				sc->lane[lane].rx_start.boost);
}

static void kserdes_phy_patch(struct kserdes_config *sc)
{
	if (sc->phy_type == KSERDES_PHY_XGE)
		kserdes_phyb_patch(sc);
	else if (sc->link_rate >= KSERDES_LINK_RATE_9P8304G)
		for_each_enable_lane(kserdes_phya_lane_patch, sc);

	/* Set ATT and BOOST start values for each lane */
	for_each_enable_lane(kserdes_set_lane_starts, sc);
}

static inline void _kserdes_set_training_pattern(void __iomem *sregs)
{
	FINSR(sregs, CML_REG(0xc8), 5, 0, 0x0f);
}

static void kserdes_set_lane_overrides(struct kserdes_config *sc, u32 lane)
{
	u32 val_0, val_1, val;

	/* read laneX_ctrl_i/laneX_pd_i */
	val_0 = _kserdes_read_select_tbus(sc->regs, lane + 1, 0);

	/* read laneX_rate_i */
	val_1 = _kserdes_read_select_tbus(sc->regs, lane + 1, 1);

	/* set RESET state */
	val = 0;
	/* user rate */
	val |= ((val_1 >> 9) & 0x3) << 1;
	/* user PD */
	val |= (val_0 & 0x3) << 3;
	/* user ctrl_i */
	val |= ((val_0 >> 2) & 0x1ff) << 5;
	/* set override */
	val |= (1 << 14);
	/* try claer TX Valid bits */
	val &= ~0x60;

	/* Only modify the reset bit and the overlay bit */
	FINSR(sc->regs, LANEX_REG(lane, 0x028), 29, 15, val);
}

static inline void kserdes_assert_reset(struct kserdes_config *sc)
{
	for_each_enable_lane(kserdes_set_lane_overrides, sc);
}

static inline void kserdes_config_c1_c2_cm(struct kserdes_config *sc, u32 lane)
{
	u32 c1, c2, cm;

	c1 = sc->lane[lane].tx_coeff.c1;
	c2 = sc->lane[lane].tx_coeff.c2;
	cm = sc->lane[lane].tx_coeff.cm;

	if (sc->phy_type == KSERDES_PHY_XGE) {
		/* TX Control override enable */
		FINSR(sc->regs, LANEX_REG(lane, 0x8), 11,  8, (cm & 0xf));
		FINSR(sc->regs, LANEX_REG(lane, 0x8),  4,  0, (c1 & 0x1f));
		FINSR(sc->regs, LANEX_REG(lane, 0x8),  7,  5, (c2 & 0x7));
		FINSR(sc->regs, LANEX_REG(lane, 0x4),
				18, 18, ((c2 >> 3) & 0x1));
	} else {
		FINSR(sc->regs, LANEX_REG(lane, 0x8), 15, 12, (cm & 0xf));
		FINSR(sc->regs, LANEX_REG(lane, 0x8),  4,  0, (c1 & 0x1f));
		FINSR(sc->regs, LANEX_REG(lane, 0x8), 11,  8, (c2 & 0xf));
	}
}

static inline void kserdes_config_att_boost(struct kserdes_config *sc, u32 lane)
{
	u32 att, boost;

	att = sc->lane[lane].rx_force.att;
	boost = sc->lane[lane].rx_force.boost;

	if (sc->phy_type == KSERDES_PHY_XGE) {
		FINSR(sc->regs, LANEX_REG(lane, 0x98), 13, 13, 0);
		FINSR(sc->regs, LANEX_REG(lane, 0x8c), 15, 12, boost);
		FINSR(sc->regs, LANEX_REG(lane, 0x8c), 11, 8, att);
	} else {
		if (att != 1) {
			FINSR(sc->regs, CML_REG(0x84), 0, 0, 0);
			FINSR(sc->regs, CML_REG(0x8c), 24, 24, 0);
			FINSR(sc->regs, LANEX_REG(lane, 0x8c), 11, 8, att);
		}
		if (boost != 1) {
			FINSR(sc->regs, CML_REG(0x84), 1, 1, 0);
			FINSR(sc->regs, CML_REG(0x8c), 25, 25, 0);
			FINSR(sc->regs, LANEX_REG(lane, 0x8c), 15, 12, boost);
		}
	}
}

static void kserdes_set_tx_rx_fir_coeff(struct kserdes_config *sc, u32 lane)
{
	struct kserdes_tx_coeff *tc = &(sc->lane[lane].tx_coeff);

	if (sc->phy_type == KSERDES_PHY_XGE) {
		/* Tx Swing */
		FINSR(sc->regs, LANEX_REG(lane, 0x004), 29, 26, tc->att);
		/* Regulator voltage */
		FINSR(sc->regs, LANEX_REG(lane, 0x0a4), 2, 0, tc->vreg);
	} else {
		/* Tx Swing */
		FINSR(sc->regs, LANEX_REG(lane, 0x004), 28, 25, tc->att);
		/* Regulator voltage */
		FINSR(sc->regs, LANEX_REG(lane, 0x084), 7, 5, tc->vreg);
	}

	kserdes_config_c1_c2_cm(sc, lane);

	if (sc->rx_force_enable)
		kserdes_config_att_boost(sc, lane);
}

static inline void _kserdes_force_signal_detect_low(
				void __iomem *sregs, u32 lane)
{
	FINSR(sregs, LANEX_REG(lane, 0x004), 2, 1, 0x2);
}

static inline void kserdes_force_signal_detect_low(
			struct kserdes_config *sc, u32 lane)
{
	_kserdes_force_signal_detect_low(sc->regs, lane);
}

static inline void _kserdes_force_signal_detect_high(
				void __iomem *sregs, u32 lane)
{
	FINSR(sregs, LANEX_REG(lane, 0x004), 2, 1, 0x0);
}

static inline void kserdes_force_signal_detect_high(
			struct kserdes_config *sc, u32 lane)
{
	_kserdes_force_signal_detect_high(sc->regs, lane);
}

static int kserdes_deassert_reset_poll_others(struct kserdes_config *sc)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(500);
	u32 lanes_not_ok = 0;
	u32 ofs = 28;
	u32 ret, i;

	/* assume all enable lanes not-ok (1) and all others
	 * ok (0) to start
	 */
	for (i = 0; i < sc->lanes; i++) {
		if (!LANE_ENABLE(sc, i))
			continue;

		lanes_not_ok |= (1 << i);
	}

	/* This is not a mistake.  For 2-laner, we
	 * check bit 29 and 30,  NOT 28 and 29.
	 */
	if (!FOUR_LANE(sc->regs))
		ofs = 29;

	do {
		for (i = 0; i < sc->lanes; i++) {
			if (!LANE_ENABLE(sc, i))
				continue;

			/* no need to check again if this lane's status
			 * is already good
			 */
			if (!(lanes_not_ok & (1 << i)))
				continue;

			ret = kserdes_readl(sc->regs, CML_REG(0x1f8));

			/* clear corresponding lane_not_ok bit if
			 * status is good (1)
			 */
			if (ret & BIT(ofs + i))
				lanes_not_ok &= ~(1 << i);
		}

		/* get out if all lanes are good to go */
		if (!lanes_not_ok)
			return 0;

		if (time_after(jiffies, timeout))
			return -ETIMEDOUT;

		cpu_relax();
	} while (true);
}

static int kserdes_deassert_reset_poll_pcie(struct kserdes_config *sc)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(500);
	u32 lanes_not_ok = 0;
	u32 ret, i;

	/* assume all enable lanes not-ok (1) and all others
	 * ok (0) to start
	 */
	for (i = 0; i < sc->lanes; i++) {
		if (!LANE_ENABLE(sc, i))
			continue;

		lanes_not_ok |= (1 << i);
	}

	do {
		for (i = 0; i < sc->lanes; i++) {
			if (!LANE_ENABLE(sc, i))
				continue;

			/* no need to check again if this lane's status
			 * is already good
			 */
			if (!(lanes_not_ok & (1 << i)))
				continue;

			ret = _kserdes_read_select_tbus(sc->regs, i + 1, 0x02);

			/* clear corresponding lane_not_ok bit if
			 * status is good (0)
			 */
			if (!(ret & BIT(4)))
				lanes_not_ok &= ~(1 << i);
		}

		/* get out if all lanes are good to go */
		if (!lanes_not_ok)
			return 0;

		if (time_after(jiffies, timeout))
			return -ETIMEDOUT;

		cpu_relax();
	} while (true);
}

static inline void _kserdes_lane_reset(void __iomem *serdes,
					u32 lane, u32 reset)
{
	if (reset)
		FINSR(serdes, LANEX_REG(lane, 0x28), 29, 29, 0x1);
	else
		FINSR(serdes, LANEX_REG(lane, 0x28), 29, 29, 0x0);
}

static inline void kserdes_release_reset(struct kserdes_config *sc, u32 lane)
{
	if (sc->phy_type == KSERDES_PHY_XGE) {
		/* set pma_cmu_sel to 1 */
		FINSR(sc->regs, LANEX_REG(lane, 0x60), 0, 0, 0x1);
	}
	/* release reset */
	_kserdes_lane_reset(sc->regs, lane, 0);
}

static int kserdes_deassert_reset(struct kserdes_config *sc, u32 poll)
{
	int ret = 0;

	for_each_enable_lane(kserdes_release_reset, sc);

	if (!poll)
		goto done;

	/* Check Lane OK */
	if (sc->phy_type == KSERDES_PHY_PCIE)
		ret = kserdes_deassert_reset_poll_pcie(sc);
	else
		ret = kserdes_deassert_reset_poll_others(sc);

done:
	return ret;
}

static inline void kserdes_lane_disable(struct kserdes_config *sc, u32 lane)
{
	FINSR(sc->regs, LANE_CTRL_STS_REG(lane), 31, 29, 0x4);
	FINSR(sc->regs, LANE_CTRL_STS_REG(lane), 15, 13, 0x4);
}

static inline void _kserdes_lane_enable(void __iomem *sregs, u32 lane)
{
	FINSR(sregs, LANE_CTRL_STS_REG(lane), 31, 29, 0x7);
	FINSR(sregs, LANE_CTRL_STS_REG(lane), 15, 13, 0x7);
}

/* Caller should make sure sgmii cannot be fullrate */
static inline int _kserdes_set_lane_ctrl_rate(
	void __iomem			*sregs,
	u32				lane,
	enum KSERDES_LANE_CTRL_RATE	lane_ctrl_rate)
{
	u32 rate_mode;

	if (lane_ctrl_rate == KSERDES_FULL_RATE)
		rate_mode = 0x4;
	else if (lane_ctrl_rate == KSERDES_QUARTER_RATE)
		rate_mode = 0x6;
	else if (lane_ctrl_rate == KSERDES_HALF_RATE)
		rate_mode = 0x5;
	else
		return -EINVAL;

	/* Tx */
	FINSR(sregs, LANE_CTRL_STS_REG(lane), 28, 26, rate_mode);
	/* Rx */
	FINSR(sregs, LANE_CTRL_STS_REG(lane), 12, 10, rate_mode);
	return 0;
}

static inline void _kserdes_set_lane_loopback(
	void __iomem			*sregs,
	u32				lane,
	enum KSERDES_LINK_RATE		link_rate)
{
	if (link_rate == KSERDES_LINK_RATE_10P3125G) {
		FINSR(sregs, LANEX_REG(lane, 0x0), 7, 0, 0x4);
		FINSR(sregs, LANEX_REG(lane, 0x4), 2, 1, 0x3);
	} else
		FINSR(sregs, LANEX_REG(lane, 0x0), 31, 24, 0x40);
}

static void kserdes_set_lane_rate(struct kserdes_config *sc, u32 lane)
{
	int ret;

	ret = _kserdes_set_lane_ctrl_rate(sc->regs, lane,
				sc->lane[lane].ctrl_rate);
	if (ret) {
		ks_err("set_lane_rate FAILED: lane = %d err = %d\n",
			lane, ret);
		return;
	}

	/* disable attenuation auto scale */
	FINSR(sc->regs, LANEX_REG(lane, 0x30), 11, 11, 0x1);
	FINSR(sc->regs, LANEX_REG(lane, 0x30), 13, 12, 0x0);

	/* set NES bit if loopback enabled */
	if (sc->lane[lane].loopback)
		_kserdes_set_lane_loopback(sc->regs, lane, sc->link_rate);

	_kserdes_lane_enable(sc->regs, lane);
}

static inline void _kserdes_set_wait_after(void __iomem *sregs)
{
	FINSR(sregs, PLL_CTRL_REG, 17, 16, 0x3);
}

static inline void _kserdes_clear_wait_after(void __iomem *sregs)
{
	FINSR(sregs, PLL_CTRL_REG, 17, 16, 0);
}

static inline void _kserdes_pll_enable(void __iomem *sregs)
{
	FINSR(sregs, PLL_CTRL_REG, 31, 29, 0x7);
}

static inline void _kserdes_pll2_enable(void __iomem *sregs)
{
	FINSR(sregs, PLL_CTRL_REG, 27, 25, 0x7);
}

static inline void _kserdes_pll_disable(void __iomem *sregs)
{
	FINSR(sregs, PLL_CTRL_REG, 31, 29, 0x4);
}

static inline void _kserdes_pll2_disable(void __iomem *sregs)
{
	FINSR(sregs, PLL_CTRL_REG, 27, 25, 0x4);
}

static inline u32 _kserdes_get_pll_status(void __iomem *sregs)
{
	return FEXTR(kserdes_readl(sregs, PLL_CTRL_REG), 28, 28);
}

static inline u32 _kserdes_get_pll2_status(void __iomem *sregs)
{
	return FEXTR(kserdes_readl(sregs, PLL_CTRL_REG), 24, 24);
}

static inline void kserdes_lane_enable_loopback(void __iomem *serdes, u32 lane)
{
	FINSR(serdes, LANEX_REG(lane, 0), 31, 24, 0x40);
}

static inline u32 _kserdes_get_lane_status(
		void __iomem		*sregs,
		u32			lane,
		enum KSERDES_PHY_TYPE	phy_type)
{
	if (phy_type == KSERDES_PHY_PCIE) {
		return FEXTR(kserdes_readl(sregs, PLL_CTRL_REG), lane, lane);
	} else if (phy_type == KSERDES_PHY_XGE) {
		return FEXTR(kserdes_readl(sregs, CML_REG(0x1f8)),
				(29 + lane), (29 + lane));
	} else {
		return FEXTR(kserdes_readl(sregs, PLL_CTRL_REG),
				(8 + lane), (8 + lane));
	}
}

static u32 kserdes_get_pll_lanes_status(struct kserdes_config *sc)
{
	u32 val, i;

	/* Check PLL OK Status Bit */
	val = _kserdes_get_pll_status(sc->regs);
	if (!val) {
		/* pll is not ready */
		goto done;
	}

	if (sc->phy_type == KSERDES_PHY_XGE) {
		val = _kserdes_get_pll2_status(sc->regs);
		if (!val)
			goto done;
	}

	/* Check Lane OK Status Bits */
	for (i = 0; i < sc->lanes; i++) {
		if (!LANE_ENABLE(sc, i))
			continue;

		val &= _kserdes_get_lane_status(sc->regs, i, sc->phy_type);
	}

done:
	/* if any of the status is 0, this is 0
	 * i.e. serdes status is not good
	 */
	return val;
}

int kserdes_get_status(struct kserdes_config *sc)
{
	unsigned long timeout;

	/* is 500 msec a good number? */
	timeout = jiffies + msecs_to_jiffies(500);
	do {
		if (kserdes_get_pll_lanes_status(sc))
			break;

		if (time_after(jiffies, timeout))
			return -ETIMEDOUT;

		cpu_relax();
	} while (true);

	return 0;
}

static inline u32 _kserdes_get_tx_termination(
		void __iomem		*sregs,
		enum KSERDES_PHY_TYPE	phy_type,
		u32			lane)
{
	return (_kserdes_read_select_tbus(sregs, lane + 1,
			((phy_type == KSERDES_PHY_XGE) ? 0x1a : 0x1b)) & 0xff);
}

static void kserdes_set_tx_terminations(struct kserdes_config *sc, u32 term)
{
	u32 i;

	for (i = 0; i < sc->lanes; i++) {
		FINSR(sc->regs, LANEX_REG(i, 0x7c), 31, 24, term);
		/* set termination override */
		FINSR(sc->regs, LANEX_REG(i, 0x7c), 20, 20, 0x1);
	}
}

/* lane is 0-based */
static void _kserdes_get_cmp_tap_offsets_xge(void __iomem *sregs,
	u32 lane, u32 cmp, struct kserdes_comparator_tap_offsets *ofs)
{
	/* set comparator number */
	FINSR(sregs, CML_REG(0x8c), 23, 21, cmp);

	/* read offsets */
	FINSR(sregs, CMU0_REG(0xfc), 26, 16, ((lane + 2) << 8) + 0x11);
	ofs->cmp = (_kserdes_read_tbus_val(sregs) & 0x0ff0) >> 4;

	FINSR(sregs, CMU0_REG(0xfc), 26, 16, ((lane + 2) << 8) + 0x11);
	ofs->tap1 = (_kserdes_read_tbus_val(sregs) & 0x000f) << 3;

	FINSR(sregs, CMU0_REG(0xfc), 26, 16, ((lane + 2) << 8) + 0x12);
	ofs->tap1 |= (_kserdes_read_tbus_val(sregs) & 0x0e00) >> 9;
	ofs->tap2  = (_kserdes_read_tbus_val(sregs) & 0x01f8) >> 3;
	ofs->tap3  = (_kserdes_read_tbus_val(sregs) & 0x0007) << 3;

	FINSR(sregs, CMU0_REG(0xfc), 26, 16, ((lane + 2) << 8) + 0x13);
	ofs->tap3 |= (_kserdes_read_tbus_val(sregs) & 0x0e00) >> 9;
	ofs->tap4  = (_kserdes_read_tbus_val(sregs) & 0x01f8) >> 3;
	ofs->tap5  = (_kserdes_read_tbus_val(sregs) & 0x0007) << 3;

	FINSR(sregs, CMU0_REG(0xfc), 26, 16, ((lane + 2) << 8) + 0x14);
	ofs->tap5 |= (_kserdes_read_tbus_val(sregs) & 0x0e00) >> 9;
}

static void kserdes_add_offsets_xge(struct kserdes_config *sc,
					struct kserdes_offsets *sofs)
{
	struct kserdes_comparator_tap_offsets *ctofs;
	struct kserdes_comparator_tap_offsets sample;
	struct kserdes_lane_offsets *lofs;
	u32 lane, cmp;

	for (lane = 0; lane < sc->lanes; lane++) {
		lofs = &(sofs->lane_ofs[lane]);
		/* yes cmp starts from 1 */
		for (cmp = 1; cmp < MAX_COMPARATORS; cmp++) {
			ctofs = &(lofs->ct_ofs[cmp]);

			_kserdes_get_cmp_tap_offsets_xge(sc->regs,
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
static void kserdes_get_cmp_tap_offsets_non_xge(void __iomem *sregs,
	u32 lane, u32 cmp, struct kserdes_comparator_tap_offsets *ofs)
{
	/* set comparator number */
	FINSR(sregs, CML_REG(0x8c), 23, 21, cmp);

	/* read offsets */
	FINSR(sregs, CMU0_REG(0x8), 31, 24, ((lane + 1) << 5) + 0x12);
	ofs->cmp = (_kserdes_read_tbus_val(sregs) & 0x0ff0) >> 4;
}

static void kserdes_add_offsets_non_xge(struct kserdes_config *sc,
					struct kserdes_offsets *sofs)
{
	struct kserdes_comparator_tap_offsets *ctofs;
	struct kserdes_comparator_tap_offsets sample;
	struct kserdes_lane_offsets *lofs;
	u32 lane, cmp;

	for (lane = 0; lane < sc->lanes; lane++) {
		lofs = &(sofs->lane_ofs[lane]);
		/* yes cmp starts from 1 */
		for (cmp = 1; cmp < MAX_COMPARATORS; cmp++) {
			ctofs = &(lofs->ct_ofs[cmp]);

			kserdes_get_cmp_tap_offsets_non_xge(sc->regs,
					lane, cmp, &sample);

			ctofs->cmp  += sample.cmp;
		}
	}
}

static void kserdes_get_average_offsets(struct kserdes_config *sc, u32 samples,
					struct kserdes_offsets *sofs)
{
	struct kserdes_comparator_tap_offsets *ctofs;
	struct kserdes_lane_offsets *lofs;
	u32 i, lane, cmp;
	int ret;

	memset(sofs, 0, sizeof(*sofs));

	/* get the total of each offset for specified number of samples */
	for (i = 0; i < samples; i++) {
		kserdes_assert_reset(sc);
		ret = kserdes_deassert_reset(sc, 1);
		if (ret) {
			ks_err("kserdes_get_average_offsets: reset failed %d\n",
				ret);
			return;
		}

		if (sc->phy_type == KSERDES_PHY_XGE)
			kserdes_add_offsets_xge(sc, sofs);
		else
			kserdes_add_offsets_non_xge(sc, sofs);
	}

	/* take the average */
	for (lane = 0; lane < sc->lanes; lane++) {
		lofs = &(sofs->lane_ofs[lane]);
		/* yes cmp starts from 1 */
		for (cmp = 1; cmp < MAX_COMPARATORS; cmp++) {
			ctofs = &(lofs->ct_ofs[cmp]);
			if (sc->phy_type == KSERDES_PHY_XGE) {
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

static void _kserdes_override_cmp_tap_offsets(void __iomem *sregs,
	u32 lane, u32 cmp, struct kserdes_comparator_tap_offsets *ofs)
{
	/* set dfe_shadow_lane_sel */
	FINSR(sregs, CML_REG(0xf0), 27, 26, (lane + 1));

	/* set cmp_offset_ovr_en to 1 */
	FINSR(sregs, CML_REG(0x98), 24, 24, 0x1);

	/* set rxeq_ovr_en to 0x1 */
	FINSR(sregs, LANEX_REG(lane, 0x2c), 2, 2, 0x1);

	/* set rxeq_dfe_cmp_sel_ovr to comp_no */
	FINSR(sregs, LANEX_REG(lane, 0x30), 7, 5, cmp);

	/* set dfe_tap_ovr_en to 1 */
	FINSR(sregs, LANEX_REG(lane, 0x5c), 31, 31, 0x1);

	/* set cmp offset override */
	FINSR(sregs, CML_REG(0x9c), 7, 0, ofs->cmp);
	/* set tap offset overrides */
	FINSR(sregs, LANEX_REG(lane, 0x58), 30, 24, ofs->tap1);
	FINSR(sregs, LANEX_REG(lane, 0x5c),  5,  0, ofs->tap2);
	FINSR(sregs, LANEX_REG(lane, 0x5c), 13,  8, ofs->tap3);
	FINSR(sregs, LANEX_REG(lane, 0x5c), 21, 16, ofs->tap4);
	FINSR(sregs, LANEX_REG(lane, 0x5c), 29, 24, ofs->tap5);

	/* set rxeq_ovr_latch_o = 0x1 */
	FINSR(sregs, LANEX_REG(lane, 0x2c), 10, 10, 0x1);
	/* set rxeq_ovr_latch_o = 0x0 */
	FINSR(sregs, LANEX_REG(lane, 0x2c), 10, 10, 0x0);

	/* set cmp_offset_ovr_en to 0 */
	FINSR(sregs, CML_REG(0x98), 24, 24, 0x0);
	/* set rxeq_ovr_en to 0x0 */
	FINSR(sregs, LANEX_REG(lane, 0x2c), 2, 2, 0x0);
	/* set dfe_tap_ovr_en to 0 */
	FINSR(sregs, LANEX_REG(lane, 0x5c), 31, 31, 0x0);
}

static inline void _kserdes_override_cmp_offset_cdfe(void __iomem *sregs,
					u32 lane, u32 cmp, u32 cmp_offset)
{
	/* enable comparator offset calibrate */
	FINSR(sregs, LANEX_REG(lane, 0x58), 18, 18, 0x1);

	/* set gcfsm sel override to comparator */
	FINSR(sregs, LANEX_REG(lane, 0x4c), 5, 2, (0x1 << (cmp - 1)));
	/* set comparator offset */
	FINSR(sregs, LANEX_REG(lane, 0x48), 24, 17, cmp_offset);
	/* latch in value */
	FINSR(sregs, LANEX_REG(lane, 0x48), 29, 29, 0x1);
	FINSR(sregs, LANEX_REG(lane, 0x48), 29, 29, 0x0);

	/* disable comparator offset calibrate */
	FINSR(sregs, LANEX_REG(lane, 0x58), 18, 18, 0x0);
}

static inline void _kserdes_override_tap_offset_cdfe(void __iomem *sregs,
	u32 lane, u32 tap, u32 width, u32 tap_offset)
{
	/* enable tap */
	FINSR(sregs, LANEX_REG(lane, 0x58), 23, 19, BIT(tap - 1));
	/* set tap offset */
	FINSR(sregs, LANEX_REG(lane, 0x48), 17 + (width - 1), 17, tap_offset);
	/* latch in value */
	FINSR(sregs, LANEX_REG(lane, 0x48), 29, 29, 0x1);
	FINSR(sregs, LANEX_REG(lane, 0x48), 29, 29, 0x0);
}

static void _kserdes_override_cmp_tap_offsets_cdfe(
	void __iomem				*sregs,
	u32					lane,
	u32					cmp,
	struct kserdes_comparator_tap_offsets	*ofs)
{
	/* enable overrides */
	FINSR(sregs, LANEX_REG(lane, 0x58), 16, 16, 0x1);
	FINSR(sregs, LANEX_REG(lane, 0x48), 16, 16, 0x1);

	_kserdes_override_cmp_offset_cdfe(sregs, lane, cmp, ofs->cmp);

	/* enable tap offset calibrate */
	FINSR(sregs, LANEX_REG(lane, 0x58), 17, 17, 0x1);

	/* set tap offsets */
	_kserdes_override_tap_offset_cdfe(sregs, lane, 1, 7, ofs->tap1);
	_kserdes_override_tap_offset_cdfe(sregs, lane, 2, 6, ofs->tap2);
	_kserdes_override_tap_offset_cdfe(sregs, lane, 3, 6, ofs->tap3);
	_kserdes_override_tap_offset_cdfe(sregs, lane, 4, 6, ofs->tap4);
	_kserdes_override_tap_offset_cdfe(sregs, lane, 5, 6, ofs->tap5);

	/* disable overrides */
	FINSR(sregs, LANEX_REG(lane, 0x58), 16, 16, 0x0);
	FINSR(sregs, LANEX_REG(lane, 0x48), 16, 16, 0x0);
	FINSR(sregs, LANEX_REG(lane, 0x58), 18, 18, 0x0);
	FINSR(sregs, LANEX_REG(lane, 0x58), 17, 17, 0x0);
}

static void kserdes_set_offsets_xge(struct kserdes_config *sc,
				struct kserdes_offsets *sofs)
{
	struct kserdes_comparator_tap_offsets *ctofs;
	struct kserdes_lane_offsets *lofs;
	u32 lane, cmp;

	for (lane = 0; lane < sc->lanes; lane++) {
		lofs = &(sofs->lane_ofs[lane]);
		for (cmp = 1; cmp < MAX_COMPARATORS; cmp++) {
			ctofs = &(lofs->ct_ofs[cmp]);
			_kserdes_override_cmp_tap_offsets(sc->regs,
						lane, cmp, ctofs);
			_kserdes_override_cmp_tap_offsets_cdfe(sc->regs,
						lane, cmp, ctofs);
		}
	}
}

static void kserdes_set_offsets_non_xge(struct kserdes_config *sc,
					struct kserdes_offsets *sofs)
{
	struct kserdes_comparator_tap_offsets *ctofs;
	struct kserdes_lane_offsets *lofs;
	u32 lane, cmp;

	for (lane = 0; lane < sc->lanes; lane++) {
		lofs = &(sofs->lane_ofs[lane]);
		for (cmp = 1; cmp < MAX_COMPARATORS; cmp++) {
			ctofs = &(lofs->ct_ofs[cmp]);
			_kserdes_override_cmp_tap_offsets(sc->regs,
						lane, cmp, ctofs);
		}
	}
}

static void kserdes_set_offsets(struct kserdes_config *sc,
				struct kserdes_offsets *sofs)
{
	if (sc->phy_type == KSERDES_PHY_XGE)
		kserdes_set_offsets_xge(sc, sofs);
	else
		kserdes_set_offsets_non_xge(sc, sofs);
}

static void kserdes_dfe_offset_calibration(struct kserdes_config *sc,
					struct kserdes_offsets *sofs)
{
	for_each_enable_lane(kserdes_force_signal_detect_low, sc);
	udelay(10);

	/* offset compensation patch */
	kserdes_get_average_offsets(sc, DFE_OFFSET_SAMPLES, sofs);
	kserdes_set_offsets(sc, sofs);
	udelay(10);

	/* re-acquire signal detect */
	for_each_lane(kserdes_force_signal_detect_high, sc);
	udelay(10);
}

static void kserdes_override_tap_offsets(struct kserdes_config *sc, u32 lane)
{
	u32 tap1val, tap2val, tap3val, tap4val, tap5val;
	void __iomem *sregs = sc->regs;
	u32 cmp, tap1_ofs;

	for (cmp = 1; cmp < MAX_COMPARATORS; cmp++) {
		/* adjust taps only for center comparators of
		 * of conparator 1 and 3
		 */
		if (!(cmp & 0x1))
			continue;

		/* set comparator number */
		FINSR(sregs, CML_REG(0x8c), 23, 21, cmp);

		/* read offsets */
		FINSR(sregs, CMU0_REG(0x8), 31, 24, ((lane + 1) << 5) + 0x12);
		tap1_ofs = (_kserdes_read_tbus_val(sregs) & 0x000f) << 3;

		FINSR(sregs, CMU0_REG(0x8), 31, 24, ((lane + 1) << 5) + 0x13);
		tap1_ofs |= (_kserdes_read_tbus_val(sregs) & 0x0e00) >> 9;

		tap1val = tap1_ofs - 14;
		tap2val = 31;
		tap3val = 31;
		tap4val = 31;
		tap5val = 31;

		/* set dfe_shadow_lane_sel */
		FINSR(sregs, CML_REG(0xf0), 27, 26, lane + 1);
		/* Set rxeq_ovr_en to 0x1 */
		FINSR(sregs, LANEX_REG(lane, 0x2c), 2, 2, 0x1);
		/* set rxeq_dfe_cmp_sel_ovr to comp_no */
		FINSR(sregs, LANEX_REG(lane, 0x30), 7, 5, cmp);
		/* set dfe_tap_ovr_en to 1 */
		FINSR(sregs, LANEX_REG(lane, 0x5c), 31, 31, 0x1);

		/* set tap overrides */
		FINSR(sregs, LANEX_REG(lane, 0x58), 30, 24, tap1val);
		FINSR(sregs, LANEX_REG(lane, 0x5c),  6,  0, tap2val);
		FINSR(sregs, LANEX_REG(lane, 0x5c), 13,  8, tap3val);
		FINSR(sregs, LANEX_REG(lane, 0x5c), 21, 16, tap4val);
		FINSR(sregs, LANEX_REG(lane, 0x5c), 29, 24, tap5val);

		/* set rxeq_ovr_latch_o = 0x1 */
		FINSR(sregs, LANEX_REG(lane, 0x2c), 10, 10, 0x1);
		/* set rxeq_ovr_latch_o = 0x0 */
		FINSR(sregs, LANEX_REG(lane, 0x2c), 10, 10, 0x0);

		/* set rxeq_ovr_en to 0 */
		FINSR(sregs, LANEX_REG(lane, 0x2c), 2, 2, 0x0);
		/* set dfe_tap_ovr_en to 0 */
		FINSR(sregs, LANEX_REG(lane, 0x5c), 31, 31, 0x0);

		/* This part of code will latch in offsets to
		 * tap adaptation logic so that if adaptation
		 * occurs, it will pick these offsets
		 */
		/* enable overrides */
		FINSR(sregs, LANEX_REG(lane, 0x58), 16, 16, 0x1);
		FINSR(sregs, LANEX_REG(lane, 0x48), 16, 16, 0x1);

		/* set gcfsm_cmp_sel to comp_no */
		FINSR(sregs, LANEX_REG(lane, 0x4c), 5, 2, (0x1 << (cmp - 1)));
		/* enable tap offset calibrate */
		FINSR(sregs, LANEX_REG(lane, 0x58), 17, 17, 0x1);

		/* enable taps */
		_kserdes_override_tap_offset_cdfe(sregs, lane, 1, 7, tap1val);
		_kserdes_override_tap_offset_cdfe(sregs, lane, 2, 6, tap2val);
		_kserdes_override_tap_offset_cdfe(sregs, lane, 3, 6, tap3val);
		_kserdes_override_tap_offset_cdfe(sregs, lane, 4, 6, tap4val);
		_kserdes_override_tap_offset_cdfe(sregs, lane, 5, 6, tap5val);

		/* Disable overrides */
		FINSR(sregs, LANEX_REG(lane, 0x58), 16, 16, 0x0);
		FINSR(sregs, LANEX_REG(lane, 0x48), 16, 16, 0x0);
		FINSR(sregs, LANEX_REG(lane, 0x58), 17, 17, 0x0);
	}
}

static int kserdes_wait_lane_rx_valid(struct kserdes_config *sc, u32 lane)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(500);
	u32 status;

	do {
		status = _kserdes_read_select_tbus(sc->regs, lane + 1, 0x02);

		if (status & 0x20)
			return 0;

		if (time_after(jiffies, timeout))
			return -ETIMEDOUT;

		cpu_relax();
	} while (true);
}

static int kserdes_att_boost_phya_macro_patch(struct kserdes_config *sc)
{
	u32 i, att_read[KSERDES_MAX_LANES], att_start[KSERDES_MAX_LANES];
	int ret;

	/* First save a copy of initial att start value */
	for (i = 0; i < sc->lanes; i++) {
		att_start[i] = kserdes_readl(sc->regs, LANEX_REG(i, 0x8c));
		att_start[i] = (att_start[i] >> 8) & 0xf;
	}
	/* Get att and fix this as start value.  Turn off att adaptation and
	 * do boost readaptation
	 */
	for (i = 0; i < sc->lanes; i++) {
		att_read[i] = _kserdes_read_select_tbus(sc->regs, i + 1,
				(sc->phy_type == KSERDES_PHY_XGE) ?
				 0x10 : 0x11);
		att_read[i] = (att_read[i] >> 4) & 0xf;
	}
	for (i = 0; i < sc->lanes; i++) {
		/* att start */
		FINSR(sc->regs, LANEX_REG(i, 0x8c), 11, 8, att_read[i]);
	}
	/* clear att init calibration */
	FINSR(sc->regs, CML_REG(0x84), 0, 0, 0x0);
	/* clear att re-calibration */
	FINSR(sc->regs, CML_REG(0x8c), 24, 24, 0x0);

	/* force calibration on all lanes */
	/* set att continuous recal */
	FINSR(sc->regs, CML_REG(0x98), 7, 7, 0x1);
	/* clear att continuous recal */
	FINSR(sc->regs, CML_REG(0x98), 7, 7, 0x0);
	udelay(300);

	/* check rx valid */
	for_each_enable_lane_return(kserdes_wait_lane_rx_valid, sc, ret);
	if (ret) {
		ks_err("kserdes_wait_lane_rx_valid FAILED %d\n", ret);
		return ret;
	}

	/* write back initial att start value */
	for (i = 0; i < sc->lanes; i++)
		FINSR(sc->regs, LANEX_REG(i, 0x8c), 11, 8, att_start[i]);

	/* turn att adaptation back on */
	FINSR(sc->regs, CML_REG(0x84),  0,  0, 0x1);
	FINSR(sc->regs, CML_REG(0x8c), 24, 24, 0x1);

	return 0;
}

static int kserdes_att_boost_phya_lane_patch(
			struct kserdes_config *sc, u32 lane)
{
	u32 boost_read;
	int ret;

	/* check lane rx valid */
	ret = kserdes_wait_lane_rx_valid(sc, lane);
	if (ret) {
		ks_err("kserdes_wait_lane_rx_valid FAILED %d\n", ret);
		return ret;
	}

	/* check boost value */
	boost_read = _kserdes_read_select_tbus(sc->regs, lane + 1,
			(sc->phy_type == KSERDES_PHY_XGE) ? 0x10 : 0x11);
	boost_read = (boost_read >> 8) & 0xf;

	/* increment boost by 1 if it's 0 */
	if (!boost_read) {
		/* Set rxeq_ovr_en to 1 */
		FINSR(sc->regs, LANEX_REG(lane, 0x2c),  2,  2, 0x1);
		/* set rxeq_ovr_load_en for boost only */
		FINSR(sc->regs, LANEX_REG(lane, 0x2c), 18, 12, 0x2);
		/* set rxeq_ovr_load for a value of 1 */
		FINSR(sc->regs, LANEX_REG(lane, 0x2c),  9,  3, 0x1);
		/* latch in new boost value */
		FINSR(sc->regs, LANEX_REG(lane, 0x2c), 10, 10, 0x1);
		FINSR(sc->regs, LANEX_REG(lane, 0x2c), 10, 10, 0x0);
		/* reset previous registers */
		FINSR(sc->regs, LANEX_REG(lane, 0x2c),  2,  2, 0x0);
		FINSR(sc->regs, LANEX_REG(lane, 0x2c), 18, 12, 0x0);
		FINSR(sc->regs, LANEX_REG(lane, 0x2c),  9,  3, 0x0);
	}
	return 0;
}

static inline void kserdes_att_boost_phya_patch(struct kserdes_config *sc)
{
	kserdes_att_boost_phya_macro_patch(sc);

	for_each_enable_lane(kserdes_att_boost_phya_lane_patch, sc);
}

static void kserdes_att_boost_phyb_lane_patch(
			struct kserdes_config *sc, u32 lane)
{
	u32 tbus_ofs, rxeq_init_reg_ofs, rxeq_ln_reg_ofs, rxeq_ln_force_bit;
	void __iomem *sregs = sc->regs;
	u32 att_start, att_read, boost_read;
	int ret;

	/* some setups */
	if (sc->phy_type == KSERDES_PHY_XGE) {
		tbus_ofs = 0x10;
		rxeq_init_reg_ofs = 0x9c;
		rxeq_ln_reg_ofs = 0x98;
		rxeq_ln_force_bit = 14;
	} else {
		tbus_ofs = 0x11;
		rxeq_init_reg_ofs = 0x84;
		rxeq_ln_reg_ofs = 0xac;
		rxeq_ln_force_bit = 11;
	}

	/* First save a copy of initial att start value */
	att_start = kserdes_readl(sregs, LANEX_REG(lane, 0x8c));
	att_start = (att_start >> 8) & 0xf;

	/* Get att and fix this as start value.  Turn off att adaptation and
	 * do boost readaptation
	 */
	att_read = _kserdes_read_select_tbus(sregs, lane + 1, tbus_ofs);
	att_read = (att_read >> 4) & 0xf;

	/* att start */
	FINSR(sregs, LANEX_REG(lane, 0x8c), 11, 8, att_read);
	/* clear att init calibration */
	FINSR(sregs, LANEX_REG(lane, rxeq_init_reg_ofs), 0, 0, 0x0);
	/* clear att re-calibration */
	FINSR(sregs, CML_REG(0x8c), 24, 24, 0x0);

	/* force calibration */
	FINSR(sregs, LANEX_REG(lane, rxeq_ln_reg_ofs),
		rxeq_ln_force_bit, rxeq_ln_force_bit, 0x1);
	FINSR(sregs, LANEX_REG(lane, rxeq_ln_reg_ofs),
		rxeq_ln_force_bit, rxeq_ln_force_bit, 0x0);

	/* check lane rx valid */
	ret = kserdes_wait_lane_rx_valid(sc, lane);
	if (ret) {
		ks_err("kserdes_wait_lane_rx_valid %d FAILED: %d\n",
			lane, ret);
	}
	udelay(300);

	/* check boost value */
	boost_read = _kserdes_read_select_tbus(sregs, lane + 1, tbus_ofs);
	boost_read = (boost_read >> 8) & 0xf;

	/* increment boost by 1 if it's 0 */
	if (!boost_read) {
		/* Set rxeq_ovr_en to 1 */
		FINSR(sregs, LANEX_REG(lane, 0x2c),  2,  2, 0x1);
		/* set rxeq_ovr_load_en for boost only */
		FINSR(sregs, LANEX_REG(lane, 0x2c), 18, 12, 0x2);
		/* set rxeq_ovr_load for a value of 1 */
		FINSR(sregs, LANEX_REG(lane, 0x2c),  9,  3, 0x1);
		/* latch in new boost value */
		FINSR(sregs, LANEX_REG(lane, 0x2c), 10, 10, 0x1);
		FINSR(sregs, LANEX_REG(lane, 0x2c), 10, 10, 0x0);
		/* reset previous registers */
		FINSR(sregs, LANEX_REG(lane, 0x2c),  2,  2, 0x0);
		FINSR(sregs, LANEX_REG(lane, 0x2c), 18, 12, 0x0);
		FINSR(sregs, LANEX_REG(lane, 0x2c),  9,  3, 0x0);
	}

	/* write back initial att start value */
	FINSR(sregs, LANEX_REG(lane, 0x8c), 11, 8, att_start);
	/* turn att adaptation back on */
	FINSR(sregs, LANEX_REG(lane, rxeq_init_reg_ofs), 0, 0, 0x1);
	FINSR(sregs, CML_REG(0x8c), 24, 24, 0x1);
}

static inline void kserdes_att_boost_phyb_patch(
			struct kserdes_config *sc, u32 lane)
{
	kserdes_att_boost_phyb_lane_patch(sc, lane);
}

static void kserdes_att_boost_phy_patch(struct kserdes_config *sc)
{
	if (sc->phy_type != KSERDES_PHY_XGE)
		kserdes_att_boost_phya_patch(sc);
	else
		for_each_lane(kserdes_att_boost_phyb_patch, sc);
}

static void kserdes_dlev_patch(struct kserdes_config *sc, u32 lane)
{
}

int kserdes_sgmii_lanes_enable(struct kserdes_config *sc)
{
	struct kserdes_offsets sofs;
	int ret, i;
	u32 val, lanes_enable = 0;

	for (i = 0; i < sc->lanes; i++) {
		if (!LANE_ENABLE(sc, i))
			continue;

		lanes_enable |= (1 << i);
	}

	/* configure Tap 1 if PHY-A and link rate greater than 8Gbaud */
	if (sc->phy_type != KSERDES_PHY_XGE) {
		if (sc->link_rate >= KSERDES_LINK_RATE_9P8304G)
			kserdes_tap1_patch(sc);
	}

	/* disable transmitter on all lanes to prevent
	 * receiver from adapting
	 */
	for_each_lane(kserdes_set_tx_idle, sc);

	/* apply patch for link rates greater than 8Gbaud */
	kserdes_phy_patch(sc);

	/* write boost training pattern for Hyperlink functional mode */
	if (sc->phy_type == KSERDES_PHY_HYPERLINK)
		_kserdes_set_training_pattern(sc->regs);

	/* assert serdes reset */
	kserdes_assert_reset(sc);

	/* apply the TX and RX FIR coefficients to the lanes */
	for_each_enable_lane(kserdes_set_tx_rx_fir_coeff, sc);

	/* force Signal Detect Low. This resets the CDR,
	 * Attenuation and Boost circuitry
	 */
	for_each_enable_lane(kserdes_force_signal_detect_low, sc);

	ret = kserdes_deassert_reset(sc, 0);
	if (ret) {
		ks_err("kserdes_deassert_reset FAILED %d\n", ret);
		return ret;
	}

	/* allow signal detect enable */
	for_each_enable_lane(kserdes_set_lane_rate, sc);

	_kserdes_pll_enable(sc->regs);

	ret = kserdes_get_status(sc);
	if (ret) {
		ks_err("kserdes_get_status FAILED %d\n", ret);
		return ret;
	}

	/* get tx termination on lane 0 */
	val = _kserdes_get_tx_termination(sc->regs, sc->phy_type, 0);

	/* apply tx termination to all lanes */
	kserdes_set_tx_terminations(sc, val);

	if (sc->phy_type == KSERDES_PHY_XGE) {
		/* perform offset averaging for phy-b only */
		kserdes_dfe_offset_calibration(sc, &sofs);
	} else if (sc->link_rate >= KSERDES_LINK_RATE_9P8304G) {
		/* manually adjust Tap 1 value for phy-a > 8GBaud */
		for_each_enable_lane(kserdes_override_tap_offsets, sc);
	}

	/* We are always in FUNCTIONAL mode, so we always
	 * continue to the following.
	 */
	/* enable transmitter on all lanes */
	for_each_enable_lane(kserdes_clr_tx_idle, sc);

	/* allow Signal Detect Enable */
	for_each_enable_lane(kserdes_force_signal_detect_high, sc);

	/* Wait for RX Valid on all lanes */
	for_each_enable_lane_return(kserdes_wait_lane_rx_valid, sc, ret);
	if (ret) {
		ks_err("kserdes_wait_lane_rx_valid FAILED %d\n", ret);
		return ret;
	}

	/* Apply Attenuation and Boost Patch if rx force
	 * flag is set
	 */
	if (!sc->rx_force_enable)
		kserdes_att_boost_phy_patch(sc);

	/* Apply DLEV workaround for PHY-B only */
	if (sc->phy_type == KSERDES_PHY_XGE)
		for_each_enable_lane(kserdes_dlev_patch, sc);

	/* If needed, check for errors or see if DLPF is
	 * railing and toggle signal detect
	 */
	/* CSL_Serdes_CDR_Reset(); */

	/* Enable MAC RX to allow MAC to take control */
	_kserdes_clear_wait_after(sc->regs);

	return lanes_enable;
}

static int kserdes_sgmii_init(struct kserdes_config *sc)
{
	if ((sc->clk_rate != KSERDES_CLOCK_RATE_156P25M) ||
		(sc->link_rate != KSERDES_LINK_RATE_1P25G))
		return -EINVAL;

	/* Disable MAC RX to avoid MAC taking control of Serdes */
	_kserdes_set_wait_after(sc->regs);

	/* Disable PLL and all lanes before loading config */
	_kserdes_pll_disable(sc->regs);
	for_each_lane(kserdes_lane_disable, sc);

	_kserdes_cfg_156p25mhz_10bit_5gbps(sc->regs);

	return 0;
}

static inline void _kserdes_txb_clk_mode(void __iomem *sregs, u32 link_rate)
{
	if (link_rate != KSERDES_LINK_RATE_1P25G)
		return;

	kserdes_writel(sregs, CPU_CTRL_REG, 0x20000000);
	kserdes_writel(sregs, PLL_CTRL_REG, 0x00380000);
	kserdes_writel(sregs, CPU_CTRL_REG, 0x00000000);
}

static inline void _kserdes_set_link_loss_wait(void __iomem *sregs,
					       u32 link_loss_wait)
{
	kserdes_writel(sregs, LINK_LOSS_WAIT_REG, link_loss_wait);
}

static inline void _kserdes_reset(void __iomem *sregs)
{
	/* Toggle POR_EN bit */
	FINSR(sregs, CPU_CTRL_REG, 29, 29, 0x1);
	udelay(10);
	FINSR(sregs, CPU_CTRL_REG, 29, 29, 0x0);
	udelay(10);
}

static inline void kserdes_cfg_phyb_10p3125g_16bit_lane(
			struct kserdes_config *sc, u32 lane)
{
	kserdes_do_config(sc->regs + LANEX_SS_OFFSET(lane),
			cfg_phyb_10p3125g_16bit_lane,
			ARRAY_SIZE(cfg_phyb_10p3125g_16bit_lane));

	/* disable auto negotiation*/
	FINSR(sc->regs, LANEX_REG(lane, 0x180), 4, 4, 0x0);

	/* disable link training */
	FINSR(sc->regs, LANEX_REG(lane, 0x1c0), 9, 9, 0x0);
}

static inline void kserdes_cfg_phyb_aneg_lane(
		struct kserdes_config *sc, u32 lane)
{
	kserdes_do_config(sc->regs + LANEX_SS_OFFSET(lane),
			cfg_phyb_aneg_lane,
			ARRAY_SIZE(cfg_phyb_aneg_lane));

	/* disable auto negotiation*/
	FINSR(sc->regs, LANEX_REG(lane, 0x180), 4, 4, 0x0);
}

static inline void _kserdes_cfg_phyb_10p3125g_comlane(void __iomem *sregs)
{
	kserdes_do_config(sregs, cfg_phyb_10p3125g_comlane,
		ARRAY_SIZE(cfg_phyb_10p3125g_comlane));
}

static inline void _kserdes_cfg_phyb_1p25g_156p25mhz_comlane(
						void __iomem *sregs)
{
	kserdes_do_config(sregs, cfg_phyb_1p25g_156p25mhz_comlane,
		ARRAY_SIZE(cfg_phyb_1p25g_156p25mhz_comlane));
}

static inline void kserdes_xge_pll_enable(struct kserdes_config *sc)
{
	/* phyb reset clear */
	if (!sc->firmware)
		FINSR(sc->regs, CML_REG(0), 7, 0, 0x1f);

	if (sc->link_rate == KSERDES_LINK_RATE_10P3125G) {
		_kserdes_pll_enable(sc->regs);
		_kserdes_pll2_enable(sc->regs);
	} else if (sc->link_rate == KSERDES_LINK_RATE_1P25G)
		kserdes_writel(sc->regs, PLL_CTRL_REG, 0xe0000000);
}

static inline void _kserdes_xge_pll_disable(void __iomem *sregs)
{
	_kserdes_pll_disable(sregs);
	_kserdes_pll2_disable(sregs);
}

static inline void _kserdes_xge_enable_pcs(void __iomem *sregs, u32 lane)
{
	/* set bus-width to 16 bit mode */
	FINSR(sregs, LANE_CTRL_STS_REG(lane), 23, 21, 0x7);
	FINSR(sregs, LANE_CTRL_STS_REG(lane),  5,  3, 0x7);

	/* enable PCS overlay and lane select 10GKR */
	FINSR(sregs, LANE_CTRL_STS_REG(lane), 16, 16, 0x1);
	FINSR(sregs, LANE_CTRL_STS_REG(lane), 19, 19, 0x1);
}

static inline void kserdes_xge_lane_enable(struct kserdes_config *sc, u32 lane)
{
	u32 lane_ctrl_rate = sc->lane[lane].ctrl_rate;

	/* Set Lane Control Rate */
	if (sc->link_rate == KSERDES_LINK_RATE_10P3125G)
		_kserdes_set_lane_ctrl_rate(sc->regs, lane, lane_ctrl_rate);
	else if (sc->link_rate == KSERDES_LINK_RATE_1P25G)
		kserdes_writel(sc->regs, LANE_CTRL_STS_REG(lane), 0xf800f8c0);

	_kserdes_xge_enable_pcs(sc->regs, lane);

	_kserdes_lane_enable(sc->regs, lane);

	if (sc->lane[lane].loopback)
		_kserdes_set_lane_loopback(sc->regs, lane, sc->link_rate);
}

static inline void _kserdes_enable_xgmii_port_select(void __iomem *sw_regs,
						     u32 port_selects)
{
	kserdes_writel(sw_regs, XGE_CTRL_OFFSET, port_selects);
}

static inline void _kserdes_enable_xgmii_port(void __iomem *sw_regs, u32 port)
{
	FINSR(sw_regs, XGE_CTRL_OFFSET, port, port, 0x1);
}

static inline void _kserdes_reset_cdr(void __iomem *sregs, int lane)
{
	/* toggle signal detect */
	_kserdes_force_signal_detect_low(sregs, lane);
	mdelay(1);
	_kserdes_force_signal_detect_high(sregs, lane);
}

/* Call every 10 ms */
static int _kserdes_check_link_status(
			void __iomem	*sregs,
			void __iomem	*sw_regs,
			u32		lanes,
			u32		lanes_enable,
			u32		*current_state,
			u32		*lane_down)
{
	u32 pcsr_rx_stat, blk_lock, blk_errs;
	int loss, i, status = 1;

	for (i = 0; i < lanes; i++) {
		if (!(lanes_enable & (1 << i)))
			continue;

		/* Rx Signal Loss bit in serdes lane control and status reg*/
		loss = (kserdes_readl(sregs, LANE_CTRL_STS_REG(i))) & 0x01;

		/* Block Errors and Block Lock bits in PCSR rx status reg */
		pcsr_rx_stat = kserdes_readl(sw_regs, PCSR_RX_STATUS(i));
		blk_lock = (pcsr_rx_stat >> 30) & 0x1;
		blk_errs = (pcsr_rx_stat >> 16) & 0x0ff;

		/* If Block error, attempt recovery! */
		if (blk_errs)
			blk_lock = 0;

		switch (current_state[i]) {
		case 0:
			/* if good link lock the signal detect ON! */
			if (!loss && blk_lock) {
				ks_debug("XGE PCSR Linked Lane: %d\n", i);
				FINSR(sregs, LANEX_REG(i, 0x04), 2, 1, 0x3);
				current_state[i] = 1;
			} else {
				/* if no lock, then reset CDR
				 * by toggling sig detect
				 */
				if (!blk_lock) {
					ks_debug("XGE PCSR Recover Lane: %d\n",
						i);

					_kserdes_reset_cdr(sregs, i);
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
				_kserdes_reset_cdr(sregs, i);

				current_state[i] = 0;
			}
			break;
		default:
			ks_info("XGE: unknown current_state[%d] %d\n",
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

static int _kserdes_wait_link_up(void __iomem *sregs,
				 void __iomem *sw_regs,
				 u32 lanes, u32 lanes_enable)
{
	u32 current_state[KSERDES_MAX_LANES];
	int retries = 0, link_up;
	u32 lane_down[KSERDES_MAX_LANES];
	int i;

	if (lanes > KSERDES_MAX_LANES)
		return -EINVAL;

	memset(current_state, 0, sizeof(current_state));
	memset(lane_down, 0, sizeof(lane_down));

	do {
		mdelay(10);
		memset(lane_down, 0, sizeof(lane_down));

		link_up = _kserdes_check_link_status(sregs, sw_regs,
				lanes, lanes_enable, current_state, lane_down);

		/* if we did not get link up then wait 100ms
		   before calling it again
		 */
		if (link_up)
			break;

		for (i = 0; i < lanes; i++) {
			if ((lanes_enable & (1 << i)) && lane_down[i])
				ks_debug("XGE: detected lane down on lane %d\n",
					 i);
		}

		if (++retries > 100) {
			ks_err("XGE: timeout waiting for serdes link up\n");
			return -ETIMEDOUT;
		}
	} while (!link_up);

	ks_info("XGE: serdes link up: retried %d times\n", retries);
	return 0;
}

static int kserdes_xge_lanes_enable(struct kserdes_config *sc)
{
	struct kserdes_offsets sofs;
	int ret, i;
	u32 lanes_enable = 0;

	if (sc->firmware) {
		/* firmware started in serdes_init and
		 * doesn't need lanes enable
		 */
		return 0;
	}

	for (i = 0; i < sc->lanes; i++) {
		if (!LANE_ENABLE(sc, i))
			continue;

		lanes_enable |= (1 << i);
	}

	kserdes_phy_patch(sc);
	kserdes_xge_pll_enable(sc);

	for_each_lane(kserdes_xge_lane_enable, sc);

	ret = kserdes_get_status(sc);
	if (ret) {
		ks_err("kserdes_xge_lanes_enable get status FAILED %d\n", ret);
		return ret;
	}

	kserdes_dfe_offset_calibration(sc, &sofs);

	for (i = 0; i < sc->lanes; i++)
		_kserdes_enable_xgmii_port(sc->sw_regs, i);

	_kserdes_wait_link_up(sc->regs, sc->sw_regs, sc->lanes, lanes_enable);

	return lanes_enable;
}

static inline void kserdes_xfw_get_lane_params(struct kserdes_config *sc,
					       int lane)
{
	struct kserdes_fw_config *fw = &sc->fw;
	u32 tx_ctrl, val_0, val_1;
	u32 phy_a = PHY_A(sc->regs);

	val_0 = kserdes_readl(sc->regs, LANEX_REG(lane, 0x04));
	val_1 = kserdes_readl(sc->regs, LANEX_REG(lane, 0x08));

	tx_ctrl = ((((val_0 >> 18) & 0x1)    << 24) |	/* TX_CTRL_O_24 */
		   (((val_1 >> 0)  & 0xffff) <<  8) |	/* TX_CTRL_O_23_8 */
		   (((val_0 >> 24) & 0xff)   <<  0));	/* TX_CTRL_O_7_0 */

	if (phy_a) {
		fw->cm = (val_1 >> 12) & 0xf;
		fw->c1 = (val_1 >> 0) & 0x1f;
		fw->c2 = (val_1 >> 8) & 0xf;
	} else {
		fw->cm = (tx_ctrl >> 16) & 0xf;
		fw->c1 = (tx_ctrl >> 8) & 0x1f;
		fw->c2 = (tx_ctrl >> 13) & 0x7;
		fw->c2 = fw->c2 | (((tx_ctrl >> 24) & 0x1) << 3);
	}

	val_0 = _kserdes_read_select_tbus(sc->regs, lane + 1,
					(phy_a ? 0x11 : 0x10));
	fw->attn = (val_0 >> 4) & 0xf;
	fw->boost = (val_0 >> 8) & 0xf;

	val_0 = _kserdes_read_select_tbus(sc->regs, lane + 1, 0x5);
	fw->dlpf = (val_0 >> 2) & 0x3ff;

	val_0 = _kserdes_read_select_tbus(sc->regs, lane + 1, 0x6);
	fw->cdrcal = (val_0 >> 3) & 0xff;
}

static inline void kserdes_xfw_mem_init(struct kserdes_config *sc)
{
	struct kserdes_fw_config *fw = &sc->fw;
	u32 i, lane_config = 0, lanes = sc->lanes;

	for (i = 0; i < lanes; i++)
		lane_config = (lane_config << 8) |
			(fw->lane_config[i] & 0xff);

	lane_config <<= 8;

	/* initialize 64B data mem */
	kserdes_writel(sc->regs, MEM_ADR_REG, 0x0000ffc0);

	for (i = 0; i < 11; i++)
		kserdes_writel(sc->regs, MEM_DATINC_REG, 0x00000000);

	/* Flush 64 bytes 10,11,12,13 */
	kserdes_writel(sc->regs, MEM_DATINC_REG, 0x00009C9C);

	/* fast train */
	kserdes_writel(sc->regs, MEM_DATINC_REG, fw->fast_train);

	kserdes_writel(sc->regs, MEM_DATINC_REG, 0x00000000);
	/* lane seeds */
	kserdes_writel(sc->regs, MEM_DATINC_REG, fw->lane_seeds);
	/* lane config */
	kserdes_writel(sc->regs, MEM_DATINC_REG, lane_config);
}

static void _kserdes_xfw_check_download(void __iomem *sregs)
{
	struct kserdes_fw_entry *ent = &(kserdes_firmware[0]);
	int a_size, i;
	u32 val, addr;

	a_size = ARRAY_SIZE(kserdes_firmware);

	for (i = 0; i < a_size; i++, ent++) {
		if (ent->reg_ofs == MEM_ADR_REG)
			kserdes_writel(sregs, MEM_ADR_REG, ent->data);
		else if (ent->reg_ofs == MEM_DATINC_REG) {
			addr = kserdes_readl(sregs, MEM_ADR_REG);
			val  = kserdes_readl(sregs, MEM_DATINC_REG);
			if (val != ent->data) {
				ks_err("diff@ %d 0x%08x: 0x%08x 0x%08x\n",
					i, addr, ent->data, val);
			}
		} else
			ks_err("unknown reg_ofs %08x\n", ent->reg_ofs);
	}
}

static void _kserdes_xfw_download(void __iomem *sregs)
{
	struct kserdes_fw_entry *ent = &(kserdes_firmware[0]);
	int a_size, i;

	a_size = ARRAY_SIZE(kserdes_firmware);

	for (i = 0; i < a_size; i++, ent++)
		kserdes_writel(sregs, ent->reg_ofs, ent->data);
}

static inline void _kserdes_xfw_restart_cpu(void __iomem *sregs)
{
	u32 val;

	/* place serdes in reset and allow cpu to access regs */
	val = (POR_EN | CPUREG_EN | AUTONEG_CTL | DATASPLIT);
	kserdes_writel(sregs, CPU_CTRL_REG, val);

	/* let reset propagate to uC */
	mdelay(50);

	val &= ~POR_EN;
	kserdes_writel(sregs, CPU_CTRL_REG, val);

	/* set VCO div to match firmware */
	FINSR(sregs, CMU0_REG(0x0), 23, 16, 0x80);
	/* override CMU1 pin reset */
	FINSR(sregs, CMU1_REG(0x10), 31, 24, 0x40);

	/* kick off cpu */
	val |= (CPU_EN | CPU_GO);
	kserdes_writel(sregs, CPU_CTRL_REG, val);
}

static inline void kserdes_xfw_get_params(struct kserdes_config *sc)
{
	struct kserdes_fw_config *fw = &sc->fw;
	u32 val, val_1, i, lanes = sc->lanes;

	val = kserdes_readl(sc->regs, PLL_CTRL_REG);
	kserdes_writel(sc->regs, MEM_ADR_REG, 0x0000ffeb);
	val_1 = kserdes_readl(sc->regs, MEM_DAT_REG);

	ks_info("Initialized KR firmware version: %x\n", val_1);
	ks_info("firmware restarted status:\n");
	ks_info("  pll_ctrl = 0x%08x", val);

	for (i = 0; i < lanes; i++) {
		if (!(BIT(i) & fw->active_lane))
			continue;

		val = kserdes_readl(sc->sw_regs, PCSR_RX_STATUS(i));
		val_1 = kserdes_readl(sc->regs, LANE_CTRL_STS_REG(i));

		/* get FW adaptation parameters from phy */
		kserdes_xfw_get_lane_params(sc, i);
		ks_info("LANE%d:\n", i);
		ks_info("  pcsr_rx_sts = 0x%08x, lane_ctrl_sts = 0x%08x\n",
				val, val_1);
		ks_info("  cm = %d, c1 = %d, c2 = %d\n",
				fw->cm, fw->c1, fw->c2);
		ks_info("  attn = %d, boost = %d, dlpf = %d, cdrcal = %d\n",
				fw->attn, fw->boost, fw->dlpf, fw->cdrcal);
	}
}

static void kserdes_xfw_auto_neg_status(struct kserdes_config *sc)
{
	struct kserdes_fw_config *fw = &sc->fw;
	u32 aneg_in_prog = 0, i, aneg_ctl, tmp;
	unsigned long timeout;

	/* set aneg_in_prog to lane(s) that is/are active,
	 * set to auto-negotiate, and on the 1G/10G rate.
	 */
	for (i = 0; i < sc->lanes; i++) {
		tmp = (fw->lane_config[i] & ANEG_1G_10G_OPT_MASK);

		if ((tmp == ANEG_1G_10G_OPT_MASK) &&
		    (fw->active_lane & BIT(i)))
			aneg_in_prog |= BIT(i);
	}

	if (aneg_in_prog == 0)
		return;

	timeout = jiffies + msecs_to_jiffies(5000);

	ks_info("Waiting for autonegotiated link up.\n");

	while (aneg_in_prog) {
		for (i = 0; i < sc->lanes; i++) {
			aneg_ctl = kserdes_readl(sc->regs, LANEX_REG(i, 0x1d8));
			aneg_ctl = (aneg_ctl & ANEG_LINK_CTL_1G10G_MASK);

			if ((aneg_ctl == ANEG_LINK_CTL_10GKR_MASK) ||
			    (aneg_ctl == ANEG_LINK_CTL_1GKX_MASK))
				aneg_in_prog &= ~BIT(i);
		}
		if (time_after(jiffies, timeout))
			break;
		cpu_relax();
	}

	ks_debug("Lanes auto neg completed (mask): 0x%x\n",
		~aneg_in_prog & fw->active_lane);
}

static int kserdes_xfw_get_status(struct kserdes_config *sc)
{
	u32 lanes_ok = 1;
	int i;

	for (i = 0; i < sc->lanes; i++) {
		lanes_ok &= _kserdes_get_lane_status(sc->regs, i, sc->phy_type);
		cpu_relax();
	}

	return lanes_ok;
}

static int kserdes_xfw_wait_pll_locked(struct kserdes_config *sc)
{
	unsigned long timeout;
	int ret = 0;
	u32 status;

	timeout = jiffies + msecs_to_jiffies(500);
	do {
		status = kserdes_xfw_get_status(sc);

		if (status)
			return 0;

		if (time_after(jiffies, timeout)) {
			ret = -ETIMEDOUT;
			break;
		}
		cpu_relax();
	} while (true);

	ks_info("XGE serdes not locked: time out.\n");
	return ret;
}

static int kserdes_xfw_start(struct kserdes_config *sc)
{
	struct kserdes_fw_config *fw = &sc->fw;
	u32 i;
	int ret = 0;

	_kserdes_pll_disable(sc->regs);
	_kserdes_pll2_disable(sc->regs);

	_kserdes_reset(sc->regs);

	for (i = 0; i < sc->lanes; i++)
		_kserdes_lane_enable(sc->regs, i);

	_kserdes_set_link_loss_wait(sc->regs, fw->link_loss_wait);

	kserdes_xge_pll_enable(sc);

	kserdes_xfw_mem_init(sc);

	_kserdes_xfw_download(sc->regs);

	_kserdes_xfw_restart_cpu(sc->regs);

	/* 10G Auto-Negotiation Handling:
	 * Wait to see if we can synchronize with other side.
	 * If it doesn't it may require an interface
	 * toggle after boot
	 */
	kserdes_xfw_auto_neg_status(sc);

	_kserdes_enable_xgmii_port_select(sc->sw_regs, MASK(sc->lanes - 1, 0));

	mdelay(100);

	ret = kserdes_xfw_wait_pll_locked(sc);
	if (ret) {
		_kserdes_xfw_check_download(sc->regs);
		return ret;
	}

	sc->fw.on = true;
	kserdes_xfw_get_params(sc);

	return ret;
}

static int kserdes_xfw_get_bindings(struct device_node *node,
				struct kserdes_config *sc)
{
	struct kserdes_fw_config *fw = &sc->fw;
	struct device_node *lane;
	u32 rate, lnum;

	/* Get random lane seeds */
	get_random_bytes(&fw->lane_seeds, sizeof(u32));
	fw->lane_seeds &= 0x00ffff00;

	fw->link_loss_wait = 20000;
	/* Flush 64 bytes 0c,0d,0e,0f FAST Train */
	fw->fast_train = 0x60000000;

	/* get lane configs via DTS */
	for_each_available_child_of_node(node, lane) {
		if (of_property_read_u32(lane, "lane", &lnum))
			lnum = 0;

		/* Set active lane(s) for polling */
		fw->active_lane |= BIT(lnum);

		/* get lane rate from DTS
		 * 0=1g/10g, 1=force 1g, 2=force 10g
		 */
		of_property_read_u32(lane, "rate", &rate);
		if (rate == 0) {
			fw->lane_config[lnum] |= BIT(6);
			fw->lane_config[lnum] |= BIT(5);
		} else if (rate == 1)
			fw->lane_config[lnum] |= BIT(5);
		else if (rate == 2)
			fw->lane_config[lnum] |= BIT(6);
		else {
			/* default to 1G/10G */
			ks_err("Invalid lane rate defined. Using 1/10G.\n");
			fw->lane_config[lnum] |= BIT(6);
			fw->lane_config[lnum] |= BIT(5);
		}

		/* get lane properties from DTS */
		if (of_find_property(lane, "autonegotiate", NULL))
			fw->lane_config[lnum] |= BIT(7);

		if (of_find_property(lane, "tx_pause", NULL))
			fw->lane_config[lnum] |= BIT(4);

		if (of_find_property(lane, "rx_pause", NULL))
			fw->lane_config[lnum] |= BIT(3);

		if (of_find_property(lane, "10g_train", NULL))
			fw->lane_config[lnum] |= BIT(2);

		if (of_find_property(lane, "fec", NULL))
			fw->lane_config[lnum] |= BIT(1);
	}

	if (fw->active_lane == 0) {
		ks_err("No active serdes firmware lanes defined.");
		return -EINVAL;
	}

	ks_debug("Active serdes fw lane(s): 0x%x", fw->active_lane);

	/* Both lanes should be configured even if one is not in use, just
	 * mirror the config over in that case.
	 */
	if (fw->active_lane == 0x1 || fw->active_lane == 0x2) {
		if (fw->lane_config[0] & 0xff)
			fw->lane_config[1] = fw->lane_config[0];
		else if (fw->lane_config[1] & 0xff)
			fw->lane_config[0] = fw->lane_config[1];
	}

	return 0;
}

static int kserdes_xge_init(struct kserdes_config *sc)
{
	if (sc->firmware) {
		if (sc->fw.on) {
			ks_info("serdes firmware already started\n");
			return 0;
		}

		return kserdes_xfw_start(sc);
	}

	if (sc->clk_rate != KSERDES_CLOCK_RATE_156P25M)
		return -EINVAL;

	_kserdes_reset(sc->regs);

	_kserdes_txb_clk_mode(sc->regs, sc->link_rate);
	_kserdes_xge_pll_disable(sc->regs);

	_kserdes_cfg_phyb_1p25g_156p25mhz_cmu0(sc->regs);
	_kserdes_cfg_phyb_10p3125g_156p25mhz_cmu1(sc->regs);

	if (sc->link_rate == KSERDES_LINK_RATE_10P3125G) {
		for_each_lane(kserdes_cfg_phyb_10p3125g_16bit_lane, sc);
		_kserdes_cfg_phyb_10p3125g_comlane(sc->regs);
	} else if (sc->link_rate == KSERDES_LINK_RATE_1P25G) {
		for_each_lane(kserdes_cfg_phyb_aneg_lane, sc);
		_kserdes_cfg_phyb_1p25g_156p25mhz_comlane(sc->regs);
	} else
		return -EINVAL;

	return 0;
}

static int kserdes_pcie_lanes_enable(struct kserdes_config *sc)
{
	int ret, i;
	u32 lanes_enable = 0;

	for (i = 0; i < sc->lanes; i++) {
		if (!LANE_ENABLE(sc, i))
			continue;

		lanes_enable |= (1 << i);
	}

	for (i = 0; i < sc->lanes; i++) {
		kserdes_release_reset(sc, i);

		if (sc->lane[i].loopback)
			_kserdes_set_lane_loopback(sc->regs, i, sc->link_rate);

		/* lane enable */
		FINSR(sc->sw_regs, 0x180c, 9, 8, i + 1);

		/* set lane rate */
		FINSR(sc->sw_regs, 0x180c, 17, 17, sc->lane[i].ctrl_rate);
	}

	ret = kserdes_get_status(sc);
	if (ret)
		return ret;
	else
		return lanes_enable;
}

static int kserdes_pcie_init(struct kserdes_config *sc)
{
	if ((sc->clk_rate != KSERDES_CLOCK_RATE_100M) ||
	    (sc->link_rate != KSERDES_LINK_RATE_5G))
		return -EINVAL;

	kserdes_do_config(sc->regs, cfg_100mhz_pci_5gbps,
			ARRAY_SIZE(cfg_100mhz_pci_5gbps));

	return 0;
}

static void kserdes_show_fw_config(struct kserdes_config *sc)
{
	struct kserdes_fw_config *fw = &sc->fw;

	ks_dump("fw configs:\n");
	ks_dump("  lane_configs: 0x%02x, 0x%02x\n",
		fw->lane_config[0], fw->lane_config[1]);
	ks_dump("  lnk_loss_wait: %d, lane_seeds: 0x%08x, fast_train: 0x%08x\n",
		fw->link_loss_wait, fw->lane_seeds, fw->fast_train);
}

static void kserdes_show_lane_config(struct kserdes_lane_config *lc)
{
	ks_dump("%s\n", (lc->enable ? "enable" : "disable"));
	ks_dump("ctrl_rate 0x%x\n", lc->ctrl_rate);
	ks_dump("rx_start %u %u\n",
		lc->rx_start.att, lc->rx_start.boost);
	ks_dump("rx_force %u %u\n",
		lc->rx_force.att, lc->rx_force.boost);
	ks_dump("tx_coeff %u %u %u %u %u\n",
		lc->tx_coeff.c1, lc->tx_coeff.c2, lc->tx_coeff.cm,
		lc->tx_coeff.att, lc->tx_coeff.vreg);
	ks_dump("loopback %u\n", lc->loopback);
}

static void kserdes_show_config(struct kserdes_config *sc)
{
	u32 i;

	if (!sc->debug)
		return;

	ks_dump("serdes regs 0x%p\n", sc->regs);
	if (sc->sw_regs)
		ks_dump("sw regs 0x%p\n", sc->sw_regs);
	ks_dump("clk_rate %u\n", sc->clk_rate);
	ks_dump("link_rate %u\n", sc->link_rate);
	ks_dump("phy_type %u\n", sc->phy_type);
	ks_dump("lanes %u\n", sc->lanes);

	if (sc->firmware) {
		kserdes_show_fw_config(sc);
		return;
	}

	ks_dump("rx-force-%s\n", (sc->rx_force_enable ? "enable" : "disable"));

	for (i = 0; i < sc->lanes; i++) {
		ks_dump("lane[%u]:\n", i);
		kserdes_show_lane_config(&sc->lane[i]);
	}
}

static int kserdes_get_lane_bindings(const char *dev,
				struct device_node *np,
				struct kserdes_lane_config *lc)
{
	struct kserdes_equalizer *eq;
	struct kserdes_tx_coeff *tc;

	if (of_find_property(np, "disable", NULL))
		lc->enable = 0;
	else
		lc->enable = 1;

	ks_debug("%s lane enable: %d\n", dev, lc->enable);

	if (of_property_read_u32(np, "control-rate", &lc->ctrl_rate)) {
		ks_info("%s use default lane control-rate: %u\n",
			dev, lc->ctrl_rate);
	}
	ks_debug("%s lane control-rate: %d\n", dev, lc->ctrl_rate);

	if (of_find_property(np, "loopback", NULL))
		lc->loopback = 1;
	else
		lc->loopback = 0;

	ks_debug("%s lane loopback: %d\n", dev, lc->loopback);

	eq = &lc->rx_start;
	if (of_property_read_u32_array(np, "rx-start", &eq->att, 2)) {
		ks_info("%s use default lane rx-start 0 0\n", dev);
		eq->att = 0;
		eq->boost = 0;
	}
	ks_debug("%s lane rx-start: %d %d\n", dev, eq->att, eq->boost);

	eq = &lc->rx_force;
	if (of_property_read_u32_array(np, "rx-force", &eq->att, 2)) {
		ks_info("%s use default lane rx-force 0 0\n", dev);
		eq->att = 0;
		eq->boost = 0;
	}
	ks_debug("%s lane rx-force: %d %d\n", dev, eq->att, eq->boost);

	tc = &lc->tx_coeff;
	if (of_property_read_u32_array(np, "tx-coeff", &tc->c1, 5)) {
		ks_info("%s use default tx-coeff 0\n", dev);
		tc->c1 = 0;
	}
	ks_debug("%s tx-coeff: %d %d %d %d %d\n", dev,
		tc->c1, tc->c2, tc->cm, tc->att, tc->vreg);

	return 0;
}

int kserdes_get_serdes_bindings(const char *dev,
				struct device_node *np,
				struct kserdes_config *sc)
{
	struct device_node *fw_node;
	struct device_node *lp;
	const char *phy_type;
	char name[16] = {'l', 'a', 'n', 'e'};
	u32 temp[2];
	int ret, i;

	if (of_property_read_u32_array(np, "regs", (u32 *)&(temp[0]), 2)) {
		ks_err("%s: No serdes regs defined\n", dev);
		return -ENODEV;
	}
	sc->regs = ioremap(temp[0], temp[1]);
	if (!sc->regs) {
		ks_err("%s: can't map serdes regs\n", dev);
		return -ENOMEM;
	}

	if (of_property_read_u32(np, "refclk-khz", &sc->clk_rate)) {
		ks_info("%s: use default refclk-khz: %u\n",
			dev, sc->clk_rate);
	}
	ks_debug("%s: ref-clock-rate %u\n", dev, sc->clk_rate);

	if (of_property_read_u32(np, "link-rate-kbps", &sc->link_rate)) {
		ks_info("%s: use default link-rate-kbps: %u\n",
			dev, sc->link_rate);
	}
	ks_debug("%s: link-rate-kbps %u\n", dev, sc->link_rate);

	ret = of_property_read_string(np, "phy-type", &phy_type);
	if (!ret) {
		if (0 == strcmp(phy_type, "sgmii"))
			sc->phy_type = KSERDES_PHY_SGMII;
		else if (0 == strcmp(phy_type, "xge"))
			sc->phy_type = KSERDES_PHY_XGE;
		else if (0 == strcmp(phy_type, "pcie"))
			sc->phy_type = KSERDES_PHY_PCIE;
		else
			return -EINVAL;
	}
	ks_debug("%s: phy-type %u\n", dev, sc->phy_type);

	if (of_property_read_u32(np, "max-lanes", &sc->lanes))
		ks_info("%s: use default max-lanes %d\n", dev,  sc->lanes);

	if (sc->lanes > KSERDES_MAX_LANES) {
		sc->lanes = KSERDES_MAX_LANES;
		ks_info("%s: use max allowed lanes %d\n", dev, sc->lanes);
	}
	ks_debug("%s: max-lanes %u\n", dev, sc->lanes);

	sc->debug = (of_get_property(np, "debug",  NULL) != NULL);

	/* check if firmware should be used */
	fw_node = of_get_child_by_name(np, "firmware");
	if (fw_node && of_device_is_available(fw_node)) {
		sc->firmware = 1;
		ret = kserdes_xfw_get_bindings(fw_node, sc);
		of_node_put(fw_node);
		return ret;
	}

	/* Not using firmware */
	if (of_find_property(np, "rx-force-enable", NULL))
		sc->rx_force_enable = 1;
	else
		sc->rx_force_enable = 0;

	ks_debug("%s: rx-force-enable: %d\n", dev, sc->rx_force_enable);

	for (i = 0; i < sc->lanes; i++) {
		sprintf(&name[4], "%d", i);
		lp = of_find_node_by_name(np, name);
		if (lp) {
			if (kserdes_get_lane_bindings(dev, lp, &sc->lane[i]))
				return -EINVAL;
		}
	}

	return 0;
}

int kserdes_lanes_enable(struct kserdes_config *sc)
{
	int ret = -EINVAL;

	if (sc->phy_type == KSERDES_PHY_SGMII)
		ret = kserdes_sgmii_lanes_enable(sc);
	else if (sc->phy_type == KSERDES_PHY_XGE)
		ret = kserdes_xge_lanes_enable(sc);
	else if (sc->phy_type == KSERDES_PHY_PCIE)
		ret = kserdes_pcie_lanes_enable(sc);

	return ret;
}

int kserdes_init(struct kserdes_config *sc)
{
	int ret = -EINVAL;

	kserdes_show_config(sc);

	if (sc->phy_type == KSERDES_PHY_SGMII)
		ret = kserdes_sgmii_init(sc);
	else if (sc->phy_type == KSERDES_PHY_XGE)
		ret = kserdes_xge_init(sc);
	else if (sc->phy_type == KSERDES_PHY_PCIE)
		ret = kserdes_pcie_init(sc);

	return ret;
}
