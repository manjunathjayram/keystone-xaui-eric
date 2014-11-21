/*
 * Copyright (C) 2014 Texas Instruments Incorporated
 * Authors: Aurelien Jacquiot <a-jacquiot@ti.com>
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
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/sysfs.h>

#include "keystone_rio_serdes.h"

#define for_each_lanes(lane_mask, lane)					\
	for_each_set_bit((lane),					\
			 (const long unsigned int *) &(lane_mask),	\
			 KEYSTONE_SERDES_MAX_LANES)

#define reg_rmw(addr, value, mask)					\
	__raw_writel(((__raw_readl(addr) & (~(mask))) | ((value) & (mask))), \
		     (addr))

#define reg_fmkr(msb, lsb, val) \
	(((val) & ((1 << ((msb) - (lsb) + 1)) - 1)) << (lsb))

#define reg_finsr(addr, msb, lsb, val)					\
	__raw_writel(((__raw_readl(addr)				\
		       & ~(((1 << ((msb) - (lsb) + 1)) - 1) << (lsb)))	\
		      | reg_fmkr(msb, lsb, val)), (addr))

static inline void reg_fill_field(void __iomem *regs, int offset, int width,
				  u32 value)
{
	u32 mask;
	int i;

	for (i = 0, mask = 0; i < width; i++)
		mask = (mask << 1) | 1;

	value &= mask;
	value <<= offset;
	mask  <<= offset;

	reg_rmw(regs, value, mask);
}

/*-------------------------- KeyStone 1 SerDes functions --------------------*/

struct k1_rio_serdes_regs {
	u32	pll;
	struct {
		u32	rx;
		u32	tx;
	} channel[4];
};

/*
 * Configure SerDes with appropriate baudrate
 *
 * Using mode 0: sRIO config 0: MPY = 5x, div rate = half,
 * link rate = 3.125 Gbps, mode 1x
 */
static int k1_rio_serdes_config_lanes(
	u32 lanes,
	u32 baud,
	struct keystone_serdes_data *serdes)
{
	u32 lane;
	int res = 0;
	u32 val;
	unsigned long timeout;
	struct k1_rio_serdes_regs *serdes_regs =
		(struct k1_rio_serdes_regs *) serdes->regs;
	struct device *dev = serdes->dev;

	dev_dbg(dev, "SerDes: configuring SerDes for lane mask 0x%x\n", lanes);

	/* K1 SerDes main configuration */
	__raw_writel(0x0229, (void __iomem *) serdes_regs->pll);

	/* Per-lanes SerDes configuration */
	for_each_lanes(lanes, lane) {
		__raw_writel(0x00440495, &serdes_regs->channel[lane].rx);
		__raw_writel(0x00180795, &serdes_regs->channel[lane].rx);
	}

	timeout = jiffies + msecs_to_jiffies(KEYSTONE_SERDES_TIMEOUT);

	/* Check for RIO SerDes PLL lock */
	while (1) {
		val = __raw_readl(serdes->sts_reg);

		if ((val & 0x1) != 0x1)
			break;

		if (time_after(jiffies, timeout)) {
			res = -EAGAIN;
			break;
		}

		usleep_range(10, 50);
	}

	return res;
}

static int k1_rio_serdes_start_tx_lanes(
	u32 lanes,
	struct keystone_serdes_data *serdes)
{
	return 0;
}

static int k1_rio_serdes_wait_lanes_ok(u32 lanes,
				       struct keystone_serdes_data *serdes)
{
	return 0;
}

static int k1_rio_serdes_shutdown_lanes(u32 lanes,
					 struct keystone_serdes_data *serdes)
{
	return 0;
}

static void k1_rio_serdes_fix_unstable_lanes(
	u32 lanes,
	struct keystone_serdes_data *serdes)
{
	return;
}

static int k1_rio_serdes_calibrate_lanes(u32 lanes,
					 struct keystone_serdes_data *serdes)
{
	return -ENOSYS;
}

static const struct keystone_serdes_ops k1_serdes_ops = {
	.config_lanes       = k1_rio_serdes_config_lanes,
	.start_tx_lanes     = k1_rio_serdes_start_tx_lanes,
	.wait_lanes_ok      = k1_rio_serdes_wait_lanes_ok,
	.shutdown_lanes     = k1_rio_serdes_shutdown_lanes,
	.fix_unstable_lanes = k1_rio_serdes_fix_unstable_lanes,
	.calibrate_lanes    = k1_rio_serdes_calibrate_lanes,
};

/*-------------------------- KeyStone 2 SerDes functions --------------------*/

#define KEYSTONE_SERDES_PRBS_7                0
#define KEYSTONE_SERDES_PRBS_15               1
#define KEYSTONE_SERDES_PRBS_23               2
#define KEYSTONE_SERDES_PRBS_31               3

#define KEYSTONE_SERDES_MAX_TAPS              5
#define KEYSTONE_SERDES_MAX_COMPS             5

#define KEYSTONE_SERDES_OFFSETS_RETRIES       100
#define KEYSTONE_SERDES_ATT_BOOST_NUM_REPEAT  20
#define KEYSTONE_SERDES_ATT_BOOST_REPEAT_MEAN 14

/* SerDes PHY center DFE TAPs and data sample comparators structure */
struct k2_rio_serdes_tap_offsets {
	u32 tap1_offsets[KEYSTONE_SERDES_MAX_LANES][KEYSTONE_SERDES_MAX_TAPS];
	u32 tap2_offsets[KEYSTONE_SERDES_MAX_LANES][KEYSTONE_SERDES_MAX_TAPS];
	u32 tap3_offsets[KEYSTONE_SERDES_MAX_LANES][KEYSTONE_SERDES_MAX_TAPS];
	u32 tap4_offsets[KEYSTONE_SERDES_MAX_LANES][KEYSTONE_SERDES_MAX_TAPS];
	u32 tap5_offsets[KEYSTONE_SERDES_MAX_LANES][KEYSTONE_SERDES_MAX_TAPS];
	u32 cmp_offsets[KEYSTONE_SERDES_MAX_LANES][KEYSTONE_SERDES_MAX_COMPS];
};

struct k2_rio_serdes_reg_field {
	u32 reg;
	u32 shift;
};

static int k2_rio_serdes_start_tx_lanes(
	u32 lanes,
	struct keystone_serdes_data *serdes);

/*
 * Main code to Read TBUS on PHY-A
 */
static void k2_rio_serdes_sb_write_tbus_addr(void __iomem *regs,
					     int select,
					     int offset)
{
	int two_laner;

	two_laner = (__raw_readl(regs + 0x1fc0) >> 16) & 0x0ffff;

	if ((two_laner == 0x4eb9) || (two_laner == 0x4ebd))
		two_laner = 0;
	else
		two_laner = 1;

	if (select && two_laner)
		select++;

	reg_rmw(regs + 0x0008, ((select << 5) + offset) << 24, 0xff000000);
}

static u32 k2_rio_serdes_sb_read_tbus_val(void __iomem *regs)
{
	u32 tmp;

	tmp  = (__raw_readl(regs + 0x0ec) >> 24) & 0x000ff;
	tmp |= (__raw_readl(regs + 0x0fc) >> 16) & 0x00f00;

	return tmp;
}

static u32 k2_rio_serdes_sb_read_selected_tbus(void __iomem *regs,
					       int select,
					       int offset)
{
	k2_rio_serdes_sb_write_tbus_addr(regs, select, offset);

	return k2_rio_serdes_sb_read_tbus_val(regs);
}

/*
 * Wait SerDes Rx valid
 * To be performed after SerDes is configured and bit lock achieved
 */
static int k2_rio_serdes_wait_rx_valid(u32 lane, void __iomem *regs)
{
	unsigned long timeout = jiffies
		+ msecs_to_jiffies(KEYSTONE_SERDES_TIMEOUT);
	unsigned int stat;

       /*
	* Check for lnX_stat[3:2] to go high on both lanes, this indicates that
	* adaptation has completed timeout for lanes that are not connected.
	*/
	stat = (k2_rio_serdes_sb_read_selected_tbus(regs, lane + 1, 0x2)
		& 0x0060) >> 5;

	while (stat != 3) {
		stat = (k2_rio_serdes_sb_read_selected_tbus(regs, lane + 1, 0x2)
			& 0x0060) >> 5;

		if (time_after(jiffies, timeout))
			return 1;

		usleep_range(10, 50);
	}

	return 0;
}

/*
 * Allow Serdes to re-acquire Signal Detect
 */
static inline void k2_rio_serdes_reacquire_sd(u32 lane, void __iomem *regs)
{
	reg_finsr(regs + (0x200 * lane) + 0x200 + 0x04, 2, 1, 0x0);
}

/*
 * Read termination on TX-N and TX-P
 */
static inline u32 k2_rio_serdes_get_termination(void __iomem *regs)
{
	return k2_rio_serdes_sb_read_selected_tbus(regs, 1, 0x1b) & 0x00ff;
}

/*
 * Set the Tx termination
 */
static void k2_rio_serdes_termination_config(u32 lane,
					     void __iomem *regs,
					     u32 tx_term_np)
{
	/* Set tx termination */
	reg_fill_field(regs + (lane * 0x200) + 0x200 + 0x7c, 24, 8, tx_term_np);

	/* Set termination override */
	reg_fill_field(regs + (lane * 0x200) + 0x200 + 0x7c, 20, 1, 1);
}

static inline void k2_rio_serdes_get_att_boost(
	u32 lane,
	void __iomem *regs,
	struct keystone_serdes_lane_rx_config *rx_coeff)
{
	u32 att;
	u32 boost;

	/* Read attenuation and boost */
	att = boost = k2_rio_serdes_sb_read_selected_tbus(regs,
							  lane + 1, 0x11);
	rx_coeff->att   = (att   >> 4) & 0x0f;
	rx_coeff->boost = (boost >> 8) & 0x0f;

	return;
}

/*
 * Forcing the Tx idle on the transmitter will prevent the receiver from
 * adapting to the un-terminated Tx and un-programmed pre and post cursor
 * settings
 */
static inline void k2_rio_serdes_force_tx_idle(u32 lane, void __iomem *regs)
{
	reg_fill_field(regs + (0x200 * lane) + 0x200 + 0xb8, 16, 2, 3);
	reg_fill_field(regs + (lane * 4) + 0x1fc0 + 0x20, 24, 2, 3);
	reg_fill_field(regs + (0x200 * lane) + 0x200 + 0x28, 20, 2, 0);
}

static inline void k2_rio_serdes_force_tx_normal(u32 lane, void __iomem *regs)
{
	reg_fill_field(regs + (0x200 * lane) + 0x200 + 0xb8, 16, 2, 0);
	reg_fill_field(regs + (lane * 4) + 0x1fc0 + 0x20, 24, 2, 0);
	reg_fill_field(regs + (0x200 * lane) + 0x200 + 0x28, 20, 2, 3);
}

static inline void k2_rio_serdes_force_rx_disable(u32 lane, void __iomem *regs)
{
	reg_fill_field(regs + (lane * 4) + 0x1fc0 + 0x20, 15, 1, 1);
	reg_fill_field(regs + (lane * 4) + 0x1fc0 + 0x20, 13, 2, 0);
}

static inline void k2_rio_serdes_force_rx_enable(u32 lane, void __iomem *regs)
{
	reg_fill_field(regs + (lane * 4) + 0x1fc0 + 0x20, 15, 1, 1);
	reg_fill_field(regs + (lane * 4) + 0x1fc0 + 0x20, 13, 2, 3);
}

/*
 * Function to reset the Clock Data Recovery, attenuation and boost circuitry
 */
static inline void k2_rio_serdes_reset_cdr_att_boost(u32 lane,
						     void __iomem *regs)
{
	reg_finsr(regs + 0x004 + (0x200 * (lane + 1)), 2, 1, 0x2);
}

/*
 * Wait lanes to be OK by checking LNn_OK_STATE bits
 */
static int k2_rio_serdes_wait_lanes_ok(u32 lanes, void __iomem *regs)
{
	unsigned long timeout;
	u32 val;
	u32 val_mask;

	/* LNn_OK_STATE bits */
	val_mask = lanes << 8;

	/* Wait for the SerDes LANE_OK lock */
	timeout = jiffies + msecs_to_jiffies(KEYSTONE_SERDES_TIMEOUT);
	while (1) {
		/* Read PLL_CTRL */
		val = __raw_readl(regs + 0x1fc0 + 0x34);

		if ((val & val_mask) == val_mask)
			break;

		if (time_after(jiffies, timeout))
			return -EAGAIN;

		usleep_range(10, 50);
	}

	return 0;
}

/*
 * Wait signal detect by checking LNn_SD_STATE bits in addition to LNn_OK_STATE
 * bits
 */
static int k2_rio_serdes_wait_lanes_sd(u32 lanes,
				       struct keystone_serdes_data *serdes)
{
	void __iomem *regs = serdes->regs;
	unsigned long timeout;
	u32 val;
	u32 val_mask;
	u32 lane;

	/* LNn_SD_STATE and LNn_OK_STATE bits */
	val_mask = lanes | (lanes << 8);

	/* Wait for the SerDes LANE_OK lock */
	timeout = jiffies + msecs_to_jiffies(KEYSTONE_SERDES_TIMEOUT);
	while (1) {
		/* Read PLL_CTRL */
		val = __raw_readl(regs + 0x1fc0 + 0x34);

		if ((val & val_mask) == val_mask)
			break;

		if (time_after(jiffies, timeout))
			return -EAGAIN;

		usleep_range(10, 50);
	}

	/* Display Rx att/boost values */
	for_each_lanes(lanes, lane) {
		/* Get the current Rx att/boost values */
		k2_rio_serdes_get_att_boost(
			lane,
			regs,
			&(serdes->config->rx[lane]));

		dev_dbg(serdes->dev,
			"SerDes: Rx signal detected, att = %d, boost = %d for lane %d\n",
			serdes->config->rx[lane].att,
			serdes->config->rx[lane].boost,
			lane);
	}

	return 0;
}

/*
 * Assert reset while preserving the lnX_ctrl_i bits
 */
static void k2_rio_serdes_sb_assert_reset(u32 lane, void __iomem *regs)
{
	unsigned int ui_tmpo;
	unsigned int ui_tmp0;
	unsigned int ui_tmp1;

	/* Read lnX_ctrl_i/lnX_pd_i */
	ui_tmp0 = k2_rio_serdes_sb_read_selected_tbus(regs, lane + 1, 0);

	/* Read lnX_rate_i */
	ui_tmp1 = k2_rio_serdes_sb_read_selected_tbus(regs, lane + 1, 1);

	ui_tmpo = 0;                              /* set reset state */
	ui_tmpo |= ((ui_tmp1 >> 9) & 0x003) << 1; /* add user rate */
	ui_tmpo |= ((ui_tmp0) & 0x003) << 3;      /* add user pd */
	ui_tmpo |= ((ui_tmp0 >> 2) & 0x1FF) << 5; /* add user ctrl_i */
	ui_tmpo |= BIT(14);                       /* set override */
	ui_tmpo &= ~0x60;                         /* clear the tx valid bits */

	/* Only modify the reset bit and the overlay bit */
	reg_fill_field(regs + (lane * 0x200) + 0x200 + 0x28, 15, 15, ui_tmpo);
}

static inline void k2_rio_serdes_sb_assert_full_reset(u32 lane,
						      void __iomem *regs)
{
	/* Toggle bit 29 of LANE_028 */
	reg_finsr(regs + (0 * 0x200) + (1 * 0x200) + 0x200 + 0x28, 29, 15,
		  0x4260);
}

static inline int k2_rio_serdes_sb_deassert_reset(u32 lane,
						  void __iomem *regs,
						  u32 block)
{
	/* Clear the reset bit */
	reg_fill_field(regs + (lane * 0x200) + 0x200 + 0x28, 15, 1, 1);

	/* Waits for LANE OK to complete after deasserting the reset */
	if (block)
		return k2_rio_serdes_wait_lanes_ok(BIT(lane), regs);

	return 0;
}

static void k2_rio_serdes_sb_clear_overlay_bit29(u32 lane, void __iomem *regs)
{
	/* Clear overlay bit, bring the lane out of reset */
	reg_fill_field(regs + (lane * 0x200) + 0x200 + 0x28, 29, 1, 0);
}

static void k2_rio_serdes_sb_set_overlay_bit29(u32 lane, void __iomem *regs)
{
	/* Clear overlay bit, bring the lane out of reset */
	reg_fill_field(regs + (lane * 0x200) + 0x200 + 0x28, 29, 1, 1);
}

static void k2_rio_serdes_init_3g(void __iomem *reg)
{
	/* Uses 6G half rate configuration */
	reg_finsr((reg + 0x0000), 31, 24, 0x00);
	reg_finsr((reg + 0x0014),  7,  0, 0x82);
	reg_finsr((reg + 0x0014), 15,  8, 0x82);
	reg_finsr((reg + 0x0060),  7,  0, 0x48);
	reg_finsr((reg + 0x0060), 15,  8, 0x2c);
	reg_finsr((reg + 0x0060), 23, 16, 0x13);
	reg_finsr((reg + 0x0064), 15,  8, 0xc7);
	reg_finsr((reg + 0x0064), 23, 16, 0xc3);
	reg_finsr((reg + 0x0078), 15,  8, 0xc0);

	/*  Setting lane 0 SerDes to 3GHz */
	reg_finsr((reg + 0x0204),  7,  0, 0x80);
	reg_finsr((reg + 0x0204), 31, 24, 0x78);
	reg_finsr((reg + 0x0208),  7,  0, 0x24);
	reg_finsr((reg + 0x020c), 31, 24, 0x02);
	reg_finsr((reg + 0x0210), 31, 24, 0x1b);
	reg_finsr((reg + 0x0214),  7,  0, 0x7c);
	reg_finsr((reg + 0x0214), 15,  8, 0x6e);
	reg_finsr((reg + 0x0218),  7,  0, 0xe4);
	reg_finsr((reg + 0x0218), 23, 16, 0x80);
	reg_finsr((reg + 0x0218), 31, 24, 0x75);
	reg_finsr((reg + 0x022c), 15,  8, 0x08);
	reg_finsr((reg + 0x022c), 23, 16, 0x20);
	reg_finsr((reg + 0x0280),  7,  0, 0x70);
	reg_finsr((reg + 0x0280), 23, 16, 0x70);
	reg_finsr((reg + 0x0284),  7,  0, 0x85);
	reg_finsr((reg + 0x0284), 23, 16, 0x0f);
	reg_finsr((reg + 0x0284), 31, 24, 0x1d);
	reg_finsr((reg + 0x028c), 15,  8, 0x3b);

	/*  Setting lane 1 SerDes to 3GHz */
	reg_finsr((reg + 0x0404),  7,  0, 0x80);
	reg_finsr((reg + 0x0404), 31, 24, 0x78);
	reg_finsr((reg + 0x0408),  7,  0, 0x24);
	reg_finsr((reg + 0x040c), 31, 24, 0x02);
	reg_finsr((reg + 0x0410), 31, 24, 0x1b);
	reg_finsr((reg + 0x0414),  7,  0, 0x7c);
	reg_finsr((reg + 0x0414), 15,  8, 0x6e);
	reg_finsr((reg + 0x0418),  7,  0, 0xe4);
	reg_finsr((reg + 0x0418), 23, 16, 0x80);
	reg_finsr((reg + 0x0418), 31, 24, 0x75);
	reg_finsr((reg + 0x042c), 15,  8, 0x08);
	reg_finsr((reg + 0x042c), 23, 16, 0x20);
	reg_finsr((reg + 0x0480),  7,  0, 0x70);
	reg_finsr((reg + 0x0480), 23, 16, 0x70);
	reg_finsr((reg + 0x0484),  7,  0, 0x85);
	reg_finsr((reg + 0x0484), 23, 16, 0x0f);
	reg_finsr((reg + 0x0484), 31, 24, 0x1d);
	reg_finsr((reg + 0x048c), 15,  8, 0x3b);

	/*  Setting lane 2 SerDes to 3GHz */
	reg_finsr((reg + 0x0604),  7,  0, 0x80);
	reg_finsr((reg + 0x0604), 31, 24, 0x78);
	reg_finsr((reg + 0x0608),  7,  0, 0x24);
	reg_finsr((reg + 0x060c), 31, 24, 0x02);
	reg_finsr((reg + 0x0610), 31, 24, 0x1b);
	reg_finsr((reg + 0x0614),  7,  0, 0x7c);
	reg_finsr((reg + 0x0614), 15,  8, 0x6e);
	reg_finsr((reg + 0x0618),  7,  0, 0xe4);
	reg_finsr((reg + 0x0618), 23, 16, 0x80);
	reg_finsr((reg + 0x0618), 31, 24, 0x75);
	reg_finsr((reg + 0x062c), 15,  8, 0x08);
	reg_finsr((reg + 0x062c), 23, 16, 0x20);
	reg_finsr((reg + 0x0680),  7,  0, 0x70);
	reg_finsr((reg + 0x0680), 23, 16, 0x70);
	reg_finsr((reg + 0x0684),  7,  0, 0x85);
	reg_finsr((reg + 0x0684), 23, 16, 0x0f);
	reg_finsr((reg + 0x0684), 31, 24, 0x1d);
	reg_finsr((reg + 0x068c), 15,  8, 0x3b);

	/*  Setting lane 3 SerDes to 3GHz */
	reg_finsr((reg + 0x0804),  7,  0, 0x80);
	reg_finsr((reg + 0x0804), 31, 24, 0x78);
	reg_finsr((reg + 0x0808),  7,  0, 0x24);
	reg_finsr((reg + 0x080c), 31, 24, 0x02);
	reg_finsr((reg + 0x0810), 31, 24, 0x1b);
	reg_finsr((reg + 0x0814),  7,  0, 0x7c);
	reg_finsr((reg + 0x0814), 15,  8, 0x6e);
	reg_finsr((reg + 0x0818),  7,  0, 0xe4);
	reg_finsr((reg + 0x0818), 23, 16, 0x80);
	reg_finsr((reg + 0x0818), 31, 24, 0x75);
	reg_finsr((reg + 0x082c), 15,  8, 0x08);
	reg_finsr((reg + 0x082c), 23, 16, 0x20);
	reg_finsr((reg + 0x0880),  7,  0, 0x70);
	reg_finsr((reg + 0x0880), 23, 16, 0x70);
	reg_finsr((reg + 0x0884),  7,  0, 0x85);
	reg_finsr((reg + 0x0884), 23, 16, 0x0f);
	reg_finsr((reg + 0x0884), 31, 24, 0x1d);
	reg_finsr((reg + 0x088c), 15,  8, 0x3b);

	reg_finsr((reg + 0x0a00), 15,  8, 0x08);
	reg_finsr((reg + 0x0a08), 23, 16, 0x72);
	reg_finsr((reg + 0x0a08), 31, 24, 0x37);
	reg_finsr((reg + 0x0a30), 15,  8, 0x77);
	reg_finsr((reg + 0x0a30), 23, 16, 0x77);
	reg_finsr((reg + 0x0a84), 15,  8, 0x06);
	reg_finsr((reg + 0x0a94), 31, 24, 0x10);
	reg_finsr((reg + 0x0aa0), 31, 24, 0x81);
	reg_finsr((reg + 0x0abc), 31, 24, 0xff);
	reg_finsr((reg + 0x0ac0),  7,  0, 0x8b);
	reg_finsr((reg + 0x0a48), 15,  8, 0x8c);
	reg_finsr((reg + 0x0a48), 23, 16, 0xfd);
	reg_finsr((reg + 0x0a54),  7,  0, 0x72);
	reg_finsr((reg + 0x0a54), 15,  8, 0xec);
	reg_finsr((reg + 0x0a54), 23, 16, 0x2f);
	reg_finsr((reg + 0x0a58), 15,  8, 0x21);
	reg_finsr((reg + 0x0a58), 23, 16, 0xf9);
	reg_finsr((reg + 0x0a58), 31, 24, 0x00);
	reg_finsr((reg + 0x0a5c),  7,  0, 0x60);
	reg_finsr((reg + 0x0a5c), 15,  8, 0x00);
	reg_finsr((reg + 0x0a5c), 23, 16, 0x04);
	reg_finsr((reg + 0x0a5c), 31, 24, 0x00);
	reg_finsr((reg + 0x0a60),  7,  0, 0x00);
	reg_finsr((reg + 0x0a60), 15,  8, 0x80);
	reg_finsr((reg + 0x0a60), 23, 16, 0x00);
	reg_finsr((reg + 0x0a60), 31, 24, 0x00);
	reg_finsr((reg + 0x0a64),  7,  0, 0x20);
	reg_finsr((reg + 0x0a64), 15,  8, 0x12);
	reg_finsr((reg + 0x0a64), 23, 16, 0x58);
	reg_finsr((reg + 0x0a64), 31, 24, 0x0c);
	reg_finsr((reg + 0x0a68),  7,  0, 0x02);
	reg_finsr((reg + 0x0a68), 15,  8, 0x06);
	reg_finsr((reg + 0x0a68), 23, 16, 0x3b);
	reg_finsr((reg + 0x0a68), 31, 24, 0xe1);
	reg_finsr((reg + 0x0a6c),  7,  0, 0xc1);
	reg_finsr((reg + 0x0a6c), 15,  8, 0x4c);
	reg_finsr((reg + 0x0a6c), 23, 16, 0x07);
	reg_finsr((reg + 0x0a6c), 31, 24, 0xb8);
	reg_finsr((reg + 0x0a70),  7,  0, 0x89);
	reg_finsr((reg + 0x0a70), 15,  8, 0xe9);
	reg_finsr((reg + 0x0a70), 23, 16, 0x02);
	reg_finsr((reg + 0x0a70), 31, 24, 0x3f);
	reg_finsr((reg + 0x0a74),  7,  0, 0x01);
	reg_finsr((reg + 0x0b20), 23, 16, 0x37);
	reg_finsr((reg + 0x0b1c), 31, 24, 0x37);
	reg_finsr((reg + 0x0b20),  7,  0, 0x5d);
	reg_finsr((reg + 0x0000),  7,  0, 0x03);
	reg_finsr((reg + 0x0a00),  7,  0, 0x5f);
}

static void k2_rio_serdes_init_5g(void __iomem *reg)
{
	/* Uses 5Gbps full rate configuration by default */
	reg_finsr((reg + 0x0000), 31, 24, 0x00);
	reg_finsr((reg + 0x0014),  7,  0, 0x82);
	reg_finsr((reg + 0x0014), 15,  8, 0x82);
	reg_finsr((reg + 0x0060),  7,  0, 0x38);
	reg_finsr((reg + 0x0060), 15,  8, 0x24);
	reg_finsr((reg + 0x0060), 23, 16, 0x14);
	reg_finsr((reg + 0x0064), 15,  8, 0xc7);
	reg_finsr((reg + 0x0064), 23, 16, 0xc3);
	reg_finsr((reg + 0x0078), 15,  8, 0xc0);

	/*  Setting lane 0 SerDes to 5GHz */
	reg_finsr((reg + 0x0204),  7,  0, 0x80);
	reg_finsr((reg + 0x0204), 31, 24, 0x78);
	reg_finsr((reg + 0x0208),  7,  0, 0x26);
	reg_finsr((reg + 0x020c), 31, 24, 0x02);
	reg_finsr((reg + 0x0214),  7,  0, 0x38);
	reg_finsr((reg + 0x0214), 15,  8, 0x6f);
	reg_finsr((reg + 0x0218),  7,  0, 0xe4);
	reg_finsr((reg + 0x0218), 23, 16, 0x80);
	reg_finsr((reg + 0x0218), 31, 24, 0x75);
	reg_finsr((reg + 0x022c), 15,  8, 0x08);
	reg_finsr((reg + 0x022c), 23, 16, 0x20);
	reg_finsr((reg + 0x0280),  7,  0, 0x86);
	reg_finsr((reg + 0x0280), 23, 16, 0x86);
	reg_finsr((reg + 0x0284),  7,  0, 0x85);
	reg_finsr((reg + 0x0284), 23, 16, 0x0f);
	reg_finsr((reg + 0x0284), 31, 24, 0x1d);
	reg_finsr((reg + 0x028c), 15,  8, 0x2c);

	/*  Setting lane 1 SerDes to 5GHz */
	reg_finsr((reg + 0x0404),  7,  0, 0x80);
	reg_finsr((reg + 0x0404), 31, 24, 0x78);
	reg_finsr((reg + 0x0408),  7,  0, 0x26);
	reg_finsr((reg + 0x040c), 31, 24, 0x02);
	reg_finsr((reg + 0x0414),  7,  0, 0x38);
	reg_finsr((reg + 0x0414), 15,  8, 0x6f);
	reg_finsr((reg + 0x0418),  7,  0, 0xe4);
	reg_finsr((reg + 0x0418), 23, 16, 0x80);
	reg_finsr((reg + 0x0418), 31, 24, 0x75);
	reg_finsr((reg + 0x042c), 15,  8, 0x08);
	reg_finsr((reg + 0x042c), 23, 16, 0x20);
	reg_finsr((reg + 0x0480),  7,  0, 0x86);
	reg_finsr((reg + 0x0480), 23, 16, 0x86);
	reg_finsr((reg + 0x0484),  7,  0, 0x85);
	reg_finsr((reg + 0x0484), 23, 16, 0x0f);
	reg_finsr((reg + 0x0484), 31, 24, 0x1d);
	reg_finsr((reg + 0x048c), 15,  8, 0x2c);

	/*  Setting lane 2 SerDes to 5GHz */
	reg_finsr((reg + 0x0604),  7,  0, 0x80);
	reg_finsr((reg + 0x0604), 31, 24, 0x78);
	reg_finsr((reg + 0x0608),  7,  0, 0x26);
	reg_finsr((reg + 0x060c), 31, 24, 0x02);
	reg_finsr((reg + 0x0614),  7,  0, 0x38);
	reg_finsr((reg + 0x0614), 15,  8, 0x6f);
	reg_finsr((reg + 0x0618),  7,  0, 0xe4);
	reg_finsr((reg + 0x0618), 23, 16, 0x80);
	reg_finsr((reg + 0x0618), 31, 24, 0x75);
	reg_finsr((reg + 0x062c), 15,  8, 0x08);
	reg_finsr((reg + 0x062c), 23, 16, 0x20);
	reg_finsr((reg + 0x0680),  7,  0, 0x86);
	reg_finsr((reg + 0x0680), 23, 16, 0x86);
	reg_finsr((reg + 0x0684),  7,  0, 0x85);
	reg_finsr((reg + 0x0684), 23, 16, 0x0f);
	reg_finsr((reg + 0x0684), 31, 24, 0x1d);
	reg_finsr((reg + 0x068c), 15,  8, 0x2c);

	/*  Setting lane 3 SerDes to 5GHz */
	reg_finsr((reg + 0x0804),  7,  0, 0x80);
	reg_finsr((reg + 0x0804), 31, 24, 0x78);
	reg_finsr((reg + 0x0808),  7,  0, 0x26);
	reg_finsr((reg + 0x080c), 31, 24, 0x02);
	reg_finsr((reg + 0x0814),  7,  0, 0x38);
	reg_finsr((reg + 0x0814), 15,  8, 0x6f);
	reg_finsr((reg + 0x0818),  7,  0, 0xe4);
	reg_finsr((reg + 0x0818), 23, 16, 0x80);
	reg_finsr((reg + 0x0818), 31, 24, 0x75);
	reg_finsr((reg + 0x082c), 15,  8, 0x08);
	reg_finsr((reg + 0x082c), 23, 16, 0x20);
	reg_finsr((reg + 0x0880),  7,  0, 0x86);
	reg_finsr((reg + 0x0880), 23, 16, 0x86);
	reg_finsr((reg + 0x0884),  7,  0, 0x85);
	reg_finsr((reg + 0x0884), 23, 16, 0x0f);
	reg_finsr((reg + 0x0884), 31, 24, 0x1d);
	reg_finsr((reg + 0x088c), 15,  8, 0x2c);

	reg_finsr((reg + 0x0a00), 15,  8, 0x80);
	reg_finsr((reg + 0x0a08), 23, 16, 0xd2);
	reg_finsr((reg + 0x0a08), 31, 24, 0x38);
	reg_finsr((reg + 0x0a30), 15,  8, 0x8d);
	reg_finsr((reg + 0x0a30), 23, 16, 0x8d);
	reg_finsr((reg + 0x0a84), 15,  8, 0x06);
	reg_finsr((reg + 0x0a94), 31, 24, 0x10);
	reg_finsr((reg + 0x0aa0), 31, 24, 0x81);
	reg_finsr((reg + 0x0abc), 31, 24, 0xff);
	reg_finsr((reg + 0x0ac0),  7,  0, 0x8b);
	reg_finsr((reg + 0x0a48), 15,  8, 0x8c);
	reg_finsr((reg + 0x0a48), 23, 16, 0xfd);
	reg_finsr((reg + 0x0a54),  7,  0, 0x72);
	reg_finsr((reg + 0x0a54), 15,  8, 0xec);
	reg_finsr((reg + 0x0a54), 23, 16, 0x2f);
	reg_finsr((reg + 0x0a58), 15,  8, 0x21);
	reg_finsr((reg + 0x0a58), 23, 16, 0xf9);
	reg_finsr((reg + 0x0a58), 31, 24, 0x00);
	reg_finsr((reg + 0x0a5c),  7,  0, 0x60);
	reg_finsr((reg + 0x0a5c), 15,  8, 0x00);
	reg_finsr((reg + 0x0a5c), 23, 16, 0x04);
	reg_finsr((reg + 0x0a5c), 31, 24, 0x00);
	reg_finsr((reg + 0x0a60),  7,  0, 0x00);
	reg_finsr((reg + 0x0a60), 15,  8, 0x80);
	reg_finsr((reg + 0x0a60), 23, 16, 0x00);
	reg_finsr((reg + 0x0a60), 31, 24, 0x00);
	reg_finsr((reg + 0x0a64),  7,  0, 0x20);
	reg_finsr((reg + 0x0a64), 15,  8, 0x12);
	reg_finsr((reg + 0x0a64), 23, 16, 0x58);
	reg_finsr((reg + 0x0a64), 31, 24, 0x0c);
	reg_finsr((reg + 0x0a68),  7,  0, 0x02);
	reg_finsr((reg + 0x0a68), 15,  8, 0x06);
	reg_finsr((reg + 0x0a68), 23, 16, 0x3b);
	reg_finsr((reg + 0x0a68), 31, 24, 0xe1);
	reg_finsr((reg + 0x0a6c),  7,  0, 0xc1);
	reg_finsr((reg + 0x0a6c), 15,  8, 0x4c);
	reg_finsr((reg + 0x0a6c), 23, 16, 0x07);
	reg_finsr((reg + 0x0a6c), 31, 24, 0xb8);
	reg_finsr((reg + 0x0a70),  7,  0, 0x89);
	reg_finsr((reg + 0x0a70), 15,  8, 0xe9);
	reg_finsr((reg + 0x0a70), 23, 16, 0x02);
	reg_finsr((reg + 0x0a70), 31, 24, 0x3f);
	reg_finsr((reg + 0x0a74),  7,  0, 0x01);
	reg_finsr((reg + 0x0b20), 23, 16, 0x37);
	reg_finsr((reg + 0x0b1c), 31, 24, 0x37);
	reg_finsr((reg + 0x0b20),  7,  0, 0x5d);
	reg_finsr((reg + 0x0000),  7,  0, 0x03);
	reg_finsr((reg + 0x0a00),  7,  0, 0x5f);
}

static void k2_rio_serdes_lane_init(u32 lane, void __iomem *regs, u32 rate)
{
	/* Bring this lane out of reset by clearing override bit 29 */
	k2_rio_serdes_sb_clear_overlay_bit29(lane, regs);

	/* Set lane control rate, force lane enable, rates, width and Tx idle */
	switch (rate) {
	case KEYSTONE_SERDES_FULL_RATE:
		__raw_writel(0xf3c0f0f0, regs + 0x1fe0 + (4 * lane));
		break;
	case KEYSTONE_SERDES_HALF_RATE:
		__raw_writel(0xf7c0f4f0, regs + 0x1fe0 + (4 * lane));
		break;
	case KEYSTONE_SERDES_QUARTER_RATE:
		__raw_writel(0xfbc0f8f0, regs + 0x1fe0 + (4 * lane));
		break;
	default:
		return;
	}
}

static inline void k2_rio_serdes_lane_enable(u32 lane, void __iomem *regs)
{
	reg_rmw(regs + 0x1fe0 + (4 * lane),
		BIT(29) | BIT(30) | BIT(13) | BIT(14),
		BIT(29) | BIT(30) | BIT(13) | BIT(14));
}

static inline void k2_rio_serdes_lane_disable(u32 lane, void __iomem *regs)
{
	reg_rmw(regs + 0x1fe0 + (4 * lane),
		0,
		BIT(29) | BIT(30) | BIT(13) | BIT(14));
}

/*
 * Force the calibration of the SerDes receiver settings of att circuitry
 * for all configured lanes
 *
 */
static int k2_rio_serdes_force_att_calibration(u32 lanes,
					       void __iomem *regs,
					       u32 rate)
{
	int res = 0;
	u32 lane;
	u32 att_read[KEYSTONE_SERDES_MAX_LANES];
	u32 att_start[KEYSTONE_SERDES_MAX_LANES];
	u32 reg;
	u32 shift;

	/* Register offsert and field shift of att starrt for various rates */
	static struct k2_rio_serdes_reg_field __k2_rio_serdes_att_start[3] = {
		{ 0x84, 16  },
		{ 0x84, 24 },
		{ 0x8c, 8  },
	};

	/* Compute the rx att start field location corresponding to the rate */
	reg   = __k2_rio_serdes_att_start[rate].reg;
	shift = __k2_rio_serdes_att_start[rate].shift;

	/* First read initial att start value */
	for_each_lanes(lanes, lane)
		att_start[lane] =
		(__raw_readl(regs + (lane * 0x200) + 0x200 + reg)
		 >> shift) & 0xf;

	/*
	 * Check att value, fix this as start value turn off att adaptation
	 * and do boost readaptation
	 */
	for_each_lanes(lanes, lane)
		att_read[lane] =
		(k2_rio_serdes_sb_read_selected_tbus(regs, lane + 1, 0x11)
		 >> 4) & 0xf;

	/* Set att_start */
	for_each_lanes(lanes, lane)
		reg_fill_field(regs + (lane * 0x200) + 0x200 + reg,
			       shift, 4, att_read[lane]);

	/* Clear att init cal and att recal */
	reg_fill_field(regs + 0x0a00 + 0x84, 0,  1, 0);
	reg_fill_field(regs + 0x0a00 + 0x8c, 24, 1, 0);

	/* Force calibration for all lanes */
	reg_fill_field(regs + 0x0a00 + 0x98, 7, 1, 1);
	reg_fill_field(regs + 0x0a00 + 0x98, 7, 1, 0);

	/* Check RX valid */
	for_each_lanes(lanes, lane) {
		if (k2_rio_serdes_wait_rx_valid(lane, regs))
			res = 1;

		/* Write back initial att start value */
		reg_fill_field(regs + (lane * 0x200) + 0x200 + reg, shift, 4,
			       att_start[lane]);
	}

	/* Turn back on att adaptation */
	reg_fill_field(regs + 0x0a00 + 0x84,  0, 1, 1);
	reg_fill_field(regs + 0x0a00 + 0x8c, 24, 1, 1);

	return res;
}

/*
 * Force the calibration of the SerDes receiver settings of boost circuitry
 *
 */
static int k2_rio_serdes_force_boost_calibration(u32 lane,
						 void __iomem *regs)
{
	u32 boost_read = 0;

	if (k2_rio_serdes_wait_rx_valid(lane, regs))
		return -1;

	/* Check boost value */
	boost_read = (k2_rio_serdes_sb_read_selected_tbus(regs, lane + 1, 0x11)
		      >> 8) & 0xf;

	/* If boost = 0, increment by 1 */
	if (boost_read != 0)
		goto do_not_inc;

	/* Set rxeq_ovr_en to 1 */
	reg_fill_field(regs + (lane * 0x200) + 0x200 + 0x2c,  2, 1, 1);

	/* Set rxeq_ovr_load_en for boost only */
	reg_fill_field(regs + (lane * 0x200) + 0x200 + 0x2c, 12, 7, 0x2);

	/* Set rxeq_ovr_load for a value of 1 */
	reg_fill_field(regs + (lane * 0x200) + 0x200 + 0x2c,  3, 7, 0x1);

	/* Latch in new boost value */
	reg_fill_field(regs + (lane * 0x200) + 0x200 + 0x2c, 10, 1, 0x1);
	reg_fill_field(regs + (lane * 0x200) + 0x200 + 0x2c, 10, 1, 0x0);

	/* Reset previous registers */
	reg_fill_field(regs + (lane * 0x200) + 0x200 + 0x2c,  2, 1, 0x0);
	reg_fill_field(regs + (lane * 0x200) + 0x200 + 0x2c, 12, 7, 0x0);
	reg_fill_field(regs + (lane * 0x200) + 0x200 + 0x2c,  3, 7, 0x0);

do_not_inc:
	if (k2_rio_serdes_wait_rx_valid(lane, regs))
		return -1;

	return 0;
}

/*
 * Calibrates the SerDes receiver by finding the averaging values for
 * attenuation and boost coefficients
 */
static void k2_rio_serdes_att_boost_calibration(
	u32 lane,
	u32 lanes,
	void __iomem *regs,
	struct keystone_serdes_lane_rx_config *rx_coeff,
	u32 rate)
{
	u32 repeat_index;
	int att = 0, num_att = 0, boost = 0, num_boost = 0;
	int att_array[KEYSTONE_SERDES_ATT_BOOST_NUM_REPEAT];
	int boost_array[KEYSTONE_SERDES_ATT_BOOST_NUM_REPEAT];
	int res;

	for (repeat_index = 0;
	     repeat_index < KEYSTONE_SERDES_ATT_BOOST_NUM_REPEAT;
	     repeat_index++) {

		/* Wait SerDes Rx valid */
		if (k2_rio_serdes_wait_rx_valid(lane, regs)) {
			att = boost = -1;
			goto skip;
		}

		/* Force att calibration for all lanes */
		(void) k2_rio_serdes_force_att_calibration(lanes, regs, rate);

		/* Force boost calibration */
		res = k2_rio_serdes_force_boost_calibration(lane, regs);
		if (res < 0) {
			att = boost = -1;
			goto skip;
		}

		/* Record adapted att and boost into an array */
		att = boost = k2_rio_serdes_sb_read_selected_tbus(regs,
								  lane + 1,
								  0x11);
		att   = (att   >> 4) & 0x0f;
		boost = (boost >> 8) & 0x0f;

skip:
		att_array[repeat_index]   = att;
		boost_array[repeat_index] = boost;

		k2_rio_serdes_reset_cdr_att_boost(lane, regs);

		/* Wait at least 10 usecs */
		usleep_range(10, 50);

		/* Allow Serdes to re-acquire Signal Detect */
		k2_rio_serdes_reacquire_sd(lane, regs);
	}

	/* Perform statistical analysis on saved att, boost values */
	att = boost = num_att = num_boost = 0;

	for (repeat_index = 0;
	     repeat_index < KEYSTONE_SERDES_ATT_BOOST_NUM_REPEAT;
	     repeat_index++) {
		if ((att_array[repeat_index] > 0)
		    && (att_array[repeat_index]
			< KEYSTONE_SERDES_ATT_BOOST_REPEAT_MEAN)) {
			att += att_array[repeat_index];
			num_att++;
		}

		if ((boost_array[repeat_index] > 0)
		    && (boost_array[repeat_index]
			< KEYSTONE_SERDES_ATT_BOOST_REPEAT_MEAN)) {
			boost += boost_array[repeat_index];
			num_boost++;
		}
	}

	rx_coeff[lane].mean_att = (num_att > 0) ?
		(((att << 4) / num_att) + 8) >> 4 : -1;

	rx_coeff[lane].mean_boost = (num_boost > 0) ?
		(((boost << 4) / num_boost) + 8) >> 4 : -1;
}

/*
 * Disable Rx att and boost initial calibration and then set att and boost start
 * values to the mean values.
 */
static void k2_rio_serdes_set_att_boost(
	u32 lane,
	void __iomem *regs,
	struct keystone_serdes_lane_rx_config *rx_coeff)
{
	/* Disable lane in Rx */
	k2_rio_serdes_force_rx_disable(lane, regs);

	/* Disable Rx calibration for all rates (RATE1, RATE2, RATE3) */
	reg_finsr(regs + 0xa00 + 0x84, 10, 8, 0x0);

	if (rx_coeff->mean_att != -1) {

		/* Disable att init cal */
		reg_finsr(regs + 0xa00 + 0x84,  0,  0, 0x0);

		/* Disable att recal */
		reg_finsr(regs + 0xa00 + 0x8c, 24, 24, 0x0);

		/* Set att for all rates */
		reg_finsr(regs + 0x200 * (lane + 1) + 0x8c, 11,  8,
			  rx_coeff->mean_att);
		reg_finsr(regs + 0x200 * (lane + 1) + 0x84, 27, 24,
			  rx_coeff->mean_att);
		reg_finsr(regs + 0x200 * (lane + 1) + 0x84, 19, 16,
			  rx_coeff->mean_att);
	}

	if (rx_coeff->mean_boost != -1) {

		/* Disable boost init cal */
		reg_finsr(regs + 0xa00 + 0x84,  1,  1, 0x0);

		/* Disable boost recal */
		reg_finsr(regs + 0xa00 + 0x8c, 25, 25, 0x0);

		/* Set boost for all rates */
		reg_finsr(regs + 0x200 * (lane + 1) + 0x8c, 15, 12,
			  rx_coeff->mean_boost);
		reg_finsr(regs + 0x200 * (lane + 1) + 0x84, 31, 28,
			  rx_coeff->mean_boost);
		reg_finsr(regs + 0x200 * (lane + 1) + 0x84, 23, 20,
			  rx_coeff->mean_boost);
	}

	/* Enable lane in Rx */
	k2_rio_serdes_force_rx_enable(lane, regs);

}

static inline void k2_rio_serdes_config_set_tx_coeffs(
	u32 lane,
	void __iomem *regs,
	struct keystone_serdes_lane_tx_config *tx_config)
{
	u32 val;

	/* Ensure we do not override out of bound bits */
	tx_config->c1_coeff &= 0x1f; /* 5 bits */
	tx_config->c2_coeff &= 0xf;  /* 4 bits */
	tx_config->cm_coeff &= 0xf;  /* 4 bits */
	tx_config->pre_1lsb &= 0x1;  /* 1 bits */
	tx_config->att      &= 0xf;  /* 4 bits */
	tx_config->vreg     &= 0x7;  /* 4 bits */

	/* Set C1, C2, CM coefficients */
	val = (tx_config->c1_coeff)
		| ((tx_config->c2_coeff) << 8)
		| ((tx_config->cm_coeff) << 12);
	reg_rmw(regs + 0x008 + (0x200 * (lane + 1)), val, 0x0000ff1f);

	/* Set attenuation and pre-emphasis 1lsb */
	val = (tx_config->att << 25) | (tx_config->pre_1lsb << 21);
	reg_rmw(regs + 0x004 + (0x200 * (lane + 1)), val, 0x9e000000);

	/* Set vreg */
	reg_rmw(regs + 0x084 + (0x200 * (lane + 1)), (tx_config->vreg << 5),
		0x000000e0);
}

static void k2_rio_serdes_config_set_rx_coeffs(
	u32 lanes,
	void __iomem *regs,
	struct device *dev,
	struct keystone_serdes_lane_rx_config *rx_config)
{
	u32 lane;

	/* Set Rx att/boost */
	for_each_lanes(lanes, lane) {
		/*
		 * We need to have at least one correct value before disabling
		 * the dynamic Rx att/boost calibration
		 */
		if ((rx_config[lane].mean_att == -1) &&
		    (rx_config[lane].mean_boost == -1))
			continue;

		/* Force signal detect low, reset CDR, attenuation and boost */
		k2_rio_serdes_reset_cdr_att_boost(lane, regs);

		usleep_range(10, 50);

		dev_dbg(dev,
			"SerDes: applying computed Rx att = %d, Rx boost = %d for lane %d\n",
			rx_config[lane].mean_att,
			rx_config[lane].mean_boost,
			lane);

		/* Apply Rx att/boost values */
		k2_rio_serdes_set_att_boost(lane, regs,
					    &(rx_config[lane]));

		/* Allow SerDes to re-acquire signal detect */
		k2_rio_serdes_reacquire_sd(lane, regs);

		/* Wait at least 10 usecs */
		usleep_range(10, 50);

		/* Wait Rx valid */
		k2_rio_serdes_wait_rx_valid(lane, regs);
	}

	return;
}

/*
 * Set the SerDes Tx output swing voltage and boost/att start values
 */
static void k2_rio_serdes_tx_rx_set_equalizer(u32 lane,
					      void __iomem *regs,
					      int vreg_enable,
					      u32 att_start,
					      u32 boost_start)
{
	/* Set Tx output swing voltage */
	if (vreg_enable) {
		/* pma_ln_vreg */
		reg_finsr(regs + 0x200 * (lane + 1) + 0x18, 25, 24, 0x2);

		/* pma_ln_vregh */
		reg_finsr(regs + 0x200 * (lane + 1) + 0x18, 27, 26, 0x2);
	}

	/* Set Rx att start for all rates (RATE1, RATE2, RATE3) */
	reg_finsr(regs + 0x200 * (lane + 1) + 0x8c, 11,  8, att_start);
	reg_finsr(regs + 0x200 * (lane + 1) + 0x84, 27, 24, att_start);
	reg_finsr(regs + 0x200 * (lane + 1) + 0x84, 19, 16, att_start);

	/* Set Rx boost start for all rates (RATE1, RATE2, RATE3) */
	reg_finsr(regs + 0x200 * (lane + 1) + 0x8c, 15, 12, boost_start);
	reg_finsr(regs + 0x200 * (lane + 1) + 0x84, 31, 28, boost_start);
	reg_finsr(regs + 0x200 * (lane + 1) + 0x84, 23, 20, boost_start);
}

/*
 * Calculate the SerDes average comparator/tap offset values
 */
static void k2_rio_serdes_get_average_offsets(
	u32 lanes,
	void __iomem *regs,
	struct k2_rio_serdes_tap_offsets *tap_offsets)
{
	u32 i, lane, comp;
	u32 cmp_offset_tmp;

	/* Find average values */
	for (i = 0; i < KEYSTONE_SERDES_OFFSETS_RETRIES; i++) {
		for_each_lanes(lanes, lane) {
			/* Serdes assert reset */
			k2_rio_serdes_sb_assert_full_reset(lane, regs);

			/* Serdes Deassert reset and wait lane ok */
			k2_rio_serdes_sb_deassert_reset(lane, regs, 1);
		}

		for_each_lanes(lanes, lane) {
			for (comp = 1;
			     comp < KEYSTONE_SERDES_MAX_COMPS;
			     comp++) {

				/* Write comparator value */
				reg_finsr(regs + 0x0a00 + 0x8c, 23, 21, comp);

				/* Set tbus address */
				reg_finsr(regs + 0x008, 31, 24,
					  0x12 + ((lane + 1) << 5));

				cmp_offset_tmp = (
					k2_rio_serdes_sb_read_selected_tbus(
						regs,
						lane + 1,
						0x12) & 0x0ff0) >> 4;

				/*
				 * Take a running count of comparator and tap
				 * offsets for num_resets number of runs
				 */
				tap_offsets->cmp_offsets[lane][comp] +=
					cmp_offset_tmp;
			}
		}
	}

	/* Get average value for each comparator and tap offset */
	for_each_lanes(lanes, lane) {
		for (comp = 1; comp < KEYSTONE_SERDES_MAX_COMPS; comp++) {
			/* Set comparator average */
			tap_offsets->cmp_offsets[lane][comp] /=
				KEYSTONE_SERDES_OFFSETS_RETRIES;
		}
	}
}

/*
 * Force Serdes comparator and tap offsets to override value
 */
static void k2_rio_serdes_override_cmp_tap_offsets(
	u32 lane,
	void __iomem *regs,
	u32 comp,
	struct k2_rio_serdes_tap_offsets *tap_offsets)

{
	/* Set DFE_SHADOW_LANE_SEL */
	reg_finsr(regs + 0x0a00 + 0xf0, 27, 26, lane + 1);

	/* Set CMP_OFFSET_OVR_EN to 0x1 */
	reg_finsr(regs + 0x0a00 + 0x98, 24, 24, 0x1);

	/* Set RXEQ_OVR_EN to 0x1 */
	reg_finsr(regs + 0x200 * (lane + 1) + 0x2c, 2, 2, 0x1);

	/* Set RXEQ_DFE_CMP_SEL_OVR to comp */
	reg_finsr(regs + 0x200 * (lane + 1) + 0x30, 7, 5, comp);

	/* Set DFE_TAP_OVR_EN to 0x1 */
	reg_finsr(regs + 0x200 * (lane + 1) + 0x5c, 31, 31, 0x1);

	/* Set CMP_OFFSET_OVR */
	reg_finsr(regs + 0x0a00 + 0x9c, 7, 0,
		  tap_offsets->cmp_offsets[lane][comp]);

	/* Set tap overrides */
	reg_finsr(regs + 0x200 * (lane + 1) + 0x58, 30, 24,
		  tap_offsets->tap1_offsets[lane][comp]);

	reg_finsr(regs + 0x200 * (lane + 1) + 0x5c, 5, 0,
		  tap_offsets->tap2_offsets[lane][comp]);

	reg_finsr(regs + 0x200 * (lane + 1) + 0x5c, 13, 8,
		  tap_offsets->tap3_offsets[lane][comp]);

	reg_finsr(regs + 0x200 * (lane + 1) + 0x5c, 21, 16,
		  tap_offsets->tap4_offsets[lane][comp]);

	reg_finsr(regs + 0x200 * (lane + 1) + 0x5c, 29, 24,
		  tap_offsets->tap5_offsets[lane][comp]);

	/* Set RXEQ_OVR_LATCH_O = 0x1 */
	reg_finsr(regs + 0x200 * (lane + 1) + 0x2c, 10, 10, 0x1);

	/* Set RXEQ_OVR_LATCH_O = 0x0 */
	reg_finsr(regs + 0x200 * (lane + 1) + 0x2c, 10, 10, 0x0);

	/* Set CMP_OFFSET_OVR_EN to 0x0 */
	reg_finsr(regs + 0x0a00 + 0x98, 24, 24, 0x0);

	/* Set RXEQ_OVR_EN to 0x0 */
	reg_finsr(regs + 0x200 * (lane + 1) + 0x2c, 2, 2, 0x0);

	/* Set DFE_TAP_OVR_EN to 0x0 */
	reg_finsr(regs + 0x200 * (lane + 1) + 0x5c, 31, 31, 0x0);
}

/*
 * Write out the average rx comparator offsets during every SerDes
 * initialization
 */
static void k2_rio_serdes_write_average_offsets(
	u32 lanes,
	void __iomem *regs,
	struct k2_rio_serdes_tap_offsets *tap_offsets)
{
	u32 lane;
	u32 comp;

	/* Rewrite average comparator and tap offsets to comparators/taps */
	for_each_lanes(lanes, lane) {
		for (comp = 1; comp < KEYSTONE_SERDES_MAX_COMPS; comp++) {
			k2_rio_serdes_override_cmp_tap_offsets(lane,
							       regs,
							       comp,
							       tap_offsets);
		}
	}
}

/*
 * Calibrates the SerDes DFE
 */
static void k2_rio_serdes_dfe_offset_calibration(
	u32 lanes,
	void __iomem *regs,
	struct device *dev,
	struct keystone_serdes_lane_tx_config *tx_config)
{
	struct k2_rio_serdes_tap_offsets tap_offsets;
	u32 lane;

	/* Reset CDR, attenuation and boost */
	for_each_lanes(lanes, lane)
		k2_rio_serdes_reset_cdr_att_boost(lane, regs);

	/* Wait at least 10 usecs */
	usleep_range(10, 50);

	/* Compute average comparator/tap offsets */
	k2_rio_serdes_get_average_offsets(lanes, regs, &tap_offsets);

	/* Display comparators */
	for_each_lanes(lanes, lane) {
		u32 i;

		for (i = 1; i < KEYSTONE_SERDES_MAX_COMPS; i++) {
			dev_dbg(dev,
				"SerDes: %s() lane %d cmp_offsets[%d] %d\n",
				__func__, lane, i,
				tap_offsets.cmp_offsets[lane][i]);
		}
	}

	dev_dbg(dev, "SerDes: %s() write average offsets\n", __func__);

	/* Offset compensation workaround */
	k2_rio_serdes_write_average_offsets(lanes, regs, &tap_offsets);

	/* Wait at least 10 usecs */
	usleep_range(10, 50);
}

/*
 * Check that PHY has successfully detected the PRBS data or not
 */
static inline int k2_rio_serdes_wait_bist_chk_synch(u32 lanes,
						    void __iomem *regs,
						    int valid)
{
	u32 lane;
	u32 temp;
	u32 bist_valid[KEYSTONE_SERDES_MAX_LANES];
	unsigned long timeout = jiffies
		+ msecs_to_jiffies(KEYSTONE_SERDES_TIMEOUT);

	do {
		for_each_lanes(lanes, lane) {
			bist_valid[lane] =
				(k2_rio_serdes_sb_read_selected_tbus(regs,
								     lane + 1,
								     0xc)
				 & 0x0400) >> 10;

			if ((__raw_readl(regs + 0x1fc0 + 0x34)
			     & BIT(lane)) == 0)
				bist_valid[lane] = 0;
		}

		temp = 1;
		for_each_lanes(lanes, lane)
			temp &= bist_valid[lane];

		if (time_after(jiffies, timeout))
			return 1;

		usleep_range(10, 50);

	} while (temp != valid);

	return 0;
}

/*
 * Check the transmitter PRBS generator
 */
static int k2_rio_serdes_prbs_check(u32 lane, void __iomem *regs)
{
	/* Clear BCHK_CLR */
	reg_finsr(regs + (0x200 * lane) + 0x200 + 0x04, 4, 4, 0x0);

	/* Set BCHK_EN */
	reg_finsr(regs + (0x200 * lane) + 0x200 + 0x04, 3, 3, 0x1);

	/* Check bist_valid for lane */
	return k2_rio_serdes_wait_bist_chk_synch(BIT(lane), regs, 1);
}

/*
 * Setting up the transmitter PRBS generator for BER testing
 */
static void k2_rio_serdes_ber_test_tx(u32 lanes, void __iomem *regs)
{
	u32 lane;

	for_each_lanes(lanes, lane) {
		/* Set bit 5 of LANE_034 to 0 (bist_gen_en) */
		reg_finsr(regs + (0x200 * lane) + 0x200 + 0x34, 5, 5, 0);

		/*
		 * Set bit 2 [bist_chk_data_mode], 6 [bist_tx_clk_en],
		 * 7 [bist_rx_clk_en] of LANE_032 to 1
		 */
		reg_finsr(regs + (0x200 * lane) + 0x200 + 0x30, 22, 22, 1);
		reg_finsr(regs + (0x200 * lane) + 0x200 + 0x30, 23, 23, 1);
		reg_finsr(regs + (0x200 * lane) + 0x200 + 0x30, 18, 18, 1);

		/*
		 * Set bus width to 20 bit interface for SRIO SerDes
		 *
		 * Clear bit 0 [bist_gen_mode8B] of LANE_032 (PHY-A) and set
		 * bit 1 of LANE_034 [bist_gen_word]
		 */
		reg_finsr(regs + (0x200 * lane) + 0x200 + 0x30, 16, 16, 0);
		reg_finsr(regs + (0x200 * lane) + 0x200 + 0x34,  6,  6,  1);

		/*
		 * Set bit 6 [bist_gen_word] & 7 [bist_gen_en] of LANE_034
		 * to 1 and 0 respectively
		 */
		reg_finsr(regs + (0x200 * lane) + 0x200 + 0x34, 7, 7, 0);

		/* Set bit 3 [bist_gen_send_pream] of LANE_037 to 0 */
		reg_finsr(regs + (0x200 * lane) + 0x200 + 0x34, 27, 27, 0);

		/* Set bit 5:3 to 0x1 on LANE_032 to PRBS-31 */
		reg_finsr(regs + (0x200 * lane) + 0x200 + 0x30, 21, 19,
			  KEYSTONE_SERDES_PRBS_31);
	}

	/*
	 * Set bit 1:0 of COMLANE_017 to 0 respectively
	 * 1 bist_gen_inv_prbs, 0 bist_chk_inv_prbs
	 */
	reg_finsr(regs + (5 * 0x200) + 0x14, 25, 24, 0);

	for_each_lanes(lanes, lane) {
		/*
		 * Set LANE_033 and 034 bits 1:0 and 7:0 to 0x283
		 * (bist_chk_pream0)
		 */
		reg_finsr(regs + (0x200 * lane) + 0x200 + 0x30, 31, 24, 0x83);
		reg_finsr(regs + (0x200 * lane) + 0x200 + 0x34,  1,  0, 0x2);

		/*
		 * Set LANE_036 and 035 bits 1:0 and 7:0 to 0x17C
		 * (bist_chk_pream1)
		 */
		reg_finsr(regs + (0x200 * lane) + 0x200 + 0x34, 17,  8, 0x17c);

		/*
		 * Clear LANE_038-03C; 042-043; 044-045; 03D-041:
		 * 34[4:2]; 37 [6:4] & 47 [3:0], 46[7:0]
		 * bist_chk_udp, bist_gen_en_low, bist_gen_en_high,
		 * bist_chk_insert_word, bist_chk_insert_length,
		 * bist_gen_insert_count, bist_gen_insert_delay
		 */
		__raw_writel(0, regs + (0x200 * lane) + 0x200 + 0x38);
		__raw_writel(0, regs + (0x200 * lane) + 0x200 + 0x3c);
		__raw_writel(0, regs + (0x200 * lane) + 0x200 + 0x40);

		reg_finsr(regs + (0x200 * lane) + 0x200 + 0x44, 27,  0, 0x0);
		reg_finsr(regs + (0x200 * lane) + 0x200 + 0x34,  4,  2, 0x0);
		reg_finsr(regs + (0x200 * lane) + 0x200 + 0x34, 30, 28, 0x0);
	}

	/* Wait a minimum of 20ns let us put 50ns just in case */
	ndelay(50);

	for_each_lanes(lanes, lane) {
		/* De-assert BIST_GEN_CDN by setting 1 on bit 5 of LANE_034 */
		reg_finsr(regs + (0x200 * lane) + 0x200 + 0x34,  5,  5, 0x1);

		/*
		 * Set LANE_000 [7:6] (dmux_txa_sel) to 0x2 and LANE_002 [6:4]
		 * (dmux_txb_sel) to 0x0
		 */
		reg_finsr(regs + (0x200 * lane) + 0x200 + 0x00,  7,  6, 0x2);
		reg_finsr(regs + (0x200 * lane) + 0x200 + 0x00, 22, 20, 0x0);
	}

	/* Wait a minimum of 20ns let us put 50ns just in case */
	ndelay(50);

	/* Assert bist_gen_en by setting bit 7 of LANE_034 */
	for_each_lanes(lanes, lane)
		reg_finsr(regs + (0x200 * lane) + 0x200 + 0x34,  7,  7, 0x1);

	/* Wait at least 10 usecs */
	usleep_range(10, 50);

	/* Signal detect enable */
	for_each_lanes(lanes, lane)
		k2_rio_serdes_reacquire_sd(lane, regs);

	/* Wait at least 10 usecs */
	usleep_range(10, 50);

	/*
	 * Wait a minimum of 2us for the analog Tx/Rx paths to obtain the
	 * bit lock on to the training pattern.
	 * While this is ocurring set LANE_004 to [3] - 0, [4] - 1, [6:5] - 2
	 * (for PHY-A only).
	 */
	for_each_lanes(lanes, lane)
		reg_finsr(regs + (0x200 * lane) + 0x200 + 0x04, 6, 3,
			  (BIT(1) | (0x2 << 2)));

	return;
}

static int k2_rio_serdes_prbs_test_start(u32 lane,
					 void __iomem *regs,
					 struct device *dev)
{
	int res;

	dev_dbg(dev, "SerDes: enable the Tx PRBS pattern for lane %d\n", lane);

	/* Enable the Tx PRBS pattern */
	k2_rio_serdes_ber_test_tx(BIT(lane), regs);

	dev_dbg(dev, "SerDes: check PRBS pattern for lane %d\n", lane);

	res = k2_rio_serdes_prbs_check(lane, regs);
	if (res) {
		dev_dbg(dev,
			"SerDes: timeout when checking PRBS pattern for lane %d\n",
			lane);
		return -EAGAIN;
	}

	return 0;
}

static void k2_rio_serdes_prbs_test_stop(u32 lanes,
					 void __iomem *regs,
					 struct device *dev)
{
	u32 lane;
	u32 value;

	dev_dbg(dev, "SerDes: disable Tx PRBS pattern for lanes 0x%x\n", lanes);

	for_each_lanes(lanes, lane) {
		/* Set BCHK_CLR */
		reg_finsr(regs + (0x200 * lane) + 0x200 + 0x04, 4, 4, 0x1);

		/* Clear BCHK_EN */
		reg_finsr(regs + (0x200 * lane) + 0x200 + 0x04, 3, 3, 0x0);

		/* Set bit 5 of LANE_034 to 0 (bist_gen_en) */
		reg_finsr(regs + (0x200 * lane) + 0x200 + 0x34, 5, 5, 0);
	}

	/* Check bist_valid de-asserted for all lanes */
	if (k2_rio_serdes_wait_bist_chk_synch(lanes, regs, 0))
		dev_warn(dev,
			 "SerDes: Tx PRBS not successfuly disabled for lanes 0x%x\n",
			 lanes);

	for_each_lanes(lanes, lane) {
		/* Verify BIST_CHK_ERRORS is 0 */
		value = __raw_readl(regs + (0x200 * lane) + 0x200 + 0x48);
		if (value & 0xffff)
			dev_warn(dev,
				 "SerDes: Tx PRBS BIST error detected (0x%x) for lanes 0x%x\n",
				 value & 0xffff, lanes);

		/*
		 * Set LANE_000 [7:6] (dmux_txa_sel) and LANE_002 [6:4]
		 * (dmux_txb_sel) to 0x0
		 */
		reg_finsr(regs + (0x200 * lane) + 0x200 + 0x00,  7,  6, 0x0);
		reg_finsr(regs + (0x200 * lane) + 0x200 + 0x00, 22, 20, 0x0);

		/* Wait 100 ns before PHY will be back in normal mode */
		ndelay(100);
	}

	return;
}

static int __k2_rio_serdes_calibrate_lane(
	u32 lane,
	u32 lanes,
	struct keystone_serdes_data *serdes)
{
	struct device *dev = serdes->dev;
	void __iomem *regs = serdes->regs;
	int res = 0;

	/* Wait lanes OK and SD */
	res = k2_rio_serdes_wait_lanes_sd(BIT(lane), serdes);
	if (res < 0) {
		dev_dbg(dev, "SerDes: %s() lane %d not OK\n",
			__func__, lane);
		return res;
	}

	/* PRBS test to calibrate the attenuation and boost in the receiver */
	res = k2_rio_serdes_prbs_test_start(lane, regs, dev);
	if (res != 0)
		return res;

	/* Do the att/boost calibration */
	dev_dbg(dev, "SerDes: do the att/boost calibration\n");
	k2_rio_serdes_att_boost_calibration(lane,
					    lanes,
					    regs,
					    serdes->config->rx,
					    serdes->config->rate);

	/* Display new computed att/boost values */
	dev_dbg(dev,
		"SerDes: computed Rx att = %d, Rx boost = %d for lane %d\n",
		serdes->config->rx[lane].mean_att,
		serdes->config->rx[lane].mean_boost,
		lane);

	return res;
}

/*
 * Calibrate SerDes using PRBS training
 */
static int k2_rio_serdes_calibrate_lanes(u32 lanes,
					 struct keystone_serdes_data *serdes)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(
		serdes->config->cal_timeout * MSEC_PER_SEC);
	struct device *dev = serdes->dev;
	void __iomem *regs = serdes->regs;
	int res = 0;
	u32 lane;
	u32 __lanes = lanes;

	/* Do not use Rx coefficients yet */
	for_each_lanes(lanes, lane) {
		serdes->config->rx[lane].mean_att   = -1;
		serdes->config->rx[lane].mean_boost = -1;
	}

	/* Start lanes */
	k2_rio_serdes_start_tx_lanes(lanes, serdes);

	/* Try to calibrate all lanes */
	while (__lanes) {
		for (lane = 0;
		     lane < KEYSTONE_SERDES_MAX_LANES;
		     lane++) {
			if (BIT(lane) & __lanes) {
				/* Calibrate SerDes */
				if (__k2_rio_serdes_calibrate_lane(
					    lane, lanes, serdes) == 0)
					__lanes &= ~(BIT(lane));
			}
		}

		if (time_after(jiffies, timeout)) {
			res = -EAGAIN;
			break;
		}
	}

	/* Wait at least 10ms for remote to finish */
	usleep_range(10000, 20000);

	/* Stop PRBS test and restore lanes to normal state */
	k2_rio_serdes_prbs_test_stop(lanes, regs, dev);

	/* Disable transmitter for all lanes */
	for_each_lanes(lanes, lane)
		k2_rio_serdes_force_tx_idle(lane, regs);

	/* Apply computed Rx coefficients */
	k2_rio_serdes_config_set_rx_coeffs(lanes, serdes->regs, serdes->dev,
					   serdes->config->rx);

	/* Disable SerDes for these lanes */
	for_each_lanes(lanes, lane)
		k2_rio_serdes_lane_disable(lane, regs);

	return res;
}

/*
 * Configure SerDes with appropriate baudrate and do Tx termination workaround
 * plus DFE offset calibration.
 * Note that all lanes are configured but Serdes are then disabled
 */
static int k2_rio_serdes_config_lanes(u32 lanes,
				      u32 baud,
				      struct keystone_serdes_data *serdes)
{
	struct device *dev = serdes->dev;
	void __iomem *regs = serdes->regs;
	u32 rate;
	u32 val;
	u32 lane;
	u32 tx_term_np;
	int res;

	dev_dbg(dev, "SerDes: configuring SerDes for lane mask 0x%x\n", lanes);

	/* Disable PLL before configuring the SerDes registers */
	__raw_writel(0x80000000, regs + 0x1ff4);

	switch (baud) {
	case KEYSTONE_SERDES_BAUD_1_250:
		rate = KEYSTONE_SERDES_QUARTER_RATE;
		k2_rio_serdes_init_5g(regs);
		break;
	case KEYSTONE_SERDES_BAUD_2_500:
		rate = KEYSTONE_SERDES_HALF_RATE;
		k2_rio_serdes_init_5g(regs);
		break;
	case KEYSTONE_SERDES_BAUD_5_000:
		rate = KEYSTONE_SERDES_FULL_RATE;
		k2_rio_serdes_init_5g(regs);
		break;
	case KEYSTONE_SERDES_BAUD_3_125:
		rate = KEYSTONE_SERDES_HALF_RATE;
		k2_rio_serdes_init_3g(regs);
		break;
	default:
		return -EINVAL;
	}

	/* Store current rate */
	serdes->config->rate = rate;

	/* We need to always use lane 0 even if not part of the lane mask */
	lanes |= KEYSTONE_SERDES_LANE(0);

	/* Disable transmitter for all lanes */
	for_each_lanes(lanes, lane)
		k2_rio_serdes_force_tx_idle(lane, regs);

	/* Initialize SerDes for requested lanes */
	for_each_lanes(lanes, lane)
		k2_rio_serdes_lane_init(lane, regs, rate);

	/* Enable PLL via the PLL_ctrl */
	__raw_writel(0xe0000000, regs + 0x1ff4);

	/* Wait PLL OK: should be done in no more than 400us */
	do {
		val = __raw_readl(regs + 0x1ff4);
	} while (!(val & BIT(28)));

	/* Wait lanes OK */
	res = k2_rio_serdes_wait_lanes_ok(lanes, regs);
	if (res < 0) {
		dev_dbg(dev, "SerDes: %s() lane mask 0x%x not OK\n",
			__func__, lanes);
		return res;
	}

	/* Read the Tx termination */
	tx_term_np = k2_rio_serdes_get_termination(regs);
	dev_dbg(dev, "SerDes: termination tx is 0x%x\n", tx_term_np);

	/* Set regulator output voltage and Rx boost/att start values */
	for_each_lanes(lanes, lane) {
		dev_dbg(dev,
			"SerDes: Tx vdreg = %d, Rx att start = %d, Rx boost start = %d for lane %d\n",
			serdes->config->tx[lane].vdreg,
			serdes->config->rx[lane].start_att,
			serdes->config->rx[lane].start_boost,
			lane);

		k2_rio_serdes_tx_rx_set_equalizer(
			lane,
			regs,
			serdes->config->tx[lane].vdreg,
			serdes->config->rx[lane].start_att,
			serdes->config->rx[lane].start_boost);
	}

	for_each_lanes(lanes, lane) {
		/* Serdes assert reset */
		k2_rio_serdes_sb_assert_full_reset(lane, regs);

		/* Set the Tx coefficients */
		k2_rio_serdes_config_set_tx_coeffs(lane, regs,
						   &(serdes->config->tx[lane]));

		/* Force signal detect low, reset CDR, attenuation and boost */
		k2_rio_serdes_reset_cdr_att_boost(lane, regs);

		/* Serdes deassert reset */
		k2_rio_serdes_sb_deassert_reset(lane, regs, 1);

		/* Set the saved Tx termination */
		k2_rio_serdes_termination_config(lane, regs, tx_term_np);
	}

	/* Wait lanes OK */
	k2_rio_serdes_wait_lanes_ok(lanes, regs);

	/* Eventually perform the DFE offset calibration */
	if (serdes->config->do_dfe_cal) {
		dev_dbg(dev, "SerDes: lanes OK, start offset calibration\n");

		k2_rio_serdes_dfe_offset_calibration(lanes, regs, dev,
						     serdes->config->tx);
	}

	/* Allow SerDes to re-acquire Signal Detect */
	for_each_lanes(lanes, lane)
		k2_rio_serdes_reacquire_sd(lane, regs);

	/* Wait at least 10 usecs */
	usleep_range(10, 50);

	/* Apply Rx coefficients if any */
	k2_rio_serdes_config_set_rx_coeffs(lanes, serdes->regs, serdes->dev,
					   serdes->config->rx);

	/* Check lane OK */
	k2_rio_serdes_wait_lanes_ok(lanes, regs);

	/* Disable lanes */
	for_each_lanes(lanes, lane)
		k2_rio_serdes_lane_disable(lane, regs);

	return res;
}

static int k2_rio_serdes_shutdown_lanes(u32 lanes,
					struct keystone_serdes_data *serdes)
{
	void __iomem *regs = serdes->regs;
	u32 lane;

	dev_dbg(serdes->dev, "shutdown lanes 0x%x\n", lanes);

	/* Disable lanes */
	for_each_lanes(lanes, lane) {
		k2_rio_serdes_lane_disable(lane, regs);
		k2_rio_serdes_sb_set_overlay_bit29(lane, regs);
	}

	/* Disable CMU TX PLL */
	reg_finsr(regs + 0x1fc0 + 0x34, 30, 29, 0x0);

	/* Reset CMU PLL for all lanes */
	reg_finsr(regs + 0x10, 28, 28, 0x1);

	return 0;
}

static void __k2_rio_serdes_fix_unstable_lane(
	int lane,
	void __iomem *regs,
	struct device *dev,
	struct keystone_serdes_config *serdes_config)
{
	int res;
	struct keystone_serdes_lane_rx_config rx_coeff;

	dev_dbg(dev, "SerDes: fix unstable lane %d\n", lane);

	/* Get serdes current Rx boost/att */
	k2_rio_serdes_get_att_boost(lane, regs, &rx_coeff);

	/* Display Rx att/boost values */
	dev_dbg(dev,
		"SerDes: current Rx att = %d, boost = %d for lane %d\n",
		rx_coeff.att, rx_coeff.boost, lane);

	/* Force signal detect low, reset CDR, attenuation and boost */
	k2_rio_serdes_reset_cdr_att_boost(lane, regs);

	usleep_range(10, 50);

	/* Allow SerDes to re-acquire signal detect */
	k2_rio_serdes_reacquire_sd(lane, regs);

	dev_dbg(dev, "SerDes: lane %d Rx path reset done\n", lane);

	/* Force boost calibration */
	res = k2_rio_serdes_force_boost_calibration(lane, regs);
	if (res) {
		dev_warn(dev,
			 "SerDes: lane %d boost calibration workaround failed\n",
			 lane);
		return;
	}

	dev_dbg(dev, "SerDes: lane %d boost/att forcing done\n", lane);

	/* Get serdes current Rx boost/att */
	k2_rio_serdes_get_att_boost(lane, regs, &rx_coeff);

	/* Display Rx att/boost values */
	dev_dbg(dev,
		"SerDes: current Rx att = %d, boost = %d for lane %d\n",
		rx_coeff.att, rx_coeff.boost, lane);

	dev_dbg(dev, "SerDes: lane %d fixed\n", lane);
}

static void k2_rio_serdes_fix_unstable_lanes(
	u32 lanes,
	struct keystone_serdes_data *serdes)
{
	struct device *dev = serdes->dev;
	void __iomem *regs = serdes->regs;
	unsigned int stat;
	unsigned int i_dlpf;
	u32 lane;

	for_each_lanes(lanes, lane) {
		/* Check Rx valid */
		stat = (k2_rio_serdes_sb_read_selected_tbus(regs,
							    lane + 1,
							    0x2) & 0x0060) >> 5;

		dev_dbg(dev, "SerDes: check rx valid for lane %d, stat = %d\n",
			lane, stat);

		if (stat == 3) {
			i_dlpf = k2_rio_serdes_sb_read_selected_tbus(
				regs, lane + 1, 5) >> 10;

			if ((i_dlpf == 0) || (i_dlpf == 3))
				__k2_rio_serdes_fix_unstable_lane(
					lane, regs, dev, serdes->config);
		}
	}
}

static int k2_rio_serdes_start_tx_lanes(
	u32 lanes,
	struct keystone_serdes_data *serdes)
{
	struct device *dev = serdes->dev;
	void __iomem *regs = serdes->regs;
	u32 lane;
	int res;

	for_each_lanes(lanes, lane) {
		dev_dbg(dev, "SerDes: start transmit for lane %d\n", lane);

		/* Serdes assert reset */
		k2_rio_serdes_sb_assert_reset(lane, regs);

		/* Set the Tx coefficients */
		k2_rio_serdes_config_set_tx_coeffs(lane, regs,
						   &(serdes->config->tx[lane]));

		dev_dbg(dev,
			"SerDes: lane %d: c1 = %d, c2 = %d, cm = %d, att = %d, 1lsb = %d, vreg = %d\n",
			lane,
			serdes->config->tx[lane].c1_coeff,
			serdes->config->tx[lane].c2_coeff,
			serdes->config->tx[lane].cm_coeff,
			serdes->config->tx[lane].att,
			serdes->config->tx[lane].pre_1lsb,
			serdes->config->tx[lane].vreg);

		/* Force Tx normal to enable the transmitter */
		k2_rio_serdes_force_tx_normal(lane, regs);

		/* Serdes de-assert reset */
		k2_rio_serdes_sb_deassert_reset(lane, regs, 0);

		/* Enable corresponding lane */
		k2_rio_serdes_lane_enable(lane, regs);
	}

	/* Clear overlay bit to bring lanes out of reset */
	for_each_lanes(lanes, lane)
		k2_rio_serdes_sb_clear_overlay_bit29(lane, regs);

	/* Wait lanes OK */
	res = k2_rio_serdes_wait_lanes_ok(lanes, regs);
	if (res < 0) {
		dev_dbg(dev, "SerDes: %s() lane mask 0x%x not OK\n",
			__func__, lanes);
		return res;
	}

	return res;
}

static const struct keystone_serdes_ops k2_serdes_ops = {
	.config_lanes       = k2_rio_serdes_config_lanes,
	.start_tx_lanes     = k2_rio_serdes_start_tx_lanes,
	.wait_lanes_ok      = k2_rio_serdes_wait_lanes_sd,
	.shutdown_lanes     = k2_rio_serdes_shutdown_lanes,
	.fix_unstable_lanes = k2_rio_serdes_fix_unstable_lanes,
	.calibrate_lanes    = k2_rio_serdes_calibrate_lanes,
};

/*---------------------------------------------------------------------------*/

/* Sysfs management */
struct serdes_attribute {
	struct attribute attr;
	ssize_t (*show)(struct kobject *kobj,
			struct serdes_attribute *attr,
			char *buf);
	ssize_t	(*store)(struct kobject *kobj,
			 struct serdes_attribute *attr,
			 const char *,
			 size_t);
	struct keystone_serdes_data *serdes;
	void *context;
};

#define __SERDES_ATTR(_name, _mode, _show, _store, _ctxt) \
	{						\
		.attr = {				\
			.name = __stringify(_name),	\
			.mode = _mode },		\
		.show	= _show,			\
		.store	= _store,		\
		.context = (_ctxt),			\
	 }

#define to_serdes_attr(_attr) container_of(_attr, struct serdes_attribute, attr)

static ssize_t serdes_tx_attr_show(struct kobject *kobj,
				   struct serdes_attribute *attr,
				   char *buf)
{
	struct keystone_serdes_data *serdes =
		(struct keystone_serdes_data *) attr->context;
	struct keystone_serdes_config *serdes_config = serdes->config;
	u32 lane;
	int len = 0;

	for (lane = 0; lane < KEYSTONE_SERDES_MAX_LANES; lane++) {
		u32 val = -1;

		if (strcmp("c1", attr->attr.name) == 0)
			val = serdes_config->tx[lane].c1_coeff;
		if (strcmp("c2", attr->attr.name) == 0)
			val = serdes_config->tx[lane].c2_coeff;
		if (strcmp("cm", attr->attr.name) == 0)
			val = serdes_config->tx[lane].cm_coeff;
		if (strcmp("pre_1lsb", attr->attr.name) == 0)
			val = serdes_config->tx[lane].pre_1lsb;
		if (strcmp("att", attr->attr.name) == 0)
			val = serdes_config->tx[lane].att;
		if (strcmp("vreg", attr->attr.name) == 0)
			val = serdes_config->tx[lane].vreg;

		if (lane == 0)
			len = snprintf(buf + len, PAGE_SIZE, "%d", val);
		else
			len += snprintf(buf + len, PAGE_SIZE, " %d", val);
	}

	len += snprintf(buf + len, PAGE_SIZE, "\n");

	return len;
}

static ssize_t serdes_tx_attr_store(struct kobject *kobj,
				    struct serdes_attribute *attr,
				    const char *buf,
				    size_t size)
{
	struct keystone_serdes_data *serdes =
		(struct keystone_serdes_data *) attr->context;
	struct keystone_serdes_config *serdes_config = serdes->config;
	u32 lane;
	u32 val[4];

	if (sscanf(buf, "%d %d %d %d", &val[0], &val[1], &val[2], &val[3]) < 4)
		return -EINVAL;

	for (lane = 0; lane < KEYSTONE_SERDES_MAX_LANES; lane++) {

		/* Update Tx coefficients */
		if (strcmp("c1", attr->attr.name) == 0)
			serdes_config->tx[lane].c1_coeff = val[lane];
		if (strcmp("c2", attr->attr.name) == 0)
			serdes_config->tx[lane].c2_coeff = val[lane];
		if (strcmp("cm", attr->attr.name) == 0)
			serdes_config->tx[lane].cm_coeff = val[lane];
		if (strcmp("pre_1lsb", attr->attr.name) == 0)
			serdes_config->tx[lane].pre_1lsb = val[lane];
		if (strcmp("att", attr->attr.name) == 0)
			serdes_config->tx[lane].att = val[lane];
		if (strcmp("vreg", attr->attr.name) == 0)
			serdes_config->tx[lane].vreg = val[lane];

		/* Serdes assert reset */
		k2_rio_serdes_sb_assert_full_reset(lane, serdes->regs);

		/* Apply the Tx coefficients */
		k2_rio_serdes_config_set_tx_coeffs(lane,
						   serdes->regs,
						   &(serdes_config->tx[lane]));

		/* Serdes de-assert reset */
		k2_rio_serdes_sb_deassert_reset(lane, serdes->regs, 1);
	}

	return size;
}

static ssize_t serdes_rx_attr_show(struct kobject *kobj,
				   struct serdes_attribute *attr,
				   char *buf)
{
	struct keystone_serdes_lane_rx_config rx_coeff;
	struct keystone_serdes_data *serdes =
		(struct keystone_serdes_data *) attr->context;
	u32 lane;
	int len = 0;

	memset(&rx_coeff, -1, sizeof(rx_coeff));

	for (lane = 0; lane < KEYSTONE_SERDES_MAX_LANES; lane++) {
		u32 val;

		/* Get current Rx att/boost values */
		k2_rio_serdes_get_att_boost(lane, serdes->regs, &rx_coeff);

		/* If lanes are not started yet use the calibrated values */
		if (rx_coeff.att == 0)
			rx_coeff.att = serdes->config->rx[lane].mean_att;

		if (rx_coeff.boost == 0)
			rx_coeff.boost = serdes->config->rx[lane].mean_boost;

		/* If coefficient is not set, set it as zero (lane disabled) */
		if (rx_coeff.att == -1)
			rx_coeff.att = 0;

		if (rx_coeff.boost == -1)
			rx_coeff.boost = 0;

		if (strcmp("att", attr->attr.name) == 0)
			val = rx_coeff.att;
		else
			val = rx_coeff.boost;

		if (lane == 0)
			len = snprintf(buf + len, PAGE_SIZE, "%d", val);
		else
			len += snprintf(buf + len, PAGE_SIZE, " %d", val);
	}

	len += snprintf(buf + len, PAGE_SIZE, "\n");

	return len;
}

static ssize_t serdes_rx_attr_store(struct kobject *kobj,
				    struct serdes_attribute *attr,
				    const char *buf,
				    size_t size)
{
	struct keystone_serdes_data *serdes =
		(struct keystone_serdes_data *) attr->context;
	struct keystone_serdes_config *serdes_config = serdes->config;
	u32 lane;
	u32 lanes = 0;
	u32 val[4];

	if (sscanf(buf, "%d %d %d %d", &val[0], &val[1], &val[2], &val[3]) < 4)
		return -EINVAL;

	for (lane = 0; lane < KEYSTONE_SERDES_MAX_LANES; lane++) {
		/* Set att or boost coefficients */
		if (strcmp("att", attr->attr.name) == 0)
			serdes_config->rx[lane].mean_att   = val[lane];
		else
			serdes_config->rx[lane].mean_boost = val[lane];

		lanes |= BIT(lane);
	}

	return size;
}

static ssize_t serdes_n_attr_show(struct kobject *kobj,
				  struct attribute *attr,
				  char *buf)
{
	struct serdes_attribute *attribute = to_serdes_attr(attr);

	if (!attribute->show)
		return -EIO;

	return attribute->show(kobj, attribute, buf);
}

static ssize_t serdes_n_attr_store(struct kobject *kobj,
				   struct attribute *attr,
				   const char *buf,
				   size_t count)
{
	struct serdes_attribute *attribute = to_serdes_attr(attr);

	if (!attribute->store)
		return -EIO;

	return attribute->store(kobj, attribute, buf, count);
}

static const struct sysfs_ops serdes_sysfs_ops = {
	.show  = serdes_n_attr_show,
	.store = serdes_n_attr_store,
};

static struct serdes_attribute serdes_rx_att_attribute =
	__SERDES_ATTR(att, S_IRUGO | S_IWUSR,
		      serdes_rx_attr_show,
		      serdes_rx_attr_store,
		      NULL);

static struct serdes_attribute serdes_rx_boost_attribute =
	__SERDES_ATTR(boost, S_IRUGO | S_IWUSR,
		      serdes_rx_attr_show,
		      serdes_rx_attr_store,
		      NULL);

static struct serdes_attribute serdes_tx_cm_attribute =
	__SERDES_ATTR(cm, S_IRUGO | S_IWUSR,
		      serdes_tx_attr_show,
		      serdes_tx_attr_store,
		      NULL);

static struct serdes_attribute serdes_tx_c1_attribute =
	__SERDES_ATTR(c1, S_IRUGO | S_IWUSR,
		      serdes_tx_attr_show,
		      serdes_tx_attr_store,
		      NULL);

static struct serdes_attribute serdes_tx_c2_attribute =
	__SERDES_ATTR(c2, S_IRUGO | S_IWUSR,
		      serdes_tx_attr_show,
		      serdes_tx_attr_store,
		      NULL);

static struct serdes_attribute serdes_tx_att_attribute =
	__SERDES_ATTR(att, S_IRUGO | S_IWUSR,
		      serdes_tx_attr_show,
		      serdes_tx_attr_store,
		      NULL);

static struct serdes_attribute serdes_tx_lsb_attribute =
	__SERDES_ATTR(pre_1lsb, S_IRUGO | S_IWUSR,
		      serdes_tx_attr_show,
		      serdes_tx_attr_store,
		      NULL);

static struct serdes_attribute serdes_tx_vreg_attribute =
	__SERDES_ATTR(vreg, S_IRUGO | S_IWUSR,
		      serdes_tx_attr_show,
		      serdes_tx_attr_store,
		      NULL);

static struct attribute *serdes_rx_attrs[] = {
	&serdes_rx_att_attribute.attr,
	&serdes_rx_boost_attribute.attr,
	NULL
};

static struct attribute *serdes_tx_attrs[] = {
	&serdes_tx_cm_attribute.attr,
	&serdes_tx_c1_attribute.attr,
	&serdes_tx_c2_attribute.attr,
	&serdes_tx_att_attribute.attr,
	&serdes_tx_lsb_attribute.attr,
	&serdes_tx_vreg_attribute.attr,
	NULL
};

static struct kobj_type serdes_rx_type = {
	.sysfs_ops     = &serdes_sysfs_ops,
	.default_attrs = serdes_rx_attrs,
};

static struct kobj_type serdes_tx_type = {
	.sysfs_ops     = &serdes_sysfs_ops,
	.default_attrs = serdes_tx_attrs,
};

static int keystone_rio_serdes_sysfs_create(
	struct keystone_serdes_data *serdes,
	struct keystone_serdes_config *serdes_config)
{
	struct device *dev = serdes->dev;
	int res;

	serdes->serdes_kobj = kobject_create_and_add(
		"serdes",
		kobject_get(&dev->kobj));

	if (!serdes->serdes_kobj) {
		dev_err(dev, "unable create sysfs serdes file\n");
		kobject_put(&dev->kobj);
		return -ENOMEM;
	}

	/* Rx entries */
	serdes_rx_att_attribute.context   = (void *) serdes;
	serdes_rx_boost_attribute.context = (void *) serdes;

	res = kobject_init_and_add(&serdes->serdes_rx_kobj,
				   &serdes_rx_type,
				   kobject_get(serdes->serdes_kobj),
				   "rx");
	if (res) {
		dev_err(dev, "unable create sysfs serdes/rx files\n");
		kobject_put(&dev->kobj);
		kobject_put(serdes->serdes_kobj);
		return res;
	}

	/* Tx entries */
	serdes_tx_cm_attribute.context   = (void *) serdes;
	serdes_tx_c1_attribute.context   = (void *) serdes;
	serdes_tx_c2_attribute.context   = (void *) serdes;
	serdes_tx_att_attribute.context  = (void *) serdes;
	serdes_tx_lsb_attribute.context  = (void *) serdes;
	serdes_tx_vreg_attribute.context = (void *) serdes;

	res = kobject_init_and_add(&serdes->serdes_tx_kobj,
				   &serdes_tx_type,
				   kobject_get(serdes->serdes_kobj),
				   "tx");
	if (res) {
		dev_err(dev, "unable create sysfs serdes/tx files\n");
		kobject_put(&dev->kobj);
		kobject_put(serdes->serdes_kobj);
		return res;
	}

	return 0;
}

/*---------------------------------------------------------------------------*/

int keystone_rio_serdes_register(u16 serdes_type,
				 void __iomem *regs,
				 void __iomem *sts_reg,
				 struct device *dev,
				 struct keystone_serdes_data *serdes,
				 struct keystone_serdes_config *serdes_config)
{
	int res = 0;

	memset(serdes, 0, sizeof(*serdes));

	if (serdes == NULL)
		return -EINVAL;

	switch (serdes_type) {
	case KEYSTONE_SERDES_TYPE_K1:
		serdes->ops = &k1_serdes_ops;
		break;
	case KEYSTONE_SERDES_TYPE_K2:
		serdes->ops = &k2_serdes_ops;
		break;
	default:
		res = -EINVAL;
	}

	if (res)
		goto error;

	serdes->dev     = dev;
	serdes->config  = serdes_config;
	serdes->regs    = regs;
	serdes->sts_reg = sts_reg;

	res = keystone_rio_serdes_sysfs_create(serdes, serdes_config);

error:
	return res;
}
