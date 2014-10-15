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
	struct device *dev,
	void __iomem *regs,
	void __iomem *sts_reg,
	struct keystone_serdes_config *serdes_config)
{
	u32 lane;
	int res = 0;
	u32 val;
	unsigned long timeout;
	struct k1_rio_serdes_regs *serdes_regs =
		(struct k1_rio_serdes_regs *) regs;

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
		val = __raw_readl(sts_reg);

		if ((val & 0x1) != 0x1)
			break;

		if (time_after(jiffies, timeout)) {
			res = -1;
			break;
		}

		usleep_range(10, 50);
	}

	return res;
}

static int k1_rio_serdes_start_tx_lanes(
	u32 lanes,
	struct device *dev,
	void __iomem *regs,
	struct keystone_serdes_config *serdes_config)
{
	return 0;
}

static int k1_rio_serdes_wait_lanes_ok(u32 lanes, void __iomem *regs)
{
	return 0;
}

static int k1_rio_serdes_shutdown_lanes(u32 lanes, void __iomem *regs)
{
	return 0;
}

static void k1_rio_serdes_fix_unstable_lanes(u32 lanes,
					     struct device *dev,
					     void __iomem *regs)
{
	return;
}

static const struct keystone_serdes_ops k1_serdes_ops = {
	.config_lanes       = k1_rio_serdes_config_lanes,
	.start_tx_lanes     = k1_rio_serdes_start_tx_lanes,
	.wait_lanes_ok      = k1_rio_serdes_wait_lanes_ok,
	.shutdown_lanes     = k1_rio_serdes_shutdown_lanes,
	.fix_unstable_lanes = k1_rio_serdes_fix_unstable_lanes,
};

/*-------------------------- KeyStone 2 SerDes functions --------------------*/

/*
 * Main code to Read TBUS on PHY-A and generate attenuation and boost values
 * for a lane given at serdes_base_address and lane_no
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
 * Wait SerDes RX valid
 * To be performed after SerDes is configured and bit lock achieved
 */
static int k2_rio_serdes_wait_rx_valid(void __iomem *regs, int lane)
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
static inline void k2_rio_serdes_reacquire_sd(void __iomem *regs, int lane)
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
 * Set the TX termination
 */
static void k2_rio_serdes_termination_config(void __iomem *regs,
					     u32 lane, u32 tx_term_np)
{
	/* Set tx termination */
	reg_fill_field(regs + (lane * 0x200) + 0x200 + 0x7c, 24, 8, tx_term_np);

	/* Set termination override */
	reg_fill_field(regs + (lane * 0x200) + 0x200 + 0x7c, 20, 1, 1);
}

static void k2_rio_serdes_display_att_boost(void __iomem *regs, u32 lane,
					    struct device *dev)
{
	u32 att;
	u32 boost;

	/* Read attenuation and boost */
	att = boost = k2_rio_serdes_sb_read_selected_tbus(regs, lane + 1, 0x11);
	att   = (att   >> 4) & 0x0f;
	boost = (boost >> 8) & 0x0f;

	dev_dbg(dev, "SerDes: lane %d att = %d, boost = %d\n",
		lane, att, boost);

	return;
}

/*
 * Forcing the Tx idle on the transmitter will prevent the receiver from
 * adapting to the un-terminated Tx and un-programmed pre and post cursor
 * settings
 */
static inline void k2_rio_serdes_force_tx_idle(void __iomem *regs, u32 lane)
{
	reg_fill_field(regs + (lane * 4) + 0x1fc0 + 0x20, 24, 2, 3);
}

static inline void k2_rio_serdes_force_tx_normal(void __iomem *regs, u32 lane)
{
	reg_fill_field(regs + (lane * 4) + 0x1fc0 + 0x20, 24, 2, 0);
}

static inline void k2_rio_serdes_force_rx_disable(void __iomem *regs, u32 lane)
{
	reg_fill_field(regs + (lane * 4) + 0x1fc0 + 0x20, 15, 1, 1);
	reg_fill_field(regs + (lane * 4) + 0x1fc0 + 0x20, 13, 2, 0);
}

static inline void k2_rio_serdes_force_rx_enable(void __iomem *regs, u32 lane)
{
	reg_fill_field(regs + (lane * 4) + 0x1fc0 + 0x20, 15, 1, 1);
	reg_fill_field(regs + (lane * 4) + 0x1fc0 + 0x20, 13, 2, 3);
}

/*
 * Function to reset the Clock Data Recovery, attenuation and boost circuitry
 */
static inline void k2_rio_serdes_reset_cdr_att_boost(void __iomem *regs,
						     u32 lane)
{
	reg_finsr(regs + 0x004 + (0x200 * (lane + 1)), 2, 1, 0x2);
}

/*
 * Wait lanes to be OK by checking LNn_OK_STATE bits
 */
static int k2_rio_serdes_wait_lanes_ok(u32 lanes, void __iomem *regs)
{
	u32 val;
	unsigned long timeout;
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
			return -1;

		usleep_range(10, 50);
	}

	return 0;
}

/*
 * Wait signal detect by checking LNn_SD_STATE bits in addition to LNn_OK_STATE
 * bits
 */
static int k2_rio_serdes_wait_lanes_sd(u32 lanes, void __iomem *regs)
{
	u32 val;
	unsigned long timeout;
	u32 val_mask;

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
			return -1;

		usleep_range(10, 50);
	}

	return 0;
}

/*
 * Assert reset while preserving the lnX_ctrl_i bits
 */
static void k2_rio_serdes_sb_assert_reset(void __iomem *regs, u32 lane)
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

static inline void k2_rio_serdes_sb_assert_full_reset(void __iomem *regs,
						      u32 lane)
{
	/* Toggle bit 29 of LANE_028 */
	reg_finsr(regs + (0 * 0x200) + (1 * 0x200) + 0x200 + 0x28, 29, 15,
		  0x4260);
}

static inline int k2_rio_serdes_sb_deassert_reset(void __iomem *regs,
						  u32 block,
						  u32 lane)
{
	/* Clear the reset bit */
	reg_fill_field(regs + (lane * 0x200) + 0x200 + 0x28, 15, 1, 1);

	/* Waits for LANE OK to complete after deasserting the reset */
	if (block)
		return k2_rio_serdes_wait_lanes_ok(BIT(lane), regs);

	return 0;
}

static void k2_rio_serdes_sb_clear_overlay_bit29(void __iomem *regs, u32 lane)
{
	/* Clear overlay bit, bring the lane out of reset */
	reg_fill_field(regs + (lane * 0x200) + 0x200 + 0x28, 29, 1, 0);
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

static void k2_rio_serdes_lane_init(u32 lane, u32 rate, void __iomem *regs)
{
	/* Bring this lane out of reset by clearing override bit 29 */
	k2_rio_serdes_sb_clear_overlay_bit29(regs, lane);

	/* Set lane control rate, force lane enable, rates, width and tx idle */
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
 * Forces the calibration of the SerDes receiver settings of attenuation
 * and boost circuitry.
 *
 * This is only valid for SerDes PHY-A (i.e. all SerDes except XGE)
 */
static void k2_rio_serdes_force_att_boost(void __iomem *regs, u32 lane)
{
	u32 boost_read = 0;
	u32 att_read   = 0;
	u32 att_start  = 0;

	if (k2_rio_serdes_wait_rx_valid(regs, lane))
		return;

	/* First read initial att start value */
	att_start = (__raw_readl(regs + (lane * 0x200) + 0x200 + 0x8c)
		     >> 8) & 0xf;

	/*
	 * Check att value, fix this as start value turn off att adaptation
	 * and do boost readaptation
	 */
	att_read = (k2_rio_serdes_sb_read_selected_tbus(regs, lane + 1, 0x11)
		    >> 4) & 0xf;

	/* att_start */
	reg_fill_field(regs + (lane * 0x200) + 0x200 + 0x8c, 8, 4, att_read);
	reg_fill_field(regs + (lane * 0x200) + 0x200 + 0x84, 0, 1, 0);
	reg_fill_field(regs + 0x0a00 + 0x8c, 24, 1, 0);

	/* Force cal */
	reg_fill_field(regs + (lane * 0x200) + 0x200 + 0xac, 11, 1, 1);
	reg_fill_field(regs + (lane * 0x200) + 0x200 + 0xac, 11, 1, 0);

	k2_rio_serdes_wait_rx_valid(regs, lane);

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
	/* Write back initial att start value */
	reg_fill_field(regs + (lane * 0x200) + 0x200 + 0x8c,  8, 4, att_start);

	/* Turn back on att adaptation */
	reg_fill_field(regs + (lane * 0x200) + 0x200 + 0x84,  0, 1, 1);
	reg_fill_field(regs + 0x0a00 + 0x8c, 24, 1, 1);

	k2_rio_serdes_wait_rx_valid(regs, lane);
}

static inline void k2_rio_serdes_config_set_tx_coeffs(
	u32 lane,
	void __iomem *regs,
	struct keystone_serdes_lane_tx_config *tx_config)
{
	u32 val;

	/* Set C1/C2/CM coefficients */
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

/*
 * Configure SerDes with appropriate baudrate and do Tx termination workaround
 * Note that all lanes are configured but Serdes are then disabled
 */
static int k2_rio_serdes_config_lanes(
	u32 lanes,
	u32 baud,
	struct device *dev,
	void __iomem *regs,
	void __iomem *sts_reg,
	struct keystone_serdes_config *serdes_config)
{
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

	/* We need to always use lane 0 even if not part of the lane mask */
	lanes |= KEYSTONE_SERDES_LANE(0);

	/* Disable transmitter for all lanes */
	for_each_lanes(lanes, lane) {
		k2_rio_serdes_force_tx_idle(regs, lane);
	}

	/* Initialize SerDes for requested lanes */
	for_each_lanes(lanes, lane) {
		k2_rio_serdes_lane_init(lane, rate, regs);
	}

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
		return -1;
	}

	/* Read the Tx termination */
	tx_term_np = k2_rio_serdes_get_termination(regs);
	dev_dbg(dev, "SerDes: termination tx is 0x%x\n", tx_term_np);

	for_each_lanes(lanes, lane) {
		/* Set the saved Tx termination */
		k2_rio_serdes_termination_config(regs, lane, tx_term_np);

		/* Disable SerDes for this lane */
		k2_rio_serdes_lane_disable(lane, regs);
	}

	return 0;
}

static int k2_rio_serdes_shutdown_lanes(u32 lanes, void __iomem *regs)
{
	u32 lane;

	/* Disable lanes */
	for_each_lanes(lanes, lane) {
		k2_rio_serdes_lane_disable(lane, regs);
	}

	/* Disable PLL */
	__raw_writel(0x80000000, regs + 0x1ff4);

	/* Reset CMU PLL for all lanes */
	reg_rmw(regs + 0x10, BIT(28), BIT(28));

	return 0;
}

static void k2_rio_serdes_fix_unstable_single_lane(int lane,
						   struct device *dev,
						   void __iomem *regs)
{
	dev_dbg(dev, "SerDes: fix unstable lane %d\n", lane);

	/* Display serdes boost/att */
	k2_rio_serdes_display_att_boost(regs, lane, dev);

	/* Force SerDes signal detect LO (reset CDR, Att and Boost) */
	reg_fill_field(regs + (lane * 0x200) + 0x200 + 0x04, 1, 2, 2);

	usleep_range(10, 50);

	/* Allow SerDes to re-acquire signal detect */
	k2_rio_serdes_reacquire_sd(regs, lane);

	dev_dbg(dev, "SerDes: lane %d Rx path reset done\n", lane);

	/* Force att/boost calibration */
	k2_rio_serdes_force_att_boost(regs, lane);

	dev_dbg(dev, "SerDes: lane %d boost/att forcing done\n", lane);

	/* Display serdes boost/att */
	k2_rio_serdes_display_att_boost(regs, lane, dev);

	dev_dbg(dev, "SerDes: lane %d fixed\n", lane);
}

static void k2_rio_serdes_fix_unstable_lanes(u32 lanes,
					     struct device *dev,
					     void __iomem *regs)
{
	unsigned int stat;
	unsigned int i_dlpf;
	u32 lane;

	for_each_lanes(lanes, lane) {
		/* Check rx valid */
		stat = (k2_rio_serdes_sb_read_selected_tbus(regs,
							    lane + 1,
							    0x2) & 0x0060) >> 5;

		dev_dbg(dev, "SerDes: check rx valid for lane %d, stat = %d\n",
			lane, stat);

		if (stat == 3) {
			i_dlpf = k2_rio_serdes_sb_read_selected_tbus(
				regs, lane + 1, 5) >> 10;

			if ((i_dlpf == 0) || (i_dlpf == 3))
				k2_rio_serdes_fix_unstable_single_lane(
					lane, dev, regs);
		}
	}
}

static int k2_rio_serdes_start_tx_lanes(
	u32 lanes,
	struct device *dev,
	void __iomem *regs,
	struct keystone_serdes_config *serdes_config)
{
	u32 lane;
	int res;

	for_each_lanes(lanes, lane) {
		dev_dbg(dev, "SerDes: start transmit for lane %d\n", lane);

		/* Serdes assert reset */
		k2_rio_serdes_sb_assert_reset(regs, lane);

		/* Set tx coefficients */
		k2_rio_serdes_config_set_tx_coeffs(
			lane, regs, &(serdes_config->lane[lane]));

		dev_dbg(dev,
			"SerDes: lane %d: c1 = %d, c2 = %d, cm = %d, att = %d, 1lsb = %d, vreg = %d\n",
			lane,
			serdes_config->lane[lane].c1_coeff,
			serdes_config->lane[lane].c2_coeff,
			serdes_config->lane[lane].cm_coeff,
			serdes_config->lane[lane].att,
			serdes_config->lane[lane].pre_1lsb,
			serdes_config->lane[lane].vreg);

		/* Force Tx normal to enable the transmitter */
		k2_rio_serdes_force_tx_normal(regs, lane);

		/* Serdes de-assert reset */
		k2_rio_serdes_sb_deassert_reset(regs, 0, lane);

		/* Enable corresponding lane */
		k2_rio_serdes_lane_enable(lane, regs);
	}

	/* Clear overlay bit to bring lane out of reset */
	for_each_lanes(lanes, lane) {
		k2_rio_serdes_sb_clear_overlay_bit29(regs, lane);
	}

	/* Wait lanes OK */
	res = k2_rio_serdes_wait_lanes_ok(lanes, regs);
	if (res < 0) {
		dev_dbg(dev, "SerDes: %s() lane mask 0x%x not OK\n",
			__func__, lanes);
		return -1;
	}

	return 0;
}

static const struct keystone_serdes_ops k2_serdes_ops = {
	.config_lanes       = k2_rio_serdes_config_lanes,
	.start_tx_lanes     = k2_rio_serdes_start_tx_lanes,
	.wait_lanes_ok      = k2_rio_serdes_wait_lanes_sd,
	.shutdown_lanes     = k2_rio_serdes_shutdown_lanes,
	.fix_unstable_lanes = k2_rio_serdes_fix_unstable_lanes,
};

/*---------------------------------------------------------------------------*/

int keystone_rio_serdes_register(u16 serdes_type,
				 const struct keystone_serdes_ops **p_ops)
{
	int res = 0;

	if (p_ops == NULL)
		return -EINVAL;

	switch (serdes_type) {
	case KEYSTONE_SERDES_TYPE_K1:
		*p_ops = &k1_serdes_ops;
		break;
	case KEYSTONE_SERDES_TYPE_K2:
		*p_ops = &k2_serdes_ops;
		break;
	default:
		res = -EINVAL;
	}

	return res;
}
