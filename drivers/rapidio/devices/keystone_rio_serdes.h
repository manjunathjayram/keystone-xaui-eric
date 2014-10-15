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
#ifndef KEYSTONE_RIO_SERDES_H
#define KEYSTONE_RIO_SERDES_H

#define KEYSTONE_SERDES_TYPE_K1      0   /* KeyStone 1 SerDes */
#define KEYSTONE_SERDES_TYPE_K2      1   /* KeyStone 2 SerDes */

#define KEYSTONE_SERDES_LANE(lane)   (BIT(lane))
#define KEYSTONE_SERDES_MAX_LANES    4   /* Max number of lanes for SRIO */
#define KEYSTONE_SERDES_LANE_MASK    0xf /* Lane mask */

#define KEYSTONE_SERDES_TIMEOUT      100 /* 100ms timeout */

/* Supported baud rates */
#define KEYSTONE_SERDES_BAUD_1_250   0
#define KEYSTONE_SERDES_BAUD_2_500   1
#define KEYSTONE_SERDES_BAUD_3_125   2
#define KEYSTONE_SERDES_BAUD_5_000   3
#define KEYSTONE_SERDES_BAUD_6_250   4

#define KEYSTONE_SERDES_FULL_RATE    0
#define KEYSTONE_SERDES_HALF_RATE    1
#define KEYSTONE_SERDES_QUARTER_RATE 2

/*
 * SerDes PHY transmistter configuration
 */
struct keystone_serdes_lane_tx_config {
	u32 pre_1lsb;		/* 1 lsb pre-emphasis */
	u32 c1_coeff;		/* C1 coefficient */
	u32 c2_coeff;		/* C2 coefficient */
	u32 cm_coeff;		/* CM coefficient */
	u32 att;		/* attenuator */
	u32 vreg;		/* regulator voltage */
};

struct keystone_serdes_config {
	u32 cfg_cntl;           /* setting control reg cfg */
	u16 serdes_cfg_pll;     /* SerDes PLL cfg */
	u16 prescalar_srv_clk;  /* prescalar fo ip_clk */

	/* Per-lane PHY Tx coefficients */
	struct keystone_serdes_lane_tx_config lane[KEYSTONE_SERDES_MAX_LANES];
};

/*
 * SerDes ops
 */
struct keystone_serdes_ops {
	int (*config_lanes)(u32 lanes,
			    u32 baud,
			    struct device *dev,
			    void __iomem *regs,
			    void __iomem *sts_reg,
			    struct keystone_serdes_config *serdes_config);
	int (*start_tx_lanes)(u32 lanes,
			      struct device *dev,
			      void __iomem *regs,
			      struct keystone_serdes_config *serdes_config);
	int (*wait_lanes_ok)(u32 lanes, void __iomem *regs);
	int (*shutdown_lanes)(u32 lanes, void __iomem *regs);
	void (*fix_unstable_lanes)(u32 lanes,
				   struct device *dev,
				   void __iomem *regs);
};

extern int keystone_rio_serdes_register(
	u16 serdes_type,
	const struct keystone_serdes_ops **p_ops);

#endif /* KEYSTONE_RIO_SERDES_H */
