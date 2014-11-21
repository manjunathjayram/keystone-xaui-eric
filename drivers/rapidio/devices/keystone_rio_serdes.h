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

#define KEYSTONE_SERDES_QUARTER_RATE 0
#define KEYSTONE_SERDES_HALF_RATE    1
#define KEYSTONE_SERDES_FULL_RATE    2

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
	u32 vdreg;              /* output swing voltage regulator */
};

struct keystone_serdes_lane_rx_config {
	u32 att;                /* current attenuator */
	u32 boost;              /* current boost */
	u32 mean_att;           /* mean request attenuator start coeff */
	u32 mean_boost;         /* mean request boost start coeff */
	u32 start_att;          /* attenuator start value */
	u32 start_boost;        /* boost start value */
};

struct keystone_serdes_config {
	u16 prescalar_srv_clk;  /* prescalar fo ip_clk */

	/* Per-lane PHY Tx and Rx coefficients */
	struct keystone_serdes_lane_tx_config tx[KEYSTONE_SERDES_MAX_LANES];
	struct keystone_serdes_lane_rx_config rx[KEYSTONE_SERDES_MAX_LANES];

	u32 cal_timeout;        /* calibration timeout */
	int do_dfe_cal;         /* if 1, perform DFE offset calibration */
	u32 rate;               /* Configurated rate */
};

struct keystone_serdes_data;

/*
 * SerDes ops
 */
struct keystone_serdes_ops {
	int (*config_lanes)(u32 lanes,
			    u32 baud,
			    struct keystone_serdes_data *serdes);
	int (*start_tx_lanes)(u32 lanes,
			      struct keystone_serdes_data *serdes);
	int (*wait_lanes_ok)(u32 lanes,
			     struct keystone_serdes_data *serdes);
	int (*shutdown_lanes)(u32 lanes,
			      struct keystone_serdes_data *serdes);
	void (*fix_unstable_lanes)(u32 lanes,
				   struct keystone_serdes_data *serdes);
	int (*calibrate_lanes)(u32 lanes,
			       struct keystone_serdes_data *serdes);
};

/*
 * SerDes structure
 */
struct keystone_serdes_data {
	/* SerDes Ops */
	const struct keystone_serdes_ops *ops;

	/* Associated device */
	struct device *dev;

	/* SerDes register base and STS register for K1 */
	void __iomem *regs;
	void __iomem *sts_reg;

	/* Pointer to the SerDes configuration */
	struct keystone_serdes_config *config;

	/* Sysfs management */
	struct kobject *serdes_kobj;
	struct kobject  serdes_rx_kobj;
	struct kobject  serdes_tx_kobj;
};

extern int keystone_rio_serdes_register(
	u16 serdes_type,
	void __iomem *regs,
	void __iomem *sts_reg,
	struct device *dev,
	struct keystone_serdes_data *serdes,
	struct keystone_serdes_config *serdes_config);

#endif /* KEYSTONE_RIO_SERDES_H */
