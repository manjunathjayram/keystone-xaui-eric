/*
 * Driver for Marvell PHY 88X2242
 *
 * Partially based on drivers/net/phy/bcm87xx.c
 *
 * Copyright (C) 2013 Texas Instruments Incorporated
 * Authors: WingMan Kwok <w-kwok2@ti.com>
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
#include <linux/phy.h>
#include <linux/of.h>

#define MARVELL_PHY_ID_88X2242	0x01410db0

#define MV88X2242_PMD_RX_SIGNAL_DETECT	(MII_ADDR_C45 | 0x1000a)
#define MV88X2242_10GBASER_PCS_STATUS	(MII_ADDR_C45 | 0x30020)
#define MV88X2242_XGXS_LANE_STATUS	(MII_ADDR_C45 | 0x40018)

#define MV88X2242_LASI_CONTROL		(MII_ADDR_C45 | 0x38000)
#define MV88X2242_LASI_STATUS		(MII_ADDR_C45 | 0x38001)

#if IS_ENABLED(CONFIG_OF_MDIO)
/* Set and/or override some configuration registers based on the
 * marvell,c45-reg-init property stored in the of_node for the phydev.
 *
 * marvell,c45-reg-init = <devid reg mask value>,...;
 *
 * There may be one or more sets of <devid reg mask value>:
 *
 * devid: which sub-device to use.
 * reg: the register.
 * mask: if non-zero, ANDed with existing register value.
 * value: ORed with the masked value and written to the regiser.
 *
 */
static int mv88x2242_of_reg_init(struct phy_device *phydev)
{
	const __be32 *paddr;
	const __be32 *paddr_end;
	int len, ret;

	if (!phydev->dev.of_node)
		return 0;

	paddr = of_get_property(phydev->dev.of_node,
				"marvell,c45-reg-init", &len);
	if (!paddr)
		return 0;

	paddr_end = paddr + (len /= sizeof(*paddr));

	ret = 0;

	while (paddr + 3 < paddr_end) {
		u16 devid	= be32_to_cpup(paddr++);
		u16 reg		= be32_to_cpup(paddr++);
		u16 mask	= be32_to_cpup(paddr++);
		u16 val_bits	= be32_to_cpup(paddr++);
		int val;
		u32 regnum = MII_ADDR_C45 | (devid << 16) | reg;
		val = 0;
		if (mask) {
			val = phy_read(phydev, regnum);
			if (val < 0) {
				ret = val;
				goto err;
			}
			val &= mask;
		}
		val |= val_bits;

		ret = phy_write(phydev, regnum, val);
		if (ret < 0)
			goto err;
	}
err:
	return ret;
}
#else
static int mv88x2242_of_reg_init(struct phy_device *phydev)
{
	return 0;
}
#endif /* CONFIG_OF_MDIO */

static int mv88x2242_config_init(struct phy_device *phydev)
{
	phydev->supported = SUPPORTED_10000baseR_FEC;
	phydev->advertising = ADVERTISED_10000baseR_FEC;
	phydev->state = PHY_NOLINK;
	phydev->autoneg = AUTONEG_DISABLE;

	mv88x2242_of_reg_init(phydev);

	return 0;
}

static int mv88x2242_config_aneg(struct phy_device *phydev)
{
	return -EINVAL;
}

static int mv88x2242_read_status(struct phy_device *phydev)
{
	int rx_signal_detect;
	int pcs_status;
/*	int xgxs_lane_status;*/

	rx_signal_detect = phy_read(phydev, MV88X2242_PMD_RX_SIGNAL_DETECT);
	if (rx_signal_detect < 0)
		return rx_signal_detect;

	if ((rx_signal_detect & 1) == 0)
		goto no_link;

	pcs_status = phy_read(phydev, MV88X2242_10GBASER_PCS_STATUS);
	if (pcs_status < 0)
		return pcs_status;

	if ((pcs_status & 1) == 0)
		goto no_link;

#if 0
	xgxs_lane_status = phy_read(phydev, MV88X2242_XGXS_LANE_STATUS);
	if (xgxs_lane_status < 0)
		return xgxs_lane_status;

	if ((xgxs_lane_status & 0x1000) == 0)
		goto no_link;
#endif

	phydev->speed = 10000;
	phydev->link = 1;
	phydev->duplex = 1;
	return 0;

no_link:
	phydev->link = 0;
	return 0;
}

static int mv88x2242_config_intr(struct phy_device *phydev)
{
	int reg, err;

	reg = phy_read(phydev, MV88X2242_LASI_CONTROL);

	if (reg < 0)
		return reg;

	if (phydev->interrupts == PHY_INTERRUPT_ENABLED)
		reg |= 1;
	else
		reg &= ~1;

	err = phy_write(phydev, MV88X2242_LASI_CONTROL, reg);
	return err;
}

static int mv88x2242_did_interrupt(struct phy_device *phydev)
{
	int reg;

	reg = phy_read(phydev, MV88X2242_LASI_STATUS);

	if (reg < 0) {
		dev_err(&phydev->dev,
			"Error: Read of MV88X2242_LASI_STATUS failed: %d\n",
			 reg);
		return 0;
	}
	return (reg & 1) != 0;
}

static int mv88x2242_ack_interrupt(struct phy_device *phydev)
{
	/* Reading the LASI status clears it. */
	mv88x2242_did_interrupt(phydev);
	return 0;
}

static int mv88x2242_match_phy_device(struct phy_device *phydev)
{
	return phydev->c45_ids.device_ids[4] == MARVELL_PHY_ID_88X2242;
}

static struct phy_driver mv88x2242_driver[] = {
{
	.phy_id		= MARVELL_PHY_ID_88X2242,
	.phy_id_mask	= 0xfffffff0,
	.name		= "Marvell 88x2242",
	.flags		= PHY_HAS_INTERRUPT,
	.config_init	= mv88x2242_config_init,
	.config_aneg	= mv88x2242_config_aneg,
	.read_status	= mv88x2242_read_status,
	.ack_interrupt	= mv88x2242_ack_interrupt,
	.config_intr	= mv88x2242_config_intr,
	.did_interrupt	= mv88x2242_did_interrupt,
	.match_phy_device = mv88x2242_match_phy_device,
	.driver		= { .owner = THIS_MODULE },
}
};

static int __init mv88x2242_init(void)
{
	return phy_drivers_register(mv88x2242_driver,
		ARRAY_SIZE(mv88x2242_driver));
}
module_init(mv88x2242_init);

static void __exit mv88x2242_exit(void)
{
	phy_drivers_unregister(mv88x2242_driver,
		ARRAY_SIZE(mv88x2242_driver));
}
module_exit(mv88x2242_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("WingMan Kwok <w-kwok2@ti.com>");
MODULE_DESCRIPTION("Driver For Marvell PHY 88X2242");
