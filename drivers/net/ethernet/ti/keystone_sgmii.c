/*
 * Copyright (C) 2012 Texas Instruments Incorporated
 * Authors: Sandeep Paulraj <s-paulraj@ti.com>
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

#define SGMII_REG_BASE                  0x02090100
#define SGMII_SERDES_BASE		0x02620340
#define SGMII_SERDES_CFGPLL		0x0
#define SGMII_SERDES_CFGRX0		0x4
#define SGMII_SERDES_CFGTX0		0x8
#define SGMII_SERDES_CFGRX1		0xC
#define SGMII_SERDES_CFGTX1		0x10
#define SGMII_SERDES_SIZE		0x14

#define SGMII_SRESET_RESET		0x1
#define SGMII_SRESET_RTRESET		0x2
#define SGMII_CTL_AUTONEG		0x01
#define SGMII_CTL_LOOPBACK		0x10
#define SGMII_CTL_MASTER		0x20
#define SGMII_REG_STATUS_LOCK		BIT(4)
#define	SGMII_REG_STATUS_LINK		BIT(0)
#define SGMII_REG_STATUS_AUTONEG	BIT(2)
#define SGMII_REG_CONTROL_AUTONEG	BIT(0)

#define SGMII23_OFFSET(x)	((x - 2) * 0x100)
#define SGMII_OFFSET(x)		((x <= 1) ? (x * 0x100) : (SGMII23_OFFSET(x)))
/*
 * SGMII registers
 */
#define SGMII_IDVER_REG(x)    (SGMII_OFFSET(x) + 0x000)
#define SGMII_SRESET_REG(x)   (SGMII_OFFSET(x) + 0x004)
#define SGMII_CTL_REG(x)      (SGMII_OFFSET(x) + 0x010)
#define SGMII_STATUS_REG(x)   (SGMII_OFFSET(x) + 0x014)
#define SGMII_MRADV_REG(x)    (SGMII_OFFSET(x) + 0x018)
#define SGMII_LPADV_REG(x)    (SGMII_OFFSET(x) + 0x020)
#define SGMII_TXCFG_REG(x)    (SGMII_OFFSET(x) + 0x030)
#define SGMII_RXCFG_REG(x)    (SGMII_OFFSET(x) + 0x034)
#define SGMII_AUXCFG_REG(x)   (SGMII_OFFSET(x) + 0x038)

static inline void sgmii_write_reg(void __iomem *base, int reg, u32 val)
{
	__raw_writel(val, base + reg);
}

static inline u32 sgmii_read_reg(void __iomem *base, int reg)
{
	return __raw_readl(base + reg);
}

static inline void sgmii_write_reg_bit(void __iomem *base, int reg, u32 val)
{
	__raw_writel((__raw_readl(base + reg) | val),
			base + reg);
}

int serdes_init(void)
{
	void __iomem *sgmii_serdes;

	sgmii_serdes = ioremap(SGMII_SERDES_BASE, SGMII_SERDES_SIZE);

	sgmii_write_reg(sgmii_serdes, SGMII_SERDES_CFGPLL, 0x00000041);

	udelay(2000);

	sgmii_write_reg(sgmii_serdes, SGMII_SERDES_CFGRX0, 0x00700621);
	sgmii_write_reg(sgmii_serdes, SGMII_SERDES_CFGRX1, 0x00700621);

	sgmii_write_reg(sgmii_serdes, SGMII_SERDES_CFGTX0, 0x000108a1);
	sgmii_write_reg(sgmii_serdes, SGMII_SERDES_CFGTX1, 0x000108a1);

	udelay(2000);

	iounmap(sgmii_serdes);

	return 0;
}

/* port is 0 based */
int keystone_sgmii_reset(void __iomem *sgmii_ofs, int port)
{
	/* Soft reset */
	sgmii_write_reg_bit(sgmii_ofs, SGMII_SRESET_REG(port), 0x1);
	while(sgmii_read_reg(sgmii_ofs, SGMII_SRESET_REG(port)) != 0x0);

	return 0;
}

/* assumes ports <= 2 */
int keystone_sgmii_link_status(void __iomem *sgmii_ofs, int ports)
{
	u32 status = 0, link = 0;
	u32 i;

	for (i = 0; i < ports; i++) {
		status = sgmii_read_reg(sgmii_ofs, SGMII_STATUS_REG(i));
		if ((status & SGMII_REG_STATUS_LINK) != 0)
			link |= BIT(i);
		else
			link &= ~BIT(i);
	}

	return link;
}

int keystone_sgmii_get_port_link(void __iomem *sgmii_ofs, int port)
{
	u32 status = 0, link = 0;

	status = sgmii_read_reg(sgmii_ofs, SGMII_STATUS_REG(port));
	if ((status & SGMII_REG_STATUS_LINK) != 0)
		link |= BIT(port);
	else
		link &= ~BIT(port);

	return link;
}


int keystone_sgmii_config(void __iomem *sgmii_ofs,
			  int port, u32 interface)
{
	unsigned int i, status, mask;
	u32 mr_adv_ability;
	u32 control;

	switch (interface) {
	case SGMII_LINK_MAC_MAC_AUTONEG:
		mr_adv_ability	= 0x9801;
		control		= 0x21;
		break;

	case SGMII_LINK_MAC_PHY:
	case SGMII_LINK_MAC_PHY_NO_MDIO:
		mr_adv_ability	= 1;
		control		= 1;
		break;

	case SGMII_LINK_MAC_MAC_FORCED:
		mr_adv_ability	= 0x9801;
		control		= 0x20;
		break;

	case SGMII_LINK_MAC_FIBER:
		mr_adv_ability	= 0x20;
		control		= 0x1;
		break;

	default:
		WARN_ONCE(1, "Invalid sgmii interface: %d\n", interface);
		return -EINVAL;
	}

	sgmii_write_reg(sgmii_ofs, SGMII_CTL_REG(port), 0);

	/*
	 * Wait for the SerDes pll to lock,
	 * but don't trap if lock is never read
	 */
	for (i = 0; i < 1000; i++)  {
		udelay(2000);
		status = sgmii_read_reg(sgmii_ofs, SGMII_STATUS_REG(port));
		if ((status & SGMII_REG_STATUS_LOCK) != 0)
			break;
	}

	sgmii_write_reg(sgmii_ofs, SGMII_MRADV_REG(port), mr_adv_ability);
	sgmii_write_reg(sgmii_ofs, SGMII_CTL_REG(port), control);


	mask = SGMII_REG_STATUS_LINK;

	if (control & SGMII_REG_CONTROL_AUTONEG)
		mask |= SGMII_REG_STATUS_AUTONEG;

	for (i = 0; i < 1000; i++)  {
		status = sgmii_read_reg(sgmii_ofs, SGMII_STATUS_REG(port));
		if ((status & mask) == mask)
			break;
	}

	return 0;
}

#define reg_rmw(addr, value, mask) \
	__raw_writel(((__raw_readl(addr) & (~(mask))) | (value) ), (addr) )

void serdes_init_6638_156p25Mhz()
{
	void __iomem *regs;
	unsigned int cnt;

	regs = ioremap(0x0232a000, 0x2000);

	reg_rmw(regs + 0x0000, 0x00800000, 0xffff0000);
	reg_rmw(regs + 0x0014, 0x00008282, 0x0000ffff);
	reg_rmw(regs + 0x0060, 0x00142438, 0x00ffffff);
	reg_rmw(regs + 0x0064, 0x00c3c700, 0x00ffff00);
	reg_rmw(regs + 0x0078, 0x0000c000, 0x0000ff00);

	reg_rmw(regs + 0x0204, 0x38000080, 0xff0000ff);
	reg_rmw(regs + 0x0208, 0x00000000, 0x000000ff);
	reg_rmw(regs + 0x020c, 0x02000000, 0xff000000);
	reg_rmw(regs + 0x0210, 0x1b000000, 0xff000000);
	reg_rmw(regs + 0x0214, 0x00006fb8, 0x0000ffff);
	reg_rmw(regs + 0x0218, 0x758000e4, 0xffff00ff);
	reg_rmw(regs + 0x02ac, 0x00004400, 0x0000ff00);
	reg_rmw(regs + 0x022c, 0x00200800, 0x00ffff00);
	reg_rmw(regs + 0x0280, 0x00820082, 0x00ff00ff);
	reg_rmw(regs + 0x0284, 0x1d0f0385, 0xffffffff);

	reg_rmw(regs + 0x0404, 0x38000080, 0xff0000ff);
	reg_rmw(regs + 0x0408, 0x00000000, 0x000000ff);
	reg_rmw(regs + 0x040c, 0x02000000, 0xff000000);
	reg_rmw(regs + 0x0410, 0x1b000000, 0xff000000);
	reg_rmw(regs + 0x0414, 0x00006fb8, 0x0000ffff);
	reg_rmw(regs + 0x0418, 0x758000e4, 0xffff00ff);
	reg_rmw(regs + 0x04ac, 0x00004400, 0x0000ff00);
	reg_rmw(regs + 0x042c, 0x00200800, 0x00ffff00);
	reg_rmw(regs + 0x0480, 0x00820082, 0x00ff00ff);
	reg_rmw(regs + 0x0484, 0x1d0f0385, 0xffffffff);

	reg_rmw(regs + 0x0604, 0x38000080, 0xff0000ff);
	reg_rmw(regs + 0x0608, 0x00000000, 0x000000ff);
	reg_rmw(regs + 0x060c, 0x02000000, 0xff000000);
	reg_rmw(regs + 0x0610, 0x1b000000, 0xff000000);
	reg_rmw(regs + 0x0614, 0x00006fb8, 0x0000ffff);
	reg_rmw(regs + 0x0618, 0x758000e4, 0xffff00ff);
	reg_rmw(regs + 0x06ac, 0x00004400, 0x0000ff00);
	reg_rmw(regs + 0x062c, 0x00200800, 0x00ffff00);
	reg_rmw(regs + 0x0680, 0x00820082, 0x00ff00ff);
	reg_rmw(regs + 0x0684, 0x1d0f0385, 0xffffffff);

	reg_rmw(regs + 0x0804, 0x38000080, 0xff0000ff);
	reg_rmw(regs + 0x0808, 0x00000000, 0x000000ff);
	reg_rmw(regs + 0x080c, 0x02000000, 0xff000000);
	reg_rmw(regs + 0x0810, 0x1b000000, 0xff000000);
	reg_rmw(regs + 0x0814, 0x00006fb8, 0x0000ffff);
	reg_rmw(regs + 0x0818, 0x758000e4, 0xffff00ff);
	reg_rmw(regs + 0x08ac, 0x00004400, 0x0000ff00);
	reg_rmw(regs + 0x082c, 0x00200800, 0x00ffff00);
	reg_rmw(regs + 0x0880, 0x00820082, 0x00ff00ff);
	reg_rmw(regs + 0x0884, 0x1d0f0385, 0xffffffff);

	reg_rmw(regs + 0x0a00, 0x00000800, 0x0000ff00);
	reg_rmw(regs + 0x0a08, 0x38a20000, 0xffff0000);
	reg_rmw(regs + 0x0a30, 0x008a8a00, 0x00ffff00);
	reg_rmw(regs + 0x0a84, 0x00000600, 0x0000ff00);
	reg_rmw(regs + 0x0a94, 0x10000000, 0xff000000);
	reg_rmw(regs + 0x0aa0, 0x81000000, 0xff000000);
	reg_rmw(regs + 0x0abc, 0xff000000, 0xff000000);
	reg_rmw(regs + 0x0ac0, 0x0000008b, 0x000000ff);
	reg_rmw(regs + 0x0b08, 0x583f0000, 0xffff0000);
	reg_rmw(regs + 0x0b0c, 0x0000004e, 0x000000ff);
	reg_rmw(regs + 0x0000, 0x00000003, 0x000000ff);
	reg_rmw(regs + 0x0a00, 0x0000005f, 0x000000ff);
	reg_rmw(regs + 0x0a48, 0x00fd8c00, 0x00ffff00);
	reg_rmw(regs + 0x0a54, 0x002fec72, 0x00ffffff);
	reg_rmw(regs + 0x0a58, 0x00f92100, 0xffffff00);
	reg_rmw(regs + 0x0a5c, 0x00040060, 0xffffffff);
	reg_rmw(regs + 0x0a60, 0x00008000, 0xffffffff);
	reg_rmw(regs + 0x0a64, 0x0c581220, 0xffffffff);
	reg_rmw(regs + 0x0a68, 0xe13b0602, 0xffffffff);
	reg_rmw(regs + 0x0a6c, 0xb8074cc1, 0xffffffff);
	reg_rmw(regs + 0x0a70, 0x3f02e989, 0xffffffff);
	reg_rmw(regs + 0x0a74, 0x00000001, 0x000000ff);
	reg_rmw(regs + 0x0b20, 0x00370000, 0x00ff0000);
	reg_rmw(regs + 0x0b1c, 0x37000000, 0xff000000);
	reg_rmw(regs + 0x0b20, 0x0000005d, 0x000000ff);

	/*Bring SerDes out of Reset if SerDes is Shutdown & is in Reset Mode*/
	reg_rmw(regs + 0x0010, 0x00000000, 1 << 28);

	/* Enable TX and RX via the LANExCTL_STS 0x0000 + x*4 */
	reg_rmw(regs + 0x0228, 0x00000000, 1 << 29);
	writel(0xF800f8c0, regs + 0x1fe0);
	reg_rmw(regs + 0x0428, 0x00000000, 1 << 29);
	writel(0xF800f8c0, regs + 0x1fe4);
	reg_rmw(regs + 0x0628, 0x00000000, 1 << 29);
	writel(0xF800f8c0, regs + 0x1fe8);
	reg_rmw(regs + 0x0828, 0x00000000, 1 << 29);
	writel(0xF800f8c0, regs + 0x1fec);

	/*Enable pll via the pll_ctrl 0x0014*/
	__raw_writel(0xe0000000, regs + 0x1ff4);

	iounmap(regs);
	regs = ioremap(0x02090000, 0x1000);

	/*Waiting for SGMII Serdes PLL lock.*/
	for (cnt = 10000;
	     cnt > 0 && ((__raw_readl(regs + 0x114) & 0x10) == 0);
	     cnt--);

	for (cnt = 10000;
	     cnt > 0 && ((__raw_readl(regs + 0x214) & 0x10) == 0);
	     cnt--);

	for (cnt = 10000;
	     cnt > 0 && ((__raw_readl(regs + 0x414) & 0x10) == 0);
	     cnt--);

	for (cnt = 10000;
	     cnt > 0 && ((__raw_readl(regs + 0x514) & 0x10) == 0);
	     cnt--);

	iounmap(regs);
	udelay(200);
}
