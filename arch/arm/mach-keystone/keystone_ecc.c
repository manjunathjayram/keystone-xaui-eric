/*
 * Copyright 2014 Texas Instruments, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <linux/io.h>
#include <linux/of.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/reboot.h>
#include "keystone.h"

/* DDR3 controller registers */
#define DDR3_EOI			0x0A0
#define DDR3_IRQ_STATUS_RAW_SYS		0x0A4
#define DDR3_IRQ_STATUS_SYS		0x0AC
#define DDR3_IRQ_ENABLE_SET_SYS		0x0B4
#define DDR3_IRQ_ENABLE_CLR_SYS		0x0BC
#define DDR3_ECC_CTRL			0x110
#define DDR3_ONE_BIT_ECC_ERR_CNT	0x130

#define DDR3_1B_ECC_ERR			BIT(5)
#define DDR3_2B_ECC_ERR			BIT(4)
#define DDR3_WR_ECC_ERR			BIT(3)

static irqreturn_t ddr3_ecc_err_irq_handler(int irq, void *reg_virt)
{
	int ret = IRQ_NONE;
	u32 irq_status;
	void __iomem *ddr_reg = (void __iomem *)reg_virt;

	irq_status = readl(ddr_reg + DDR3_IRQ_STATUS_SYS);
	if ((irq_status & DDR3_2B_ECC_ERR) ||
	    (irq_status & DDR3_WR_ECC_ERR)) {
		pr_err("Unrecoverable DDR3 ECC error, irq status 0x%x, "
		       "rebooting kernel ..\n", irq_status);
		machine_restart(NULL);
		ret = IRQ_HANDLED;
	}
	return ret;
}

int keystone_init_ddr3_ecc(struct device_node *node)
{
	void __iomem *ddr_reg;
	int error_irq = 0;
	int ret;

	/* ddr3 controller reg is configured in the sysctrl node at index 0 */
	ddr_reg = of_iomap(node, 0);
	if (!ddr_reg) {
		pr_warn("Warning!! DDR3 controller regs not defined\n");
		return -ENODEV;
	}

	/* add DDR3 ECC error handler */
	error_irq = irq_of_parse_and_map(node, 1);
	if (!error_irq) {
		/* No GIC interrupt, need to map CIC2 interupt to GIC */
		pr_warn("Warning!! DDR3 ECC irq number not defined\n");
		return -ENODEV;
	}

	ret = request_irq(error_irq, ddr3_ecc_err_irq_handler, 0,
		"ddr3-ecc-err-irq", (void *)ddr_reg);
	if (ret) {
		WARN_ON("request_irq fail for DDR3 ECC error irq\n");
		return ret;
	}

	return 0;
}
