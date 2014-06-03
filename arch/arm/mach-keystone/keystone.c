/*
 * Copyright 2010-2014 Texas Instruments, Inc.
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
#include <linux/slab.h>
#include <linux/clocksource.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/irqchip/arm-gic.h>
#include <linux/dma-mapping.h>
#include <linux/irqchip/keystone-ipc.h>
#include <linux/irqchip.h>
#include <linux/platform_data/davinci-clock.h>
#include <linux/reboot.h>

#include <asm/setup.h>
#include <asm/smp_plat.h>
#include <asm/mach/map.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include "keystone.h"

#define RSTMUX8_OMODE_DEVICE_RESET		5
#define RSTMUX8_OMODE_DEVICE_RESET_SHIFT	1
#define RSTMUX8_OMODE_DEVICE_RESET_MASK		(BIT(1) | BIT(2) | BIT(3))
#define RSTMUX8_LOCK_MASK			BIT(0)

unsigned long __arch_dma_pfn_offset;

#ifdef CONFIG_ZONE_DMA
extern phys_addr_t arm_dma_limit;
extern unsigned long arm_dma_zone_size;
#endif

static struct map_desc io_desc[] = {
	{
		.virtual        = 0xfe600000UL,
		.pfn            = __phys_to_pfn(0x02000000UL),
		.length         = 0x800000UL,
		.type           = MT_DEVICE
	},
};

static void __init keystone_map_io(void)
{
	iotable_init(io_desc, ARRAY_SIZE(io_desc));
}

static void __init keystone_timer_init(void)
{
	davinci_of_clk_init();
	clocksource_of_init();
}

static bool is_coherent(struct device *dev)
{
	struct device_node *node = of_node_get(dev->of_node);

	while (node) {
		if (of_property_read_bool(node, "dma-coherent")) {
			of_node_put(node);
			return true;
		}
		node = of_get_next_parent(node);
	}
	return false;
}

static int keystone_platform_notifier(struct notifier_block *nb,
				      unsigned long event, void *_dev)
{
	struct device *dev = _dev;

	if (event == BUS_NOTIFY_ADD_DEVICE) {
		dev->dma_mask = kmalloc(sizeof(*dev->dma_mask), GFP_KERNEL);
		dev->coherent_dma_mask = arm_dma_limit;
		if (dev->dma_mask)
			*dev->dma_mask = arm_dma_limit;
		if (is_coherent(dev))
			set_dma_ops(dev, &arm_coherent_dma_ops);
		return NOTIFY_OK;
	} else if (event == BUS_NOTIFY_DEL_DEVICE) {
		kfree(dev->dma_mask);
		return NOTIFY_OK;
	}
	return NOTIFY_DONE;
}

static struct notifier_block keystone_platform_nb;

static struct of_device_id keystone_dt_match_table[] __initdata = {
	{ .compatible = "simple-bus",},
	{ .compatible = "ti,davinci-aemif", },
	{}
};

static void __init keystone_init(void)
{
	if (keystone_platform_nb.notifier_call)
		bus_register_notifier(&platform_bus_type, &keystone_platform_nb);

	keystone_pm_runtime_init();
	of_platform_populate(NULL, keystone_dt_match_table, NULL, NULL);
}

#define L2_INTERN_ASYNC_ERROR	BIT(30)

static irqreturn_t arm_l1l2_ecc_err_irq_handler(int irq, void *reg_virt)
{
	int ret = IRQ_NONE;
	u32 status, fault;

	/* read and clear L2ECTLR CP15 register for L2 ECC error */
	asm("mrc p15, 1, %0, c9, c0, 3" : "=r"(status));

	if (status & L2_INTERN_ASYNC_ERROR) {
		status &= ~L2_INTERN_ASYNC_ERROR;
		asm("mcr p15, 1, %0, c9, c0, 3" : : "r" (status));
		asm("mcr p15, 0, %0, c5, c1, 0" : "=r" (fault));
		/*
		 * Do a machine restart as this is double bit ECC error
		 * that can't be corrected
		 */
		pr_err("ARM Cortex A15 L1/L2 ECC error, CP15 ADFSR 0x%x\n",
			fault);
		machine_restart(NULL);
		ret = IRQ_HANDLED;
	}
	return ret;
}

static int __init keystone_init_misc(void)
{
	struct device_node *node = NULL;
	void __iomem *rstmux8;
	int error_irq = 0;
	u32 val;

	/*
	 * For WD reset to function, rstmux8 should be configured
	 * so that this will trigger a device reset.
	 */
	node = of_find_compatible_node(NULL, NULL, "ti,keystone-reset");
	if (!node) {
		pr_warn("ti, keystone-reset node undefined\n");
		return -EINVAL;
	}

	/* rstmux8 address is configured in the rstctrl node at index 1 */
	rstmux8 = of_iomap(node, 1);
	if (WARN_ON(!rstmux8)) {
		pr_warn("rstmux8 iomap error\n");
		return -ENODEV;
	}

	val = __raw_readl(rstmux8) & ~RSTMUX8_OMODE_DEVICE_RESET_MASK;
	if (!(val & RSTMUX8_LOCK_MASK)) {
		val |= (RSTMUX8_OMODE_DEVICE_RESET <<
				RSTMUX8_OMODE_DEVICE_RESET_SHIFT);
		__raw_writel(val, rstmux8);
	}
	iounmap(rstmux8);

	/* add ARM ECC L1/L2 cache error handler */
	node = of_find_compatible_node(NULL, NULL, "ti,keystone-sys");
	if (node)
		error_irq = irq_of_parse_and_map(node, 0);
	if (!error_irq) {
		pr_warn("Warning!! arm L1/L2 ECC irq number not defined\n");
		return 0;
	}
	if (request_irq(error_irq, arm_l1l2_ecc_err_irq_handler, 0,
		"a15-l1l2-ecc-err-irq", 0) < 0) {
		WARN_ON("request_irq fail for arm L1/L2 ECC error irq\n");
	}
	return 0;
}
postcore_initcall(keystone_init_misc);

static const char *keystone2_match[] __initconst = {
	"ti,keystone",
	NULL,
};

static void __init keystone_init_meminfo(void)
{
	bool lpae = IS_ENABLED(CONFIG_ARM_LPAE);
	bool pvpatch = IS_ENABLED(CONFIG_ARM_PATCH_PHYS_VIRT);
	phys_addr_t offset = PHYS_OFFSET - KEYSTONE_LOW_PHYS_START;
	phys_addr_t mem_start, mem_end;

	BUG_ON(meminfo.nr_banks < 1);
	mem_start = meminfo.bank[0].start;
	mem_end = mem_start + meminfo.bank[0].size - 1;

	/* nothing to do if we are running out of the <32-bit space */
	if (mem_start >= KEYSTONE_LOW_PHYS_START &&
	    mem_end   <= KEYSTONE_LOW_PHYS_END)
		return;

	if (!lpae || !pvpatch) {
		panic("Enable %s%s%s to run outside 32-bit space\n",
		      !lpae ? __stringify(CONFIG_ARM_LPAE) : "",
		      (!lpae && !pvpatch) ? " and " : "",
		      !pvpatch ? __stringify(CONFIG_ARM_PATCH_PHYS_VIRT) : "");
	}

	if (mem_start < KEYSTONE_HIGH_PHYS_START ||
	    mem_end   > KEYSTONE_HIGH_PHYS_END) {
		panic("Invalid address space for memory (%08llx-%08llx)\n",
		      (u64)mem_start, (u64)mem_end);
	}

	offset += KEYSTONE_HIGH_PHYS_START;
	pr_info("switching to high address space at 0x%llx\n", (u64)offset);
	__pv_phys_offset = offset;
	__pv_offset      = offset - PAGE_OFFSET;

	__arch_dma_pfn_offset = PFN_DOWN(KEYSTONE_HIGH_PHYS_START -
					 KEYSTONE_LOW_PHYS_START);
#ifdef CONFIG_ZONE_DMA
	arm_dma_limit = __pv_phys_offset + arm_dma_zone_size - 1;
	keystone_platform_nb.notifier_call = keystone_platform_notifier;
#endif
}

void keystone_restart(char mode, const char *cmd)
{
	struct device_node *node;
	void __iomem *rstctrl;
	u32 val;

	node = of_find_compatible_node(NULL, NULL, "ti,keystone-reset");
	if (WARN_ON(!node)) {
		pr_warn("ti, keystone-reset node undefined\n");
		return;
	}

	rstctrl = of_iomap(node, 0);
	if (WARN_ON(!rstctrl)) {
		pr_warn("ti, pllctrl-reset iomap error\n");
		return;
	}

	val = __raw_readl(rstctrl);
	val &= 0xffff0000;
	val |= 0x5a69;
	__raw_writel(val, rstctrl);

	val = __raw_readl(rstctrl);
	val &= 0xfffe0000;
	__raw_writel(val, rstctrl);
}

DT_MACHINE_START(KEYSTONE2, "KeyStone2")
	.map_io		= keystone_map_io,
	.init_irq	= irqchip_init,
	.init_time	= keystone_timer_init,
	.init_machine	= keystone_init,
	.init_meminfo	= keystone_init_meminfo,
	.restart	= keystone_restart,
	.smp		= smp_ops(keystone_smp_ops),
	.dt_compat	= keystone2_match,
#ifdef CONFIG_ZONE_DMA
	.dma_zone_size	= SZ_2G,
#endif
MACHINE_END
