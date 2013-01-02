/*
 * Copyright 2010-2012 Texas Instruments, Inc.
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
#include <linux/slab.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/dma-mapping.h>
#include <linux/irqchip/tci6614.h>
#include <linux/irqchip/keystone-ipc.h>
#include <linux/platform_data/davinci-clock.h>

#include <asm/setup.h>
#include <asm/smp_plat.h>
#include <asm/mach/map.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/arch_timer.h>
#include <asm/hardware/gic.h>

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
		.virtual        = 0xfe800000UL,
		.pfn            = __phys_to_pfn(0x02000000UL),
		.length         = 0x800000UL,
		.type           = MT_DEVICE
	},
};

static void __init keystone_map_io(void)
{
	iotable_init(io_desc, ARRAY_SIZE(io_desc));
}

static const struct of_device_id irq_match[] = {
	{
		.compatible = "arm,cortex-a15-gic",
		.data = gic_of_init,
	},
	{
		.compatible = "ti,tci6614-intctrl",
		.data = tci6614_of_init_irq,
	},
	{
		.compatible = "ti,keystone-ipc-irq",
		.data = keystone_ipc_irq_of_init,
	},
	{}
};

static void __init keystone_init_irq(void)
{
	of_irq_init(irq_match);
}


static void __init keystone_timer_init(void)
{
	int error;

	davinci_of_clk_init();

	error = tci6614_timer_init();
	if (!error)
		return;

	error = arch_timer_of_register();
	if (!error) {
		arch_timer_sched_clock_init();
		return;
	}

	panic("no timer!\n");
}

static struct sys_timer keystone_timer = {
	.init = keystone_timer_init,
};

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

#ifdef CONFIG_ZONE_DMA
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
#endif
	return NOTIFY_DONE;
}

static struct notifier_block keystone_platform_nb = {
	.notifier_call = keystone_platform_notifier,
};

static struct of_device_id keystone_dt_match_table[] __initdata = {
	{ .compatible = "simple-bus",},
	{ .compatible = "ti,davinci-aemif", },
	{}
};

static void __init keystone_init(void)
{
	bus_register_notifier(&platform_bus_type, &keystone_platform_nb);
	keystone_pm_runtime_init();
	of_platform_populate(NULL, keystone_dt_match_table, NULL, NULL);
}

static int __init keystone_wd_rstmux_init(void)
{
	struct device_node *node;
	void __iomem *rstmux8;
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
	return 0;
}
postcore_initcall(keystone_wd_rstmux_init);

static const char *keystone1_match[] __initconst = {
	"ti,tci6614-evm",
	NULL,
};

static const char *keystone2_match[] __initconst = {
	"ti,keystone-evm",
	"ti,tci6638-evm",
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

#define KEYSTONE_MACHINE_DEFS				\
	.map_io		= keystone_map_io,		\
	.init_irq	= keystone_init_irq,		\
	.timer		= &keystone_timer,		\
	.init_machine	= keystone_init,		\
	.init_meminfo	= keystone_init_meminfo,	\
	.restart	= keystone_restart,

DT_MACHINE_START(KEYSTONE1, "KeyStone1")
	KEYSTONE_MACHINE_DEFS
	.handle_irq	= tci6614_handle_irq,
	.dt_compat	= keystone1_match,
MACHINE_END

DT_MACHINE_START(KEYSTONE2, "KeyStone2")
	KEYSTONE_MACHINE_DEFS
	.smp		= smp_ops(keystone_smp_ops),
	.handle_irq	= gic_handle_irq,
	.dt_compat	= keystone2_match,
#ifdef CONFIG_ZONE_DMA
	.dma_zone_size	= SZ_2G,
#endif
MACHINE_END
