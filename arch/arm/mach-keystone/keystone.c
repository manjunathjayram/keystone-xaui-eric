/*
 * Keystone2 based boards and SOC related code.
 *
 * Copyright 2013 Texas Instruments, Inc.
 *	Cyril Chemparathy <cyril@ti.com>
 *	Santosh Shilimkar <santosh.shillimkar@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 */
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/reboot.h>

#include <asm/mach/map.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/setup.h>
#include <asm/smp_plat.h>
#include <asm/memory.h>

#include "memory.h"

#include "keystone.h"

static struct notifier_block platform_nb;

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
				      unsigned long event, void *dev)
{
	if (event != BUS_NOTIFY_ADD_DEVICE)
		return NOTIFY_DONE;

	if (is_coherent(dev)) {
		set_dma_ops(dev, &arm_coherent_dma_ops);
		pr_info("\t\t%s: keystone device is coherent\n", dev_name(dev));
	}

	return NOTIFY_OK;
}

static void __init keystone_init(void)
{
	keystone_pm_runtime_init();
	if (platform_nb.notifier_call)
		bus_register_notifier(&platform_bus_type, &platform_nb);
	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);
}

static phys_addr_t keystone_virt_to_idmap(unsigned long x)
{
	return (phys_addr_t)(x) - CONFIG_PAGE_OFFSET + KEYSTONE_LOW_PHYS_START;
}

static unsigned long keystone_dma_pfn_offset __read_mostly;

static dma_addr_t keystone_pfn_to_dma(struct device *dev, unsigned long pfn)
{
	return PFN_PHYS(pfn - keystone_dma_pfn_offset);
}

static unsigned long keystone_dma_to_pfn(struct device *dev, dma_addr_t addr)
{
	return PFN_DOWN(addr) + keystone_dma_pfn_offset;
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
	int error_irq = 0;
	int ret;

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

	ret = keystone_init_ddr3_ecc(node);
	return ret;
}
subsys_initcall(keystone_init_misc);

static void *keystone_dma_to_virt(struct device *dev, dma_addr_t addr)
{
	return phys_to_virt(addr + PFN_PHYS(keystone_dma_pfn_offset));
}

static dma_addr_t keystone_virt_to_dma(struct device *dev, void *addr)
{
	return virt_to_phys(addr) - PFN_PHYS(keystone_dma_pfn_offset);
}

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
		pr_crit("Enable %s%s%s to run outside 32-bit space\n",
		      !lpae ? __stringify(CONFIG_ARM_LPAE) : "",
		      (!lpae && !pvpatch) ? " and " : "",
		      !pvpatch ? __stringify(CONFIG_ARM_PATCH_PHYS_VIRT) : "");
	}

	if (mem_start < KEYSTONE_HIGH_PHYS_START ||
	    mem_end   > KEYSTONE_HIGH_PHYS_END) {
		pr_crit("Invalid address space for memory (%08llx-%08llx)\n",
		      (u64)mem_start, (u64)mem_end);
	}

	offset += KEYSTONE_HIGH_PHYS_START;
	__pv_phys_offset = offset;
	__pv_offset = (offset - PAGE_OFFSET);

	/* Populate the arch idmap hook */
	arch_virt_to_idmap = keystone_virt_to_idmap;

	/* Populate the arch DMA hooks */
	keystone_dma_pfn_offset = PFN_DOWN(KEYSTONE_HIGH_PHYS_START -
					   KEYSTONE_LOW_PHYS_START);
	__arch_pfn_to_dma = keystone_pfn_to_dma;
	__arch_dma_to_pfn = keystone_dma_to_pfn;
	__arch_dma_to_virt = keystone_dma_to_virt;
	__arch_virt_to_dma = keystone_virt_to_dma;

	platform_nb.notifier_call = keystone_platform_notifier;

	pr_info("Switching to high address space at 0x%llx\n", (u64)offset);
}

static const char *keystone_match[] __initconst = {
	"ti,keystone-evm",
	NULL,
};

DT_MACHINE_START(KEYSTONE, "Keystone")
#if defined(CONFIG_ZONE_DMA) && defined(CONFIG_ARM_LPAE)
	.dma_zone_size	= SZ_2G,
#endif
	.smp		= smp_ops(keystone_smp_ops),
	.init_machine	= keystone_init,
	.dt_compat	= keystone_match,
	.init_meminfo   = keystone_init_meminfo,
MACHINE_END
