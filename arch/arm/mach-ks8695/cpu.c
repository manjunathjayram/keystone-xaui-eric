/*
 * arch/arm/mach-ks8695/cpu.c
 *
 * Copyright (C) 2006 Ben Dooks <ben@simtec.co.uk>
 * Copyright (C) 2006 Simtec Electronics
 *
 * KS8695 CPU support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>

#include <mach/hardware.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <mach/regs-sys.h>
#include <mach/regs-misc.h>


static struct __initdata map_desc ks8695_io_desc[] = {
	{
		.virtual	= (unsigned long)KS8695_IO_VA,
		.pfn		= __phys_to_pfn(KS8695_IO_PA),
		.length		= KS8695_IO_SIZE,
		.type		= MT_DEVICE,
	}
};

static void __init ks8695_processor_info(void)
{
	unsigned long id, rev;

	id = __raw_readl(KS8695_MISC_VA + KS8695_DID);
	rev = __raw_readl(KS8695_MISC_VA + KS8695_RID);

	printk("KS8695 ID=%04lx  SubID=%02lx  Revision=%02lx\n", (id & DID_ID), (rev & RID_SUBID), (rev & RID_REVISION));
}

static unsigned int sysclk[8] = { 125000000, 100000000, 62500000, 50000000, 41700000, 33300000, 31300000, 25000000 };
static unsigned int cpuclk[8] = { 166000000, 166000000, 83000000, 83000000, 55300000, 55300000, 41500000, 41500000 };

static void __init ks8695_clock_info(void)
{
	unsigned int scdc = __raw_readl(KS8695_SYS_VA + KS8695_CLKCON) & CLKCON_SCDC;

	printk("Clocks: System %u MHz, CPU %u MHz\n",
			sysclk[scdc] / 1000000, cpuclk[scdc] / 1000000);
}

#ifdef CONFIG_PCI

/* PCI mappings */
#define __virt_to_lbus(x)	((x) - PAGE_OFFSET + KS8695_PCIMEM_PA)
#define __lbus_to_virt(x)	((x) - KS8695_PCIMEM_PA + PAGE_OFFSET)

/* Platform-bus mapping */
extern struct bus_type platform_bus_type;
#define is_lbus_device(dev)		(dev && dev->bus == &platform_bus_type)

static dma_addr_t ks8695_pfn_to_dma(struct device *dev, unsigned long pfn)
{
	dma_addr_t __dma = __pfn_to_phys(pfn);

	if (!is_lbus_device(dev))
		__dma = __dma - PHYS_OFFSET + KS8695_PCIMEM_PA;
	return __dma;
}
static unsigned long ks8695_dma_to_pfn(struct device *dev, dma_addr_t addr)
{
	dma_addr_t __dma = addr;

	if (!is_lbus_device(dev))
		__dma += PHYS_OFFSET - KS8695_PCIMEM_PA;
	return __phys_to_pfn(__dma);
}

static void *ks8695_dma_to_virt(struct device *dev, dma_addr_t addr)
{
	return (void *) (is_lbus_device(dev) ? __phys_to_virt(addr) : __lbus_to_virt(addr));
}

static dma_addr_t ks8695_virt_to_dma(struct device *dev, void *addr)
{
	unsigned long __addr = (unsigned long)(addr);
	return (dma_addr_t) (is_lbus_device(dev) ? __virt_to_phys(__addr) : __virt_to_lbus(__addr));
}

#endif


void __init ks8695_map_io(void)
{
#ifdef CONFIG_PCI
	__arch_pfn_to_dma = ks8695_pfn_to_dma;
	__arch_dma_to_pfn = ks8695_dma_to_pfn;
	__arch_dma_to_virt = ks8695_dma_to_virt;
	__arch_virt_to_dma = ks8695_virt_to_dma;
#endif

	iotable_init(ks8695_io_desc, ARRAY_SIZE(ks8695_io_desc));

	ks8695_processor_info();
	ks8695_clock_info();
}
