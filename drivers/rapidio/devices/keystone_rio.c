/*
 * Copyright (C) 2010, 2011, 2012, 2013, 2014 Texas Instruments Incorporated
 * Authors: Aurelien Jacquiot <a-jacquiot@ti.com>
 * - Main driver implementation.
 * - Updated for support on TI KeyStone 2 platform.
 *
 * Copyright (C) 2012, 2013 Texas Instruments Incorporated
 * WingMan Kwok <w-kwok2@ti.com>
 * - Updated for support on TI KeyStone 1 platform.
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
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/kfifo.h>
#include <linux/platform_device.h>
#include <linux/keystone-dma.h>
#include <linux/rio.h>
#include <linux/rio_drv.h>

#include "keystone_rio.h"

#define DRIVER_VER	        "v1.2"

/*
 * Main KeyStone RapidIO driver data
 */
struct keystone_rio_data {
	struct device	       *dev;
	struct rio_mport       *mport[KEYSTONE_RIO_MAX_PORT];
	struct clk	       *clk;
	struct completion	lsu_completion;
	struct mutex		lsu_lock;
	u8                      lsu_dio;
	u8                      lsu_maint;
	u32			rio_pe_feat;

	struct port_write_msg	port_write_msg;
	struct work_struct	pw_work;
	struct kfifo		pw_fifo;
	spinlock_t		pw_fifo_lock;

	u32                     pe_ports;
	u32                     pe_cnt;
	struct delayed_work     pe_work;

	u32                     ports;
	u32			ports_registering;
	spinlock_t		port_chk_lock;
	u32			port_chk_cnt;
	struct delayed_work	port_chk_task;

	struct tasklet_struct	task;

	unsigned long		rxu_map_bitmap[2];

	struct dma_chan	       *tx_channel;
	const char	       *tx_chan_name;
	u32			tx_queue_depth;

	u32                              num_mboxes;
	struct keystone_rio_mbox_info    tx_mbox[KEYSTONE_RIO_MAX_MBOX];
	struct keystone_rio_mbox_info    rx_mbox[KEYSTONE_RIO_MAX_MBOX];
	struct keystone_rio_rx_chan_info rx_channels[KEYSTONE_RIO_MAX_MBOX];

	u32 __iomem					*jtagid_reg;
	u32 __iomem					*serdes_sts_reg;
	struct keystone_srio_serdes_regs __iomem	*serdes_regs;
	struct keystone_rio_regs	 __iomem	*regs;

	struct keystone_rio_car_csr_regs __iomem	*car_csr_regs;
	struct keystone_rio_serial_port_regs __iomem	*serial_port_regs;
	struct keystone_rio_err_mgmt_regs __iomem	*err_mgmt_regs;
	struct keystone_rio_phy_layer_regs __iomem	*phy_regs;
	struct keystone_rio_transport_layer_regs __iomem *transport_regs;
	struct keystone_rio_pkt_buf_regs __iomem	*pkt_buf_regs;
	struct keystone_rio_evt_mgmt_regs __iomem	*evt_mgmt_regs;
	struct keystone_rio_port_write_regs __iomem	*port_write_regs;
	struct keystone_rio_link_layer_regs __iomem	*link_regs;
	struct keystone_rio_fabric_regs __iomem		*fabric_regs;
	u32						 car_csr_regs_base;

	struct keystone_rio_board_controller_info	 board_rio_cfg;
};

static void dbell_handler(struct keystone_rio_data *krio_priv);
static void keystone_rio_port_write_handler(struct keystone_rio_data *krio_priv);

struct keystone_lane_config {
	int start; /* lane start number of the port */
	int end;   /* lane end number of the port */
};

static struct keystone_lane_config keystone_lane_configs[5][KEYSTONE_RIO_MAX_PORT] = {
	{ {0, 1}, {1, 2},   {2, 3},   {3, 4}   }, /* path mode 0: 4 ports in 1x    */
	{ {0, 2}, {-1, -1}, {2, 3},   {3, 4}   }, /* path mode 1: 3 ports in 2x/1x */
	{ {0, 1}, {1, 2},   {2, 4},   {-1, -1} }, /* path mode 2: 3 ports in 1x/2x */
	{ {0, 2}, {-1, -1}, {2, 4},   {-1, -1} }, /* path mode 3: 2 ports in 2x    */
	{ {0, 4}, {-1, -1}, {-1, -1}, {-1, -1} }, /* path mode 4: 1 ports in 4x    */
};

/*----------------------- Interrupt management -------------------------*/

static irqreturn_t lsu_interrupt_handler(int irq, void *data)
{
	struct keystone_rio_data *krio_priv = data;
	u32 pending_lsu_int =
		__raw_readl(&(krio_priv->regs->lsu_int[0].status));

	if (pending_lsu_int & KEYSTONE_RIO_ICSR_LSU0(0))
		complete(&krio_priv->lsu_completion);

	/* ACK the interrupt */
	__raw_writel(pending_lsu_int,
		     &(krio_priv->regs->lsu_int[0].clear));

	/* Re-arm interrupt */
	__raw_writel(0,
		     &(krio_priv->regs->intdst_rate_cntl[KEYSTONE_LSU_RIO_INT]));

	return IRQ_HANDLED;
}

static void special_interrupt_handler(int ics, struct keystone_rio_data *krio_priv)
{
	/* Acknowledge the interrupt */
	__raw_writel(BIT(ics), &krio_priv->regs->err_rst_evnt_int_clear);

	dev_dbg(krio_priv->dev, "ics = %d\n", ics);

	switch (ics) {
	case KEYSTONE_RIO_MCAST_EVT_INT:
		/* Multi-cast event control symbol interrupt
		   received on any port */
		break;

	case KEYSTONE_RIO_PORT_WRITEIN_INT:
		/* Port-write-in request received on any port */
		keystone_rio_port_write_handler(krio_priv);
		break;

	case KEYSTONE_RIO_EVT_CAP_ERROR_INT:
		/* Logical layer error management event capture */
		break;

	case KEYSTONE_RIO_PORT0_ERROR_INT:
	case KEYSTONE_RIO_PORT1_ERROR_INT:
	case KEYSTONE_RIO_PORT2_ERROR_INT:
	case KEYSTONE_RIO_PORT3_ERROR_INT:
		/* Port error */

		/* Add port to failed ports and schedule immediate recovery */
		krio_priv->pe_cnt =
			krio_priv->board_rio_cfg.port_register_timeout /
			(KEYSTONE_RIO_REGISTER_DELAY / HZ);
		krio_priv->pe_ports |= BIT(ics - KEYSTONE_RIO_PORT0_ERROR_INT);

		schedule_delayed_work(&krio_priv->pe_work, 0);
		break;

	case KEYSTONE_RIO_RESET_INT:
		/* Device reset interrupt on any port */
		break;
	}
	return;
}

static irqreturn_t rio_interrupt_handler(int irq, void *data)
{
	struct keystone_rio_data *krio_priv = data;

	u32 pending_err_rst_evnt_int =
		__raw_readl(&(krio_priv->regs->err_rst_evnt_int_stat)) &
		KEYSTONE_RIO_ERR_RST_EVNT_MASK;

	/* Handle special interrupts (error, reset, special event) */
	while (pending_err_rst_evnt_int) {
		u32 ics = __ffs(pending_err_rst_evnt_int);
		pending_err_rst_evnt_int &= ~(1 << ics);
		special_interrupt_handler(ics, krio_priv);
	}

	/* Call doorbell handler(s) */
	dbell_handler(krio_priv);

	/* Re-arm the interrupt */
	__raw_writel(0, &(krio_priv->regs->intdst_rate_cntl[KEYSTONE_GEN_RIO_INT]));

	return IRQ_HANDLED;
}

/*
 * Map a sRIO event to a sRIO interrupt
 */
static void keystone_rio_interrupt_map(u32 __iomem *reg, u32 mask, u32 rio_int)
{
	int i;
	u32 reg_val;

	reg_val = __raw_readl(reg);

	for (i = 0; i <= 32; i += 4) {
		if ((mask >> i) & 0xf) {
			reg_val &= ~(0xf << i);
			reg_val |= (rio_int << i);
		}
	}
	__raw_writel(reg_val, reg);
}

/*
 * Setup RIO interrupts
 */
static int keystone_rio_interrupt_setup(struct keystone_rio_data *krio_priv)
{
	int i, res;

	/* Clear all pending interrupts */
	__raw_writel(0x0000ffff,
		     &(krio_priv->regs->doorbell_int[0].clear));
	__raw_writel(0x0000ffff,
		     &(krio_priv->regs->doorbell_int[1].clear));
	__raw_writel(0x0000ffff,
		     &(krio_priv->regs->doorbell_int[2].clear));
	__raw_writel(0x0000ffff,
		     &(krio_priv->regs->doorbell_int[3].clear));
	__raw_writel(0xffffffff,
		     &(krio_priv->regs->lsu_int[krio_priv->lsu_dio].clear));
	__raw_writel(KEYSTONE_RIO_ERR_RST_EVNT_MASK,
		     &(krio_priv->regs->err_rst_evnt_int_clear));

	/* LSU interrupts are routed to RIO interrupt dest 0 (LSU) */
	keystone_rio_interrupt_map(&(krio_priv->regs->lsu0_int_route[0]),
				   0x11111111, KEYSTONE_LSU_RIO_INT);
	keystone_rio_interrupt_map(&(krio_priv->regs->lsu0_int_route[1]),
				   0x11111111, KEYSTONE_LSU_RIO_INT);
	keystone_rio_interrupt_map(&(krio_priv->regs->lsu0_int_route[2]),
				   0x11111111, KEYSTONE_LSU_RIO_INT);
	keystone_rio_interrupt_map(&(krio_priv->regs->lsu0_int_route[3]),
				   0x11111111, KEYSTONE_LSU_RIO_INT);
	keystone_rio_interrupt_map(&(krio_priv->regs->lsu1_int_route1),
				   0x11111111, KEYSTONE_LSU_RIO_INT);

	/* Error, reset and special event interrupts are routed to RIO interrupt dest 1 (Rx/Tx) */
	keystone_rio_interrupt_map(&(krio_priv->regs->err_rst_evnt_int_route[0]),
				   0x00000111, KEYSTONE_GEN_RIO_INT);
	keystone_rio_interrupt_map(&(krio_priv->regs->err_rst_evnt_int_route[1]),
				   0x00001111, KEYSTONE_GEN_RIO_INT);
	keystone_rio_interrupt_map(&(krio_priv->regs->err_rst_evnt_int_route[2]),
				   0x00000001, KEYSTONE_GEN_RIO_INT);

	/* The doorbell interrupts routing table is for the 16 general purpose interrupts */
	__raw_writel(0x1, &(krio_priv->regs->interrupt_ctl));

	/* Do not use pacing */
	for (i = 0; i < 15; i++)
		__raw_writel(0x0, &(krio_priv->regs->intdst_rate_cntl[i]));

	/* Attach interrupt handlers */
	res = request_irq(krio_priv->board_rio_cfg.rio_irq,
			  rio_interrupt_handler,
			  0,
			  "sRIO",
			  krio_priv);
	if (res) {
		dev_err(krio_priv->dev,
			"Failed to request RIO irq (%d)\n",
			krio_priv->board_rio_cfg.rio_irq);
		return res;
	}

	res = request_irq(krio_priv->board_rio_cfg.lsu_irq,
			  lsu_interrupt_handler,
			  0,
			  "sRIO LSU",
			  krio_priv);
	if (res) {
		dev_err(krio_priv->dev,
			"Failed to request LSU irq (%d)\n",
			krio_priv->board_rio_cfg.lsu_irq);
		free_irq(krio_priv->board_rio_cfg.rio_irq, krio_priv);
		return res;
	}

	return 0;
}

static void keystone_rio_interrupt_release(struct keystone_rio_data *krio_priv)
{
	free_irq(krio_priv->board_rio_cfg.rio_irq, krio_priv);
	free_irq(krio_priv->board_rio_cfg.lsu_irq, krio_priv);
}

/*---------------------------- Direct I/O -------------------------------*/

static u32 keystone_rio_dio_get_lsu_cc(u32 lsu_id, u8 ltid, u8 *lcb,
				       struct keystone_rio_data *krio_priv)
{
	u32 idx;
	u32 shift;
	u32 value;
	u32 cc;
	/* lSU shadow register status mapping */
	u32 lsu_index[8] = { 0, 9, 15, 20, 24, 33, 39, 44 };

	/* Compute LSU stat index from LSU id and LTID */
	idx   = (lsu_index[lsu_id] + ltid) >> 3;
	shift = ((lsu_index[lsu_id] + ltid) & 0x7) << 2;

	/* Get completion code and context */
	value  = __raw_readl(&(krio_priv->regs->lsu_stat_reg[idx]));
	cc     = (value >> (shift + 1)) & 0x7;
	*lcb   = (value >> shift) & 0x1;

	return cc;
}

static u32 keystone_rio_dio_packet_type(int dio_mode)
{
	switch (dio_mode) {
	case RIO_DIO_MODE_READ:
		return KEYSTONE_RIO_PACKET_TYPE_NREAD;
	case RIO_DIO_MODE_WRITER:
		return KEYSTONE_RIO_PACKET_TYPE_NWRITE_R;
	case RIO_DIO_MODE_WRITE:
		return KEYSTONE_RIO_PACKET_TYPE_NWRITE;
	case RIO_DIO_MODE_SWRITE:
		return KEYSTONE_RIO_PACKET_TYPE_SWRITE;
	}
	return KEYSTONE_RIO_PACKET_TYPE_NREAD;
}

/*
 * DIO transfer using LSU directly
 */
static inline int keystone_rio_dio_raw_transfer(int port_id,
						u16 dest_id,
						dma_addr_t src_addr,
						u32 tgt_addr,
						int size_bytes,
						int size,
						int dio_mode,
						struct keystone_rio_data *krio_priv)
{
	unsigned int count;
	unsigned int status = 0;
	unsigned int res = 0;
	unsigned int retry_count = KEYSTONE_RIO_RETRY_CNT;
	u8           context;
	u8           ltid;
	u8           lsu = krio_priv->lsu_dio;

	size_bytes &= (KEYSTONE_RIO_MAX_DIO_PKT_SIZE - 1);

retry_transfer:

	mutex_lock(&krio_priv->lsu_lock);

	reinit_completion(&krio_priv->lsu_completion);

	/* Check if there is space in the LSU shadow reg and that it is free */
	count = 0;
	while (1) {
		status = __raw_readl(&(krio_priv->regs->lsu_reg[lsu].busy_full));
		if (((status & KEYSTONE_RIO_LSU_FULL_MASK) == 0x0)
		    && ((status & KEYSTONE_RIO_LSU_BUSY_MASK) == 0x0))
			break;
		count++;
		if (count >= KEYSTONE_RIO_TIMEOUT_CNT) {
			dev_err(krio_priv->dev,
				"DIO: cannot get a free LSU\n");
			res = -EIO;
			goto out;
		}
		ndelay(1000);
	}

	/* Get LCB and LTID, LSU reg 6 is already read */
	context = (status >> 4) & 0x1;
	ltid    = status & 0xf;

	/* LSU Reg 0 - MSB of destination */
	__raw_writel(0, &(krio_priv->regs->lsu_reg[lsu].addr_msb));

	/* LSU Reg 1 - LSB of destination */
	__raw_writel(tgt_addr, &(krio_priv->regs->lsu_reg[lsu].addr_lsb_cfg_ofs));

	/* LSU Reg 2 - source address */
	__raw_writel(src_addr, &(krio_priv->regs->lsu_reg[lsu].phys_addr));

	/* LSU Reg 3 - Byte count */
	__raw_writel(size_bytes,
		     &(krio_priv->regs->lsu_reg[lsu].dbell_val_byte_cnt));

	/* LSU Reg 4 -
	 * out port ID = rio.port
	 * priority = LSU_PRIO
	 * XAM = 0
	 * ID size = 8 or 16 bit
	 * Dest ID specified as arg
	 * interrupt request = 1 */
	__raw_writel(((port_id << 8)
		      | (KEYSTONE_RIO_LSU_PRIO << 4)
		      | (size ? (1 << 10) : 0)
		      | ((u32) dest_id << 16)
		      | 1),
		     &(krio_priv->regs->lsu_reg[lsu].destid));

	/* LSU Reg 5 -
	 * doorbell info = 0 for this packet type
	 * hop count = 0 for this packet type
	 * Writing this register should initiate the transfer */
	__raw_writel(keystone_rio_dio_packet_type(dio_mode),
		     &(krio_priv->regs->lsu_reg[lsu].dbell_info_fttype));

	/* Wait for transfer to complete */
	res = wait_for_completion_timeout(&krio_priv->lsu_completion,
					  msecs_to_jiffies(KEYSTONE_RIO_TIMEOUT_MSEC));
	if (res == 0) {
		/* Timeout expired */
		res = -EIO;
		goto out;
	}

	dev_dbg(krio_priv->dev, "DIO: LSU interrupt received\n");

	/* Retrieve our completion code */
	count = 0;
	res   = 0;
	while (1) {
		u8 lcb;
		status = keystone_rio_dio_get_lsu_cc(lsu, ltid, &lcb, krio_priv);
		if (lcb == context)
			break;
		count++;
		if (count >= KEYSTONE_RIO_TIMEOUT_CNT) {
			res = -EIO;
			break;
		}
		ndelay(1000);
	}
out:
	mutex_unlock(&krio_priv->lsu_lock);

	if (res) {
		dev_err(krio_priv->dev, "DIO: LSU error = %d\n", res);
		return res;
	}

	dev_dbg(krio_priv->dev, "DIO: status = 0x%x\n", status);

	switch (status & KEYSTONE_RIO_LSU_CC_MASK) {
	case KEYSTONE_RIO_LSU_CC_TIMEOUT:
	case KEYSTONE_RIO_LSU_CC_XOFF:
	case KEYSTONE_RIO_LSU_CC_ERROR:
	case KEYSTONE_RIO_LSU_CC_INVALID:
	case KEYSTONE_RIO_LSU_CC_DMA:
		res = -EIO;
		/* LSU Reg 6 - Flush this transaction */
		__raw_writel(1, &(krio_priv->regs->lsu_reg[lsu].busy_full));
		break;
	case KEYSTONE_RIO_LSU_CC_RETRY:
	case KEYSTONE_RIO_LSU_CC_CANCELED:
		res = -EAGAIN;
		break;
	default:
		break;
	}

	/*
	 * Try to transfer again in case of retry doorbell receive
	 * or canceled LSU transfer.
	 */
	if ((res == -EAGAIN) && (retry_count-- > 0)) {
		ndelay(1000);
		goto retry_transfer;
	}

	return res;
}

/**
 * keystone_rio_dio_transfer - Transfer bytes data from/to physical address
 * to device ID's global address.
 * @mport: RapidIO master port info
 * @index: ID of the RapidIO interface
 * @dest_id: destination device id
 * @src_addr: source (host) address
 * @tgt_addr: target global address
 * @size_bytes: size in bytes
 * @dio_mode: DIO transfer mode (write, write_r, swrite , read)
 *
 * Return %0 on success. Return %-EIO, %-EBUSY or %-EAGAIN on failure.
 */
static int keystone_rio_dio_transfer(struct rio_mport *mport,
				     int index,
				     u16 dest_id,
				     u32 src_addr,
				     u32 tgt_addr,
				     int size_bytes,
				     int dio_mode)
{
	struct keystone_rio_data *krio_priv = mport->priv;
	struct device *dev = ((struct keystone_rio_data *)(mport->priv))->dev;
	int count  = size_bytes;
	int length;
	int res    = 0;
	u32 s_addr = src_addr;
	u32 t_addr = tgt_addr;
	dma_addr_t dma;
	enum dma_data_direction dir;

	/* SWRITE implies double-word alignment for address and size */
	if ((dio_mode == RIO_DIO_MODE_SWRITE) &&
	    ((size_bytes % 8) || (tgt_addr % 8) || (src_addr % 8)))
		return -EINVAL;

	dev_dbg(krio_priv->dev,
		"DIO: transferring chunk addr = 0x%x, size = %d, tgt_addr=%08x, mode=%d\n",
		src_addr, size_bytes, tgt_addr, dio_mode);

	if (dio_mode == RIO_DIO_MODE_READ)
		dir = DMA_FROM_DEVICE;
	else
		dir = DMA_TO_DEVICE;

	/* Transfer packet by packet */
	while (count) {
		length = (count <= KEYSTONE_RIO_MAX_DIO_PKT_SIZE) ?
			count : KEYSTONE_RIO_MAX_DIO_PKT_SIZE;

		dma = dma_map_single(dev, (void *)s_addr, length, dir);

		res = keystone_rio_dio_raw_transfer(mport->index,
						    dest_id,
						    dma,
						    t_addr,
						    length,
						    mport->sys_size,
						    dio_mode,
						    krio_priv);

		dma_unmap_single(dev, dma, length, dir);

		if (res)
			break;

		s_addr += length;
		t_addr += length;

		count -= length;
	}
	return res;
}

/*------------------------------ Doorbell management --------------------------*/

static inline int dbell_get(u32 *pending)
{
	if (*pending) {
		int n = __ffs(*pending);
		*pending &= ~(1 << n);
		return n;
	} else
		return -1;
}

static inline void dbell_call_handler(u32 dbell_num, struct keystone_rio_data *krio_priv)
{
	struct rio_dbell *dbell;
	int i;
	int found = 0;

	for (i = 0; i < KEYSTONE_RIO_MAX_PORT; i++) {
		if (krio_priv->mport[i]) {
			struct rio_mport *mport = krio_priv->mport[i];
			list_for_each_entry(dbell, &mport->dbells, node) {
				if ((dbell->res->start <= dbell_num) &&
				    (dbell->res->end   >= dbell_num)) {
					found = 1;
					break;
				}
			}

			if (found && dbell->dinb) {
				/* Call the registered handler if any */
				dbell->dinb(mport,
					    dbell->dev_id,
					    -1, /* we don't know the source Id */
					    mport->host_deviceid,
					    dbell_num);
				break;
			}
		}
	}

	if (!found)
		dev_dbg(krio_priv->dev, "DBELL: received spurious doorbell %d\n", dbell_num);
}

static void dbell_handler(struct keystone_rio_data *krio_priv)
{
	u32 pending_dbell;
	unsigned int i;

	for (i = 0; i < KEYSTONE_RIO_DBELL_NUMBER; i++) {
		pending_dbell =  __raw_readl(&(krio_priv->regs->doorbell_int[i].status));

		if (pending_dbell)
			/* Acknowledge the interrupts for these doorbells */
			__raw_writel(pending_dbell,
				     &(krio_priv->regs->doorbell_int[i].clear));

		while (pending_dbell) {
			u32 dbell_num = dbell_get(&pending_dbell) + (i << 4);

			/* Call the registered dbell handler(s) */
			dbell_call_handler(dbell_num, krio_priv);
		}
	}
}

/**
 * keystone_rio_doorbell_send - Send a KeyStone doorbell message
 * @mport: RapidIO master port info
 * @index: ID of the RapidIO interface
 * @destid: device ID of target device
 * @num: doorbell number
 *
 * Sends a KeyStone doorbell message. Returns %0 on success or
 * %-EINVAL, %-EIO, %-EBUSY or %-EAGAIN on failure.
 */
static int keystone_rio_dbell_send(struct rio_mport *mport,
				   int index,
				   u16 dest_id,
				   u16 num)
{
	struct keystone_rio_data *krio_priv = mport->priv;
	unsigned int count;
	unsigned int status = 0;
	unsigned int res    = 0;
	/* Transform doorbell number into info field */
	u16          info   = (num & 0xf) | (((num >> 4) & 0x3) << 5);
	u8           port_id = mport->index;
	u8           context;
	u8           ltid;
	u8           lsu = krio_priv->lsu_dio;

	dev_dbg(krio_priv->dev, "DBELL: sending doorbell (info = %d) to %x\n",
		info & KEYSTONE_RIO_DBELL_MASK, dest_id);

	mutex_lock(&krio_priv->lsu_lock);

	reinit_completion(&krio_priv->lsu_completion);

	/* Check is there is space in the LSU shadow reg and that it is free */
	count = 0;
	while (1) {
		status = __raw_readl(&(krio_priv->regs->lsu_reg[lsu].busy_full));
		if (((status & KEYSTONE_RIO_LSU_FULL_MASK) == 0x0)
		    && ((status & KEYSTONE_RIO_LSU_BUSY_MASK) == 0x0))
			break;
		count++;
		if (count >= KEYSTONE_RIO_TIMEOUT_CNT) {
			res = -EIO;
			goto out;
		}
		ndelay(KEYSTONE_RIO_TIMEOUT_NSEC);
	}

	/* Get LCB and LTID, LSU reg 6 is already read */
	context = (status >> 4) & 0x1;
	ltid    = status & 0xf;

	/* LSU Reg 0 - MSB of destination */
	__raw_writel(0, &(krio_priv->regs->lsu_reg[lsu].addr_msb));

	/* LSU Reg 1 - LSB of destination */
	__raw_writel(0, &(krio_priv->regs->lsu_reg[lsu].addr_lsb_cfg_ofs));

	/* LSU Reg 2 - source address */
	__raw_writel(0, &(krio_priv->regs->lsu_reg[lsu].phys_addr));

	/* LSU Reg 3 - byte count */
	__raw_writel(0, &(krio_priv->regs->lsu_reg[lsu].dbell_val_byte_cnt));

	/* LSU Reg 4 - */
	__raw_writel(((port_id << 8)
		      | (KEYSTONE_RIO_LSU_PRIO << 4)
		      | ((mport->sys_size) ? (1 << 10) : 0)
		      | ((u32) dest_id << 16)
		      | 1),
		     &(krio_priv->regs->lsu_reg[lsu].destid));

	/* LSU Reg 5
	 * doorbell info = info
	 * hop count = 0
	 * Packet type = 0xa0 ftype = 10, ttype = 0 */
	__raw_writel(((info & 0xffff) << 16) | (KEYSTONE_RIO_PACKET_TYPE_DBELL & 0xff),
		     &(krio_priv->regs->lsu_reg[lsu].dbell_info_fttype));

	/* Wait for transfer to complete */
	res = wait_for_completion_timeout(&krio_priv->lsu_completion,
					  msecs_to_jiffies(KEYSTONE_RIO_TIMEOUT_MSEC));
	if (res == 0) {
		/* Timeout expired */
		res = -EIO;
		goto out;
	}

	dev_dbg(krio_priv->dev, "DBELL: LSU interrupt received\n");

	/* Retrieve our completion code */
	count = 0;
	res   = 0;
	while (1) {
		u8 lcb;
		status = keystone_rio_dio_get_lsu_cc(lsu, ltid, &lcb, krio_priv);
		if (lcb == context)
			break;
		count++;
		if (count >= KEYSTONE_RIO_TIMEOUT_CNT) {
			res = -EIO;
			break;
		}
		ndelay(KEYSTONE_RIO_TIMEOUT_NSEC);
	}
out:
	mutex_unlock(&krio_priv->lsu_lock);

	if (res) {
		dev_dbg(krio_priv->dev, "DBELL: LSU error = %d\n", res);
		return res;
	}

	dev_dbg(krio_priv->dev, "DBELL: status = 0x%x\n", status);

	switch (status & KEYSTONE_RIO_LSU_CC_MASK) {
	case KEYSTONE_RIO_LSU_CC_TIMEOUT:
	case KEYSTONE_RIO_LSU_CC_XOFF:
	case KEYSTONE_RIO_LSU_CC_ERROR:
	case KEYSTONE_RIO_LSU_CC_INVALID:
	case KEYSTONE_RIO_LSU_CC_DMA:
		res = -EIO;
		/* LSU Reg 6 - Flush this transaction */
		__raw_writel(1, &(krio_priv->regs->lsu_reg[lsu].busy_full));
		break;
	case KEYSTONE_RIO_LSU_CC_RETRY:
	case KEYSTONE_RIO_LSU_CC_CANCELED:
		res = -EAGAIN;
		break;
	default:
		dev_dbg(krio_priv->dev, "DBELL: doorbell sent\n");
		break;
	}

	return res;
}

/*---------------------- Maintenance Request Management  ---------------------*/

/**
 * keystone_rio_maint_request - Perform a maintenance request
 * @port_id: output port ID
 * @dest_id: destination ID of target device
 * @hopcount: hopcount for this request
 * @offset: offset in the RapidIO configuration space
 * @buff: dma address of the data on the host
 * @buff_len: length of the data
 * @size: 1 for 16bit, 0 for 8bit ID size
 * @type: packet type
 *
 * Returns %0 on success or %-EINVAL, %-EIO, %-EAGAIN or %-EBUSY on failure.
 */
static inline int keystone_rio_maint_request(int port_id,
					     u32 dest_id,
					     u8  hopcount,
					     u32 offset,
					     dma_addr_t buff,
					     int buff_len,
					     u16 size,
					     u16 type,
					     struct keystone_rio_data *krio_priv)
{
	unsigned int count;
	unsigned int status = 0;
	unsigned int res    = 0;
	u8           context;
	u8           ltid;
	u8           lsu = krio_priv->lsu_maint;

	mutex_lock(&krio_priv->lsu_lock);

	/* Check is there is space in the LSU shadow reg and that it is free */
	count = 0;
	while (1) {
		status = __raw_readl(&(krio_priv->regs->lsu_reg[lsu].busy_full));
		if (((status & KEYSTONE_RIO_LSU_FULL_MASK) == 0x0)
		    && ((status & KEYSTONE_RIO_LSU_BUSY_MASK) == 0x0))
			break;
		count++;

		if (count >= KEYSTONE_RIO_TIMEOUT_CNT) {
			dev_dbg(krio_priv->dev,
				"no LSU available, status = 0x%x\n", status);
			res = -EIO;
			goto out;
		}
		ndelay(1000);
	}

	/* Get LCB and LTID, LSU reg 6 is already read */
	context = (status >> 4) & 0x1;
	ltid    = status & 0xf;

	/* LSU Reg 0 - MSB of RapidIO address */
	__raw_writel(0, &(krio_priv->regs->lsu_reg[lsu].addr_msb));

	/* LSU Reg 1 - LSB of destination */
	__raw_writel(offset, &(krio_priv->regs->lsu_reg[lsu].addr_lsb_cfg_ofs));

	/* LSU Reg 2 - source address */
	__raw_writel(buff, &(krio_priv->regs->lsu_reg[lsu].phys_addr));

	/* LSU Reg 3 - byte count */
	__raw_writel(buff_len, &(krio_priv->regs->lsu_reg[lsu].dbell_val_byte_cnt));

	/* LSU Reg 4 - */
	__raw_writel(((port_id << 8)
		      | (KEYSTONE_RIO_LSU_PRIO << 4)
		      | (size ? (1 << 10) : 0)
		      | ((u32) dest_id << 16)),
		     &(krio_priv->regs->lsu_reg[lsu].destid));

	/* LSU Reg 5 */
	__raw_writel(((hopcount & 0xff) << 8) | (type & 0xff),
		     &(krio_priv->regs->lsu_reg[lsu].dbell_info_fttype));

	/* Retrieve our completion code */
	count = 0;
	res   = 0;
	while (1) {
		u8 lcb;
		status = keystone_rio_dio_get_lsu_cc(lsu, ltid, &lcb, krio_priv);
		if (lcb == context)
			break;
		count++;
		if (count >= KEYSTONE_RIO_TIMEOUT_CNT) {
			dev_dbg(krio_priv->dev,
				"timeout %d, ltid = %d, context = %d, "
				"lcb = %d, cc = %d\n",
				count, ltid, context, lcb, status);
			res = -EIO;
			break;
		}
		ndelay(1000);
	}
out:
	mutex_unlock(&krio_priv->lsu_lock);

	if (res)
		return res;

	if (status)
		dev_dbg(krio_priv->dev, "transfer error = 0x%x\n", status);

	switch (status) {
	case KEYSTONE_RIO_LSU_CC_TIMEOUT:
	case KEYSTONE_RIO_LSU_CC_XOFF:
	case KEYSTONE_RIO_LSU_CC_ERROR:
	case KEYSTONE_RIO_LSU_CC_INVALID:
	case KEYSTONE_RIO_LSU_CC_DMA:
		res = -EIO;
		/* LSU Reg 6 - Flush this transaction */
		__raw_writel(1 , &(krio_priv->regs->lsu_reg[lsu].busy_full));
		break;
	case KEYSTONE_RIO_LSU_CC_RETRY:
		res = -EBUSY;
		break;
	case KEYSTONE_RIO_LSU_CC_CANCELED:
		res = -EAGAIN;
		break;
	default:
		break;
	}

	return res;
}

static int keystone_rio_maint_read(struct keystone_rio_data *krio_priv,
				   int port_id,
				   u16 destid,
				   u16 size,
				   u8  hopcount,
				   u32 offset,
				   int len,
				   u32 *val)
{
	u32 *tbuf;
	int res;
	dma_addr_t dma;
	struct device *dev = krio_priv->dev;
	size_t align_len = L1_CACHE_ALIGN(len);

	tbuf = kzalloc(align_len, GFP_KERNEL | GFP_DMA);
	if (!tbuf)
		return -ENOMEM;

	dma = dma_map_single(dev, tbuf, len, DMA_FROM_DEVICE);

	res = keystone_rio_maint_request(port_id, destid, hopcount, offset, dma,
					 len, size, KEYSTONE_RIO_PACKET_TYPE_MAINT_R,
					 krio_priv);

	dma_unmap_single(dev, dma, len, DMA_FROM_DEVICE);

	/* Taking care of byteswap */
	switch (len) {
	case 1:
		*val = *((u8 *)tbuf);
		break;
	case 2:
		*val = ntohs(*((u16 *)tbuf));
		break;
	default:
		*val = ntohl(*((u32 *)tbuf));
		break;
	}

	dev_dbg(dev,
		"maint_r: index %d destid %d hopcount %d offset 0x%x "
		"len %d val 0x%x res %d\n",
		port_id, destid, hopcount, offset, len, *val, res);

	kfree(tbuf);

	return res;
}

static int keystone_rio_maint_write(struct keystone_rio_data *krio_priv,
				    int port_id,
				    u16 destid,
				    u16 size,
				    u8  hopcount,
				    u32 offset,
				    int len,
				    u32 val)
{
	u32 *tbuf;
	int res;
	dma_addr_t dma;
	struct device *dev = krio_priv->dev;
	size_t align_len = L1_CACHE_ALIGN(len);

	tbuf = kzalloc(align_len, GFP_KERNEL | GFP_DMA);
	if (!tbuf)
		return -ENOMEM;

	/* Taking care of byteswap */
	switch (len) {
	case 1:
		*tbuf = ((u8) val);
		break;
	case 2:
		*tbuf = htons((u16) val);
		break;
	default:
		*tbuf = htonl((u32) val);
		break;
	}

	dma = dma_map_single(dev, tbuf, len, DMA_TO_DEVICE);

	res = keystone_rio_maint_request(port_id, destid, hopcount, offset, dma,
					 len, size, KEYSTONE_RIO_PACKET_TYPE_MAINT_W,
					 krio_priv);

	dma_unmap_single(dev, dma, len, DMA_TO_DEVICE);

	dev_dbg(dev,
		"maint_w: index %d destid %d hopcount %d offset 0x%x "
		"len %d val 0x%x res %d\n",
		port_id, destid, hopcount, offset, len, val, res);

	kfree(tbuf);

	return res;
}

/*------------------------- RapidIO SerDes setup ---------------------*/

#define K2_SERDES(p)      ((p)->board_rio_cfg.keystone2_serdes)
#define SERDES_LANE(lane) (1 << (lane))
#define SERDES_MAX_LANES  4   /* Max number of lanes for SRIO */
#define SERDES_LANE_MASK  0xf /* lane mask */

#define for_each_lanes(lane_mask, lane)					\
	for_each_set_bit((lane),					\
			 (const long unsigned int *) &(lane_mask),	\
			 SERDES_MAX_LANES)

#define reg_rmw(addr, value, mask)					\
	__raw_writel(((__raw_readl(addr) & (~(mask))) | ((value) & (mask))), \
		     (addr))

#define reg_fmkr(msb, lsb, val) \
	(((val) & ((1 << ((msb) - (lsb) + 1)) - 1)) << (lsb))

#define reg_finsr(addr, msb, lsb, val)					\
	__raw_writel(((__raw_readl(addr)				\
		       & ~(((1 << ((msb) - (lsb) + 1)) - 1) << (lsb)))	\
		      | reg_fmkr(msb, lsb, val)), (addr))

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

	tmp = (__raw_readl(regs + 0x0ec) >> 24) & 0x000ff;
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
		+ msecs_to_jiffies(KEYSTONE_PLL_LOCK_TIMEOUT);
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

static void k2_rio_serdes_reg_fill_field(void __iomem *regs,
					 int offset,
					 int width,
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

/*
 * Do the Att and Boost workaround
 *
 * This is only valid for SerDes PHY-A (i.e. all SerDes except XGE)
 */
static void k2_rio_serdes_att_boost(void __iomem *regs, int lane)
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

	k2_rio_serdes_reg_fill_field(regs + (lane * 0x200) + 0x200 + 0x8c,
				     8, 4, att_read); /* att_start */
	k2_rio_serdes_reg_fill_field(regs + (lane * 0x200) + 0x200 + 0x84,
				     0, 1, 0);
	k2_rio_serdes_reg_fill_field(regs + 0x0a00 + 0x8c, 24, 1, 0);

	/* Force cal */
	k2_rio_serdes_reg_fill_field(regs + (lane * 0x200) + 0x200 + 0xac,
				     11, 1, 1);
	k2_rio_serdes_reg_fill_field(regs + (lane * 0x200) + 0x200 + 0xac,
				     11, 1, 0);

	k2_rio_serdes_wait_rx_valid(regs, lane);

	/* Check boost value */
	boost_read = (k2_rio_serdes_sb_read_selected_tbus(regs, lane + 1, 0x11)
		      >> 8) & 0xf;

	/* If boost = 0, increment by 1 */
	if (boost_read != 0)
		goto do_not_inc;

	/* Set rxeq_ovr_en to 1 */
	k2_rio_serdes_reg_fill_field(regs + (lane * 0x200) + 0x200 + 0x2c,
				     2, 1, 1);

	/* Set rxeq_ovr_load_en for boost only */
	k2_rio_serdes_reg_fill_field(regs + (lane * 0x200) + 0x200 + 0x2c,
				     12, 7, 0x2);

	/* Set rxeq_ovr_load for a value of 1 */
	k2_rio_serdes_reg_fill_field(regs + (lane * 0x200) + 0x200 + 0x2c,
				     3, 7, 0x1);

	/* Latch in new boost value */
	k2_rio_serdes_reg_fill_field(regs + (lane * 0x200) + 0x200 + 0x2c,
				     10, 1, 0x1);
	k2_rio_serdes_reg_fill_field(regs + (lane * 0x200) + 0x200 + 0x2c,
				     10, 1, 0x0);

	/* Reset previous registers */
	k2_rio_serdes_reg_fill_field(regs + (lane * 0x200) + 0x200 + 0x2c,
				     2, 1, 0x0);
	k2_rio_serdes_reg_fill_field(regs + (lane * 0x200) + 0x200 + 0x2c,
				     12, 7, 0x0);
	k2_rio_serdes_reg_fill_field(regs + (lane * 0x200) + 0x200 + 0x2c,
				     3, 7, 0x0);

do_not_inc:
	/* Write back initial att start value */
	k2_rio_serdes_reg_fill_field(regs + (lane * 0x200) + 0x200 + 0x8c,
				     8, 4, att_start);

	/* Turn back on att adaptation */
	k2_rio_serdes_reg_fill_field(regs + (lane * 0x200) + 0x200 + 0x84,
				     0, 1, 1);
	k2_rio_serdes_reg_fill_field(regs + 0x0a00 + 0x8c, 24, 1, 1);

	k2_rio_serdes_wait_rx_valid(regs, lane);
}

/*
 * Read termination on TX-N and TX-P
 */
static inline u32 k2_rio_serdes_get_termination(void __iomem *regs)
{
	return k2_rio_serdes_sb_read_selected_tbus(regs, 1, 0x1b) & 0x00ff;
}

/*
 * Do the termination workaround
 */
static void k2_rio_serdes_termination_config(void __iomem *regs,
					     u32 lane, u32 tx_term_np)
{
	/* Set tx termination */
	k2_rio_serdes_reg_fill_field(regs + (lane * 0x200) + 0x200 + 0x7c,
				     24, 8, tx_term_np);

	/* Set termination override */
	k2_rio_serdes_reg_fill_field(regs + (lane * 0x200) + 0x200 + 0x7c,
				     20, 1, 1);
}

static void k2_rio_serdes_display_att_boost(void __iomem *regs, int lane,
					    struct device *dev)
{
	u32 att;
	u32 boost;

	/*  Read att and boost */
	att = boost = k2_rio_serdes_sb_read_selected_tbus(regs, lane + 1, 0x11);
	att   = (att   >> 4) & 0x0f;
	boost = (boost >> 8) & 0x0f;

	dev_dbg(dev, "SerDes: lane %d att = %d, boost = %d\n", lane, att,
		boost);

	return;
}

/*
 * Forcing the Tx idle on the transmitter will prevent the receiver from
 * adapting to the un-terminated Tx and un-programmed pre and post cursor
 * settings
 */
static inline void k2_rio_serdes_force_tx_idle(void __iomem *regs, u32 lane)
{
	k2_rio_serdes_reg_fill_field(regs + (lane * 4) + 0x1fc0 + 0x20,
				     24, 2, 3);
}

static inline void k2_rio_serdes_force_tx_normal(void __iomem *regs, u32 lane)
{
	k2_rio_serdes_reg_fill_field(regs + (lane * 4) + 0x1fc0 + 0x20,
				     24, 2, 0);
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
	ui_tmpo |= (1 << 14);                     /* set override */
	ui_tmpo &= ~0x60;                         /* clear the tx valid bits */

	/* Only modify the reset bit and the overlay bit */
	k2_rio_serdes_reg_fill_field(regs + (lane * 0x200) + 0x200 + 0x28,
				     15, 15, ui_tmpo);
}

static void k2_rio_serdes_sb_deassert_reset(void __iomem *regs, u32 lane)
{
	/* Clear the reset bit */
	k2_rio_serdes_reg_fill_field(regs + (lane * 0x200) + 0x200 + 0x28,
				     15, 1, 1);
}

static void k2_rio_serdes_sb_clear_overlay_bit29(void __iomem *regs, u32 lane)
{
	/* Clear overlay bit, bring the lane out of reset */
	k2_rio_serdes_reg_fill_field(regs + (lane * 0x200) + 0x200 + 0x28,
				     29, 1, 0);
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
	case KEYSTONE_RIO_FULL_RATE:
		__raw_writel(0xf3c0f0f0, regs + 0x1fe0 + (4 * lane));
		break;
	case KEYSTONE_RIO_HALF_RATE:
		__raw_writel(0xf7c0f4f0, regs + 0x1fe0 + (4 * lane));
		break;
	case KEYSTONE_RIO_QUARTER_RATE:
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
	timeout = jiffies + msecs_to_jiffies(KEYSTONE_PLL_LOCK_TIMEOUT);
	while (1) {
		/* Read PLL_CTRL */
		val = __raw_readl(regs + 0x1ff4);
		if ((val & val_mask) == val_mask)
			break;
		if (time_after(jiffies, timeout))
			return -1;
		usleep_range(10, 50);
	}

	return 0;
}

/*
 * Configure SerDes with appropriate baudrate and do Tx termination workaround
 * Note that all lanes are configured but Serdes are then disabled
 */
static int k2_rio_serdes_config(u32 lanes, u32 baud, struct device *dev,
				void __iomem *regs)
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
	case KEYSTONE_RIO_BAUD_1_250:
		rate = KEYSTONE_RIO_QUARTER_RATE;
		k2_rio_serdes_init_5g(regs);
		break;
	case KEYSTONE_RIO_BAUD_2_500:
		rate = KEYSTONE_RIO_HALF_RATE;
		k2_rio_serdes_init_5g(regs);
		break;
	case KEYSTONE_RIO_BAUD_5_000:
		rate = KEYSTONE_RIO_FULL_RATE;
		k2_rio_serdes_init_5g(regs);
		break;
	case KEYSTONE_RIO_BAUD_3_125:
		rate = KEYSTONE_RIO_HALF_RATE;
		k2_rio_serdes_init_3g(regs);
		break;
	default:
		return -EINVAL;
	}

	/* We need to always use lane 0 even if not part of the lane mask */
	lanes |= SERDES_LANE(0);

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

static int k2_rio_serdes_shutdown(void __iomem *regs, u32 lanes)
{
	u32 lane;

	for_each_lanes(lanes, lane) {
		/* Disable SerDes for this lane */
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
	k2_rio_serdes_reg_fill_field(regs + (lane * 0x200) + 0x200 + 0x04,
				     1, 2, 2);

	usleep_range(10, 50);

	/* Allow SerDes to re-acquire signal detect */
	k2_rio_serdes_reg_fill_field(regs + (lane * 0x200) + 0x200 + 0x04,
				     1, 2, 0);

	dev_dbg(dev, "SerDes: lane %d Rx path reset done\n", lane);

	/* Perform SerDes boost/att workaround */
	k2_rio_serdes_att_boost(regs, lane);

	dev_dbg(dev, "SerDes: lane %d boost/att workaround done\n", lane);

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

static int k2_rio_serdes_tx_lanes_start(
	u32 lanes,
	struct device *dev,
	void __iomem *regs,
	struct keystone_serdes_config *serdes_config)
{
	u32 val;
	u32 lane;

	for_each_lanes(lanes, lane) {
		dev_dbg(dev, "SerDes: start transmit for lane %d\n", lane);

		/* Serdes assert reset */
		k2_rio_serdes_sb_assert_reset(regs, lane);

		/* Set c1/c2/cm */
		val = (serdes_config->serdes_c1)
			| ((serdes_config->serdes_c2) << 8)
			| ((serdes_config->serdes_cm) << 12);
		reg_rmw(regs + 0x008 + (0x200 * (lane + 1)), val, 0x0000ff1f);

		/* Set att and 1lsb */
		val = (serdes_config->serdes_att << 25)
			| (serdes_config->serdes_1sb << 31);
		reg_rmw(regs + 0x004 + (0x200 * (lane + 1)), val, 0x9e000000);

		/* Set vreg */
		val = serdes_config->serdes_vreg << 5;
		reg_rmw(regs + 0x084 + (0x200 * (lane + 1)), val, 0x000000e0);

		/* Force Tx normal to enable the transmitter */
		k2_rio_serdes_force_tx_normal(regs, lane);

		/* Serdes de-assert reset */
		k2_rio_serdes_sb_deassert_reset(regs, lane);

		/* Enable corresponding lane */
		k2_rio_serdes_lane_enable(lane, regs);
	}

	for_each_lanes(lanes, lane) {
		/* Clear overlay bit to bring lane is out of reset */
		k2_rio_serdes_sb_clear_overlay_bit29(regs, lane);
	}

	return 0;
}

/*------------------------- RapidIO hw controller setup ---------------------*/

/* Retrieve the corresponding lanes bitmask from ports bitmask and path_mode */
static int keystone_rio_get_lane_config(u32 ports, u32 path_mode)
{
	u32 lanes = 0;

	while (ports) {
		u32 lane;
		u32 port = __ffs(ports);
		ports &= ~(1 << port);

		if (keystone_lane_configs[path_mode][port].start == -1)
			return -1;

		for (lane = keystone_lane_configs[path_mode][port].start;
		     lane < keystone_lane_configs[path_mode][port].end;
		     lane++) {
			lanes |= SERDES_LANE(lane);
		}
	}
	return (int) lanes;
}

/**
 * keystone_rio_lanes_init_and_wait - Initialize and wait lanes for a given
 * RIO port
 *
 * @port: RIO port
 * @start: if non null, lanes will be started
 *
 * Returns %0 on success or %1 if lane is not OK during the expected timeout
 */
static int keystone_rio_lanes_init(u32 port, int start,
				   struct keystone_rio_data *krio_priv)
{
	u32 path_mode = krio_priv->board_rio_cfg.path_mode;
	int lanes     = keystone_rio_get_lane_config(1 << port,
						     path_mode);
	void __iomem *regs =
		(void __iomem *) krio_priv->serdes_regs;
	struct keystone_serdes_config *serdes_config =
		&(krio_priv->board_rio_cfg.serdes_config[port]);
	int res;

	/* This is only needed for K2 SerDes */
	if (!K2_SERDES(krio_priv))
		return 1;

	dev_dbg(krio_priv->dev,
		"Initializing lane mask 0x%x for port %d",
		lanes, port);

	/* Eventually start the lanes */
	if (start) {
		dev_dbg(krio_priv->dev,
			"Starting lane mask 0x%x for port %d",
			lanes, port);

		k2_rio_serdes_tx_lanes_start((u32) lanes,
					     krio_priv->dev,
					     krio_priv->serdes_regs,
					     serdes_config);
	}

	/* Wait lanes to be OK */
	res = k2_rio_serdes_wait_lanes_ok(lanes, regs);
	if (res < 0) {
		dev_dbg(krio_priv->dev,
			"port %d lane mask 0x%x is not OK\n",
			port, lanes);

		return 1;
	}

	return 0;
}

/**
 * keystone_rio_hw_init - Configure a RapidIO controller
 * @mode: serdes configuration
 * @baud: serdes baudrate
 *
 * Returns %0 on success or %-EINVAL or %-EIO on failure.
 */
static int keystone_rio_hw_init(u32 mode, u32 baud,
				struct keystone_rio_data *krio_priv)
{
	struct keystone_serdes_config *serdes_config
		= &(krio_priv->board_rio_cfg.serdes_config[mode]);
	u32 val;
	u32 block;
	u32 port;
	int res = 0;
	int i;

	/* Reset blocks */
	__raw_writel(0, &krio_priv->regs->gbl_en);
	for (block = 0; block <= KEYSTONE_RIO_BLK_NUM; block++)
		__raw_writel(0, &(krio_priv->regs->blk[block].enable));

	ndelay(1000);

	/* Set sRIO out of reset */
	__raw_writel((KEYSTONE_RIO_PER_RESTORE | KEYSTONE_RIO_PER_FREE),
		     &krio_priv->regs->pcr);

	/* Disable blocks */
	__raw_writel(0, &krio_priv->regs->gbl_en);
	for (block = 0; block <= KEYSTONE_RIO_BLK_NUM; block++)
		__raw_writel(0, &(krio_priv->regs->blk[block].enable));

	/* Clear BOOT_COMPLETE bit (allowing write) */
	__raw_writel(0x00000000, &krio_priv->regs->per_set_cntl);

	/* Set the sRIO shadow registers for 9/3/2/2 */
	__raw_writel(0x00190019, &krio_priv->regs->lsu_setup_reg[0]);

	/* Enable blocks */
	__raw_writel(1, &krio_priv->regs->gbl_en);
	for (block = 0; block <= KEYSTONE_RIO_BLK_NUM; block++)
		__raw_writel(1, &(krio_priv->regs->blk[block].enable));

	/* Set control register 1 configuration */
	__raw_writel(0x00000000, &krio_priv->regs->per_set_cntl1);

	/* Set control register */
	__raw_writel(0x0009c000 | (krio_priv->board_rio_cfg.pkt_forwarding ?
				   BIT(21) | BIT(8) : 0),
		     &krio_priv->regs->per_set_cntl);

	if (!K2_SERDES(krio_priv)) {
		unsigned long timeout;

		/* K1 SerDes main configuration */
		__raw_writel(serdes_config->serdes_cfg_pll,
			     &krio_priv->serdes_regs->pll);

		/* Per-port SerDes configuration */
		for (port = 0; port < KEYSTONE_RIO_MAX_PORT; port++) {
			__raw_writel(serdes_config->rx_chan_config[port],
				     &krio_priv->serdes_regs->channel[port].rx);
			__raw_writel(serdes_config->tx_chan_config[port],
				     &krio_priv->serdes_regs->channel[port].rx);
		}

		/* Check for RIO SerDes PLL lock */
		res = 0;
		timeout = jiffies + msecs_to_jiffies(KEYSTONE_PLL_LOCK_TIMEOUT);
		while (1) {
			val = __raw_readl(krio_priv->serdes_sts_reg);
			if ((val & 0x1) != 0x1)
				break;
			if (time_after(jiffies, timeout)) {
				res = -1;
				break;
			}
			usleep_range(10, 50);
		}
	} else {
		u32 path_mode = krio_priv->board_rio_cfg.path_mode;
		u32 ports     = krio_priv->board_rio_cfg.ports;

		/* K2 SerDes main configuration */
		res = keystone_rio_get_lane_config(ports, path_mode);
		if (res > 0) {
			u32 lanes = (u32) res;

			/* Initialize KeyStone 2 Serdes */
			res = k2_rio_serdes_config(lanes, baud, krio_priv->dev,
						   krio_priv->serdes_regs);
		}

		if (res < 0) {
			dev_err(krio_priv->dev,
				"invalid lane config for port mask 0x%x\n",
				ports);
		}
	}

	if (res < 0) {
		dev_err(krio_priv->dev, "initialization of SerDes failed\n");
		return res;
	}

	/* Set prescalar for ip_clk */
	__raw_writel(serdes_config->prescalar_srv_clk,
		     &krio_priv->link_regs->prescalar_srv_clk);

	/* Peripheral-specific configuration and capabilities */
	__raw_writel(KEYSTONE_RIO_DEV_ID_VAL,
		     &krio_priv->car_csr_regs->dev_id);
	__raw_writel(KEYSTONE_RIO_DEV_INFO_VAL,
		     &krio_priv->car_csr_regs->dev_info);
	__raw_writel(KEYSTONE_RIO_ID_TI,
		     &krio_priv->car_csr_regs->assembly_id);
	__raw_writel(KEYSTONE_RIO_EXT_FEAT_PTR,
		     &krio_priv->car_csr_regs->assembly_info);
	__raw_writel(0x00ffffff,
		     &krio_priv->car_csr_regs->base_dev_id);

	krio_priv->rio_pe_feat = RIO_PEF_PROCESSOR
		| RIO_PEF_CTLS
		| KEYSTONE_RIO_PEF_FLOW_CONTROL
		| RIO_PEF_EXT_FEATURES
		| RIO_PEF_ADDR_34
		| RIO_PEF_STD_RT
		| RIO_PEF_INB_DOORBELL
		| RIO_PEF_INB_MBOX;

	__raw_writel(krio_priv->rio_pe_feat,
		     &krio_priv->car_csr_regs->pe_feature);

	__raw_writel(KEYSTONE_RIO_MAX_PORT << 8,
		     &krio_priv->car_csr_regs->sw_port);

	__raw_writel((RIO_SRC_OPS_READ
		      | RIO_SRC_OPS_WRITE
		      | RIO_SRC_OPS_STREAM_WRITE
		      | RIO_SRC_OPS_WRITE_RESPONSE
		      | RIO_SRC_OPS_DATA_MSG
		      | RIO_SRC_OPS_DOORBELL
		      | RIO_SRC_OPS_ATOMIC_TST_SWP
		      | RIO_SRC_OPS_ATOMIC_INC
		      | RIO_SRC_OPS_ATOMIC_DEC
		      | RIO_SRC_OPS_ATOMIC_SET
		      | RIO_SRC_OPS_ATOMIC_CLR
		      | RIO_SRC_OPS_PORT_WRITE),
		     &krio_priv->car_csr_regs->src_op);

	__raw_writel((RIO_DST_OPS_READ
		      | RIO_DST_OPS_WRITE
		      | RIO_DST_OPS_STREAM_WRITE
		      | RIO_DST_OPS_WRITE_RESPONSE
		      | RIO_DST_OPS_DATA_MSG
		      | RIO_DST_OPS_DOORBELL
		      | RIO_DST_OPS_PORT_WRITE),
		     &krio_priv->car_csr_regs->dest_op);

	__raw_writel(RIO_PELL_ADDR_34,
		     &krio_priv->car_csr_regs->pe_logical_ctl);

	val = (((KEYSTONE_RIO_SP_HDR_NEXT_BLK_PTR & 0xffff) << 16) |
	       KEYSTONE_RIO_SP_HDR_EP_REC_ID);
	__raw_writel(val, &krio_priv->serial_port_regs->sp_maint_blk_hdr);

	/* clear high bits of local config space base addr */
	__raw_writel(0x00000000, &krio_priv->car_csr_regs->local_cfg_hbar);

	/* set local config space base addr */
	__raw_writel(0x00520000, &krio_priv->car_csr_regs->local_cfg_bar);

	/* Enable HOST BIT(31) & MASTER_ENABLE BIT(30) bits */
	__raw_writel(0xc0000000, &krio_priv->serial_port_regs->sp_gen_ctl);

	/* set link timeout value */
	__raw_writel(0x000FFF00,
		     &krio_priv->serial_port_regs->sp_link_timeout_ctl);

	/* set response timeout value */
	__raw_writel(0x000FFF00,
		     &krio_priv->serial_port_regs->sp_rsp_timeout_ctl);

	/* allows SELF_RESET and PWDN_PORT resets to clear stcky reg bits */
	__raw_writel(0x00000001, &krio_priv->link_regs->reg_rst_ctl);

	/* clear all errors */
	__raw_writel(0x00000000, &krio_priv->err_mgmt_regs->err_det);

	/* disable all error detection */
	__raw_writel(0x00000000, &krio_priv->err_mgmt_regs->err_en);
	__raw_writel(0x00000000, &krio_priv->link_regs->local_err_en);

	/* set err det block header */
	val = (((KEYSTONE_RIO_ERR_HDR_NEXT_BLK_PTR & 0xffff) << 16) |
	       KEYSTONE_RIO_ERR_EXT_FEAT_ID);
	__raw_writel(val, &krio_priv->err_mgmt_regs->err_report_blk_hdr);

	/* clear msb of err captured addr reg */
	__raw_writel(0x00000000, &krio_priv->err_mgmt_regs->h_addr_capt);

	/* clear lsb of err captured addr reg */
	__raw_writel(0x00000000, &krio_priv->err_mgmt_regs->addr_capt);

	/* clear err captured source and dest DevID reg */
	__raw_writel(0x00000000, &krio_priv->err_mgmt_regs->id_capt);

	/* clear err captured packet info */
	__raw_writel(0x00000000, &krio_priv->err_mgmt_regs->ctrl_capt);

	__raw_writel(0x41004141, &krio_priv->phy_regs->phy_sp[0].__rsvd[3]);

	/* Set packet forwarding */
	for (i = 0; i < KEYSTONE_RIO_MAX_PKT_FW_ENTRIES; i++) {
		if ((krio_priv->board_rio_cfg.pkt_forwarding) && (i < 8)) {
			struct keystone_routing_config *routing = krio_priv->board_rio_cfg.routing_config;

			/* Enable packet forwarding DevId and port defined in DTS */
			__raw_writel(routing[i].dev_id_low
				     | (routing[i].dev_id_high << 16),
				     &(krio_priv->regs->pkt_fwd_cntl[i].pf_16b));
			__raw_writel((routing[i].dev_id_low & 0xff)
				     | ((routing[i].dev_id_high & 0xff) << 8)
				     | (routing[i].port << 16),
				     &(krio_priv->regs->pkt_fwd_cntl[i].pf_8b));

			dev_info(krio_priv->dev,
				 "enabling packet forwarding to port %d for DestID 0x%04x - 0x%04x\n",
				 routing[i].port, routing[i].dev_id_low, routing[i].dev_id_high);
		} else {
			/* Disable packet forwarding */
			__raw_writel(0xffffffff, &(krio_priv->regs->pkt_fwd_cntl[i].pf_16b));
			__raw_writel(0x0003ffff, &(krio_priv->regs->pkt_fwd_cntl[i].pf_8b));
		}
	}
	if (!krio_priv->board_rio_cfg.pkt_forwarding)
		dev_info(krio_priv->dev, "packet forwarding disabled\n");

	/* Force all writes to finish */
	val = __raw_readl(&krio_priv->err_mgmt_regs->ctrl_capt);

	return res;
}

/**
 * keystone_rio_start - Start RapidIO controller
 */
static void keystone_rio_start(struct keystone_rio_data *krio_priv)
{
	u32 val;

	/* Set PEREN bit to enable logical layer data flow */
	val = (KEYSTONE_RIO_PER_EN | KEYSTONE_RIO_PER_FREE);
	__raw_writel(val, &krio_priv->regs->pcr);

	/* Set BOOT_COMPLETE bit */
	val = __raw_readl(&krio_priv->regs->per_set_cntl);
	__raw_writel(val | KEYSTONE_RIO_BOOT_COMPLETE,
		     &krio_priv->regs->per_set_cntl);
}

/**
 * keystone_rio_stop - Stop RapidIO controller
 */
static void keystone_rio_stop(struct keystone_rio_data *krio_priv)
{
	u32 val;

	/* Disable PEREN bit to stop all new logical layer transactions */
	val = __raw_readl(&krio_priv->regs->pcr);
	val &= ~KEYSTONE_RIO_PER_EN;
	__raw_writel(val, &krio_priv->regs->pcr);
}

static int keystone_rio_test_link(u8 port, struct keystone_rio_data *krio_priv)
{
	int res;
	u32 value;

	res = keystone_rio_maint_read(krio_priv, port, 0xffff,
				      krio_priv->board_rio_cfg.size,
				      0, 0, sizeof(value), &value);
	return res;
}

static int keystone_rio_get_remote_port(u8 port, struct keystone_rio_data *krio_priv)
{
	int res;
	u32 value;

	res = keystone_rio_maint_read(krio_priv, port, 0xffff,
				      krio_priv->board_rio_cfg.size,
				      0, RIO_SWP_INFO_CAR, 4, &value);
	if (res < 0)
		return res;

	return RIO_GET_PORT_NUM(value);
}

static int keystone_rio_port_error_recovery(u32 port, struct keystone_rio_data *krio_priv)
{
	int res;
	u32 err_stat;
	u32 err_det;
	u32 plm_status;
	int r_port = krio_priv->board_rio_cfg.ports_remote[port];
	int i;

	if (unlikely(port >= KEYSTONE_RIO_MAX_PORT))
		return -EINVAL;

	err_stat   = __raw_readl(&krio_priv->serial_port_regs->sp[port].err_stat);
	err_det    = __raw_readl(&krio_priv->err_mgmt_regs->sp_err[port].det);
	plm_status = __raw_readl(&krio_priv->phy_regs->phy_sp[port].status);

	dev_dbg(krio_priv->dev,
		"ER port %d: err_stat = 0x%08x, err_det = 0x%08x, "
		"plm_status = 0x%08x\n",
		port, err_stat, err_det, plm_status);

	if (unlikely(!(err_stat & RIO_PORT_N_ERR_STS_PORT_OK))) {
		dev_dbg(krio_priv->dev,
			"ER port %d not initialized - PORT_OK not set\n", port);
		return -EINVAL;
	}

	/* Acknowledge errors on this port */
	__raw_writel(err_stat & KEYSTONE_RIO_PORT_ERROR_MASK,
		     &krio_priv->serial_port_regs->sp[port].err_stat);
	__raw_writel(0, &krio_priv->err_mgmt_regs->sp_err[port].det);
	__raw_writel(plm_status & KEYSTONE_RIO_PORT_PLM_STATUS_ERRORS,
		     &krio_priv->phy_regs->phy_sp[port].status);

	if (err_stat & RIO_PORT_N_ERR_STS_PW_OUT_ES) {
		u32 lm_resp;
		u32 ackid_stat;
		u32 l_ackid;
		u32 r_ackid;

		dev_dbg(krio_priv->dev,
			"ER port %d: Output Error-Stopped recovery\n", port);

		/*
		 * Clear valid bit in maintenance response register.
		 * Send both Input-Status Link-Request and PNA control symbols and
		 * wait for valid maintenance response
		 */
		__raw_readl(&krio_priv->serial_port_regs->sp[port].link_maint_resp);
		__raw_writel(0x2003f044,
			     &krio_priv->phy_regs->phy_sp[port].long_cs_tx1);
		i = 0;
		do {
			if (++i > KEYSTONE_RIO_TIMEOUT_CNT) {
				dev_dbg(krio_priv->dev,
					"ER port %d: Input-Status response "
					"timeout\n", port);
				goto oes_rd_err;
			}

			ndelay(KEYSTONE_RIO_TIMEOUT_NSEC);
			lm_resp = __raw_readl(
				&krio_priv->serial_port_regs->sp[port].link_maint_resp);
		} while (!(lm_resp & RIO_PORT_N_MNT_RSP_RVAL));

		dev_dbg(krio_priv->dev,
			"ER port %d: Input-Status response = 0x%08x\n",
			port, lm_resp);

		/* Set outbound ackID to the value expected by link partner */
		ackid_stat = __raw_readl(
			&krio_priv->serial_port_regs->sp[port].ackid_stat);

		dev_dbg(krio_priv->dev,
			"ER port %d: ackid_stat = 0x%08x\n", port, ackid_stat);

		l_ackid = (ackid_stat & RIO_PORT_N_ACK_INBOUND) >> 24;
		r_ackid = (lm_resp & RIO_PORT_N_MNT_RSP_ASTAT) >> 5;

		__raw_writel((l_ackid << 24) | r_ackid,
			     &krio_priv->serial_port_regs->sp[port].ackid_stat);

		/*
		 * We do not know the remote port but we may be lucky where
		 * ackId did not changed...
		 */
		if (r_port < 0) {
			dev_dbg(krio_priv->dev,
				"ER port %d: remote port not yet detected!\n",
				port);
			return -EINVAL;
		}

		udelay(50);

		/*
		 * Reread outbound ackID as it may have changed as a result of
		 * outstanding unacknowledged packets retransmission
		 */
		ackid_stat = __raw_readl(
			&krio_priv->serial_port_regs->sp[port].ackid_stat);

		dev_dbg(krio_priv->dev,
			"ER port %d: ackid_stat = 0x%08x\n", port, ackid_stat);

		r_ackid = ackid_stat & RIO_PORT_N_ACK_OUTBOUND;

		/*
		 * Set link partner inbound ackID to outbound ackID + 1.
		 * Set link partner outbound and outstanding ackID to inbound ackID.
		 */
		res = keystone_rio_maint_write(krio_priv,
					       port,
					       0xffff,
					       krio_priv->board_rio_cfg.size,
					       0,
					       0x100 + RIO_PORT_N_ACK_STS_CSR(r_port),
					       sizeof(u32),
					       ((++r_ackid << 24) & RIO_PORT_N_ACK_INBOUND) |
					       (l_ackid << 8) | l_ackid);

		if (res < 0) {
			dev_dbg(krio_priv->dev,
				"ER port %d: failed to align ackIDs with link "
				"partner port %d\n", port, r_port);
		}
	}

oes_rd_err:

	if (err_stat & RIO_PORT_N_ERR_STS_PW_INP_ES) {

		if (r_port < 0) {
			dev_dbg(krio_priv->dev,
				"ER port %d: remote port not yet detected!\n",
				port);
			return -EINVAL;
		}

		dev_dbg(krio_priv->dev,
			"ER port %d: Input Error-Stopped recovery\n", port);

		res = keystone_rio_maint_write(krio_priv,
					       port,
					       0xffff,
					       krio_priv->board_rio_cfg.size,
					       0,
					       0x100 + RIO_PORT_N_MNT_REQ_CSR(r_port),
					       sizeof(u32),
					       RIO_MNT_REQ_CMD_IS);

		if (res < 0) {
			dev_dbg(krio_priv->dev,
				"ER port %d: failed to issue Input-Status "
				"request from link partner port %d\n",
				port, r_port);
		}

		udelay(50);
	}

	err_stat = __raw_readl(&krio_priv->serial_port_regs->sp[port].err_stat);
	err_det = __raw_readl(&krio_priv->err_mgmt_regs->sp_err[port].det);
	plm_status = __raw_readl(&krio_priv->phy_regs->phy_sp[port].status);

	dev_dbg(krio_priv->dev,
		"ER port %d: ending with err_stat = 0x%08x, err_det = 0x%08x, "
		"plm_status = 0x%08x\n", port, err_stat, err_det, plm_status);

	return err_stat & KEYSTONE_RIO_PORT_ERRORS;
}

static void keystone_rio_pe_dpc(struct work_struct *work)
{
	struct keystone_rio_data *krio_priv = container_of(
		to_delayed_work(work), struct keystone_rio_data, pe_work);
	u32 port;

	dev_dbg(krio_priv->dev, "errors on ports: 0x%x\n", krio_priv->pe_ports);

	for (port = 0; port < KEYSTONE_RIO_MAX_PORT; port++) {
		if (test_and_clear_bit(port, (void *)&krio_priv->pe_ports)) {

			/* Wait lanes to be OK */
			if (keystone_rio_lanes_init(port, 0, krio_priv)) {
				dev_dbg(krio_priv->dev,
					"port %d: lanes are not OK\n", port);
				goto retry;
			}

			/*  Recover from port error state */
			if (keystone_rio_port_error_recovery(port,
							     krio_priv)) {
				dev_dbg(krio_priv->dev,
					"port %d: failed to perform error"
					" recovery\n",
					port);
				goto retry;
			}

			/* Perform test read on successful recovery */
			if (keystone_rio_test_link(port, krio_priv)) {
				dev_dbg(krio_priv->dev,
					"port %d: link test failed after "
					"error recovery\n",
					port);
				goto retry;
			}

			/* Continue to next port */
			continue;
retry:
			/*
			 * If error recovery failed schedule another one if
			 * there is time left
			 */
			if (krio_priv->pe_cnt-- > 1) {
				krio_priv->pe_ports |= BIT(port);
				schedule_delayed_work(
					&krio_priv->pe_work,
					KEYSTONE_RIO_REGISTER_DELAY);
			} else {
				dev_err(krio_priv->dev,
					"port %d: failed to recover from "
					"errors\n",
					port);
			}
		}
	}
}

/**
 * keystone_rio_port_status - Return if the port is OK or not
 * @port: index of the port
 *
 * Return %0 if the port is ready or %-EIO on failure.
 */
static int keystone_rio_port_status(int port, struct keystone_rio_data *krio_priv)
{
	unsigned int count = 0, value;
	int res = 0;
	int solid_ok = 0;

	if (port >= KEYSTONE_RIO_MAX_PORT)
		return -EINVAL;

	/* Check port status */
	for (count = 0; count < 300; count++) {
		value = __raw_readl(&(krio_priv->serial_port_regs->sp[port].err_stat));
		if (value & RIO_PORT_N_ERR_STS_PORT_OK) {
			solid_ok++;
			if (solid_ok == 100)
				break;
		} else {
			if (solid_ok) {
				dev_dbg(krio_priv->dev,
					"unstable port %d (solid_ok = %d)\n",
					port, solid_ok);
				goto port_error;
			}
			solid_ok = 0;
		}
		usleep_range(10, 50);
	}

	if (solid_ok == 100) {
		/* Test the link with a dummy maintenance read */
		res = keystone_rio_test_link(port, krio_priv);
		if (res != 0) {
			dev_dbg(krio_priv->dev,
				"link test failed on port %d\n", port);

			goto port_error;
		}

		/* Check if we need to retrieve the corresponding remote port */
		if (krio_priv->board_rio_cfg.ports_remote[port] < 0) {
			int rport;

			rport = keystone_rio_get_remote_port(port, krio_priv);
			if (rport < 0) {
				dev_warn(krio_priv->dev,
					 "cannot retrieve remote port on port %d\n", port);
				rport = port;
			} else {
				dev_info(krio_priv->dev,
					 "detected remote port %d on port %d\n",
					 rport, port);
			}
			krio_priv->board_rio_cfg.ports_remote[port] = rport;
		}
	} else {
		dev_dbg(krio_priv->dev,
			"port %d is not initialized - port is not solid ok\n",
			port);

		goto port_error;
	}

	return 0; /* Port must be solid OK */

port_error:
	/* We encountered an error on the port, try to fix the unstable lane */
	if (K2_SERDES(krio_priv)) {
		u32 path_mode = krio_priv->board_rio_cfg.path_mode;
		int lanes = keystone_rio_get_lane_config(1 << port,
							 path_mode);

		dev_dbg(krio_priv->dev,
			"fix unstable lane mask 0x%x for port %d\n",
			lanes, port);

		k2_rio_serdes_fix_unstable_lanes(lanes,
						 krio_priv->dev,
						 krio_priv->serdes_regs);
	}

	return -EIO;
}

/**
 * keystone_rio_port_disable - Disable a RapidIO port
 * @port: index of the port to configure
 */
static void keystone_rio_port_disable(u32 port, struct keystone_rio_data *krio_priv)
{
	/* Disable port */
	__raw_writel(0x800000, &(krio_priv->serial_port_regs->sp[port].ctl));
}

/**
 * keystone_rio_port_init - Configure a RapidIO port
 * @port: index of the port to configure
 * @mode: serdes configuration
 */
static int keystone_rio_port_init(u32 port, u32 path_mode, struct keystone_rio_data *krio_priv)
{
	if (port >= KEYSTONE_RIO_MAX_PORT)
		return -EINVAL;

	/* Silence and discovery timers */
	if ((port == 0) || (port == 2)) {
		__raw_writel(0x20000000,
			     &(krio_priv->phy_regs->phy_sp[port].silence_timer));
		__raw_writel(0x20000000,
			     &(krio_priv->phy_regs->phy_sp[port].discovery_timer));
	}

	/* Increase the number of valid code-groups required for sync */
	__raw_writel(0x0f030300, &(krio_priv->phy_regs->phy_sp[port].vmin_exp));

	/* Enable port in input and output */
	__raw_writel(0x600000, &(krio_priv->serial_port_regs->sp[port].ctl));

	/* Program channel allocation to ports (1x, 2x or 4x) */
	__raw_writel(path_mode, &(krio_priv->phy_regs->phy_sp[port].path_ctl));

	return 0;
}

/**
 * keystone_rio_port_set_routing - Configure routing for a RapidIO port
 * @port: index of the port to configure
 */
static void keystone_rio_port_set_routing(u32 port, struct keystone_rio_data *krio_priv)
{
	u32 base_dev_id = krio_priv->board_rio_cfg.size ?
		__raw_readl(&krio_priv->car_csr_regs->base_dev_id) & 0xffff :
		(__raw_readl(&krio_priv->car_csr_regs->base_dev_id) >> 16) & 0xff;

	u32 brr = KEYSTONE_RIO_PKT_FW_BRR_NUM;

	/* Enable routing to LLM for this BRR and port */
	__raw_writel(0x84000000,
		     &(krio_priv->transport_regs->transport_sp[port].base_route[brr].ctl));

	/*
	 * Configure the Base Routing Register (BRR) to ensure that all packets
	 * matching our DevId are admitted.
	 */
	__raw_writel((base_dev_id << 16) |
		     (krio_priv->board_rio_cfg.size ? 0xffff : 0xff),
		     &(krio_priv->transport_regs->transport_sp[port].base_route[brr].pattern_match));

	dev_dbg(krio_priv->dev, "pattern_match = 0x%x for BRR %d\n",
		__raw_readl(&krio_priv->transport_regs->transport_sp[port].base_route[brr].pattern_match),
		brr);

	/* Enable routing to LLM for this BRR and port */
	brr += 1;
	__raw_writel(0x84000000,
		     &(krio_priv->transport_regs->transport_sp[port].base_route[brr].ctl));

	/*
	 * Configure the Base Routing Register (BRR) to ensure that all broadcast
	 * packets are admitted as well.
	 */
	__raw_writel((0xffff << 16) |
		     (krio_priv->board_rio_cfg.size ? 0xffff : 0xff),
		     &(krio_priv->transport_regs->transport_sp[port].base_route[brr].pattern_match));

	dev_dbg(krio_priv->dev, "pattern_match = 0x%x for BRR %d\n",
		__raw_readl(&krio_priv->transport_regs->transport_sp[port].base_route[brr].pattern_match),
		brr);
}

/**
 * keystone_rio_port_activate - Start using a RapidIO port
 * @port: index of the port to configure
 */
static int keystone_rio_port_activate(u32 port, struct keystone_rio_data *krio_priv)
{
	u32 val;

	/*
	 * Disable all errors reporting if using packet forwarding
	 * otherwise enable them.
	 */
	__raw_writel((krio_priv->board_rio_cfg.pkt_forwarding) ?
		     0x0 : 0xffffffff,
		     &(krio_priv->err_mgmt_regs->sp_err[port].rate_en));

	/* Cleanup port error status */
	__raw_writel(KEYSTONE_RIO_PORT_ERROR_MASK,
		     &(krio_priv->serial_port_regs->sp[port].err_stat));
	__raw_writel(0, &(krio_priv->err_mgmt_regs->sp_err[port].det));

	/* Enable interrupt for reset request */
	val = __raw_readl(&(krio_priv->evt_mgmt_regs->evt_mgmt_rst_int_en));
	__raw_writel(val | (1 << port),
		     &(krio_priv->evt_mgmt_regs->evt_mgmt_rst_int_en));

	/* Enable all PLM interrupts */
	__raw_writel(0xffffffff,
		     &(krio_priv->phy_regs->phy_sp[port].int_enable));
	__raw_writel(1, &(krio_priv->phy_regs->phy_sp[port].all_int_en));

	/* Set multicast and packet forwarding mode otherwise unicast mode */
	val = krio_priv->board_rio_cfg.pkt_forwarding ? 0x00209000 : 0x00109000;
	__raw_writel(val, &(krio_priv->transport_regs->transport_sp[port].control));

	/* Disable generation of port-write request if packet forwarding used */
	if (krio_priv->board_rio_cfg.pkt_forwarding) {
		val = __raw_readl(
			&(krio_priv->evt_mgmt_regs->evt_mgmt_dev_port_wr_en));
		__raw_writel(
			val & 0xfffffffe,
			&(krio_priv->evt_mgmt_regs->evt_mgmt_dev_port_wr_en));

		__raw_writel(
			0,
			&(krio_priv->phy_regs->phy_sp[port].port_wr_enable));

		val = __raw_readl(
			&(krio_priv->phy_regs->phy_sp[port].all_port_wr_en));
		__raw_writel(
			val & 0xfffffffe,
			&(krio_priv->phy_regs->phy_sp[port].all_port_wr_en));
	}

	/* Enable Port-write reception capture */
	__raw_writel(0, &(krio_priv->port_write_regs->port_wr_rx_capt[port]));

	return 0;
}

/*------------------------- Configuration space mngt  ----------------------*/

/**
 * keystone_local_config_read - Generate a KeyStone local config space read
 * @mport: RapidIO master port info
 * @index: ID of RapidIO interface
 * @offset: Offset into configuration space
 * @len: Length (in bytes) of the maintenance transaction
 * @data: Value to be read into
 *
 * Generates a KeyStone local configuration space read. Returns %0 on
 * success or %-EINVAL on failure.
 */
static int keystone_local_config_read(struct rio_mport *mport,
				      int index, u32 offset, int len, u32 *data)
{
	struct keystone_rio_data *krio_priv = mport->priv;

	/*
	 * Workaround for rionet: the processing element features must content
	 * RIO_PEF_INB_MBOX and RIO_PEF_INB_DOORBELL bits that cannot be set on
	 * KeyStone hardware. So cheat the read value in this case...
	 */
	if (unlikely(offset == RIO_PEF_CAR))
		*data = krio_priv->rio_pe_feat;
	else
		*data = __raw_readl((void __iomem *)
				    (krio_priv->car_csr_regs_base + offset));

	dev_dbg(krio_priv->dev,
		"index %d offset 0x%x data 0x%x\n", index, offset, *data);

	return 0;
}

/**
 * keystone_local_config_write - Generate a KeyStone local config space write
 * @mport: RapidIO master port info
 * @index: ID of RapidIO interface
 * @offset: Offset into configuration space
 * @len: Length (in bytes) of the maintenance transaction
 * @data: Value to be written
 *
 * Generates a KeyStone local configuration space write. Returns %0 on
 * success or %-EINVAL on failure.
 */
static int keystone_local_config_write(struct rio_mport *mport,
				       int index, u32 offset, int len, u32 data)
{
	struct keystone_rio_data *krio_priv = mport->priv;

	dev_dbg(krio_priv->dev,
		"index %d offset 0x%x data 0x%x\n", index, offset, data);
	__raw_writel(data,
		     (void __iomem *)(krio_priv->car_csr_regs_base + offset));

	return 0;
}

/**
 * keystone_rio_config_read - Generate a KeyStone read maintenance transaction
 * @mport: RapidIO master port info
 * @index: ID of RapidIO interface
 * @destid: Destination ID of transaction
 * @hopcount: Number of hops to target device
 * @offset: Offset into configuration space
 * @len: Length (in bytes) of the maintenance transaction
 * @val: Location to be read into
 *
 * Generates a KeyStone read maintenance transaction. Returns %0 on
 * success or %-EINVAL on failure.
 */
static int
keystone_rio_config_read(struct rio_mport *mport, int index, u16 destid,
			 u8 hopcount, u32 offset, int len, u32 *val)
{
	return keystone_rio_maint_read((struct keystone_rio_data *)mport->priv,
				       mport->index, destid, mport->sys_size,
				       hopcount, offset, len, val);
}

/**
 * keystone__rio_config_write - Generate a KeyStone write
 * maintenance transaction
 * @mport: RapidIO master port info
 * @index: ID of RapidIO interface
 * @destid: Destination ID of transaction
 * @hopcount: Number of hops to target device
 * @offset: Offset into configuration space
 * @len: Length (in bytes) of the maintenance transaction
 * @val: Value to be written
 *
 * Generates an KeyStone write maintenance transaction. Returns %0 on
 * success or %-EINVAL on failure.
 */
static int
keystone_rio_config_write(struct rio_mport *mport, int index, u16 destid,
			  u8 hopcount, u32 offset, int len, u32 val)
{
	return keystone_rio_maint_write((struct keystone_rio_data *)mport->priv,
					mport->index, destid, mport->sys_size,
					hopcount, offset, len, val);
}

/*------------------------------- Port-Write management --------------------------*/

/**
 * keystone_rio_pw_enable - enable/disable port-write interface init
 * @mport: Master port implementing the port write unit
 * @enable: 1=enable; 0=disable port-write message handling
 */
static int keystone_rio_pwenable(struct rio_mport *mport, int enable)
{
	/* Enable/Disable port-write-in interrupt */
	return 0;
}

static void keystone_rio_pw_dpc(struct work_struct *work)
{
	struct keystone_rio_data *krio_priv =
		container_of(work, struct keystone_rio_data, pw_work);
	unsigned long flags;
	u32 msg_buffer[RIO_PW_MSG_SIZE / sizeof(u32)];

	/*
	 * Process port-write messages
	 */
	spin_lock_irqsave(&krio_priv->pw_fifo_lock, flags);
	while (kfifo_out(&krio_priv->pw_fifo,
			 (unsigned char *) msg_buffer,
			 RIO_PW_MSG_SIZE)) {

		/* Process one message */
		spin_unlock_irqrestore(&krio_priv->pw_fifo_lock, flags);

#ifdef KEYSTONE_RIO_DEBUG_PW
		{
			u32 i;
			printk(KERN_DEBUG "%s : Port-Write Message:", __func__);
			for (i = 0; i < RIO_PW_MSG_SIZE / sizeof(u32); i++) {
				if ((i % 4) == 0)
					printk(KERN_DEBUG "\n0x%02x: 0x%08x", i * 4,
					       msg_buffer[i]);
				else
					printk(KERN_DEBUG " 0x%08x", msg_buffer[i]);
			}
			printk(KERN_DEBUG "\n");
		}
#endif /* KEYSTONE_RIO_DEBUG_PW */

		/* Pass the port-write message to RIO core for processing */
		rio_inb_pwrite_handler((union rio_pw_msg *) msg_buffer);
		spin_lock_irqsave(&krio_priv->pw_fifo_lock, flags);
	}
	spin_unlock_irqrestore(&krio_priv->pw_fifo_lock, flags);
}

/**
 *  keystone_rio_port_write_handler - KeyStone port write interrupt handler
 *
 * Handles port write interrupts. Parses a list of registered
 * port write event handlers and executes a matching event handler.
 */
static void keystone_rio_port_write_handler(struct keystone_rio_data *krio_priv)
{
	int pw;

	/* Check that we have a port-write-in case */
	pw = __raw_readl(&(krio_priv->port_write_regs->port_wr_rx_stat)) & 0x1;

	/* Schedule deferred processing if PW was received */
	if (pw) {
		/*
		 * Retrieve PW message
		 */
		krio_priv->port_write_msg.msg.em.comptag =
			__raw_readl(&(krio_priv->port_write_regs->port_wr_rx_capt[0]));
		krio_priv->port_write_msg.msg.em.errdetect =
			__raw_readl(&(krio_priv->port_write_regs->port_wr_rx_capt[1]));
		krio_priv->port_write_msg.msg.em.is_port =
			__raw_readl(&(krio_priv->port_write_regs->port_wr_rx_capt[2]));
		krio_priv->port_write_msg.msg.em.ltlerrdet =
			__raw_readl(&(krio_priv->port_write_regs->port_wr_rx_capt[3]));

		/*
		 * Save PW message (if there is room in FIFO), otherwise discard it.
		 */
		if (kfifo_avail(&krio_priv->pw_fifo) >= RIO_PW_MSG_SIZE) {
			krio_priv->port_write_msg.msg_count++;
			kfifo_in(&krio_priv->pw_fifo,
				 (void const *) &krio_priv->port_write_msg.msg,
				 RIO_PW_MSG_SIZE);
		} else {
			krio_priv->port_write_msg.discard_count++;
			dev_warn(krio_priv->dev, "ISR Discarded Port-Write Msg(s) (%d)\n",
				 krio_priv->port_write_msg.discard_count);
		}
		schedule_work(&krio_priv->pw_work);
	}

	/* Acknowledge port-write-in */
	return;
}

/**
 * keystone_rio_port_write_init - KeyStone port write interface init
 * @mport: Master port implementing the port write unit
 *
 * Initializes port write unit hardware and buffer
 * ring. Called from keystone_rio_setup(). Returns %0 on success
 * or %-ENOMEM on failure.
 */
static int keystone_rio_port_write_init(struct keystone_rio_data *krio_priv)
{
	int i;

	/* Following configurations require a disabled port write controller */
	keystone_rio_pwenable(NULL, 0);

	/* Clear port-write-in capture registers */
	for (i = 0; i < 4; i++)
		__raw_writel(0, &(krio_priv->port_write_regs->port_wr_rx_capt[i]));

	INIT_WORK(&krio_priv->pw_work, keystone_rio_pw_dpc);
	spin_lock_init(&krio_priv->pw_fifo_lock);
	if (kfifo_alloc(&krio_priv->pw_fifo, RIO_PW_MSG_SIZE * 32, GFP_KERNEL)) {
		dev_err(krio_priv->dev, "FIFO allocation failed\n");
		return -ENOMEM;
	}
	return 0;
}

/*------------------------- Message passing management  ----------------------*/

/*
 * Retrieve MP receive code completion
 */
static inline u32 keystone_rio_mp_get_cc(u32 psdata1, u32 packet_type)
{
	return (packet_type == RIO_PACKET_TYPE_MESSAGE ?
		psdata1 >> 15 : psdata1 >> 8) & 0x3;
}

/*
 * This function retrieves the packet type for a given mbox
 */
static inline u32 keystone_rio_mp_get_type(int mbox,
					   struct keystone_rio_data *krio_priv)
{
	struct keystone_rio_rx_chan_info *krx_chan = &(krio_priv->rx_channels[mbox]);

	return (u32) ((krx_chan->packet_type != RIO_PACKET_TYPE_STREAM) &&
		      (krx_chan->packet_type != RIO_PACKET_TYPE_MESSAGE)) ?
		RIO_PACKET_TYPE_MESSAGE : krx_chan->packet_type;
}

/*
 * This function retrieves the mapping from Linux RIO mailbox to stream id for type 9 packets
 */
static inline u32 keystone_rio_mbox_to_strmid(int mbox,
					      struct keystone_rio_data *krio_priv)
{
	struct keystone_rio_rx_chan_info *krx_chan = &(krio_priv->rx_channels[mbox]);

	return (u32) krx_chan->stream_id;
}

static void keystone_rio_rx_complete(void *data)
{
	struct keystone_rio_packet *p_info = data;
	struct keystone_rio_data *krio_priv = p_info->priv;
	struct keystone_rio_rx_chan_info *krx_chan;
	struct keystone_rio_mbox_info *p_mbox;
	int mbox;

	mbox = (p_info->psdata[1] & 0x3f);
	p_info->mbox = mbox;

	krx_chan = &(krio_priv->rx_channels[mbox]);

	p_info->status = dma_async_is_tx_complete(krx_chan->dma_channel,
						  p_info->cookie, NULL, NULL);
	WARN_ON(p_info->status != DMA_COMPLETE && p_info->status != DMA_ERROR);

	p_mbox = &(krio_priv->rx_mbox[mbox]);

	spin_lock(&krx_chan->lock);
	list_add_tail(&p_info->node, &krx_chan->packet_list);
	spin_unlock(&krx_chan->lock);

	if (p_mbox->running) {
		/* Client callback (slot is not used) */
		p_mbox->port->inb_msg[p_mbox->id].mcback(p_mbox->port,
							 p_mbox->dev_id,
							 p_mbox->id,
							 0);
	}
}

/*
 * Release a free receive buffer
 */
static void keystone_rio_rxpool_free(void *arg, unsigned q_num, unsigned bufsize,
				     struct dma_async_tx_descriptor *desc)
{
       struct keystone_rio_rx_chan_info *krx_chan = arg;
       struct keystone_rio_data *krio_priv = krx_chan->priv;
       struct keystone_rio_packet *p_info = desc->callback_param;

       dma_unmap_sg(krio_priv->dev, &p_info->sg[2], 1, DMA_FROM_DEVICE);
       p_info->buff = NULL;
       kfree(p_info);

       return;
}

static void keystone_rio_chan_work_handler(unsigned long data)
{
	struct keystone_rio_data *krio_priv = (struct keystone_rio_data *)data;
	struct keystone_rio_rx_chan_info *krx_chan;
	int i;

	for (i = 0; i < KEYSTONE_RIO_MAX_MBOX; i++) {
		krx_chan = &(krio_priv->rx_channels[i]);
		if (krx_chan->dma_channel) {
			dma_poll(krx_chan->dma_channel, -1);
			dmaengine_resume(krx_chan->dma_channel);
		}
	}
}

/* Allocate a free receive buffer */
static struct dma_async_tx_descriptor *keystone_rio_rxpool_alloc(void *arg,
								 unsigned q_num,
								 unsigned bufsize)
{
	struct keystone_rio_rx_chan_info *krx_chan = arg;
	struct keystone_rio_data *krio_priv = krx_chan->priv;
	struct dma_async_tx_descriptor *desc = NULL;
	struct keystone_rio_packet *p_info;
	struct keystone_rio_packet *next;

	spin_lock(&krx_chan->lock);
	p_info = list_first_entry_or_null(&krx_chan->free_list,
					  struct keystone_rio_packet,
					  node);
	if (p_info) {
		next = list_next_entry(p_info, node);
		list_splice_init(&next->node, &krx_chan->free_list);
	}
	spin_unlock(&krx_chan->lock);

	if (p_info == NULL)
		return NULL;

	desc = dmaengine_prep_slave_sg(krx_chan->dma_channel, p_info->sg,
				       p_info->sg_ents, DMA_DEV_TO_MEM,
				       DMA_HAS_EPIB | DMA_HAS_PSINFO);

	if (IS_ERR_OR_NULL(desc)) {
		u32 err = 0;
		dma_unmap_sg(krio_priv->dev, &p_info->sg[2],
			     1, DMA_FROM_DEVICE);
		kfree(p_info);
		err = PTR_ERR(desc);
		if (err != -ENOMEM) {
			dev_err(krio_priv->dev,
				"dma prep failed, error %d\n", err);
		}
		return NULL;
	}

	desc->callback_param = p_info;
	desc->callback = keystone_rio_rx_complete;
	p_info->cookie = desc->cookie;

	return desc;
}

static void keystone_rio_rx_notify(struct dma_chan *chan, void *arg)
{
	struct keystone_rio_data *krio_priv = arg;

	dmaengine_pause(chan);

	tasklet_schedule(&krio_priv->task);

	return;
}

static void keystone_rio_mp_inb_exit(int mbox, struct keystone_rio_data *krio_priv)
{
	struct keystone_rio_rx_chan_info *krx_chan;

	krx_chan = &(krio_priv->rx_channels[mbox]);

	if (!(krx_chan->dma_channel))
		return;

	dmaengine_pause(krx_chan->dma_channel);
	dma_release_channel(krx_chan->dma_channel);
	krx_chan->dma_channel = NULL;

	return;
}

static int keystone_rio_mp_inb_init(int mbox, struct keystone_rio_data *krio_priv)
{
	struct keystone_rio_rx_chan_info *krx_chan;
	struct dma_keystone_info config;
	dma_cap_mask_t mask;
	int err = -ENODEV;
	int i;

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);

	/* DMA RX channel */
	krx_chan = &(krio_priv->rx_channels[mbox]);
	krx_chan->priv = krio_priv;
	krx_chan->chan_num = mbox;
	krx_chan->dma_channel = dma_request_channel_by_name(mask, krx_chan->name);
	if (IS_ERR_OR_NULL(krx_chan->dma_channel))
		goto fail;

	memset(&config, 0, sizeof(config));
	config.direction	    = DMA_DEV_TO_MEM;
	config.scatterlist_size	    = KEYSTONE_RIO_SGLIST_SIZE;
	config.rxpool_allocator	    = keystone_rio_rxpool_alloc;
	config.rxpool_destructor    = keystone_rio_rxpool_free;
	config.rxpool_param	    = krx_chan;
	config.rxpool_thresh_enable = DMA_THRESH_NONE;

	for (i = 0; i < KEYSTONE_QUEUES_PER_CHAN &&
		     krx_chan->queue_depths[i] &&
		     krx_chan->buffer_sizes[i]; ++i) {
		config.rxpools[i].pool_depth  = krx_chan->queue_depths[i];
		config.rxpools[i].buffer_size = krx_chan->buffer_sizes[i];
		dev_dbg(krio_priv->dev, "rx_pool[%d] depth %d, size %d\n", i,
			config.rxpools[i].pool_depth,
			config.rxpools[i].buffer_size);
	}
	config.rxpool_count = i;

	err = dma_keystone_config(krx_chan->dma_channel, &config);
	if (err) {
		dev_err(krio_priv->dev,
			"Error configuring RX channel, err %d\n", err);
		goto fail;
	}

	tasklet_init(&krio_priv->task, keystone_rio_chan_work_handler,
		     (unsigned long) krio_priv);

	dma_set_notify(krx_chan->dma_channel,
		       keystone_rio_rx_notify,
		       krio_priv);

	krx_chan->flow_num = dma_get_rx_flow(krx_chan->dma_channel);
	krx_chan->queue_num = dma_get_rx_queue(krx_chan->dma_channel);

	spin_lock_init(&krx_chan->lock);
	INIT_LIST_HEAD(&krx_chan->packet_list);
	INIT_LIST_HEAD(&krx_chan->free_list);

	dev_info(krio_priv->dev,
		 "Opened rx channel: %p (mbox=%d, flow=%d, rx_q=%d, pkt_type=%d)\n",
		 krx_chan->dma_channel, mbox, krx_chan->flow_num,
		 krx_chan->queue_num, krx_chan->packet_type);

	return 0;

fail:
	if (krx_chan->dma_channel) {
		dma_release_channel(krx_chan->dma_channel);
		krx_chan->dma_channel = NULL;
	}
	return err;
}

static int keystone_rio_get_rxu_map(struct keystone_rio_data *krio_priv)
{
	int id;
	unsigned long bit_sz = 2 * 8 * sizeof(u32);

	id = find_first_zero_bit(&(krio_priv->rxu_map_bitmap[0]), bit_sz);
	if (id >= bit_sz)
		return -1;

	__set_bit(id, &(krio_priv->rxu_map_bitmap[0]));
	return id;
}

static void keystone_rio_free_rxu_map(int id, struct keystone_rio_data *krio_priv)
{
	clear_bit(id, &(krio_priv->rxu_map_bitmap[0]));
}

/**
 * keystone_rio_map_mbox - Map a mailbox to a given queue.
 * for both type 11 and type 9 packets.
 * @mbox: mailbox to map
 * @queue: associated queue number
 *
 * Returns %0 on success or %-ENOMEM on failure.
 */
static int keystone_rio_map_mbox(int mbox,
				 int queue, int flowid, int size,
				 struct keystone_rio_data *krio_priv)
{
	struct keystone_rio_mbox_info *rx_mbox = &krio_priv->rx_mbox[mbox];
	u32 mapping_entry_low = 0;
	u32 mapping_entry_high = 0;
	u32 mapping_entry_qid;
	u32 mapping_t9_reg[3];
	u32 pkt_type;
	int i;

	/* Retrieve the packet type */
	pkt_type = keystone_rio_mp_get_type(mbox, krio_priv);

	if (pkt_type == RIO_PACKET_TYPE_MESSAGE) {
		/* Map the multi-segment mailbox to the corresponding Rx queue for type 11 */
		mapping_entry_low = ((mbox & 0x1f) << 16)
			| (0x3f000000); /* Given mailbox, all letters, srcid = 0 */

		/* multi-segment messaging and promiscuous (don't care about src/dst id) */
		mapping_entry_high = KEYSTONE_RIO_MAP_FLAG_SEGMENT
			| KEYSTONE_RIO_MAP_FLAG_SRC_PROMISC
			| KEYSTONE_RIO_MAP_FLAG_DST_PROMISC;
	} else {
		/* Map the multi-segment mailbox for type 9 */
		mapping_t9_reg[0] = 0; /* accept all COS and srcid = 0 */
		mapping_t9_reg[1] = KEYSTONE_RIO_MAP_FLAG_SRC_PROMISC
			| KEYSTONE_RIO_MAP_FLAG_DST_PROMISC; /* promiscuous (don't care about src/dst id) */
		mapping_t9_reg[2] = (0xffff << 16)
			| (keystone_rio_mbox_to_strmid(mbox, krio_priv));
	}

	/* Set TT flag */
	if (size) {
		mapping_entry_high |= KEYSTONE_RIO_MAP_FLAG_TT_16;
		mapping_t9_reg[1]  |= KEYSTONE_RIO_MAP_FLAG_TT_16;
	}

	/* QMSS/PktDMA mapping (generic for both type 9 and 11) */
	mapping_entry_qid = (queue & 0x3fff) | (flowid << 16);

	i = keystone_rio_get_rxu_map(krio_priv);
	if (i < 0)
		return -ENOMEM;

	rx_mbox->rxu_map_id[0] = i;

	dev_dbg(krio_priv->dev,
		"Using RXU map %d @ 0x%08x: mbox = %d,"
		" flowid = %d, queue = %d pkt_type = %d\n",
		i, (u32)&(krio_priv->regs->rxu_map[i]), mbox,
		flowid, queue, pkt_type);

	if (pkt_type == RIO_PACKET_TYPE_MESSAGE) {
		/* Set packet type 11 rx mapping */
		__raw_writel(mapping_entry_low,
			     &(krio_priv->regs->rxu_map[i].ltr_mbox_src));
		__raw_writel(mapping_entry_high,
			     &(krio_priv->regs->rxu_map[i].dest_prom_seg));
	} else {
		/* Set packet type 9 rx mapping */
		__raw_writel(mapping_t9_reg[0],
			     &(krio_priv->regs->rxu_type9_map[i].cos_src));
		__raw_writel(mapping_t9_reg[1],
			     &(krio_priv->regs->rxu_type9_map[i].dest_prom));
		__raw_writel(mapping_t9_reg[2],
			     &(krio_priv->regs->rxu_type9_map[i].stream));
	}

	__raw_writel(mapping_entry_qid,
		     &(krio_priv->regs->rxu_map[i].flow_qid));

	if (pkt_type == RIO_PACKET_TYPE_MESSAGE) {
		/*
		 *  The RapidIO peripheral looks at the incoming RapidIO msgs
		 *  and if there is only one segment (the whole msg fits into one
		 *  RapidIO msg), the peripheral uses the single segment mapping
		 *  table. Therefore we need to map the single-segment mailbox too.
		 *  The same Rx CPPI Queue is used (as for the multi-segment
		 *  mailbox).
		 */
		mapping_entry_high &= ~KEYSTONE_RIO_MAP_FLAG_SEGMENT;

		i = keystone_rio_get_rxu_map(krio_priv);
		if (i < 0)
			return -ENOMEM;

		rx_mbox->rxu_map_id[1] = i;
		dev_dbg(krio_priv->dev,
			"Using RXU map %d @ 0x%08x: mbox = %d,"
			" flowid = %d, queue = %d pkt_type = %d\n",
			i, (u32)&(krio_priv->regs->rxu_map[i]), mbox,
			flowid, queue, pkt_type);

		__raw_writel(mapping_entry_low,
			     &(krio_priv->regs->rxu_map[i].ltr_mbox_src));
		__raw_writel(mapping_entry_high,
			     &(krio_priv->regs->rxu_map[i].dest_prom_seg));
		__raw_writel(mapping_entry_qid,
			     &(krio_priv->regs->rxu_map[i].flow_qid));
	}

	return 0;
}

/**
 * keystone_rio_open_inb_mbox - Initialize KeyStone inbound mailbox
 * @mport: Master port implementing the inbound message unit
 * @dev_id: Device specific pointer to pass on event
 * @mbox: Mailbox to open
 * @entries: Number of entries in the inbound mailbox ring
 *
 * Initializes buffer ring, request the inbound message interrupt,
 * and enables the inbound message unit. Returns %0 on success
 * and %-EINVAL, %-EBUSY or %-ENOMEM on failure.
 */
static int keystone_rio_open_inb_mbox(struct rio_mport *mport,
				      void *dev_id,
				      int mbox,
				      int entries)
{
	struct keystone_rio_data *krio_priv = mport->priv;
	struct keystone_rio_mbox_info *rx_mbox = &krio_priv->rx_mbox[mbox];
	struct keystone_rio_rx_chan_info *krx_chan = &krio_priv->rx_channels[mbox];
	int res;

	if (mbox >= KEYSTONE_RIO_MAX_MBOX)
		return -EINVAL;

	/* Check that number of entries is a power of two to ease ring management */
	if ((entries & (entries - 1)) != 0)
		return -EINVAL;

	/* Check if already initialized */
	if (rx_mbox->port)
		return -EBUSY;

	dev_dbg(krio_priv->dev,
		"open inb mbox: mport = 0x%x, dev_id = 0x%x,"
		" mbox = %d, entries = %d\n",
		(u32) mport, (u32) dev_id, mbox, entries);

	/* Initialization of RapidIO inbound MP */
	if (!(krx_chan->dma_channel)) {
		res = keystone_rio_mp_inb_init(mbox, krio_priv);
		if (res)
			return res;
	}

	rx_mbox->dev_id    = dev_id;
	rx_mbox->entries   = entries;
	rx_mbox->port      = mport;
	rx_mbox->id        = mbox;
	rx_mbox->running   = 1;

	/* Map the mailbox to queue/flow */
	res = keystone_rio_map_mbox(mbox,
				    krx_chan->queue_num,
				    krx_chan->flow_num,
				    mport->sys_size,
				    krio_priv);

	dmaengine_resume(krx_chan->dma_channel);

	return res;
}

/**
 * keystone_rio_close_inb_mbox - Shut down KeyStone inbound mailbox
 * @mport: Master port implementing the inbound message unit
 * @mbox: Mailbox to close
 *
 * Disables the outbound message unit, stop queues and free all resources
 */
static void keystone_rio_close_inb_mbox(struct rio_mport *mport, int mbox)
{
	struct keystone_rio_data *krio_priv = mport->priv;
	struct keystone_rio_mbox_info *rx_mbox = &krio_priv->rx_mbox[mbox];

	dev_info(krio_priv->dev, "close inb mbox: mport = 0x%x, mbox = %d\n",
		 (u32) mport, mbox);

	rx_mbox->running = 0;

	if (!(rx_mbox->port))
		return;

	rx_mbox->port = NULL;

	/* Release associated resource */
	keystone_rio_free_rxu_map(rx_mbox->rxu_map_id[0], krio_priv);
	keystone_rio_free_rxu_map(rx_mbox->rxu_map_id[1], krio_priv);

	keystone_rio_mp_inb_exit(mbox, krio_priv);

	return;
}

/**
 * keystone_rio_hw_add_inb_buffer - Add buffer to the KeyStone
 *   inbound message queue
 * @mport: Master port implementing the inbound message unit
 * @mbox: Inbound mailbox number
 * @buf: Buffer to add to inbound queue
 *
 * Adds the @buf buffer to the KeyStone inbound message queue. Returns
 * %0 on success or %-EINVAL on failure.
 */
static int keystone_rio_hw_add_inb_buffer(struct rio_mport *mport,
					  int mbox, void *buffer)
{
	struct keystone_rio_data *krio_priv = mport->priv;
	struct keystone_rio_rx_chan_info *krx_chan = &krio_priv->rx_channels[mbox];
	struct keystone_rio_packet *p_info;

	/* Allocate a primary receive queue entry */
	p_info = kzalloc(sizeof(*p_info), GFP_ATOMIC);
	if (!p_info) {
		dev_err(krio_priv->dev, "packet alloc failed\n");
		return -ENOMEM;
	}
	p_info->priv = krio_priv;
	p_info->buff = buffer;

	sg_init_table(p_info->sg, KEYSTONE_RIO_SGLIST_SIZE);
	sg_set_buf(&p_info->sg[0], p_info->epib, sizeof(p_info->epib));
	sg_set_buf(&p_info->sg[1], p_info->psdata, sizeof(p_info->psdata));
	sg_set_buf(&p_info->sg[2], p_info->buff, krx_chan->buffer_sizes[0]);

	p_info->sg_ents = 2 + dma_map_sg(krio_priv->dev, &p_info->sg[2],
					 1, DMA_FROM_DEVICE);

	if (p_info->sg_ents != 3) {
		dev_err(krio_priv->dev, "dma map failed\n");
		kfree(p_info);
		return -EINVAL;
	}

	spin_lock_bh(&krx_chan->lock);
	list_add_tail(&p_info->node, &krx_chan->free_list);
	spin_unlock_bh(&krx_chan->lock);

	dma_rxfree_refill(krx_chan->dma_channel);

	return 0;
}

/**
 * keystone_rio_hw_get_inb_message - Fetch inbound message from
 * the KeyStone message unit
 * @mport: Master port implementing the inbound message unit
 * @mbox: Inbound mailbox number
 *
 * Gets the next available inbound message from the inbound message queue.
 * A pointer to the message is returned on success or NULL on failure.
 */
static void *keystone_rio_hw_get_inb_message(struct rio_mport *mport, int mbox)
{
	struct keystone_rio_data *krio_priv = mport->priv;
	struct keystone_rio_rx_chan_info *krx_chan = &(krio_priv->rx_channels[mbox]);
	struct keystone_rio_packet *p_info = NULL;
	struct keystone_rio_packet *next;
	void *buff = NULL;
	u32   cc;

	spin_lock_bh(&krx_chan->lock);
	p_info = list_first_entry_or_null(&krx_chan->packet_list,
					  struct keystone_rio_packet,
					  node);
	if (p_info) {
		next = list_next_entry(p_info, node);
		list_splice_init(&next->node, &krx_chan->packet_list);
	}
	spin_unlock_bh(&krx_chan->lock);

	if (p_info == NULL)
		return NULL;

	buff = p_info->buff;

	dma_unmap_sg(krio_priv->dev, &p_info->sg[2], 1, DMA_FROM_DEVICE);

	/* Check CC from PS descriptor word 1 */
	cc = keystone_rio_mp_get_cc(p_info->psdata[1], krx_chan->packet_type);
	if (cc)
		dev_warn(krio_priv->dev,
			 "MP receive completion code is non zero (0x%x)\n", cc);

	if (p_info)
		kfree(p_info);

	return buff;
}

static void keystone_rio_mp_outb_exit(struct keystone_rio_data *krio_priv)
{
	if (!(krio_priv->tx_channel))
		return;

	dmaengine_pause(krio_priv->tx_channel);
	dma_release_channel(krio_priv->tx_channel);
	krio_priv->tx_channel = NULL;

	return;
}

static int keystone_rio_mp_outb_init(u8 port_id, struct keystone_rio_data *krio_priv)
{
	struct dma_keystone_info config;
	dma_cap_mask_t mask;
	int err = -ENODEV;
	const char *name;

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);

	/* DMA TX channel */
	name = krio_priv->tx_chan_name;
	krio_priv->tx_channel = dma_request_channel_by_name(mask, name);
	if (IS_ERR_OR_NULL(krio_priv->tx_channel)) {
		dev_err(krio_priv->dev,
			"Error requesting TX channel, err %d\n", err);
		goto fail;
	}

	memset(&config, 0, sizeof(config));
	config.direction	= DMA_MEM_TO_DEV;
	config.tx_queue_depth	= krio_priv->tx_queue_depth;
	err = dma_keystone_config(krio_priv->tx_channel, &config);
	if (err) {
		dev_err(krio_priv->dev,
			"Error configuring TX channel, err %d\n", err);
		goto fail;
	}

	dev_info(krio_priv->dev, "Opened tx channel: %p\n",
		 krio_priv->tx_channel);

	/*
	 * For the time being we are using only the first transmit queue
	 * In the future we may use queue 0 to 16 with multiple mbox support
	 */
	__raw_writel(port_id << 4, &(krio_priv->regs->tx_queue_sch_info[0]));

	return 0;

fail:
	if (krio_priv->tx_channel) {
		dma_release_channel(krio_priv->tx_channel);
		krio_priv->tx_channel = NULL;
	}

	return err;
}

/**
 * keystone_rio_open_outb_mbox - Initialize KeyStone outbound mailbox
 * @mport: Master port implementing the outbound message unit
 * @dev_id: Device specific pointer to pass on event
 * @mbox: Mailbox to open
 * @entries: Number of entries in the outbound mailbox ring
 *
 * Initializes buffer ring, request the outbound message interrupt,
 * and enables the outbound message unit. Returns %0 on success and
 * %-EINVAL, %-EBUSY or %-ENOMEM on failure.
 */
static int keystone_rio_open_outb_mbox(struct rio_mport *mport,
				       void *dev_id,
				       int mbox,
				       int entries)
{
	struct keystone_rio_data *krio_priv = mport->priv;
	struct keystone_rio_mbox_info *tx_mbox = &(krio_priv->tx_mbox[mbox]);
	int res;

	if (mbox >= KEYSTONE_RIO_MAX_MBOX)
		return -EINVAL;

	/* Check that number of entries is a power of two to ease ring management */
	if ((entries & (entries - 1)) != 0)
		return -EINVAL;

	/* Check if already initialized */
	if (tx_mbox->port)
		return -EBUSY;

	dev_dbg(krio_priv->dev,
		"open_outb_mbox: mport = 0x%x, dev_id = 0x%x, "
		"mbox = %d, entries = %d\n",
		(u32) mport, (u32) dev_id, mbox, entries);

	/* Initialization of RapidIO outbound MP */
	if (!(krio_priv->tx_channel)) {
		res = keystone_rio_mp_outb_init(mport->index, krio_priv);
		if (res)
			return res;
	}

	tx_mbox->dev_id  = dev_id;
	tx_mbox->entries = entries;
	tx_mbox->port    = mport;
	tx_mbox->id      = mbox;
	tx_mbox->slot    = 0;
	tx_mbox->running = 1;
	spin_lock_init(&tx_mbox->lock);

	return 0;
}

/**
 * keystone_rio_close_outb_mbox - Shut down KeyStone outbound mailbox
 * @mport: Master port implementing the outbound message unit
 * @mbox: Mailbox to close
 *
 * Disables the outbound message unit, stop queues and free all resources
 */
static void keystone_rio_close_outb_mbox(struct rio_mport *mport, int mbox)
{
	struct keystone_rio_data *krio_priv = mport->priv;
	struct keystone_rio_mbox_info *tx_mbox = &(krio_priv->tx_mbox[mbox]);

	if (mbox >= KEYSTONE_RIO_MAX_MBOX)
		return;

	dev_info(krio_priv->dev, "close_outb_mbox: mport = 0x%x, mbox = %d\n",
		(u32) mport, mbox);

	tx_mbox->port    = NULL;
	tx_mbox->running = 0;

	keystone_rio_mp_outb_exit(krio_priv);

	return;
}

static void keystone_rio_tx_complete(void *data)
{
	struct keystone_rio_packet *p_info  = data;
	struct keystone_rio_data *krio_priv = p_info->priv;
	int mbox_id			    = p_info->mbox;
	struct keystone_rio_mbox_info *mbox = &(krio_priv->tx_mbox[mbox_id]);
	struct rio_mport *port		    = mbox->port;
	void *dev_id			    = mbox->dev_id;

	dev_dbg(krio_priv->dev,	"tx_complete: psdata[0] = %08x, psdata[1] = %08x\n",
		p_info->psdata[0], p_info->psdata[1]);

	p_info->status = dma_async_is_tx_complete(krio_priv->tx_channel,
						  p_info->cookie, NULL, NULL);

	WARN_ON(p_info->status != DMA_COMPLETE && p_info->status != DMA_ERROR);

	dma_unmap_sg(krio_priv->dev, &p_info->sg[2], 1, DMA_TO_DEVICE);

	if (p_info->status == DMA_ERROR) {
		dev_warn(krio_priv->dev, "dma transfer failed\n");
		goto end;
	}

#if defined(CONFIG_RAPIDIO_CHMAN) || defined(CONFIG_RAPIDIO_CHMAN_MODULE)
	/*
	 * Free the temporary buffer
	 */
	if (p_info->buff)
		kfree(p_info->buff);
#endif

	if (mbox->running) {
		/*
		 * Client is in charge of freeing the associated buffers
		 * because we do not have explicit hardware ring but queues, we
		 * do not know where we are in the sw ring, let use fake slot.
		 * But the semantic hereafter is dangerous in case of re-order:
		 * bad buffer may be released...
		 */
		u32 slot;
		spin_lock(&mbox->lock);
		/* Move slot index to the next message to be sent */
		mbox->slot++;
		if (mbox->slot == mbox->entries)
			mbox->slot = 0;
		slot = mbox->slot;
		spin_unlock(&mbox->lock);

		port->outb_msg[mbox_id].mcback(port, dev_id,
					       mbox_id, slot);
	}
end:
	kfree(p_info);
}

/**
 * keystone_rio_hw_add_outb_message - Add a message to the KeyStone
 * outbound message queue
 * @mport: Master port with outbound message queue
 * @rdev: Target of outbound message
 * @mbox: Outbound mailbox
 * @buffer: Message to add to outbound queue
 * @len: Length of message
 *
 * Adds the @buffer message to the KeyStone outbound message queue. Returns
 * %0 on success or %-EBUSY on failure.
 */
static int keystone_rio_hw_add_outb_message(struct rio_mport *mport,
					    struct rio_dev *rdev,
					    int mbox,
					    void *buffer,
					    const size_t len)
{
	struct keystone_rio_data *krio_priv = mport->priv;
	struct dma_async_tx_descriptor *desc;
	struct keystone_rio_packet *p_info;
	u32 packet_type;
	int ret = 0;
	void *send_buffer = NULL;

	/*
	 * Ensure that the number of bytes being transmitted is a multiple
	 * of double-word. This is as per the specification.
	 */
	u32 plen = ((len + 7) & ~0x7);

#if defined(CONFIG_RAPIDIO_CHMAN) || defined(CONFIG_RAPIDIO_CHMAN_MODULE)
	/*
	 * Copy the outbound message in a temporary buffer. This is needed for
	 * RIO_CM.
	 */
	send_buffer = kmalloc(plen, GFP_ATOMIC | GFP_DMA);
	if (!send_buffer) {
		dev_warn(krio_priv->dev, "failed to alloc send buffer\n");
		return -ENOMEM;
	}

	memcpy(send_buffer, buffer, plen);
#endif

	p_info = kzalloc(sizeof(*p_info), GFP_ATOMIC);
	if (!p_info) {
		if (send_buffer)
			kfree(send_buffer);
		dev_warn(krio_priv->dev, "failed to alloc packet info\n");
		return -ENOMEM;
	}

	p_info->priv = krio_priv;

	/* Word 1: source id and dest id (common to packet 11 and packet 9) */
	p_info->psdata[0] = (rdev->destid & 0xffff) | (mport->host_deviceid << 16);

	/*
	 * Warning - Undocumented HW requirement:
	 *      For type9, packet type MUST be set to 30 in
	 *	keystone_hw_desc.desc_info[29:25] bits.
	 *
	 *	For type 11, setting packet type to 31 in
	 *	those bits is optional.
	 */
	if (keystone_rio_mp_get_type(mbox, krio_priv) == RIO_PACKET_TYPE_MESSAGE) {
		/* Packet 11 case (Message) */
		packet_type = 31;

		/* Word 2: ssize = 32 dword, 4 retries, letter = 0, mbox */
		p_info->psdata[1] = (KEYSTONE_RIO_MSG_SSIZE << 17) | (4 << 21)
			| (mbox & 0x3f);
	} else {
		/* Packet 9 case (Data Streaming) */
		packet_type = 30;

		/* Word 2: COS = 0, stream id */
		p_info->psdata[1] = keystone_rio_mbox_to_strmid(mbox, krio_priv) << 16;
	}

	if (rdev->net->hport->sys_size)
		p_info->psdata[1] |= KEYSTONE_RIO_DESC_FLAG_TT_16; /* tt */

	dev_dbg(krio_priv->dev,
		"packet type %d: psdata[0] = %08x, psdata[1] = %08x\n",
		keystone_rio_mp_get_type(mbox, krio_priv),
		p_info->psdata[0], p_info->psdata[1]);

	dev_dbg(krio_priv->dev, "buf(len=%d, plen=%d)\n", len, plen);

	p_info->mbox = mbox;
#if defined(CONFIG_RAPIDIO_CHMAN) || defined(CONFIG_RAPIDIO_CHMAN_MODULE)
	p_info->buff = send_buffer;
#else
	p_info->buff = buffer;
#endif

	sg_init_table(p_info->sg, KEYSTONE_RIO_SGLIST_SIZE);
	sg_set_buf(&p_info->sg[0], p_info->epib, sizeof(p_info->epib));
	sg_set_buf(&p_info->sg[1], p_info->psdata, sizeof(p_info->psdata));
	sg_set_buf(&p_info->sg[2], p_info->buff, plen);

	p_info->sg_ents = 2 + dma_map_sg(krio_priv->dev, &p_info->sg[2],
					 1, DMA_TO_DEVICE);

	if (p_info->sg_ents != KEYSTONE_RIO_SGLIST_SIZE) {
		dev_warn(krio_priv->dev, "failed to map transmit packet\n");
		ret = -ENXIO;
		goto error;
	}

	desc = dmaengine_prep_slave_sg(krio_priv->tx_channel, p_info->sg,
				       p_info->sg_ents, DMA_MEM_TO_DEV,
				       DMA_HAS_EPIB | DMA_HAS_PSINFO | DMA_HAS_PKTTYPE
				       | (packet_type  << DMA_PKTTYPE_SHIFT));

	if (IS_ERR_OR_NULL(desc)) {
		dma_unmap_sg(krio_priv->dev, &p_info->sg[2], 1, DMA_TO_DEVICE);
		dev_warn(krio_priv->dev, "failed to prep slave dma\n");
		ret = -ENOBUFS;
		goto error;
	}

	desc->callback_param = p_info;
	desc->callback = keystone_rio_tx_complete;
	p_info->cookie = dmaengine_submit(desc);

	if (dma_submit_error(p_info->cookie)) {
		dma_unmap_sg(krio_priv->dev, &p_info->sg[2], 1, DMA_TO_DEVICE);
		dev_warn(krio_priv->dev, "failed to submit packet for dma: %d\n",
			 p_info->cookie);
		ret = -EBUSY;
		goto error;
	}

	return ret;

error:
	kfree(p_info);
	if (send_buffer)
		kfree(send_buffer);

	return ret;
}

/*------------------------ Main Linux driver functions -----------------------*/

struct rio_mport *keystone_rio_register_mport(u32 port_id, u32 size,
					      struct keystone_rio_data *krio_priv)
{
	struct rio_ops   *ops;
	struct rio_mport *port;

	ops = kzalloc(sizeof(struct rio_ops), GFP_KERNEL);

	ops->lcread   = keystone_local_config_read;
	ops->lcwrite  = keystone_local_config_write;
	ops->cread    = keystone_rio_config_read;
	ops->cwrite   = keystone_rio_config_write;
	ops->dsend    = keystone_rio_dbell_send;
	ops->transfer = keystone_rio_dio_transfer;

	ops->open_outb_mbox   = keystone_rio_open_outb_mbox;
	ops->close_outb_mbox  = keystone_rio_close_outb_mbox;
	ops->open_inb_mbox    = keystone_rio_open_inb_mbox;
	ops->close_inb_mbox   = keystone_rio_close_inb_mbox;
	ops->add_outb_message = keystone_rio_hw_add_outb_message;
	ops->add_inb_buffer   = keystone_rio_hw_add_inb_buffer;
	ops->get_inb_message  = keystone_rio_hw_get_inb_message;

	port = kzalloc(sizeof(struct rio_mport), GFP_KERNEL);

	/*
	 * Set the sRIO port physical Id into the index field,
	 * the id field will be set by rio_register_mport() to
	 * the logical Id
	 */
	port->index = port_id;
	port->priv  = krio_priv;
	port->dev.parent = krio_priv->dev;
	INIT_LIST_HEAD(&port->dbells);

	/*
	 * Make a dummy per port region as ports are not
	 * really separated on KeyStone
	 */
	port->iores.start = krio_priv->board_rio_cfg.rio_regs_base +
		(u32)(krio_priv->serial_port_regs) +
		offsetof(struct keystone_rio_serial_port_regs,
			 sp[port_id].link_maint_req);

	port->iores.end = krio_priv->board_rio_cfg.rio_regs_base +
		(u32)(krio_priv->serial_port_regs) +
		offsetof(struct keystone_rio_serial_port_regs,
			 sp[port_id].ctl);

	port->iores.flags = IORESOURCE_MEM;

	rio_init_dbell_res(&port->riores[RIO_DOORBELL_RESOURCE], 0, 0xffff);
	rio_init_mbox_res(&port->riores[RIO_INB_MBOX_RESOURCE], 0, KEYSTONE_RIO_MAX_MBOX);
	rio_init_mbox_res(&port->riores[RIO_OUTB_MBOX_RESOURCE], 0, KEYSTONE_RIO_MAX_MBOX);

	sprintf(port->name, "RIO%d mport", port_id);

	port->ops      = ops;
	port->sys_size = size;
	port->phy_type = RIO_PHY_SERIAL;

	/*
	 * Hard coded here because in rio_disc_mport(), it is used in
	 * rio_enum_complete() before it is retrieved in
	 * rio_disc_peer() => rio_setup_device()
	 */
	port->phys_efptr = 0x100;

	/*
	 * Register the new mport
	 */
	rio_register_mport(port);

	krio_priv->mport[port_id] = port;

#ifdef CONFIG_RAPIDIO_ENUM_BASIC
	/*
	 * We may have a very late mport registration
	 * so perform explicitely the basic attachement (and
	 * eventually scanning here).
	 */
	rio_basic_attach();
#endif

	return port;
}

static int keystone_rio_get_mbox_defaults(int mbox,
					  struct device_node *node_rio,
					  struct keystone_rio_data *krio_priv)
{
	struct keystone_rio_rx_chan_info *krx_chan = &(krio_priv->rx_channels[mbox]);
	struct device_node *node;
	char node_name[24];

	snprintf(node_name, sizeof(node_name), "mbox-%d", mbox);
	node = of_get_child_by_name(node_rio, node_name);
	if (!node) {
		dev_err(krio_priv->dev, "could not find %s\n", node_name);
		of_node_put(node);
		return -ENODEV;
	}

	dev_dbg(krio_priv->dev, "using node \"%s\"\n", node_name);

	/* DMA rx chan config */
	if (of_property_read_string(node, "rx_channel", &krx_chan->name) < 0) {
		char name[8];
		dev_err(krio_priv->dev,
			"missing \"rx_channel\" parameter\n");
		snprintf(name, sizeof(name), "riorx-%d", mbox);
		krx_chan->name = name;
	}

	if (of_property_read_u32_array(node, "rx_queue_depth",
				       krx_chan->queue_depths,
				       KEYSTONE_QUEUES_PER_CHAN) < 0) {
		dev_err(krio_priv->dev,
			"missing \"rx_queue_depth\" parameter\n");
		krx_chan->queue_depths[0] = 128;
	}

	if (of_property_read_u32_array(node, "rx_buffer_size",
				       krx_chan->buffer_sizes,
				       KEYSTONE_QUEUES_PER_CHAN) < 0) {
		dev_err(krio_priv->dev,
			"missing \"rx_buffer_size\" parameter\n");
		krx_chan->buffer_sizes[0] = 4096;
	}

	/*
	 * If stream_id is defined, this mbox is mapped to the corresponding
	 * streamid and the channel is for type 9 packets.
	 */
	if (of_property_read_u32(node, "stream_id",
				 &krx_chan->stream_id) < 0) {
		krx_chan->packet_type = RIO_PACKET_TYPE_MESSAGE;
		krx_chan->stream_id = -1;
	} else {
		krx_chan->packet_type = RIO_PACKET_TYPE_STREAM;
	}
	of_node_put(node);

	return 0;
}

static void keystone_rio_get_controller_defaults(struct device_node *node,
						 struct keystone_rio_data *krio_priv)
{
	struct keystone_rio_board_controller_info *c = &krio_priv->board_rio_cfg;
	u32 temp[24];
	int i;
	int mbox;

	if (of_property_read_u32_array(node, "reg", &temp[0], 6)) {
		dev_err(krio_priv->dev, "Could not get default reg\n");
	} else {
		c->rio_regs_base = temp[0];
		c->rio_regs_size = temp[1];
		c->boot_cfg_regs_base = temp[2];
		c->boot_cfg_regs_size = temp[3];
		c->serdes_cfg_regs_base = temp[4];
		c->serdes_cfg_regs_size = temp[5];
	}

	if (of_property_read_u32 (node, "dev-id-size", &c->size))
		dev_err(krio_priv->dev, "Could not get default dev-id-size\n");

	if (of_property_read_u32 (node, "ports", &c->ports))
		dev_err(krio_priv->dev, "Could not get default ports\n");

	if (of_property_read_u32_array(node, "ports_remote",
				       c->ports_remote, KEYSTONE_RIO_MAX_PORT)) {
		/* Remote ports will be detected during port status */
		for (i = 0; i < KEYSTONE_RIO_MAX_PORT; i++)
			c->ports_remote[i] = -1;
	}

	/* SerDes config */
	if (!of_find_property(node, "keystone2-serdes", NULL)) {
		/* total number of serdes_config[] entries */
		c->serdes_config_num = 1;

		/* default serdes_config[] entry to be used */
		c->mode = 0;

		/*
		 * Mode 0: sRIO config 0: MPY = 5x, div rate = half,
		 * link rate = 3.125 Gbps, mode 1x
		 */
		c->serdes_config[0].cfg_cntl	      = 0x0c053860;
		c->serdes_config[0].serdes_cfg_pll    = 0x0229;
		c->serdes_config[0].prescalar_srv_clk = 0x001e;
		c->path_mode                          = 0x0000;

		for (i = 0; i < KEYSTONE_RIO_MAX_PORT; i++)
			c->serdes_config[0].rx_chan_config[i] = 0x00440495;

		for (i = 0; i < KEYSTONE_RIO_MAX_PORT; i++)
			c->serdes_config[0].tx_chan_config[i] = 0x00180795;
	} else {
		c->mode                               = 0;
		c->serdes_config_num                  = 1;
		c->keystone2_serdes                   = 1;
		c->serdes_config[0].prescalar_srv_clk = 0x001f;
		c->path_mode                          = 0x0004;

		if (of_property_read_u32(node, "baudrate", &c->serdes_baudrate)) {
			dev_err(krio_priv->dev,
				"Missing \"baudrate\" parameter. "
				"Setting 5Gbps as a default\n");
			c->serdes_baudrate = KEYSTONE_RIO_BAUD_5_000;
		}
	}

	/* SerDes 1sb, c1, c2, cm, ATT and VREG config */
	if (of_property_read_u32_array(node, "serdes_1sb", &temp[0], 4)) {
		for (i = 0; i < 4; i++)
			c->serdes_config[i].serdes_1sb = 0;
	} else {
		for (i = 0; i < 4; i++)
			c->serdes_config[i].serdes_1sb = temp[i];
	}

	if (of_property_read_u32_array(node, "serdes_c1", &temp[0], 4)) {
		if (c->serdes_baudrate == KEYSTONE_RIO_BAUD_3_125) {
			for (i = 0; i < 4; i++)
				c->serdes_config[i].serdes_c1 = 4;
		} else {
			for (i = 0; i < 4; i++)
				c->serdes_config[i].serdes_c1 = 6;
		}
	} else {
		for (i = 0; i < 4; i++)
			c->serdes_config[i].serdes_c1 = temp[i];
	}

	if (of_property_read_u32_array(node, "serdes_c2", &temp[0], 4)) {
		for (i = 0; i < 4; i++)
			c->serdes_config[i].serdes_c2 = 0;
	} else {
		for (i = 0; i < 4; i++)
			c->serdes_config[i].serdes_c2 = temp[i];
	}

	if (of_property_read_u32_array(node, "serdes_cm", &temp[0], 4)) {
		for (i = 0; i < 4; i++)
			c->serdes_config[i].serdes_cm = 0;
	} else {
		for (i = 0; i < 4; i++)
			c->serdes_config[i].serdes_cm = temp[i];
	}

	if (of_property_read_u32_array(node, "serdes_att", &temp[0], 4)) {
		for (i = 0; i < 4; i++)
			c->serdes_config[i].serdes_att = 12;
	} else {
		for (i = 0; i < 4; i++)
			c->serdes_config[i].serdes_att = temp[i];
	}

	if (of_property_read_u32_array(node, "serdes_vreg", &temp[0], 4)) {
		for (i = 0; i < 4; i++)
			c->serdes_config[i].serdes_vreg = 4;
	} else {
		for (i = 0; i < 4; i++)
			c->serdes_config[i].serdes_vreg = temp[i];
	}

	/* Path mode config (mapping of SerDes lanes to port widths) */
	if (of_property_read_u32(node, "path_mode", &c->path_mode)) {
		dev_err(krio_priv->dev,
			"Missing \"path_mode\" parameter\n");
	}

       /* Max possible ports configurations per path_mode */
	if ((c->path_mode == 0 &&
	     c->ports & ~KEYSTONE_MAX_PORTS_PATH_MODE_0) ||
	    (c->path_mode == 1 &&
	     c->ports & ~KEYSTONE_MAX_PORTS_PATH_MODE_1) ||
	    (c->path_mode == 2 &&
	     c->ports & ~KEYSTONE_MAX_PORTS_PATH_MODE_2) ||
	    (c->path_mode == 3 &&
	     c->ports & ~KEYSTONE_MAX_PORTS_PATH_MODE_3) ||
	    (c->path_mode == 4 &&
	     c->ports & ~KEYSTONE_MAX_PORTS_PATH_MODE_4)) {
		dev_err(krio_priv->dev,
			"\"path_mode\" and \"ports\" configuration mismatch\n");
	}

	/* Port register timeout */
	if (of_property_read_u32(node, "port-register-timeout",
				 &(c->port_register_timeout))) {
		c->port_register_timeout = 30;
	}

	/* LSUs */
	if (of_property_read_u32_array(node, "lsu", &temp[0], 2)) {
		krio_priv->lsu_dio   = 0;
		krio_priv->lsu_maint = 0;
	} else {
		krio_priv->lsu_dio   = (u8) temp[0];
		krio_priv->lsu_maint = (u8) temp[1];
	}
	dev_dbg(krio_priv->dev, "using LSU #%d for DIO, LSU #%d for maintenance packets\n",
		krio_priv->lsu_dio, krio_priv->lsu_maint);

	/* DMA tx chan config */
	if (of_property_read_string(node, "tx_channel",
				    &krio_priv->tx_chan_name) < 0){
		dev_err(krio_priv->dev,
			"missing \"tx_channel\" parameter\n");
		krio_priv->tx_chan_name = "riotx";
	}

	if (of_property_read_u32(node, "tx_queue_depth",
				 &krio_priv->tx_queue_depth) < 0) {
		dev_err(krio_priv->dev,
			"missing \"tx_queue_depth\" parameter\n");
		krio_priv->tx_queue_depth = 128;
	}

	/* Mailboxes configuration */
	if (of_property_read_u32(node, "num-mboxes",
				 &krio_priv->num_mboxes) < 0) {
		dev_err(krio_priv->dev,
			"missing \"num-mboxes\" parameter\n");
		krio_priv->num_mboxes = 1;
	}

	if (krio_priv->num_mboxes > KEYSTONE_RIO_MAX_MBOX) {
		dev_err(krio_priv->dev,
			"wrong \"num_mboxes\" parameter value %d, set to %d\n",
			krio_priv->num_mboxes, KEYSTONE_RIO_MAX_MBOX);
			krio_priv->num_mboxes = KEYSTONE_RIO_MAX_MBOX;
	}

	for (mbox = 0; mbox < krio_priv->num_mboxes; mbox++)
		(void) keystone_rio_get_mbox_defaults(mbox, node, krio_priv);

	/* Interrupt config */
	c->rio_irq = irq_of_parse_and_map(node, 0);
	if (c->rio_irq < 0)
		dev_err(krio_priv->dev, "missing \"rio_irq\" parameter\n");

	c->lsu_irq = irq_of_parse_and_map(node, 1);
	if (c->lsu_irq < 0)
		dev_err(krio_priv->dev, "missing \"lsu_irq\" parameter\n");

	/* Packet forwarding */
	if (of_property_read_u32_array(node, "pkt-forward", &temp[0], 24)) {
		c->pkt_forwarding = 0;
	} else {
		c->pkt_forwarding = 1;
		for (i = 0; i < 8; i++) {
			c->routing_config[i].dev_id_low  = (u16) temp[(i * 3)];
			c->routing_config[i].dev_id_high = (u16) temp[(i * 3) + 1];
			c->routing_config[i].port        = (u8)  temp[(i * 3) + 2];
		}
	}
}

static int keystone_rio_port_chk(struct keystone_rio_data *krio_priv)
{
	u32 ports = krio_priv->ports_registering;
	u32 size  = krio_priv->board_rio_cfg.size;
	struct rio_mport *mport;

	while (ports) {
		int status;
		u32 port = __ffs(ports);
		ports &= ~(1 << port);

		/* Wait lanes to be OK */
		if (keystone_rio_lanes_init(port, 0, krio_priv))
			return -1;

		/* Check port status */
		status = keystone_rio_port_status(port, krio_priv);
		if (status == 0) {
			unsigned long flags;

			/*
			 * The link has been established from an hw standpoint
			 * so do not try to check the port again.
			 * Only mport registration may fail now.
			 */
			spin_lock_irqsave(&krio_priv->port_chk_lock, flags);
			krio_priv->ports |= (1 << port);
			krio_priv->ports_registering &= ~(1 << port);
			spin_unlock_irqrestore(&krio_priv->port_chk_lock,
					       flags);

			/* Register mport only if this is initial port check */
			if (!krio_priv->mport[port]) {
				mport = keystone_rio_register_mport(port, size,
								    krio_priv);
				if (!mport) {
					dev_err(krio_priv->dev,
						"failed to register mport %d\n",
						port);
					return -1;
				}
				dev_info(krio_priv->dev,
					 "port RIO%d host_deviceid %d "
					 "registered\n",
					 port, mport->host_deviceid);
			} else {
				dev_info(krio_priv->dev,
					 "port RIO%d host_deviceid %d ready\n",
					 port,
					 krio_priv->mport[port]->host_deviceid);
			}

			/*
			 * update routing after discovery/enumeration
			 * with new dev id
			 */
			if (krio_priv->board_rio_cfg.pkt_forwarding)
				keystone_rio_port_set_routing(port, krio_priv);

		} else {
			if (status == -EINVAL)
				return -1;

			dev_dbg(krio_priv->dev, "port %d not ready\n", port);
		}
	}

	return krio_priv->ports_registering;
}

static void keystone_rio_port_chk_task(struct work_struct *work)
{
	struct keystone_rio_data *krio_priv = container_of(
		to_delayed_work(work), struct keystone_rio_data, port_chk_task);
	int res;

	res = keystone_rio_port_chk(krio_priv);
	if (res) {
		unsigned long flags;

		/* If port check failed schedule next check (if any) */
		spin_lock_irqsave(&krio_priv->port_chk_lock, flags);
		if (krio_priv->port_chk_cnt-- > 1) {
			spin_unlock_irqrestore(&krio_priv->port_chk_lock,
					       flags);

			schedule_delayed_work(&krio_priv->port_chk_task,
					      KEYSTONE_RIO_REGISTER_DELAY);
		} else {
			spin_unlock_irqrestore(&krio_priv->port_chk_lock,
					       flags);

			dev_info(krio_priv->dev,
				 "RIO port register timeout, "
				 "port mask 0x%x not ready",
				 krio_priv->ports_registering);
		}
	}
}

/*
 * Sysfs management
 */
static ssize_t keystone_rio_ports_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t count)
{
	struct keystone_rio_data *krio_priv = (struct keystone_rio_data *)
		dev->platform_data;
	long unsigned int ports;
	unsigned long flags;

	if (kstrtoul(buf, 0, &ports))
		return -EINVAL;

	if (ports > ((1 << KEYSTONE_RIO_MAX_PORT) - 1))
		return -EINVAL;

	/*
	 * Only the ports defined in DTS can be rescanned because SerDes
	 * initialization is not restarted here, only link status check.
	 */
	ports &=  krio_priv->board_rio_cfg.ports;

	spin_lock_irqsave(&krio_priv->port_chk_lock, flags);
	krio_priv->ports_registering = (ports & ~krio_priv->ports);
	spin_unlock_irqrestore(&krio_priv->port_chk_lock, flags);

	if (krio_priv->ports_registering) {
		unsigned long flags;

		dev_dbg(dev, "initializing link for port mask 0x%x\n",
			krio_priv->ports_registering);

		spin_lock_irqsave(&krio_priv->port_chk_lock, flags);
		krio_priv->port_chk_cnt =
			krio_priv->board_rio_cfg.port_register_timeout /
			(KEYSTONE_RIO_REGISTER_DELAY / HZ);
		spin_unlock_irqrestore(&krio_priv->port_chk_lock, flags);

		schedule_delayed_work(&krio_priv->port_chk_task, 0);
	}

	return count;
}

static ssize_t keystone_rio_ports_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct keystone_rio_data *krio_priv = (struct keystone_rio_data *)
		dev->platform_data;

	if (krio_priv == NULL)
		return -EINVAL;

	return scnprintf(buf, PAGE_SIZE, "0x%x\n", krio_priv->ports);
}

static DEVICE_ATTR(ports,
		   S_IRUGO | S_IWUSR,
		   keystone_rio_ports_show,
		   keystone_rio_ports_store);

static void keystone_rio_sysfs_remove(struct device *dev)
{
	device_remove_file(dev, &dev_attr_ports);
}

static int keystone_rio_sysfs_create(struct device *dev)
{
	int res = 0;

	res = device_create_file(dev, &dev_attr_ports);
	if (res)
		dev_err(dev, "unable create sysfs ports file\n");

	return res;
}

/*
 * Platform configuration setup
 */
static int keystone_rio_setup_controller(struct platform_device *pdev,
					 struct keystone_rio_data *krio_priv)
{
	u32 ports;
	u32 p;
	u32 mode;
	u32 baud;
	u32 path_mode;
	u32 size = 0;
	int res = 0;
	struct rio_mport *mport;
	char str[8];
	unsigned long flags;

	size      = krio_priv->board_rio_cfg.size;
	ports     = krio_priv->board_rio_cfg.ports;
	mode      = krio_priv->board_rio_cfg.mode;
	baud      = krio_priv->board_rio_cfg.serdes_baudrate;
	path_mode = krio_priv->board_rio_cfg.path_mode;

	dev_dbg(&pdev->dev, "size = %d, ports = 0x%x, mode = %d, baud = %d, path_mode = %d\n",
		size, ports, mode, baud, path_mode);

	if (mode >= krio_priv->board_rio_cfg.serdes_config_num) {
		dev_warn(&pdev->dev,
			 "invalid port mode %d, forcing it to %d\n", mode, 0);
		mode = 0;
	}

	if (baud > KEYSTONE_RIO_BAUD_5_000) {
		baud = KEYSTONE_RIO_BAUD_5_000;
		dev_warn(&pdev->dev,
			 "invalid baud rate, forcing it to 5Gbps\n");
	}

	switch (baud) {
	case KEYSTONE_RIO_BAUD_1_250:
		snprintf(str, sizeof(str), "1.25");
		break;
	case KEYSTONE_RIO_BAUD_2_500:
		snprintf(str, sizeof(str), "2.50");
		break;
	case KEYSTONE_RIO_BAUD_3_125:
		snprintf(str, sizeof(str), "3.125");
		break;
	case KEYSTONE_RIO_BAUD_5_000:
		snprintf(str, sizeof(str), "5.00");
		break;
	default:
		res = -EINVAL;
		goto out;
	}

	dev_info(&pdev->dev,
		 "initializing %s Gbps interface with port configuration %d\n",
		 str, path_mode);

	/* Hardware set up of the controller */
	res = keystone_rio_hw_init(mode, baud, krio_priv);
	if (res < 0) {
		dev_err(&pdev->dev,
			"initialization of SRIO hardware failed\n");
		return res;
	}

	/* Initialize port write interface */
	res = keystone_rio_port_write_init(krio_priv);
	if (res)
		return res;

	/* Disable all ports */
	for (p = 0; p < KEYSTONE_RIO_MAX_PORT; p++)
		keystone_rio_port_disable(p, krio_priv);

	/* Initialize interrupts */
	res = keystone_rio_interrupt_setup(krio_priv);
	if (res)
		return res;

	/* Start the controller */
	keystone_rio_start(krio_priv);

	/* Use and check ports status (but only the requested ones) */
	spin_lock_irqsave(&krio_priv->port_chk_lock, flags);
	krio_priv->ports_registering = 0;
	spin_unlock_irqrestore(&krio_priv->port_chk_lock, flags);

	while (ports) {
		int status;
		u32 port = __ffs(ports);
		ports &= ~(1 << port);

		res = keystone_rio_port_init(port, path_mode, krio_priv);
		if (res < 0) {
			dev_err(&pdev->dev,
				"initialization of port %d failed\n", port);
			return res;
		}

		/* Start the port */
		keystone_rio_port_activate(port, krio_priv);

		/* Start lanes and wait them to be OK */
		if (keystone_rio_lanes_init(port, 1, krio_priv))
			goto port_not_ready;

		/*
		 * Check the port status here before calling the generic RapidIO
		 * layer. Port status check is done in rio_mport_is_active() as
		 * well but we need to do it our way first due to some delays in
		 * hw initialization.
		 */
		status = keystone_rio_port_status(port, krio_priv);
		if (status == 0) {
			/* Register this port  */
			mport = keystone_rio_register_mport(port, size, krio_priv);
			if (!mport)
				goto out;

			/*
			 * Update routing after discovery/enumeration
			 * with new dev id
			 */
			if (krio_priv->board_rio_cfg.pkt_forwarding)
				keystone_rio_port_set_routing(port, krio_priv);

			spin_lock_irqsave(&krio_priv->port_chk_lock, flags);
			krio_priv->ports |= (1 << port);
			spin_unlock_irqrestore(&krio_priv->port_chk_lock,
					       flags);

			dev_info(&pdev->dev,
				 "port RIO%d host_deviceid %d registered\n",
				 port, mport->host_deviceid);

			continue;
		}
		if (status == -EINVAL)
			return status;
port_not_ready:
		spin_lock_irqsave(&krio_priv->port_chk_lock, flags);
		krio_priv->ports_registering |= (1 << port);
		spin_unlock_irqrestore(&krio_priv->port_chk_lock, flags);

		dev_warn(&pdev->dev, "port %d not ready\n", port);
	}

	if (krio_priv->ports_registering) {
		unsigned long flags;

		spin_lock_irqsave(&krio_priv->port_chk_lock, flags);
		krio_priv->port_chk_cnt =
			krio_priv->board_rio_cfg.port_register_timeout /
			(KEYSTONE_RIO_REGISTER_DELAY / HZ);
		spin_unlock_irqrestore(&krio_priv->port_chk_lock, flags);

		schedule_delayed_work(&krio_priv->port_chk_task,
				      KEYSTONE_RIO_REGISTER_DELAY);
	}
out:
	return res;
}

static int keystone_rio_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct keystone_rio_data *krio_priv;
	int res = 0;
	void __iomem *regs;

	if (!node) {
		dev_err(&pdev->dev, "could not find device info\n");
		return -EINVAL;
	}

	krio_priv = kzalloc(sizeof(struct keystone_rio_data), GFP_KERNEL);
	if (!krio_priv) {
		dev_err(&pdev->dev, "memory allocation failed\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, krio_priv);
	krio_priv->dev = &(pdev->dev);

	/* Get default config from device tree */
	keystone_rio_get_controller_defaults(node, krio_priv);

	/* sRIO main driver (global ressources) */
	mutex_init(&krio_priv->lsu_lock);
	init_completion(&krio_priv->lsu_completion);
	spin_lock_init(&krio_priv->port_chk_lock);
	INIT_DELAYED_WORK(&krio_priv->port_chk_task,
			  keystone_rio_port_chk_task);

	regs = ioremap(krio_priv->board_rio_cfg.boot_cfg_regs_base,
		       krio_priv->board_rio_cfg.boot_cfg_regs_size);

	krio_priv->jtagid_reg = regs + 0x0018;

	if (!K2_SERDES(krio_priv))
		krio_priv->serdes_sts_reg = regs + 0x154;

	regs = ioremap(krio_priv->board_rio_cfg.serdes_cfg_regs_base,
		       krio_priv->board_rio_cfg.serdes_cfg_regs_size);
	krio_priv->serdes_regs = regs;

	regs = ioremap(krio_priv->board_rio_cfg.rio_regs_base,
		       krio_priv->board_rio_cfg.rio_regs_size);
	krio_priv->regs		     = regs;
	krio_priv->car_csr_regs	     = regs + 0xb000;
	krio_priv->serial_port_regs  = regs + 0xb100;
	krio_priv->err_mgmt_regs     = regs + 0xc000;
	krio_priv->phy_regs	     = regs + 0x1b000;
	krio_priv->transport_regs    = regs + 0x1b300;
	krio_priv->pkt_buf_regs	     = regs + 0x1b600;
	krio_priv->evt_mgmt_regs     = regs + 0x1b900;
	krio_priv->port_write_regs   = regs + 0x1ba00;
	krio_priv->link_regs	     = regs + 0x1bd00;
	krio_priv->fabric_regs	     = regs + 0x1be00;
	krio_priv->car_csr_regs_base = (u32) regs + 0xb000;

	INIT_DELAYED_WORK(&krio_priv->pe_work, keystone_rio_pe_dpc);

	/* Enable srio clock */
	krio_priv->clk = clk_get(&pdev->dev, "clk_srio");
	if (IS_ERR(krio_priv->clk)) {
		dev_err(&pdev->dev, "Unable to get Keystone SRIO clock\n");
		return -EBUSY;
	} else {
		/* Workaround for K1 RIO clocks */
		clk_prepare_enable(krio_priv->clk);
		ndelay(100);
		clk_disable_unprepare(krio_priv->clk);
		ndelay(100);
		clk_prepare_enable(krio_priv->clk);
	}

	pdev->dev.platform_data = (void *) krio_priv;

	keystone_rio_sysfs_create(&pdev->dev);

	dev_info(&pdev->dev, "KeyStone RapidIO driver %s\n", DRIVER_VER);

	/* Setup the sRIO controller */
	res = keystone_rio_setup_controller(pdev, krio_priv);
	if (res < 0) {
		clk_disable_unprepare(krio_priv->clk);
		clk_put(krio_priv->clk);
		kfree(krio_priv);
		return res;
	}

	return 0;
}

static void keystone_rio_shutdown(struct platform_device *pdev)
{
	struct keystone_rio_data *krio_priv = platform_get_drvdata(pdev);
	int i;
	u32 lanes = keystone_rio_get_lane_config(krio_priv->board_rio_cfg.ports,
						 krio_priv->board_rio_cfg.path_mode);

	keystone_rio_interrupt_release(krio_priv);
	keystone_rio_mp_outb_exit(krio_priv);

	for (i = 0; i < KEYSTONE_RIO_MAX_MBOX; i++)
		keystone_rio_mp_inb_exit(i, krio_priv);

	keystone_rio_stop(krio_priv);

	/* wait current DMA transfers to finish */
	mdelay(10);

	/* Disable blocks */
	__raw_writel(0, &krio_priv->regs->gbl_en);
	for (i = 0; i <= KEYSTONE_RIO_BLK_NUM; i++)
		__raw_writel(0, &(krio_priv->regs->blk[i].enable));

	/* Shutdown associated SerDes */
	if (!K2_SERDES(krio_priv))
		k2_rio_serdes_shutdown(krio_priv->serdes_regs, lanes);

	if (krio_priv->clk) {
		clk_disable_unprepare(krio_priv->clk);
		clk_put(krio_priv->clk);
	}

	platform_set_drvdata(pdev, NULL);

	kfree(krio_priv);
}

static int __exit keystone_rio_remove(struct platform_device *pdev)
{
	keystone_rio_shutdown(pdev);

	keystone_rio_sysfs_remove(&pdev->dev);

	return 0;
}

static struct of_device_id of_match[] = {
	{ .compatible = "ti,keystone-rapidio", },
	{},
};

MODULE_DEVICE_TABLE(of, keystone_hwqueue_of_match);

static struct platform_driver keystone_rio_driver  = {
	.driver = {
		.name	        = "keystone-rapidio",
		.owner	        = THIS_MODULE,
		.of_match_table	= of_match,
	},
	.probe	= keystone_rio_probe,
	.remove = __exit_p(keystone_rio_remove),
	.shutdown = keystone_rio_shutdown,
};

static int __init keystone_rio_module_init(void)
{
	return platform_driver_register(&keystone_rio_driver);
}

static void __exit keystone_rio_module_exit(void)
{
	platform_driver_unregister(&keystone_rio_driver);
}

module_init(keystone_rio_module_init);
module_exit(keystone_rio_module_exit);

MODULE_AUTHOR("Aurelien Jacquiot");
MODULE_DESCRIPTION("TI KeyStone RapidIO device driver");
MODULE_LICENSE("GPL");
