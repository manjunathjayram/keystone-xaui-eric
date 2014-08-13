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
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/clk.h>
#include <linux/timer.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/hardirq.h>
#include <linux/kfifo.h>
#include <linux/rio.h>
#include <linux/rio_drv.h>
#include <linux/keystone-dma.h>

#include "keystone_rio.h"

#define DRIVER_VER "v1.3"

static int keystone_rio_setup_controller(struct keystone_rio_data *krio_priv);
static void keystone_rio_shutdown_controller(struct keystone_rio_data *krio_priv);
static void keystone_rio_serdes_lane_disable(u32 lane, struct keystone_rio_data *krio_priv);
static void dbell_handler(struct keystone_rio_data *krio_priv);
static void keystone_rio_port_write_handler(struct keystone_rio_data *krio_priv);

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
	u32 pending_err_int = __raw_readl(&(krio_priv->regs->lsu_int[0].status));

#ifdef CONFIG_RAPIDIO_DMA_ENGINE
	u32 pending_lsu_int;

	/* Call DMA interrupt handling */
	while((pending_lsu_int = __raw_readl(&(krio_priv->regs->lsu_int[1].status))
	       & KEYSTONE_RIO_ICSR_LSU1_COMPLETE_MASK)) {
		u32 lsu = __ffs(pending_lsu_int);
		keystone_rio_dma_interrupt_handler(krio_priv, lsu, 0);
		__raw_writel(1 << lsu, &(krio_priv->regs->lsu_int[1].clear));
	}

	/* In case of LSU completion with error */
	if (pending_err_int & KEYSTONE_RIO_ICSR_LSU0_ERROR_MASK) {
		keystone_rio_dma_interrupt_handler(krio_priv, 0, 1);
		__raw_writel(pending_err_int & KEYSTONE_RIO_ICSR_LSU0_ERROR_MASK,
			     &(krio_priv->regs->lsu_int[0].clear));
	}
#endif

	return IRQ_HANDLED;
}

static void special_interrupt_handler(int ics, struct keystone_rio_data *krio_priv)
{
	u32 lanes;

	/* Acknowledge the interrupt */
	__raw_writel(BIT(ics), &krio_priv->regs->err_rst_evnt_int_clear);

	dev_dbg(krio_priv->dev, "ics = %d\n", ics);

	switch(ics) {
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

		/* Disable SerDes lanes asap to generate a loss of link */
		lanes = krio_priv->board_rio_cfg.lanes;
 		while (lanes) {
 			u32 lane = __ffs(lanes);
 			lanes &= ~(1 << lane);
			keystone_rio_serdes_lane_disable(lane, krio_priv);
		}

		/* Schedule SRIO peripheral reinitialization */
		schedule_work(&krio_priv->reset_work);
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

	for (i = 0; i <= 32; i+= 4) {
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
static void keystone_rio_interrupt_setup(struct keystone_rio_data *krio_priv)
{
	int res;
	u8 lsu;

	/* Clear all pending interrupts */
	__raw_writel(0x0000ffff,
		     &(krio_priv->regs->doorbell_int[0].clear));
	__raw_writel(0x0000ffff,
		     &(krio_priv->regs->doorbell_int[1].clear));
	__raw_writel(0x0000ffff,
		     &(krio_priv->regs->doorbell_int[2].clear));
	__raw_writel(0x0000ffff,
		     &(krio_priv->regs->doorbell_int[3].clear));
	__raw_writel(0x00010f07,
		     &(krio_priv->regs->err_rst_evnt_int_clear));

	for (lsu = krio_priv->lsu_start; lsu <= krio_priv->lsu_end; lsu++) {
		__raw_writel(0xffffffff,
			     &(krio_priv->regs->lsu_int[lsu].clear));
	}

	/* LSU interrupts are routed to RIO interrupt dest 1 (LSU) */
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

	/* Error, reset and special event interrupts are routed to RIO interrupt dest 0 (Rx/Tx) */
	keystone_rio_interrupt_map(&(krio_priv->regs->err_rst_evnt_int_route[0]),
				   0x00000111, KEYSTONE_GEN_RIO_INT);
	keystone_rio_interrupt_map(&(krio_priv->regs->err_rst_evnt_int_route[1]),
				   0x00001111, KEYSTONE_GEN_RIO_INT);
	keystone_rio_interrupt_map(&(krio_priv->regs->err_rst_evnt_int_route[2]),
				   0x00000001, KEYSTONE_GEN_RIO_INT);

	/* The doorbell interrupts routing table is for the 16 general purpose interrupts */
	__raw_writel(0x1, &(krio_priv->regs->interrupt_ctl));

	/* Do not use pacing */
	__raw_writel(0x0000ffff, &krio_priv->regs->intdst_rate_disable);

	/* Attach interrupt handlers */
	res = request_irq(krio_priv->board_rio_cfg.rio_irq,
			  rio_interrupt_handler,
			  0,
			  "sRIO",
			  krio_priv);
	if (res)
		dev_err(krio_priv->dev,
			"Failed to request RIO irq (%d)\n",
			krio_priv->board_rio_cfg.rio_irq);

	res = request_irq(krio_priv->board_rio_cfg.lsu_irq,
			  lsu_interrupt_handler,
			  0,
			  "sRIO LSU",
			  krio_priv);
	if (res)
		dev_err(krio_priv->dev,
			"Failed to request LSU irq (%d)\n",
			krio_priv->board_rio_cfg.lsu_irq);
}

static void keystone_rio_interrupt_release(struct keystone_rio_data *krio_priv)
{
	free_irq(krio_priv->board_rio_cfg.rio_irq, krio_priv);
	free_irq(krio_priv->board_rio_cfg.lsu_irq, krio_priv);
}

/*---------------------------- LSU management -------------------------------*/

u8 keystone_rio_lsu_alloc(struct keystone_rio_data *krio_priv)
{
	u8 lsu_id = krio_priv->lsu_free++;
	if (krio_priv->lsu_free > krio_priv->lsu_end)
		krio_priv->lsu_free = krio_priv->lsu_start;

	return lsu_id;
}

static u32 keystone_rio_lsu_get_cc(u32 lsu_id, u8 ltid, u8 *lcb,
				   struct keystone_rio_data *krio_priv)
{
	u32 idx;
	u32 shift;
	u32 value;
	u32 cc;
	/*  LSU shadow register status mapping */
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

/*
 * Initiate a LSU transfer
 */
int keystone_rio_lsu_start_transfer(int lsu,
				    int port_id,
				    u16 dest_id,
				    dma_addr_t src_addr,
				    u64 tgt_addr,
				    u32 size_bytes,
				    int size,
				    u32 packet_type,
				    u32 *lsu_context,
				    int interrupt_req,
				    struct keystone_rio_data *krio_priv)
{
	u32 count;
	u32 status = 0;
	u32 res = 0;
	u8  lcb;
	u8  ltid;

	if (size_bytes > KEYSTONE_RIO_MAX_DIO_PKT_SIZE)
		return -EINVAL;

	size_bytes &= (KEYSTONE_RIO_MAX_DIO_PKT_SIZE - 1);

	/* if interrupt mode, do not spin */
	if (interrupt_req)
		count = KEYSTONE_RIO_TIMEOUT_CNT;
	else
		count = 0;

	/* Check if there is space in the LSU shadow reg and that it is free */
	while(1)
        {
		/* LSU Reg 6 - get full and busy bit */
		status = __raw_readl(&(krio_priv->regs->lsu_reg[lsu].busy_full));
		if (((status & KEYSTONE_RIO_LSU_FULL_MASK) == 0x0)
		    && ((status & KEYSTONE_RIO_LSU_BUSY_MASK) == 0x0))
			break;
		count++;
		if (count >= KEYSTONE_RIO_TIMEOUT_CNT) {
			dev_err(krio_priv->dev,
                                "no LSU%d shadow register available, status = 0x%x\n",
				lsu, status);
			res = -EBUSY;
			goto out;
		}
		ndelay(KEYSTONE_RIO_TIMEOUT_NSEC);
        }

	/* Get LCB and LTID, LSU reg 6 is already read */
	lcb  = (status >> 4) & 0x1;
	ltid = status & 0xf;
	*lsu_context = status;

	/* LSU Reg 0 - MSB of destination */
	__raw_writel((u32) (tgt_addr >> 32), &(krio_priv->regs->lsu_reg[lsu].addr_msb));

	/* LSU Reg 1 - LSB of destination */
	__raw_writel((u32) (tgt_addr), &(krio_priv->regs->lsu_reg[lsu].addr_lsb_cfg_ofs));

	/* LSU Reg 2 - source address */
	__raw_writel(src_addr, &(krio_priv->regs->lsu_reg[lsu].phys_addr));

	/* LSU Reg 3 - byte count */
	__raw_writel(size_bytes, &(krio_priv->regs->lsu_reg[lsu].dbell_val_byte_cnt));

	/*
	 * LSU Reg 4 -
	 * out port ID = rio.port
         * priority = LSU_PRIO
	 * Xambs = 0
	 * ID size = 8 or 16 bit
	 * Dest ID specified as arg
	 * interrupt request
	 */
	__raw_writel(((port_id << 8)
		      | (KEYSTONE_RIO_LSU_PRIO << 4)
		      | (size ? (1 << 10) : 0)
		      | ((u32) dest_id << 16)
		      | (interrupt_req & 0x1)),
		     &(krio_priv->regs->lsu_reg[lsu].destid));

	/*
	 * LSU Reg 5 -
	 * doorbell info = packet_type[16-31], hop count = packet_type [8-15]
	 * FType = packet_type[4-7], TType = packet_type[0-3]
	 * Writing this register will initiate the transfer
	 */
	__raw_writel(packet_type, &(krio_priv->regs->lsu_reg[lsu].dbell_info_fttype));

out:
	return res;
}

/*
 * Complete a LSU transfer
 */
int keystone_rio_lsu_complete_transfer(int lsu, u32 lsu_context,
				       struct keystone_rio_data *krio_priv)
{
	u32 status = 0;
	u32 res = 0;
	u8  lcb = (lsu_context >> 4) & 0x1;
	u8  ltid = (lsu_context) & 0xf;
	u8  n_lcb;

	/* Retrieve our completion code */
	status = keystone_rio_lsu_get_cc(lsu, ltid, &n_lcb, krio_priv);
	if (n_lcb != lcb)
		return -EAGAIN;

	dev_dbg(krio_priv->dev, "LSU%d status 0x%x\n", lsu, status);

	switch (status & KEYSTONE_RIO_LSU_CC_MASK) {
	case KEYSTONE_RIO_LSU_CC_TIMEOUT:
		res = -ETIMEDOUT;
		/* LSU Reg 6 - flush the transaction */
		__raw_writel(1, &(krio_priv->regs->lsu_reg[lsu].busy_full));
		break;
	case KEYSTONE_RIO_LSU_CC_XOFF:
	case KEYSTONE_RIO_LSU_CC_ERROR:
	case KEYSTONE_RIO_LSU_CC_INVALID:
	case KEYSTONE_RIO_LSU_CC_DMA:
		res = -EIO;
		/* LSU Reg 6 - flush the transaction */
		__raw_writel(1, &(krio_priv->regs->lsu_reg[lsu].busy_full));
		break;
	case KEYSTONE_RIO_LSU_CC_RETRY:
		res = -EAGAIN;
		break;
	case KEYSTONE_RIO_LSU_CC_CANCELED:
		res = -EBUSY;
		break;
	default:
		break;
	}
	return res;
}

#ifdef CONFIG_RAPIDIO_DMA_ENGINE
/*
 * DMA callback
 */
static void keystone_rio_lsu_dma_callback(void *data)
{
	struct completion *lsu_completion = (struct completion *) data;
	complete(lsu_completion);
}

static int keystone_rio_lsu_dma_allocate_channel(struct rio_mport *mport)
{
	struct keystone_rio_data *krio_priv = mport->priv;
	struct dma_chan *dchan;

	dchan = rio_request_mport_dma(mport);
	if (!dchan) {
		dev_err(krio_priv->dev, "%s cannot find DMA channel for port %d\n",
			__FUNCTION__, mport->index);
		return -ENODEV;
	}

	dev_dbg(krio_priv->dev, "get channel 0x%x for port %d\n",
		(u32) dchan, mport->index);

	krio_priv->dma_chan[mport->index] = dchan;

	return 0;
}

static void keystone_rio_lsu_dma_free_channel(struct rio_mport *mport)
{
	struct keystone_rio_data *krio_priv = mport->priv;
	struct dma_chan *dchan = krio_priv->dma_chan[mport->index];

	if (dchan) {
		struct keystone_rio_dma_chan *chan = from_dma_chan(dchan);

		/* Remove from global list */
		list_del_init(&chan->node);

		rio_release_dma(dchan);
		krio_priv->dma_chan[mport->index] = NULL;
	}
}

/*
 * Perform a raw LSU transfer using DMA engine (used for doorbells)
 */
static int keystone_rio_lsu_raw_async_transfer(struct rio_mport *mport,
					       struct keystone_rio_dma_packet_raw *pkt,
					       enum dma_transfer_direction dir)
{
	struct keystone_rio_data *krio_priv = mport->priv;
	struct dma_chan *dma_chan;
	struct completion lsu_completion;
	enum dma_status	status;
	dma_cookie_t cookie;
	struct dma_async_tx_descriptor *tx = NULL;
	int res = 0;
	int ret;

	/* We reserved a channel for raw transfers */
	dma_chan = krio_priv->dma_chan[mport->index];

	res = dmaengine_device_control(dma_chan,
				       DMA_KEYSTONE_RIO_PREP_RAW_PACKET,
				       (u32) pkt);
	if ((res) || (pkt->tx == NULL)) {
		dev_err(krio_priv->dev, "%s cannot prepare DMA (raw) transfer\n", __FUNCTION__);
		return -EIO;
	}

	tx = pkt->tx;

	init_completion(&lsu_completion);

	tx->callback = keystone_rio_lsu_dma_callback;
	tx->callback_param = &lsu_completion;

	cookie = dmaengine_submit(tx);

	if (dma_submit_error(cookie)) {
		dev_warn(krio_priv->dev, "failed to submit LSU transfer for dma: %d\n",
			 cookie);
		return -EBUSY;
	}

	/* Wait for transfer to complete */
	ret = wait_for_completion_interruptible_timeout(&lsu_completion,
							msecs_to_jiffies(KEYSTONE_RIO_TIMEOUT_MSEC));
	status = dma_async_is_tx_complete(dma_chan, cookie, NULL, NULL);
	if (status != DMA_SUCCESS)
		res = -EIO;
	if (ret == 0)
		res = -ETIMEDOUT;

	return res;
}
#endif /* CONFIG_RAPIDIO_DMA_ENGINE */

/*------------------------------ Doorbell management --------------------------*/

static inline int dbell_get(u32* pending)
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

	for (i = 0; i < KEYSTONE_RIO_MAX_PORT; i ++) {
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

	if (!found) {
		dev_dbg(krio_priv->dev, "DBELL: received spurious doorbell %d\n", dbell_num);
	}
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
#ifdef CONFIG_RAPIDIO_DMA_ENGINE

	struct keystone_rio_dma_packet_raw pkt;

	/* Transform doorbell number into info field */
	u16 info   = (num & 0xf) | (((num >> 4) & 0x3) << 5);

	/* Create a doorbell raw packet */
	pkt.port_id     = mport->index;
	pkt.dest_id     = dest_id;
	pkt.rio_addr    = 0;
	pkt.rio_addr_u  = 0;
	pkt.buff_addr   = 0;
	pkt.size        = 0;
	pkt.sys_size    = mport->sys_size;
	pkt.packet_type = ((info & 0xffff) << 16) | KEYSTONE_RIO_PACKET_TYPE_DBELL;
	pkt.tx          = NULL;

	return keystone_rio_lsu_raw_async_transfer(mport, &pkt, DMA_MEM_TO_DEV);

#else /* CONFIG_RAPIDIO_DMA_ENGINE */

	return -ENOTSUPP;

#endif /* CONFIG_RAPIDIO_DMA_ENGINE */
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
					     u16 packet_type,
					     struct keystone_rio_data *krio_priv)
{
	int res;
	u32 lsu_context;
	u32 count = 0;
	u32 type = ((hopcount & 0xff) << 8) | (packet_type & 0xff);

	mutex_lock(&krio_priv->lsu_lock_maint);

	res = keystone_rio_lsu_start_transfer(krio_priv->lsu_maint, port_id, dest_id,
					      buff, offset, buff_len, size, type,
					      &lsu_context, 0, krio_priv);
	if (res) {
		mutex_unlock(&krio_priv->lsu_lock_maint);
		dev_err(krio_priv->dev, "Maintenance packet transfer error\n");
		return res;
	}

	while(1) {
		res = keystone_rio_lsu_complete_transfer(krio_priv->lsu_maint,
							 lsu_context, krio_priv);
		if (res != -EAGAIN)
			break;

		count++;
		if (count >= KEYSTONE_RIO_TIMEOUT_CNT) {
			res = -EIO;
			break;
		}
		ndelay(KEYSTONE_RIO_TIMEOUT_NSEC);
	}

	mutex_unlock(&krio_priv->lsu_lock_maint);

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
		*val = *((u8*)tbuf);
		break;
	case 2:
		*val = ntohs(*((u16*)tbuf));
		break;
	default:
		*val = ntohl(*((u32*)tbuf));
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
		     lane ++) {
			lanes |= (1 << lane);
		}
	}
	return (int) lanes;
}

#define reg_rmw(addr, value, mask) \
	__raw_writel(((__raw_readl(addr) & (~(mask))) | (value)), (addr))

static void k1_rio_serdes_init(struct keystone_rio_data *krio_priv)
{
	struct keystone_serdes_config *serdes_config =
		&krio_priv->board_rio_cfg.serdes_config;
	u32 port;
	u32 val;

	/* K1 SerDes main configuration */
	__raw_writel(serdes_config->serdes_cfg_pll,
			 &krio_priv->serdes_regs->pll);

	/* Per-port SerDes configuration */
	for (port = 0; port < KEYSTONE_RIO_MAX_PORT; port++) {
		__raw_writel(serdes_config->rx_chan_config[port],
			     &krio_priv->serdes_regs->channel[port].rx);
		__raw_writel(serdes_config->tx_chan_config[port],
			     &krio_priv->serdes_regs->channel[port].tx);
	}

	/* Check for RIO SerDes PLL lock */
	do {
		val = __raw_readl(krio_priv->serdes_sts_reg);
	} while (!(val & 0x1));
}

static void k2_rio_serdes_init_3g(u32 lanes, struct keystone_rio_data *krio_priv)
{
	void __iomem *regs = (void __iomem *) krio_priv->k2_serdes_regs;

	/* Uses Half Rate configuration */
	reg_rmw(regs + 0x000, 0x00000000, 0xff000000);
	reg_rmw(regs + 0x014, 0x00008282, 0x0000ffff);
	reg_rmw(regs + 0x060, 0x00132c48, 0x00ffffff);
	reg_rmw(regs + 0x064, 0x00c3c700, 0x00ffff00);
	reg_rmw(regs + 0x078, 0x0000c000, 0x0000ff00);

	if (IS_SERDES_LANE_USED(lanes, 0)) {
		dev_dbg(krio_priv->dev, "setting lane 0 SerDes to 3GHz\n");
		reg_rmw(regs + 0x204, 0x78000080, 0xff0000ff);
		reg_rmw(regs + 0x208, 0x00000024, 0x000000ff);
		reg_rmw(regs + 0x20c, 0x02000000, 0xff000000);
		reg_rmw(regs + 0x210, 0x1b000000, 0xff000000);
		reg_rmw(regs + 0x214, 0x00006e7c, 0x0000ffff);
		reg_rmw(regs + 0x218, 0x758000e4, 0xffff00ff);
		reg_rmw(regs + 0x22c, 0x00100800, 0x00ffff00);
		reg_rmw(regs + 0x280, 0x00700070, 0x00ff00ff);
		reg_rmw(regs + 0x284, 0x1d0f0085, 0xffff00ff);
		reg_rmw(regs + 0x28c, 0x00003b00, 0x0000ff00);
	}

	if (IS_SERDES_LANE_USED(lanes, 1)) {
		dev_dbg(krio_priv->dev, "setting lane 1 SerDes to 3GHz\n");
		reg_rmw(regs + 0x404, 0x78000080, 0xff0000ff);
		reg_rmw(regs + 0x408, 0x00000024, 0x000000ff);
		reg_rmw(regs + 0x40c, 0x02000000, 0xff000000);
		reg_rmw(regs + 0x410, 0x1b000000, 0xff000000);
		reg_rmw(regs + 0x414, 0x00006e7c, 0x0000ffff);
		reg_rmw(regs + 0x418, 0x758000e4, 0xffff0000);
		reg_rmw(regs + 0x42c, 0x00100800, 0x00ffff00);
		reg_rmw(regs + 0x480, 0x00700070, 0x00ff00ff);
		reg_rmw(regs + 0x484, 0x1d0f0085, 0xffff00ff);
		reg_rmw(regs + 0x48c, 0x00003b00, 0x0000ff00);
	}

	if (IS_SERDES_LANE_USED(lanes, 2)) {
		dev_dbg(krio_priv->dev, "setting lane 2 SerDes to 3GHz\n");
		reg_rmw(regs + 0x604, 0x78000080, 0xff0000ff);
		reg_rmw(regs + 0x608, 0x00000024, 0x000000ff);
		reg_rmw(regs + 0x60c, 0x02000000, 0xff000000);
		reg_rmw(regs + 0x610, 0x1b000000, 0xff000000);
		reg_rmw(regs + 0x614, 0x00006e7c, 0x0000ffff);
		reg_rmw(regs + 0x618, 0x758000e4, 0xffff00ff);
		reg_rmw(regs + 0x62c, 0x00100800, 0x00ffff00);
		reg_rmw(regs + 0x680, 0x00700070, 0x00ff00ff);
		reg_rmw(regs + 0x684, 0x1d0f0085, 0xffff00ff);
		reg_rmw(regs + 0x68c, 0x00003b00, 0x0000ff00);
	}

	if (IS_SERDES_LANE_USED(lanes, 3)) {
		dev_dbg(krio_priv->dev, "setting lane 3 SerDes to 3GHz\n");
		reg_rmw(regs + 0x804, 0x78000080, 0xff0000ff);
		reg_rmw(regs + 0x808, 0x00000024, 0x000000ff);
		reg_rmw(regs + 0x80c, 0x02000000, 0xff000000);
		reg_rmw(regs + 0x810, 0x1b000000, 0xff000000);
		reg_rmw(regs + 0x814, 0x00006e7c, 0x0000ffff);
		reg_rmw(regs + 0x818, 0x758000e4, 0xffff00ff);
		reg_rmw(regs + 0x82c, 0x00100800, 0x00ffff00);
		reg_rmw(regs + 0x880, 0x00700070, 0x00ff00ff);
		reg_rmw(regs + 0x884, 0x1d0f0085, 0xffff00ff);
		reg_rmw(regs + 0x88c, 0x00003b00, 0x0000ff00);
	}

	reg_rmw(regs + 0xa00, 0x00000800, 0x0000ff00);
	reg_rmw(regs + 0xa08, 0x37720000, 0xffff0000);
	reg_rmw(regs + 0xa30, 0x00777700, 0x00ffff00);
	reg_rmw(regs + 0xa84, 0x00000600, 0x0000ff00);
	reg_rmw(regs + 0xa94, 0x10000000, 0xff000000);
	reg_rmw(regs + 0xaa0, 0x81000000, 0xff000000);
	reg_rmw(regs + 0xabc, 0xff000000, 0xff000000);
	reg_rmw(regs + 0xac0, 0x0000008b, 0x000000ff);

	reg_rmw(regs + 0x000, 0x00000003, 0x000000ff);
	reg_rmw(regs + 0xa00, 0x0000005f, 0x000000ff);
}

static void k2_rio_serdes_init_5g(u32 lanes, struct keystone_rio_data *krio_priv)
{
	void __iomem *regs = (void __iomem *) krio_priv->k2_serdes_regs;

	/* Uses Full Rate configuration by default */
	reg_rmw(regs + 0x000, 0x00000000, 0xff000000);
	reg_rmw(regs + 0x014, 0x00008282, 0x0000ffff);
	reg_rmw(regs + 0x060, 0x00142438, 0x00ffffff);
	reg_rmw(regs + 0x064, 0x00c3c700, 0x00ffff00);
	reg_rmw(regs + 0x078, 0x0000c000, 0x0000ff00);

	if (IS_SERDES_LANE_USED(lanes, 0)) {
		dev_dbg(krio_priv->dev, "setting lane 0 SerDes to 5GHz\n");
		reg_rmw(regs + 0x204, 0x78000080, 0xff0000ff);
		reg_rmw(regs + 0x208, 0x00000026, 0x000000ff);
		reg_rmw(regs + 0x20c, 0x02000000, 0xff000000);
		reg_rmw(regs + 0x214, 0x00006f38, 0x0000ffff);
		reg_rmw(regs + 0x218, 0x758000e4, 0xffff00ff);
		reg_rmw(regs + 0x22c, 0x00200800, 0x00ffff00);
		reg_rmw(regs + 0x280, 0x00860086, 0x00ff00ff);
		reg_rmw(regs + 0x284, 0x1d0f0085, 0xffff00ff);
		reg_rmw(regs + 0x28c, 0x00002c00, 0x0000ff00);
	}

	if (IS_SERDES_LANE_USED(lanes, 1)) {
		dev_dbg(krio_priv->dev, "setting lane 1 SerDes to 5GHz\n");
		reg_rmw(regs + 0x404, 0x78000080, 0xff0000ff);
		reg_rmw(regs + 0x408, 0x00000026, 0x000000ff);
		reg_rmw(regs + 0x40c, 0x02000000, 0xff000000);
		reg_rmw(regs + 0x414, 0x00006f38, 0x0000ffff);
		reg_rmw(regs + 0x418, 0x758000e4, 0xffff00ff);
		reg_rmw(regs + 0x42c, 0x00200800, 0x00ffff00);
		reg_rmw(regs + 0x480, 0x00860086, 0x00ff00ff);
		reg_rmw(regs + 0x484, 0x1d0f0085, 0xffff00ff);
		reg_rmw(regs + 0x48c, 0x00002c00, 0x0000ff00);
	}

	if (IS_SERDES_LANE_USED(lanes, 2)) {
		dev_dbg(krio_priv->dev, "setting lane 2 SerDes to 5GHz\n");
		reg_rmw(regs + 0x604, 0x78000080, 0xff0000ff);
		reg_rmw(regs + 0x608, 0x00000026, 0x000000ff);
		reg_rmw(regs + 0x60c, 0x02000000, 0xff000000);
		reg_rmw(regs + 0x614, 0x00006f38, 0x0000ffff);
		reg_rmw(regs + 0x618, 0x758000e4, 0xffff00ff);
		reg_rmw(regs + 0x62c, 0x00200800, 0x00ffff00);
		reg_rmw(regs + 0x680, 0x00860086, 0x00ff00ff);
		reg_rmw(regs + 0x684, 0x1d0f0085, 0xffff00ff);
		reg_rmw(regs + 0x68c, 0x00002c00, 0x0000ff00);
	}

	if (IS_SERDES_LANE_USED(lanes, 3)) {
		dev_dbg(krio_priv->dev, "setting lane 3 SerDes to 5GHz\n");
		reg_rmw(regs + 0x804, 0x78000080, 0xff0000ff);
		reg_rmw(regs + 0x808, 0x00000026, 0x000000ff);
		reg_rmw(regs + 0x80c, 0x02000000, 0xff000000);
		reg_rmw(regs + 0x814, 0x00006f38, 0x0000ffff);
		reg_rmw(regs + 0x818, 0x758000e4, 0xffff00ff);
		reg_rmw(regs + 0x82c, 0x00200800, 0x00ffff00);
		reg_rmw(regs + 0x880, 0x00860086, 0x00ff00ff);
		reg_rmw(regs + 0x884, 0x1d0f0085, 0xffff00ff);
		reg_rmw(regs + 0x88c, 0x00002c00, 0x0000ff00);
	}

	reg_rmw(regs + 0xa00, 0x00008000, 0x0000ff00);
	reg_rmw(regs + 0xa08, 0x38d20000, 0xffff0000);
	reg_rmw(regs + 0xa30, 0x008d8d00, 0x00ffff00);
	reg_rmw(regs + 0xa84, 0x00000600, 0x0000ff00);
	reg_rmw(regs + 0xa94, 0x10000000, 0xff000000);
	reg_rmw(regs + 0xaa0, 0x81000000, 0xff000000);
	reg_rmw(regs + 0xabc, 0xff000000, 0xff000000);
	reg_rmw(regs + 0xac0, 0x0000008b, 0x000000ff);
	reg_rmw(regs + 0x000, 0x00000003, 0x000000ff);
	reg_rmw(regs + 0xa00, 0x0000005f, 0x000000ff);

	reg_rmw(regs + 0xa48, 0x00fd8c00, 0x00ffff00);
	reg_rmw(regs + 0xa54, 0x002fec72, 0x00ffffff);
	reg_rmw(regs + 0xa58, 0x00f92100, 0xffffff00);
	reg_rmw(regs + 0xa5c, 0x00040060, 0xffffffff);
	reg_rmw(regs + 0xa60, 0x00008000, 0xffffffff);
	reg_rmw(regs + 0xa64, 0x0c581220, 0xffffffff);
	reg_rmw(regs + 0xa68, 0xe13b0602, 0xffffffff);
	reg_rmw(regs + 0xa6c, 0xb8074cc1, 0xffffffff);
	reg_rmw(regs + 0xa70, 0x3f02e989, 0xffffffff);
	reg_rmw(regs + 0xa74, 0x00000001, 0x000000ff);
	reg_rmw(regs + 0xb20, 0x00370000, 0x00ff0000);
	reg_rmw(regs + 0xb1c, 0x37000000, 0xff000000);
	reg_rmw(regs + 0xb20, 0x0000005d, 0x000000ff);
}

static void k2_rio_serdes_lane_enable(u32 lane, u32 rate, struct keystone_rio_data *krio_priv)
{
	struct keystone2_srio_serdes_regs *serdes_regs = krio_priv->k2_serdes_regs;
	u32 val;

	/* Bring this lane out of reset by clearing override bit 29 */
	val = __raw_readl(&serdes_regs->lane[lane].__lane_rsvd0[8]);
	val &= ~BIT(29);
	__raw_writel(val, &serdes_regs->lane[lane].__lane_rsvd0[8]);

	/* Set Lane Control Rate */
	switch (rate) {
	case KEYSTONE_RIO_FULL_RATE:
		__raw_writel(0xF0C0F0F0, &serdes_regs->wiz_lane[lane].ctl_sts);
		break;
	case KEYSTONE_RIO_HALF_RATE:
		__raw_writel(0xF4C0F4F0, &serdes_regs->wiz_lane[lane].ctl_sts);
		break;
	case KEYSTONE_RIO_QUARTER_RATE:
		__raw_writel(0xF8C0F8F0, &serdes_regs->wiz_lane[lane].ctl_sts);
		break;
	default:
		return;
	}
}

static int k2_rio_serdes_config(u32 lanes, u32 baud, struct keystone_rio_data *krio_priv)
{
	struct keystone2_srio_serdes_regs *serdes_regs =
		krio_priv->k2_serdes_regs;
	u32 rate;
	u32 val;

	/* Disable pll before configuring the SerDes registers */
	__raw_writel(0x00000000, &serdes_regs->wiz_pll_ctrl);

	switch (baud) {
	case KEYSTONE_RIO_BAUD_1_250:
		rate = KEYSTONE_RIO_QUARTER_RATE;
		k2_rio_serdes_init_5g(lanes, krio_priv);
		break;
	case KEYSTONE_RIO_BAUD_2_500:
		rate = KEYSTONE_RIO_HALF_RATE;
		k2_rio_serdes_init_5g(lanes, krio_priv);
		break;
	case KEYSTONE_RIO_BAUD_5_000:
		rate = KEYSTONE_RIO_FULL_RATE;
		k2_rio_serdes_init_5g(lanes, krio_priv);
		break;
	case KEYSTONE_RIO_BAUD_3_125:
		rate = KEYSTONE_RIO_HALF_RATE;
		k2_rio_serdes_init_3g(lanes, krio_priv);
		break;
	default:
		dev_warn(krio_priv->dev, "unsupported baud rate %d\n", baud);
		return -EINVAL;
	}

	/* Enable serdes for requested lanes */
	while(lanes) {
		u32 lane = __ffs(lanes);
		lanes &= ~(1 << lane);
		k2_rio_serdes_lane_enable(lane, rate, krio_priv);
	}

	/* Enable pll via the pll_ctrl */
	__raw_writel(0xe0000000, &serdes_regs->wiz_pll_ctrl);

	/* Wait until CMU_OK bit is set */
	do {
		val = __raw_readl(&serdes_regs->comlane_1F8);
	} while (!(val & BIT(16)));

	return 0;
}

static int k2_rio_serdes_wait_lock(struct keystone_rio_data *krio_priv, u32 lanes)
{
	u32 val;
	unsigned long timeout;
	struct keystone2_srio_serdes_regs *regs = krio_priv->k2_serdes_regs;
	u32 val_mask;

	val_mask = lanes | (lanes << 8);

	/* Wait for the SerDes PLL lock */
	timeout = jiffies + msecs_to_jiffies(K2_PLL_LOCK_TIMEOUT);
	while (1) {
		/* read PLL_CTRL */
		val = __raw_readl(&regs->wiz_pll_ctrl);
		if ((val & val_mask) == val_mask)
			break;
		if (time_after(jiffies, timeout))
			return -1;
		udelay(10);
	}
	return 0;
}

static int keystone_rio_serdes_config(u32 baud, struct keystone_rio_data *krio_priv)
{
	int res = 0;

	if (!K2_SERDES(krio_priv)) {
		k1_rio_serdes_init(krio_priv);
	} else {
		/* K2 SerDes main configuration */
		res = k2_rio_serdes_config(krio_priv->board_rio_cfg.lanes,
					   baud, krio_priv);
	}

	return res;
}

static void keystone_rio_serdes_lane_disable(u32 lane, struct keystone_rio_data *krio_priv)
{
	if (!K2_SERDES(krio_priv)) {
		__raw_writel(0, &krio_priv->serdes_regs->channel[lane].rx);
		__raw_writel(0, &krio_priv->serdes_regs->channel[lane].tx);
	} else {
		u32 val;
		val = __raw_readl(&krio_priv->k2_serdes_regs->wiz_lane[lane].ctl_sts);
		val &= ~(BIT(29) | BIT(30) | BIT(13) | BIT(14));
		__raw_writel(val, &krio_priv->k2_serdes_regs->wiz_lane[lane].ctl_sts);
	}
}

static void keystone_rio_serdes_shutdown(struct keystone_rio_data *krio_priv, u32 lanes)
{
	while(lanes) {
		u32 lane = __ffs(lanes);
		lanes &= ~(1 << lane);

		/* Disable SerDes for this lane */
		keystone_rio_serdes_lane_disable(lane, krio_priv);
	}

	if (!K2_SERDES(krio_priv)) {
		__raw_writel(0, &krio_priv->serdes_regs->pll);
	} else {
		u32 val;

		__raw_writel(0, &krio_priv->k2_serdes_regs->wiz_pll_ctrl);

		/* Reset CMU PLL for all lanes */
		val = __raw_readl(&krio_priv->k2_serdes_regs->__cmu0_rsvd1[1]);
		val |= BIT(28);
		__raw_writel(val, &krio_priv->k2_serdes_regs->__cmu0_rsvd1[1]);
	}
}

/**
 * keystone_rio_hw_init - Configure a RapidIO controller
 * @baud: serdes baudrate
 *
 * Returns %0 on success or %-EINVAL or %-EIO on failure.
 */
static int keystone_rio_hw_init(u32 baud, struct keystone_rio_data *krio_priv)
{
	u32 val;
	u32 block;
	u32 lsu_mask = 0;
	u32 port;
	int res = 0;
	int i;

	/* Reset blocks */
	__raw_writel(0, &krio_priv->regs->gbl_en);
	for (block = 0; block < KEYSTONE_RIO_BLK_NUM; block++) {
		__raw_writel(0, &(krio_priv->regs->blk[block].enable));
		while(__raw_readl(&(krio_priv->regs->blk[block].status)) & 0x1);
	}

	ndelay(1000);

	/* Set sRIO out of reset */
	__raw_writel((KEYSTONE_RIO_PER_RESTORE | KEYSTONE_RIO_PER_FREE),
		     &krio_priv->regs->pcr);

	/* Clear BOOT_COMPLETE bit (allowing write) */
	__raw_writel(0x00000000, &krio_priv->regs->per_set_cntl);

	/* Enable blocks */
	__raw_writel(1, &krio_priv->regs->gbl_en);
	for (block = 0; block < KEYSTONE_RIO_BLK_NUM; block++)
		__raw_writel(1, &(krio_priv->regs->blk[block].enable));

	/* Set control register 1 configuration */
	__raw_writel(0x00000000, &krio_priv->regs->per_set_cntl1);

	/* Set control register */
	__raw_writel(krio_priv->board_rio_cfg.serdes_config.cfg_cntl,
		     &krio_priv->regs->per_set_cntl);

	/* SerDes main configuration */
	res = keystone_rio_serdes_config(baud, krio_priv);
	if (res < 0) {
		dev_err(krio_priv->dev, "initialization of SerDes failed\n");
		return res;
	}

	/* Set prescalar for ip_clk */
	__raw_writel(krio_priv->board_rio_cfg.serdes_config.prescalar_srv_clk,
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

	/* Clear high bits of local config space base addr */
	__raw_writel(0x00000000, &krio_priv->car_csr_regs->local_cfg_hbar);

	/* Set local config space base addr */
	__raw_writel(0x00520000, &krio_priv->car_csr_regs->local_cfg_bar);

	/* Enable HOST BIT(31) & MASTER_ENABLE BIT(30) bits */
	__raw_writel(0xc0000000, &krio_priv->serial_port_regs->sp_gen_ctl);

	/* Set link timeout value */
	__raw_writel(0x000FFF00,
		     &krio_priv->serial_port_regs->sp_link_timeout_ctl);

	/* Set response timeout value */
	__raw_writel(0x000FFF00,
		     &krio_priv->serial_port_regs->sp_rsp_timeout_ctl);

	/* Allows SELF_RESET and PWDN_PORT resets to clear stcky reg bits */
	__raw_writel(0x00000001, &krio_priv->link_regs->reg_rst_ctl);

	/* Clear all errors */
	__raw_writel(0x00000000, &krio_priv->err_mgmt_regs->err_det);

	/* Enable all error detection */
	__raw_writel(0x00000000, &krio_priv->err_mgmt_regs->err_en);

	/* Set err det block header */
	val = (((KEYSTONE_RIO_ERR_HDR_NEXT_BLK_PTR & 0xffff) << 16) |
	       KEYSTONE_RIO_ERR_EXT_FEAT_ID);
	__raw_writel(val, &krio_priv->err_mgmt_regs->err_report_blk_hdr);

	/* Clear msb of err captured addr reg */
	__raw_writel(0x00000000, &krio_priv->err_mgmt_regs->h_addr_capt);

	/* Clear lsb of err captured addr reg */
	__raw_writel(0x00000000, &krio_priv->err_mgmt_regs->addr_capt);

	/* Clear err captured source and dest DevID reg */
	__raw_writel(0x00000000, &krio_priv->err_mgmt_regs->id_capt);

	/* Clear err captured packet info */
	__raw_writel(0x00000000, &krio_priv->err_mgmt_regs->ctrl_capt);

	/* Set per port information */
	for (port = 0; port < KEYSTONE_RIO_MAX_PORT; port++) {
		__raw_writel(0x41004141, &krio_priv->phy_regs->phy_sp[port].__rsvd[3]);

		/* Set the baud rate of the port information */
		val = __raw_readl(&krio_priv->serial_port_regs->sp[port].ctl2);
		val |= BIT(24 - (baud << 1));
		__raw_writel(val, &krio_priv->serial_port_regs->sp[port].ctl2);
	}

	/* Disable LSU to perform LSU configuration */
	__raw_writel(0, &(krio_priv->regs->blk[KEYSTONE_RIO_BLK_LSU_ID].enable));
	while(__raw_readl(&(krio_priv->regs->blk[KEYSTONE_RIO_BLK_LSU_ID].status)) & 0x1);

	/* Set the sRIO shadow registers configuration to 4/4/4/4 */
	__raw_writel(0x00000000, &krio_priv->regs->lsu_setup_reg[0]);

	/* Use LSU completion interrupt per LSU (not per SRCID) */
	for (i = krio_priv->lsu_start; i <= krio_priv->lsu_end; i++)
		lsu_mask |= (1 << i);

	__raw_writel(lsu_mask, &krio_priv->regs->lsu_setup_reg[1]);

	/* Enable LSU */
	__raw_writel(1, &(krio_priv->regs->blk[KEYSTONE_RIO_BLK_LSU_ID].enable));

	/* Set packet forwarding */
	for (i = 0; i < KEYSTONE_RIO_MAX_PKT_FW_ENTRIES; i++) {
		if ((krio_priv->board_rio_cfg.pkt_forwarding) && (i < 8)) {
			struct keystone_routing_config *routing = krio_priv->board_rio_cfg.routing_config;

			/* Enable packet forwarding DevId and port defined in DTS */
			__raw_writel(routing[i].dev_id_low
				     | (routing[i].dev_id_high << 16),
				     &(krio_priv->regs->pkt_fwd_cntl[i].pf_16b));
			__raw_writel((routing[i].dev_id_low & 0xff)
				     | ((routing[i].dev_id_high & 0xff ) << 8)
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

static void keystone_rio_reset_dpc(struct work_struct *work)
{
	struct keystone_rio_data *krio_priv =
		container_of(work, struct keystone_rio_data, reset_work);
	u32 ports_rst;
	u32 ports;
	u32 port;

	ports_rst = __raw_readl(&krio_priv->evt_mgmt_regs->evt_mgmt_rst_port_stat);

	dev_info(krio_priv->dev,
		 "reset device request received on ports: 0x%x\n", ports_rst);

	/* Acknowledge reset */
	ports = ports_rst;
	while (ports) {
		port = __ffs(ports);
		ports &= ~(1 << port);
		__raw_writel(KEYSTONE_RIO_PORT_PLM_STATUS_RST_REQ,
			     &krio_priv->phy_regs->phy_sp[port].status);
	}

	__raw_writel(ports_rst, &krio_priv->evt_mgmt_regs->evt_mgmt_rst_port_stat);

	/* Reinitialize SRIO peripheral */
	keystone_rio_shutdown_controller(krio_priv);
	keystone_rio_setup_controller(krio_priv);
}

static int keystone_rio_test_link(u8 port, struct keystone_rio_data *krio_priv)
{
	int res = 0;
	u32 value;

	res = keystone_rio_maint_read(krio_priv, port, 0xffff,
				      krio_priv->board_rio_cfg.size,
				      0, 0, sizeof(value), &value);

	return res;
}

static int keystone_rio_port_error_recovery(u32 port, struct keystone_rio_data *krio_priv)
{
	int res;
	u32 err_stat;
	u32 err_det;
	u32 plm_status;
	u32 r_port = krio_priv->board_rio_cfg.ports_remote[port];
	int i;

	if (unlikely(port >= KEYSTONE_RIO_MAX_PORT))
                return -EINVAL;

	err_stat   = __raw_readl(&krio_priv->serial_port_regs->sp[port].err_stat);
	err_det    = __raw_readl(&krio_priv->err_mgmt_regs->sp_err[port].det);
	plm_status = __raw_readl(&krio_priv->phy_regs->phy_sp[port].status);
	dev_dbg(krio_priv->dev,
		"port %d: err_stat = 0x%08x, err_det = 0x%08x, plm_status = 0x%08x\n",
		port, err_stat, err_det, plm_status);

	/* Acknowledge errors on this port */
	__raw_writel(err_stat & KEYSTONE_RIO_PORT_ERROR_MASK,
		     &krio_priv->serial_port_regs->sp[port].err_stat);
	__raw_writel(0, &krio_priv->err_mgmt_regs->sp_err[port].det);
	__raw_writel(plm_status & KEYSTONE_RIO_PORT_PLM_STATUS_ERRORS,
		     &krio_priv->phy_regs->phy_sp[port].status);

	if (unlikely(!(err_stat & RIO_PORT_N_ERR_STS_PORT_OK)))
                return -EINVAL;

	if (err_stat & RIO_PORT_N_ERR_STS_PW_OUT_ES) {
		u32 lm_resp;
		u32 ackid_stat;
		u32 l_ackid;
		u32 r_ackid;

		dev_dbg(krio_priv->dev,
		       "port %d: Output Error-Stopped recovery\n", port);

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
					"port %d: Input-Status response timeout\n", port);
				goto oes_rd_err;
			}

                        ndelay(KEYSTONE_RIO_TIMEOUT_NSEC);
			lm_resp = __raw_readl(
				&krio_priv->serial_port_regs->sp[port].link_maint_resp);
		} while (!(lm_resp & RIO_PORT_N_MNT_RSP_RVAL));

		dev_dbg(krio_priv->dev,
			"port %d: Input-Status response = 0x%08x\n", port, lm_resp);

		/* Set outbound ackID to the value expected by link partner */
		ackid_stat = __raw_readl(
			&krio_priv->serial_port_regs->sp[port].ackid_stat);

		dev_dbg(krio_priv->dev,
			"port %d: ackid_stat = 0x%08x\n", port, ackid_stat);

		l_ackid = (ackid_stat & RIO_PORT_N_ACK_INBOUND) >> 24;
		r_ackid = (lm_resp & RIO_PORT_N_MNT_RSP_ASTAT) >> 5;

		__raw_writel((l_ackid << 24) | r_ackid,
			     &krio_priv->serial_port_regs->sp[port].ackid_stat);

		udelay(50);

                /*
		 * Reread outbound ackID as it may have changed as a result of
		 * outstanding unacknowledged packets retransmission
                 */
		ackid_stat = __raw_readl(
			&krio_priv->serial_port_regs->sp[port].ackid_stat);

		dev_dbg(krio_priv->dev,
			"port %d: ackid_stat = 0x%08x\n", port, ackid_stat);

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
                               "port %d: failed to align ackIDs with link partner port %d\n",
                               port, r_port);
               }

oes_rd_err:
               err_stat   = __raw_readl(&krio_priv->serial_port_regs->sp[port].err_stat);
               err_det    = __raw_readl(&krio_priv->err_mgmt_regs->sp_err[port].det);
               plm_status = __raw_readl(&krio_priv->phy_regs->phy_sp[port].status);

               dev_dbg(krio_priv->dev,
                       "port %d: err_stat = 0x%08x, err_det = 0x%08x, plm_status = 0x%08x\n",
                       port, err_stat, err_det, plm_status);
	}

	if (err_stat & RIO_PORT_N_ERR_STS_PW_INP_ES) {
		dev_dbg(krio_priv->dev,
			"port %d: Input Error-Stopped recovery\n", port);

		res = keystone_rio_maint_write(krio_priv,
					       port,
					       0xffff,
					       krio_priv->board_rio_cfg.size,
					       0,
					       0x100 + RIO_PORT_N_MNT_REQ_CSR(r_port),
					       sizeof(u32),
					       RIO_MNT_REQ_CMD_IS);

                if (res < 0) {
                        dev_err(krio_priv->dev,
                               "port %d: failed to issue Input-Status request from link partner port %d\n",
                                port, r_port);
                }

		udelay(50);

		err_stat   = __raw_readl(&krio_priv->serial_port_regs->sp[port].err_stat);
		err_det    = __raw_readl(&krio_priv->err_mgmt_regs->sp_err[port].det);
		plm_status = __raw_readl(&krio_priv->phy_regs->phy_sp[port].status);

		dev_dbg(krio_priv->dev,
			"port %d: err_stat = 0x%08x, err_det = 0x%08x, plm_status = 0x%08x\n",
			port, err_stat, err_det, plm_status);
        }

	return err_stat & KEYSTONE_RIO_PORT_ERRORS;
}

static void keystone_rio_pe_dpc(struct work_struct *work)
{
	struct keystone_rio_data *krio_priv = container_of(
		to_delayed_work(work), struct keystone_rio_data, pe_work);
	u32 port;

	dev_dbg(krio_priv->dev, "errors on ports: 0x%x\n", krio_priv->pe_ports);

	for (port = 0; port < KEYSTONE_RIO_MAX_PORT; port++) {
		if (test_and_clear_bit(port, (void*)&krio_priv->pe_ports)) {

			/*  Recover from port error state */
			if (keystone_rio_port_error_recovery(port, krio_priv)) {

				/* If error recovery failed schedule another one if there is time left */
				if (krio_priv->pe_cnt-- > 1) {
					krio_priv->pe_ports |= BIT(port);
					schedule_delayed_work(&krio_priv->pe_work,
							      KEYSTONE_RIO_REGISTER_DELAY);
					continue;
				} else {
					dev_err(krio_priv->dev,
						"port %d: failed to recover from errors\n",
						port);
					continue;
				}
			}

			/* Perform test read on successful recovery */
			if (keystone_rio_test_link(port, krio_priv)) {
				dev_err(krio_priv->dev,
					"port %d: link test failed after error recovery\n",
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

	if (port >= KEYSTONE_RIO_MAX_PORT)
		return -EINVAL;

	/* Check port status */
	for (count = 0; count < 100; count++) {
		value = __raw_readl(&(krio_priv->serial_port_regs->sp[port].err_stat));
		if ((value & RIO_PORT_N_ERR_STS_PORT_OK) != 0)
			break;
		udelay(10);
	}

	if ((value & RIO_PORT_N_ERR_STS_PORT_OK) != 0) {
		res = keystone_rio_test_link(port, krio_priv);
		if (res != 0) {
			dev_err(krio_priv->dev,
				"link test failed on port %d\n", port);
			return -EIO;
		}
	} else {
		dev_dbg(krio_priv->dev,
			"port %d is not initialized - PORT_OK not set\n",
			port);
		return -EIO;
	}

	return 0; /* port must be solid OK */
}

/**
 * keystone_rio_port_enable - Enable a RapidIO port
 * @port: index of the port to configure
 */
static void keystone_rio_port_enable(u32 port, struct keystone_rio_data *krio_priv)
{
	/* Enable port */
	__raw_writel(0x600000, &(krio_priv->serial_port_regs->sp[port].ctl));
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
 * @path_mode: serdes configuration
 */
static int keystone_rio_port_init(u32 port, u32 path_mode, struct keystone_rio_data *krio_priv)
{
	u32 val;

	if (unlikely(port >= KEYSTONE_RIO_MAX_PORT))
		return -EINVAL;

	/* Program channel allocation to ports (1x, 2x or 4x) */
	__raw_writel(path_mode, &(krio_priv->phy_regs->phy_sp[port].path_ctl));

	/* Silence and discovery timers */
	if ((port == 0)|| (port == 2)) {
		__raw_writel(0x20000000,
			     &(krio_priv->phy_regs->phy_sp[port].silence_timer));
		__raw_writel(0x20000000,
			     &(krio_priv->phy_regs->phy_sp[port].discovery_timer));
	}

	/* Set multicast and packet forwarding mode otherwise unicast mode */
	val = krio_priv->board_rio_cfg.pkt_forwarding ? 0x00209000 : 0x00109000;
	__raw_writel(val, &(krio_priv->transport_regs->transport_sp[port].control));

	/* Enable interrupt for reset request */
	val = __raw_readl(&(krio_priv->evt_mgmt_regs->evt_mgmt_rst_int_en));
	__raw_writel(val | BIT(port),
		     &(krio_priv->evt_mgmt_regs->evt_mgmt_rst_int_en));

	/* Enable all PLM interrupts */
	__raw_writel(0xffffffff, &(krio_priv->phy_regs->phy_sp[port].int_enable));
	__raw_writel(1, &(krio_priv->phy_regs->phy_sp[port].all_int_en));

	/* Enable all errors */
	__raw_writel(0xffffffff, &(krio_priv->err_mgmt_regs->sp_err[port].rate_en));

	/* Cleanup port error status */
	__raw_writel(KEYSTONE_RIO_PORT_ERROR_MASK,
		     &(krio_priv->serial_port_regs->sp[port].err_stat));
	__raw_writel(0, &(krio_priv->err_mgmt_regs->sp_err[port].det));
	__raw_writel(KEYSTONE_RIO_PORT_PLM_STATUS_ERRORS,
		     &krio_priv->phy_regs->phy_sp[port].status);

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
		*data = __raw_readl(
			(void __iomem *)krio_priv->car_csr_regs + offset);

	dev_dbg(krio_priv->dev,
		"local_conf_r: index %d offset 0x%x data 0x%x\n",
		index, offset, *data);

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
		"local_conf_w: index %d offset 0x%x data 0x%x\n",
		index, offset, data);
	__raw_writel(data,
		(void __iomem *)krio_priv->car_csr_regs + offset);

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
	for (i = 0; i < 4; i++) {
		__raw_writel(0, &(krio_priv->port_write_regs->port_wr_rx_capt[i]));
	}

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
			struct keystone_rio_mbox_info *p_mbox;
			p_mbox = &(krio_priv->rx_mbox[i]);
			if (p_mbox->running) {
				/* Client callback (slot is not used) */
				p_mbox->port->inb_msg[p_mbox->id].mcback(p_mbox->port,
									 p_mbox->dev_id,
									 p_mbox->id,
									 0);
			}
		}
	}
}

/*
 * A dummy receive callback is needed for dmaengine teardown
 */
static void keystone_rio_rx_complete(void *data)
{
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

static void keystone_rio_close_rx_mbox(int mbox, struct keystone_rio_data *krio_priv)
{
	struct keystone_rio_mbox_info *rx_mbox = &krio_priv->rx_mbox[mbox];

	if (mbox >= KEYSTONE_RIO_MAX_MBOX)
		return;

	rx_mbox->running = 0;

	if (!rx_mbox->port)
		return;

	rx_mbox->port = NULL;

	/* Release associated resource */
	keystone_rio_free_rxu_map(rx_mbox->rxu_map_id[0], krio_priv);
	keystone_rio_free_rxu_map(rx_mbox->rxu_map_id[1], krio_priv);

	keystone_rio_mp_inb_exit(mbox, krio_priv);
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

	dev_info(krio_priv->dev, "close inb mbox: mport = 0x%x, mbox = %d\n",
		(u32) mport, mbox);

	keystone_rio_close_rx_mbox(mbox, krio_priv);
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
	struct dma_async_tx_descriptor *desc = NULL;
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
		p_info->buff = NULL;
		kfree(p_info);
		return -EINVAL;
	}

	desc = dmaengine_prep_slave_sg(krx_chan->dma_channel, p_info->sg,
				       p_info->sg_ents, DMA_DEV_TO_MEM,
				       DMA_HAS_EPIB | DMA_HAS_PSINFO);

	if (IS_ERR_OR_NULL(desc)) {
		u32 err = 0;
		dma_unmap_sg(krio_priv->dev, &p_info->sg[2],
			     1, DMA_FROM_DEVICE);
		p_info->buff = NULL;
		kfree(p_info);
		err = PTR_ERR(desc);
		if (err != -ENOMEM) {
			dev_err(krio_priv->dev,
				"dma prep failed, error %d\n", err);
		}
		return -EINVAL;
	}

	desc->callback_param = p_info;
	desc->callback = keystone_rio_rx_complete;
	p_info->cookie = desc->cookie;

	return dma_rxfree_refill_one(krx_chan->dma_channel, 0, desc);
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
	void *buff = NULL;
	u32   cc;

	p_info = (struct keystone_rio_packet *) dma_get_one(krx_chan->dma_channel);
	if (!p_info) {
		goto end;
	}

	buff = p_info->buff;

	dma_unmap_sg(krio_priv->dev, &p_info->sg[2], 1, DMA_FROM_DEVICE);

	/* Check CC from PS descriptor word 1 */
	cc = keystone_rio_mp_get_cc(p_info->psdata[1], krx_chan->packet_type);
	if (cc)
		dev_warn(krio_priv->dev,
			 "MP receive completion code is non zero (0x%x)\n", cc);
end:
	dmaengine_resume(krx_chan->dma_channel);

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

static void keystone_rio_close_tx_mbox(int mbox, struct keystone_rio_data *krio_priv)
{
	struct keystone_rio_mbox_info *tx_mbox = &(krio_priv->tx_mbox[mbox]);

	if (mbox >= KEYSTONE_RIO_MAX_MBOX)
		return;

	tx_mbox->running = 0;

	if (!tx_mbox->port)
		return;

	tx_mbox->port = NULL;

	keystone_rio_mp_outb_exit(krio_priv);
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

	dev_info(krio_priv->dev, "close outb mbox: mport = 0x%x, mbox = %d\n",
		(u32) mport, mbox);

	keystone_rio_close_tx_mbox(mbox, krio_priv);
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

	WARN_ON(p_info->status != DMA_SUCCESS && p_info->status != DMA_ERROR);

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
	u32 plen;
	u32 packet_type;
	int ret = 0;
	void *send_buffer = NULL;

	if (unlikely(krio_priv->tx_mbox[mbox].port != mport))
		return -EINVAL;

	/*
	 * Ensure that the number of bytes being transmitted is a multiple
	 * of double-word. This is as per the specification.
	 */
	plen = ((len + 7) & ~0x7);

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
		kfree(p_info);
		if (send_buffer)
			kfree(send_buffer);
		dev_warn(krio_priv->dev, "failed to map transmit packet\n");
		return -ENXIO;
	}

	desc = dmaengine_prep_slave_sg(krio_priv->tx_channel, p_info->sg,
				       p_info->sg_ents, DMA_MEM_TO_DEV,
				       DMA_HAS_EPIB | DMA_HAS_PSINFO | DMA_HAS_PKTTYPE
				       | (packet_type  << DMA_PKTTYPE_SHIFT));

	if (IS_ERR_OR_NULL(desc)) {
		dma_unmap_sg(krio_priv->dev, &p_info->sg[2], 1, DMA_TO_DEVICE);
		kfree(p_info);
		if (send_buffer)
			kfree(send_buffer);
		dev_warn(krio_priv->dev, "failed to prep slave dma\n");
		return -ENOBUFS;
	}

	desc->callback_param = p_info;
	desc->callback = keystone_rio_tx_complete;
	p_info->cookie = dmaengine_submit(desc);

	if (dma_submit_error(p_info->cookie)) {
		dev_warn(krio_priv->dev, "failed to submit packet for dma: %d\n",
			 p_info->cookie);
		kfree(p_info);
		if (send_buffer)
			kfree(send_buffer);
		return -EBUSY;
	}

	return ret;
}

/*------------------------ Main Linux driver functions -----------------------*/

static int keystone_rio_query_mport(struct rio_mport *mport,
				    struct rio_mport_attr *attr)
{
	struct keystone_rio_data *krio_priv = mport->priv;
	u32 port = mport->index;
	u32 rval;

	if (!attr)
		return -EINVAL;

	rval = __raw_readl(&krio_priv->serial_port_regs->sp[port].err_stat);
	if (rval & RIO_PORT_N_ERR_STS_PORT_OK) {
		rval = __raw_readl(&krio_priv->serial_port_regs->sp[port].ctl2);
		attr->link_speed = (rval & RIO_PORT_N_CTL2_SEL_BAUD) >> 28;
		rval = __raw_readl(&krio_priv->serial_port_regs->sp[port].ctl);
		attr->link_width = (rval & RIO_PORT_N_CTL_IPW) >> 27;
	} else
		attr->link_speed = RIO_LINK_DOWN;

#ifdef CONFIG_RAPIDIO_DMA_ENGINE
	attr->flags        = RIO_MPORT_DMA; /* Supporting DMA but not HW SG mode*/
	attr->dma_max_sge  = KEYSTONE_RIO_DMA_MAX_DESC - 1;
	attr->dma_max_size = KEYSTONE_RIO_MAX_DIO_PKT_SIZE;
	attr->dma_align    = KEYSTONE_RIO_DIO_ALIGNMENT;
#else
	attr->flags        = 0;
	attr->dma_max_sge  = 0;
	attr->dma_max_size = 0;
	attr->dma_align    = 0;
#endif

	return 0;
}

struct rio_mport *keystone_rio_register_mport(u32 port_id, u32 size,
					      struct keystone_rio_data *krio_priv)
{
	struct rio_ops   *ops;
	struct rio_mport *mport;

	ops = kzalloc(sizeof(struct rio_ops), GFP_KERNEL);

	ops->lcread   = keystone_local_config_read;
	ops->lcwrite  = keystone_local_config_write;
	ops->cread    = keystone_rio_config_read;
	ops->cwrite   = keystone_rio_config_write;
	ops->dsend    = keystone_rio_dbell_send;

	ops->open_outb_mbox   = keystone_rio_open_outb_mbox;
	ops->close_outb_mbox  = keystone_rio_close_outb_mbox;
	ops->open_inb_mbox    = keystone_rio_open_inb_mbox;
	ops->close_inb_mbox   = keystone_rio_close_inb_mbox;
	ops->add_outb_message = keystone_rio_hw_add_outb_message;
	ops->add_inb_buffer   = keystone_rio_hw_add_inb_buffer;
	ops->get_inb_message  = keystone_rio_hw_get_inb_message;
	ops->query_mport      = keystone_rio_query_mport;

	mport = kzalloc(sizeof(struct rio_mport), GFP_KERNEL);

	/*
	 * Set the sRIO port physical Id into the index field,
	 * the id field will be set by rio_register_mport() to
	 * the logical Id
	 */
	mport->index = port_id;
	mport->priv  = krio_priv;
	mport->dev.parent = krio_priv->dev;
	INIT_LIST_HEAD(&mport->dbells);

	/*
	 * Make a dummy per port region as ports are not
	 * really separated on KeyStone
	 */
	mport->iores.start = (u32)(krio_priv->serial_port_regs) +
		offsetof(struct keystone_rio_serial_port_regs,
			 sp[port_id].link_maint_req);

	mport->iores.end = (u32)(krio_priv->serial_port_regs) +
		offsetof(struct keystone_rio_serial_port_regs,
			 sp[port_id].ctl);

	mport->iores.flags = IORESOURCE_MEM;

	rio_init_dbell_res(&mport->riores[RIO_DOORBELL_RESOURCE], 0, 0xffff);
	rio_init_mbox_res(&mport->riores[RIO_INB_MBOX_RESOURCE], 0, KEYSTONE_RIO_MAX_MBOX);
	rio_init_mbox_res(&mport->riores[RIO_OUTB_MBOX_RESOURCE], 0, KEYSTONE_RIO_MAX_MBOX);

	sprintf(mport->name, "RIO%d mport", port_id);

	mport->ops      = ops;
	mport->sys_size = size;
	mport->phy_type = RIO_PHY_SERIAL;
	/* Hard coded here because in rio_disc_mport(), it is used in
	   rio_enum_complete() before it is retrieved in
	   rio_disc_peer() => rio_setup_device() */
	mport->phys_efptr = 0x100;

	/*
	 * Register the new mport
	 */
	rio_register_mport(mport);

	krio_priv->mport[port_id] = mport;

#ifdef CONFIG_RAPIDIO_DMA_ENGINE
	/*
	 * Register the DMA engine for DirectIO transfers
	 */
	keystone_rio_dma_register(mport, krio_priv->board_rio_cfg.dma_channel_num);

	/*
	 * Reserve one channel for doorbells
	 */
	keystone_rio_lsu_dma_allocate_channel(mport);
#endif

#ifdef CONFIG_RAPIDIO_ENUM_BASIC
	/*
	 * We may have a very late mport registration
	 * so perform explicitely the basic attachement (and
	 * eventually scanning here).
	 */
	rio_basic_attach();
#endif

	return mport;
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
		dev_err(krio_priv->dev, "could not find %s node\n", node_name);
		of_node_put(node);
		return -ENODEV;
	}

	dev_dbg(krio_priv->dev, "using node \"%s\"\n", node_name);

	/* DMA rx chan config */
	if (of_property_read_string(node, "rx_channel", &krx_chan->name) < 0) {
		char name[8];
		dev_warn(krio_priv->dev,
			 "missing \"rx_channel\" parameter\n");
		snprintf(name, sizeof(name), "riorx-%d", mbox);
		krx_chan->name = name;
	}

	if (of_property_read_u32_array(node, "rx_queue_depth",
				       krx_chan->queue_depths,
				       KEYSTONE_QUEUES_PER_CHAN) < 0) {
		dev_warn(krio_priv->dev,
			 "missing \"rx_queue_depth\" parameter\n");
		krx_chan->queue_depths[0] = 128;
	}

	if (of_property_read_u32_array(node, "rx_buffer_size",
				       krx_chan->buffer_sizes,
				       KEYSTONE_QUEUES_PER_CHAN) < 0) {
		dev_warn(krio_priv->dev,
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

static int keystone_rio_get_controller_defaults(struct device_node *node,
						 struct keystone_rio_data *krio_priv)
{
	int ret = 0;
	struct keystone_rio_board_controller_info *c = &krio_priv->board_rio_cfg;
	u32 temp[24];
	int i;
	int lanes;
	int mbox;

	i = of_property_match_string(node, "reg-names", "rio");
	krio_priv->regs = of_iomap(node, i);
	if (!krio_priv->regs) {
		dev_err(krio_priv->dev, "missing \"rio\" regs\n");
		ret = -ENOENT;
		goto error;
	}

	krio_priv->car_csr_regs	    = (void __iomem *) krio_priv->regs + 0xb000;
	krio_priv->serial_port_regs = (void __iomem *) krio_priv->regs + 0xb100;
	krio_priv->err_mgmt_regs    = (void __iomem *) krio_priv->regs + 0xc000;
	krio_priv->phy_regs	    = (void __iomem *) krio_priv->regs + 0x1b000;
	krio_priv->transport_regs   = (void __iomem *) krio_priv->regs + 0x1b300;
	krio_priv->pkt_buf_regs	    = (void __iomem *) krio_priv->regs + 0x1b600;
	krio_priv->evt_mgmt_regs    = (void __iomem *) krio_priv->regs + 0x1b900;
	krio_priv->port_write_regs  = (void __iomem *) krio_priv->regs + 0x1ba00;
	krio_priv->link_regs	    = (void __iomem *) krio_priv->regs + 0x1bd00;
	krio_priv->fabric_regs	    = (void __iomem *) krio_priv->regs + 0x1be00;

	i = of_property_match_string(node, "reg-names", "jtagid");
	krio_priv->jtagid_reg = of_iomap(node, i);
	if (!krio_priv->jtagid_reg) {
		dev_err(krio_priv->dev, "missing \"jtagid\" reg\n");
		ret = -ENOENT;
		goto error;
	}

	i = of_property_match_string(node, "reg-names", "serdes");
	krio_priv->serdes_regs = of_iomap(node, i);
	if (!krio_priv->serdes_regs) {
		dev_err(krio_priv->dev, "missing \"serdes\" regs\n");
		ret = -ENOENT;
		goto error;
	}

	if (of_property_read_u32 (node, "dev-id-size", &c->size)) {
		dev_err(krio_priv->dev, "missing \"dev-id-size\" parameter\n");
		ret = -ENOENT;
		goto error;
	}

	if (of_property_read_u32 (node, "ports", &c->ports)) {
		dev_err(krio_priv->dev, "missing \"ports\" parameter\n");
		ret = -ENOENT;
		goto error;
	}

	if (of_property_read_u32_array(node, "ports_remote",
				       c->ports_remote, KEYSTONE_RIO_MAX_PORT)) {
		/* Assume by default that remote ports are same as local ports */
		for (i = 0; i < KEYSTONE_RIO_MAX_PORT; i++)
			c->ports_remote[i] = i;

		dev_warn(krio_priv->dev, "missing \"remote_ports\" parameter, "
			 "using default mapping\n");
	}

	/* SerDes config */
	if (!of_find_property(node, "keystone2-serdes", NULL)) {
		/*
		 * Mode 0: sRIO config 0: MPY = 5x, div rate = half,
		 * link rate = 3.125 Gbps, mode 1x
		 */
		c->path_mode			   = 0;
		c->serdes_config.cfg_cntl	   = 0x0c053860;
		c->serdes_config.serdes_cfg_pll	   = 0x0229;
		c->serdes_config.prescalar_srv_clk = 0x001e;

		for (i = 0; i < KEYSTONE_RIO_MAX_PORT; i++) {
			c->serdes_config.rx_chan_config[i] = 0x00440495;
			c->serdes_config.tx_chan_config[i] = 0x00180795;
		}

		i = of_property_match_string(node, "reg-names", "serdes_sts");
		krio_priv->serdes_sts_reg = of_iomap(node, i);
		if (!krio_priv->serdes_sts_reg) {
			dev_err(krio_priv->dev, "missing \"serdes_sts\" reg\n");
			ret = -ENOENT;
			goto error;
		}
	} else {
		c->keystone2_serdes	           = 1;
		c->serdes_config.cfg_cntl          = 0x00053800;
		c->serdes_config.prescalar_srv_clk = 0x001f;

		if (of_property_read_u32(node, "baudrate", &c->serdes_baudrate)) {
			dev_warn(krio_priv->dev,
				 "missing \"baudrate\" parameter, "
				 "setting 5Gbps as a default\n");
			c->serdes_baudrate = KEYSTONE_RIO_BAUD_5_000;
		}

		/* Path mode config (mapping of SerDes lanes to port widths) */
		if (of_property_read_u32(node, "path_mode", &c->path_mode)) {
			dev_err(krio_priv->dev,
				"missing \"path_mode\" parameter\n");
			ret = -ENOENT;
			goto error;
		}
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
		ret = -EINVAL;
		goto error;
	}

	lanes = keystone_rio_get_lane_config(c->ports, c->path_mode);
	if (lanes < 0) {
		dev_err(krio_priv->dev,
			"cannot determine used SerDes lanes\n");
		ret = -EINVAL;
		goto error;
	}
	c->lanes = (u32) lanes;

	/* Port register timeout */
	if (of_property_read_u32(node, "port-register-timeout",
				 &(c->port_register_timeout))) {
		c->port_register_timeout = 30;
	}

	/* LSUs */
	if (of_property_read_u32_array(node, "lsu", &temp[0], 2)) {
		krio_priv->lsu_start = 0;
		krio_priv->lsu_end   = 0;
	} else {
		krio_priv->lsu_start = (u8) temp[0];
		krio_priv->lsu_end   = (u8) temp[1];
	}

	dev_dbg(krio_priv->dev, "using LSU %d - %d range\n",
		krio_priv->lsu_start, krio_priv->lsu_end);

#ifdef CONFIG_RAPIDIO_DMA_ENGINE
	/* DIO (virtual) DMA channels */
	if (of_property_read_u32(node, "num-dio-channels",
				 &(c->dma_channel_num)) < 0) {
		dev_err(krio_priv->dev,
			"missing \"num-dio-channels\" parameter\n");
		return -ENOENT;
	}
#endif

	/* DMA tx chan config */
	if (of_property_read_string(node, "tx_channel",
				    &krio_priv->tx_chan_name) < 0){
		dev_warn(krio_priv->dev,
			 "missing \"tx_channel\" parameter\n");
		krio_priv->tx_chan_name = "riotx";
	}

	if (of_property_read_u32(node, "tx_queue_depth",
				 &krio_priv->tx_queue_depth) < 0) {
		dev_warn(krio_priv->dev,
			 "missing \"tx_queue_depth\" parameter\n");
		krio_priv->tx_queue_depth = 128;
	}

	/* Mailboxes configuration */
	if (of_property_read_u32(node, "num-mboxes",
				 &krio_priv->num_mboxes) < 0) {
		dev_warn(krio_priv->dev,
			 "missing \"num-mboxes\" parameter\n");
		krio_priv->num_mboxes = 1;
	}

	if (krio_priv->num_mboxes > KEYSTONE_RIO_MAX_MBOX) {
		dev_warn(krio_priv->dev,
			 "wrong \"num_mboxes\" parameter value %d, set to %d\n",
			 krio_priv->num_mboxes, KEYSTONE_RIO_MAX_MBOX);
		krio_priv->num_mboxes = KEYSTONE_RIO_MAX_MBOX;
	}

	for (mbox = 0; mbox < krio_priv->num_mboxes; mbox++) {
		(void) keystone_rio_get_mbox_defaults(mbox, node, krio_priv);
	}

	/* Interrupt config */
	c->rio_irq = irq_of_parse_and_map(node, 0);
	if (c->rio_irq < 0) {
		dev_err(krio_priv->dev, "missing \"rio_irq\" parameter\n");
		ret = -ENOENT;
		goto error;
	}

	c->lsu_irq = irq_of_parse_and_map(node, 1);
	if (c->lsu_irq < 0) {
		dev_err(krio_priv->dev, "missing \"lsu_irq\" parameter\n");
		ret = -ENOENT;
		goto error;
	}

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

	goto exit;

error:
	if (krio_priv->serdes_sts_reg)
		iounmap(krio_priv->serdes_sts_reg);
	if (krio_priv->serdes_regs)
		iounmap(krio_priv->serdes_regs);
	if (krio_priv->jtagid_reg)
		iounmap(krio_priv->jtagid_reg);
	if (krio_priv->regs)
		iounmap(krio_priv->regs);

exit:
	return ret;
}

static int keystone_rio_port_chk(struct keystone_rio_data *krio_priv)
{
	u32 ports = krio_priv->ports_registering;
	u32 size  = krio_priv->board_rio_cfg.size;
	struct rio_mport *mport;

	/* Check ports status (only the requested ones) */
	krio_priv->ports_registering = 0;
	while (ports) {
		int status;
		u32 port = __ffs(ports);
		ports &= ~(1 << port);

		/*
		 * Check the port status here before calling the generic RapidIO
		 * layer. Port status check is done in rio_mport_is_active() as
		 * well but we need to do it our way first due to some delays in
		 * hw initialization.
		 */
		status = keystone_rio_port_status(port, krio_priv);
		if (status == 0) {
			/* Register mport only if this is initial port check */
 			if (!krio_priv->mport[port]) {
				mport = keystone_rio_register_mport(port, size, krio_priv);
				if (!mport) {
					dev_err(krio_priv->dev,
						"failed to register mport %d\n", port);
					return -1;
				}
				dev_info(krio_priv->dev,
					 "port RIO%d host_deviceid %d registered\n",
					 port, mport->host_deviceid);
			} else {
				dev_info(krio_priv->dev,
					 "port RIO%d host_deviceid %d ready\n",
					 port, krio_priv->mport[port]->host_deviceid);
			}

			/* Update routing after discovery/enumeration with new dev id */
			if (krio_priv->board_rio_cfg.pkt_forwarding)
				keystone_rio_port_set_routing(port, krio_priv);
		} else {
			krio_priv->ports_registering |= (1 << port);
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
		/* If port check failed schedule next check if there is time left */
		if (krio_priv->port_chk_cnt-- > 1) {
			schedule_delayed_work(&krio_priv->port_chk_task,
					      KEYSTONE_RIO_REGISTER_DELAY);
		} else {
			dev_info(krio_priv->dev,
				 "RIO port register timeout, port mask 0x%x not ready",
				 krio_priv->ports_registering);
               }
	}
}

/*
 * Platform configuration setup
 */
static int keystone_rio_setup_controller(struct keystone_rio_data *krio_priv)
{
	u32 ports;
	u32 lanes;
	u32 p;
	u32 baud;
	u32 path_mode;
	u32 size = 0;
	int res = 0;
	char str[8];

	size      = krio_priv->board_rio_cfg.size;
	ports     = krio_priv->board_rio_cfg.ports;
	lanes     = krio_priv->board_rio_cfg.lanes;
	baud      = krio_priv->board_rio_cfg.serdes_baudrate;
	path_mode = krio_priv->board_rio_cfg.path_mode;

	dev_dbg(krio_priv->dev,
		"size = %d, ports = 0x%x, lanes = 0x%x, baud = %d, path_mode = %d\n",
		size, ports, lanes, baud, path_mode);

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
		return -EINVAL;
	}

	dev_info(krio_priv->dev,
		 "initializing %s Gbps interface with port configuration %d\n",
		 str, path_mode);

	/* Hardware set up of the controller */
	res = keystone_rio_hw_init(baud, krio_priv);
	if (res < 0) {
		dev_err(krio_priv->dev,
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
	keystone_rio_interrupt_setup(krio_priv);

	/* Start the controller */
	keystone_rio_start(krio_priv);

	if (K2_SERDES(krio_priv)) {
		if (lanes > 0) {
			res = k2_rio_serdes_wait_lock(krio_priv, lanes);
			if (res < 0)
			    dev_info(krio_priv->dev,
				     "SerDes for lane mask 0x%x on %s Gbps not locked\n",
				     lanes, str);
		}
	}

	/* Use ports (only the requested ones) */
	while (ports) {
		u32 port = __ffs(ports);
		ports &= ~(1 << port);

		res = keystone_rio_port_init(port, path_mode, krio_priv);
		if (res < 0) {
			dev_err(krio_priv->dev,
				"initialization of port %d failed\n", port);
			return res;
		}

		/* Start the port */
		keystone_rio_port_enable(port, krio_priv);
	}

	krio_priv->ports_registering = krio_priv->board_rio_cfg.ports;

	/* Check if port is working and register the associated mport */
	res = keystone_rio_port_chk(krio_priv);
	if (res) {
		if (res == -1)
			return -ENOMEM;

		krio_priv->port_chk_cnt =
			krio_priv->board_rio_cfg.port_register_timeout /
			(KEYSTONE_RIO_REGISTER_DELAY / HZ);

		schedule_delayed_work(&krio_priv->port_chk_task,
				      KEYSTONE_RIO_REGISTER_DELAY);
        }

       return 0;
}

static void keystone_rio_shutdown_controller(struct keystone_rio_data *krio_priv)
{
	int i;

	keystone_rio_interrupt_release(krio_priv);

	for (i = 0; i < KEYSTONE_RIO_MAX_MBOX; i++) {
		keystone_rio_close_tx_mbox(i, krio_priv);
		keystone_rio_close_rx_mbox(i, krio_priv);
	}

	/* Stop the hw controller */
	keystone_rio_stop(krio_priv);

	/* Wait current DMA transfers to finish */
	mdelay(10);

	/* Shutdown associated SerDes */
	keystone_rio_serdes_shutdown(krio_priv, krio_priv->board_rio_cfg.lanes);

	/* Disable blocks */
	__raw_writel(0, &krio_priv->regs->gbl_en);
	for (i = 0; i < KEYSTONE_RIO_BLK_NUM; i++) {
		__raw_writel(0, &(krio_priv->regs->blk[i].enable));
		while(__raw_readl(&(krio_priv->regs->blk[i].status)) & 0x1);
	}
}

static int keystone_rio_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct keystone_rio_data *krio_priv;
	int res = 0;
	int i;

	dev_info(&pdev->dev, "KeyStone RapidIO driver %s\n", DRIVER_VER);

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
	res = keystone_rio_get_controller_defaults(node, krio_priv);
	if (res < 0) {
		dev_err(&pdev->dev, "failed to get configuration\n");
		return res;
	}

	/* sRIO main driver (global ressources) */
	krio_priv->lsu_free = krio_priv->lsu_start;
	krio_priv->lsu_maint = keystone_rio_lsu_alloc(krio_priv);

	mutex_init(&krio_priv->lsu_lock_maint);

#ifdef CONFIG_RAPIDIO_DMA_ENGINE
	for (i = 0; i < KEYSTONE_RIO_LSU_NUM; i++)
		INIT_LIST_HEAD(&krio_priv->dma_channels[i]);
#endif

	INIT_DELAYED_WORK(&krio_priv->port_chk_task, keystone_rio_port_chk_task);
	INIT_DELAYED_WORK(&krio_priv->pe_work, keystone_rio_pe_dpc);
	INIT_WORK(&krio_priv->reset_work, keystone_rio_reset_dpc);

	/* Enable sRIO clock */
	krio_priv->clk = clk_get(&pdev->dev, "clk_srio");
	if (IS_ERR(krio_priv->clk)) {
		dev_err(&pdev->dev, "Unable to get Keystone SRIO clock\n");
		return -EBUSY;
	} else {
		clk_prepare_enable(krio_priv->clk);
		ndelay(100);
		clk_disable_unprepare(krio_priv->clk);
		ndelay(100);
		clk_prepare_enable(krio_priv->clk);
	}

	/* Setup the sRIO controller */
	res = keystone_rio_setup_controller(krio_priv);
	if (res < 0)
		return res;

	return 0;
}

static void keystone_rio_shutdown(struct platform_device *pdev)
{
	struct keystone_rio_data *krio_priv = platform_get_drvdata(pdev);

	keystone_rio_shutdown_controller(krio_priv);

	if (krio_priv->clk) {
		clk_disable_unprepare(krio_priv->clk);
		clk_put(krio_priv->clk);
	}
}

static int __exit keystone_rio_remove(struct platform_device *pdev)
{
	struct keystone_rio_data *krio_priv = platform_get_drvdata(pdev);
	u32 ports = krio_priv->board_rio_cfg.ports;

	/* Shutdown the hw controller */
	keystone_rio_shutdown(pdev);

	/* Retrieve all registered mports */
	while (ports) {
		struct rio_mport *mport;
		u32 port = __ffs(ports);
		ports &= ~(1 << port);

		mport = krio_priv->mport[port];

#ifdef CONFIG_RAPIDIO_DMA_ENGINE
		if (mport) {
			keystone_rio_lsu_dma_free_channel(mport);
			keystone_rio_dma_unregister(mport);
		}
#endif
		/* We should also unregister the mport from the RIO framework */
		krio_priv->mport[port] = NULL;
	}

	platform_set_drvdata(pdev, NULL);
	kfree(krio_priv);

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
