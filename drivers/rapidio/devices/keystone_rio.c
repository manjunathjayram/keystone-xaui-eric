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
#include <linux/device.h>
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

#include "keystone_rio_serdes.h"
#include "keystone_rio.h"

#define DRIVER_VER "v1.3"

static bool serdes_calibration;
module_param(serdes_calibration, bool, 0);
MODULE_PARM_DESC(
	serdes_calibration,
	"Perform Serdes calibration before starting RapidIO (default = 0)");

static bool enable_ports = 1;
module_param(enable_ports, bool, 0);
MODULE_PARM_DESC(
	enable_port,
	"Enable RapidIO ports at boottime (default = 1)");

static void dbell_handler(
	struct keystone_rio_data *krio_priv);
static void keystone_rio_port_write_handler(
	struct keystone_rio_data *krio_priv);
static void keystone_rio_handle_logical_error(
	struct keystone_rio_data *krio_priv);
static int  keystone_rio_setup_controller(
	struct keystone_rio_data *krio_priv);
static void keystone_rio_shutdown_controller(
	struct keystone_rio_data *krio_priv);

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
	u32 pending_err_int = __raw_readl(
		&(krio_priv->regs->lsu_int[0].status));

#ifdef CONFIG_RAPIDIO_DMA_ENGINE
	u32 pending_lsu_int;

	/* Call DMA interrupt handling */
	while ((pending_lsu_int = __raw_readl(
			&(krio_priv->regs->lsu_int[1].status))
		& KEYSTONE_RIO_ICSR_LSU1_COMPLETE_MASK)) {
		u32 lsu = __ffs(pending_lsu_int);

		keystone_rio_dma_interrupt_handler(krio_priv, lsu, 0);

		__raw_writel(1 << lsu, &(krio_priv->regs->lsu_int[1].clear));
	}

	/* In case of LSU completion with error */
	if (pending_err_int & KEYSTONE_RIO_ICSR_LSU0_ERROR_MASK) {

		keystone_rio_dma_interrupt_handler(krio_priv, 0, 1);

		__raw_writel(
			pending_err_int & KEYSTONE_RIO_ICSR_LSU0_ERROR_MASK,
			&(krio_priv->regs->lsu_int[0].clear));
	}
#endif

	return IRQ_HANDLED;
}

static void reset_symbol_handler(struct keystone_rio_data *krio_priv)
{
	/* Disable SerDes lanes asap to generate a loss of link */
	krio_priv->serdes.ops->disable_lanes(krio_priv->board_rio_cfg.lanes,
					     &krio_priv->serdes);

	/* Schedule SRIO peripheral reinitialization */
	schedule_work(&krio_priv->reset_work);
}

static void special_interrupt_handler(int ics,
				      struct keystone_rio_data *krio_priv)
{
	/* Acknowledge the interrupt */
	__raw_writel(BIT(ics), &krio_priv->regs->err_rst_evnt_int_clear);

	dev_dbg(krio_priv->dev, "ics = %d\n", ics);

	switch (ics) {
	case KEYSTONE_RIO_MCAST_EVT_INT:
		/* Multi-cast event control symbol interrupt received */
		break;

	case KEYSTONE_RIO_PORT_WRITEIN_INT:
		/* Port-write-in request received on any port */
		keystone_rio_port_write_handler(krio_priv);
		break;

	case KEYSTONE_RIO_EVT_CAP_ERROR_INT:
		/* Logical layer error management event capture */
		keystone_rio_handle_logical_error(krio_priv);
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
		reset_symbol_handler(krio_priv);

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
		pending_err_rst_evnt_int &= ~BIT(ics);
		special_interrupt_handler(ics, krio_priv);
	}

	/* Call doorbell handler(s) */
	dbell_handler(krio_priv);

	return IRQ_HANDLED;
}

/*
 * Map a SRIO event to a SRIO interrupt
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
	__raw_writel(KEYSTONE_RIO_ERR_RST_EVNT_MASK,
		     &(krio_priv->regs->err_rst_evnt_int_clear));

	for (lsu = krio_priv->lsu_start; lsu <= krio_priv->lsu_end; lsu++)
		__raw_writel(0xffffffff,
			     &(krio_priv->regs->lsu_int[lsu].clear));

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

	/*
	 * Error, reset and special event interrupts are routed to RIO
	 * interrupt dest 0 (Rx/Tx)
	 */
	keystone_rio_interrupt_map(&(krio_priv->regs->err_rst_evnt_int_route[0]),
				   0x00000111, KEYSTONE_GEN_RIO_INT);
	keystone_rio_interrupt_map(&(krio_priv->regs->err_rst_evnt_int_route[1]),
				   0x00001111, KEYSTONE_GEN_RIO_INT);
	keystone_rio_interrupt_map(&(krio_priv->regs->err_rst_evnt_int_route[2]),
				   0x00000001, KEYSTONE_GEN_RIO_INT);

	/*
	 * The doorbell interrupts routing table is for the 16 general purpose
	 * interrupts
	 */
	__raw_writel(0x1, &(krio_priv->regs->interrupt_ctl));

	/* Do not use pacing */
	__raw_writel(0x0000ffff, &krio_priv->regs->intdst_rate_disable);

	/* Attach interrupt handlers */
	res = request_irq(krio_priv->board_rio_cfg.rio_irq,
			  rio_interrupt_handler,
			  0,
			  "SRIO",
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
			  "SRIO LSU",
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

	/* If interrupt mode, do not spin */
	if (interrupt_req)
		count = KEYSTONE_RIO_TIMEOUT_CNT;
	else
		count = 0;

	/* Check if there is space in the LSU shadow reg and that it is free */
	while (1) {
		status = __raw_readl(
			&(krio_priv->regs->lsu_reg[lsu].busy_full));

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
	__raw_writel((u32) (tgt_addr >> 32),
		     &(krio_priv->regs->lsu_reg[lsu].addr_msb));

	/* LSU Reg 1 - LSB of destination */
	__raw_writel((u32) (tgt_addr),
		     &(krio_priv->regs->lsu_reg[lsu].addr_lsb_cfg_ofs));

	/* LSU Reg 2 - source address */
	__raw_writel(src_addr, &(krio_priv->regs->lsu_reg[lsu].phys_addr));

	/* LSU Reg 3 - byte count */
	__raw_writel(size_bytes,
		     &(krio_priv->regs->lsu_reg[lsu].dbell_val_byte_cnt));

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
		      | (size ? BIT(10) : 0)
		      | ((u32) dest_id << 16)
		      | (interrupt_req & 0x1)),
		     &(krio_priv->regs->lsu_reg[lsu].destid));

	/*
	 * LSU Reg 5 -
	 * doorbell info = packet_type[16-31],
	 * hop count = packet_type [8-15]
	 * FType = packet_type[4-7], TType = packet_type[0-3]
	 * Writing this register will initiate the transfer
	 */
	__raw_writel(packet_type,
		     &(krio_priv->regs->lsu_reg[lsu].dbell_info_fttype));

out:
	return res;
}

/*
 * Cancel a LSU transfer
 */
static inline void keystone_rio_lsu_cancel_transfer(
	int lsu,
	struct keystone_rio_data *krio_priv)
{
	u32 status;
	u32 count = 0;

	while (1) {
		/* Read register 6 to get the lock */
		status = __raw_readl(
			&(krio_priv->regs->lsu_reg[lsu].busy_full));

		/* If not busy or if full, we can flush */
		if (((status & KEYSTONE_RIO_LSU_FULL_MASK) == 0x0)
		    || (status & KEYSTONE_RIO_LSU_BUSY_MASK)) {
			break;
		}

		count++;
		if (count >= KEYSTONE_RIO_TIMEOUT_CNT) {
			dev_err(krio_priv->dev,
				"no LSU%d shadow register available for flushing\n",
				lsu);
			return;
		}

		ndelay(KEYSTONE_RIO_TIMEOUT_NSEC);
	}

	if (status & KEYSTONE_RIO_LSU_FULL_MASK) {
		/* Flush the transaction with our privID */
		__raw_writel(BIT(0) | (8 << 28),
			     &(krio_priv->regs->lsu_reg[lsu].busy_full));
	} else {
		/* Flush the transaction with our privID and CBusy bit */
		__raw_writel(BIT(0) | BIT(27) | (8 << 28),
			     &(krio_priv->regs->lsu_reg[lsu].busy_full));
	}
}

/*
 * Complete a LSU transfer
 */
int keystone_rio_lsu_complete_transfer(int lsu,
				       u32 lsu_context,
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

	if (unlikely(status))
		dev_dbg(krio_priv->dev, "LSU%d status 0x%x\n", lsu, status);

	switch (status & KEYSTONE_RIO_LSU_CC_MASK) {
	case KEYSTONE_RIO_LSU_CC_TIMEOUT:
		res = -ETIMEDOUT;
		keystone_rio_lsu_cancel_transfer(lsu, krio_priv);
		break;
	case KEYSTONE_RIO_LSU_CC_XOFF:
	case KEYSTONE_RIO_LSU_CC_ERROR:
	case KEYSTONE_RIO_LSU_CC_INVALID:
	case KEYSTONE_RIO_LSU_CC_DMA:
		res = -EIO;
		keystone_rio_lsu_cancel_transfer(lsu, krio_priv);
		break;
	case KEYSTONE_RIO_LSU_CC_RETRY:
		res = -EBUSY;
		break;
	case KEYSTONE_RIO_LSU_CC_CANCELED:
		res = -EIO;
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
		dev_err(krio_priv->dev,
			"cannot find DMA channel for port %d\n",
			mport->index);
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
static int keystone_rio_lsu_raw_async_transfer(
	struct rio_mport *mport,
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
		dev_err(krio_priv->dev,
			"cannot prepare DMA (raw) transfer\n");
		return -EIO;
	}

	tx = pkt->tx;

	init_completion(&lsu_completion);

	tx->callback = keystone_rio_lsu_dma_callback;
	tx->callback_param = &lsu_completion;

	cookie = dmaengine_submit(tx);

	if (dma_submit_error(cookie)) {
		dev_warn(krio_priv->dev,
			 "failed to submit LSU transfer for dma: %d\n",
			 cookie);
		return -EBUSY;
	}

	/* Wait for transfer to complete */
	ret = wait_for_completion_interruptible_timeout(
		&lsu_completion,
		msecs_to_jiffies(KEYSTONE_RIO_TIMEOUT_MSEC));

	status = dma_async_is_tx_complete(dma_chan, cookie, NULL, NULL);
	if (status != DMA_SUCCESS)
		res = -EIO;

	/* case of transfer timeout */
	if (ret == 0) {
		dev_dbg(krio_priv->dev,
			"transfer incomplete (timeout) for dma channel 0x%x\n",
			(u32) dma_chan);
		res = -ETIMEDOUT;
	}

	/* Interrupted transfer */
	if (ret == -ERESTARTSYS)
		res = -ERESTARTSYS;

	if (ret <= 0) {
		/* Need to cancel current pending transfers in case of error */
		dmaengine_device_control(dma_chan,
					 DMA_TERMINATE_ALL,
					 0);
	}

	return res;
}
#endif /* CONFIG_RAPIDIO_DMA_ENGINE */

/*------------------------------ Doorbell management --------------------------*/

static inline int dbell_get(u32 *pending)
{
	if (*pending) {
		int n = __ffs(*pending);
		*pending &= ~BIT(n);
		return n;
	} else
		return -1;
}

static inline void dbell_call_handler(u32 dbell_num,
				      struct keystone_rio_data *krio_priv)
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
					    -1, /* unknown source Id */
					    mport->host_deviceid,
					    dbell_num);
				break;
			}
		}
	}

	if (!found)
		dev_dbg(krio_priv->dev,
			"DBELL: received spurious doorbell %d\n",
			dbell_num);
}

static void dbell_handler(struct keystone_rio_data *krio_priv)
{
	u32 pending_dbell;
	unsigned int i;

	for (i = 0; i < KEYSTONE_RIO_DBELL_NUMBER; i++) {
		pending_dbell =
			__raw_readl(&(krio_priv->regs->doorbell_int[i].status));

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
	pkt.packet_type =
		((info & 0xffff) << 16) | KEYSTONE_RIO_PACKET_TYPE_DBELL;
	pkt.tx          = NULL;

	return keystone_rio_lsu_raw_async_transfer(mport,
						   &pkt,
						   DMA_MEM_TO_DEV);

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

	res = keystone_rio_lsu_start_transfer(krio_priv->lsu_maint,
					      port_id,
					      dest_id,
					      buff,
					      offset,
					      buff_len,
					      size,
					      type,
					      &lsu_context,
					      0,
					      krio_priv);
	if (res) {
		mutex_unlock(&krio_priv->lsu_lock_maint);
		dev_err(krio_priv->dev, "maintenance packet transfer error\n");
		return res;
	}

	while (1) {
		res = keystone_rio_lsu_complete_transfer(krio_priv->lsu_maint,
							 lsu_context,
							 krio_priv);
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

	res = keystone_rio_maint_request(port_id,
					 destid,
					 hopcount,
					 offset,
					 dma,
					 len,
					 size,
					 KEYSTONE_RIO_PACKET_TYPE_MAINT_R,
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

	res = keystone_rio_maint_request(port_id,
					 destid,
					 hopcount,
					 offset,
					 dma,
					 len,
					 size,
					 KEYSTONE_RIO_PACKET_TYPE_MAINT_W,
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
		ports &= ~BIT(port);

		if (keystone_lane_configs[path_mode][port].start == -1)
			return -1;

		for (lane = keystone_lane_configs[path_mode][port].start;
		     lane < keystone_lane_configs[path_mode][port].end;
		     lane++) {
			lanes |= KEYSTONE_SERDES_LANE(lane);
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
 * @init_rx: if non null, lanes Rx coefficients will be applied
 *
 * Returns %0 on success or %1 if lane is not OK during the expected timeout
 */
static int keystone_rio_lanes_init_and_wait(u32 port, int start,
					    struct keystone_rio_data *krio_priv)
{
	u32 path_mode = krio_priv->board_rio_cfg.path_mode;
	int lanes     = keystone_rio_get_lane_config(BIT(port), path_mode);
	int res;

	dev_dbg(krio_priv->dev,
		"initializing lane mask 0x%x for port %d",
		lanes, port);

	/* Eventually start the lane */
	if (start) {
		dev_dbg(krio_priv->dev,
			"starting lane mask 0x%x for port %d",
			lanes, port);

		krio_priv->serdes.ops->start_tx_lanes((u32) lanes,
						      &krio_priv->serdes);
	}

	/* Wait lanes to be OK */
	res = krio_priv->serdes.ops->wait_lanes_ok(lanes,
						   &krio_priv->serdes);
	if (res < 0) {
		dev_dbg(krio_priv->dev,
			"port %d lane mask 0x%x is not OK\n",
			port, lanes);

		return 1;
	}

	return 0;
}

/*
 * SerDes main configuration
 */
static int keystone_rio_serdes_init(u32 baud,
				    int calibrate,
				    struct keystone_rio_data *krio_priv)
{
	u32 path_mode = krio_priv->board_rio_cfg.path_mode;
	u32 ports = krio_priv->board_rio_cfg.ports;
	u32 lanes;
	int res;

	/* Retrieve lane termination */
	res = keystone_rio_get_lane_config(ports, path_mode);
	if (res <= 0)
		return res;

	/* Initialize SerDes */
	lanes = (u32) res;
	krio_priv->board_rio_cfg.lanes = lanes;

	res = krio_priv->serdes.ops->config_lanes(lanes,
						  baud,
						  &krio_priv->serdes);
	if (res < 0)
		return res;

	/* Check if we need to perform SerDes calibration */
	if ((calibrate) && (!krio_priv->calibrating)) {

		krio_priv->calibrating = 1;

		/* Set calibration timeout */
		krio_priv->board_rio_cfg.serdes_config.cal_timeout =
			krio_priv->board_rio_cfg.port_register_timeout;

		/* Do the calibration */
		krio_priv->serdes.ops->calibrate_lanes(
			lanes,
			&krio_priv->serdes);

		krio_priv->calibrating = 0;
	}

	return 0;
}

/**
 * keystone_rio_hw_init - Configure a RapidIO controller
 * @baud: serdes baudrate
 *
 * Returns %0 on success or %-EINVAL or %-EIO on failure.
 */
static int keystone_rio_hw_init(u32 baud, struct keystone_rio_data *krio_priv)
{
	struct keystone_serdes_config *serdes_config
		= &(krio_priv->board_rio_cfg.serdes_config);
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
		while (__raw_readl(&(krio_priv->regs->blk[block].status)) & 0x1)
			usleep_range(10, 50);
	}

	ndelay(KEYSTONE_RIO_TIMEOUT_NSEC);

	/* Set SRIO out of reset */
	__raw_writel((KEYSTONE_RIO_PER_RESTORE | KEYSTONE_RIO_PER_FREE),
		     &krio_priv->regs->pcr);

	/* Clear BOOT_COMPLETE bit (allowing write) */
	__raw_writel(0x00000000, &krio_priv->regs->per_set_cntl);

	/* Set LSU timeout count to zero */
	__raw_writel(0x000000ff, &krio_priv->regs->lsu_setup_reg[1]);

	/* Enable blocks */
	__raw_writel(1, &krio_priv->regs->gbl_en);
	for (block = 0; block < KEYSTONE_RIO_BLK_NUM; block++)
		__raw_writel(1, &(krio_priv->regs->blk[block].enable));

	/* Set control register 1 configuration */
	__raw_writel(0x00000000, &krio_priv->regs->per_set_cntl1);

	/* Set control register */
	__raw_writel(0x0009c000 | (krio_priv->board_rio_cfg.pkt_forwarding ?
				   BIT(21) | BIT(8) : 0),
		     &krio_priv->regs->per_set_cntl);

	/* Initialize SerDes and eventually perform their calibration */
	res = keystone_rio_serdes_init(
		baud,
		krio_priv->board_rio_cfg.serdes_calibration,
		krio_priv);

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
	__raw_writel(krio_priv->base_dev_id,
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
	__raw_writel(0x00000000, &krio_priv->link_regs->local_err_det);

	/* Set error detection */
	if (krio_priv->board_rio_cfg.pkt_forwarding) {
		/* Disable all error detection if using packet forwarding */
		__raw_writel(0x00000000, &krio_priv->link_regs->local_err_en);
		__raw_writel(0x00000000, &krio_priv->err_mgmt_regs->err_en);
	} else {
		/* Enable logical layer error detection */
		__raw_writel(BIT(24) | BIT(25) | BIT(31),
			     &krio_priv->err_mgmt_regs->err_en);
		__raw_writel(BIT(22) | BIT(26),
			     &krio_priv->link_regs->local_err_en);
	}

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
		__raw_writel(0x41004141,
			     &krio_priv->phy_regs->phy_sp[port].__rsvd[3]);

		/* Set the baud rate to the port information */
		val = __raw_readl(&krio_priv->serial_port_regs->sp[port].ctl2);
		val |= BIT(24 - (baud << 1));
		__raw_writel(val, &krio_priv->serial_port_regs->sp[port].ctl2);
	}

	/* Disable LSU to perform LSU configuration */
	__raw_writel(0,
		     &(krio_priv->regs->blk[KEYSTONE_RIO_BLK_LSU_ID].enable));
	while (__raw_readl(
		       &(krio_priv->regs->blk[KEYSTONE_RIO_BLK_LSU_ID].status))
	       & 0x1)
		usleep_range(10, 50);

	/* Set the SRIO shadow registers configuration to 4/4/4/4 */
	__raw_writel(0x00000000, &krio_priv->regs->lsu_setup_reg[0]);

	/* Use LSU completion interrupt per LSU (not per SRCID) */
	for (i = krio_priv->lsu_start; i <= krio_priv->lsu_end; i++)
		lsu_mask |= (1 << i);

	__raw_writel(lsu_mask, &krio_priv->regs->lsu_setup_reg[1]);

	/* Enable LSU */
	__raw_writel(1,
		     &(krio_priv->regs->blk[KEYSTONE_RIO_BLK_LSU_ID].enable));

	/* Global port-write generation */
	if (krio_priv->board_rio_cfg.pkt_forwarding) {
		/*
		 * Disable generation of port-write requests if using
		 * packet forwarding
		 */
		val = __raw_readl(
			&(krio_priv->evt_mgmt_regs->evt_mgmt_port_wr_enable));
		__raw_writel(
			val & ~(BIT(8) | BIT(28)), /* LOG | LOCALOG */
			&(krio_priv->evt_mgmt_regs->evt_mgmt_port_wr_enable));

		val = __raw_readl(
			&(krio_priv->evt_mgmt_regs->evt_mgmt_dev_port_wr_en));
		__raw_writel(
			val & ~(BIT(0)), /* PW_EN */
			&(krio_priv->evt_mgmt_regs->evt_mgmt_dev_port_wr_en));
	} else {
		/*
		 * Enable generation of port-write requests
		 */
		val = __raw_readl(
			&(krio_priv->evt_mgmt_regs->evt_mgmt_port_wr_enable));
		__raw_writel(
			val | (BIT(8) | BIT(28)), /* LOG | LOCALOG */
			&(krio_priv->evt_mgmt_regs->evt_mgmt_port_wr_enable));

		val = __raw_readl(
			&(krio_priv->evt_mgmt_regs->evt_mgmt_dev_port_wr_en));
		__raw_writel(
			val  | BIT(0), /* PW_EN */
			&(krio_priv->evt_mgmt_regs->evt_mgmt_dev_port_wr_en));
	}

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

static void keystone_rio_reset_dpc(struct work_struct *work)
{
	struct keystone_rio_data *krio_priv =
		container_of(work, struct keystone_rio_data, reset_work);
	u32 ports_rst;
	u32 ports;
	u32 port;

	if (krio_priv->started == 0)
		return;

	ports_rst = __raw_readl(
		&krio_priv->evt_mgmt_regs->evt_mgmt_rst_port_stat);

	dev_dbg(krio_priv->dev,
		"reset device request received on ports: 0x%x\n", ports_rst);

	/* Acknowledge reset */
	ports = ports_rst;
	while (ports) {
		port = __ffs(ports);
		ports &= ~BIT(port);
		__raw_writel(KEYSTONE_RIO_PORT_PLM_STATUS_RST_REQ,
			     &krio_priv->phy_regs->phy_sp[port].status);
	}

	__raw_writel(ports_rst,
		     &krio_priv->evt_mgmt_regs->evt_mgmt_rst_port_stat);

	/* Reinitialize SRIO peripheral */
	keystone_rio_shutdown_controller(krio_priv);
	keystone_rio_setup_controller(krio_priv);
}

/**
 * keystone_rio_stop - Stop RapidIO controller
 */
static void keystone_rio_stop(struct keystone_rio_data *krio_priv)
{
	u32 val;

	/* Disable PEREN bit to stop all new logical layer transactions */
	val = __raw_readl(&krio_priv->regs->pcr);
	__raw_writel(val & ~KEYSTONE_RIO_PER_EN, &krio_priv->regs->pcr);

	/* Clear BOOT_COMPLETE bit */
	val = __raw_readl(&krio_priv->regs->per_set_cntl);
	__raw_writel(val & ~KEYSTONE_RIO_BOOT_COMPLETE,
		     &krio_priv->regs->per_set_cntl);
}

static void keystone_rio_handle_logical_error(
	struct keystone_rio_data *krio_priv)
{
	u32 err_det = __raw_readl(&krio_priv->err_mgmt_regs->err_det);

	while (err_det) {
		u32 err = __ffs(err_det);
		u32 val;

		err_det &= ~BIT(err);

		/* Acknowledge logical layer error */
		val = __raw_readl(&krio_priv->err_mgmt_regs->err_det);
		__raw_writel(val & ~BIT(err),
			     &krio_priv->err_mgmt_regs->err_det);

		dev_dbg(krio_priv->dev,
			"logical layer error %d detected\n", err);

		/* Acknowledge local logical layer error as well if needed */
		if ((err == 22) || (err == 26)) {
			val = __raw_readl(&krio_priv->link_regs->local_err_det);
			__raw_writel(val & ~BIT(err),
				     &krio_priv->link_regs->local_err_det);
		}
	}
}

static int keystone_rio_get_remote_port(
	u8 port,
	struct keystone_rio_data *krio_priv)
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

static int keystone_rio_port_sync_ackid(u32 port,
					struct keystone_rio_data *krio_priv)
{
	u32 lm_resp;
	u32 ackid_stat;
	u32 l_ackid;
	u32 r_ackid;
	int i = 0;

	/*
	 * Clear valid bit in maintenance response register.
	 * Send both Input-Status Link-Request and PNA control symbols and
	 * wait for valid maintenance response
	 */
	__raw_readl(&krio_priv->serial_port_regs->sp[port].link_maint_resp);
	__raw_writel(0x2003f044,
		     &krio_priv->phy_regs->phy_sp[port].long_cs_tx1);

	do {
		if (++i > KEYSTONE_RIO_TIMEOUT_CNT) {
			dev_err(krio_priv->dev,
				"port %d: Input-Status response timeout\n",
				port);
			return -1;
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

	return (int) l_ackid;
}

static int keystone_rio_port_error_recovery(u32 port, struct keystone_rio_data *krio_priv)
{
	int res;
	u32 err_stat;
	u32 err_det;
	u32 plm_status;
	int r_port = krio_priv->board_rio_cfg.ports_remote[port];

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
		u32 ackid_stat;
		u32 l_ackid;
		u32 r_ackid;

		/* Sync ackID */
		res = keystone_rio_port_sync_ackid(port, krio_priv);
		if (res == -1)
			goto oes_rd_err;

		l_ackid = (u32) res;

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

	if (krio_priv->started == 0)
		return;

	dev_dbg(krio_priv->dev,
		"ER errors on ports: 0x%x\n",
		krio_priv->pe_ports);

	for (port = 0; port < KEYSTONE_RIO_MAX_PORT; port++) {

		/* Skip port if we are currently registering it */
		if (krio_priv->ports_registering & BIT(port))
			continue;

		if (test_and_clear_bit(port, (void *)&krio_priv->pe_ports)) {
			/* Wait lanes to be OK */
			if (keystone_rio_lanes_init_and_wait(port, 0,
							     krio_priv)) {
				dev_dbg(krio_priv->dev,
					"ER port %d: lanes are not OK\n", port);
				krio_priv->pe_ports |= BIT(port);
			}

			/*  Recover from port error state */
			if (keystone_rio_port_error_recovery(port,
							     krio_priv)) {
				dev_dbg(krio_priv->dev,
					"ER port %d: failed to perform error"
					" recovery\n",
					port);
				krio_priv->pe_ports |= BIT(port);
			}
		}
	}

	/* If error recovery failed delay another one if there is time left */
	if (krio_priv->pe_ports) {
		if (krio_priv->pe_cnt-- > 1) {
			schedule_delayed_work(
				&krio_priv->pe_work,
				KEYSTONE_RIO_REGISTER_DELAY);
		} else {
			dev_err(krio_priv->dev,
				"ER port %d: failed to recover from "
				"errors\n",
				port);
		}
	}
}

/**
 * keystone_rio_port_status - Return if the port is OK or not
 * @port: index of the port
 *
 * Return %0 if the port is ready or %-EIO on failure.
 */
static int keystone_rio_port_status(int port,
				    struct keystone_rio_data *krio_priv)
{
	u32 path_mode = krio_priv->board_rio_cfg.path_mode;
	int lanes     = keystone_rio_get_lane_config(BIT(port), path_mode);
	u32 count     = 0;
	int res       = 0;
	int solid_ok  = 0;
	u32 value;

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
				goto port_phy_error;
			}
			solid_ok = 0;
		}
		usleep_range(10, 50);
	}

	if (solid_ok == 100) {
		/* Sync ackID */
		res = keystone_rio_port_sync_ackid(port, krio_priv);
		if (res == -1)
			goto port_error;

		/* Check if we need to retrieve the corresponding remote port */
		if (krio_priv->board_rio_cfg.ports_remote[port] < 0) {
			int rport;

			rport = keystone_rio_get_remote_port(port, krio_priv);
			if (rport < 0) {
				dev_warn(krio_priv->dev,
					 "cannot retrieve remote port on port %d\n",
					 port);
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

port_phy_error:
	dev_dbg(krio_priv->dev,
		"fix unstable lane mask 0x%x for port %d\n",
		lanes, port);

	krio_priv->serdes.ops->fix_unstable_lanes(lanes,
						  &krio_priv->serdes);
port_error:
	return -EIO;
}

/**
 * keystone_rio_port_disable - Disable a RapidIO port
 * @port: index of the port to configure
 */
static void keystone_rio_port_disable(
	u32 port,
	struct keystone_rio_data *krio_priv)
{
	/* Disable port */
	__raw_writel(0x800000, &(krio_priv->serial_port_regs->sp[port].ctl));
}

/**
 * keystone_rio_port_enable - Enable a RapidIO port
 * @port: index of the port to configure
 */
static void keystone_rio_port_enable(
	u32 port,
	struct keystone_rio_data *krio_priv)
{
	/* Enable port in input and output */
	__raw_writel(0x600000, &(krio_priv->serial_port_regs->sp[port].ctl));
}

/**
 * keystone_rio_port_init - Configure a RapidIO port
 * @port: index of the port to configure
 * @mode: serdes configuration
 */
static int keystone_rio_port_init(
	u32 port,
	u32 path_mode,
	struct keystone_rio_data *krio_priv)
{
	u32 val;

	if (unlikely(port >= KEYSTONE_RIO_MAX_PORT))
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

	/* Program channel allocation to ports (1x, 2x or 4x) */
	__raw_writel(path_mode, &(krio_priv->phy_regs->phy_sp[port].path_ctl));

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
	__raw_writel(val | BIT(port),
		     &(krio_priv->evt_mgmt_regs->evt_mgmt_rst_int_en));

	/* Enable all PLM interrupts */
	__raw_writel(0xffffffff,
		     &(krio_priv->phy_regs->phy_sp[port].int_enable));
	__raw_writel(1, &(krio_priv->phy_regs->phy_sp[port].all_int_en));

	/* Set unicast mode */
	__raw_writel(0x00109000,
		     &(krio_priv->transport_regs->transport_sp[port].control));

	return 0;
}

/**
 * keystone_rio_port_set_routing - Configure routing for a RapidIO port
 * @port: index of the port to configure
 */
static void keystone_rio_port_set_routing(
	u32 port,
	struct keystone_rio_data *krio_priv)
{
	struct keystone_rio_transport_layer_regs *t = krio_priv->transport_regs;
	u32 base_dev_id = krio_priv->board_rio_cfg.size ?
		__raw_readl(&krio_priv->car_csr_regs->base_dev_id) & 0xffff :
		(__raw_readl(&krio_priv->car_csr_regs->base_dev_id) >> 16)
		& 0xff;
	u32 brr = KEYSTONE_RIO_PKT_FW_BRR_NUM;

	/*
	 * Configure the Base Routing Register (BRR) to ensure that all packets
	 * matching our DevId are admitted.
	 */
	__raw_writel((base_dev_id << 16) |
		     (krio_priv->board_rio_cfg.size ? 0xffff : 0xff),
		     &(t->transport_sp[port].base_route[brr].pattern_match));

	dev_dbg(krio_priv->dev, "pattern_match = 0x%x for BRR %d\n",
		__raw_readl(
			&(t->transport_sp[port].base_route[brr].pattern_match)),
		brr);

	/* Enable routing to LLM for this BRR and port */
	__raw_writel(0x84000000,
		     &(t->transport_sp[port].base_route[brr].ctl));

	/* Use next BRR */
	brr += 1;

	/*
	 * Configure the Base Routing Register (BRR) to ensure that all
	 * broadcast packets are admitted as well.
	 */
	__raw_writel((0xffff << 16) |
		     (krio_priv->board_rio_cfg.size ? 0xffff : 0xff),
		     &(t->transport_sp[port].base_route[brr].pattern_match));

	dev_dbg(krio_priv->dev, "pattern_match = 0x%x for BRR %d\n",
		__raw_readl(
			&(t->transport_sp[port].base_route[brr].pattern_match)),
		brr);

	/* Enable routing to LLM for this BRR and port */
	__raw_writel(0x84000000,
		     &(t->transport_sp[port].base_route[brr].ctl));

	/* Set multicast and packet forwarding mode */
	__raw_writel(0x00209000,
		     &(t->transport_sp[port].control));
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

	if (len != sizeof(u32))
		return -EINVAL; /* only 32-bit access is supported */

	if ((offset + len) > (krio_priv->board_rio_cfg.rio_regs_size
			      - KEYSTONE_RIO_CAR_CSR_REGS))
		return -EINVAL; /* only within the RIO regs range */

	/*
	 * Workaround for rionet: the processing element features must content
	 * RIO_PEF_INB_MBOX and RIO_PEF_INB_DOORBELL bits that cannot be set on
	 * KeyStone hardware. So cheat the read value in this case...
	 */
	if (unlikely(offset == RIO_PEF_CAR))
		*data = krio_priv->rio_pe_feat;
	else
		*data = __raw_readl((void __iomem *)
				    krio_priv->car_csr_regs + offset);

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

	if (len != sizeof(u32))
		return -EINVAL; /* only 32-bit access is supported */

	if ((offset + len) > (krio_priv->board_rio_cfg.rio_regs_size
			      - KEYSTONE_RIO_CAR_CSR_REGS))
		return -EINVAL; /* only within the RIO regs range */

	dev_dbg(krio_priv->dev,
		"local_conf_w: index %d offset 0x%x data 0x%x\n",
		index, offset, data);

	__raw_writel(data, (void __iomem *)krio_priv->car_csr_regs + offset);

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

/*----------------------------- Port-Write management ------------------------*/

static int keystone_rio_port_write_enable(struct keystone_rio_data *krio_priv,
					  u32 port,
					  int enable)
{
	u32 val;

	/* Clear port-write reception capture */
	__raw_writel(0, &(krio_priv->port_write_regs->port_wr_rx_capt[port]));

	if (enable) {
		/*
		 * Enable generation of port-write requests
		 */
		__raw_writel(
			BIT(25) | BIT(26) | BIT(28),
			&(krio_priv->phy_regs->phy_sp[port].port_wr_enable));

		val = __raw_readl(
			&(krio_priv->phy_regs->phy_sp[port].all_port_wr_en));

		__raw_writel(
			val | BIT(0), /* PW_EN */
			&(krio_priv->phy_regs->phy_sp[port].all_port_wr_en));
	} else {
		/*
		 * Disable generation of port-write requests
		 */
		__raw_writel(
			0,
			&(krio_priv->phy_regs->phy_sp[port].port_wr_enable));

		val = __raw_readl(
			&(krio_priv->phy_regs->phy_sp[port].all_port_wr_en));

		__raw_writel(
			val & ~(BIT(0)), /* PW_EN */
			&(krio_priv->phy_regs->phy_sp[port].all_port_wr_en));
	}

	return 0;
}

static void keystone_rio_pw_dpc(struct work_struct *work)
{
	struct keystone_rio_data *krio_priv =
		container_of(work, struct keystone_rio_data, pw_work);
	unsigned long flags;
	u32 msg_buffer[RIO_PW_MSG_SIZE / sizeof(u32)];

	if (krio_priv->started == 0)
		return;

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
	int port;

	for (port = 0; port < KEYSTONE_RIO_MAX_PORT; port++) {
		/* Disabling port write */
		keystone_rio_port_write_enable(krio_priv, port, 0);

		/* Clear port-write-in capture registers */
		__raw_writel(
			0,
			&(krio_priv->port_write_regs->port_wr_rx_capt[port]));
	}

	INIT_WORK(&krio_priv->pw_work, keystone_rio_pw_dpc);
	spin_lock_init(&krio_priv->pw_fifo_lock);

	if (kfifo_alloc(&krio_priv->pw_fifo, RIO_PW_MSG_SIZE * 32, GFP_KERNEL)) {
		dev_err(krio_priv->dev, "FIFO allocation failed\n");
		return -ENOMEM;
	}

	return 0;
}

/**
 * keystone_rio_pw_enable - enable/disable port-write interface init
 * @mport: Master port implementing the port write unit
 * @enable: 1=enable; 0=disable port-write message handling
 */
static int keystone_rio_pw_enable(struct rio_mport *mport, int enable)
{
	return keystone_rio_port_write_enable(mport->priv,
					      mport->index,
					      enable);
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
	/* Supporting DMA but not HW SG mode */
	attr->flags        = RIO_MPORT_DMA;
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

static void keystone_rio_mport_release(struct device *dev)
{
	struct rio_mport *mport = to_rio_mport(dev);

	dev_dbg(dev, "%s %s id=%d\n", __func__, mport->name, mport->id);
}

struct rio_mport *keystone_rio_register_mport(u32 port_id, u32 size,
					      struct keystone_rio_data *krio_priv)
{
	struct rio_ops   *ops;
	struct rio_mport *mport;
	int res;

	ops = kzalloc(sizeof(struct rio_ops), GFP_KERNEL);

	ops->lcread  = keystone_local_config_read;
	ops->lcwrite = keystone_local_config_write;
	ops->cread   = keystone_rio_config_read;
	ops->cwrite  = keystone_rio_config_write;
	ops->dsend   = keystone_rio_dbell_send;

	ops->open_outb_mbox   = keystone_rio_open_outb_mbox;
	ops->close_outb_mbox  = keystone_rio_close_outb_mbox;
	ops->open_inb_mbox    = keystone_rio_open_inb_mbox;
	ops->close_inb_mbox   = keystone_rio_close_inb_mbox;
	ops->add_outb_message = keystone_rio_hw_add_outb_message;
	ops->add_inb_buffer   = keystone_rio_hw_add_inb_buffer;
	ops->get_inb_message  = keystone_rio_hw_get_inb_message;
	ops->query_mport      = keystone_rio_query_mport;
	ops->pwenable	      = keystone_rio_pw_enable;

	mport = kzalloc(sizeof(struct rio_mport), GFP_KERNEL);

	/* Initialize the mport structure */
	res = rio_mport_initialize(mport);
	if (res) {
		kfree(mport);
		return NULL;
	}

	/*
	 * Set the SRIO port physical Id into the index field,
	 * the id field has been set by rio_mport_initialize() to
	 * the logical Id
	 */
	mport->index = port_id;
	mport->priv  = krio_priv;
	mport->dev.parent  = krio_priv->dev;
	mport->dev.release = keystone_rio_mport_release;
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
	rio_init_mbox_res(&mport->riores[RIO_INB_MBOX_RESOURCE], 0,
			  KEYSTONE_RIO_MAX_MBOX);
	rio_init_mbox_res(&mport->riores[RIO_OUTB_MBOX_RESOURCE], 0,
			  KEYSTONE_RIO_MAX_MBOX);

	sprintf(mport->name, "RIO%d mport", port_id);

	mport->ops      = ops;
	mport->sys_size = size;
	mport->phy_type = RIO_PHY_SERIAL;

	/*
	 * Hard coded here because in rio_disc_mport(), it is used in
	 * rio_enum_complete() before it is retrieved in
	 * rio_disc_peer() => rio_setup_device()
	 */
	mport->phys_efptr = 0x100;

	/*
	 * Register the new mport
	 */
	res = rio_register_mport(mport);
	if (res) {
		kfree(mport);
		return NULL;
	}

	krio_priv->mport[port_id] = mport;

#ifdef CONFIG_RAPIDIO_DMA_ENGINE
	/*
	 * Register the DMA engine for DirectIO transfers
	 */
	keystone_rio_dma_register(mport,
				  krio_priv->board_rio_cfg.dma_channel_num);
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
	struct keystone_rio_rx_chan_info *krx_chan =
	    &(krio_priv->rx_channels[mbox]);
	struct keystone_rio_tx_chan_info *ktx_chan =
	    &(krio_priv->tx_channels[mbox]);
	struct device_node *node;
	char node_name[24];

	snprintf(node_name, sizeof(node_name), "mbox-%d", mbox);
	node = of_get_child_by_name(node_rio, node_name);
	if (!node) {
		dev_err(krio_priv->dev, "could not find %s node\n", node_name);
		return -ENODEV;
	}

	dev_dbg(krio_priv->dev, "using node \"%s\"\n", node_name);

	/* DMA rx chan config */
	if (of_property_read_string(node, "rx_channel", &krx_chan->name) < 0) {
		dev_err(krio_priv->dev, "missing \"rx_channel\" parameter\n");
		of_node_put(node);
		return -ENOENT;
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

	/* DMA tx chan config */
	if (of_property_read_string(node, "tx_channel", &ktx_chan->name) < 0) {
		dev_err(krio_priv->dev, "missing \"tx_channel\" parameter\n");
		of_node_put(node);
		return -ENOENT;
	}

	if (of_property_read_u32(node, "tx_queue_depth",
				 &ktx_chan->queue_depth) < 0) {
		dev_warn(krio_priv->dev,
			 "missing \"tx_queue_depth\" parameter\n");
		ktx_chan->queue_depth = 128;
	}

	of_node_put(node);

	return 0;
}

static int keystone_rio_get_controller_defaults(
	struct device_node *node,
	struct keystone_rio_data *krio_priv)
{
	struct keystone_rio_board_controller_info *c =
		&krio_priv->board_rio_cfg;
	u32 temp[24];
	int i;
	int mbox;

	/* Get SRIO registers */
	i = of_property_match_string(node, "reg-names", "rio");
	if (i < 0) {
		dev_err(krio_priv->dev,
			"missing reg-names \"boot_config\" parameter\n");
		return -ENOENT;
	}
	if (of_property_read_u32_index(node, "reg", (i << 1), &temp[0])) {
		dev_err(krio_priv->dev, "missing \"reg\" parameters\n");
		return -ENOENT;
	}
	if (of_property_read_u32_index(node, "reg", (i << 1) + 1, &temp[1])) {
		dev_err(krio_priv->dev, "missing \"reg\" parameters\n");
		return -ENOENT;
	}

	c->rio_regs_base = temp[0];
	c->rio_regs_size = temp[1];

	/* Get boot config registers */
	i = of_property_match_string(node, "reg-names", "boot_config");
	if (i < 0) {
		dev_err(krio_priv->dev,
			"missing reg-names \"boot_config\" parameter\n");
		return -ENOENT;
	}
	if (of_property_read_u32_index(node, "reg", (i << 1), &temp[2])) {
		dev_err(krio_priv->dev, "missing \"reg\" parameters\n");
		return -ENOENT;
	}
	if (of_property_read_u32_index(node, "reg", (i << 1) + 1, &temp[3])) {
		dev_err(krio_priv->dev, "missing \"reg\" parameters\n");
		return -ENOENT;
	}

	c->boot_cfg_regs_base = temp[2];
	c->boot_cfg_regs_size = temp[3];

	/* Get SerDes registers */
	i = of_property_match_string(node, "reg-names", "serdes");
	if (i < 0) {
		dev_err(krio_priv->dev,
			"missing reg-names \"serdes\" parameter\n");
		return -ENOENT;
	}
	if (of_property_read_u32_index(node, "reg", (i << 1), &temp[4])) {
		dev_err(krio_priv->dev, "missing \"reg\" parameters\n");
		return -ENOENT;
	}
	if (of_property_read_u32_index(node, "reg", (i << 1) + 1, &temp[5])) {
		dev_err(krio_priv->dev, "missing \"reg\" parameters\n");
		return -ENOENT;
	}

	c->serdes_cfg_regs_base = temp[4];
	c->serdes_cfg_regs_size = temp[5];

	if (of_property_read_u32 (node, "dev-id-size", &c->size))
		dev_warn(krio_priv->dev, "missing \"dev-id-size\" parameter\n");

	if (of_property_read_u32 (node, "ports", &c->ports))
		dev_warn(krio_priv->dev, "missing \"ports\" parameter\n");

	if (of_property_read_u32_array(node, "ports_remote",
				       c->ports_remote, KEYSTONE_RIO_MAX_PORT)) {
		/* Remote ports will be detected during port status */
		for (i = 0; i < KEYSTONE_RIO_MAX_PORT; i++)
			c->ports_remote[i] = -1;
	}

	/* SerDes config */
	if (!of_find_property(node, "keystone2-serdes", NULL)) {
		/* K1 setup*/
		c->serdes_type                     = KEYSTONE_SERDES_TYPE_K1;
		c->serdes_config.prescalar_srv_clk = 0x001e;
		c->serdes_config.do_dfe_cal        = 0; /* no DFE calibration */
		c->path_mode                       = 0x0000;
	} else {
		/* K2 setup*/
		c->serdes_type                     = KEYSTONE_SERDES_TYPE_K2;
		c->serdes_config.prescalar_srv_clk = 0x001f;
		c->serdes_config.do_dfe_cal        = 0; /* no DFE calibration */
		c->path_mode                       = 0x0004;

		if (of_property_read_u32(node, "baudrate", &c->serdes_baudrate)) {
			dev_warn(krio_priv->dev,
				 "missing \"baudrate\" parameter, using 5Gbps\n");
			c->serdes_baudrate = KEYSTONE_SERDES_BAUD_5_000;
		}
	}

	/* Set if performing optional SerDes calibration sequence at boot */
	c->serdes_calibration = serdes_calibration;

	/* SerDes pre-1lsb, c1, c2, cm, att and vreg config */
	if (of_property_read_u32_array(node, "serdes_1sb", &temp[0], 4)) {
		for (i = 0; i < 4; i++)
			c->serdes_config.tx[i].pre_1lsb = 0;
	} else {
		for (i = 0; i < 4; i++)
			c->serdes_config.tx[i].pre_1lsb = temp[i];
	}

	if (of_property_read_u32_array(node, "serdes_c1", &temp[0], 4)) {
		if (c->serdes_baudrate == KEYSTONE_SERDES_BAUD_3_125) {
			for (i = 0; i < 4; i++)
				c->serdes_config.tx[i].c1_coeff = 4;
		} else {
			for (i = 0; i < 4; i++)
				c->serdes_config.tx[i].c1_coeff = 6;
		}
	} else {
		for (i = 0; i < 4; i++)
			c->serdes_config.tx[i].c1_coeff = temp[i];
	}

	if (of_property_read_u32_array(node, "serdes_c2", &temp[0], 4)) {
		for (i = 0; i < 4; i++)
			c->serdes_config.tx[i].c2_coeff = 0;
	} else {
		for (i = 0; i < 4; i++)
			c->serdes_config.tx[i].c2_coeff = temp[i];
	}

	if (of_property_read_u32_array(node, "serdes_cm", &temp[0], 4)) {
		for (i = 0; i < 4; i++)
			c->serdes_config.tx[i].cm_coeff = 0;
	} else {
		for (i = 0; i < 4; i++)
			c->serdes_config.tx[i].cm_coeff = temp[i];
	}

	if (of_property_read_u32_array(node, "serdes_att", &temp[0], 4)) {
		for (i = 0; i < 4; i++)
			c->serdes_config.tx[i].att = 12;
	} else {
		for (i = 0; i < 4; i++)
			c->serdes_config.tx[i].att = temp[i];
	}

	if (of_property_read_u32_array(node, "serdes_vreg", &temp[0], 4)) {
		for (i = 0; i < 4; i++)
			c->serdes_config.tx[i].vreg = 4;
	} else {
		for (i = 0; i < 4; i++)
			c->serdes_config.tx[i].vreg = temp[i];
	}

	if (of_property_read_u32_array(node, "serdes_vdreg", &temp[0], 4)) {
		for (i = 0; i < 4; i++)
			c->serdes_config.tx[i].vdreg = 1;
	} else {
		for (i = 0; i < 4; i++)
			c->serdes_config.tx[i].vdreg = temp[i];
	}

	if (of_property_read_u32_array(node, "serdes_rx_att_start",
				       &temp[0], 4)) {
		for (i = 0; i < 4; i++)
			c->serdes_config.rx[i].start_att = 3;
	} else {
		for (i = 0; i < 4; i++)
			c->serdes_config.rx[i].start_att = temp[i];
	}

	if (of_property_read_u32_array(node, "serdes_rx_boost_start",
				       &temp[0], 4)) {
		for (i = 0; i < 4; i++)
			c->serdes_config.rx[i].start_boost = 3;
	} else {
		for (i = 0; i < 4; i++)
			c->serdes_config.rx[i].start_boost = temp[i];
	}

	if (of_property_read_u32_array(node, "serdes_rx_att", &temp[0], 4)) {
		for (i = 0; i < 4; i++)
			/* Use dynamic Rx calibration */
			c->serdes_config.rx[i].mean_att = -1;
	} else {
		for (i = 0; i < 4; i++)
			c->serdes_config.rx[i].mean_att = temp[i];
	}

	if (of_property_read_u32_array(node, "serdes_rx_boost", &temp[0], 4)) {
		for (i = 0; i < 4; i++)
			/* Use dynamic Rx calibration */
			c->serdes_config.rx[i].mean_boost = -1;
	} else {
		for (i = 0; i < 4; i++)
			c->serdes_config.rx[i].mean_boost = temp[i];
	}

	/* Path mode config (mapping of SerDes lanes to port widths) */
	if (of_property_read_u32(node, "path_mode", &c->path_mode)) {
		dev_warn(krio_priv->dev,
			 "missing \"path_mode\" parameter\n");
	}

	/* Max possible ports configurations per path_mode */
	if ((c->path_mode == 0 &&
	     c->ports & ~KEYSTONE_RIO_MAX_PORTS_PATH_MODE_0) ||
	    (c->path_mode == 1 &&
	     c->ports & ~KEYSTONE_RIO_MAX_PORTS_PATH_MODE_1) ||
	    (c->path_mode == 2 &&
	     c->ports & ~KEYSTONE_RIO_MAX_PORTS_PATH_MODE_2) ||
	    (c->path_mode == 3 &&
	     c->ports & ~KEYSTONE_RIO_MAX_PORTS_PATH_MODE_3) ||
	    (c->path_mode == 4 &&
	     c->ports & ~KEYSTONE_RIO_MAX_PORTS_PATH_MODE_4)) {
		dev_err(krio_priv->dev,
			"\"path_mode\" and \"ports\" configuration mismatch\n");
		return -EINVAL;
	}

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
		dev_warn(krio_priv->dev,
			 "missing \"num-dio-channels\" parameter\n");
		c->dma_channel_num = 8;
	}
#endif

	/* RXU mapping resources */
	if (of_property_read_u32_array(node, "rxu_map_range", &temp[0], 2)) {
		krio_priv->rxu_map_start = KEYSTONE_RIO_RXU_MAP_MIN;
		krio_priv->rxu_map_end   = KEYSTONE_RIO_RXU_MAP_MAX;
	} else if ((temp[1] > KEYSTONE_RIO_RXU_MAP_MAX) ||
		   (temp[0] > temp[1])) {
		dev_err(krio_priv->dev,
			"invalid \"rxu_map_range\" parameter\n");
		return -EINVAL;
	} else {
		krio_priv->rxu_map_start = temp[0];
		krio_priv->rxu_map_end   = temp[1];
	}

	dev_dbg(krio_priv->dev, "using RXU map %lu - %lu range\n",
		krio_priv->rxu_map_start, krio_priv->rxu_map_end);

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

	/* Retrieve the per-mailboxes properties */
	for (mbox = 0; mbox < krio_priv->num_mboxes; mbox++) {
		int res;

		res = keystone_rio_get_mbox_defaults(mbox, node, krio_priv);
		if (res)
			return res;
	}

	/* Interrupt config */
	c->rio_irq = irq_of_parse_and_map(node, 0);
	if (c->rio_irq < 0) {
		dev_err(krio_priv->dev, "missing \"rio_irq\" parameter\n");
		return -ENOENT;
	}

	c->lsu_irq = irq_of_parse_and_map(node, 1);
	if (c->lsu_irq < 0) {
		dev_err(krio_priv->dev, "missing \"lsu_irq\" parameter\n");
		return -ENOENT;
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

	return 0;
}

static int keystone_rio_port_chk(struct keystone_rio_data *krio_priv, int init)
{
	u32 ports = krio_priv->ports_registering;
	u32 size  = krio_priv->board_rio_cfg.size;
	struct rio_mport *mport;

	while (ports) {
		int status;
		u32 port = __ffs(ports);
		ports &= ~BIT(port);

		/* Eventually start lanes and wait them to be OK and with SD */
		if (keystone_rio_lanes_init_and_wait(port, init, krio_priv))
			continue;

		/*
		 * Check the port status here before calling the generic RapidIO
		 * layer. Port status check is done in rio_mport_is_active() as
		 * well but we need to do it our way first due to some delays in
		 * hw initialization.
		 */
		status = keystone_rio_port_status(port, krio_priv);
		if (status == 0) {
			unsigned long flags;

			/*
			 * The link has been established from an hw standpoint
			 * so do not try to check the port again.
			 * Only mport registration may fail now.
			 */
			spin_lock_irqsave(&krio_priv->port_chk_lock, flags);
			krio_priv->ports |= BIT(port);
			krio_priv->ports_registering &= ~BIT(port);
			spin_unlock_irqrestore(&krio_priv->port_chk_lock,
					       flags);

			/* Register mport only if this is initial port check */
			if (!krio_priv->mport[port]) {
				mport = keystone_rio_register_mport(
					port, size, krio_priv);

				if (!mport) {
					dev_err(krio_priv->dev,
						"failed to register mport %d\n",
						port);
					return -1;
				}

				dev_info(krio_priv->dev,
					 "port RIO%d host_deviceid %d registered\n",
					 port, mport->host_deviceid);
			} else {
				dev_info(krio_priv->dev,
					 "port RIO%d host_deviceid %d ready\n",
					 port,
					 krio_priv->mport[port]->host_deviceid);
			}

			/*
			 * Update routing after discovery/enumeration
			 * with new dev id
			 */
			if (krio_priv->board_rio_cfg.pkt_forwarding)
				keystone_rio_port_set_routing(port, krio_priv);

			/* Save the current base dev Id */
			krio_priv->base_dev_id = __raw_readl(
			    &krio_priv->car_csr_regs->base_dev_id);
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

	res = keystone_rio_port_chk(krio_priv, 0);
	if (res) {
		unsigned long flags;

		if (res == -1)
			return;

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

	if (ports > (BIT(KEYSTONE_RIO_MAX_PORT) - 1))
		return -EINVAL;

	/*
	 * Only the ports defined in DTS can be rescanned because SerDes
	 * initialization is not restarted here, only link status check.
	 */
	ports &= krio_priv->board_rio_cfg.ports;

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

static ssize_t keystone_rio_start_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t count)
{
	struct keystone_rio_data *krio_priv = (struct keystone_rio_data *)
		dev->platform_data;
	long unsigned int new_start;

	if (kstrtoul(buf, 0, &new_start))
		return -EINVAL;

	/* Start SRIO peripheral if not started */
	if ((new_start) && (krio_priv->started == 0)) {
		keystone_rio_setup_controller(krio_priv);
		return count;
	}

	/* Stop SRIO peripheral if started */
	if ((new_start == 0) && (krio_priv->started == 1)) {
		keystone_rio_shutdown_controller(krio_priv);
		return count;
	}

	return count;
}

static ssize_t keystone_rio_start_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct keystone_rio_data *krio_priv = (struct keystone_rio_data *)
		dev->platform_data;

	if (krio_priv == NULL)
		return -EINVAL;

	return scnprintf(buf, PAGE_SIZE, "%d\n", krio_priv->started);
}

static ssize_t keystone_rio_calibrate_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf,
					    size_t count)
{
	struct keystone_rio_data *krio_priv = (struct keystone_rio_data *)
		dev->platform_data;
	long unsigned int new_calibrate;

	if (kstrtoul(buf, 0, &new_calibrate))
		return -EINVAL;

	/* Start SRIO calibration */
	if ((new_calibrate) && (krio_priv->started == 0)) {
		int res;
		u32 block;

		/* Enable RIO SerDes blocks */
		__raw_writel(1, &krio_priv->regs->gbl_en);
		for (block = KEYSTONE_RIO_BLK_PORT0_ID;
		     block <= KEYSTONE_RIO_BLK_PORT3_ID;
		     block++)
			__raw_writel(1, &(krio_priv->regs->blk[block].enable));

		/* Do SerDes initialization and calibration */
		res = keystone_rio_serdes_init(
			krio_priv->board_rio_cfg.serdes_baudrate,
			1,
			krio_priv);

		if (res < 0)
			dev_err(krio_priv->dev,
				"calibration of SerDes failed\n");
	}

	return count;
}

static ssize_t keystone_rio_calibrate_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct keystone_rio_data *krio_priv = (struct keystone_rio_data *)
		dev->platform_data;

	if (krio_priv == NULL)
		return -EINVAL;

	return scnprintf(buf, PAGE_SIZE, "%d\n", krio_priv->calibrating);
}

static DEVICE_ATTR(ports,
		   S_IRUGO | S_IWUSR,
		   keystone_rio_ports_show,
		   keystone_rio_ports_store);

static DEVICE_ATTR(start,
		   S_IRUGO | S_IWUSR,
		   keystone_rio_start_show,
		   keystone_rio_start_store);

static DEVICE_ATTR(calibrate,
		   S_IRUGO | S_IWUSR,
		   keystone_rio_calibrate_show,
		   keystone_rio_calibrate_store);

static void keystone_rio_sysfs_remove(struct device *dev)
{
	device_remove_file(dev, &dev_attr_ports);
	device_remove_file(dev, &dev_attr_start);
	device_remove_file(dev, &dev_attr_calibrate);
}

static int keystone_rio_sysfs_create(struct device *dev)
{
	int res = 0;

	res = device_create_file(dev, &dev_attr_ports);
	if (res) {
		dev_err(dev, "unable create sysfs ports file\n");
		return res;
	}

	res = device_create_file(dev, &dev_attr_start);
	if (res) {
		dev_err(dev, "unable create sysfs start file\n");
		return res;
	}

	res = device_create_file(dev, &dev_attr_calibrate);
	if (res)
		dev_err(dev, "unable create sysfs calibrate file\n");

	return res;
}

/*
 * Platform configuration setup
 */
static int keystone_rio_setup_controller(struct keystone_rio_data *krio_priv)
{
	u32 ports;
	u32 p;
	u32 baud;
	u32 path_mode;
	u32 size = 0;
	int res = 0;
	char str[8];
	unsigned long flags;

	size      = krio_priv->board_rio_cfg.size;
	ports     = krio_priv->board_rio_cfg.ports;
	baud      = krio_priv->board_rio_cfg.serdes_baudrate;
	path_mode = krio_priv->board_rio_cfg.path_mode;

	krio_priv->started = 1;

	dev_dbg(krio_priv->dev, "size = %d, ports = 0x%x, baud = %d, path_mode = %d\n",
		size, ports, baud, path_mode);

	if (baud > KEYSTONE_SERDES_BAUD_5_000) {
		baud = KEYSTONE_SERDES_BAUD_5_000;
		dev_warn(krio_priv->dev,
			 "invalid baud rate, forcing it to 5Gbps\n");
	}

	switch (baud) {
	case KEYSTONE_SERDES_BAUD_1_250:
		snprintf(str, sizeof(str), "1.25");
		break;
	case KEYSTONE_SERDES_BAUD_2_500:
		snprintf(str, sizeof(str), "2.50");
		break;
	case KEYSTONE_SERDES_BAUD_3_125:
		snprintf(str, sizeof(str), "3.125");
		break;
	case KEYSTONE_SERDES_BAUD_5_000:
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

	/* Register all configured ports */
	krio_priv->ports_registering = krio_priv->board_rio_cfg.ports;

	/* Initialize interrupts */
	res = keystone_rio_interrupt_setup(krio_priv);
	if (res)
		return res;

	/* Start the controller */
	keystone_rio_start(krio_priv);

	while (ports) {
		u32 port = __ffs(ports);
		ports &= ~BIT(port);

		res = keystone_rio_port_init(port, path_mode, krio_priv);
		if (res < 0) {
			dev_err(krio_priv->dev,
				"initialization of port %d failed\n", port);
			return res;
		}

		/* Start the port */
		keystone_rio_port_enable(port, krio_priv);
	}

	/* Complete port initialization and wait link */
	res = keystone_rio_port_chk(krio_priv, 1);
	if (res) {
		if (res == -1)
			return -ENOMEM;

		/* If port check failed schedule asynchronous periodic check */
		spin_lock_irqsave(&krio_priv->port_chk_lock, flags);
		krio_priv->port_chk_cnt =
			krio_priv->board_rio_cfg.port_register_timeout /
			(KEYSTONE_RIO_REGISTER_DELAY / HZ);
		spin_unlock_irqrestore(&krio_priv->port_chk_lock, flags);

		schedule_delayed_work(&krio_priv->port_chk_task,
				      KEYSTONE_RIO_REGISTER_DELAY);
	}

	return res;
}

static int keystone_rio_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct keystone_rio_data *krio_priv;
	int res = 0;
	int i;
	u16 serdes_type;
	void __iomem *regs;

	dev_info(&pdev->dev, "KeyStone RapidIO driver %s\n", DRIVER_VER);

	if (!node) {
		dev_err(&pdev->dev, "could not find device info\n");
		return -EINVAL;
	}

	krio_priv = devm_kzalloc(&pdev->dev,
				 sizeof(struct keystone_rio_data),
				 GFP_KERNEL);
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

	serdes_type = krio_priv->board_rio_cfg.serdes_type;

	/* SRIO main driver (global ressources) */
	krio_priv->lsu_free = krio_priv->lsu_start;
	krio_priv->lsu_maint = keystone_rio_lsu_alloc(krio_priv);

	mutex_init(&krio_priv->lsu_lock_maint);

	spin_lock_init(&krio_priv->port_chk_lock);

	INIT_DELAYED_WORK(&krio_priv->port_chk_task,
			  keystone_rio_port_chk_task);
	INIT_DELAYED_WORK(&krio_priv->pe_work, keystone_rio_pe_dpc);
	INIT_WORK(&krio_priv->reset_work, keystone_rio_reset_dpc);

#ifdef CONFIG_RAPIDIO_DMA_ENGINE
	for (i = 0; i < KEYSTONE_RIO_LSU_NUM; i++)
		INIT_LIST_HEAD(&krio_priv->dma_channels[i]);
#endif

	/* Initial base dev Id */
	krio_priv->base_dev_id = 0x00ffffff;

	regs = ioremap(krio_priv->board_rio_cfg.boot_cfg_regs_base,
		       krio_priv->board_rio_cfg.boot_cfg_regs_size);

	krio_priv->jtagid_reg = regs + 0x0018;

	if (serdes_type == KEYSTONE_SERDES_TYPE_K1)
		krio_priv->serdes_sts_reg = regs + 0x154;

	regs = ioremap(krio_priv->board_rio_cfg.serdes_cfg_regs_base,
		       krio_priv->board_rio_cfg.serdes_cfg_regs_size);
	krio_priv->serdes_regs = regs;

	regs = ioremap(krio_priv->board_rio_cfg.rio_regs_base,
		       krio_priv->board_rio_cfg.rio_regs_size);
	krio_priv->regs		     = regs;
	krio_priv->car_csr_regs	     = regs + KEYSTONE_RIO_CAR_CSR_REGS;
	krio_priv->serial_port_regs  = regs + KEYSTONE_RIO_SERIAL_PORT_REGS;
	krio_priv->err_mgmt_regs     = regs + KEYSTONE_RIO_ERR_MGMT_REGS;
	krio_priv->phy_regs	     = regs + KEYSTONE_RIO_PHY_REGS;
	krio_priv->transport_regs    = regs + KEYSTONE_RIO_TRANSPORT_REGS;
	krio_priv->pkt_buf_regs	     = regs + KEYSTONE_RIO_PKT_BUF_REGS;
	krio_priv->evt_mgmt_regs     = regs + KEYSTONE_RIO_EVT_MGMT_REGS;
	krio_priv->port_write_regs   = regs + KEYSTONE_RIO_PORT_WRITE_REGS;
	krio_priv->link_regs	     = regs + KEYSTONE_RIO_LINK_REGS;
	krio_priv->fabric_regs	     = regs + KEYSTONE_RIO_FABRIC_REGS;

	/* Register SerDes */
	res = keystone_rio_serdes_register(
		serdes_type,
		krio_priv->serdes_regs,
		krio_priv->serdes_sts_reg,
		&pdev->dev,
		&krio_priv->serdes,
		&krio_priv->board_rio_cfg.serdes_config);

	if (res < 0) {
		dev_err(&pdev->dev, "cannot register SerDes type %d\n",
			serdes_type);
		return -EINVAL;
	}

	dev_info(&pdev->dev, "using K%d SerDes\n",
		 (serdes_type == KEYSTONE_SERDES_TYPE_K2) ? 2 : 1);

	/* Enable SRIO clock */
	krio_priv->clk = clk_get(&pdev->dev, "clk_srio");
	if (IS_ERR(krio_priv->clk)) {
		dev_err(&pdev->dev, "Unable to get Keystone SRIO clock\n");
		return -EBUSY;
	} else {
		/* Workaround for K1 SRIO clocks */
		clk_prepare_enable(krio_priv->clk);
		ndelay(100);
		clk_disable_unprepare(krio_priv->clk);
		ndelay(100);
		clk_prepare_enable(krio_priv->clk);
	}

	pdev->dev.platform_data = (void *) krio_priv;

	keystone_rio_sysfs_create(&pdev->dev);

	/* Setup the SRIO controller */
	if (enable_ports) {
		res = keystone_rio_setup_controller(krio_priv);
		if (res < 0) {
			clk_disable_unprepare(krio_priv->clk);
			clk_put(krio_priv->clk);
			return res;
		}
	}

	return 0;
}

static void keystone_rio_shutdown_controller(
	struct keystone_rio_data *krio_priv)
{
	u32 lanes = krio_priv->board_rio_cfg.lanes;
	int i;

	dev_dbg(krio_priv->dev, "shutdown controller\n");

	/* Unregister interrupt handlers */
	keystone_rio_interrupt_release(krio_priv);

	/* Shutdown associated SerDes */
	krio_priv->serdes.ops->shutdown_lanes(lanes, &krio_priv->serdes);

	/* Stop the hw controller */
	keystone_rio_stop(krio_priv);

	/* Disable blocks */
	__raw_writel(0, &krio_priv->regs->gbl_en);
	for (i = 0; i < KEYSTONE_RIO_BLK_NUM; i++) {
		__raw_writel(0, &(krio_priv->regs->blk[i].enable));
		while (__raw_readl(&(krio_priv->regs->blk[i].status)) & 0x1)
			usleep_range(10, 50);
	}

	krio_priv->started = 0;
}

static void keystone_rio_shutdown(struct platform_device *pdev)
{
	struct keystone_rio_data *krio_priv = platform_get_drvdata(pdev);

	if (krio_priv->started)
		keystone_rio_shutdown_controller(krio_priv);

	/* Wait current DMA transfers to finish */
	mdelay(10);

	if (krio_priv->clk) {
		clk_disable_unprepare(krio_priv->clk);
		clk_put(krio_priv->clk);
	}
}

static int keystone_rio_remove(struct platform_device *pdev)
{
	struct keystone_rio_data *krio_priv = platform_get_drvdata(pdev);
	u32 ports = krio_priv->board_rio_cfg.ports;

	/* Notify devices attached to all registered mports */
	while (ports) {
		struct rio_mport *mport;
		u32 port = __ffs(ports);
		ports &= ~BIT(port);

		mport = krio_priv->mport[port];

		if (mport) {
			/* Remove devices attached to this port */
			rio_mport_remove_childs(mport);
		}
	}

	/* Shutdown the hw controller */
	keystone_rio_shutdown(pdev);

	/* Retrieve all registered mports */
	ports = krio_priv->board_rio_cfg.ports;
	while (ports) {
		struct rio_mport *mport;
		u32 port = __ffs(ports);
		ports &= ~BIT(port);

		mport = krio_priv->mport[port];

		if (mport) {
#ifdef CONFIG_RAPIDIO_DMA_ENGINE
			keystone_rio_lsu_dma_free_channel(mport);
			keystone_rio_dma_unregister(mport);
#endif
			/* Unregister the mport from the RIO framework */
			rio_unregister_mport(mport);
		}
	}

	/* Remove io mapping */
	iounmap(krio_priv->jtagid_reg);
	iounmap(krio_priv->serdes_regs);
	iounmap(krio_priv->regs);

	/* Unregister sysfs and free mport private structures */
	keystone_rio_serdes_unregister(&pdev->dev, &krio_priv->serdes);
	keystone_rio_sysfs_remove(&pdev->dev);
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
	.remove = keystone_rio_remove,
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
