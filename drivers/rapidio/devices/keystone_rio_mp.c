/*
 * Copyright (C) 2014 Texas Instruments Incorporated
 * Authors: Aurelien Jacquiot <a-jacquiot@ti.com>
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
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/keystone-dma.h>
#include <linux/kfifo.h>
#include <linux/rio.h>
#include <linux/rio_drv.h>

#include "keystone_rio_serdes.h"
#include "keystone_rio.h"

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
	struct keystone_rio_rx_chan_info *krx_chan =
		&(krio_priv->rx_channels[mbox]);

	return (u32) ((krx_chan->packet_type != RIO_PACKET_TYPE_STREAM) &&
		      (krx_chan->packet_type != RIO_PACKET_TYPE_MESSAGE)) ?
		RIO_PACKET_TYPE_MESSAGE : krx_chan->packet_type;
}

/*
 * This function retrieves the mapping from Linux RIO mailbox to stream id for
 * type 9 packets
 */
static inline u32 keystone_rio_mbox_to_strmid(
	int mbox,
	struct keystone_rio_data *krio_priv)
{
	struct keystone_rio_rx_chan_info *krx_chan =
		&(krio_priv->rx_channels[mbox]);

	return (u32) krx_chan->stream_id;
}

/*
 * Release a free receive buffer
 */
static void keystone_rio_rxpool_free(void *arg,
				     unsigned q_num,
				     unsigned bufsize,
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
				p_mbox->port->inb_msg[p_mbox->id].mcback(
					p_mbox->port,
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

static void keystone_rio_mp_inb_exit(int mbox,
				     struct keystone_rio_data *krio_priv)
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

static int keystone_rio_mp_inb_init(int mbox,
				    struct keystone_rio_data *krio_priv)
{
	struct keystone_rio_rx_chan_info *krx_chan;
	struct dma_keystone_info config;
	dma_cap_mask_t mask;
	int err = -ENODEV;
	int i;

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);

	/* DMA Rx channel */
	krx_chan = &(krio_priv->rx_channels[mbox]);
	krx_chan->priv = krio_priv;
	krx_chan->chan_num = mbox;
	krx_chan->dma_channel =
		dma_request_channel_by_name(mask, krx_chan->name);
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
			"error configuring Rx channel, err %d\n", err);
		goto fail;
	}

	tasklet_init(&krio_priv->task, keystone_rio_chan_work_handler,
		     (unsigned long) krio_priv);

	dma_set_notify(krx_chan->dma_channel,
		       keystone_rio_rx_notify,
		       krio_priv);

	krx_chan->flow_num = dma_get_rx_flow(krx_chan->dma_channel);
	krx_chan->queue_num = dma_get_rx_queue(krx_chan->dma_channel);

	dev_dbg(krio_priv->dev,
		"opened Rx channel: %p (mbox=%d, flow=%d, rx_q=%d, pkt_type=%d)\n",
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
	unsigned long bit_sz = sizeof(krio_priv->rxu_map_bitmap) * 8;

	id = find_first_zero_bit(&(krio_priv->rxu_map_bitmap[0]), bit_sz);

	while (id < krio_priv->rxu_map_start)
		id = find_next_zero_bit(&(krio_priv->rxu_map_bitmap[0]),
					bit_sz, ++id);

	if (id > krio_priv->rxu_map_end)
		return -1;

	__set_bit(id, &(krio_priv->rxu_map_bitmap[0]));

	return id;
}

static void keystone_rio_free_rxu_map(int id,
				      struct keystone_rio_data *krio_priv)
{
	clear_bit(id, &(krio_priv->rxu_map_bitmap[0]));
}

/**
 * keystone_rio_map_mbox - Map a mailbox to a given queue for both type 11
 * and type 9 packets.
 * @mbox: mailbox to map
 * @queue: associated queue number
 * @flowid: flow Id
 * @size: device Id size
 *
 * Returns %0 on success or %-ENOMEM on failure.
 */
static int keystone_rio_map_mbox(int mbox,
				 int queue,
				 int flowid,
				 int size,
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
		/*
		 * Map the multi-segment mailbox to the corresponding Rx queue
		 * for type 11.
		 * Given mailbox, all letters, srcid = 0
		 */
		mapping_entry_low = ((mbox & 0x1f) << 16) | (0x3f000000);

		/*
		 * Multi-segment messaging and promiscuous (don't care about
		 * src/dst id)
		 */
		mapping_entry_high = KEYSTONE_RIO_MAP_FLAG_SEGMENT
			| KEYSTONE_RIO_MAP_FLAG_SRC_PROMISC
			| KEYSTONE_RIO_MAP_FLAG_DST_PROMISC;
	} else {
		/*
		 * Map the multi-segment mailbox for type 9
		 * accept all COS and srcid = 0, use promiscuous (don't care
		 * about src/dst id)
		 */
		mapping_t9_reg[0] = 0;
		mapping_t9_reg[1] = KEYSTONE_RIO_MAP_FLAG_SRC_PROMISC
			| KEYSTONE_RIO_MAP_FLAG_DST_PROMISC;
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
		"using RXU map %d @ 0x%08x: mbox = %d, flowid = %d, queue = %d pkt_type = %d\n",
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
		 * The RapidIO peripheral looks at the incoming RapidIO msgs
		 * and if there is only one segment (the whole msg fits into
		 * one RapidIO msg), the peripheral uses the single segment
		 * mapping table. Therefore we need to map the single-segment
		 * mailbox too.
		 * The same Rx CPPI Queue is used (as for the multi-segment
		 * mailbox).
		 */
		mapping_entry_high &= ~KEYSTONE_RIO_MAP_FLAG_SEGMENT;

		i = keystone_rio_get_rxu_map(krio_priv);
		if (i < 0)
			return -ENOMEM;

		rx_mbox->rxu_map_id[1] = i;

		dev_dbg(krio_priv->dev,
			"using RXU map %d @ 0x%08x: mbox = %d, flowid = %d, queue = %d pkt_type = %d\n",
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
int keystone_rio_open_inb_mbox(struct rio_mport *mport,
			       void *dev_id,
			       int mbox,
			       int entries)
{
	struct keystone_rio_data *krio_priv = mport->priv;
	struct keystone_rio_mbox_info *rx_mbox = &krio_priv->rx_mbox[mbox];
	struct keystone_rio_rx_chan_info *krx_chan =
		&krio_priv->rx_channels[mbox];
	int res;

	if (mbox >= KEYSTONE_RIO_MAX_MBOX)
		return -EINVAL;

	/*
	 * Check that number of entries is a power of two to ease ring
	 * management
	 */
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

void keystone_rio_close_rx_mbox(int mbox,
				struct keystone_rio_data *krio_priv)
{
	struct keystone_rio_mbox_info *rx_mbox = &krio_priv->rx_mbox[mbox];

	if (mbox >= KEYSTONE_RIO_MAX_MBOX)
		return;

	rx_mbox->running = 0;

	if (!rx_mbox->port)
		return;

	rx_mbox->port = NULL;

	/* Release associated resources */
	keystone_rio_free_rxu_map(rx_mbox->rxu_map_id[0], krio_priv);
	keystone_rio_free_rxu_map(rx_mbox->rxu_map_id[1], krio_priv);

	keystone_rio_mp_inb_exit(mbox, krio_priv);
}

/**
 * keystone_rio_close_inb_mbox - Shutdown KeyStone inbound mailbox
 * @mport: Master port implementing the inbound message unit
 * @mbox: Mailbox to close
 *
 * Disables the outbound message unit, stop queues and free all resources
 */
void keystone_rio_close_inb_mbox(struct rio_mport *mport, int mbox)
{
	struct keystone_rio_data *krio_priv = mport->priv;

	dev_dbg(krio_priv->dev, "close inb mbox: mport = 0x%x, mbox = %d\n",
		(u32) mport, mbox);

	keystone_rio_close_rx_mbox(mbox, krio_priv);
}

/**
 * keystone_rio_hw_add_inb_buffer - Add buffer to the KeyStone inbound message
 * queue
 * @mport: Master port implementing the inbound message unit
 * @mbox: Inbound mailbox number
 * @buf: Buffer to add to inbound queue
 *
 * Adds the @buf buffer to the KeyStone inbound message queue. Returns
 * %0 on success or %-EINVAL on failure.
 */
int keystone_rio_hw_add_inb_buffer(struct rio_mport *mport,
				   int mbox,
				   void *buffer)
{
	struct keystone_rio_data *krio_priv = mport->priv;
	struct keystone_rio_rx_chan_info *krx_chan =
		&krio_priv->rx_channels[mbox];
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
void *keystone_rio_hw_get_inb_message(struct rio_mport *mport, int mbox)
{
	struct keystone_rio_data *krio_priv = mport->priv;
	struct keystone_rio_rx_chan_info *krx_chan =
		&(krio_priv->rx_channels[mbox]);
	struct keystone_rio_packet *p_info = NULL;
	void *buff = NULL;
	u32   cc;

	p_info = (struct keystone_rio_packet *)
		dma_get_one(krx_chan->dma_channel);
	if (!p_info)
		goto end;

	buff = p_info->buff;

	dma_unmap_sg(krio_priv->dev, &p_info->sg[2], 1, DMA_FROM_DEVICE);

	/* Check CC from PS descriptor word 1 */
	cc = keystone_rio_mp_get_cc(p_info->psdata[1], krx_chan->packet_type);
	if (cc)
		dev_warn(krio_priv->dev,
			 "MP receive completion code is non zero (0x%x)\n",
			 cc);

end:
	dmaengine_resume(krx_chan->dma_channel);

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

static int keystone_rio_mp_outb_init(u8 port_id,
				     struct keystone_rio_data *krio_priv)
{
	struct dma_keystone_info config;
	dma_cap_mask_t mask;
	int err = -ENODEV;
	const char *name;

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);

	/* DMA Tx channel */
	name = krio_priv->tx_chan_name;
	krio_priv->tx_channel = dma_request_channel_by_name(mask, name);
	if (IS_ERR_OR_NULL(krio_priv->tx_channel)) {
		dev_err(krio_priv->dev,
			"error requesting Tx channel, err %d\n", err);
		goto fail;
	}

	memset(&config, 0, sizeof(config));
	config.direction	= DMA_MEM_TO_DEV;
	config.tx_queue_depth	= krio_priv->tx_queue_depth;
	err = dma_keystone_config(krio_priv->tx_channel, &config);
	if (err) {
		dev_err(krio_priv->dev,
			"error configuring Tx channel, err %d\n", err);
		goto fail;
	}

	dev_dbg(krio_priv->dev, "opened Tx channel: %p\n",
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
int keystone_rio_open_outb_mbox(struct rio_mport *mport,
				void *dev_id,
				int mbox,
				int entries)
{
	struct keystone_rio_data *krio_priv = mport->priv;
	struct keystone_rio_mbox_info *tx_mbox = &(krio_priv->tx_mbox[mbox]);
	int res;

	if (mbox >= KEYSTONE_RIO_MAX_MBOX)
		return -EINVAL;

	/*
	 * Check that number of entries is a power of two to ease ring
	 * management
	 */
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

void keystone_rio_close_tx_mbox(int mbox,
				struct keystone_rio_data *krio_priv)
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
 * keystone_rio_close_outb_mbox - Shutdown KeyStone outbound mailbox
 * @mport: Master port implementing the outbound message unit
 * @mbox: Mailbox to close
 *
 * Disables the outbound message unit, stop queues and free all resources
 */
void keystone_rio_close_outb_mbox(struct rio_mport *mport, int mbox)
{
	struct keystone_rio_data *krio_priv = mport->priv;

	dev_dbg(krio_priv->dev, "close outb mbox: mport = 0x%x, mbox = %d\n",
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

	dev_dbg(krio_priv->dev,
		"tx_complete: psdata[0] = %08x, psdata[1] = %08x\n",
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
int keystone_rio_hw_add_outb_message(struct rio_mport *mport,
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
		kfree(send_buffer);
		dev_warn(krio_priv->dev, "failed to alloc packet info\n");
		return -ENOMEM;
	}

	p_info->priv = krio_priv;

	/* Word 1: source id and dest id (common to packet 11 and packet 9) */
	p_info->psdata[0] = (rdev->destid & 0xffff)
		| (mport->host_deviceid << 16);

	/*
	 * Warning - Undocumented HW requirement:
	 *      For type9, packet type MUST be set to 30 in
	 *	keystone_hw_desc.desc_info[29:25] bits.
	 *
	 *	For type 11, setting packet type to 31 in
	 *	those bits is optional.
	 */
	if (keystone_rio_mp_get_type(mbox, krio_priv)
	    == RIO_PACKET_TYPE_MESSAGE) {
		/* Packet 11 case (Message) */
		packet_type = 31;

		/* Word 2: ssize = 32 dword, 4 retries, letter = 0, mbox */
		p_info->psdata[1] = (KEYSTONE_RIO_MSG_SSIZE << 17) | (4 << 21)
			| (mbox & 0x3f);
	} else {
		/* Packet 9 case (Data Streaming) */
		packet_type = 30;

		/* Word 2: COS = 0, stream id */
		p_info->psdata[1] =
			keystone_rio_mbox_to_strmid(mbox, krio_priv) << 16;
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

	desc = dmaengine_prep_slave_sg(
		krio_priv->tx_channel,
		p_info->sg,
		p_info->sg_ents,
		DMA_MEM_TO_DEV,
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
		dev_warn(krio_priv->dev,
			 "failed to submit packet for dma: %d\n",
			 p_info->cookie);
		ret = -EBUSY;
		goto error;
	}

	return ret;

error:
	kfree(p_info);
	kfree(send_buffer);

	return ret;
}
