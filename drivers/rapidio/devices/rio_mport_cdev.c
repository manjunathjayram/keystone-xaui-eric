/*
 * RapidIO mport character device
 *
 * Copyright (C) 2014 Texas Instruments Incorporated
 * Aurelien Jacquiot <a-jacquiot@ti.com>
 * - Introduced DirectI/O, doorbell and basic maintenance operations
 *
 * Copyright 2014 Integrated Device Technology, Inc.
 * Alexandre Bounine <alexandre.bounine@idt.com>
 * - Added DMA data transfer IOCTL requests
 *
 * Copyright 2013 Prodrive B.V.
 * Jerry Jacobs <jerry.jacobs@prodrive.nl>
 * - Initial version of char device driver for RapidIO mport
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/cdev.h>
#include <linux/ioctl.h>
#include <linux/uaccess.h>
#include <linux/list.h>
#include <linux/fs.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/net.h>
#include <linux/poll.h>
#include <linux/spinlock.h>
#include <linux/sched.h>

#include <linux/mm.h>
#include <linux/slab.h>

#include <linux/dma-mapping.h>
#ifdef CONFIG_RAPIDIO_DMA_ENGINE
#include <linux/dmaengine.h>
#endif

#include <linux/rio.h>
#include <linux/rio_ids.h>
#include <linux/rio_drv.h>
#include <linux/rio_mport_cdev.h>

#define DRV_NAME	"rio_mport"

MODULE_AUTHOR("Jerry Jacobs <jerry.jacobs@prodrive.nl>");
MODULE_AUTHOR("Aurelien Jacquiot <a-jacquiot@ti.com>");
MODULE_AUTHOR("Alexandre Bounine <alexandre.bounine@idt.com>");
MODULE_DESCRIPTION("RapidIO mport character device driver");
MODULE_LICENSE("GPL");

#define DRV_PREFIX		"RIO " DRV_NAME ": "
#define DEV_NAME		"rio_mport"


/*
 * An internal DMA coherent buffer
 */
struct mport_dma_buf {
	void		*ib_base;
	dma_addr_t	ib_phys;
	u32		ib_size;
	u64		ib_rio_base;
};

#define MPORT_MAX_DMA_BUFS	8

/*
 * mport_dev  driver-specific structure that represents mport device
 * @node      list node to maintain list of registered mports
 * @cdev      character device
 * @dev       associated device object
 * @mport     associated subsystem's master port device object
 * @mbuf      table of descriptors for allocated kernel space buffers
 * @buf_count number of valid entries in the buffer descriptor table
 */
struct mport_dev {
	struct list_head	node;
	struct cdev		cdev;
	struct device		*dev;
	struct rio_mport	*mport;
	struct mport_dma_buf	mbuf[MPORT_MAX_DMA_BUFS];
	int			buf_count;
};

/*
 * mport_cdev_priv - this structure is used to track an open device
 * @md    master port character device object
 * @dmach DMA engine channel allocated for specific file object
 *
 */
struct mport_cdev_priv {
	struct mport_dev	*md;
	struct dma_chan		*dmach;
};


static LIST_HEAD(mport_devs);
static DEFINE_MUTEX(mport_devs_lock);
#if (0) /* used by commented out portion of poll function : FIXME */
static DECLARE_WAIT_QUEUE_HEAD(mport_cdev_wait);
#endif

static struct class *dev_class;
static int dev_count;
static dev_t dev_number;

static spinlock_t       dbell_i_lock;
static spinlock_t       dbell_list_lock;
static struct list_head dbell_list;

struct dbell_cell {
	struct list_head  node;
	u16               info;
	u16               src_id;
	u16               dst_id;
	wait_queue_head_t waitq;
};

/*
 * dbell_send() - Send a doorbell
 * @priv: driver private data
 * @arg:  Dbell information
 */
static int dbell_send(struct mport_cdev_priv *priv, void __user *arg)
{
	struct mport_dev *md = priv->md;
	struct rio_mport_dbell dbell;
	int ret;

	if (copy_from_user(&dbell, arg, sizeof(struct rio_mport_dbell)))
		return -EFAULT;

	if (!md->mport->ops->dsend)
		return -EPROTONOSUPPORT;

	pr_debug(DRV_PREFIX "Send doorbell %d to dest Id %d\n",
		 dbell.id, dbell.num);

	ret = md->mport->ops->dsend(md->mport, md->mport->id,
				      dbell.id, dbell.num);

	return ret;
}

static void dbell_callback(struct rio_mport *mport, 
			   void *dev_id,
			   u16 src,
			   u16 dst,
			   u16 info)
{
	struct dbell_cell *dbell = (struct dbell_cell*) dev_id;

	/* Wake up user process */
	if (waitqueue_active(&dbell->waitq)) {
		dbell->src_id = src;
		dbell->dst_id = dst;
		dbell->info   = info;
		wake_up_all(&dbell->waitq);
	}
}

static struct dbell_cell* dbell_lookup(u16 info)
{
	struct dbell_cell *dbell;
	int found = 0;

	spin_lock(&dbell_list_lock);

	/* Look if a waitqueue already exists for this doorbell */
	list_for_each_entry(dbell, &dbell_list, node) {
		if (dbell->info == info) {
			found = 1;
			break;
		}
	}

	if (found) {
		goto out;
	}

	/* Allocate and insert the doorbell */
	dbell = kmalloc(sizeof(struct dbell_cell), GFP_KERNEL);
	if (dbell == NULL)
		goto out;
	
	dbell->info = info;
	init_waitqueue_head(&dbell->waitq);

	list_add_tail(&dbell->node, &dbell_list);
out:
	spin_unlock(&dbell_list_lock);

	return dbell;
}

static int dbell_release(u16 info)
{
	struct dbell_cell *dbell;
	int found = 0;
	int res   = 0;

	spin_lock(&dbell_list_lock);

	/* Look for the corresponding waitqueue */
	list_for_each_entry(dbell, &dbell_list, node) {
		if (dbell->info == info) {
			found = 1;
			break;
		}
	}

	if (!found) {
		res = -EINVAL;
		goto out;
	}

	/* Delete and free waitqueue from list */
	list_del(&dbell->node);
	kfree(dbell);
out:
	spin_unlock(&dbell_list_lock);

	return res;
}

static int dbell_wait(struct rio_mport *mport, u16 info, u16 *src_id)
{
	struct dbell_cell *dbell;
	unsigned long flags;
	int res;

	DECLARE_WAITQUEUE(wait, current);

	dbell = dbell_lookup(info);
	if (dbell == NULL)
		return -ENOMEM;

	/* Request a doorbell with our callback handler */
	res = rio_request_inb_dbell(mport,
				    (void *) dbell,
				    info,
				    info,
				    dbell_callback);

	if ((res != 0) && (res != -EBUSY)) {
		pr_debug(DRV_PREFIX "cannot request such doorbell (info = %d)\n", info);
		return res;
	}

	/* Schedule until handler is called */
	spin_lock_irqsave(&dbell_i_lock, flags);
	add_wait_queue(&dbell->waitq, &wait);
	set_current_state(TASK_INTERRUPTIBLE);
	spin_unlock_irqrestore(&dbell_i_lock, flags);

 	schedule();

	spin_lock_irqsave(&dbell_i_lock, flags);
	__set_current_state(TASK_RUNNING);
	remove_wait_queue(&dbell->waitq, &wait);
	spin_unlock_irqrestore(&dbell_i_lock, flags);

	if (src_id)
		*src_id = dbell->src_id;

	/* Release the doorbell */
	rio_release_inb_dbell(mport, info, info);
	dbell_release(info);

	if (signal_pending(current))
		return -ERESTARTSYS;

	pr_debug(DRV_PREFIX "receiving doorbell (info = %d)\n", info);

	return info;
}

/*
 * dbell_receive() - receive a doorbell
 * @priv: driver private data
 * @arg: Dbell information
 */
static int dbell_receive(struct mport_cdev_priv *priv, void __user *arg)
{
	struct mport_dev *md = priv->md;
	struct rio_mport_dbell dbell;
	int ret;

	if (copy_from_user(&dbell, arg, sizeof(struct rio_mport_dbell)))
		return -EFAULT;

	pr_debug(DRV_PREFIX "Wait doorbell %d\n", dbell.num);

	ret = dbell_wait(md->mport, dbell.num, &dbell.id);

	if (copy_to_user(arg, &dbell, sizeof(struct rio_mport_dbell)))
		ret = -EFAULT;

	return ret;
}

#ifdef CONFIG_TI_KEYSTONE_RAPIDIO
/*
 * dio_transfer() - Perform a DirectI/O transfer
 * @priv: driver private data
 * @arg:  DirectI/O transfer information
 */
static int dio_transfer(struct mport_cdev_priv *priv, void __user *arg)
{
	struct mport_dev *md = priv->md;
	struct rio_mport_dio_transfer dio_transfer;
	int ret;
	void *vaddr;

	if (copy_from_user(&dio_transfer, arg,
			   sizeof(struct rio_mport_dio_transfer)))
		return -EFAULT;

	if (!md->mport->ops->transfer)
		return -EPROTONOSUPPORT;

	if (dio_transfer.length >= RIO_MAX_DIO_CHUNK_SIZE)
		return -EINVAL;

	pr_debug(DRV_PREFIX "Perform DirectI/O %d to dest Id %d\n",
		 dio_transfer.id, dio_transfer.mode);

	/*
	 * We use a copy there, zero-copy will be implemented when moving
	 * to DMA engine interface.
	 */
	vaddr = kmalloc(dio_transfer.length, GFP_KERNEL);
	if (!vaddr)
		return -ENOMEM;

	if ((dio_transfer.mode ==  RIO_DIO_MODE_WRITER)
	    || (dio_transfer.mode ==  RIO_DIO_MODE_WRITE)
	    || (dio_transfer.mode ==  RIO_DIO_MODE_SWRITE)) {
		if (copy_from_user(vaddr,
				   dio_transfer.src_addr,
				   dio_transfer.length)) {
			ret = -EFAULT;
			goto end;
		}
	}

	ret = md->mport->ops->transfer(md->mport,
				       md->mport->id,
				       dio_transfer.id,
				       (u32) vaddr,
				       (u32) dio_transfer.tgt_addr,
				       (int) dio_transfer.length,
				       (int) dio_transfer.mode);
	if (ret < 0)
		goto end;

	if (dio_transfer.mode ==  RIO_DIO_MODE_READ) {
		if (copy_to_user(dio_transfer.src_addr,
				 vaddr,
				 dio_transfer.length)) {
			ret = -EFAULT;
			goto end;
		}
	}
end:
	kfree(vaddr);
	return ret;
}
#endif /* CONFIG_TI_KEYSTONE_RAPIDIO */

/*
 * maint_lconfig_read() - Perform a local config space read transaction
 * @priv: driver private data
 * @arg:  Maintenance transaction information
 */
static int maint_lconfig_read(struct mport_cdev_priv *priv, void __user *arg)
{
	struct mport_dev *md = priv->md;
	struct rio_mport_maint_transfer maint_transfer;
	int ret;
	void *buf;

	if (copy_from_user(&maint_transfer, arg,
			   sizeof(struct rio_mport_maint_transfer)))
		return -EFAULT;

	if (!md->mport->ops->lcread)
		return -EPROTONOSUPPORT;

	pr_debug(DRV_PREFIX "Perform a local maintenance read at offset %d\n",
		 maint_transfer.offset);

	buf = kmalloc(maint_transfer.length, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = md->mport->ops->lcread(md->mport, md->mport->id,
				     maint_transfer.offset,
				     maint_transfer.length,
				     (u32 *) buf);

	if (copy_to_user(maint_transfer.val, buf, maint_transfer.length))
		return -EFAULT;

	return ret;
}

/*
 * maint_lconfig_write() - Perform a local config space write transaction
 * @priv: driver private data
 * @arg:  Maintenance transaction information
 */
static int maint_lconfig_write(struct mport_cdev_priv *priv, void __user *arg)
{
	struct mport_dev *md = priv->md;
	struct rio_mport_maint_transfer maint_transfer;
	int ret;
	u32 val;

	if (copy_from_user(&maint_transfer, arg,
			   sizeof(struct rio_mport_maint_transfer)))
		return -EFAULT;

	if (!md->mport->ops->lcwrite)
		return -EPROTONOSUPPORT;

	pr_debug(DRV_PREFIX "Perform a local maintenance read at offset %d\n",
		 maint_transfer.offset);

	if (copy_from_user(&val, maint_transfer.val, sizeof(val)))
		return -EFAULT;

	ret = md->mport->ops->lcwrite(md->mport, md->mport->id,
				      maint_transfer.offset,
				      maint_transfer.length,
				      val);

	return ret;
}

/*
 * maint_config_read() - Perform a remote config space read transaction
 * @priv: driver private data
 * @arg:	Maintenance transaction information
 */
static int maint_config_read(struct mport_cdev_priv *priv, void __user *arg)
{
	struct mport_dev *md = priv->md;
	struct rio_mport_maint_transfer maint_transfer;
	int ret;
	void *buf;

	if (copy_from_user(&maint_transfer, arg, sizeof(struct rio_mport_maint_transfer)))
		return -EFAULT;

	if (!md->mport->ops->cread)
		return -EPROTONOSUPPORT;

	pr_debug(DRV_PREFIX "Perform a remote maintenance read at offset %d\n",
		 maint_transfer.offset);

	buf = kmalloc(maint_transfer.length, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = md->mport->ops->cread(md->mport,
				    md->mport->id,
				    maint_transfer.id,
				    maint_transfer.hopcount,
				    maint_transfer.offset,
				    maint_transfer.length,
				    (u32 *) buf);

	if (copy_to_user(maint_transfer.val, buf, maint_transfer.length))
		return -EFAULT;

	return ret;
}

/*
 * maint_config_write() - Perform a remote config space write transaction
 * @priv: driver private data
 * @arg:	Maintenance transaction information
 */
static int maint_config_write(struct mport_cdev_priv *priv, void __user *arg)
{
	struct mport_dev *md = priv->md;
	struct rio_mport_maint_transfer maint_transfer;
	int ret;
	u32 val;

	if (copy_from_user(&maint_transfer, arg,
			   sizeof(struct rio_mport_maint_transfer)))
		return -EFAULT;

	if (!md->mport->ops->cwrite)
		return -EPROTONOSUPPORT;

	pr_debug(DRV_PREFIX "Perform a remote maintenance read at offset %d\n",
		 maint_transfer.offset);

	if (copy_from_user(&val, maint_transfer.val, sizeof(val)))
		return -EFAULT;

	ret = md->mport->ops->cwrite(md->mport,
				     md->mport->id,
				     maint_transfer.id,
				     maint_transfer.hopcount,
				     maint_transfer.offset,
				     maint_transfer.length,
				     val);

	return ret;
}

/*
 * maint_hdid_set() - Set the host Device ID
 * @priv: driver private data
 * @arg:	Device Id
 */
static int maint_hdid_set(struct mport_cdev_priv *priv, void __user *arg)
{
	struct mport_dev *md = priv->md;
	uint16_t hdid;

	if (copy_from_user(&hdid, arg, sizeof(uint16_t)))
		return -EFAULT;

	md->mport->host_deviceid = hdid;
	rio_local_set_device_id(md->mport, hdid);
	
	pr_debug(DRV_PREFIX "Set host device Id to %d\n", hdid);

	return 0;
}

/*
 * maint_comptag_set() - Set the host Component Tag
 * @priv: driver private data
 * @arg:	Component Tag
 */
static int maint_comptag_set(struct mport_cdev_priv *priv, void __user *arg)
{
	struct mport_dev *md = priv->md;
	uint32_t comptag;

	if (copy_from_user(&comptag, arg, sizeof(uint32_t)))
		return -EFAULT;

	rio_local_write_config_32(md->mport, RIO_COMPONENT_TAG_CSR, comptag);

	pr_debug(DRV_PREFIX "Set host Component Tag to %d\n", comptag);

	return 0;
}

#ifdef CONFIG_RAPIDIO_DMA_ENGINE

/* DMA Transfer Timeout in msec */
static int timeout = 3000;
module_param(timeout, int, S_IRUGO);
MODULE_PARM_DESC(timeout, "DMA Transfer Timeout in msec (default: 3000)");

static void dma_callback(void *completion)
{
	complete(completion);
}

static struct dma_async_tx_descriptor
*prep_dma_xfer(struct dma_chan *chan, struct rio_mport_dma_transfer *pdt,
	struct sg_table *sgt, int nents, enum dma_transfer_direction dir,
	enum dma_ctrl_flags flags)
{
	struct rio_dma_data tx_data;
	struct dma_async_tx_descriptor *tx = NULL;

	tx_data.sg = sgt->sgl;
	tx_data.sg_len = nents;
	tx_data.rio_addr_u = 0;
	tx_data.rio_addr = pdt->tgt_addr;
	if (dir == DMA_MEM_TO_DEV) {
		switch (pdt->mode) {
		case RIO_DMA_ALL_NWRITE:
			tx_data.wr_type = RDW_ALL_NWRITE;
			break;
		case RIO_DMA_ALL_NWRITE_R:
			tx_data.wr_type = RDW_ALL_NWRITE_R;
			break;
		case RIO_DMA_LAST_NWRITE_R:
			tx_data.wr_type = RDW_LAST_NWRITE_R;
			break;
		case RIO_DMA_DEFAULT:
		default:
			tx_data.wr_type = RDW_DEFAULT;
			break;
		}
	}

	tx = rio_dma_prep_slave_sg(chan, pdt->id, &tx_data, dir, flags);

	return tx;
}

static int do_dma_request(struct mport_cdev_priv *priv, struct sg_table *sgt,
			  struct rio_mport_dma_transfer *dt,
			  enum dma_transfer_direction direction)
{
	struct dma_chan *chan;
	struct dma_async_tx_descriptor *tx;
	struct completion cmp;
	dma_cookie_t cookie;
	unsigned long tmo = msecs_to_jiffies(timeout);
	enum dma_status	status;
	int nents, ret = 0;

	/* Request DMA channel associated with this mport device */
	if (!priv->dmach) {
		priv->dmach = rio_request_dma(priv->md->mport);
		if (!priv->dmach) {
			pr_err(DRV_PREFIX "%s: Failed to get DMA channel\n",
				__func__);
			return -ENODEV;
		}
	}

	chan = priv->dmach;

	nents = dma_map_sg(chan->device->dev, sgt->sgl, sgt->nents, direction);
	if (nents == -EFAULT) {
		pr_err(DRV_PREFIX "%s: Failed to map SG list\n", __func__);
		return -EFAULT;
	}

	/* Initialize DMA transaction request */
	tx = prep_dma_xfer(chan, dt, sgt, nents, direction,
			   DMA_CTRL_ACK | DMA_PREP_INTERRUPT);

	if (!tx) {
		pr_err(DRV_PREFIX "%s: prep error for %s A:0x%llx L:0x%x\n",
			__func__, (direction == DMA_DEV_TO_MEM)?"READ":"WRITE",
			dt->tgt_addr, dt->length);
		ret = -EIO;
		goto err_out;
	}

	init_completion(&cmp);
	tx->callback = dma_callback;
	tx->callback_param = &cmp;

	/* Submit DMA transaction request */
	cookie = tx->tx_submit(tx);
	pr_debug(DRV_PREFIX "%s: DMA tx_cookie = %d\n", __func__, cookie);

	if (dma_submit_error(cookie)) {
		pr_err(DRV_PREFIX "%s: submit err=%d (addr:0x%llx len:0x%x)\n",
			__func__, cookie, dt->tgt_addr, dt->length);
		ret = -EIO;
		goto err_out;
	}

	dma_async_issue_pending(chan);

	tmo = wait_for_completion_interruptible_timeout(&cmp, tmo);
	status = dma_async_is_tx_complete(chan, cookie, NULL, NULL);

	if (tmo == 0) {
		/* Timeout on wait_for_completion occurred */
		pr_err(DRV_PREFIX "%s: timed out waiting for DMA\n", __func__);
		ret = -ETIMEDOUT;
	} else if (tmo == -ERESTARTSYS) {
		/* Wait_for_completion was interrupted by a signal but DMA may
		   be still in progress */
		pr_warn(DRV_PREFIX "%s: wait for DMA completion interrupted\n",
			__func__);
		ret = -EINTR;
	} else if (status != DMA_SUCCESS) {
		/* DMA transaction completion was signaled for failed data
		   transfer */
		pr_warn(DRV_PREFIX "%s: DMA completion with status %d\n",
			__func__, status);
		ret = -EIO;
	}

	if (ret)
		dmaengine_terminate_all(chan);

err_out:
	dma_unmap_sg(chan->device->dev, sgt->sgl, sgt->nents, direction);
	return ret;
}


/*
 * rio_dma_transfer() - Perform RapidIO DMA data transfer to/from
 *                      the remote RapidIO device
 * @priv:      driver private data
 * @arg:       DMA transfer information
 * @direction: DMA transfer direction (DMA_MEM_TO_DEV = write OR
 *                                      DMA_DEV_TO_MEM = read)
 */
static int rio_dma_transfer(struct mport_cdev_priv *priv, void __user *arg,
			    enum dma_transfer_direction direction)
{
	struct mport_dev *md = priv->md;
	struct rio_mport_dma_transfer dt;
	unsigned int nr_pages = 0;
	struct page **page_list = NULL;
	struct sg_table sgt;
	int i, ret;

	if (copy_from_user(&dt, arg,
			   sizeof(struct rio_mport_dma_transfer)))
		return -EFAULT;

	if (dt.length == 0)
		return -EINVAL;

	/*
	 * If parameter loc_addr != NULL, we are transferring data from/to
	 * data buffer allocated in user-space: lock in memory user-space
	 * buffer pages and build an SG table for DMA transfer request
	 *
	 * Otherwise (loc_addr == NULL) contiguous kernel-space buffer is
	 * used for DMA data transfers: build single entry SG table using
	 * offset within the internal buffer specified by handle parameter.
	 */
	if (dt.loc_addr) {
		unsigned long offset;
		long pinned;

		offset = (unsigned long)dt.loc_addr & ~PAGE_MASK;
		nr_pages = PAGE_ALIGN(dt.length + offset) >> PAGE_SHIFT;

		page_list = kmalloc(nr_pages * sizeof(*page_list), GFP_KERNEL);
		if (page_list == NULL)
			return -ENOMEM;

		down_read(&current->mm->mmap_sem);
		pinned = get_user_pages(current, current->mm,
				(unsigned long)dt.loc_addr & PAGE_MASK,
				nr_pages, direction == DMA_DEV_TO_MEM, 0,
				page_list, NULL);
		up_read(&current->mm->mmap_sem);

		if (pinned != nr_pages) {
			pr_err(DRV_PREFIX "%s: ERROR: pinned %ld out of %d pages\n",
				__func__, pinned, nr_pages);
			kfree(page_list);
			return -ENOMEM;
		}

		ret = sg_alloc_table_from_pages(&sgt, page_list, nr_pages,
						offset, dt.length, GFP_KERNEL);
		if (ret) {
			pr_err(DRV_PREFIX "%s: sg_alloc_table failed with err=%d\n",
				__func__, ret);
			goto err_out;
		}
	} else {
		dma_addr_t baddr;
		struct mport_dma_buf *mbuf;

		baddr = (dma_addr_t)dt.handle;

		for (i = 0; i < MPORT_MAX_DMA_BUFS; i++)
			if (md->mbuf[i].ib_base && baddr >= md->mbuf[i].ib_phys &&
			    baddr < (md->mbuf[i].ib_phys + md->mbuf[i].ib_size))
				break;

		if (i == MPORT_MAX_DMA_BUFS)
			return -ENOMEM;

		mbuf = &md->mbuf[i];

		if (dt.length + dt.offset > mbuf->ib_size)
			return -EINVAL;

		ret = sg_alloc_table(&sgt, 1, GFP_KERNEL);
		if (unlikely(ret)) {
			pr_err("%s: sg_alloc_table failed for internal buf\n",
				__func__);
			return ret;
		}

		sg_set_buf(sgt.sgl, mbuf->ib_base + dt.offset, dt.length);

	}

	ret = do_dma_request(priv, &sgt, &dt, direction);

	sg_free_table(&sgt);
err_out:
	if (page_list) {
		for (i = 0; i < nr_pages; i++)
			put_page(page_list[i]);
		kfree(page_list);
	}
	return ret;
}

/*
 * rio_dma_buf_alloc() - allocate DMA coherent memory buffer for inbound RapidIO
 *                  reas/write requests and map it into RapidIO address space
 * @priv: driver private data
 * @arg:  buffer descriptor structure
 */
static int rio_dma_buf_alloc(struct mport_cdev_priv *priv, void __user *arg)
{
	struct mport_dev *md = priv->md;
	struct rio_mport_dma_buf db;
	struct mport_dma_buf *mbuf;
	u64 handle;
	int i, ret;

	if (md->buf_count == MPORT_MAX_DMA_BUFS)
		return -EBUSY;

	/*
	 * Find a free entry in the buffer allocation table
	 */
	for (i = 0; i < MPORT_MAX_DMA_BUFS; i++)
		if (md->mbuf[i].ib_base == NULL)
			break;

	if (i == MPORT_MAX_DMA_BUFS)
		return -EFAULT;

	mbuf = &md->mbuf[i];

	if (copy_from_user(&db, arg, sizeof(struct rio_mport_dma_buf)))
		return -EFAULT;

	mbuf->ib_base = dma_alloc_coherent(md->mport->dev.parent,
				     db.length, &mbuf->ib_phys, GFP_KERNEL);
	if (!mbuf->ib_base) {
		pr_err("%s: Unable allocate DMA coherent memory (size=0x%x)\n",
			__func__, db.length);
		return -ENOMEM;
	}

	mbuf->ib_size = db.length;
	mbuf->ib_rio_base = 0;
	md->buf_count++;

	pr_debug("%s: internal DMA buffer %d @ %p (pa = %llx) for %s\n",
		__func__, i, mbuf->ib_base,
		(unsigned long long)(mbuf->ib_phys), md->mport->name);

	handle = mbuf->ib_phys;
	if (copy_to_user(db.handle, &handle, sizeof(u64))) {
		ret = -EFAULT;
		goto err_out;
	}

	return 0;

err_out:
	dma_free_coherent(&md->mport->dev,
			  db.length, mbuf->ib_base, mbuf->ib_phys);
	mbuf->ib_base = NULL;
	mbuf->ib_phys = 0;
	mbuf->ib_size = 0;
	return ret;
}

/*
 * rio_dma_buf_free() - free a previously allocated DMA coherent buffer
 * @priv: driver private data
 * @arg:  buffer handle returned by allocation routine
 */
static int rio_dma_buf_free(struct mport_cdev_priv *priv, void __user *arg)
{
	struct mport_dev *md = priv->md;
	struct mport_dma_buf *mbuf;
	u64 handle;
	int i;

	if (!md->buf_count)
		return -EINVAL;

	if (copy_from_user(&handle, arg, sizeof(u64)))
		return -EFAULT;

	for (i = 0; i < MPORT_MAX_DMA_BUFS; i++)
		if (md->mbuf[i].ib_phys == handle)
			break;

	if (i == MPORT_MAX_DMA_BUFS)
		return -EFAULT;

	mbuf = &md->mbuf[i];

	pr_debug("%s: free internal DMA buffer %d @ %p (pa = %llx) for %s\n",
		__func__, i, mbuf->ib_base,
		(unsigned long long)(mbuf->ib_phys), md->mport->name);

	dma_free_coherent(md->mport->dev.parent,
			  mbuf->ib_size, mbuf->ib_base, mbuf->ib_phys);
	mbuf->ib_base = NULL;
	mbuf->ib_phys = 0;
	mbuf->ib_size = 0;
	md->buf_count--;
	return 0;
}

#endif /* CONFIG_RAPIDIO_DMA_ENGINE */

/*
 * ibw_map_alloc() - allocate DMA coherent memory buffer for inbound RapidIO
 *                   reas/write requests and map it into RapidIO address space
 * @priv: driver private data
 * @arg:  inbound mapping info structure
 */
static int ibw_map_alloc(struct mport_cdev_priv *priv, void __user *arg)
{
	struct mport_dev *md = priv->md;
	struct rio_mport_inbound_map ibw;
	struct mport_dma_buf *mbuf;
	u64 handle;
	int i, ret;

	if (!md->mport->ops->map_inb)
		return -EPROTONOSUPPORT;

	if (md->buf_count == MPORT_MAX_DMA_BUFS)
		return -EBUSY;

	for (i = 0; i < MPORT_MAX_DMA_BUFS; i++)
		if (md->mbuf[i].ib_base == NULL)
			break;

	if (i == MPORT_MAX_DMA_BUFS)
		return -EFAULT;

	mbuf = &md->mbuf[i];

	if (copy_from_user(&ibw, arg, sizeof(struct rio_mport_inbound_map)))
		return -EFAULT;

	/*
	 * Allocate and map inbound DMA data buffer
	 */
	mbuf->ib_base = dma_alloc_coherent(md->mport->dev.parent,
				     ibw.length, &mbuf->ib_phys, GFP_KERNEL);
	if (!mbuf->ib_base) {
		pr_err("%s: Unable allocate IB DMA memory (size=0x%x)\n",
			__func__, ibw.length);
		return -ENOMEM;
	}

	ret = rio_map_inb_region(md->mport,
				 mbuf->ib_phys, ibw.rio_base, ibw.length, 0);
	if (ret) {
		pr_err("%s: Request to map IB region failed for %s, err=%d\n",
			__func__, md->mport->name, ret);
		goto err_map;
	}

	mbuf->ib_size = ibw.length;
	mbuf->ib_rio_base = ibw.rio_base;
	md->buf_count++;

	pr_debug("%s: Configured IB DMA buffer @ %p (phys = %llx) for %s\n",
		__func__, mbuf->ib_base,
		(unsigned long long)(mbuf->ib_phys), md->mport->name);

	handle = mbuf->ib_phys;
	if (copy_to_user(ibw.handle, &handle, sizeof(u64))) {
		ret = -EFAULT;
		goto err_out;
	}

	return 0;

err_out:
	rio_unmap_inb_region(md->mport, mbuf->ib_phys);
err_map:
	dma_free_coherent(&md->mport->dev,
			  ibw.length, mbuf->ib_base, mbuf->ib_phys);
	mbuf->ib_base = NULL;
	mbuf->ib_phys = 0;
	mbuf->ib_size = 0;
	return ret;
}

/*
 * ibw_unmap_free() - unmap from RapidIO address space and free a previously
 *                    allocated inbound DMA coherent buffer
 * @priv: driver private data
 * @arg:  buffer handle returned by allocation routine
 */
static int ibw_unmap_free(struct mport_cdev_priv *priv, void __user *arg)
{
	struct mport_dev *md = priv->md;
	struct mport_dma_buf *mbuf;
	u64 handle;
	int i;

	if (!md->mport->ops->unmap_inb)
		return -EPROTONOSUPPORT;
	if (!md->buf_count)
		return -EINVAL;

	if (copy_from_user(&handle, arg, sizeof(u64)))
		return -EFAULT;

	for (i = 0; i < MPORT_MAX_DMA_BUFS; i++)
		if (md->mbuf[i].ib_phys == handle)
			break;

	if (i == MPORT_MAX_DMA_BUFS)
		return -EFAULT;

	mbuf = &md->mbuf[i];

	pr_debug("%s: unmap IB DMA buffer @ %p (phys = %llx) for %s\n",
		__func__, mbuf->ib_base,
		(unsigned long long)(mbuf->ib_phys), md->mport->name);

	rio_unmap_inb_region(md->mport, mbuf->ib_phys);
	dma_free_coherent(md->mport->dev.parent,
			  mbuf->ib_size, mbuf->ib_base, mbuf->ib_phys);
	mbuf->ib_base = NULL;
	mbuf->ib_phys = 0;
	mbuf->ib_size = 0;
	md->buf_count--;
	return 0;
}

/*
 * maint_port_idx_get() - Get the port index of the mport instance
 * @priv: driver private data
 * @arg:  port index
 */
static int maint_port_idx_get(struct mport_cdev_priv *priv, void __user *arg)
{
	struct mport_dev *md = priv->md;
	uint32_t port_idx = md->mport->index;

	pr_debug(DRV_PREFIX "Get port index %d\n", port_idx);

	if (copy_to_user(arg, &port_idx, sizeof(port_idx)))
		return -EFAULT;

	return 0;
}

/*
 * Mport cdev management
 */

/*
 * mport_cdev_open() - Open character device (mport)
 */
static int mport_cdev_open(struct inode *inode, struct file *filp)
{
	int minor = iminor(inode);
	struct mport_dev *chdev = NULL;
	struct mport_cdev_priv *priv;

	/* Test for valid device */
	if (minor >= RIO_MAX_MPORTS) {
		pr_err(DRV_PREFIX "Invalid minor device number\n");
		return -EINVAL;
	}

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	chdev = container_of(inode->i_cdev, struct mport_dev, cdev);
	priv->md = chdev;
	filp->private_data = priv;

	return 0;
}

/*
 * mport_cdev_release() - Release character device
 */
static int mport_cdev_release(struct inode *inode, struct file *filp)
{
	struct mport_cdev_priv *priv = filp->private_data;

#ifdef CONFIG_RAPIDIO_DMA_ENGINE
	if (priv->dmach) {
		dmaengine_terminate_all(priv->dmach);
		dma_release_channel(priv->dmach);
	}
#endif
	kfree(priv);
	return 0;
}

/*
 * mport_cdev_ioctl() - IOCTLs for character device
 */
static long mport_cdev_ioctl(struct file *filp,
		unsigned int cmd, unsigned long arg)
{
	int err = 0;
	struct mport_cdev_priv *data = filp->private_data;

	switch (cmd) {
	case RIO_MPORT_DBELL_SEND:
		err = dbell_send(data, (void __user *)arg);
		break;
	case RIO_MPORT_DBELL_RECEIVE:
		err = dbell_receive(data, (void __user *)arg);
		break;
#ifdef CONFIG_TI_KEYSTONE_RAPIDIO
	case RIO_MPORT_DIO_TRANSFER:
		err = dio_transfer(data, (void __user *)arg);
		break;
#endif
#ifdef CONFIG_RAPIDIO_DMA_ENGINE
	case RIO_MPORT_DMA_GET_XFER_SIZE:
		err = -EINVAL;
		break;
	case RIO_MPORT_DMA_READ:
		err = rio_dma_transfer(data,
				       (void __user *)arg, DMA_DEV_TO_MEM);
		break;
	case RIO_MPORT_DMA_WRITE:
		err = rio_dma_transfer(data,
				       (void __user *)arg, DMA_MEM_TO_DEV);
		break;
	case RIO_MPORT_DMA_BUF_ALLOC:
		err = rio_dma_buf_alloc(data, (void __user *)arg);
		break;
	case RIO_MPORT_DMA_BUF_FREE:
		err = rio_dma_buf_free(data, (void __user *)arg);
		break;
#endif
	case RIO_MPORT_MAINT_LOCAL_CONFIG_READ:
		err = maint_lconfig_read(data, (void __user *)arg);
		break;
	case RIO_MPORT_MAINT_LOCAL_CONFIG_WRITE:
		err = maint_lconfig_write(data, (void __user *)arg);
		break;
	case RIO_MPORT_MAINT_CONFIG_READ:
		err = maint_config_read(data, (void __user *)arg);
		break;
	case RIO_MPORT_MAINT_CONFIG_WRITE:
		err = maint_config_write(data, (void __user *)arg);
		break;
	case RIO_MPORT_MAINT_HDID_SET:
		err = maint_hdid_set(data, (void __user *)arg);
		break;
	case RIO_MPORT_MAINT_COMPTAG_SET:
		err = maint_comptag_set(data, (void __user *)arg);
		break;
	case RIO_MPORT_MAINT_PORT_IDX_GET:
		err = maint_port_idx_get(data, (void __user *)arg);
		break;
	case RIO_MPORT_INBOUND_ALLOC:
		err = ibw_map_alloc(data, (void __user *)arg);
		break;
	case RIO_MPORT_INBOUND_FREE:
		err = ibw_unmap_free(data, (void __user *)arg);
		break;
	default:
		err = -EINVAL;
		break;
	}

	return err;
}

static int mport_cdev_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct mport_cdev_priv *priv = filp->private_data;
	struct mport_dev *md;
	struct mport_dma_buf *mbuf;
	size_t size = vma->vm_end - vma->vm_start;
	dma_addr_t baddr;
	unsigned long offset;
	int i, ret;

	pr_debug(DRV_PREFIX "%s 0x%x bytes at offset 0x%lx\n",
		__func__, (unsigned int)size, vma->vm_pgoff);

	md = priv->md;
	baddr = ((dma_addr_t)vma->vm_pgoff << PAGE_SHIFT);

	for (i = 0; i < MPORT_MAX_DMA_BUFS; i++)
		if (md->mbuf[i].ib_base && baddr >= md->mbuf[i].ib_phys &&
		    baddr < (md->mbuf[i].ib_phys + md->mbuf[i].ib_size))
			break;

	if (i == MPORT_MAX_DMA_BUFS)
		return -ENOMEM;

	mbuf = &md->mbuf[i];
	offset = baddr & (mbuf->ib_size - 1);

	if (size + offset > mbuf->ib_size)
		return -EINVAL;

	vma->vm_pgoff = offset >> PAGE_SHIFT;
	pr_debug(DRV_PREFIX "MMAP adjusted offset = 0x%lx\n", vma->vm_pgoff);

	ret = dma_mmap_coherent(md->mport->dev.parent, vma, mbuf->ib_base,
				mbuf->ib_phys, mbuf->ib_size);

	if (ret)
		pr_err(DRV_PREFIX "MMAP exit with err=%d\n", ret);

	return ret;
}

unsigned int mport_poll(struct file *filp, poll_table *wait)
{
	unsigned int mask = 0;
#if 0

	struct mport_dev *chdev = filp->private_data->md;

	poll_wait(file, &mport_cdev_wait, wait);

	if (chdev->rx_head != chdev->rx_tail)
		mask |= POLLIN | POLLRDNORM;

#endif
	return mask;
}

static const struct file_operations mport_fops = {
	.owner		= THIS_MODULE,
	.open		= mport_cdev_open,
	.release	= mport_cdev_release,
	.poll		= mport_poll,
	.mmap		= mport_cdev_mmap,
	.unlocked_ioctl = mport_cdev_ioctl
};

/*
 * Character device management
 */

/*
 * mport_cdev_add() - Create mport_dev from rio_mport
 * @mport:	RapidIO master port
 * TODO add created mport_devs to global list
 */
static struct mport_dev *mport_cdev_add(struct rio_mport *mport)
{
	int ret = 0;
	dev_t devno;
	struct mport_dev *device;

	device = kzalloc(sizeof(struct mport_dev), GFP_KERNEL);
	if (!device) {
		pr_err(DRV_PREFIX "Unable allocate a device object");
		return NULL;
	}

	device->mport = mport;
	devno = MKDEV(MAJOR(dev_number), mport->id);
	cdev_init(&device->cdev, &mport_fops);
	device->cdev.owner = THIS_MODULE;
	ret = cdev_add(&device->cdev, devno, 1);
	if (ret < 0) {
		kfree(device);
		pr_err(DRV_PREFIX
			"Cannot register a device with error %d\n", ret);
		return NULL;
	}

	device->dev = device_create(dev_class, NULL, devno,
			NULL, DEV_NAME "%d", mport->id);
	if (IS_ERR(device->dev)) {
		cdev_del(&device->cdev);
		kfree(device);
		return NULL;
	}

	mutex_lock(&mport_devs_lock);
	list_add_tail(&device->node, &mport_devs);
	dev_count++;
	mutex_unlock(&mport_devs_lock);

	pr_info(DRV_PREFIX "Added %s cdev(%d:%d)\n",
		mport->name, MAJOR(dev_number), mport->id);

	return device;
}

/*
 * mport_cdev_remove() - Remove mport character device from mport_devs
 * @dev:	Mport device to remove
 */
static void mport_cdev_remove(struct mport_dev *dev)
{
	if (!dev)
		return;

	pr_debug(DRV_PREFIX "%s: Removing %s cdev\n", __func__, dev->mport->name);
	device_unregister(dev->dev);
	cdev_del(&(dev->cdev));
}

/*
 * RIO rio_mport_interface driver
 */

/*
 * mport_add_mport() - Add rio_mport from LDM device struct
 * @dev:		Linux device model struct
 * @class_intf:	Linux class_interface
 */
static int mport_add_mport(struct device *dev,
		struct class_interface *class_intf)
{
	struct rio_mport *mport = NULL;
	struct mport_dev *chdev = NULL;

	mport = to_rio_mport(dev);
	if (!mport)
		return -ENODEV;

	chdev = mport_cdev_add(mport);
	if (!chdev)
		return -ENODEV;

	return 0;
}

/*
 * mport_remove_mport() - Remove rio_mport from global list
 * TODO remove device from global mport_dev list
 */
static void mport_remove_mport(struct device *dev,
		struct class_interface *class_intf)
{
	struct rio_mport *mport = NULL;
	struct mport_dev *chdev;
	mport = to_rio_mport(dev);
	if (!mport)
		return;

	pr_debug(DRV_PREFIX "%s: Removing mport %s\n", __func__, mport->name);

	mutex_lock(&mport_devs_lock);
	list_for_each_entry(chdev, &mport_devs, node) {
		if (chdev->mport->id == mport->id) {
			mport_cdev_remove(chdev);
			list_del(&chdev->node);
			kfree(chdev);
			break;
		}
	}
	mutex_unlock(&mport_devs_lock);
}

/* the rio_mport_interface is used to handle local mport devices */
static struct class_interface rio_mport_interface __refdata = {
	.class		= &rio_mport_class,
	.add_dev	= mport_add_mport,
	.remove_dev	= mport_remove_mport,
};

/*
 * Linux kernel module
 */

/*
 * mport_init - Driver module loading
 */
static int __init mport_init(void)
{
	int ret;

	spin_lock_init(&dbell_i_lock);
	spin_lock_init(&dbell_list_lock);
	INIT_LIST_HEAD(&dbell_list);

	/* Create device class needed by udev */
	dev_class = class_create(THIS_MODULE, DRV_NAME);
	if (!dev_class) {
		pr_err(DRV_PREFIX "Cannot create " DRV_NAME " class\n");
		return -EINVAL;
	}

	ret = alloc_chrdev_region(&dev_number, 0, RIO_MAX_MPORTS, DRV_NAME);
	if (ret < 0)
		return ret;

	pr_debug(DRV_PREFIX "Registered class with %d major\n", MAJOR(dev_number));

	/* Register to rio_mport_interface */
	ret = class_interface_register(&rio_mport_interface);
	if (ret) {
		pr_warn(DRV_PREFIX
			"class_interface_register error: %d\n", ret);
		return -EINVAL;
	}

	return 0;
}

/**
 * mport_exit - Driver module unloading
 */
static void __exit mport_exit(void)
{
	class_interface_unregister(&rio_mport_interface);
	class_destroy(dev_class);
	unregister_chrdev_region(dev_number, RIO_MAX_MPORTS);
}

module_init(mport_init);
module_exit(mport_exit);
