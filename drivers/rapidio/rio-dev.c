/*
 * RapidIO userspace interface for Direct I/O and doorbells
 *
 * Copyright (C) 2010, 2013, 2014 Texas Instruments Incorporated
 * Author: Aurelien Jacquiot <a-jacquiot@ti.com>
 *
 * DMA engine based data transfer borrowed from rio_mport_cdev.c
 * Copyright 2014 Integrated Device Technology, Inc.
 * Alexandre Bounine <alexandre.bounine@idt.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/rio.h>
#include <linux/rio_drv.h>
#include <linux/rio_ids.h>
#include <linux/rio_regs.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <asm/uaccess.h>

#include "rio.h"

#include <linux/rio_dev.h>

/*
 * This supports acccess to RapidIO devices using normal userspace I/O calls.
 *
 * RapidIO has a character major number assigned. We allocate minor numbers
 * dynamically using a bitmask. You must use hotplug tools, such as udev
 * (or mdev with busybox) to create and destroy the /dev/rio0.1 device
 * nodes, since there is no fixed association of minor numbers with any
 * particular RapidIO site or device.
 */
#define RIO_DEV_MAJOR 	154	/* assigned */
#define RIO_DEV_NAME    "rio"
#define N_RIO_MINORS	32	/* number of minors per instance (up to 256) */

#define DRV_NAME	"riodev"
#define DRV_PREFIX	"RIO " DRV_NAME ": "

static unsigned long minors[N_RIO_MINORS / BITS_PER_LONG];
static unsigned long init_done = 0;

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);
static spinlock_t       dbell_i_lock;
static spinlock_t       dbell_list_lock;
static struct list_head dbell_list;

struct dbell_cell {
	struct list_head   node;
	u16                info;
	wait_queue_head_t  waitq;
};

struct rio_dev_private {
	struct rio_dev        *rdev;
	struct dma_chan       *dma_chan;
	u16                    write_mode;
	u64                    base_offset;
	struct rio_mport_attr  attr;
};

/* DMA transfer timeout in msec */
static int timeout = 3000;
module_param(timeout, int, S_IRUGO);
MODULE_PARM_DESC(timeout, "DMA Transfer Timeout in msec (default: 3000)");

/* Use DMA scatter-gather (even if driver has not hw support for SG) */
static bool dma_use_sg = 0;
module_param(dma_use_sg, bool, S_IRUGO);
MODULE_PARM_DESC(dma_use_sg, "DMA Use DMA scatter-gather (default: 0)");

#define MIN(a, b) ((a) < (b) ? (a) : (b))

static void rio_dev_dma_callback(void *completion)
{
	complete(completion);
}

static struct dma_async_tx_descriptor *rio_dev_prep_dma_xfer(
	struct dma_chan *chan, u64 tgt_addr, u16 mode, u16 dest_id,
	struct sg_table *sgt, int nents, enum dma_transfer_direction dir,
	enum dma_ctrl_flags flags)
{
	struct rio_dma_data tx_data;
	struct dma_async_tx_descriptor *tx = NULL;

	tx_data.sg = sgt->sgl;
	tx_data.sg_len = nents;
	tx_data.rio_addr_u = 0;
	tx_data.rio_addr = tgt_addr;

	if (dir == DMA_MEM_TO_DEV) {
		switch(mode & 0xf) {
		case RIO_DIO_MODE_SWRITE:
		case RIO_DIO_MODE_WRITE:
			tx_data.wr_type = RDW_ALL_NWRITE;
			break;
		case RIO_DIO_MODE_WRITER:
			tx_data.wr_type = RDW_ALL_NWRITE_R;
			break;
		default:
			tx_data.wr_type = RDW_DEFAULT;
			break;
		}
	}

	tx = rio_dma_prep_slave_sg(chan, dest_id, &tx_data, dir, flags);

	return tx;
}

static int rio_dev_do_dma_request(struct dma_chan *chan, struct rio_mport *mport,
				  struct sg_table *sgt, u64 tgt_addr, u16 mode,
				  u16 dest_id, u32 length,
				  enum dma_transfer_direction direction)
{
	struct dma_async_tx_descriptor *tx;
	struct completion cmp;
	dma_cookie_t cookie;
	unsigned long tmo = msecs_to_jiffies(timeout);
	enum dma_status	status;
	int nents, ret = 0;

	nents = dma_map_sg(chan->device->dev, sgt->sgl, sgt->nents, direction);
	if (nents == -EFAULT) {
		pr_err(DRV_PREFIX "%s: Failed to map SG list\n", __func__);
		return -EFAULT;
	}

	/* Initialize DMA transaction request */
	tx = rio_dev_prep_dma_xfer(chan, tgt_addr, mode, dest_id, sgt, nents, direction,
				   DMA_CTRL_ACK | DMA_PREP_INTERRUPT);
	if (!tx) {
		pr_err(DRV_PREFIX "%s: prep error for %s A:0x%llx L:0x%x\n",
			__func__, (direction == DMA_DEV_TO_MEM)?"READ":"WRITE",
		       tgt_addr, length);
		ret = -EIO;
		goto err_out;
	}

	init_completion(&cmp);
	tx->callback = rio_dev_dma_callback;
	tx->callback_param = &cmp;

	/* Submit DMA transaction request */
	cookie = tx->tx_submit(tx);

	pr_debug(DRV_PREFIX "%s: DMA tx_cookie = %d\n", __func__, cookie);

	if (dma_submit_error(cookie)) {
		pr_err(DRV_PREFIX "%s: submit err=%d (addr:0x%llx len:0x%x)\n",
			__func__, cookie, tgt_addr, length);
		ret = -EIO;
		goto err_out;
	}

	dma_async_issue_pending(chan);

	tmo = wait_for_completion_interruptible_timeout(&cmp, tmo);
	status = dma_async_is_tx_complete(chan, cookie, NULL, NULL);

	if (tmo == -ERESTARTSYS) {
		ret = tmo;
	} else	if (tmo == 0) {
		pr_err(DRV_PREFIX "%s: timed out waiting for DMA\n", __func__);
		ret = -ETIMEDOUT;
	} else if (status != DMA_SUCCESS) {
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

static int rio_dev_dma_transfer(struct dma_chan *chan, struct rio_mport *mport,
				void __user *addr, u64 tgt_addr, u16 mode,
				u16 dest_id, u32 length, int dma_has_sg,
				enum dma_transfer_direction direction)
{
	unsigned int nr_pages = 0;
	struct page **page_list = NULL;
	struct sg_table sgt;
	int i, ret;

	if (length == 0)
		return -EINVAL;

	if (dma_has_sg) {
		unsigned long offset;
		long pinned;

		offset = (unsigned long) addr & ~PAGE_MASK;
		nr_pages = PAGE_ALIGN(length + offset) >> PAGE_SHIFT;

		page_list = kmalloc(nr_pages * sizeof(*page_list), GFP_KERNEL);
		if (page_list == NULL)
			return -ENOMEM;

		down_read(&current->mm->mmap_sem);
		pinned = get_user_pages(current, current->mm,
					(unsigned long) addr & PAGE_MASK,
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
						offset, length, GFP_KERNEL);
		if (ret) {
			pr_err(DRV_PREFIX "%s: sg_alloc_table failed with err=%d\n",
			       __func__, ret);
			goto err_out;
		}
	} else {
		ret = sg_alloc_table(&sgt, 1, GFP_KERNEL);
		if (unlikely(ret)) {
			pr_err("%s: sg_alloc_table failed for internal buf\n",
			       __func__);
			return ret;
		}

		/* There is only one buffer */
		sg_set_buf(sgt.sgl, (void*) addr, length);
	}

	ret = rio_dev_do_dma_request(chan, mport, &sgt, tgt_addr, mode,
				     dest_id, length, direction);

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
 * RapidIO File ops
 */
static loff_t rio_dev_llseek(struct file* filp, loff_t off, int whence)
{
	loff_t new;

	switch (whence) {
	case 0:	 new = off; break;
	case 1:	 new = filp->f_pos + off; break;
	case 2:	 new = 1 + off; break;
	default: return -EINVAL;
	}

	return (filp->f_pos = new);
}

static ssize_t rio_dev_write(struct file *filp, const char __user *buf,
			     size_t size, loff_t *ppos)
{
	struct rio_dev_private *rio_dev = filp->private_data;
	struct rio_dev *rdev = rio_dev->rdev;
	int dma_has_sg = rio_dev->attr.flags & RIO_MPORT_DMA_SG;
	int res;
	int write_pos = 0;
	size_t count = size;
	size_t write_sz;
	size_t transfer_size;
	char *src_buf = NULL;
	char *p;

	if (!size)
		return 0;

	pr_debug(DRV_PREFIX "%s: write buffer, p = 0x%x, size = %d\n",
		 __func__, (unsigned int) buf, size);

	if (dma_has_sg) {
		/* User zero-copy and scatter-gather user buffers */
		transfer_size = rio_dev->attr.dma_max_sge * PAGE_SIZE;
		p = (char *) buf;
	} else {
		/* Use a kernel copy and maximal DMA transfer size */
		transfer_size = rio_dev->attr.dma_max_size;
		p = src_buf = kmalloc(MIN(transfer_size, size), GFP_KERNEL);
		if (p == NULL)
			return -ENOMEM;
	}

	while(count) {
		write_sz = MIN(count, transfer_size);
	      	count -= write_sz;

		if (!dma_has_sg) {
			if (copy_from_user(p, buf + write_pos, write_sz)) {
				res = -EFAULT;
				goto out;
			}
		}

		pr_debug(DRV_PREFIX "%s: writing, size = %d, ppos = 0x%x, buf = 0x%x, mode = 0x%x\n",
			 __func__, write_sz, (u32) (*ppos + rio_dev->base_offset),
			 (unsigned int) p, rio_dev->write_mode);

		/* Start the DIO transfer */
		res = rio_dev_dma_transfer(rio_dev->dma_chan,
					   rdev->net->hport,
					   (void __user *) p,
					   *ppos + rio_dev->base_offset,
					   rio_dev->write_mode,
					   rdev->destid,
					   write_sz,
					   rio_dev->attr.flags & RIO_MPORT_DMA_SG,
					   DMA_MEM_TO_DEV);
		if (res) {
			pr_debug(DRV_PREFIX "%s: write transfer failed (%d)\n",
				 __func__, res);
			goto out;
		}

		if (dma_has_sg)
			p += write_sz;

		*ppos += (u64) write_sz;
		write_pos += write_sz;
	}

	pr_debug(DRV_PREFIX "%s: write finished, size = %d, ppos = 0x%llx\n",
		 __func__, size, *ppos + rio_dev->base_offset);

	res = size;
out:
	if (src_buf)
		kfree(src_buf);

	return res;
}

static ssize_t rio_dev_read(struct file *filp, char __user *buf,
			    size_t size, loff_t *ppos)
{
	struct rio_dev_private *rio_dev = filp->private_data;
	struct rio_dev *rdev = rio_dev->rdev;
	int dma_has_sg = rio_dev->attr.flags & RIO_MPORT_DMA_SG;
	int res;
	int read_pos = 0;
	size_t count = size;
	size_t read_sz;
	size_t transfer_size;
	char *dst_buf = NULL;
	char *p;

	if (!size)
		return 0;

	pr_debug(DRV_PREFIX "%s: read buffer, buf = 0x%x, size = %d\n",
		 __func__, (unsigned int) buf, size);

	if (dma_has_sg) {
		/* User zero-copy and scatter-gather user buffers */
		transfer_size = rio_dev->attr.dma_max_sge * PAGE_SIZE;
		p = (char *) buf;
	} else {
		/* Use a kernel copy and maximal DMA transfer size */
		transfer_size = rio_dev->attr.dma_max_size;
		p = dst_buf = kmalloc(MIN(transfer_size, size), GFP_KERNEL);
		if (p == NULL)
			return -ENOMEM;
	}

	while(count) {
		read_sz = MIN(count, transfer_size);
	      	count -= read_sz;

		pr_debug(DRV_PREFIX "%s: reading, size = %d, ppos = 0x%x, buf = 0x%x\n",
			 __func__, read_sz, (u32) (*ppos + rio_dev->base_offset),
			 (unsigned int) p);

		/* Start the DIO transfer */
		res = rio_dev_dma_transfer(rio_dev->dma_chan,
					   rdev->net->hport,
					   (void __user *) p,
					   *ppos + rio_dev->base_offset,
					   rio_dev->write_mode,
					   rdev->destid,
					   read_sz,
					   rio_dev->attr.flags & RIO_MPORT_DMA_SG,
					   DMA_DEV_TO_MEM);
		if (res) {
			pr_debug(DRV_PREFIX "%s: write transfer failed (%d)\n",
				 __func__, res);
			goto out;
		}


		if (!dma_has_sg) {
			if (copy_to_user(buf + read_pos, p, read_sz)) {
				res = -EFAULT;
				goto out;
			}
		} else {
			p += read_sz;
		}

		*ppos += (u64) read_sz;
		read_pos += read_sz;
	}

	pr_debug(DRV_PREFIX "%s: read finished, size = %d, ppos = 0x%llx\n",
		 __func__, size, *ppos + rio_dev->base_offset);

	res = size;
out:
	if (dst_buf)
		kfree(dst_buf);

	return res;
}

static void rio_dev_dbell_callback(struct rio_mport *mport,
				   void *dev_id,
				   u16 src,
				   u16 dst,
				   u16 info)
{
	wait_queue_head_t *dbell_waitq = (wait_queue_head_t*) dev_id;

	/* Wake up user process */
	if (waitqueue_active(dbell_waitq))
		wake_up_all(dbell_waitq);
}

static struct dbell_cell* rio_dev_dbell_lookup(u16 info)
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
	dbell = (struct dbell_cell*) kmalloc(sizeof(struct dbell_cell), GFP_KERNEL);
	if (dbell == NULL)
		goto out;

	dbell->info = info;
	init_waitqueue_head(&dbell->waitq);

	list_add_tail(&dbell->node, &dbell_list);
out:
	spin_unlock(&dbell_list_lock);

	return dbell;
}

static int rio_dev_dbell_release(u16 info)
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

static int rio_dev_dbell_wait(struct rio_dev *rdev, u16 info)
{
	struct rio_mport *mport = rdev->net->hport;
	struct dbell_cell *dbell;
	unsigned long flags;
	int res;

	DECLARE_WAITQUEUE(wait, current);

	dbell = rio_dev_dbell_lookup(info);
	if (dbell == NULL)
		return -ENOMEM;

	/* Request a doorbell with our callback handler */
	res = rio_request_inb_dbell(mport,
				    (void *) &dbell->waitq,
				    info,
				    info,
				    rio_dev_dbell_callback);

	if ((res != 0) && (res != -EBUSY)) {
		pr_err(DRV_PREFIX "%s: cannot request such doorbell (info = %d)\n",
		       __func__, info);
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

	/* Release the doorbell */
	rio_release_inb_dbell(mport, info, info);
	rio_dev_dbell_release(info);

	if (signal_pending(current))
		return -ERESTARTSYS;

	pr_debug(DRV_PREFIX "%s: receiving doorbell (info = %d)\n", __func__, info);

	return info;
}

static long rio_dev_ioctl(struct file *filp,
			  unsigned int cmd,
			  unsigned long arg)
{
	struct rio_dev_private *rio_dev = filp->private_data;
	struct rio_dev *rdev = rio_dev->rdev;
	u32 dbell_info;
	int mode;
	int status = 0;
	u32 base;

	switch (cmd) {

	case RIO_DIO_BASE_SET:
		if (get_user(base, (u32 *) arg)) {
                        status = -EFAULT;
			break;
		}
		rio_dev->base_offset = base;
		break;

	case RIO_DIO_BASE_GET:
		base = rio_dev->base_offset;
		if (put_user(base, (u32 *) arg)) {
                        status = -EFAULT;
			break;
		}
		break;

	case RIO_DIO_MODE_SET:
		if (get_user(mode, (int *) arg)) {
                        status = -EFAULT;
			break;
		}
		rio_dev->write_mode = (u16) mode;
		break;

	case RIO_DIO_MODE_GET:
		mode = rio_dev->write_mode;
		if (put_user(mode, (u32 *) arg)) {
                        status = -EFAULT;
			break;
		}
		break;

	case RIO_DBELL_TX:
		if (get_user(dbell_info, (int *) arg)) {
                        status = -EFAULT;
			break;
		}

		/* Send a doorbell */
		if (rdev->net->hport->ops->dsend) {
			status = rdev->net->hport->ops->dsend(rdev->net->hport,
							      rdev->net->hport->id,
							      rdev->destid,
							      (u16) dbell_info);
		} else {
			status = -EPROTONOSUPPORT;
			pr_debug(DRV_PREFIX "%s: no dsend method\n", __func__);
		}
		break;

	case RIO_DBELL_RX:
		if (get_user(dbell_info, (int *) arg)) {
                        status = -EFAULT;
			break;
		}

		/* Wait a doorbell */
		status = rio_dev_dbell_wait(rdev, (u16) dbell_info);
		break;

	default:
		status = -EINVAL;
        }
	return status;
}

static int rio_dev_open(struct inode *inode, struct file *filp)
{
	struct rio_dev_private *rio_dev;
	struct rio_mport *mport;
	struct rio_dev *rdev;

	rdev = rio_get_devt(inode->i_rdev, NULL);
	if (rdev == NULL)
		return -ENXIO;

	rdev = rio_dev_get(rdev);
	if (rdev == NULL)
		return -ENXIO;

	mport = rdev->net->hport;

	rio_dev = (struct rio_dev_private *) kmalloc(sizeof(*rio_dev), GFP_KERNEL);
	if (rio_dev == NULL) {
		rio_dev_put(rdev);
		return -ENOMEM;
	}

	if (rio_query_mport(mport, &rio_dev->attr)) {
		dev_err(&rdev->dev, "cannot get mport device attributes\n");
		rio_dev_put(rdev);
		kfree(rio_dev);
		return -ENOTSUPP;
	}

	if (!rio_dev->attr.flags & RIO_MPORT_DMA) {
		dev_err(&rdev->dev, "mport device does not support DMA\n");
		rio_dev_put(rdev);
		kfree(rio_dev);
		return -ENOTSUPP;
	}

	/* If want to use SG and we have support SG list entries, force HW SG mode */
	if (dma_use_sg && (rio_dev->attr.dma_max_sge != 1))
		rio_dev->attr.flags |= RIO_MPORT_DMA_SG;

	rio_dev->rdev        = rdev;
	rio_dev->base_offset = 0;
	rio_dev->write_mode  = RIO_DIO_MODE_WRITER;

	rio_dev->dma_chan    = rio_request_dma(mport);
	if (rio_dev->dma_chan == NULL) {
		dev_err(&rdev->dev, "cannot get DMA channel\n");
		rio_dev_put(rdev);
		kfree(rio_dev);
		return -ENXIO;
	}

	filp->private_data = rio_dev;

	return 0;
}

static int rio_dev_release(struct inode *inode, struct file *filp)
{
	struct rio_dev_private *rio_dev = filp->private_data;
	int status = 0;

	if (rio_dev->dma_chan)
		dma_release_channel(rio_dev->dma_chan);

	rio_dev_put(rio_dev->rdev);

	kfree(rio_dev);

	filp->private_data = NULL;

	return status;
}

static struct file_operations rio_dev_fops = {
	.owner =	  THIS_MODULE,
	.llseek =         rio_dev_llseek,
	.write =	  rio_dev_write,
	.read =		  rio_dev_read,
	.unlocked_ioctl = rio_dev_ioctl,
	.open =		  rio_dev_open,
	.release =	  rio_dev_release,
};

/* The main reason to have this class is to make mdev/udev create the
 * /dev/rio0.0 character device nodes exposing our userspace API.
 * It also simplifies memory management.
 */
static void rio_dev_classdev_release(struct device *dev)
{
}

static struct class rio_dev_class = {
	.name	     = "rio_dev",
	.owner	     = THIS_MODULE,
	.dev_release = rio_dev_classdev_release,
};

/*
 * Called when adding a site, this will create the corresponding char device
 * with udev/mdev
 */
static int rio_dev_add(struct device *dev, struct subsys_interface *sif)
{
	struct rio_dev *rdev = to_rio_dev(dev);
	struct rio_mport *port = rdev->net->hport;
	int status;
	unsigned long minor;

	/* Check if the remote device is a switch */
	if (rdev->pef & RIO_PEF_SWITCH)
		return 0;

	/*
	 * If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */
	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_RIO_MINORS);

	if (minor < N_RIO_MINORS) {
		struct device *dev;
		rdev->dev.devt = MKDEV(RIO_DEV_MAJOR, minor);
		dev = device_create(&rio_dev_class,
				    &rdev->dev,
				    rdev->dev.devt,
				    rdev,
				    "%s%d.%d",
				    RIO_DEV_NAME,
				    port->index,
				    rdev->destid);

		status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	} else {
		pr_debug(DRV_PREFIX "%s: no minor number available!\n", __func__);
		status = -ENODEV;
	}

	if (status == 0)
		set_bit(minor, minors);

	mutex_unlock(&device_list_lock);

	return status;
}

static int rio_dev_remove(struct device *dev, struct subsys_interface *sif)
{
	struct rio_dev *rdev = to_rio_dev(dev);
	mutex_lock(&device_list_lock);
	device_destroy(&rio_dev_class, rdev->dev.devt);
	clear_bit(MINOR(rdev->dev.devt), minors);
	mutex_unlock(&device_list_lock);

	return 0;
}

static struct subsys_interface rio_dev_interface = {
	.name		= DRV_NAME,
	.subsys		= &rio_bus_type,
	.add_dev	= rio_dev_add,
	.remove_dev	= rio_dev_remove,
};

static int __init rio_dev_init(void)
{
	int status;

	spin_lock_init(&dbell_i_lock);
	spin_lock_init(&dbell_list_lock);
	INIT_LIST_HEAD(&dbell_list);

	/*
	 * Claim our 256 reserved device numbers.  Then register a class
	 * that will key udev/mdev to add/remove /dev nodes. Last, register
	 * the driver which manages those device numbers.
	 */
	BUILD_BUG_ON(N_RIO_MINORS > 256);
	status = register_chrdev(RIO_DEV_MAJOR, RIO_DEV_NAME, &rio_dev_fops);
	if (status < 0)
		return status;

	status = class_register(&rio_dev_class);
	if (status < 0) {
		unregister_chrdev(RIO_DEV_MAJOR, RIO_DEV_NAME);
		return status;
	}

	status = subsys_interface_register(&rio_dev_interface);
	if (status) {
		class_unregister(&rio_dev_class);
		unregister_chrdev(RIO_DEV_MAJOR, RIO_DEV_NAME);
		return status;
	}

	init_done = 1;

	return 0;
}

static void __exit rio_dev_exit(void)
{
	subsys_interface_unregister(&rio_dev_interface);
	class_unregister(&rio_dev_class);
	unregister_chrdev(RIO_DEV_MAJOR, RIO_DEV_NAME);
	init_done = 0;
}

late_initcall(rio_dev_init);
module_exit(rio_dev_exit);
