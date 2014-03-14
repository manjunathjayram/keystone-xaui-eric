/*
 * RapidIO mport character device
 *
 * Copyright 2013 Prodrive B.V.
 * Jerry Jacobs <jerry.jacobs@prodrive.nl>
 *
 * Original version was modified by Alex Bounine <alexandre.bounine@idt.com>
 * for testing of RIO_CM RapidIO Messaging Chanel Management driver.
 * 
 * Copyright (C) 2014 Texas Instruments Incorporated
 * Aurelien Jacquiot <a-jacquiot@ti.com>
 * - Introduced DirectI/O, doorbell and basic maintenance operations
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

#include <linux/rio.h>
#include <linux/rio_ids.h>
#include <linux/rio_drv.h>
#include <linux/rio_mport_cdev.h>

#define DRV_NAME        "rio-mport-cdev"
#define DRV_AUTHOR      "Jerry Jacobs <jerry.jacobs@prodrive.nl>"
#define DRV_DESC        "RapidIO mport character device driver"

MODULE_AUTHOR(DRV_AUTHOR);
MODULE_DESCRIPTION(DRV_DESC);
MODULE_LICENSE("GPL");

#define DRV_PREFIX		"RIO " DRV_NAME ": "
#define DEV_NAME		"rio_mport"

/*
 * An mport_dev represents a structure on mport
 * @node	List node to maintain list of registered mports
 * @cdev	Character device
 * @dev		Associated device object
 * @mport	Associated master port
 */
struct mport_dev {
	struct list_head  node;
	struct cdev	  cdev;
	struct device	 *dev;
	struct rio_mport *mport;
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
 * @data:	Driver private data
 * @arg:	Dbell information
 */
static int dbell_send(struct mport_dev *data, void __user *arg)
{
	struct rio_mport_dbell dbell;
	int ret;

	if (copy_from_user(&dbell, arg, sizeof(struct rio_mport_dbell)))
		return -EFAULT;

	if (!data->mport->ops->dsend)
		return -EPROTONOSUPPORT;

	pr_debug(DRV_PREFIX "Send doorbell %d to dest Id %d\n", dbell.id, dbell.num);

	ret = data->mport->ops->dsend(data->mport,
				      data->mport->id,
				      dbell.id,
				      dbell.num);

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
 * dbell_send() - Send a doorbell
 * @data:	Driver private data
 * @arg:	Dbell information
 */
static int dbell_receive(struct mport_dev *data, void __user *arg)
{
	struct rio_mport_dbell dbell;
	int ret;

	if (copy_from_user(&dbell, arg, sizeof(struct rio_mport_dbell)))
		return -EFAULT;

	pr_debug(DRV_PREFIX "Wait doorbell %d\n", dbell.num);

	ret = dbell_wait(data->mport, dbell.num, &dbell.id);

	if (copy_to_user(arg, &dbell, sizeof(struct rio_mport_dbell)))
		ret = -EFAULT;

	return ret;
}

#ifdef CONFIG_TI_KEYSTONE_RAPIDIO
/*
 * dio_transfer() - Perform a DirectI/O transfer
 * @data:	Driver private data
 * @arg:	DirectI/O transfer information
 */
static int dio_transfer(struct mport_dev *data, void __user *arg)
{
	struct rio_mport_dio_transfer dio_transfer;
	int ret;
	void *vaddr;

	if (copy_from_user(&dio_transfer, arg, sizeof(struct rio_mport_dio_transfer)))
		return -EFAULT;

	if (!data->mport->ops->transfer)
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

	ret = data->mport->ops->transfer(data->mport,
					 data->mport->id,
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
 * @data:	Driver private data
 * @arg:	Maintenance transaction information
 */
static int maint_lconfig_read(struct mport_dev *data, void __user *arg)
{
	struct rio_mport_maint_transfer maint_transfer;
	int ret;
	void *buf;

	if (copy_from_user(&maint_transfer, arg, sizeof(struct rio_mport_maint_transfer)))
		return -EFAULT;

	if (!data->mport->ops->lcread)
		return -EPROTONOSUPPORT;

	pr_debug(DRV_PREFIX "Perform a local maintenance read at offset %d\n",
		 maint_transfer.offset);

	buf = kmalloc(maint_transfer.length, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = data->mport->ops->lcread(data->mport,
				       data->mport->id,
				       maint_transfer.offset,
				       maint_transfer.length,
				       (u32 *) buf);

	if (copy_to_user(maint_transfer.val, buf, maint_transfer.length))
		return -EFAULT;

	return ret;
}

/*
 * maint_lconfig_write() - Perform a local config space write transaction
 * @data:	Driver private data
 * @arg:	Maintenance transaction information
 */
static int maint_lconfig_write(struct mport_dev *data, void __user *arg)
{
	struct rio_mport_maint_transfer maint_transfer;
	int ret;
	u32 val;

	if (copy_from_user(&maint_transfer, arg, sizeof(struct rio_mport_maint_transfer)))
		return -EFAULT;

	if (!data->mport->ops->lcwrite)
		return -EPROTONOSUPPORT;

	pr_debug(DRV_PREFIX "Perform a local maintenance read at offset %d\n",
		 maint_transfer.offset);

	if (copy_from_user(&val, maint_transfer.val, sizeof(data)))
		return -EFAULT;

	ret = data->mport->ops->lcwrite(data->mport,
					data->mport->id,
					maint_transfer.offset,
					maint_transfer.length,
					val);

	return ret;
}

/*
 * maint_config_read() - Perform a remote config space read transaction
 * @data:	Driver private data
 * @arg:	Maintenance transaction information
 */
static int maint_config_read(struct mport_dev *data, void __user *arg)
{
	struct rio_mport_maint_transfer maint_transfer;
	int ret;
	void *buf;

	if (copy_from_user(&maint_transfer, arg, sizeof(struct rio_mport_maint_transfer)))
		return -EFAULT;

	if (!data->mport->ops->cread)
		return -EPROTONOSUPPORT;

	pr_debug(DRV_PREFIX "Perform a remote maintenance read at offset %d\n",
		 maint_transfer.offset);

	buf = kmalloc(maint_transfer.length, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = data->mport->ops->cread(data->mport,
				      data->mport->id,
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
 * @data:	Driver private data
 * @arg:	Maintenance transaction information
 */
static int maint_config_write(struct mport_dev *data, void __user *arg)
{
	struct rio_mport_maint_transfer maint_transfer;
	int ret;
	u32 val;

	if (copy_from_user(&maint_transfer, arg, sizeof(struct rio_mport_maint_transfer)))
		return -EFAULT;

	if (!data->mport->ops->cwrite)
		return -EPROTONOSUPPORT;

	pr_debug(DRV_PREFIX "Perform a remote maintenance read at offset %d\n",
		 maint_transfer.offset);

	if (copy_from_user(&val, maint_transfer.val, sizeof(data)))
		return -EFAULT;

	ret = data->mport->ops->cwrite(data->mport,
				       data->mport->id,
				       maint_transfer.id,
				       maint_transfer.hopcount,
				       maint_transfer.offset,
				       maint_transfer.length,
				       val);

	return ret;
}

/*
 * maint_hdid_set() - Set the host Device ID
 * @data:	Driver private data
 * @arg:	Device Id
 */
static int maint_hdid_set(struct mport_dev *data, void __user *arg)
{
	uint16_t hdid;

	if (copy_from_user(&hdid, arg, sizeof(uint16_t)))
		return -EFAULT;

	data->mport->host_deviceid = hdid;
	rio_local_set_device_id(data->mport, hdid);
	
	pr_debug(DRV_PREFIX "Set host device Id to %d\n", hdid);

	return 0;
}

/*
 * maint_comptag_set() - Set the host Component Tag
 * @data:	Driver private data
 * @arg:	Component Tag
 */
static int maint_comptag_set(struct mport_dev *data, void __user *arg)
{
	uint32_t comptag;

	if (copy_from_user(&comptag, arg, sizeof(uint32_t)))
		return -EFAULT;

	rio_local_write_config_32(data->mport, RIO_COMPONENT_TAG_CSR, comptag);

	pr_debug(DRV_PREFIX "Set host Component Tag to %d\n", comptag);

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

	/* Test for valid device */
	if (minor >= RIO_MAX_MPORTS) {
		pr_err(DRV_PREFIX "Invalid minor device number\n");
		return -EINVAL;
	}

	chdev = container_of(inode->i_cdev, struct mport_dev, cdev);
	filp->private_data = chdev;

	return 0;
}

/*
 * mport_cdev_release() - Release character device
 */
static int mport_cdev_release(struct inode *inode, struct file *filp)
{
	return 0;
}

/*
 * mport_cdev_ioctl() - IOCTLs for character device
 */
static long mport_cdev_ioctl(struct file *filp,
		unsigned int cmd, unsigned long arg)
{
	int err = 0;
	struct mport_dev *data = filp->private_data;

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
		err = -EINVAL;
		break;
	case RIO_MPORT_DMA_WRITE:
		err = -EINVAL;
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
	default:
		err = -EINVAL;
		break;
	}

	return err;
}

unsigned int mport_poll(struct file *file, poll_table *wait)
{
	unsigned int mask = 0;
#if 0

	struct mport_dev *chdev = file->private_data;

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
