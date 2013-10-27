/*
 * RapidIO channel management on mports
 *
 * Copyright 2013 Prodrive B.V.
 * Jerry Jacobs <jerry.jacobs@prodrive.nl>
 *
 * Original version was modified by Alex Bounine <alexandre.bounine@idt.com>
 * for testing of RIO_CM RapidIO Messaging Chanel Management driver.
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

#include <linux/mm.h>
#include <linux/slab.h>

#include <linux/rio.h>
#include <linux/rio_ids.h>
#include <linux/rio_drv.h>
#include "../rio_cm.h"
#include "rio_mport.h"
#include "channel_msg.h"

#define DRV_NAME        "rio-mport-cdev"
#define DRV_AUTHOR      "Jerry Jacobs <jerry.jacobs@prodrive.nl>"
#define DRV_DESC        "RapidIO mport character device driver"

MODULE_AUTHOR(DRV_AUTHOR);
MODULE_DESCRIPTION(DRV_DESC);
MODULE_LICENSE("GPL");

#define DRV_PREFIX		"RIO " DRV_NAME ": "
#define DEV_NAME		"rio_mport"
#define CHANNEL_MINORS	256

/*
 * An channel_dev represents a structure on mport
 * @node	List node to maintain list of registered mports
 * @cdev	Character device
 * @dev		Associated device object
 * @mport	Associated master port
 */
struct channel_dev {
	struct list_head node;
	struct cdev	cdev;
	struct device	*dev;
	struct rio_mport *mport;
};

static LIST_HEAD(channel_devs);
static DEFINE_MUTEX(channel_devs_lock);
#if (0) /* used by commented out portion of poll function : FIXME */
static DECLARE_WAIT_QUEUE_HEAD(channel_cdev_wait);
#endif

static struct class *dev_class;
static int dev_count;
static int dev_major;
static dev_t dev_number;

/**
 * channel_ep_get_list_size() - Get amount of endpoints in network
 * @data:	Driver private data
 * @size:	List size
 */
static int channel_ep_get_list_size(struct channel_dev *data,
				void __user *size)
{
	u32 count = 0;
	int err;

	err = riocm_get_peer_count(data->mport, &count);
	if (err)
		return err;

	if (copy_to_user(size, &count, sizeof(count)))
		return -EFAULT;

	return 0;
}

/**
 * channel_ep_get_list() - Get next endpoint in network
 * @data:	Driver private data
 * @arg:	List of endpoint destination IDs in the network
 */
static int channel_ep_get_list(struct channel_dev *data,
		void __user *arg)
{
	int ret = 0;
	uint32_t entries = 0;
	void *buf;

	if (copy_from_user(&entries, arg, sizeof(entries)))
		return -EFAULT;

	buf = kcalloc(entries + 1, sizeof(u32), GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = riocm_get_peer_list(data->mport, (u8 *)buf + sizeof(u32), &entries);
	if (ret)
		goto out;

	if (copy_to_user(arg, buf, sizeof(u32) * (entries + 1)))
		ret = -EFAULT;
out:
	kfree(buf);
	return ret;
}

/**
 * channel_close() - Close channel
 * @data:	Driver private data
 * @arg:	Channel to close
 */
static int channel_close(struct channel_dev *data, void __user *arg)
{
	u16 __user *p = arg;
	u16 ch_num;

	if (get_user(ch_num, p))
		return -EFAULT;
	return riocm_ch_close(ch_num);
}

/**
 * channel_create() - Create channel
 * @data:	Driver private data
 * @arg:	Channel information
 */
static int channel_create(struct channel_dev *data, void __user *arg)
{
	u16 __user *p = arg;
	u16 ch_num;
	int ret;

	if (get_user(ch_num, p))
		return -EFAULT;
	ret = riocm_ch_create(&ch_num);
	if (ret)
		return ret;

	return put_user(ch_num, p);
}

/**
 * channel_bind() - Bind channel
 * @data:	Driver private data
 * @arg:	Channel number
 */
static int channel_bind(struct channel_dev *data, void __user *arg)
{
	u16 __user *p = arg;
	u16 ch_num;

	if (get_user(ch_num, p))
		return -EFAULT;

	return riocm_ch_bind(ch_num, data->mport, NULL);
}

/**
 * channel_listen() - Listen on channel
 * @data:	Driver private data
 * @arg:	Channel number
 */
static int channel_listen(struct channel_dev *data, void __user *arg)
{
	u16 __user *p = arg;
	u16 ch_num;

	if (get_user(ch_num, p))
		return -EFAULT;

	return riocm_ch_listen(ch_num);
}

/**
 * channel_accept() - Accept incomming connection
 * @data:	Driver private data
 * @arg:	Channel number
 */
static int channel_accept(struct channel_dev *data, void __user *arg)
{
	u16 __user *p = arg;
	u16 ch_num;
	int rc;

	if (get_user(ch_num, p))
		return -EFAULT;

	rc = riocm_ch_accept(ch_num, &ch_num, RIO_MPORT_CHANNEL_ACCEPT_TO * HZ);
	if (rc)
		return rc;

	return put_user(ch_num, p);
}

/**
 * channel_connect() - Connect on channel
 * @data:	Driver private data
 * @arg:	Channel information
 */
static int channel_connect(struct channel_dev *data, void __user *arg)
{
	int ret = 0;
	struct rio_mport_channel chan;
	struct rio_mport *mport = data->mport;

	if (copy_from_user(&chan, arg, sizeof(struct rio_mport_channel))) {
		ret = -EFAULT;
		goto err;
	}

	ret = riocm_ch_connect(chan.id, mport, chan.remote_destid, chan.remote_channel);

err:
	return ret;
}

/**
 * channel_msg_send() - Connect on channel
 * @data:	Driver private data
 * @arg:	Outbound message information
 */
static int channel_msg_send(struct channel_dev *data, void __user *arg)
{
	struct rio_mport_ch_msg msg;
	void *buf;
	int ret = 0;

	if (copy_from_user(&msg, arg, sizeof(struct rio_mport_ch_msg)))
		return -EFAULT;

	pr_debug(DRV_PREFIX "TX msg from %p size=%d through ch %d\n", msg.msg, msg.size, msg.ch_num);

	buf = kmalloc(RIO_MAX_MSG_SIZE, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	if (copy_from_user(buf, msg.msg, msg.size)) {
		ret = -EFAULT;
		goto out;
	}

	pr_debug(DRV_PREFIX "TX MSG: %s\n", (char *)buf + 20);
	ret = riocm_ch_send(msg.ch_num, buf, msg.size);
out:
	kfree(buf);
	return ret;
}

/*
 * channel_msg_rcv() - Connect on channel
 * @data:	Driver private data
 * @arg:	Inbound message information
 */
static int channel_msg_rcv(struct channel_dev *data, void __user *arg)
{
	struct rio_mport_ch_msg msg;
	void *buf;
	int msg_len = RIO_MAX_MSG_SIZE;
	int ret = 0;

	if (copy_from_user(&msg, arg, sizeof(struct rio_mport_ch_msg)))
		return -EFAULT;

	pr_debug(DRV_PREFIX "RX msg into %p size=%d through ch %d\n", msg.msg, msg.size, msg.ch_num);

	ret = riocm_ch_receive(msg.ch_num, &buf, &msg_len);
	if (ret)
		goto out;

	if (copy_to_user(msg.msg, buf, RIO_MAX_MSG_SIZE)) {  // check msg.size for max allowed copy size ???
		ret = -EFAULT;
		goto out;
	}

	//msg.size = RIO_MAX_MSG_SIZE;

	ret = riocm_ch_free_rxbuf(msg.ch_num, buf);
out:
	return ret;
}

/*
 * Mport cdev management
 */

/**
 * channel_cdev_open() - Open character device (mport)
 */
static int channel_cdev_open(struct inode *inode, struct file *filp)
{
	int minor = iminor(inode);
	struct channel_dev *chdev = NULL;

	/* Test for valid device */
	if (minor >= dev_count) {
		pr_err(DRV_PREFIX "Invalid minor device number\n");
		return -EINVAL;
	}

	chdev = container_of(inode->i_cdev, struct channel_dev, cdev);
	filp->private_data = chdev;

	return 0;
}

/**
 * channel_cdev_release() - Release character device
 */
static int channel_cdev_release(struct inode *inode, struct file *filp)
{
	return 0;
}

/**
 * channel_cdev_ioctl() - IOCTLs for character device
 */
static long channel_cdev_ioctl(struct file *filp,
		unsigned int cmd, unsigned long arg)
{
	int err = 0;
	struct channel_dev *data = filp->private_data;

	switch (cmd) {
	case RIO_MPORT_CHANNEL_EP_GET_LIST_SIZE:
		err = channel_ep_get_list_size(data, (void __user *)arg);
		break;
	case RIO_MPORT_CHANNEL_EP_GET_LIST:
		err = channel_ep_get_list(data, (void __user *)arg);
		break;
	case RIO_MPORT_CHANNEL_CREATE:
		err = channel_create(data, (void __user *)arg);
		break;
	case RIO_MPORT_CHANNEL_CLOSE:
		err =  channel_close(data, (void __user *)arg);
		break;
	case RIO_MPORT_CHANNEL_BIND:
		err = channel_bind(data, (void __user *)arg);
		break;
	case RIO_MPORT_CHANNEL_LISTEN:
		err = channel_listen(data, (void __user *)arg);
		break;
	case RIO_MPORT_CHANNEL_ACCEPT:
		err = channel_accept(data, (void __user *)arg);
		break;
	case RIO_MPORT_CHANNEL_CONNECT:
		err = channel_connect(data, (void __user *)arg);
		break;
	case RIO_MPORT_CHANNEL_SEND:
		err = channel_msg_send(data, (void __user *)arg);
		break;
	case RIO_MPORT_CHANNEL_RECEIVE:
		err = channel_msg_rcv(data, (void __user *)arg);
		break;
	default:
		err = -EINVAL;
		break;
	}

	return err;
}

unsigned int channel_poll(struct file *file, poll_table *wait)
{
	unsigned int mask = 0;
#if 0
	struct channel_dev *chdev = file->private_data;

	poll_wait(file, &channel_cdev_wait, wait);

	if (chdev->rx_head != chdev->rx_tail)
		mask |= POLLIN | POLLRDNORM;

#endif
	return mask;
}

static const struct file_operations channel_fops = {
	.owner		= THIS_MODULE,
	.open		= channel_cdev_open,
	.release	= channel_cdev_release,
	.poll		= channel_poll,
	.unlocked_ioctl = channel_cdev_ioctl
};

static int channel_cdev_init(void)
{
	int ret;
	int major;

	ret = alloc_chrdev_region(&dev_number, 0, CHANNEL_MINORS, DRV_NAME);
	if (ret < 0)
		return ret;

	major = MAJOR(dev_number);

	return major;
}

/*
 * Character device management
 */

/**
 * channel_cdev_add() - Create channel_dev from rio_mport
 * @mport:	RapidIO master port
 * TODO add created channel_devs to global list
 */
static struct channel_dev *channel_cdev_add(struct rio_mport *mport)
{
	int ret = 0;
	dev_t devno;
	struct channel_dev *device;

	device = kzalloc(sizeof(struct channel_dev), GFP_KERNEL);
	device->mport = mport;

	/* Initialize cdev structure in crypter_dev structure,
	 * set owner, operations and register it */
	devno = MKDEV(dev_major, dev_count);
	cdev_init(&device->cdev, &channel_fops);
	device->cdev.owner = THIS_MODULE;
	device->cdev.ops = &channel_fops;
	ret = cdev_add(&device->cdev, devno, 1);
	if (ret < 0) {
		if (device) {
			cdev_del(&device->cdev);
			kfree(device);
		}
		pr_err(DRV_PREFIX
			"Cannot register a device with error %d\n", ret);
		return NULL;
	}

	/* Setup sysfs */
	device->dev = device_create(dev_class, NULL, devno,
			NULL, DEV_NAME "%d", dev_count);
	if (IS_ERR(device->dev)) {
		kfree(device);
		return NULL;
	}

	mutex_lock(&channel_devs_lock);
	list_add_tail(&device->node, &channel_devs);
	dev_count++;
	mutex_unlock(&channel_devs_lock);

	pr_info(DRV_PREFIX "Added %s cdev(%d:%d)\n",
		mport->name, dev_major, dev_count);

	return device;
}

/**
 * channel_dev_remove() - Remove channel character device from channel_devs
 * @dev:	Channel device to remove
 */
static void channel_dev_remove(struct channel_dev *dev)
{
	if (!dev)
		return;

	pr_info(DRV_PREFIX "Removing %s cdev\n", dev->mport->name);
	device_unregister(dev->dev);
	cdev_del(&(dev->cdev));
	kfree(dev);
}

/*
 * RIO rio_mport_interface driver
 */

/**
 * channel_add_mport() - Add rio_mport from LDM device struct
 * @dev:		Linux device model struct
 * @class_intf:	Linux class_interface
 */
static int channel_add_mport(struct device *dev,
		struct class_interface *class_intf)
{
	struct rio_mport *mport = NULL;
	struct channel_dev *chdev = NULL;

	mport = to_rio_mport(dev);
	if (!mport)
		return -ENODEV;

	chdev = channel_cdev_add(mport);
	if (!chdev)
		return -ENODEV;

	return 0;
}

/**
 * channel_remove_mport() - Remove rio_mport from global list
 * TODO remove device from global channel_dev list
 */
static void channel_remove_mport(struct device *dev,
		struct class_interface *class_intf)
{
	struct rio_mport *mport = NULL;
	struct channel_dev *chdev;
	mport = to_rio_mport(dev);
	if (!mport)
		return;

	mutex_lock(&channel_devs_lock);
	list_for_each_entry(chdev, &channel_devs, node) {
		if (chdev->mport->id == mport->id)
			goto found;
	}
found:
	/* TODO cleanup */
	mutex_unlock(&channel_devs_lock);
}

/* the rio_mport_interface is used to handle local mport devices */
static struct class_interface rio_mport_interface __refdata = {
	.class		= &rio_mport_class,
	.add_dev	= channel_add_mport,
	.remove_dev	= channel_remove_mport,
};

/*
 * Linux kernel module
 */

/**
 * channel_init - Driver module loading
 */
static int __init channel_init(void)
{
	int ret;

	/* Create device class needed by udev */
	dev_class = class_create(THIS_MODULE, DRV_NAME);
	if (!dev_class) {
		pr_err(DRV_PREFIX "Cannot create " DRV_NAME " class\n");
		return -EINVAL;
	}

	/* Get class major number */
	dev_major = channel_cdev_init();
	if (dev_major < 0)
		return -ENOMEM;

	pr_debug(DRV_PREFIX "Registered class with %d major\n", dev_major);

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
 * channel_exit - Driver module unloading
 */
static void __exit channel_exit(void)
{
	struct channel_dev *chdev;

	/* Cleanup channel devices */
	list_for_each_entry(chdev, &channel_devs, node)
		channel_dev_remove(chdev);

	unregister_chrdev_region(dev_number, CHANNEL_MINORS);
	class_destroy(dev_class);
	class_interface_unregister(&rio_mport_interface);
}

module_init(channel_init);
module_exit(channel_exit);
