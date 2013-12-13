/*
 * RapidIO channel management on mports
 *
 * Copyright 2013 Prodrive B.V.
 * Jerry Jacobs <jerry.jacobs@prodrive.nl>
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
//#include <linux/rio_buff.h>
#include <linux/rio_drv.h>

#include "../rio_cm.h"

#include "rio_mport.h"

#include "channel_msg.h"

#define DRV_NAME        "rio-mport-channel"
#define DRV_AUTHOR      "Jerry Jacobs <jerry.jacobs@prodrive.nl>"
#define DRV_DESC        "RapidIO driver to manage channels on mports"

MODULE_AUTHOR(DRV_AUTHOR);
MODULE_DESCRIPTION(DRV_DESC);
MODULE_LICENSE("GPL");

#define DRV_PREFIX		"RIO " DRV_NAME ": "
#define DEV_NAME		"rio_mport"
#define CHANNEL_MINORS	256
#define CHANNELS_MAX	65535	/* 16 bit channel IDs */

#define CHANNEL_MAILBOX			0

/*
 * An channel_dev represents a structure on mport
 * @mport	Associated master port
 * @cdev	Character device
 * @state	Channel state
 * @rdev	Channels associated rio_dev
 * @rx_lock	Receive spinlock
 * @rx_head	buffer enqueue index (hardware)
 * @rx_tail	buffer dequeue index (user-space)
 * @rx_dropped	Dropped rx packet counter
 * @buffer	Receive ringbuffer
 */
struct channel_dev {
	struct list_head node;
	struct cdev		cdev;
	struct device	*dev;
	struct rio_mport *mport;
	socket_state	state[CHANNELS_MAX];
	struct rio_dev	*rdev[CHANNELS_MAX];
	struct semaphore sem;
	spinlock_t lock;
	spinlock_t tx_lock;
//	struct rio_buffer rx_buffer;
//	struct rio_buffer tx_buffer;

	int tx_slot;
	void *tx_ack[RIO_MPORT_CHANNEL_TX_RING];
};

static LIST_HEAD(channel_devs);
static DEFINE_MUTEX(channel_devs_lock);
static DECLARE_WAIT_QUEUE_HEAD(channel_cdev_wait);

static struct class *dev_class;
static int dev_count;
static int dev_major;
static dev_t dev_number;

void channel_queue_tx_msg(struct channel_dev *chdev,
	struct rio_mport *mport, struct rio_dev *dev,
	void *buffer, size_t size)
{
	chdev->tx_slot++;
	if (chdev->tx_slot >= RIO_MPORT_CHANNEL_TX_RING)
		chdev->tx_slot = 0;
	pr_debug("Adding buffer 0x%p at slot %d\n", buffer, chdev->tx_slot);
	rio_add_outb_message(mport, dev, CHANNEL_MAILBOX,
		buffer, size);
	chdev->tx_ack[chdev->tx_slot] = buffer;
}

/*
 * Channel management
 */

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
 * @list:	List of endpoint destination IDs in the network
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
	int __user *p = arg;
	int ch_num;

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
	int __user *p = arg;
	int ch_num;

	if (get_user(ch_num, p))
		return -EFAULT;
	ch_num = riocm_ch_create(ch_num);
	return put_user(ch_num, p);
}

/**
 * channel_bind() - Bind channel
 * @data:	Driver private data
 */
static int channel_bind(struct channel_dev *data, void __user *arg)
{
	int __user *p = arg;
	int ch_num;

	if (get_user(ch_num, p))
		return -EFAULT;

	return riocm_ch_bind(ch_num, data->mport, NULL);
}

/**
 * channel_listen() - Listen on channel
 * @data:	Driver private data
 */
static int channel_listen(struct channel_dev *data, void __user *arg)
{
	int __user *p = arg;
	int ch_num;

	if (get_user(ch_num, p))
		return -EFAULT;

	return riocm_ch_listen(ch_num);
}

/**
 * channel_accept() - Accept incomming connection
 * @data:	Driver private data
 */
static int channel_accept(struct channel_dev *data, void __user *arg)
{
	int __user *p = arg;
	int ch_num;
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

#if (0) // AB
	struct rio_dev *dev;
	struct rio_mport_channel *channel;
	struct rio_channel_msg *msg;
	void *buffer = NULL;
	uint32_t channel_id;
	uint32_t src_id;

	channel = kmalloc(sizeof(struct rio_mport_channel), GFP_KERNEL);
	if (!channel) {
		ret = -ENOMEM;
		goto err;
	}

	if (copy_from_user(channel, arg, sizeof(struct rio_mport_channel))) {
		ret = -EFAULT;
		goto err;
	}

	ret = rio_buff_request_slot(&data->tx_buffer, &buffer, RIO_BUFF_FREE);
	if (ret < 0) {
		ret = -EBUSY;
		goto err;
	}

	channel_id = channel->id;
	if (channel_id > CHANNELS_MAX) {
		ret = -EADDRNOTAVAIL;
		goto err;
	}

	if (data->state[channel->id] != SS_UNCONNECTED) {
		ret = -EADDRINUSE;
		pr_debug(DRV_PREFIX
			"Channel %u currently in use\n", channel->id);
		goto err;
	}

	src_id = rio_local_get_device_id(data->mport);

	/* Search for rio_dev endpoint in network */
	list_for_each_entry(dev, &data->mport->net->devices, net_list) {
		if ((!rio_is_switch(dev)) &&
				(dev->destid == channel->remote_destid)) {
			data->rdev[channel->id] = dev;
			data->state[channel->id] = SS_CONNECTING;
			pr_debug(DRV_PREFIX "Connected on Channel %d to EP %d\n",
				channel->id,
				channel->remote_destid);

			/* Create header */
			msg = kzalloc(sizeof(*msg), GFP_KERNEL);
			if(!msg) {
				ret = -ENOMEM;
				goto err;
			}

			/* Create header */
			rio_msg_set_src_id(msg, src_id);
			rio_msg_set_dest_id(msg, channel->remote_destid);
			rio_msg_set_src_mbox(msg, 127);
			rio_msg_set_dest_mbox(msg, 127);
			rio_msg_set_type(msg, 127);
			rio_msg_set_sock_type(msg, 3);
			rio_msg_set_dest_chan(msg, 255);
			rio_msg_set_src_chan(msg, 255);

			/* Write header and data */
			memcpy(buffer, msg, sizeof(*msg));
			snprintf(buffer + RIO_MSG_HDR_SIZE, 32, "Connecting...");

			/* TODO example first handshake */
			channel_queue_tx_msg(data, data->mport, dev, buffer, 128);

			kfree(msg);

			return 0;
		}
	}

err:
	kfree(channel);
	return ret;
#endif
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
 * @arg:	Outbound message information
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

	/* Test for rio_net */
//	if (!chdev->mport->net) {
//		pr_warn(DRV_PREFIX "Network on mport%d is not enumerated\n",
//			chdev->mport->id);
//		return -ENETDOWN;
//	}

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
//	void __user *argp = (void __user *)arg;
//	int __user *p = argp;

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

#if (0) //AB
void channel_vma_open(struct vm_area_struct *vma)
{
	pr_debug(DRV_PREFIX "%s\n", __func__);
}

void channel_vma_close(struct vm_area_struct *vma)
{
	int ret;
	int state;
	struct rio_buff_vma_info *info = vma->vm_private_data;

	if (!info)
		return;

	pr_debug(DRV_PREFIX "%s\n\tinfo->buffer: %p\n"
			"\tinfo->slot: %p\n"
			"\tinfo->state: %d\n",
			__func__,
			info->buffer,
			info->slot,
			info->state
		);

	switch(info->state) {
		case RIO_BUFF_FREE:
			state = RIO_BUFF_FILLED;
#if 0
			rio_add_outb_message(data->mport, dev, 0,
				msg_buffer, 128);
#endif
			break;
		case RIO_BUFF_FILLED:
			state = RIO_BUFF_FREE;
			ret = rio_add_inb_buffer(info->mport, CHANNEL_MAILBOX, info->slot);
			if (ret < 0)
				pr_debug(DRV_PREFIX "%s, Error in rio_inb_buffer for mport%d\n", __func__, info->mport->id);
			break;
		default:
			return;
	}

	ret = rio_buff_release_slot(info->buffer, info->slot, state);
	if (ret < 0)
		pr_debug(DRV_PREFIX "%s, Error in rio_buff_release_slot\n", __func__);

	kfree(info);
}

struct vm_operations_struct channel_vm_ops = {
	.open  = channel_vma_open,
	.close = channel_vma_close,
};

static int channel_mmap(struct file *filp, struct vm_area_struct *vma)
{
	int ret;
	void *buffer = NULL;
	uint32_t packets = 0;
	struct channel_dev *dev = filp->private_data;
	struct rio_buff_vma_info *info = NULL;

	pr_debug(DRV_PREFIX "MMAP request at offset %lu\n", vma->vm_pgoff);

	/* Only map exact one packet */
	packets = (vma->vm_end - vma->vm_start) / RIO_MAX_MSG_SIZE;
	if (packets != 1) {
		pr_debug(DRV_PREFIX "MMAP request failed, invalid size\n");
		return -EBADSLT;
	}

	/* Test page offset for buffer type */
	switch(vma->vm_pgoff) {
		case RIO_BUFF_FILLED:
			pr_debug(DRV_PREFIX "MMAP request of filled rx buffer\n");

			ret = rio_buff_request_slot(&dev->rx_buffer, &buffer, RIO_BUFF_FILLED);
			if (ret < 0)
				return -ENODATA;

			/* Map receive buffer */
			ret = remap_pfn_range(vma,
				vma->vm_start,
				virt_to_phys((void *)buffer) >> PAGE_SHIFT,
				RIO_MAX_MSG_SIZE,
				vma->vm_page_prot);
			if (ret < 0)
				return -EIO;

			/* Append mmap information */
			info = (struct rio_buff_vma_info *)kmalloc(sizeof(*info), GFP_KERNEL);
			info->state  = RIO_BUFF_FILLED;
			info->mport  = dev->mport;
			info->buffer = &dev->rx_buffer;
			info->slot   = buffer;

		break;

		/* MMAP transceive buffer */
		case RIO_BUFF_FREE:
			pr_debug(DRV_PREFIX "MMAP tx buffer\n");
			return -EXFULL;
		break;

		/* Bad request, should not come here */
		default:
			pr_err(DRV_PREFIX "Invalid MMAP request\n");
			return -EBADSLT;
	}

	/* Setup vma structure */
	vma->vm_ops = &channel_vm_ops;
	vma->vm_flags |= VM_IO | VM_READ | VM_WRITE;
	vma->vm_private_data = info;

	channel_vma_open(vma);
	return 0;
}
#endif

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
//	.mmap		= channel_mmap,
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

#if (0) //AB

/*
 * Mport mailbox
 */
static void channel_inb_msg_event(struct rio_mport *mport,
	void *dev_id, int mbox, int slot)
{
	int ret = 0;
	void *data = NULL;
	struct channel_dev *dev = dev_id;

	pr_debug(DRV_PREFIX "inbound message event, %s: mbox %d slot %d\n",
			dev->mport->name, mbox, slot);

	/* Get messages from hardware */
	while(1)
	{
		data = rio_get_inb_message(dev->mport, CHANNEL_MAILBOX);
		if(data == NULL)
			goto out;

		pr_err(DRV_PREFIX "msg FILLED at address 0x%p:0x%p\n", (void *)dev->rx_buffer.buffer, data);
		ret = rio_buff_release_slot(&dev->rx_buffer, data, RIO_BUFF_FILLED);
		if (ret < 0) {
			pr_debug(DRV_PREFIX "Error %d in rio_buff_release_slot\n", ret);
		}
	}

	wake_up_interruptible(&channel_cdev_wait);
out:
	return;
}

static void channel_outb_msg_event(struct rio_mport *mport,
		void *dev_id, int mbox, int slot)
{
	int ret = 0;
	struct channel_dev *dev = dev_id;

	pr_debug(DRV_PREFIX "outbound message event, %s: mbox %d slot %d\n",
			dev->mport->name, mbox, slot);

	//pr_debug(DRV_PREFIX "outb till slot %d, tx_slot at %d, slot addr %p\n",slot, dev->tx_slot, dev->tx_ack[slot]);
	/* TODO From HW to Subsystem slot -1 means error in TX */
	if (slot == -1) {
		pr_debug(DRV_PREFIX "There went something wrong with TX, we need to release all buffers now!");
		rio_buff_release_all(&dev->tx_buffer);
		return;
	}

	if (!dev->tx_ack[slot])
		pr_debug(DRV_PREFIX "Error dev->tx_ack[slot] IS NULL!\n");
	ret = rio_buff_release_slot(&dev->tx_buffer, dev->tx_ack[slot], RIO_BUFF_FREE);
	if (ret < 0)
		pr_debug(DRV_PREFIX "error in rio_buff_release_slot\n");
	dev->tx_ack[slot] = NULL;
}

/**
 * channel_mbox_init() - Channels mbox initialisation
 * @dev:	Channel device
 * TODO claim all mport mailboxes (now only CHANNEL_MAILBOX)
 */
static int channel_mbox_init(struct channel_dev *dev)
{
	int rc = 0;
	void *buffer = NULL;
	unsigned int n = 0;
	struct rio_mport *mport = dev->mport;

	pr_debug(DRV_PREFIX "Registering %s\n", mport->name);

	/* Request inbound mport */
	rc = rio_request_inb_mbox(mport, (void *)dev, CHANNEL_MAILBOX,
			RIO_MPORT_CHANNEL_RX_RING, channel_inb_msg_event);
	if (rc < 0) {
		pr_debug(DRV_PREFIX "Error in inbound mailbox %d request on %s\n",
			CHANNEL_MAILBOX, mport->name);
		return -ENXIO;
	}
	pr_info(DRV_PREFIX "Claimed inbound mbox%d on %s with %u slots\n",
		CHANNEL_MAILBOX, mport->name, RIO_MPORT_CHANNEL_RX_RING);

	/* Request outbound mbox */
	rc = rio_request_outb_mbox(mport, (void *)dev, CHANNEL_MAILBOX,
			RIO_MPORT_CHANNEL_TX_RING, channel_outb_msg_event);
	if (rc < 0) {
		pr_debug(DRV_PREFIX "Error in outbound mailbox %d request on %s\n",
			CHANNEL_MAILBOX, mport->name);
		return -ENXIO;
	}
	pr_info(DRV_PREFIX "Claimed outbound mbox%d on %s\n",
			CHANNEL_MAILBOX, mport->name);

	/* Initialise rx_buffer */
	rc = rio_buff_malloc(&dev->rx_buffer, RIO_MPORT_CHANNEL_RX_RING);
	if (rc < 0)
		return -ENOMEM;

	/* Add packet buffers to hardware */
	for (n = 0; n < RIO_MPORT_CHANNEL_RX_RING; n++) {

		rc = rio_buff_request_slot(&dev->rx_buffer, &buffer, RIO_BUFF_FREE);
		if (rc < 0) {
			pr_debug(DRV_PREFIX "No free slot available\n");
			break;
		}

		pr_debug(DRV_PREFIX "Adding free buffer at count %d (0x%p)\n", n, buffer);

		rio_add_inb_buffer(dev->mport,
			CHANNEL_MAILBOX,
			buffer);
	}

	/* Initialise tx_buffer */
	rc = rio_buff_malloc(&dev->tx_buffer, RIO_MPORT_CHANNEL_TX_RING);
	if (rc < 0)
		return -ENOMEM;

	sema_init(&(dev->sem), 1);

	return rc;
}
#endif

/*
 * Character device management
 * TODO test with multiple mport probes
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

	device->tx_slot = 0;
	spin_lock_init(&device->lock);
	spin_lock_init(&device->tx_lock);

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

//	rio_release_inb_mbox(dev->mport, CHANNEL_MAILBOX);
//	rio_release_outb_mbox(dev->mport, CHANNEL_MAILBOX);

	device_unregister(dev->dev);
	cdev_del(&(dev->cdev));

//	rio_buff_free(&dev->rx_buffer);
//	rio_buff_free(&dev->tx_buffer);
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
//	int ret;
	struct rio_mport *mport = NULL;
	struct channel_dev *chdev = NULL;

	mport = to_rio_mport(dev);
	if (!mport)
		return -ENODEV;

	chdev = channel_cdev_add(mport);
	if (!dev)
		return -ENODEV;

//	ret = channel_mbox_init(chdev);
//	if (ret < 0)
//		return ret;

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
 * TODO initialise global channel_devs list
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
