/*
 * rio_cm - RapidIO messaging channel manager
 *
 * Copyright 2013 Integrated Device Technology, Inc.
 * Alexandre Bounine <alexandre.bounine@idt.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * THIS PROGRAM IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL,
 * BUT WITHOUT ANY WARRANTY; WITHOUT EVEN THE IMPLIED WARRANTY OF
 * MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE.  SEE THE
 * GNU GENERAL PUBLIC LICENSE FOR MORE DETAILS.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301 USA.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/rio.h>
#include <linux/rio_drv.h>
#include <linux/slab.h>
#include <linux/idr.h>
#include <linux/interrupt.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/rio_ids.h>
#include <linux/rio_cm_cdev.h>
#include "rio_cm.h"

#define DRV_NAME        "rio_cm"
#define DRV_VERSION     "0.5"
#define DRV_AUTHOR      "Alexandre Bounine <alexandre.bounine@idt.com>"
#define DRV_DESC        "RapidIO Messaging Channel Manager"
#define DEV_NAME	"rio_cm"

static int cmbox = 1;
module_param(cmbox, int, S_IRUGO);
MODULE_PARM_DESC(cmbox, "RapidIO Mailbox number (default 1)");

static int chstart = 256;
module_param(chstart, int, S_IRUGO);
MODULE_PARM_DESC(chstart,
		 "Start channel number for dynamic allocation (default 256)");

#ifdef DEBUG
static u32 dbg_level = DBG_NONE;
module_param(dbg_level, uint, S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(dbg_level, "Debugging output level (default 0 = none)");
#endif

MODULE_AUTHOR(DRV_AUTHOR);
MODULE_DESCRIPTION(DRV_DESC);
MODULE_LICENSE("GPL");

#define RIOCM_TX_RING_SIZE	128
#define RIOCM_RX_RING_SIZE	128
#define RIOCM_CONNECT_TO	3 /* connect response TO (in sec) */

enum rio_cm_state {
	RIO_CM_IDLE,
	RIO_CM_CONNECT,
	RIO_CM_CONNECTED,
	RIO_CM_NACK,
	RIO_CM_DISCONNECT,
	RIO_CM_CHAN_BOUND,
	RIO_CM_LISTEN,
	RIO_CM_DEVICE_REMOVAL,
	RIO_CM_DESTROYING,
	RIO_CM_ERROR,
};

enum rio_cm_pkt_type {
	RIO_CM_SYS	= 0xaa,
	RIO_CM_CHAN	= 0x55,
};

enum rio_cm_chop {
	CM_CONN_REQ,
	CM_CONN_ACK,
	CM_CONN_NACK,
	CM_CONN_CLOSE,
	CM_DATA_MSG,
};

enum rio_cm_nackerr {
	CM_NACK_NOLIS = 1,	/* No listener on destination channel */
	CM_NACK_NODEV,		/* Requesting device object is not available */
};

struct rio_ch_base_bhdr {
	u32 src_id;
	u32 dst_id;
#define RIO_HDR_LETTER_MASK 0xffff0000
#define RIO_HDR_MBOX_MASK   0x0000ffff
	u8  src_mbox;
	u8  dst_mbox;
	u8  type;
} __attribute__((__packed__));

struct rio_ch_chan_hdr {
	struct rio_ch_base_bhdr bhdr;
	u8 ch_op;
	u16 dst_ch;
	u16 src_ch;
	u16 msg_len; /* for NACK response acts as an error code */
	u16 rsrvd;
} __attribute__((__packed__));

struct cm_dev {
	struct list_head	list;
	struct rio_mport	*mport;
	void			*rx_buf[RIOCM_RX_RING_SIZE];
	int			rx_slots;

	void			*tx_buf[RIOCM_TX_RING_SIZE];
	void			*tx_ctxt[RIOCM_TX_RING_SIZE];
	int			tx_slot;
	int			tx_cnt;
	int			tx_ack_slot;
	int			tx_int_slot;

	spinlock_t		cm_lock;
	spinlock_t		tx_lock;
	struct list_head	peers;
	int			npeers;
	struct tasklet_struct	rx_tasklet;
	struct tasklet_struct	tx_tasklet;
};

struct chan_rx_ring {
	void	*buf[RIOCM_RX_RING_SIZE];
	int	head;
	int	tail;
	int	count;

	/* Tracking RX buffers reported to upper level */
	void	*inuse[RIOCM_RX_RING_SIZE];
	int	inuse_cnt;
};

struct rio_channel {
	u16			id;	/* local channel ID */
	struct cm_dev		*cmdev;	/* associated CM device object */
	struct rio_dev		*rdev;	/* remote RapidIO device */
	enum rio_cm_state	state;
	int			error;
	spinlock_t		lock;
	void			*context;
	u32			loc_destid;	/* local destID */
	u32			rem_destid;	/* remote destID */
	u16			rem_channel;	/* remote channel ID */
	struct list_head	accept_queue;
	struct list_head	ch_node;
	wait_queue_head_t	wait_q;
	struct chan_rx_ring	rx_ring;
};

struct cm_peer {
	struct list_head node;
	struct rio_dev *rdev;
};

struct rio_cm_work {
	struct work_struct work;
	struct cm_dev *cm;
	void *data;
};

/*
 * A channel_dev represents a structure on mport
 * @cdev	Character device
 * @dev		Associated device object
 */
struct channel_dev {
	struct cdev	cdev;
	struct device	*dev;
};

static struct rio_channel *riocm_ch_alloc(u16 ch_num);
static void riocm_ch_free(struct rio_channel *ch);
static int riocm_post_send(struct cm_dev *cm, struct rio_dev *rdev,
			   void *buffer, size_t len, void *context);

static DEFINE_SPINLOCK(idr_lock);
static DEFINE_IDR(ch_idr);

static LIST_HEAD(cm_dev_list);
static LIST_HEAD(listen_any_list);
static LIST_HEAD(connect_list);
static DEFINE_SPINLOCK(rio_list_lock);
static struct workqueue_struct *riocm_wq;

static struct class *dev_class;
static unsigned int dev_major;
static unsigned int dev_minor_base;
static dev_t dev_number;
static struct channel_dev *riocm_cdev;

#define is_msg_capable(src_ops, dst_ops)			\
			((src_ops & RIO_SRC_OPS_DATA_MSG) &&	\
			 (dst_ops & RIO_DST_OPS_DATA_MSG))
#define dev_cm_capable(dev) \
	is_msg_capable(dev->src_ops, dev->dst_ops)

static int riocm_comp(struct rio_channel *ch, enum rio_cm_state comp)
{
	int ret;

	spin_lock_bh(&ch->lock);
	ret = (ch->state == comp);
	spin_unlock_bh(&ch->lock);
	return ret;
}

static int riocm_comp_exch(struct rio_channel *ch,
			   enum rio_cm_state comp, enum rio_cm_state exch)
{
	int ret;

	spin_lock_bh(&ch->lock);
	if ((ret = (ch->state == comp)))
		ch->state = exch;
	spin_unlock_bh(&ch->lock);
	return ret;
}

static enum rio_cm_state riocm_exch(struct rio_channel *ch,
				    enum rio_cm_state exch)
{
	enum rio_cm_state old;

	spin_lock_bh(&ch->lock);
	old = ch->state;
	ch->state = exch;
	spin_unlock_bh(&ch->lock);
	return old;
}

static struct rio_channel *riocm_get_channel(u16 nr)
{
	struct rio_channel *ch;

	spin_lock_bh(&idr_lock);
	ch = idr_find(&ch_idr, nr);
	spin_unlock_bh(&idr_lock);
	return ch;
}

static void *riocm_rx_get_msg(struct cm_dev *cm)
{
	void *msg;
	int i;

	msg = rio_get_inb_message(cm->mport, cmbox);
	if (msg) {
		for (i = 0; i < RIOCM_RX_RING_SIZE; i++) {
			if (cm->rx_buf[i] == msg) {
				cm->rx_buf[i] = NULL;
				cm->rx_slots++;
				break;
			}
		}

		if (i == RIOCM_RX_RING_SIZE)
			riocm_warn("no record for buffer 0x%p", msg);
	}

	return msg;
}

/*
 * riocm_rx_fill - sends a data packet to a remote device
 * @cm: cm_dev object
 * @nent: max number of entries to fill
 *
 * Returns: 0 if success, or
 *          -EINVAL if one or more input parameters is/are not valid,
 *          -ENODEV if cannot find a channel with specified ID,
 *          -EAGAIN if a channel is not in connected state,
 *	    error codes returned by HW send routine.
 */
static void riocm_rx_fill(struct cm_dev *cm, int nent)
{
	int i;

	if (cm->rx_slots == 0)
		return;

	for (i = 0; i < RIOCM_RX_RING_SIZE && cm->rx_slots && nent; i++) {
		if (cm->rx_buf[i] == NULL) {
			cm->rx_buf[i] = kmalloc(RIO_MAX_MSG_SIZE, GFP_ATOMIC);
			if (cm->rx_buf[i] == NULL)
				break;
			rio_add_inb_buffer(cm->mport, cmbox, cm->rx_buf[i]);
			cm->rx_slots--;
			nent--;
		}
	}
}

static int riocm_req_handler(struct cm_dev *cm, void *req_data)
{
	struct rio_channel *listen_id;
	struct rio_channel *conn_id;
	struct rio_ch_chan_hdr *hh = req_data;
	struct cm_peer *peer;
	u32 rem_destid;
	u16 snum;
	int found = 0;
	int ret;

	if (hh->ch_op != CM_CONN_REQ) {
		riocm_error("Invalid request header");
		return -EINVAL;
	}

	rem_destid = ntohl(hh->bhdr.src_id);

	/* Find requester's device object */
	spin_lock_bh(&cm->cm_lock);
	list_for_each_entry(peer, &cm->peers, node) {
		if (peer->rdev->destid == rem_destid) {
			riocm_debug(DBG_RX_CMD, "%s found matching device(%s)",
				 __func__, rio_name(peer->rdev));
			found++;
			break;
		}
	}
	spin_unlock_bh(&cm->cm_lock);

	if (!found) {
		/* If peer device object not found simply ignore the request */
		return -ENODEV;
	}

	found = 0;
	snum = ntohs(hh->dst_ch);

	/* Find if there is any listener on request's destination port */
	spin_lock(&rio_list_lock);
	list_for_each_entry(listen_id, &listen_any_list, ch_node)
		if (listen_id->id == snum) {
			riocm_debug(DBG_RX_CMD, "matching listener on ch=%d", snum);
			found++;
			break;
		}

	spin_unlock(&rio_list_lock);

	if (!found) {
		riocm_debug(DBG_RX_CMD, "listener on channel %d not found", snum);
		listen_id = NULL;
		hh->msg_len = CM_NACK_NOLIS;
		goto out_nack;
	}

	/* Create new channel for this connection */
	conn_id = riocm_ch_alloc(RIOCM_CHNUM_AUTO);

	if (IS_ERR(conn_id)) {
		riocm_error("failed to get channel for new req (%ld)",
			PTR_ERR(conn_id));
		return -ENOMEM;
	}

	spin_lock_init(&conn_id->lock);
	conn_id->cmdev = listen_id->cmdev;

	conn_id->loc_destid = listen_id->loc_destid;
	conn_id->rem_destid = ntohl(hh->bhdr.src_id);
	conn_id->rem_channel = ntohs(hh->src_ch);
	conn_id->rdev = peer->rdev;
	conn_id->state = RIO_CM_CONNECTED;

	spin_lock(&listen_id->lock);
	list_add_tail(&conn_id->ch_node, &listen_id->accept_queue);
	spin_unlock(&listen_id->lock);

	/*
	 * Acknowledge the connection request.
	 * Send back the same buffer with modified fields.
	 */
	hh->bhdr.src_id = htonl(conn_id->loc_destid);
	hh->bhdr.dst_id = htonl(conn_id->rem_destid);
	hh->bhdr.src_mbox = cmbox;
	hh->bhdr.dst_mbox = cmbox;
	hh->bhdr.type = RIO_CM_CHAN;
	hh->ch_op = CM_CONN_ACK;
	hh->dst_ch = htons(conn_id->rem_channel);
	hh->src_ch = htons(conn_id->id);

	/* FIXME: the function call below relies on the fact that underlying
	 * add_outb_message() routine copies TX data into its internal transfer
	 * buffer. Needs to be reviewed if switched to direct buffer version.
	 */

	ret = riocm_post_send(cm, conn_id->rdev, hh, sizeof *hh, NULL);
	//if (ret)
	//	goto err;

	if (waitqueue_active(&listen_id->wait_q))
		wake_up(&listen_id->wait_q);
	return ret;

out_nack:
	/*
	 * Reply with NACK (no matching listener or
	 *                  active peer with specified destid).
	 * Send back the same buffer with modified fields.
	 * NOTE: hh->msg_len holds a corresponding NACK error code and
	 *       must be preserved.
	 */
	hh->bhdr.dst_id = htonl(rem_destid);
	hh->bhdr.src_id = htonl(cm->mport->host_deviceid);
	hh->bhdr.src_mbox = cmbox;
	hh->bhdr.dst_mbox = cmbox;
	hh->bhdr.type = RIO_CM_CHAN;
	hh->ch_op = CM_CONN_NACK;
	hh->dst_ch = hh->src_ch;
	hh->src_ch = 0;

	/* FIXME: the function call below relies on the fact that underlying
	 * add_outb_message() routine copies TX data into its internal transfer
	 * buffer. Needs to be reviewed if switched to direct buffer version.
	 */
	ret = riocm_post_send(cm, peer->rdev, hh, sizeof *hh, NULL);

	return ret;
}

static int riocm_resp_handler(void *resp_data)
{
	struct rio_channel *conn_id;
	struct rio_ch_chan_hdr *hh = resp_data;
	int found = 0;
	u16 snum;

	if (hh->ch_op != CM_CONN_ACK && hh->ch_op != CM_CONN_NACK) {
		riocm_error("Invalid request header");
		return -EINVAL;
	}

	snum = ntohs(hh->dst_ch);

	/* Find if any requester waits on resp's destination port */
	list_for_each_entry(conn_id, &connect_list, ch_node)
		if (conn_id->id == snum) {
			found++;
			break;
		}

	if (!found)
		return -ECONNABORTED;

	spin_lock(&rio_list_lock);
	list_del(&conn_id->ch_node);
	spin_unlock(&rio_list_lock);

	if (hh->ch_op == CM_CONN_ACK) {
		riocm_exch(conn_id, RIO_CM_CONNECTED);
		/* Update remote channel number
		 * (changed as result of connection acceptance by remote)
		 */
		conn_id->rem_channel = ntohs(hh->src_ch);
	} else { /* hh->ch_op == CM_CONN_NACK */
		riocm_exch(conn_id, RIO_CM_NACK);
	}

	if (waitqueue_active(&conn_id->wait_q))
		wake_up(&conn_id->wait_q);

	return 0;
}

static int riocm_close_handler(void *data)
{
	struct rio_channel *ch;
	struct rio_ch_chan_hdr *hh = data;

	if (hh->ch_op != CM_CONN_CLOSE) {
		riocm_error("%s Invalid request header", __func__);
		return -EINVAL;
	}

	riocm_debug(DBG_RX_CMD, "%s for ch=%d", __func__, ntohs(hh->dst_ch));

	/* Find if there is an active channel with specified ID */
	ch = riocm_get_channel(ntohs(hh->dst_ch));

	if (!ch)
		return -ENODEV;

	riocm_exch(ch, RIO_CM_DISCONNECT);

	if (waitqueue_active(&ch->wait_q))
		wake_up(&ch->wait_q);

	return 0;
}

static void rio_cm_handler(struct work_struct *_work)
{
	struct rio_cm_work *work = container_of(_work, struct rio_cm_work, work);
	void *data = work->data;
	struct rio_ch_chan_hdr *hdr;

	hdr = (struct rio_ch_chan_hdr *)data;

	riocm_debug(DBG_RX_CMD, "%s: OP=%x for ch=%d from %d", __func__,
		    hdr->ch_op, ntohs(hdr->dst_ch), ntohs(hdr->src_ch));

	switch (hdr->ch_op) {
	case CM_CONN_REQ:
		riocm_req_handler(work->cm, data);
		break;
	case CM_CONN_ACK:
	case CM_CONN_NACK:
		riocm_resp_handler(data);
		break;
	case CM_CONN_CLOSE:
		riocm_close_handler(data);
		break;
	default:
		riocm_error("Invalid packet header");
		break;
	}

	kfree(data);
	kfree(work);
}

static int rio_rx_data_handler(struct cm_dev *cm, void *buf)
{
	struct rio_ch_chan_hdr *hdr;
	struct rio_channel *ch;

	hdr = (struct rio_ch_chan_hdr *)buf;

	riocm_debug(DBG_RX_DATA, "%s: for ch=%d", __func__, ntohs(hdr->dst_ch));

	spin_lock(&idr_lock);
	ch = idr_find(&ch_idr, ntohs(hdr->dst_ch));
	spin_unlock(&idr_lock);

	if (!ch) {
		/* Discard data message for non-existing channel */
		kfree(buf);
		return -ENODEV;
	}
#if (0)
	riocm_debug(DBG_RX_DATA, "%s: found ch=%d", __func__, ch->id);
	riocm_debug(DBG_RX_DATA, "%s: msg=%s", __func__,
		 (char *)((u8 *)buf + sizeof(struct rio_ch_chan_hdr)));
#endif
	/* Place pointer to the buffer into channel's RX queue */
	spin_lock(&ch->lock);

	if (ch->state != RIO_CM_CONNECTED) {
		/* Channel is not ready to receive data, discard a packet */
		riocm_debug(DBG_RX_DATA, "%s: ch=%d is in wrong state=%d",
			__func__, ch->id, ch->state);
		spin_unlock(&ch->lock);
		kfree(buf);
		return -EIO;
	}

	if (ch->rx_ring.count == RIOCM_RX_RING_SIZE) {
		/* If RX ring is full, discard a packet */
		riocm_warn("%s: ch=%d is full", __func__, ch->id);
		spin_unlock(&ch->lock);
		kfree(buf);
		return -ENOMEM;
	}

	ch->rx_ring.buf[ch->rx_ring.head] = buf;
	ch->rx_ring.head++;
	ch->rx_ring.count++;
	ch->rx_ring.head %= RIOCM_RX_RING_SIZE;
	spin_unlock(&ch->lock);

	/* Wake up the caller if there is waiting receive call */
	if (waitqueue_active(&ch->wait_q))
		wake_up(&ch->wait_q);

	return 0;
}

static void rio_ibmsg_handler(unsigned long context)
{
	struct cm_dev *cm = (struct cm_dev *)context;
	void *data;
	struct rio_ch_chan_hdr *hdr;
	int i;

	for (i = 0; i < 8; i++) {
		spin_lock(&cm->cm_lock);
		data = riocm_rx_get_msg(cm);
		if (data)
			riocm_rx_fill(cm, 1);
		spin_unlock(&cm->cm_lock);

		if (data == NULL)
			break;

		hdr = (struct rio_ch_chan_hdr *)data;

		if (hdr->bhdr.type != RIO_CM_CHAN) {
			/* For now simply discard packets other than channel */
			riocm_error("Unsupported TYPE code (0x%x). Message dropped", hdr->bhdr.type);
			kfree(data);
			continue;
		}

		/* Process a channel message */

		if (hdr->ch_op == CM_CONN_REQ || hdr->ch_op == CM_CONN_ACK ||
		    hdr->ch_op == CM_CONN_NACK || hdr->ch_op == CM_CONN_CLOSE) {

			struct rio_cm_work *work;

			work = kmalloc(sizeof *work, GFP_ATOMIC);
			if (!work) {
				/* Discard a packet if we cannot process it */
				riocm_error("Failed to allocate memory for work struct");
				kfree(data);
				continue;
			}

			INIT_WORK(&work->work, rio_cm_handler);
			work->data = data;
			work->cm = cm;
			queue_work(riocm_wq, &work->work);
		} else if (hdr->ch_op == CM_DATA_MSG) {
			rio_rx_data_handler(cm, data);
		} else {
			/* Discard packets with invalid OP code */
			riocm_error("Unsupported CH_OP code (0x%x). Message dropped", hdr->ch_op);
			kfree(data);
		}
	}

	if (i == 8)
		tasklet_schedule(&cm->rx_tasklet);
}

void riocm_inb_msg_event(struct rio_mport *mport, void *dev_id, int mbox, int slot)
{
	struct cm_dev *cm;

	list_for_each_entry(cm, &cm_dev_list, list)
		if (cm->mport == mport) {
			tasklet_schedule(&cm->rx_tasklet);
			break;
		}
}

static void riocm_chan_tx_complete(struct rio_channel *ch, void *buf)
{
	/* FIXME: Implement TX buffer completion notification for given
	 * channel if direct buffer transfer is implemented.
	 */
}

static void rio_txcq_handler(unsigned long context)
{
	struct cm_dev *cm = (struct cm_dev *)context;
	int ack_slot;

	/* FIXME: We do not need TX completion notification until direct buffer
	 * transfer is implemented. At this moment only correct tracking
	 * of tx_count is important.
	 */
	riocm_debug(DBG_TX_EVENT, "%s for mport_%d", __func__, cm->mport->id);

	spin_lock(&cm->tx_lock);
	ack_slot = cm->tx_ack_slot;

	while (cm->tx_cnt && (ack_slot != cm->tx_int_slot)) {
		if (cm->tx_ctxt[ack_slot])
			riocm_chan_tx_complete(cm->tx_ctxt[ack_slot], cm->tx_buf[ack_slot]);

		cm->tx_buf[ack_slot] = NULL;
		cm->tx_ctxt[ack_slot] = NULL;
		++ack_slot;
		ack_slot &= (RIOCM_TX_RING_SIZE - 1);
		cm->tx_cnt--;
	}

	cm->tx_ack_slot = ack_slot;
	spin_unlock(&cm->tx_lock);
}

void riocm_outb_msg_event(struct rio_mport *mport, void *dev_id, int mbox, int slot)
{
	struct cm_dev *cm;

	list_for_each_entry(cm, &cm_dev_list, list) {
		if (cm->mport == mport) {
			cm->tx_int_slot = slot;
			tasklet_schedule(&cm->tx_tasklet);
			break;
		}
	}
}

static int riocm_post_send(struct cm_dev *cm, struct rio_dev *rdev,
			   void *buffer, size_t len, void *context)
{
	unsigned long flags;
	int rc;

	spin_lock_irqsave(&cm->tx_lock, flags);

	if (cm->tx_cnt + 1 > RIOCM_TX_RING_SIZE) {
		riocm_warn("Tx Ring is full");
		rc = -EBUSY;
		goto err_out;
	}

	cm->tx_buf[cm->tx_slot] = buffer;
	cm->tx_ctxt[cm->tx_slot] = context;
	rc = rio_add_outb_message(cm->mport, rdev, cmbox, buffer, len);

	riocm_debug(DBG_TX, "Add buf@%p destid=%x tx_slot=%d tx_cnt=%d",
		 buffer, rdev->destid, cm->tx_slot, cm->tx_cnt);

	if (++cm->tx_cnt == RIOCM_TX_RING_SIZE)
		riocm_warn("TX QUEUE IS FULL");

	++cm->tx_slot;
	cm->tx_slot &= (RIOCM_TX_RING_SIZE - 1);

err_out:
	spin_unlock_irqrestore(&cm->tx_lock, flags);
	return rc;
}

/*
 * riocm_ch_send - sends a data packet to a remote device
 * @ch_id: local channel ID
 * @buf: pointer to a data buffer to send (including CM header)
 * @len: length of data to transfer (including CM header)
 *
 * ATTN: ASSUMES THAT THE HEADER SPACE IS RESERVED PART OF THE DATA PACKET
 *
 * Returns: 0 if success, or
 *          -EINVAL if one or more input parameters is/are not valid,
 *          -ENODEV if cannot find a channel with specified ID,
 *          -EAGAIN if a channel is not in connected state,
 *	    error codes returned by HW send routine.
 */
int riocm_ch_send(u16 ch_id, void *buf, int len)
{
	struct rio_channel *ch;
	struct rio_ch_chan_hdr *hdr;
	int ret;

	if (buf == NULL || ch_id == 0 || len == 0 || len > RIO_MAX_MSG_SIZE)
		return -EINVAL;

	ch = riocm_get_channel(ch_id);
	if (!ch)
		return -ENODEV;

	if (!riocm_comp(ch, RIO_CM_CONNECTED))
		return -EAGAIN;

	/*
	 * Fill buffer header section with corresponding channel data
	 */
	hdr = (struct rio_ch_chan_hdr *)buf;

	hdr->bhdr.src_id = htonl(ch->loc_destid);
	hdr->bhdr.dst_id = htonl(ch->rem_destid);
	hdr->bhdr.src_mbox = cmbox;
	hdr->bhdr.dst_mbox = cmbox;
	hdr->bhdr.type = RIO_CM_CHAN;
	hdr->ch_op = CM_DATA_MSG;
	hdr->dst_ch = htons(ch->rem_channel);
	hdr->src_ch = htons(ch->id);
	hdr->msg_len = htons((u16)len);

	/* FIXME: the function call below relies on the fact that underlying
	 * add_outb_message() routine copies TX data into its internal transfer
	 * buffer. Needs to be reviewed if switched to direct buffer version.
	 */

	ret = riocm_post_send(ch->cmdev, ch->rdev, buf, len, ch);
	if (ret) {
		riocm_error("%s ch %d send_err=%d", __func__, ch->id, ret);
	}

	return ret;
}
EXPORT_SYMBOL_GPL(riocm_ch_send);

/*
 * riocm_wait_for_rx_data - waits for received data message
 * @ch: channel object
 *
 * ATTN: THIS FUNCTION MUST BE CALLED WITH CHANNEL SPINLOCK HELD BY CALLER.
 *
 * Returns: 0 if success (accept queue is not empty), or
 *          -EINTR if wait was interrupted by signal
 */
static int riocm_wait_for_rx_data(struct rio_channel *ch)
{
	int err;
	DEFINE_WAIT(wait);

	riocm_debug(DBG_WAIT, "%s on %d", __func__, ch->id);

	for (;;) {
		prepare_to_wait_exclusive(&ch->wait_q, &wait,
					  TASK_INTERRUPTIBLE);
		spin_unlock_bh(&ch->lock);
		if (ch->rx_ring.count == 0)
			schedule();

		spin_lock_bh(&ch->lock);
		if (ch->rx_ring.count) {
			err = 0;
			break;
		}

		if (ch->state != RIO_CM_CONNECTED) {
			err = -ECONNRESET;
			break;
		}

		if (signal_pending(current)) {
			err = -EINTR;
			break;
		}
	}

	finish_wait(&ch->wait_q, &wait);
	riocm_debug(DBG_WAIT, "%s on %d returns %d", __func__, ch->id, err);
	return err;
}

/*
 * riocm_ch_free_rxbuf - release an inbound data buffer
 * @ch_id: local channel ID
 * @buf: pointer to data buffer to be released
 *
 * Returns: 0 if success, or
 *          -EINVAL if one or more input parameters is/are not valid,
 *          -ENODEV if cannot find a channel with specified ID,
 */
int riocm_ch_free_rxbuf(u16 ch_id, void *buf)
{
	struct rio_channel *ch;
	int i, ret = -EINVAL;

	if (buf == NULL || ch_id == 0)
		goto out;

	ch = riocm_get_channel(ch_id);
	if (!ch) {
		ret = -ENODEV;
		goto out;
	}

	spin_lock_bh(&ch->lock);

	for (i = 0; i < RIOCM_RX_RING_SIZE; i++) {
		if (ch->rx_ring.inuse[i] == buf) {
			ch->rx_ring.inuse[i] = NULL;
			ch->rx_ring.inuse_cnt--;
			ret = 0;
			break;
		}
	}

	spin_unlock_bh(&ch->lock);

	if (!ret)
		kfree(buf);
out:
	return ret;
}
EXPORT_SYMBOL_GPL(riocm_ch_free_rxbuf);

/*
 * riocm_ch_receive - receives a data packet from a remote device
 * @ch_id: local channel ID
 * @buf: pointer to data buffer with received message (including CM header)
 * @len: length of received data (including CM header)
 *
 * Returns: 0 if success, or
 *          -EINVAL if one or more input parameters is/are not valid,
 *          -ENODEV if cannot find a channel with specified ID,
 *          -EAGAIN if a channel is not in connected state,
 *          -ENOMEM if there is no free entry for buffer tracking.
 */
int riocm_ch_receive(u16 ch_id, void **buf, int *len)
{
	struct rio_channel *ch;
	void *rxmsg = NULL;
	int i, ret = 0;

	if (buf == NULL || ch_id == 0 || len == NULL) {
		ret = -EINVAL;
		goto out;
	}

	ch = riocm_get_channel(ch_id);
	if (!ch) {
		ret = -ENODEV;
		goto out;
	}

	if (!riocm_comp(ch, RIO_CM_CONNECTED)) {
		ret = -EAGAIN;
		goto out;
	}

	if (ch->rx_ring.inuse_cnt == RIOCM_RX_RING_SIZE) {
		/* If we do not have entries to track buffers given to upper
		 * layer, reject request.
		 */
		ret = -ENOMEM;
		goto out;
	}

	spin_lock_bh(&ch->lock);

	if (ch->rx_ring.count == 0) {
		ret = riocm_wait_for_rx_data(ch); /* blocking wait, no timeout */
		if (ret) {
			spin_unlock_bh(&ch->lock);
			*buf = NULL;
			return ret;
		}
	}

	rxmsg = ch->rx_ring.buf[ch->rx_ring.tail];
	ch->rx_ring.buf[ch->rx_ring.tail] = NULL;
	ch->rx_ring.count--;
	ch->rx_ring.tail++;
	ch->rx_ring.tail %= RIOCM_RX_RING_SIZE;

	for (i = 0; i < RIOCM_RX_RING_SIZE; i++) {
		if (ch->rx_ring.inuse[i] == NULL) {
			ch->rx_ring.inuse[i] = rxmsg;
			ch->rx_ring.inuse_cnt++;
			break;
		}
	}

	spin_unlock_bh(&ch->lock);
out:
	*buf = rxmsg;
	return ret;
}
EXPORT_SYMBOL_GPL(riocm_ch_receive);

/*
 * riocm_wait_for_connect_resp - waits for connect response (ACK/NACK) from
 *                               a remote device
 * @ch: channel object
 * @timeo: timeout value in jiffies
 *
 * ATTN: THIS FUNCTION MUST BE CALLED WITH CHANNEL SPINLOCK HELD BY CALLER.
 *
 * Returns: 0 if success (accept queue is not empty), or
 *          -EINTR if wait was interrupted by signal,
 *          -ETIME if wait timeout expired.
 */
static int riocm_wait_for_connect_resp(struct rio_channel *ch, long timeo)
{
	int err;
	DEFINE_WAIT(wait);

	riocm_debug(DBG_WAIT, "%s on %d", __func__, ch->id);

	for (;;) {
		prepare_to_wait_exclusive(&ch->wait_q, &wait,
					  TASK_INTERRUPTIBLE);
		spin_unlock_bh(&ch->lock);
		if (ch->state == RIO_CM_CONNECT)
			timeo = schedule_timeout(timeo);

		spin_lock_bh(&ch->lock);
		err = 0;
		if (ch->state != RIO_CM_CONNECT)
			break;
		err = -EINTR;
		if (signal_pending(current))
			break;
		err = -ETIME;
		if (!timeo)
			break;
	}

	finish_wait(&ch->wait_q, &wait);
	riocm_debug(DBG_WAIT, "%s on %d returns %d", __func__, ch->id, err);
	return err;
}

/*
 * riocm_ch_connect - sends a connect request to a remote device
 * @loc_ch: local channel ID
 * @mport_id:  corresponding RapidIO mport device
 * @rem_destid: destination ID of target RapidIO device
 * @rem_ch: remote channel ID
 *
 * Returns: 0 if success, or
 *          -ENODEV if cannot find specified channel or mport,
 *          -EINVAL if the channel is not in IDLE state,
 *          -EAGAIN if no connection request available immediately.
 */
int riocm_ch_connect(u16 loc_ch, u8 mport_id, u32 rem_destid, u16 rem_ch)
{
	struct rio_channel *ch = NULL;
	struct rio_ch_chan_hdr hdr;
	struct cm_dev *cm;
	struct cm_peer *peer;
	int found = 0;
	int ret;

	spin_lock(&rio_list_lock);

	/* Find matching cm_dev object */
	list_for_each_entry(cm, &cm_dev_list, list) {
		if (cm->mport->id == mport_id) {
			found++;
			break;
		}
	}

	if (!found) {
		spin_unlock(&rio_list_lock);
		riocm_error("cm_dev not found");
		return -ENODEV;
	}

	/* Find corresponding RapidIO endpoint device object */

	found = 0;

	list_for_each_entry(peer, &cm->peers, node) {
		if (peer->rdev->destid == rem_destid) {
			found++;
			break;
		}
	}

	spin_unlock(&rio_list_lock);

	if (!found) {
		riocm_error("Target RapidIO device not found");
		return -ENODEV;
	}

	ch = riocm_get_channel(loc_ch);
	if (!ch)
		return -ENODEV;

	if (!riocm_comp_exch(ch, RIO_CM_IDLE, RIO_CM_CONNECT))
		return -EINVAL;

	ch->cmdev = cm;
	ch->rdev = peer->rdev;
	ch->context = NULL;
	ch->loc_destid = cm->mport->host_deviceid;
	ch->rem_channel = rem_ch;

	spin_lock(&rio_list_lock);
	list_add_tail(&ch->ch_node, &connect_list);
	spin_unlock(&rio_list_lock);

	/*
	 * Send connect request to the remote RapidIO device
	 */

	hdr.bhdr.src_id = htonl(ch->loc_destid);
	hdr.bhdr.dst_id = htonl(rem_destid);
	hdr.bhdr.src_mbox = cmbox;
	hdr.bhdr.dst_mbox = cmbox;
	hdr.bhdr.type = RIO_CM_CHAN;
	hdr.ch_op = CM_CONN_REQ;
	hdr.dst_ch = htons(rem_ch);
	hdr.src_ch = htons(loc_ch);

	/* FIXME: the function call below relies on the fact that underlying
	 * add_outb_message() routine copies TX data into its internal transfer
	 * buffer. Needs to be reviewed if switched to direct buffer version.
	 */

	ret = riocm_post_send(cm, peer->rdev, &hdr, sizeof hdr, NULL);
	if (ret) {
		riocm_comp_exch(ch, RIO_CM_CONNECT, RIO_CM_IDLE);
		spin_lock(&rio_list_lock);
		list_del(&ch->ch_node);
		spin_unlock(&rio_list_lock);
		goto conn_done;
	}

	/* Wait for connect response from the remote device */
	spin_lock_bh(&ch->lock);

	/* Check if we still in CONNECT state */
	if (ch->state == RIO_CM_CONNECT) {
		ret = riocm_wait_for_connect_resp(ch, RIOCM_CONNECT_TO * HZ);
		if (ret) {
			spin_unlock_bh(&ch->lock);
			goto conn_done;
		}
	}

	spin_unlock_bh(&ch->lock);

	ret = (ch->state == RIO_CM_CONNECTED)? 0 : -1;

conn_done:
	return ret;
}
EXPORT_SYMBOL_GPL(riocm_ch_connect);

/*
 * riocm_wait_for_connect_req - waits for connect request from a remote device
 * @ch: channel object
 * @timeo: timeout value in jiffies
 *
 * ATTN: THIS FUNCTION MUST BE CALLED WITH CHANNEL SPINLOCK HELD BY CALLER.
 *
 * Returns: 0 if success (accept queue is not empty), or
 *          -EINTR if wait was interrupted by signal,
 *          -ETIME if wait timeout expired.
 */
static int riocm_wait_for_connect_req(struct rio_channel *ch, long timeo)
{
	int err;
	DEFINE_WAIT(wait);

	riocm_debug(DBG_WAIT, "%s on %d", __func__, ch->id);

	for (;;) {
		prepare_to_wait_exclusive(&ch->wait_q, &wait,
					  TASK_INTERRUPTIBLE);
		spin_unlock_bh(&ch->lock);
		if (list_empty(&ch->accept_queue) && ch->state == RIO_CM_LISTEN)
			timeo = schedule_timeout(timeo);

		spin_lock_bh(&ch->lock);
		err = 0;
		if (!list_empty(&ch->accept_queue))
			break;
		err = -EINTR;
		if (ch->state != RIO_CM_LISTEN)
			break;
		err = -EINTR;
		if (signal_pending(current))
			break;
		err = -ETIME;
		if (!timeo)
			break;
	}

	finish_wait(&ch->wait_q, &wait);
	riocm_debug(DBG_WAIT, "%s on %d returns %d", __func__, ch->id, err);
	return err;
}

/*
 * riocm_ch_accept - associate a channel object and an mport device
 * @ch_id: channel ID
 * @new_ch_id: local mport device
 * @timeout: wait timeout (if 0 non-blocking call, do not wait if connection request
 *           is not available).
 *
 * Returns: 0 if success, or
 *          -ENODEV if cannot find specified channel or mport,
 *          -EINVAL if the channel is not in IDLE state,
 *          -EAGAIN if no connection request available immediately.
 */
int riocm_ch_accept(u16 ch_id, u16 *new_ch_id, long timeout)
{
	struct rio_channel *ch = NULL;
	struct rio_channel *new_ch = NULL;
	int err;

	ch = riocm_get_channel(ch_id);
	if (!ch)
		return -EINVAL;

	spin_lock_bh(&ch->lock);

	if (ch->state != RIO_CM_LISTEN) {
		err = -EINVAL;
		goto out_err;
	}

	/* Check if we have already established connection */
	if (list_empty(&ch->accept_queue)) {

		/* Don't sleep if this is a non blocking call */
		if (!timeout) {
			err = -EAGAIN;
			goto out_err;
		}

		err = riocm_wait_for_connect_req(ch, timeout);
		if (err)
			goto out_err;
	}

	new_ch = list_entry(ch->accept_queue.next, struct rio_channel, ch_node);
	list_del(&new_ch->ch_node);
	spin_unlock_bh(&ch->lock);

	*new_ch_id = new_ch->id;
	return 0;
out_err:
	spin_unlock_bh(&ch->lock);
	*new_ch_id = 0;
	return err;
}
EXPORT_SYMBOL_GPL(riocm_ch_accept);

/*
 * riocm_ch_listen - puts a channel into LISTEN state
 * @ch_id: channel ID
 *
 * Returns: 0 if success, or
 *          -EINVAL if the specified channel does not exists or
 *                  is not in CHAN_BOUND state.
 */
int riocm_ch_listen(u16 ch_id)
{
	struct rio_channel *ch = NULL;

	riocm_debug(DBG_CHOP, "%s(ch_%d)", __func__, ch_id);

	ch = riocm_get_channel(ch_id);
	if (!ch)
		return -EINVAL;

	spin_lock_bh(&ch->lock);
	if (ch->state != RIO_CM_CHAN_BOUND) {
		spin_unlock_bh(&ch->lock);
		return -EINVAL;
	}

	/* Add the channel into the global list of listeners */
	spin_lock(&rio_list_lock);
	list_add_tail(&ch->ch_node, &listen_any_list);
	spin_unlock(&rio_list_lock);
	ch->state = RIO_CM_LISTEN;
	spin_unlock_bh(&ch->lock);

	return 0;
}
EXPORT_SYMBOL_GPL(riocm_ch_listen);

/*
 * riocm_ch_bind - associate a channel object and an mport device
 * @ch_id: channel ID
 * @mport_id: local mport device ID
 * @context: pointer to the additional caller's context (???)
 *
 * Returns: 0 if success, or
 *          -ENODEV if cannot find specified channel or mport,
 *          -EINVAL if the channel is not in IDLE state.
 */
int riocm_ch_bind(u16 ch_id, u8 mport_id, void *context)
{
	struct rio_channel *ch = NULL;
	struct cm_dev *cm;

	riocm_debug(DBG_CHOP, "%s ch_%d to mport_%d", __func__, ch_id, mport_id);

	/* Find matching cm_dev object */
	list_for_each_entry(cm, &cm_dev_list, list) {
		if (cm->mport->id == mport_id)
			goto found;
	}

	return -ENODEV;
found:
	ch = riocm_get_channel(ch_id);
	if (!ch)
		return -ENODEV;

	spin_lock_bh(&ch->lock);
	if (ch->state != RIO_CM_IDLE) {
		spin_unlock_bh(&ch->lock);
		return -EINVAL;
	}

	ch->cmdev = cm;
	ch->loc_destid = cm->mport->host_deviceid;
	ch->context = context;
	ch->state = RIO_CM_CHAN_BOUND;
	spin_unlock_bh(&ch->lock);

	return 0;
}
EXPORT_SYMBOL_GPL(riocm_ch_bind);

/*
 * riocm_ch_alloc - channel object allocation helper routine
 * @ch_num: channel ID (1 ... RIOCM_MAX_CHNUM, 0 = automatic)
 *
 * Return value: pointer to newly created channel object, or error code
 */
static struct rio_channel *riocm_ch_alloc(u16 ch_num)
{
	int id;
	int start, end;
	struct rio_channel *ch = NULL;

	ch = kzalloc(sizeof(struct rio_channel), GFP_KERNEL);
	if (!ch)
		return ERR_PTR(-ENOMEM);

	ch->state = RIO_CM_IDLE;
	spin_lock_init(&ch->lock);
	INIT_LIST_HEAD(&ch->accept_queue);
	init_waitqueue_head(&ch->wait_q);
	ch->rx_ring.head = 0;
	ch->rx_ring.tail = 0;
	ch->rx_ring.count = 0;
	ch->rx_ring.inuse_cnt = 0;

	if (ch_num) {
		/* If requested, try to obtain the specified channel ID */
		start = ch_num;
		end = ch_num + 1;
	} else {
		/* Obtain channel ID from the dynamic allocation range */
		start = chstart;
		end = RIOCM_MAX_CHNUM + 1;
	}

	spin_lock_bh(&idr_lock);
	id = idr_alloc(&ch_idr, ch, start, end, GFP_KERNEL);
	ch->id = (u16)id;
	spin_unlock_bh(&idr_lock);

	if (id < 0) {
		kfree(ch);
		return ERR_PTR(id == -ENOSPC ? -EBUSY : id);
	}

	return ch;
}

/*
 * riocm_ch_create - creates a new channel object and allocates ID for it
 * @ch_num: channel ID (1 ... RIOCM_MAX_CHNUM, 0 = automatic)
 *
 * Allocates and initializes a new channel object. If the parameter ch_num > 0
 * and is within the valid range, riocm_ch_create tries to allocate the
 * specified ID for the new channel. If ch_num = 0, channel ID will be assigned
 * automatically from the range (chstart ... RIOCM_MAX_CHNUM).
 * Module parameter 'chstart' defines start of an ID range available for dynamic
 * allocation. Range below 'chstart' is reserved for pre-defined ID numbers.
 * Available channel numbers are limited by 16-bit size of channel numbers used
 * in the packet header.
 *
 * Return value: 0 if successful (with channel number updated via pointer) or
 *               -1 if error.
 */
int riocm_ch_create(u16 *ch_num)
{
	struct rio_channel *ch = NULL;

	ch = riocm_ch_alloc(*ch_num);

	if (IS_ERR(ch)) {
		riocm_error("Failed to allocate channel %d (err=%ld)",
			 *ch_num, PTR_ERR(ch));
		return -1;
	}

	*ch_num = ch->id;
	return 0;
}
EXPORT_SYMBOL_GPL(riocm_ch_create);

/*
 * riocm_ch_free - channel object release helper routine
 * @ch: pointer to a channel object to be freed
 */
static void riocm_ch_free(struct rio_channel *ch)
{
	int i;

	spin_lock_bh(&idr_lock);
	idr_remove(&ch_idr, ch->id);
	spin_unlock_bh(&idr_lock);

	if (ch->rx_ring.inuse_cnt)
		for (i = 0; i < RIOCM_RX_RING_SIZE; i++)
			if (ch->rx_ring.inuse[i] != NULL)
				kfree(ch->rx_ring.inuse[i]);

	if (ch->rx_ring.count)
		for (i = 0; i < RIOCM_RX_RING_SIZE; i++)
			if (ch->rx_ring.buf[i] != NULL)
				kfree(ch->rx_ring.buf[i]);

	kfree(ch);
}

/*
 * riocm_ch_close - closes a channel object with specified ID (by local request)
 * @ch_id: channel ID to be closed
 *
 */
int riocm_ch_close(u16 ch_id)
{
	struct rio_channel *ch;
	struct cm_dev *cm;
	enum rio_cm_state state;
	int ret;

	riocm_debug(DBG_CHOP, "%s(%d)", __func__, ch_id);
	ch = riocm_get_channel(ch_id);
	if (!ch)
		return -ENODEV;

	state = riocm_exch(ch, RIO_CM_DESTROYING);

	if (state == RIO_CM_IDLE || state == RIO_CM_CHAN_BOUND ||
	    state == RIO_CM_DISCONNECT)
		goto out_free;

	if (state == RIO_CM_LISTEN || state == RIO_CM_CONNECT) {
		/* Remove the channel from the corresponding list */
		spin_lock(&rio_list_lock);
		list_del(&ch->ch_node);
		spin_unlock(&rio_list_lock);
	} else if (state == RIO_CM_CONNECTED) {
		/*
		 * Send CLOSE notification to the remote RapidIO device
		 */
		struct rio_ch_chan_hdr hdr;

		cm = ch->cmdev;
		hdr.bhdr.src_id = htonl(ch->loc_destid);
		hdr.bhdr.dst_id = htonl(ch->rem_destid);
		hdr.bhdr.src_mbox = cmbox;
		hdr.bhdr.dst_mbox = cmbox;
		hdr.bhdr.type = RIO_CM_CHAN;
		hdr.ch_op = CM_CONN_CLOSE;
		hdr.dst_ch = htons(ch->rem_channel);
		hdr.src_ch = htons((u16)ch_id);

		/* FIXME: the function call below relies on the fact that underlying
		 * add_outb_message() routine copies TX data into its internal transfer
		 * buffer. Needs to be reviewed if switched to direct buffer version.
		 */

		ret = riocm_post_send(cm, ch->rdev, &hdr, sizeof hdr, NULL);
		if (ret) {
			riocm_error("%s(%d) sending CLOSE failed",
				__func__, ch_id);
		}
	}

	if (waitqueue_active(&ch->wait_q))
		wake_up_all(&ch->wait_q);

out_free:
	riocm_ch_free(ch);
	return 0;
}
EXPORT_SYMBOL_GPL(riocm_ch_close);

/*
 * riocm_get_peer_list - report number of remote peer endpoints connected
 *                        to the specified mport device
 * @mport_id: mport device ID
 * @buf: peer list buffer
 * @nent: number of 32-bit entries in the buffer
 */
static int riocm_get_peer_list(u8 mport_id, void *buf, u32 *nent)
{
	struct cm_dev *cm;
	struct cm_peer *peer;
	u32 *entry_ptr = buf;
	int i = 0;

	/* Find a matching cm_dev object */
	list_for_each_entry(cm, &cm_dev_list, list)
		if (cm->mport->id == mport_id)
			goto found;

	*nent = 0;
	return -ENODEV;

found:
	list_for_each_entry(peer, &cm->peers, node) {
		*entry_ptr = (u32)peer->rdev->destid;
		entry_ptr++;
		if (++i >= *nent)
			break;
	}

	*nent = i;
	return 0;
}

/*
 * riocm_cdev_open() - Open character device (mport)
 */
static int riocm_cdev_open(struct inode *inode, struct file *filp)
{
	struct channel_dev *chdev = NULL;

	chdev = container_of(inode->i_cdev, struct channel_dev, cdev);
	filp->private_data = chdev;

	return 0;
}

/*
 * riocm_cdev_release() - Release character device
 */
static int riocm_cdev_release(struct inode *inode, struct file *filp)
{
	return 0;
}

unsigned int riocm_cdev_poll(struct file *file, poll_table *wait)
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

/*
 * cm_ep_get_list_size() - Reports number of endpoints in the network
 */
static int cm_ep_get_list_size(struct channel_dev *data, void __user *arg)
{
	u16 __user *p = arg;
	u32 mport_id;
	u32 count = 0;
	struct cm_dev *cm;

	if (get_user(mport_id, p))
		return -EFAULT;

	/* Find a matching cm_dev object */
	list_for_each_entry(cm, &cm_dev_list, list) {
		if (cm->mport->id == mport_id) {
			count = cm->npeers;
			if (copy_to_user(arg, &count, sizeof(count)))
				return -EFAULT;
			return 0;
		}
	}

	return -ENODEV;
}

/*
 * cm_ep_get_list() - Returns list of attached endpoints
 */
static int cm_ep_get_list(struct channel_dev *data, void __user *arg)
{
	int ret = 0;
	uint32_t info[2];
	void *buf;

	if (copy_from_user(&info, arg, sizeof(info)))
		return -EFAULT;

	buf = kcalloc(info[0] + 2, sizeof(u32), GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = riocm_get_peer_list(info[1], (u8 *)buf + 2*sizeof(u32), &info[0]);
	if (ret)
		goto out;

	((u32 *)buf)[0] = info[0]; /* report an updated number of entries */
	((u32 *)buf)[1] = info[1]; /* put back an mport ID */
	if (copy_to_user(arg, buf, sizeof(u32) * (info[0] + 2)))
		ret = -EFAULT;
out:
	kfree(buf);
	return ret;
}

/*
 * cm_mport_get_list() - Returns list of attached endpoints
 */
static int cm_mport_get_list(struct channel_dev *data, void __user *arg)
{
	int ret = 0;
	uint32_t entries;
	void *buf;
	struct cm_dev *cm;
	u32 *entry_ptr;
	int count = 0;

	if (copy_from_user(&entries, arg, sizeof(entries)))
		return -EFAULT;
	if (entries == 0)
		return -ENOMEM;
	buf = kcalloc(entries + 1, sizeof(u32), GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	/* Scan all registered cm_dev objects */
	entry_ptr = (u32 *)((u8 *)buf + sizeof(u32));
	list_for_each_entry(cm, &cm_dev_list, list) {
		if (count++ < entries) {
			*entry_ptr = (cm->mport->id << 16) | cm->mport->host_deviceid;
			entry_ptr++;
		}
	}

	*((u32 *)buf) = count; /* report a real number of entries */
	if (copy_to_user(arg, buf, sizeof(u32) * (count + 1)))
		ret = -EFAULT;

	kfree(buf);
	return ret;
}

/*
 * cm_chan_create() - Create a message exchange channel
 */
static int cm_chan_create(struct channel_dev *data, void __user *arg)
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

/*
 * cm_chan_close() - Close channel
 * @data:	Driver private data
 * @arg:	Channel to close
 */
static int cm_chan_close(struct channel_dev *data, void __user *arg)
{
	u16 __user *p = arg;
	u16 ch_num;

	if (get_user(ch_num, p))
		return -EFAULT;
	return riocm_ch_close(ch_num);
}

/*
 * cm_chan_bind() - Bind channel
 * @data:	Driver private data
 * @arg:	Channel number
 */
static int cm_chan_bind(struct channel_dev *data, void __user *arg)
{
	struct rio_cm_channel chan;

	if (copy_from_user(&chan, arg, sizeof(struct rio_cm_channel)))
		return -EFAULT;

	return riocm_ch_bind(chan.id, chan.mport_id, NULL);
}

/*
 * cm_chan_listen() - Listen on channel
 * @data:	Driver private data
 * @arg:	Channel number
 */
static int cm_chan_listen(struct channel_dev *data, void __user *arg)
{
	u16 __user *p = arg;
	u16 ch_num;

	if (get_user(ch_num, p))
		return -EFAULT;

	return riocm_ch_listen(ch_num);
}

/*
 * cm_chan_accept() - Accept incomming connection
 * @data:	Driver private data
 * @arg:	Channel number
 */
static int cm_chan_accept(struct channel_dev *data, void __user *arg)
{
	u16 __user *p = arg;
	u16 ch_num;
	int rc;

	if (get_user(ch_num, p))
		return -EFAULT;

	rc = riocm_ch_accept(ch_num, &ch_num, RIO_CM_ACCEPT_TO * HZ);
	if (rc)
		return rc;

	return put_user(ch_num, p);
}

/*
 * cm_chan_connect() - Connect on channel
 * @data:	Driver private data
 * @arg:	Channel information
 */
static int cm_chan_connect(struct channel_dev *data, void __user *arg)
{
	struct rio_cm_channel chan;

	if (copy_from_user(&chan, arg, sizeof(struct rio_cm_channel)))
		return -EFAULT;

	return riocm_ch_connect(chan.id, chan.mport_id,
				chan.remote_destid, chan.remote_channel);
}

/*
 * cm_chan_msg_send() - Connect on channel
 * @data:	Driver private data
 * @arg:	Outbound message information
 */
static int cm_chan_msg_send(struct channel_dev *data, void __user *arg)
{
	struct rio_cm_msg msg;
	void *buf;
	int ret = 0;

	if (copy_from_user(&msg, arg, sizeof(struct rio_cm_msg)))
		return -EFAULT;

	buf = kmalloc(RIO_MAX_MSG_SIZE, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	if (copy_from_user(buf, msg.msg, msg.size)) {
		ret = -EFAULT;
		goto out;
	}

	ret = riocm_ch_send(msg.ch_num, buf, msg.size);
out:
	kfree(buf);
	return ret;
}

/*
 * cm_chan_msg_rcv() - Connect on channel
 * @data:	Driver private data
 * @arg:	Inbound message information
 */
static int cm_chan_msg_rcv(struct channel_dev *data, void __user *arg)
{
	struct rio_cm_msg msg;
	void *buf;
	int msg_len = RIO_MAX_MSG_SIZE;
	int ret = 0;

	if (copy_from_user(&msg, arg, sizeof(struct rio_cm_msg)))
		return -EFAULT;

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
 * riocm_cdev_ioctl() - IOCTLs for character device
 */
static long
riocm_cdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	struct channel_dev *data = filp->private_data;

	switch (cmd) {
	case RIO_CM_EP_GET_LIST_SIZE:
		err = cm_ep_get_list_size(data, (void __user *)arg);
		break;
	case RIO_CM_EP_GET_LIST:
		err = cm_ep_get_list(data, (void __user *)arg);
		break;
	case RIO_CM_CHAN_CREATE:
		err = cm_chan_create(data, (void __user *)arg);
		break;
	case RIO_CM_CHAN_CLOSE:
		err = cm_chan_close(data, (void __user *)arg);
		break;
	case RIO_CM_CHAN_BIND:
		err = cm_chan_bind(data, (void __user *)arg);
		break;
	case RIO_CM_CHAN_LISTEN:
		err = cm_chan_listen(data, (void __user *)arg);
		break;
	case RIO_CM_CHAN_ACCEPT:
		err = cm_chan_accept(data, (void __user *)arg);
		break;
	case RIO_CM_CHAN_CONNECT:
		err = cm_chan_connect(data, (void __user *)arg);
		break;
	case RIO_CM_CHAN_SEND:
		err = cm_chan_msg_send(data, (void __user *)arg);
		break;
	case RIO_CM_CHAN_RECEIVE:
		err = cm_chan_msg_rcv(data, (void __user *)arg);
		break;
	case RIO_CM_MPORT_GET_LIST:
		err = cm_mport_get_list(data, (void __user *)arg);
		break;
	default:
		err = -EINVAL;
		break;
	}

	return err;
}

static const struct file_operations riocm_cdev_fops = {
	.owner		= THIS_MODULE,
	.open		= riocm_cdev_open,
	.release	= riocm_cdev_release,
	.poll		= riocm_cdev_poll,
	.unlocked_ioctl = riocm_cdev_ioctl,
};

/*
 * riocm_add_dev - add new remote RapidIO device into channel management core
 * @dev: device object associated with RapidIO device
 * @sif: subsystem interface
 *
 * Adds the specified RapidIO device (if applicable) into peers list of
 * the corresponding channel management device (cm_dev).
 */
static int riocm_add_dev(struct device *dev, struct subsys_interface *sif)
{
	struct cm_peer *peer;
	struct rio_dev *rdev = to_rio_dev(dev);
	struct cm_dev *cm;

	/* Check if the remote device has capabilities required to support CM */
	if (!dev_cm_capable(rdev))
		return 0;

	riocm_debug(DBG_RDEV, "%s(%s)", __func__, rio_name(rdev));

	/* Find a corresponding cm_dev object */
	spin_lock(&rio_list_lock);
	list_for_each_entry(cm, &cm_dev_list, list) {
		if (cm->mport == rdev->net->hport)
			goto found;
	}

	spin_unlock(&rio_list_lock);
	return -ENODEV;

found:
	peer = kmalloc(sizeof(struct cm_peer), GFP_KERNEL);
	if (!peer)
		return -ENOMEM;
	peer->rdev = rdev;
	list_add_tail(&peer->node, &cm->peers); // lock here ???
	cm->npeers++;

	spin_unlock(&rio_list_lock);
	return 0;
}

/*
 * riocm_remove_dev - remove remote RapidIO device from channel management core
 * @dev: device object associated with RapidIO device
 * @sif: subsystem interface
 *
 * Removes the specified RapidIO device (if applicable) from peers list of
 * the corresponding channel management device (cm_dev).
 */
static int riocm_remove_dev(struct device *dev, struct subsys_interface *sif)
{
	struct rio_dev *rdev = to_rio_dev(dev);
	struct cm_dev *cm;
	struct cm_peer *peer;

	/* Check if the remote device has capabilities required to support CM */
	if (!dev_cm_capable(rdev))
		return -ENODEV;

	riocm_debug(DBG_RDEV, "%s(%s)", __func__, rio_name(rdev));

	/* Find matching cm_dev object */
	spin_lock(&rio_list_lock);
	list_for_each_entry(cm, &cm_dev_list, list) {
		if (cm->mport == rdev->net->hport)
			goto found;
	}

	spin_unlock(&rio_list_lock);
	return -ENODEV;
found:
	list_for_each_entry(peer, &cm->peers, node) {
		if (peer->rdev == rdev) {
			riocm_debug(DBG_RDEV, "%s removing peer %s",
				 __func__, rio_name(rdev));
			list_del(&peer->node);
			cm->npeers--;
			kfree(peer);
			break;
		}
	}

	spin_unlock(&rio_list_lock);
	return 0;
}


/*
 * riocm_cdev_add() - Create rio_cm char device
 * @devno: device number assigned to device (MAJ + MIN)
 */
static struct channel_dev *riocm_cdev_add(dev_t devno)
{
	int ret = 0;
	struct channel_dev *device;

	device = kzalloc(sizeof(struct channel_dev), GFP_KERNEL);
	if (!device) {
		riocm_error("Unable allocate a device object");
		return NULL;
	}

	cdev_init(&device->cdev, &riocm_cdev_fops);
	device->cdev.owner = THIS_MODULE;
	ret = cdev_add(&device->cdev, devno, 1);
	if (ret < 0) {
		kfree(device);
		riocm_error("Cannot register a device with error %d", ret);
		return NULL;
	}

	device->dev = device_create(dev_class, NULL, devno, NULL, DEV_NAME);
	if (IS_ERR(device->dev)) {
		cdev_del(&device->cdev);
		kfree(device);
		return NULL;
	}

	riocm_debug(DBG_MPORT, "Added %s cdev(%d:%d)",
		    DEV_NAME, MAJOR(devno), MINOR(devno));

	return device;
}

/*
 * riocm_add_mport - add new local mport device into channel management core
 * @dev: device object associated with mport
 * @class_intf: class interface
 *
 * When a new mport device is added, CM immediately reserves inbound and
 * outbound RapidIO mailboxes that will be used.
 */
static int riocm_add_mport(struct device *dev,
			   struct class_interface *class_intf)
{
	int rc;
	int i;
	struct cm_dev *cm;
	struct rio_mport *mport = to_rio_mport(dev);

	riocm_debug(DBG_MPORT, "add mport %s", mport->name);

	cm = kmalloc(sizeof *cm, GFP_KERNEL);
	if (!cm)
		return -ENOMEM;

	cm->mport = mport;

	rc = rio_request_outb_mbox(mport, (void *)mport, cmbox,
				   RIOCM_TX_RING_SIZE, riocm_outb_msg_event);
	if (rc) {
		riocm_error("%s failed to allocate OBMBOX_%d on %s",
			    __func__, cmbox, mport->name);
		kfree(cm);
		return -ENODEV;
	}

	rc = rio_request_inb_mbox(mport, (void *)mport, cmbox,
				  RIOCM_RX_RING_SIZE, riocm_inb_msg_event);
	if (rc) {
		riocm_error("%s failed to allocate IBMBOX_%d on %s",
			    __func__, cmbox, mport->name);
		rio_release_outb_mbox(mport, cmbox);
		kfree(cm);
		return -ENODEV;
	}

	/*
	 * Allocate and register inbound messaging buffers to be ready
	 * to receive channel and system management requests
	 */
	for (i = 0; i < RIOCM_RX_RING_SIZE; i++)
		cm->rx_buf[i] = NULL;

	cm->rx_slots = RIOCM_RX_RING_SIZE;
	riocm_rx_fill(cm, RIOCM_RX_RING_SIZE);

	cm->tx_slot = 0;
	cm->tx_int_slot = 0;
	cm->tx_cnt = 0;
	cm->tx_ack_slot = 0;
	spin_lock_init(&cm->cm_lock);
	spin_lock_init(&cm->tx_lock);

	tasklet_init(&cm->rx_tasklet, rio_ibmsg_handler, (unsigned long)cm);
	tasklet_init(&cm->tx_tasklet, rio_txcq_handler, (unsigned long)cm);
	INIT_LIST_HEAD(&cm->peers);
	cm->npeers = 0;

	spin_lock(&rio_list_lock);
	list_add_tail(&cm->list, &cm_dev_list);
	spin_unlock(&rio_list_lock);

	return 0;
}

/*
 * riocm_remove_mport - remove local mport device from channel management core
 * @dev: device object associated with mport
 * @class_intf: class interface
 *
 * Removes a local mport device from the list of registered devices that provide
 * channel management services. Returns an error if the specified mport is not
 * registered with the CM core.
 */
static void riocm_remove_mport(struct device *dev,
			       struct class_interface *class_intf)
{
	struct rio_mport *mport = to_rio_mport(dev);
	int i;
	struct cm_dev *cm;

	riocm_debug(DBG_MPORT, "remove mport %s", mport->name);

	/* Find a matching cm_dev object */
	list_for_each_entry(cm, &cm_dev_list, list) {
		if (cm->mport == mport)
			goto found;
	}

	return;
found:
	spin_lock(&rio_list_lock);
	list_del(&cm->list);
	spin_unlock(&rio_list_lock);

	rio_release_inb_mbox(mport, cmbox);
	rio_release_outb_mbox(mport, cmbox);

	for (i = 0; i < RIOCM_RX_RING_SIZE; i++)
		kfree(cm->rx_buf[i]);
	kfree(cm);
}

/*
 * riocm_interface handles addition/removal of remote RapidIO devices
 */
static struct subsys_interface riocm_interface = {
	.name		= "rio_cm",
	.subsys		= &rio_bus_type,
	.add_dev	= riocm_add_dev,
	.remove_dev	= riocm_remove_dev,
};

/*
 * rio_mport_interface handles addition/removal local mport devices
 */
static struct class_interface rio_mport_interface __refdata = {
	.class = &rio_mport_class,
	.add_dev = riocm_add_mport,
	.remove_dev = riocm_remove_mport,
};

static int __init riocm_init(void)
{
	int ret;

	/* Create device class needed by udev */
	dev_class = class_create(THIS_MODULE, DRV_NAME);
	if (!dev_class) {
		riocm_error("Cannot create " DRV_NAME " class");
		return -EINVAL;
	}

#if (0)
	/* Get class major number */
	dev_major = riocm_cdev_init();
	if (dev_major < 0) {
		class_destroy(dev_class);
		return -ENOMEM;
	}
#else
	ret = alloc_chrdev_region(&dev_number, 0, 1, DRV_NAME);
	if (ret) {
		class_destroy(dev_class);
		return ret;
	}

	dev_major = MAJOR(dev_number);
	dev_minor_base = MINOR(dev_number);
#endif

	riocm_debug(DBG_INIT, "Registered class with %d major", dev_major);

	riocm_wq = create_singlethread_workqueue("riocm_wq");
	if (!riocm_wq)
		return -ENOMEM;

	/*
	 * Register as rapidio_port class interface to get notifications about
	 * mport additions and removals.
	 */
	ret = class_interface_register(&rio_mport_interface);
	if (ret) {
		riocm_error("class_interface_register error: %d", ret);
		goto err_wq;
	}

	/*
	 * Register as RapidIO bus interface to get notifications about
	 * addition/removal of remote RapidIO devices.
	 */
	ret = subsys_interface_register(&riocm_interface);
	if (ret) {
		riocm_error("subsys_interface_register error: %d", ret);
		goto err_cl;
	}

	riocm_cdev = riocm_cdev_add(dev_number);
	if (!riocm_cdev) {
		subsys_interface_unregister(&riocm_interface);
		ret = -ENODEV;
		goto err_cl;
	}

	return 0;
err_cl:
	class_interface_unregister(&rio_mport_interface);
err_wq:
	destroy_workqueue(riocm_wq);
	return ret;
}

static void __exit riocm_exit(void)
{
	riocm_debug(DBG_EXIT, "enter %s", __func__);
	subsys_interface_unregister(&riocm_interface);
	class_interface_unregister(&rio_mport_interface);
	destroy_workqueue(riocm_wq);
	idr_destroy(&ch_idr);

	device_unregister(riocm_cdev->dev);
	cdev_del(&(riocm_cdev->cdev));

	class_destroy(dev_class);
	unregister_chrdev_region(dev_number, 1);
}

late_initcall(riocm_init);
module_exit(riocm_exit);
