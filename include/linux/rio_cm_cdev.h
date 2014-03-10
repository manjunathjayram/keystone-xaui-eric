/* include/linux/rio_cm_cdev.h
 *
 * Copyright (C) 2013 Prodrive B.V.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef _RIO_MPORT_H_
#define _RIO_MPORT_H_

#ifndef __user
#define __user
#endif

struct rio_cm_channel {
	uint16_t id;
	uint32_t remote_destid;
	uint32_t remote_mbox;
	uint16_t remote_channel;
	uint8_t mport_id;
};

struct rio_cm_msg {
	uint16_t ch_num;
	void __user *msg;
	uint16_t size;
};

#define RIO_CM_ACCEPT_TO	180 /* Accept time-out in seconds */

/* RapidIO Channel Manager driver IOCTLs */
#define RIO_CM_IOC_MAGIC	'c'

#define RIO_CM_EP_GET_LIST_SIZE		_IOWR(RIO_CM_IOC_MAGIC, 1, uint32_t *)
#define RIO_CM_EP_GET_LIST		_IOWR(RIO_CM_IOC_MAGIC, 2, uint32_t *)
#define RIO_CM_CHAN_CREATE		_IOR(RIO_CM_IOC_MAGIC, 3, uint16_t *)
#define RIO_CM_CHAN_CLOSE		_IOR(RIO_CM_IOC_MAGIC, 4, uint16_t *)
#define RIO_CM_CHAN_BIND		_IOR(RIO_CM_IOC_MAGIC, 5, struct rio_cm_channel *)
#define RIO_CM_CHAN_LISTEN		_IOR(RIO_CM_IOC_MAGIC, 6, uint16_t *)
#define RIO_CM_CHAN_ACCEPT		_IOR(RIO_CM_IOC_MAGIC, 7, uint16_t *)
#define RIO_CM_CHAN_CONNECT		_IOR(RIO_CM_IOC_MAGIC, 8, struct rio_cm_channel *)
#define RIO_CM_CHAN_SEND		_IOR(RIO_CM_IOC_MAGIC, 9, struct rio_cm_msg *)
#define RIO_CM_CHAN_RECEIVE		_IOR(RIO_CM_IOC_MAGIC, 10, struct rio_cm_msg *)
#define RIO_CM_MPORT_GET_LIST		_IOWR(RIO_CM_IOC_MAGIC, 11 , uint32_t *)

#endif /* _RIO_MPORT_H_ */
