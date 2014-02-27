/* include/linux/rio_mport.h
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

struct rio_mport_channel {
	uint16_t id;
	uint32_t remote_destid;
	uint32_t remote_mbox;
	uint16_t remote_channel;
};

struct rio_mport_channel_msg {
	struct rio_mport_channel *channel;
	uint32_t offset;
};

struct rio_mport_ch_msg {
	uint16_t ch_num;
	void __user *msg;
	uint16_t size;
};

struct rio_mport_dbell {
	uint16_t id;
	uint16_t num;
};

struct rio_mport_dio_transfer {
	uint16_t id;
	uint64_t tgt_addr;      /* physical RapidIO address on 64 bits */
        void __user *src_addr;  /* user virtual source address on 32 bits */
	uint32_t length;
	uint16_t mode;
};

struct rio_mport_maint_transfer {
	uint16_t id;       /* for remote config only */
	uint16_t hopcount; /* for remote config only */
	uint32_t offset;
	uint32_t length;
	uint32_t __user *val;
};

#define RIO_MPORT_CHANNEL_ACCEPT_TO	180 /* Accept time-out in seconds */

/* Mport channel driver IOCTLs */
#define RIO_MPORT_CHANNEL_IOC_MAGIC	'c'

#define RIO_MPORT_CHANNEL_EP_GET_LIST_SIZE	_IOWR(RIO_MPORT_CHANNEL_IOC_MAGIC, 1, uint32_t *)
#define RIO_MPORT_CHANNEL_EP_GET_LIST		_IOWR(RIO_MPORT_CHANNEL_IOC_MAGIC, 2, uint32_t *)

#define RIO_MPORT_CHANNEL_CREATE		_IOR(RIO_MPORT_CHANNEL_IOC_MAGIC, 3, uint16_t *)
#define RIO_MPORT_CHANNEL_CLOSE			_IOR(RIO_MPORT_CHANNEL_IOC_MAGIC, 4, uint16_t *)
#define RIO_MPORT_CHANNEL_BIND			_IOR(RIO_MPORT_CHANNEL_IOC_MAGIC, 5, uint16_t *)
#define RIO_MPORT_CHANNEL_LISTEN		_IOR(RIO_MPORT_CHANNEL_IOC_MAGIC, 6, uint16_t *)
#define RIO_MPORT_CHANNEL_ACCEPT		_IOR(RIO_MPORT_CHANNEL_IOC_MAGIC, 7, uint16_t *)
#define RIO_MPORT_CHANNEL_CONNECT		_IOR(RIO_MPORT_CHANNEL_IOC_MAGIC, 8, struct rio_mport_channel *)
#define RIO_MPORT_CHANNEL_SEND			_IOR(RIO_MPORT_CHANNEL_IOC_MAGIC, 9, struct rio_mport_ch_msg *)
#define RIO_MPORT_CHANNEL_RECEIVE		_IOR(RIO_MPORT_CHANNEL_IOC_MAGIC, 10, struct rio_mport_ch_msg *)

/* Mport DirectI/O and doorbells IOCTLs */
#define RIO_MPORT_DIO_AND_DBELL_MAGIC   'd'

#define RIO_MPORT_DBELL_SEND                    _IOR(RIO_MPORT_DIO_AND_DBELL_MAGIC, 1, struct rio_mport_dbell *)
#define RIO_MPORT_DBELL_RECEIVE                 _IOR(RIO_MPORT_DIO_AND_DBELL_MAGIC, 2, struct rio_mport_dbell *)
#define RIO_MPORT_DIO_TRANSFER                  _IOR(RIO_MPORT_DIO_AND_DBELL_MAGIC, 3, struct rio_mport_dio_transfer *)

/* Maintenance IOCTLs */
#define RIO_MPORT_MAINT_MAGIC           'm'

#define RIO_MPORT_MAINT_LOCAL_CONFIG_READ       _IOR(RIO_MPORT_MAINT_MAGIC, 1, struct rio_mport_maint_transfer *)
#define RIO_MPORT_MAINT_LOCAL_CONFIG_WRITE      _IOR(RIO_MPORT_MAINT_MAGIC, 2, struct rio_mport_maint_transfer *)
#define RIO_MPORT_MAINT_CONFIG_READ             _IOR(RIO_MPORT_MAINT_MAGIC, 3, struct rio_mport_maint_transfer *)
#define RIO_MPORT_MAINT_CONFIG_WRITE            _IOR(RIO_MPORT_MAINT_MAGIC, 4, struct rio_mport_maint_transfer *)

#define RIO_MPORT_MAINT_HDID_SET                _IOR(RIO_MPORT_MAINT_MAGIC, 5, uint16_t *)
#define RIO_MPORT_MAINT_COMPTAG_SET             _IOR(RIO_MPORT_MAINT_MAGIC, 7, uint32_t *)
#define RIO_MPORT_MAINT_PORT_IDX_GET            _IOR(RIO_MPORT_MAINT_MAGIC, 8, uint32_t *)

#endif /* _RIO_MPORT_H_ */
