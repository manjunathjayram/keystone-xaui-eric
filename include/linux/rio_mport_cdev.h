/* include/linux/rio_mport_cdev.h
 *
 * Copyright (C) 2014 Integrated Device Technology, Inc.
 * Copyright (C) 2014 Texas Instruments Incorporated
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
#ifndef _RIO_MPORT_CDEV_H_
#define _RIO_MPORT_CDEV_H_

#ifndef __user
#define __user
#endif

struct rio_mport_dbell {
	uint16_t id;
	uint16_t num;
};

#ifdef CONFIG_RAPIDIO_DMA_ENGINE
struct rio_mport_dma_transfer {
	uint16_t id;
	uint64_t tgt_addr;      /* RapidIO address (64 bits) */
	void __user *loc_addr;
	uint64_t handle;
	uint32_t offset;
	uint32_t length;
	uint16_t mode;
};

/*
 * RapidIO specification defines two types of data write requests: NWRITE and
 * NWRITE_R (write-with-response). RapidIO DMA channel interface allows to
 * specify required type of write operation or combination of them when only
 * the last data packet requires response.
 */
#define	RIO_DMA_DEFAULT		0 /* default method used by DMA driver */
#define	RIO_DMA_ALL_NWRITE	1 /* send all packets using NWRITE */
#define	RIO_DMA_ALL_NWRITE_R	2 /* send all packets using NWRITE_R */
#define	RIO_DMA_LAST_NWRITE_R	3 /* last packet NWRITE_R, others - NWRITE */

struct rio_mport_dma_buf {
	uint32_t length;
	uint64_t __user *handle;
};

#endif

struct rio_mport_maint_transfer {
	uint16_t destid;   /* for remote config only */
	uint16_t hopcount; /* for remote config only */
	uint32_t offset;
	uint32_t length;
	uint32_t __user *val;
};

struct rio_mport_inbound_map {
	uint64_t rio_base;
	uint32_t length;
	uint64_t __user *handle;
};

struct rio_mport_query_resp {
	uint16_t hdid;
	uint8_t  id;
	uint8_t  index;
	uint32_t flags;
	uint8_t  sys_size;
	uint8_t  port_ok;
	uint8_t  link_speed;
	uint8_t  link_width;
	uint32_t dma_max_sge;
	uint32_t dma_max_size;
	uint32_t dma_align;
};

/* Mport DirectI/O and doorbells IOCTLs */
#define RIO_MPORT_DIO_AND_DBELL_MAGIC   'd'

#define RIO_MPORT_DBELL_SEND		\
	_IOW(RIO_MPORT_DIO_AND_DBELL_MAGIC, 1, struct rio_mport_dbell)
#define RIO_MPORT_DBELL_RECEIVE		\
	_IOWR(RIO_MPORT_DIO_AND_DBELL_MAGIC, 2, struct rio_mport_dbell)
#ifdef CONFIG_RAPIDIO_DMA_ENGINE
#define RIO_MPORT_DMA_GET_XFER_SIZE	\
	_IOR(RIO_MPORT_DIO_AND_DBELL_MAGIC, 4, uint32_t)
#define RIO_MPORT_DMA_READ		\
	_IOW(RIO_MPORT_DIO_AND_DBELL_MAGIC, 5, struct rio_mport_dma_transfer)
#define RIO_MPORT_DMA_WRITE		\
	_IOW(RIO_MPORT_DIO_AND_DBELL_MAGIC, 6, struct rio_mport_dma_transfer)
#define RIO_MPORT_DMA_BUF_ALLOC		\
	_IOWR(RIO_MPORT_DIO_AND_DBELL_MAGIC, 7, struct rio_mport_dma_buf)
#define RIO_MPORT_DMA_BUF_FREE		\
	_IOW(RIO_MPORT_DIO_AND_DBELL_MAGIC, 8, uint64_t)
#endif

/* Maintenance IOCTLs */
#define RIO_MPORT_MAINT_MAGIC           'm'

#define RIO_MPORT_MAINT_LOCAL_CONFIG_READ	\
	_IOWR(RIO_MPORT_MAINT_MAGIC, 1, struct rio_mport_maint_transfer)
#define RIO_MPORT_MAINT_LOCAL_CONFIG_WRITE	\
	_IOW(RIO_MPORT_MAINT_MAGIC, 2, struct rio_mport_maint_transfer)
#define RIO_MPORT_MAINT_CONFIG_READ		\
	_IOWR(RIO_MPORT_MAINT_MAGIC, 3, struct rio_mport_maint_transfer)
#define RIO_MPORT_MAINT_CONFIG_WRITE		\
	_IOW(RIO_MPORT_MAINT_MAGIC, 4, struct rio_mport_maint_transfer)

#define RIO_MPORT_MAINT_HDID_SET	\
	_IOW(RIO_MPORT_MAINT_MAGIC, 5, uint16_t)
#define RIO_MPORT_MAINT_COMPTAG_SET	\
	_IOW(RIO_MPORT_MAINT_MAGIC, 7, uint32_t)
#define RIO_MPORT_MAINT_PORT_IDX_GET	\
	_IOR(RIO_MPORT_MAINT_MAGIC, 8, uint32_t)
#define RIO_MPORT_QUERY_DEVICE	\
	_IOR(RIO_MPORT_MAINT_MAGIC, 9, struct rio_mport_query_resp)

/* Inbound Memory Mapping IOCTLs */
#define RIO_MPORT_IB_MAGIC	'i'

#define RIO_MPORT_INBOUND_ALLOC		\
	_IOWR(RIO_MPORT_IB_MAGIC, 1, struct rio_mport_inbound_map)
#define RIO_MPORT_INBOUND_FREE		\
	_IOW(RIO_MPORT_IB_MAGIC, 2, uint64_t)

#endif /* _RIO_MPORT_CDEV_H_ */
