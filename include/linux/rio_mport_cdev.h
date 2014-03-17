/* include/linux/rio_mport_cdev.h
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
#ifndef _RIO_MPORT_CDEV_H_
#define _RIO_MPORT_CDEV_H_

#ifndef __user
#define __user
#endif

struct rio_mport_dbell {
	uint16_t id;
	uint16_t num;
};

#ifdef CONFIG_TI_KEYSTONE_RAPIDIO
struct rio_mport_dio_transfer {
	uint16_t id;
	uint64_t tgt_addr;      /* physical RapidIO address on 64 bits */
        void __user *src_addr;  /* user virtual source address on 32 bits */
	uint32_t length;
	uint16_t mode;
};
#endif

#ifdef CONFIG_RAPIDIO_DMA_ENGINE
struct rio_mport_dma_transfer {
	uint16_t id;
	uint64_t tgt_addr;      /* physical RapidIO address on 64 bits */
        void __user *loc_addr;  /* user virtual source address on 32 bits */
	uint32_t length;
	uint16_t mode;
};
#endif

struct rio_mport_maint_transfer {
	uint16_t id;       /* for remote config only */
	uint16_t hopcount; /* for remote config only */
	uint32_t offset;
	uint32_t length;
	uint32_t __user *val;
};

/* Mport DirectI/O and doorbells IOCTLs */
#define RIO_MPORT_DIO_AND_DBELL_MAGIC   'd'

#define RIO_MPORT_DBELL_SEND                    _IOR(RIO_MPORT_DIO_AND_DBELL_MAGIC, 1, struct rio_mport_dbell *)
#define RIO_MPORT_DBELL_RECEIVE                 _IOR(RIO_MPORT_DIO_AND_DBELL_MAGIC, 2, struct rio_mport_dbell *)

#ifdef CONFIG_TI_KEYSTONE_RAPIDIO
#define RIO_MPORT_DIO_TRANSFER                  _IOR(RIO_MPORT_DIO_AND_DBELL_MAGIC, 3, struct rio_mport_dio_transfer *)
#endif

#ifdef CONFIG_RAPIDIO_DMA_ENGINE
#define RIO_MPORT_DMA_GET_XFER_SIZE             _IOR(RIO_MPORT_DIO_AND_DBELL_MAGIC, 4, uint32_t *)
#define RIO_MPORT_DMA_READ                      _IOR(RIO_MPORT_DIO_AND_DBELL_MAGIC, 5, struct rio_mport_dma_transfer *)
#define RIO_MPORT_DMA_WRITE                     _IOR(RIO_MPORT_DIO_AND_DBELL_MAGIC, 6, struct rio_mport_dma_transfer *)
#endif

/* Maintenance IOCTLs */
#define RIO_MPORT_MAINT_MAGIC           'm'

#define RIO_MPORT_MAINT_LOCAL_CONFIG_READ       _IOR(RIO_MPORT_MAINT_MAGIC, 1, struct rio_mport_maint_transfer *)
#define RIO_MPORT_MAINT_LOCAL_CONFIG_WRITE      _IOR(RIO_MPORT_MAINT_MAGIC, 2, struct rio_mport_maint_transfer *)
#define RIO_MPORT_MAINT_CONFIG_READ             _IOR(RIO_MPORT_MAINT_MAGIC, 3, struct rio_mport_maint_transfer *)
#define RIO_MPORT_MAINT_CONFIG_WRITE            _IOR(RIO_MPORT_MAINT_MAGIC, 4, struct rio_mport_maint_transfer *)

#define RIO_MPORT_MAINT_HDID_SET                _IOR(RIO_MPORT_MAINT_MAGIC, 5, uint16_t *)
#define RIO_MPORT_MAINT_COMPTAG_SET             _IOR(RIO_MPORT_MAINT_MAGIC, 7, uint32_t *)
#define RIO_MPORT_MAINT_PORT_IDX_GET            _IOR(RIO_MPORT_MAINT_MAGIC, 8, uint32_t *)

#endif /* _RIO_MPORT_CDEV_H_ */
