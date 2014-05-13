/*
 * include/linux/rio_dev.h
 *
 * Copyright (C) 2014 Texas Instruments Incorporated
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
#ifndef _RIO_DEV_H_
#define _RIO_DEV_H_

/* Direct I/O modes */
#define RIO_DIO_MODE_READ    0x0000
#define RIO_DIO_MODE_WRITER  0x0001
#define RIO_DIO_MODE_WRITE   0x0002
#define RIO_DIO_MODE_SWRITE  0x0003

/* IOCTLs definitions */
#define RIO_DIO_BASE_SET     _IOR('R', 0, int) /* Set base offset */
#define RIO_DIO_BASE_GET     _IOW('R', 1, int) /* Get base offset */
#define RIO_DIO_MODE_SET     _IOR('R', 2, int) /* Set Direct I/O mode */
#define RIO_DIO_MODE_GET     _IOW('R', 3, int) /* Get Direct I/O mode */
#define RIO_DBELL_TX         _IOR('R', 4, int) /* Send a doorbell */
#define RIO_DBELL_RX         _IOR('R', 5, int) /* Receive a doorbell */

#endif /* _RIO_DEV_H_ */
