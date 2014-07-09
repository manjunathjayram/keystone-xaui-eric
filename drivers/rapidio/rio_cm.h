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

#ifndef _RIO_CM_H
#define _RIO_CM_H

#include <linux/bitops.h>
#include <linux/printk.h>

#define DRIVER_PREFIX "RIO_CM: "

/* Debug output filtering masks */
enum {
	DBG_NONE	= 0,
	DBG_INIT	= BIT(0), /* driver init */
	DBG_EXIT	= BIT(1), /* driver exit */
	DBG_MPORT	= BIT(2), /* mport add/remove */
	DBG_RDEV	= BIT(3), /* RapidIO device add/remove */
	DBG_CHOP	= BIT(4), /* channel operations */
	DBG_WAIT	= BIT(5), /* waiting for events */
	DBG_TX		= BIT(6), /* message TX */
	DBG_TX_EVENT	= BIT(7), /* message TX event */
	DBG_RX_DATA	= BIT(8), /* inbound data messages */
	DBG_RX_CMD	= BIT(9), /* inbound REQ/ACK/NACK messages */
	DBG_ALL		= ~0,
};

#ifdef DEBUG
#define riocm_debug(level, fmt, arg...) \
	do { \
		if (level & dbg_level) \
			printk(KERN_DEBUG pr_fmt(DRIVER_PREFIX fmt "\n"), \
			       ##arg); \
	} while (0)
#else
#define riocm_debug(level, fmt, arg...) \
		no_printk(KERN_DEBUG pr_fmt(DRIVER_PREFIX fmt "\n"), ##arg)
#endif

#define riocm_warn(fmt, arg...) \
	pr_warning(DRIVER_PREFIX "WARNING " fmt "\n", ##arg)

#define riocm_error(fmt, arg...) \
	pr_err(DRIVER_PREFIX "ERROR " fmt "\n", ##arg)

#define RIOCM_MAX_CHNUM		0xffff /* Use full range of u16 field */
#define RIOCM_CHNUM_AUTO	0

extern int riocm_ch_create(u16 *ch_num);
extern int riocm_ch_close(u16 ch_id);
extern int riocm_ch_bind(u16 ch_id, u8 mport_id, void *context);
extern int riocm_ch_listen(u16 ch_id);
extern int riocm_ch_accept(u16 ch_id, u16 *new_ch_id, long timeout);
extern int riocm_ch_connect(u16 loc_ch, u8 mport_id,
			    u32 rem_destid, u16 rem_ch);
extern int riocm_ch_send(u16 ch_id, void *buf, int len);
extern int riocm_ch_receive(u16 ch_id, void **buf, int *len);
extern int riocm_ch_free_rxbuf(u16 ch_id, void *buf);

#endif /* _RIO_CM_H */
