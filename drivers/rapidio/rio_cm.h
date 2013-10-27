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

#define RIOCM_MAX_CHNUM		0xffff /* Use full range of u16 field */
#define RIOCM_CHNUM_AUTO	0

extern int riocm_get_peer_count(struct rio_mport *mport, u32 *npeers);
extern int riocm_get_peer_list(struct rio_mport *mport, void *buf, u32 *nent);
extern int riocm_ch_create(u16 *ch_num);
extern int riocm_ch_close(u16 ch_id);
extern int riocm_ch_bind(u16 ch_id, struct rio_mport *mport, void *context);
extern int riocm_ch_listen(u16 ch_id);
extern int riocm_ch_accept(u16 ch_id, u16 *new_ch_id, long timeout);
extern int riocm_ch_connect(u16 loc_ch, struct rio_mport *mport,
			    u32 rem_destid, u16 rem_ch);
extern int riocm_ch_send(u16 ch_id, void *buf, int len);
extern int riocm_ch_receive(u16 ch_id, void **buf, int *len);
extern int riocm_ch_free_rxbuf(u16 ch_id, void *buf);

#endif /* _RIO_CM_H */
