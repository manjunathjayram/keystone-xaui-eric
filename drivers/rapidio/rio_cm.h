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

#ifndef RIO_CM_H
#define RIO_CM_H

//#define RIOCM_MAX_CHNUM		0xffff
//#define RIOCM_CHNUM_AUTO	(-1)
//#define RIOCM_CONNECT_TO	3 /* connect response TO (in sec) */

extern int riocm_get_peer_count(struct rio_mport *mport, u32 *npeers);
extern int riocm_get_peer_list(struct rio_mport *mport, void *buf, u32 *nent);
extern int riocm_ch_create(int ch_num);
extern int riocm_ch_close(int ch_id);
extern int riocm_ch_bind(int ch_id, struct rio_mport *mport, void *context);
extern int riocm_ch_listen(int ch_id);
extern int riocm_ch_accept(int ch_id, int *new_ch_id, long timeout);
extern int riocm_ch_connect(int loc_ch, struct rio_mport *mport,
			    u32 rem_destid, int rem_ch);
extern int riocm_ch_send(int ch_id, void *buf, int len);
extern int riocm_ch_receive(int ch_id, void **buf, int *len);
extern int riocm_ch_free_rxbuf(int ch_id, void *buf);

#endif /* RIO_CM_H */
