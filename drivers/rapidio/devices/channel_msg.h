/*
 * drivers/rapidio/mport/channel_msg.h: Include file for RIO message header routines
 * Based on net/tipc/msg.h
 *
 * Copyright (c) 2013, Prodrive B.V.
 * Copyright (c) 2000-2007, Ericsson AB
 * Copyright (c) 2005-2008, 2010-2011, Wind River Systems
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the names of the copyright holders nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _RIO_MSG_H
#define _RIO_MSG_H

#define RIO_MSG_HDR_SIZE 20

struct rio_channel_msg {
	__be32 hdr[5];
};

static inline u32 rio_msg_word(struct rio_channel_msg *m, u32 pos)
{
	return m->hdr[pos];
}

static inline void rio_msg_set_word(struct rio_channel_msg *m, u32 w, u32 val)
{
	m->hdr[w] = val;
}

static inline u32 rio_msg_bits(struct rio_channel_msg *m, u32 w, u32 pos, u32 mask)
{
	return (rio_msg_word(m, w) >> pos) & mask;
}

static inline void rio_msg_set_bits(struct rio_channel_msg *m, u32 w,
				u32 pos, u32 mask, u32 val)
{
	val = (val & mask) << pos;
	mask = mask << pos;
	m->hdr[w] &= ~mask;
	m->hdr[w] |= val;
}

/* Word 1 - src_id */
static inline u32 rio_msg_src_id(struct rio_channel_msg *m)
{
	return rio_msg_word(m, 0);
}

static inline void rio_msg_set_src_id(struct rio_channel_msg *m, u32 n)
{
	rio_msg_set_word(m, 0, n);
}

/* Word 2 - dest_id */
static inline u32 rio_msg_dest_id(struct rio_channel_msg *m)
{
	return rio_msg_word(m, 1);
}

static inline void rio_msg_set_dest_id(struct rio_channel_msg *m, u32 n)
{
	rio_msg_set_word(m, 1, n);
}

/* Word 3 */
static inline u8 rio_msg_src_mbox(struct rio_channel_msg *m)
{
	return rio_msg_bits(m, 2, 0, 0xff);
}

static inline void rio_msg_set_src_mbox(struct rio_channel_msg *m, u32 n)
{
	rio_msg_set_bits(m, 2, 0, 0xff, n);
}

static inline u8 rio_msg_dest_mbox(struct rio_channel_msg *m)
{
	return rio_msg_bits(m, 2, 8, 0xff);
}

static inline void rio_msg_set_dest_mbox(struct rio_channel_msg *m, u32 n)
{
	rio_msg_set_bits(m, 2, 8, 0xff, n);
}

static inline u8 rio_msg_type(struct rio_channel_msg *m)
{
	return rio_msg_bits(m, 2, 16, 0xff);
}

static inline void rio_msg_set_type(struct rio_channel_msg *m, u32 n)
{
	rio_msg_set_bits(m, 2, 16, 0xff, n);
}

static inline u8 rio_msg_sock_type(struct rio_channel_msg *m)
{
	return rio_msg_bits(m, 2, 24, 0xff);
}

static inline void rio_msg_set_sock_type(struct rio_channel_msg *m, u32 n)
{
	rio_msg_set_bits(m, 2, 24, 0xff, n);
}

/* Word 4 */
static inline u16 rio_msg_dest_chan(struct rio_channel_msg *m)
{
	return rio_msg_bits(m, 3, 0, 0xffff);
}

static inline void rio_msg_set_dest_chan(struct rio_channel_msg *m, u32 n)
{
	rio_msg_set_bits(m, 3, 0, 0xffff, n);
}

static inline u8 rio_msg_src_chan(struct rio_channel_msg *m)
{
	return rio_msg_bits(m, 3, 16, 0xffff);
}

static inline void rio_msg_set_src_chan(struct rio_channel_msg *m, u32 n)
{
	rio_msg_set_bits(m, 3, 16, 0xffff, n);
}

#endif

