/*
 * Texas Instruments 3-Port Ethernet Switch Address Lookup Engine
 *
 * Copyright (C) 2012 Texas Instruments
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/stat.h>
#include <linux/sysfs.h>
#include <linux/etherdevice.h>

#include "cpsw_ale.h"

#define BITMASK(bits)		(BIT(bits) - 1)
#define ADDR_FMT_STR		"%02x:%02x:%02x:%02x:%02x:%02x"
#define ADDR_FMT_ARGS(addr)	(addr)[0], (addr)[1], (addr)[2], \
				(addr)[3], (addr)[4], (addr)[5]
#define ALE_ENTRY_BITS		68
#define ALE_ENTRY_WORDS	DIV_ROUND_UP(ALE_ENTRY_BITS, 32)

#define ALE_VERSION_MAJOR(rev)	((rev >> 8) & 0xff)
#define ALE_VERSION_MINOR(rev)	(rev & 0xff)

/* ALE Registers */
#define ALE_IDVER		0x00
#define ALE_CONTROL		0x08
#define ALE_PRESCALE		0x10
#define ALE_UNKNOWNVLAN		0x18
#define ALE_TABLE_CONTROL	0x20
#define ALE_TABLE		0x34
#define ALE_PORTCTL		0x40

#define ALE_TABLE_WRITE		BIT(31)

#define ALE_TYPE_FREE			0
#define ALE_TYPE_ADDR			1
#define ALE_TYPE_VLAN			2
#define ALE_TYPE_VLAN_ADDR		3

#define ALE_UCAST_PERSISTANT		0
#define ALE_UCAST_UNTOUCHED		1
#define ALE_UCAST_OUI			2
#define ALE_UCAST_TOUCHED		3

#define ALE_TBL_ENTRY_SHOW_LEN		160
#define ALE_RAW_TBL_ENTRY_SHOW_LEN	32

static inline int cpsw_ale_get_field(u32 *ale_entry, u32 start, u32 bits)
{
	int idx;

	idx    = start / 32;
	start -= idx * 32;
	idx    = 2 - idx; /* flip */
	return (ale_entry[idx] >> start) & BITMASK(bits);
}

static inline void cpsw_ale_set_field(u32 *ale_entry, u32 start, u32 bits,
				      u32 value)
{
	int idx;

	value &= BITMASK(bits);
	idx    = start / 32;
	start -= idx * 32;
	idx    = 2 - idx; /* flip */
	ale_entry[idx] &= ~(BITMASK(bits) << start);
	ale_entry[idx] |=  (value << start);
}

#define DEFINE_ALE_FIELD(name, start, bits)				\
static inline int cpsw_ale_get_##name(u32 *ale_entry)			\
{									\
	return cpsw_ale_get_field(ale_entry, start, bits);		\
}									\
static inline void cpsw_ale_set_##name(u32 *ale_entry, u32 value)	\
{									\
	cpsw_ale_set_field(ale_entry, start, bits, value);		\
}

DEFINE_ALE_FIELD(entry_type,		60,	2)
DEFINE_ALE_FIELD(vlan_id,		48,	12)
DEFINE_ALE_FIELD(mcast_state,		62,	2)
DEFINE_ALE_FIELD(port_mask,		66,     3)
DEFINE_ALE_FIELD(super,			65,	1)
DEFINE_ALE_FIELD(ucast_type,		62,     2)
DEFINE_ALE_FIELD(port_num,		66,     2)
DEFINE_ALE_FIELD(blocked,		65,     1)
DEFINE_ALE_FIELD(secure,		64,     1)
DEFINE_ALE_FIELD(vlan_untag_force,	24,	3)
DEFINE_ALE_FIELD(vlan_reg_mcast,	16,	3)
DEFINE_ALE_FIELD(vlan_unreg_mcast,	8,	3)
DEFINE_ALE_FIELD(vlan_member_list,	0,	3)
DEFINE_ALE_FIELD(mcast,			40,	1)

/* The MAC address field in the ALE entry cannot be macroized as above */
static inline void cpsw_ale_get_addr(u32 *ale_entry, u8 *addr)
{
	int i;

	for (i = 0; i < 6; i++)
		addr[i] = cpsw_ale_get_field(ale_entry, 40 - 8*i, 8);
}

static inline void cpsw_ale_set_addr(u32 *ale_entry, u8 *addr)
{
	int i;

	for (i = 0; i < 6; i++)
		cpsw_ale_set_field(ale_entry, 40 - 8*i, 8, addr[i]);
}

static int cpsw_ale_read(struct cpsw_ale *ale, int idx, u32 *ale_entry)
{
	int i;

	WARN_ON(idx > ale->params.ale_entries);

	__raw_writel(idx, ale->params.ale_regs + ALE_TABLE_CONTROL);

	for (i = 0; i < ALE_ENTRY_WORDS; i++)
		ale_entry[i] = __raw_readl(ale->params.ale_regs +
					   ALE_TABLE + 4 * i);

	return idx;
}

static int cpsw_ale_write(struct cpsw_ale *ale, int idx, u32 *ale_entry)
{
	int i;

	WARN_ON(idx > ale->params.ale_entries);

	for (i = 0; i < ALE_ENTRY_WORDS; i++)
		__raw_writel(ale_entry[i], ale->params.ale_regs +
			     ALE_TABLE + 4 * i);

	__raw_writel(idx | ALE_TABLE_WRITE, ale->params.ale_regs +
		     ALE_TABLE_CONTROL);

	return idx;
}

int cpsw_ale_match_addr(struct cpsw_ale *ale, u8 *addr, u16 vid)
{
	u32 ale_entry[ALE_ENTRY_WORDS];
	int type, idx;

	for (idx = 0; idx < ale->params.ale_entries; idx++) {
		u8 entry_addr[6];

		cpsw_ale_read(ale, idx, ale_entry);
		type = cpsw_ale_get_entry_type(ale_entry);
		if (type != ALE_TYPE_ADDR && type != ALE_TYPE_VLAN_ADDR)
			continue;
		if (cpsw_ale_get_vlan_id(ale_entry) != vid)
			continue;
		cpsw_ale_get_addr(ale_entry, entry_addr);
		if (memcmp(entry_addr, addr, 6) == 0)
			return idx;
	}
	return -ENOENT;
}

int cpsw_ale_match_vlan(struct cpsw_ale *ale, u16 vid)
{
	u32 ale_entry[ALE_ENTRY_WORDS];
	int type, idx;

	for (idx = 0; idx < ale->params.ale_entries; idx++) {
		cpsw_ale_read(ale, idx, ale_entry);
		type = cpsw_ale_get_entry_type(ale_entry);
		if (type != ALE_TYPE_VLAN)
			continue;
		if (cpsw_ale_get_vlan_id(ale_entry) == vid)
			return idx;
	}
	return -ENOENT;
}

static int cpsw_ale_match_free(struct cpsw_ale *ale)
{
	u32 ale_entry[ALE_ENTRY_WORDS];
	int type, idx;

	for (idx = 0; idx < ale->params.ale_entries; idx++) {
		cpsw_ale_read(ale, idx, ale_entry);
		type = cpsw_ale_get_entry_type(ale_entry);
		if (type == ALE_TYPE_FREE)
			return idx;
	}
	return -ENOENT;
}

static int cpsw_ale_find_ageable(struct cpsw_ale *ale)
{
	u32 ale_entry[ALE_ENTRY_WORDS];
	int type, idx;

	for (idx = 0; idx < ale->params.ale_entries; idx++) {
		cpsw_ale_read(ale, idx, ale_entry);
		type = cpsw_ale_get_entry_type(ale_entry);
		if (type != ALE_TYPE_ADDR && type != ALE_TYPE_VLAN_ADDR)
			continue;
		if (cpsw_ale_get_mcast(ale_entry))
			continue;
		type = cpsw_ale_get_ucast_type(ale_entry);
		if (type != ALE_UCAST_PERSISTANT &&
		    type != ALE_UCAST_OUI)
			return idx;
	}
	return -ENOENT;
}

static void cpsw_ale_flush_mcast(struct cpsw_ale *ale, u32 *ale_entry,
				 int port_mask)
{
	int mask;

	mask = cpsw_ale_get_port_mask(ale_entry);
	if ((mask & port_mask) == 0)
		return; /* ports dont intersect, not interested */
	mask &= ~port_mask;

	/* free if only remaining port is host port */
	if (mask)
		cpsw_ale_set_port_mask(ale_entry, mask);
	else
		cpsw_ale_set_entry_type(ale_entry, ALE_TYPE_FREE);
}

int cpsw_ale_flush_multicast(struct cpsw_ale *ale, int port_mask)
{
	u32 ale_entry[ALE_ENTRY_WORDS];
	int ret, idx;

	for (idx = 0; idx < ale->params.ale_entries; idx++) {
		cpsw_ale_read(ale, idx, ale_entry);
		ret = cpsw_ale_get_entry_type(ale_entry);
		if (ret != ALE_TYPE_ADDR && ret != ALE_TYPE_VLAN_ADDR)
			continue;

		if (cpsw_ale_get_mcast(ale_entry)) {
			u8 addr[6];

			cpsw_ale_get_addr(ale_entry, addr);
			if (!is_broadcast_ether_addr(addr))
				cpsw_ale_flush_mcast(ale, ale_entry, port_mask);
		}

		cpsw_ale_write(ale, idx, ale_entry);
	}
	return 0;
}

static void cpsw_ale_flush_ucast(struct cpsw_ale *ale, u32 *ale_entry,
				 int port_mask)
{
	int port;

	port = cpsw_ale_get_port_num(ale_entry);
	if ((BIT(port) & port_mask) == 0)
		return; /* ports dont intersect, not interested */
	cpsw_ale_set_entry_type(ale_entry, ALE_TYPE_FREE);
}

int cpsw_ale_flush(struct cpsw_ale *ale, int port_mask)
{
	u32 ale_entry[ALE_ENTRY_WORDS];
	int ret, idx;

	for (idx = 0; idx < ale->params.ale_entries; idx++) {
		cpsw_ale_read(ale, idx, ale_entry);
		ret = cpsw_ale_get_entry_type(ale_entry);
		if (ret != ALE_TYPE_ADDR && ret != ALE_TYPE_VLAN_ADDR)
			continue;

		if (cpsw_ale_get_mcast(ale_entry))
			cpsw_ale_flush_mcast(ale, ale_entry, port_mask);
		else
			cpsw_ale_flush_ucast(ale, ale_entry, port_mask);

		cpsw_ale_write(ale, idx, ale_entry);
	}
	return 0;
}

static inline void cpsw_ale_set_vlan_entry_type(u32 *ale_entry,
						int flags, u16 vid)
{
	if (flags & ALE_VLAN) {
		cpsw_ale_set_entry_type(ale_entry, ALE_TYPE_VLAN_ADDR);
		cpsw_ale_set_vlan_id(ale_entry, vid);
	} else {
		cpsw_ale_set_entry_type(ale_entry, ALE_TYPE_ADDR);
	}
}

int cpsw_ale_add_ucast(struct cpsw_ale *ale, u8 *addr, int port,
		       int flags, u16 vid)
{
	u32 ale_entry[ALE_ENTRY_WORDS] = {0, 0, 0};
	int idx;

	cpsw_ale_set_vlan_entry_type(ale_entry, flags, vid);

	cpsw_ale_set_addr(ale_entry, addr);
	cpsw_ale_set_ucast_type(ale_entry, ALE_UCAST_PERSISTANT);
	cpsw_ale_set_secure(ale_entry, (flags & ALE_SECURE) ? 1 : 0);
	cpsw_ale_set_blocked(ale_entry, (flags & ALE_BLOCKED) ? 1 : 0);
	cpsw_ale_set_port_num(ale_entry, port);

	idx = cpsw_ale_match_addr(ale, addr, (flags & ALE_VLAN) ? vid : 0);
	if (idx < 0)
		idx = cpsw_ale_match_free(ale);
	if (idx < 0)
		idx = cpsw_ale_find_ageable(ale);
	if (idx < 0)
		return -ENOMEM;

	cpsw_ale_write(ale, idx, ale_entry);
	return 0;
}

int cpsw_ale_del_ucast(struct cpsw_ale *ale, u8 *addr, int port,
		       int flags, u16 vid)
{
	u32 ale_entry[ALE_ENTRY_WORDS] = {0, 0, 0};
	int idx;

	idx = cpsw_ale_match_addr(ale, addr, (flags & ALE_VLAN) ? vid : 0);
	if (idx < 0)
		return -ENOENT;

	cpsw_ale_set_entry_type(ale_entry, ALE_TYPE_FREE);
	cpsw_ale_write(ale, idx, ale_entry);
	return 0;
}

int cpsw_ale_add_mcast(struct cpsw_ale *ale, u8 *addr, int port_mask,
		       int flags, u16 vid, int mcast_state)
{
	u32 ale_entry[ALE_ENTRY_WORDS] = {0, 0, 0};
	int idx, mask;

	idx = cpsw_ale_match_addr(ale, addr, (flags & ALE_VLAN) ? vid : 0);
	if (idx >= 0)
		cpsw_ale_read(ale, idx, ale_entry);

	cpsw_ale_set_vlan_entry_type(ale_entry, flags, vid);

	cpsw_ale_set_addr(ale_entry, addr);
	cpsw_ale_set_super(ale_entry, (flags & ALE_BLOCKED) ? 1 : 0);
	cpsw_ale_set_mcast_state(ale_entry, mcast_state);

	mask = cpsw_ale_get_port_mask(ale_entry);
	port_mask |= mask;
	cpsw_ale_set_port_mask(ale_entry, port_mask);

	if (idx < 0)
		idx = cpsw_ale_match_free(ale);
	if (idx < 0)
		idx = cpsw_ale_find_ageable(ale);
	if (idx < 0)
		return -ENOMEM;

	cpsw_ale_write(ale, idx, ale_entry);
	return 0;
}

int cpsw_ale_del_mcast(struct cpsw_ale *ale, u8 *addr, int port_mask,
		       int flags, u16 vid)
{
	u32 ale_entry[ALE_ENTRY_WORDS] = {0, 0, 0};
	int idx;

	idx = cpsw_ale_match_addr(ale, addr, (flags & ALE_VLAN) ? vid : 0);
	if (idx < 0)
		return -EINVAL;

	cpsw_ale_read(ale, idx, ale_entry);

	if (port_mask)
		cpsw_ale_set_port_mask(ale_entry, port_mask);
	else
		cpsw_ale_set_entry_type(ale_entry, ALE_TYPE_FREE);

	cpsw_ale_write(ale, idx, ale_entry);
	return 0;
}

int cpsw_ale_add_vlan(struct cpsw_ale *ale, u16 vid, int port, int untag,
		      int reg_mcast, int unreg_mcast)
{
	u32 ale_entry[ALE_ENTRY_WORDS] = {0, 0, 0};
	int idx;

	idx = cpsw_ale_match_vlan(ale, vid);
	if (idx >= 0)
		cpsw_ale_read(ale, idx, ale_entry);

	cpsw_ale_set_entry_type(ale_entry, ALE_TYPE_VLAN);
	cpsw_ale_set_vlan_id(ale_entry, vid);

	cpsw_ale_set_vlan_untag_force(ale_entry, untag);
	cpsw_ale_set_vlan_reg_mcast(ale_entry, reg_mcast);
	cpsw_ale_set_vlan_unreg_mcast(ale_entry, unreg_mcast);
	cpsw_ale_set_vlan_member_list(ale_entry, port);

	if (idx < 0)
		idx = cpsw_ale_match_free(ale);
	if (idx < 0)
		idx = cpsw_ale_find_ageable(ale);
	if (idx < 0)
		return -ENOMEM;

	cpsw_ale_write(ale, idx, ale_entry);
	return 0;
}

int cpsw_ale_del_vlan(struct cpsw_ale *ale, u16 vid, int port_mask)
{
	u32 ale_entry[ALE_ENTRY_WORDS] = {0, 0, 0};
	int idx;

	idx = cpsw_ale_match_vlan(ale, vid);
	if (idx < 0)
		return -ENOENT;

	cpsw_ale_read(ale, idx, ale_entry);

	if (port_mask)
		cpsw_ale_set_vlan_member_list(ale_entry, port_mask);
	else
		cpsw_ale_set_entry_type(ale_entry, ALE_TYPE_FREE);

	cpsw_ale_write(ale, idx, ale_entry);
	return 0;
}

struct ale_control_info {
	const char	*name;
	int		offset, port_offset;
	int		shift, port_shift;
	int		bits;
};

static const struct ale_control_info ale_controls[ALE_NUM_CONTROLS] = {
	[ALE_VERSION]		= {
		.name		= "version",
		.offset		= ALE_IDVER,
		.port_offset	= 0,
		.shift		= 0,
		.port_shift	= 0,
		.bits		= 32,
	},
	[ALE_ENABLE]		= {
		.name		= "enable",
		.offset		= ALE_CONTROL,
		.port_offset	= 0,
		.shift		= 31,
		.port_shift	= 0,
		.bits		= 1,
	},
	[ALE_CLEAR]		= {
		.name		= "clear",
		.offset		= ALE_CONTROL,
		.port_offset	= 0,
		.shift		= 30,
		.port_shift	= 0,
		.bits		= 1,
	},
	[ALE_AGEOUT]		= {
		.name		= "ageout",
		.offset		= ALE_CONTROL,
		.port_offset	= 0,
		.shift		= 29,
		.port_shift	= 0,
		.bits		= 1,
	},
	[ALE_UNI_FLOOD]		= {
		.name		= "p0_uni_flood_en",
		.offset		= ALE_CONTROL,
		.port_offset	= 0,
		.shift		= 8,
		.port_shift	= 0,
		.bits		= 1,
	},
	[ALE_VLAN_NOLEARN]	= {
		.name		= "vlan_nolearn",
		.offset		= ALE_CONTROL,
		.port_offset	= 0,
		.shift		= 7,
		.port_shift	= 0,
		.bits		= 1,
	},
	[ALE_NO_PORT_VLAN]	= {
		.name		= "no_port_vlan",
		.offset		= ALE_CONTROL,
		.port_offset	= 0,
		.shift		= 6,
		.port_shift	= 0,
		.bits		= 1,
	},
	[ALE_OUI_DENY]		= {
		.name		= "oui_deny",
		.offset		= ALE_CONTROL,
		.port_offset	= 0,
		.shift		= 5,
		.port_shift	= 0,
		.bits		= 1,
	},
	[ALE_BYPASS]		= {
		.name		= "bypass",
		.offset		= ALE_CONTROL,
		.port_offset	= 0,
		.shift		= 4,
		.port_shift	= 0,
		.bits		= 1,
	},
	[ALE_RATE_LIMIT_TX]	= {
		.name		= "rate_limit_tx",
		.offset		= ALE_CONTROL,
		.port_offset	= 0,
		.shift		= 3,
		.port_shift	= 0,
		.bits		= 1,
	},
	[ALE_VLAN_AWARE]	= {
		.name		= "vlan_aware",
		.offset		= ALE_CONTROL,
		.port_offset	= 0,
		.shift		= 2,
		.port_shift	= 0,
		.bits		= 1,
	},
	[ALE_AUTH_ENABLE]	= {
		.name		= "auth_enable",
		.offset		= ALE_CONTROL,
		.port_offset	= 0,
		.shift		= 1,
		.port_shift	= 0,
		.bits		= 1,
	},
	[ALE_RATE_LIMIT]	= {
		.name		= "rate_limit",
		.offset		= ALE_CONTROL,
		.port_offset	= 0,
		.shift		= 0,
		.port_shift	= 0,
		.bits		= 1,
	},
	[ALE_PORT_STATE]	= {
		.name		= "port_state",
		.offset		= ALE_PORTCTL,
		.port_offset	= 4,
		.shift		= 0,
		.port_shift	= 0,
		.bits		= 2,
	},
	[ALE_PORT_DROP_UNTAGGED] = {
		.name		= "drop_untagged",
		.offset		= ALE_PORTCTL,
		.port_offset	= 4,
		.shift		= 2,
		.port_shift	= 0,
		.bits		= 1,
	},
	[ALE_PORT_DROP_UNKNOWN_VLAN] = {
		.name		= "drop_unknown",
		.offset		= ALE_PORTCTL,
		.port_offset	= 4,
		.shift		= 3,
		.port_shift	= 0,
		.bits		= 1,
	},
	[ALE_PORT_NOLEARN]	= {
		.name		= "nolearn",
		.offset		= ALE_PORTCTL,
		.port_offset	= 4,
		.shift		= 4,
		.port_shift	= 0,
		.bits		= 1,
	},
	[ALE_PORT_MCAST_LIMIT]	= {
		.name		= "mcast_limit",
		.offset		= ALE_PORTCTL,
		.port_offset	= 4,
		.shift		= 16,
		.port_shift	= 0,
		.bits		= 8,
	},
	[ALE_PORT_BCAST_LIMIT]	= {
		.name		= "bcast_limit",
		.offset		= ALE_PORTCTL,
		.port_offset	= 4,
		.shift		= 24,
		.port_shift	= 0,
		.bits		= 8,
	},
	[ALE_PORT_UNKNOWN_VLAN_MEMBER] = {
		.name		= "unknown_vlan_member",
		.offset		= ALE_UNKNOWNVLAN,
		.port_offset	= 0,
		.shift		= 0,
		.port_shift	= 0,
		.bits		= 6,
	},
	[ALE_PORT_UNKNOWN_MCAST_FLOOD] = {
		.name		= "unknown_mcast_flood",
		.offset		= ALE_UNKNOWNVLAN,
		.port_offset	= 0,
		.shift		= 8,
		.port_shift	= 0,
		.bits		= 6,
	},
	[ALE_PORT_UNKNOWN_REG_MCAST_FLOOD] = {
		.name		= "unknown_reg_flood",
		.offset		= ALE_UNKNOWNVLAN,
		.port_offset	= 0,
		.shift		= 16,
		.port_shift	= 0,
		.bits		= 6,
	},
	[ALE_PORT_UNTAGGED_EGRESS] = {
		.name		= "unknown_force_untag_egress",
		.offset		= ALE_UNKNOWNVLAN,
		.port_offset	= 0,
		.shift		= 24,
		.port_shift	= 0,
		.bits		= 6,
	},
};

int cpsw_ale_control_set(struct cpsw_ale *ale, int port, int control,
			 int value)
{
	const struct ale_control_info *info;
	int offset, shift;
	u32 tmp, mask;

	if (control < 0 || control >= ARRAY_SIZE(ale_controls))
		return -EINVAL;

	info = &ale_controls[control];
	if (info->port_offset == 0 && info->port_shift == 0)
		port = 0; /* global, port is a dont care */

	if (port < 0 || port > ale->params.ale_ports)
		return -EINVAL;

	mask = BITMASK(info->bits);
	if (value & ~mask)
		return -EINVAL;

	offset = info->offset + (port * info->port_offset);
	shift  = info->shift  + (port * info->port_shift);

	tmp = __raw_readl(ale->params.ale_regs + offset);
	tmp = (tmp & ~(mask << shift)) | (value << shift);
	__raw_writel(tmp, ale->params.ale_regs + offset);

	return 0;
}

int cpsw_ale_control_get(struct cpsw_ale *ale, int port, int control)
{
	const struct ale_control_info *info;
	int offset, shift;
	u32 tmp;

	if (control < 0 || control >= ARRAY_SIZE(ale_controls))
		return -EINVAL;

	info = &ale_controls[control];
	if (info->port_offset == 0 && info->port_shift == 0)
		port = 0; /* global, port is a dont care */

	if (port < 0 || port > ale->params.ale_ports)
		return -EINVAL;

	offset = info->offset + (port * info->port_offset);
	shift  = info->shift  + (port * info->port_shift);

	tmp = __raw_readl(ale->params.ale_regs + offset) >> shift;
	return tmp & BITMASK(info->bits);
}

static int cpsw_ale_dump_mcast(u32 *ale_entry, char *buf, int len)
{
	int outlen = 0;
	static const char * const str_mcast_state[] = {"f", "blf", "lf", "f"};
	int mcast_state = cpsw_ale_get_mcast_state(ale_entry);
	int port_mask   = cpsw_ale_get_port_mask(ale_entry);
	int super       = cpsw_ale_get_super(ale_entry);

	outlen += snprintf(buf + outlen, len - outlen,
			   "mcstate: %s(%d), ", str_mcast_state[mcast_state],
			   mcast_state);
	outlen += snprintf(buf + outlen, len - outlen,
			   "port mask: %x, %ssuper\n", port_mask,
			   super ? "" : "no ");
	return outlen;
}

static int cpsw_ale_dump_ucast(u32 *ale_entry, char *buf, int len)
{
	int outlen = 0;
	static const char * const str_ucast_type[] = {"persistant", "untouched",
							"oui", "touched"};
	int ucast_type  = cpsw_ale_get_ucast_type(ale_entry);
	int port_num    = cpsw_ale_get_port_num(ale_entry);
	int secure      = cpsw_ale_get_secure(ale_entry);
	int blocked     = cpsw_ale_get_blocked(ale_entry);

	outlen += snprintf(buf + outlen, len - outlen,
			   "uctype: %s(%d)", str_ucast_type[ucast_type],
			   ucast_type);
	if (ucast_type == ALE_UCAST_OUI)
		outlen += snprintf(buf + outlen, len - outlen, "\n");
	else
		outlen += snprintf(buf + outlen, len - outlen,
				", port: %d%s%s\n", port_num,
				secure ? ", Secure" : "",
				blocked ? ", Blocked" : "");
	return outlen;
}

static int cpsw_ale_dump_vlan(u32 *ale_entry, char *buf, int len)
{
	int outlen = 0;
	int force_utag_egress	= cpsw_ale_get_vlan_untag_force(ale_entry);
	int reg_mc_fld		= cpsw_ale_get_vlan_reg_mcast(ale_entry);
	int unreg_mc_fld	= cpsw_ale_get_vlan_unreg_mcast(ale_entry);
	int mem_list		= cpsw_ale_get_vlan_member_list(ale_entry);

	outlen += snprintf(buf + outlen, len - outlen,
			   "force_untag_egress: %02x, ", force_utag_egress);
	outlen += snprintf(buf + outlen, len - outlen,
			   "reg_fld: %02x, ", reg_mc_fld);
	outlen += snprintf(buf + outlen, len - outlen,
			   "unreg_fld: %02x, ", unreg_mc_fld);
	outlen += snprintf(buf + outlen, len - outlen,
			   "mem_list: %02x\n", mem_list);
	return outlen;
}

static int cpsw_ale_dump_entry(int idx, u32 *ale_entry, char *buf, int len)
{
	int type, outlen = 0;
	u8 addr[6];
	static const char * const str_type[] = {"free", "addr",
						"vlan", "vlan+addr"};

	type = cpsw_ale_get_entry_type(ale_entry);
	if (type == ALE_TYPE_FREE)
		return outlen;

	if (len < ALE_TBL_ENTRY_SHOW_LEN)
		return outlen;

	if (idx >= 0) {
		outlen += snprintf(buf + outlen, len - outlen,
				   "index %d, ", idx);
	}

	outlen += snprintf(buf + outlen, len - outlen, "raw: %08x %08x %08x, ",
			   ale_entry[0], ale_entry[1], ale_entry[2]);

	outlen += snprintf(buf + outlen, len - outlen,
			   "type: %s(%d), ", str_type[type], type);

	if (type != ALE_TYPE_VLAN) {
		cpsw_ale_get_addr(ale_entry, addr);
		outlen += snprintf(buf + outlen, len - outlen,
			   "addr: " ADDR_FMT_STR ", ", ADDR_FMT_ARGS(addr));
	}

	if (type == ALE_TYPE_VLAN || type == ALE_TYPE_VLAN_ADDR) {
		outlen += snprintf(buf + outlen, len - outlen, "vlan: %d, ",
				   cpsw_ale_get_vlan_id(ale_entry));
	}

	if (type == ALE_TYPE_VLAN)
		outlen += cpsw_ale_dump_vlan(ale_entry,
				buf + outlen, len - outlen);
	else
		outlen += cpsw_ale_get_mcast(ale_entry) ?
		  cpsw_ale_dump_mcast(ale_entry, buf + outlen, len - outlen) :
		  cpsw_ale_dump_ucast(ale_entry, buf + outlen, len - outlen);

	return outlen;
}

static ssize_t cpsw_ale_control_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int i, port, len = 0;
	const struct ale_control_info *info;
	struct cpsw_ale *ale = control_attr_to_ale(attr);
	u32 reg;

	for (i = 0, info = ale_controls; i < ALE_NUM_CONTROLS; i++, info++) {
		if (i == ALE_VERSION) {
			reg = cpsw_ale_control_get(ale, 0, i);
			len += snprintf(buf + len, SZ_4K - len,
					"%s=(ALE_ID=0x%04x) Rev %d.%d\n",
					info->name,
					(reg & 0xffff0000) >> 16,
					ALE_VERSION_MAJOR(reg),
					ALE_VERSION_MINOR(reg));
			continue;
		}

		/* global controls */
		if (info->port_shift == 0 &&  info->port_offset == 0) {
			len += snprintf(buf + len, SZ_4K - len,
					"%s=%d\n", info->name,
					cpsw_ale_control_get(ale, 0, i));
			continue;
		}

		/* port specific controls */
		for (port = 0; port < ale->params.ale_ports; port++) {
			len += snprintf(buf + len, SZ_4K - len,
					"%s.%d=%d\n", info->name, port,
					cpsw_ale_control_get(ale, port, i));
		}
	}

	return len;
}

static ssize_t cpsw_ale_control_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	char ctrl_str[33], tmp_str[9];
	int port = 0, value, len, ret, control;
	unsigned long end;
	struct cpsw_ale *ale = control_attr_to_ale(attr);

	len = strcspn(buf, ".=");
	if (len >= 32)
		return -ENOMEM;

	strncpy(ctrl_str, buf, len);
	ctrl_str[len] = '\0';
	buf += len;

	if (*buf == '.') {
		++buf;
		len = strcspn(buf, "=");
		if (len >= 8)
			return -ENOMEM;
		strncpy(tmp_str, buf, len);
		tmp_str[len] = '\0';
		if (kstrtoul(tmp_str, 0, &end))
			return -EINVAL;
		port = (int)end;
		buf += len;
	}

	if (*buf != '=')
		return -EINVAL;

	if (kstrtoul(buf + 1, 0, &end))
		return -EINVAL;

	value = (int)end;

	for (control = 0; control < ALE_NUM_CONTROLS; control++)
		if (strcmp(ctrl_str, ale_controls[control].name) == 0)
			break;

	if (control >= ALE_NUM_CONTROLS)
		return -ENOENT;

	dev_dbg(ale->params.dev, "processing command %s.%d=%d\n",
		ale_controls[control].name, port, value);

	ret = cpsw_ale_control_set(ale, port, control, value);
	if (ret < 0)
		return ret;

	return count;
}
DEVICE_ATTR(ale_control, S_IRUGO | S_IWUSR,
	cpsw_ale_control_show, cpsw_ale_control_store);

static ssize_t cpsw_ale_table_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	int len = SZ_4K, outlen = 0, idx, start;
	u32 ale_entry[ALE_ENTRY_WORDS];
	struct cpsw_ale *ale = table_attr_to_ale(attr);
	int not_shown = 0, total_outlen = 0, type, shown = 0;

	start = ale->show_next;

	for (idx = start; (idx < ale->params.ale_entries) &&
			(len > total_outlen); idx++) {
		cpsw_ale_read(ale, idx, ale_entry);
		outlen = cpsw_ale_dump_entry(idx, ale_entry,
				buf + total_outlen, len - total_outlen);
		if (outlen == 0) {
			type = cpsw_ale_get_entry_type(ale_entry);
			if (type != ALE_TYPE_FREE) {
				++not_shown;
				break;
			}
		} else {
			total_outlen += outlen;
			++shown;
		}
	}

	/* update next show index */
	if (idx >= ale->params.ale_entries)
		ale->show_next = 0;
	else
		ale->show_next = idx;

	if (len > total_outlen + 32)
		total_outlen += snprintf(buf + total_outlen, len - total_outlen,
				"[%d..%d]: %d entries%s\n", start, idx - 1,
				shown, not_shown ? ", +" : "");

	return total_outlen;
}

struct ale_table_param {
	const char *name;
	union	{
		int	val;
		u8	addr[6];
	};
};

struct ale_table_cmd {
	const char *name;
	int (*process)(struct cpsw_ale *ale,
		const u8 *params_str, size_t str_len);
};

static struct ale_table_param vlan_params[] = {
	[ALE_VP_VID]		= { .name = "vid", },
	[ALE_VP_FORCE_UT_EGR]	= { .name = "force_untag_egress", },
	[ALE_VP_REG_FLD]	= { .name = "reg_fld_mask", },
	[ALE_VP_UNREG_FLD]	= { .name = "unreg_fld_mask", },
	[ALE_VP_M_LIST]		= { .name = "mem_list", },
};

static struct ale_table_param vlan_ucast_params[] = {
	[ALE_UP_PORT]		= { .name = "port", },
	[ALE_UP_BLOCK]		= { .name = "block", },
	[ALE_UP_SECURE]		= { .name = "secure", },
	[ALE_UP_AGEABLE]	= { .name = "ageable", },
	[ALE_UP_ADDR]		= { .name = "addr", },
	[ALE_UP_VID]		= { .name = "vid", },
};

static struct ale_table_param vlan_mcast_params[] = {
	[ALE_MP_PORT_MASK]	= { .name = "port_mask", },
	[ALE_MP_SUPER]		= { .name = "supervisory", },
	[ALE_MP_FW_ST]		= { .name = "mc_fw_st", },
	[ALE_MP_ADDR]		= { .name = "addr", },
	[ALE_MP_VID]		= { .name = "vid", },
};

static struct ale_table_param oui_params[] = {
	{ .name	= "addr", },
};

void cpsw_ale_table_store_init_params(
	struct ale_table_param *params, int param_num)
{
	int i;

	for (i = 0; i < param_num; i++)
		memset(params[i].addr, 0, 6);
}

int cpsw_ale_table_store_get_params(struct cpsw_ale *ale,
	struct ale_table_param *params, int param_num,
	const u8 *params_str, size_t str_len)
{
	char param_name[33], val_str[33];
	size_t tmp_len = str_len;
	unsigned int iaddr[6];
	unsigned long end;
	int len, i, n, addr_len;

	while (tmp_len > 0) {
		len = strcspn(params_str, "=");
		if (len >= 32)
			return -ENOMEM;

		strncpy(param_name, params_str, len);
		param_name[len] = '\0';
		params_str += len;
		tmp_len -= len;

		if (*params_str != '=')
			return -EINVAL;

		++params_str;
		--tmp_len;

		len = strcspn(params_str, ".");
		if (len >= 32)
			return -ENOMEM;

		strncpy(val_str, params_str, len);
		val_str[len] = '\0';
		params_str += len;
		tmp_len -= len;

		if (*params_str == '.') {
			++params_str;
			--tmp_len;
		}

		for (n = 0; n < param_num; n++) {
			if (strcmp(param_name, params[n].name) != 0)
				continue;

			if (strcmp(param_name, "addr") == 0) {
				addr_len = sscanf(val_str,
					"%02x:%02x:%02x:%02x:%02x:%02x",
					&iaddr[0], &iaddr[1], &iaddr[2],
					&iaddr[3], &iaddr[4], &iaddr[5]);
				if (addr_len != 6 && addr_len != 3)
					return -EINVAL;

				for (i = 0; i < addr_len; i++)
					params[n].addr[i] = iaddr[i];

				break;
			}

			if (kstrtoul(val_str, 0, &end))
				return -EINVAL;

			params[n].val = (int)end;
			break;
		}

		if (n >= param_num)
			return -EINVAL;
	}

	return str_len;
}

int cpsw_ale_table_store_vlan(struct cpsw_ale *ale,
			const u8 *params_str, size_t str_len)
{
	int ret;

	cpsw_ale_table_store_init_params(vlan_params, ALE_VP_NUM);
	vlan_params[ALE_VP_VID].val = -1;

	ret = cpsw_ale_table_store_get_params(ale,
		vlan_params, ALE_VP_NUM, params_str, str_len);

	if (ret < 0)
		return ret;

	ret = cpsw_ale_add_vlan(ale,
		vlan_params[ALE_VP_VID].val,
		vlan_params[ALE_VP_M_LIST].val,
		vlan_params[ALE_VP_FORCE_UT_EGR].val,
		vlan_params[ALE_VP_REG_FLD].val,
		vlan_params[ALE_VP_UNREG_FLD].val);

	if (ret < 0)
		return ret;
	else
		return str_len;
}

int cpsw_ale_table_store_vlan_ucast(struct cpsw_ale *ale,
		const u8 *params_str, size_t str_len, int has_vid)
{
	int ret, flags = 0;

	cpsw_ale_table_store_init_params(vlan_ucast_params, ALE_UP_NUM);
	vlan_ucast_params[ALE_UP_VID].val = -1;

	ret = cpsw_ale_table_store_get_params(ale,
		vlan_ucast_params, ALE_UP_NUM,
		params_str, str_len);

	if (ret < 0)
		return ret;

	if (!has_vid && vlan_ucast_params[ALE_UP_VID].val >= 0)
		return -EINVAL;

	if (vlan_ucast_params[ALE_UP_BLOCK].val)
		flags |= ALE_BLOCKED;

	if (vlan_ucast_params[ALE_UP_SECURE].val)
		flags |= ALE_SECURE;

	cpsw_ale_add_ucast(ale,
		vlan_ucast_params[ALE_UP_ADDR].addr,
		vlan_ucast_params[ALE_UP_PORT].val,
		flags,
		vlan_ucast_params[ALE_UP_VID].val);

	return str_len;
}

int cpsw_ale_table_store_u_proc(struct cpsw_ale *ale,
		const u8 *params_str, size_t str_len)
{
	return  cpsw_ale_table_store_vlan_ucast(ale, params_str, str_len, 0);
}

int cpsw_ale_table_store_vu_proc(struct cpsw_ale *ale,
		const u8 *params_str, size_t str_len)
{
	return  cpsw_ale_table_store_vlan_ucast(ale, params_str, str_len, 1);
}

int cpsw_ale_table_store_vlan_mcast(struct cpsw_ale *ale,
		const u8 *params_str, size_t str_len, int has_vid)
{
	int ret;

	cpsw_ale_table_store_init_params(vlan_mcast_params, ALE_MP_NUM);
	vlan_mcast_params[ALE_MP_VID].val = -1;

	ret = cpsw_ale_table_store_get_params(ale,
		vlan_mcast_params, ALE_MP_NUM,
		params_str, str_len);

	if (ret < 0)
		return ret;

	if (!has_vid && vlan_mcast_params[ALE_MP_VID].val >= 0)
		return -EINVAL;

	cpsw_ale_add_mcast(ale,
		vlan_mcast_params[ALE_MP_ADDR].addr,
		vlan_mcast_params[ALE_MP_PORT_MASK].val,
		vlan_mcast_params[ALE_MP_SUPER].val,
		vlan_mcast_params[ALE_MP_FW_ST].val,
		vlan_mcast_params[ALE_MP_VID].val);

	return str_len;
}

int cpsw_ale_table_store_m_proc(struct cpsw_ale *ale,
			const u8 *params_str, size_t str_len)
{
	return  cpsw_ale_table_store_vlan_mcast(ale, params_str, str_len, 0);
}

int cpsw_ale_table_store_vm_proc(struct cpsw_ale *ale,
			const u8 *params_str, size_t str_len)
{
	return  cpsw_ale_table_store_vlan_mcast(ale, params_str, str_len, 1);
}

int cpsw_ale_add_oui(struct cpsw_ale *ale, u8 *addr)
{
	u32 ale_entry[ALE_ENTRY_WORDS] = {0, 0, 0};
	int idx;

	cpsw_ale_set_entry_type(ale_entry, ALE_TYPE_ADDR);

	cpsw_ale_set_addr(ale_entry, addr);
	cpsw_ale_set_ucast_type(ale_entry, ALE_UCAST_OUI);

	idx = cpsw_ale_match_addr(ale, addr, -1);
	if (idx < 0)
		idx = cpsw_ale_match_free(ale);
	if (idx < 0)
		idx = cpsw_ale_find_ageable(ale);
	if (idx < 0)
		return -ENOMEM;

	cpsw_ale_write(ale, idx, ale_entry);
	return 0;
}

int cpsw_ale_table_store_oui(struct cpsw_ale *ale,
			const u8 *params_str, size_t str_len)
{
	int ret;

	cpsw_ale_table_store_init_params(oui_params, 1);

	ret = cpsw_ale_table_store_get_params(ale,
		oui_params, 1, params_str, str_len);

	if (ret < 0)
		return ret;

	/* Clear out the don't cares */
	oui_params[0].addr[3] = 0;
	oui_params[0].addr[4] = 0;
	oui_params[0].addr[5] = 0;

	cpsw_ale_add_oui(ale, oui_params[0].addr);

	return str_len;
}

int cpsw_ale_table_store_del(struct cpsw_ale *ale, int idx)
{
	u32 ale_entry[ALE_ENTRY_WORDS];
	int type;

	dev_dbg(ale->params.dev, "deleting entry[%d] ...\n", idx);

	if (idx >= ale->params.ale_entries)
		return -EINVAL;

	cpsw_ale_read(ale, idx, ale_entry);

	type = cpsw_ale_get_entry_type(ale_entry);
	if (type == ALE_TYPE_FREE)
		return -EINVAL;

	cpsw_ale_set_entry_type(ale_entry, ALE_TYPE_FREE);
	cpsw_ale_write(ale, idx, ale_entry);
	return 0;
}

static struct ale_table_cmd ale_table_cmds[] = {
	{
		.name		= "v",
		.process	= cpsw_ale_table_store_vlan,
	},
	{
		.name		= "m",
		.process	= cpsw_ale_table_store_m_proc,
	},
	{
		.name		= "vm",
		.process	= cpsw_ale_table_store_vm_proc,
	},
	{
		.name		= "u",
		.process	= cpsw_ale_table_store_u_proc,
	},
	{
		.name		= "vu",
		.process	= cpsw_ale_table_store_vu_proc,
	},
	{
		.name		= "o",
		.process	= cpsw_ale_table_store_oui,
	},
};

static ssize_t cpsw_ale_table_store_proc(struct cpsw_ale *ale,
				const char *buf, size_t count)
{
	char ctrl_str[33];
	unsigned long end;
	int len, i, tmp_count = count, ret = -EINVAL;

	len = strcspn(buf, ".:");
	if (len >= 5)
		return -ENOMEM;

	strncpy(ctrl_str, buf, len);
	ctrl_str[len] = '\0';

	/* skip to param beginning */
	buf += len;
	tmp_count -= len;

	if (*buf == ':') {
		/* delete cmd */
		if (kstrtoul(ctrl_str, 0, &end))
			return -EINVAL;
		ret = cpsw_ale_table_store_del(ale, end);
		if (ret != 0)
			return ret;
		else
			return count;
	}

	if (len >= 3)
		return -ENOMEM;

	if (*buf != '.')
		return -EINVAL;

	++buf;
	--tmp_count;

	for (i = 0; i < ARRAY_SIZE(ale_table_cmds); i++) {
		if (strcmp(ale_table_cmds[i].name, ctrl_str) == 0) {
			ret = ale_table_cmds[i].process(ale, buf, tmp_count);
			break;
		}
	}

	if (ret < 0)
		return ret;
	else
		return count;
}

static ssize_t cpsw_ale_table_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct cpsw_ale *ale = table_attr_to_ale(attr);

	return cpsw_ale_table_store_proc(ale, buf, count);
}
DEVICE_ATTR(ale_table, S_IRUGO | S_IWUSR,
	cpsw_ale_table_show, cpsw_ale_table_store);

static int cpsw_ale_dump_entry_raw(int idx, u32 *ale_entry, char *buf, int len)
{
	int type, outlen = 0;

	type = cpsw_ale_get_entry_type(ale_entry);
	if (type == ALE_TYPE_FREE)
		return outlen;

	if (len < ALE_RAW_TBL_ENTRY_SHOW_LEN)
		return outlen;

	if (idx >= 0)
		outlen += snprintf(buf + outlen, len - outlen,
				   "%d: ", idx);

	outlen += snprintf(buf + outlen, len - outlen, "%02x %08x %08x\n",
			   ale_entry[0], ale_entry[1], ale_entry[2]);

	return outlen;
}

static ssize_t cpsw_ale_table_raw_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct cpsw_ale *ale = table_raw_attr_to_ale(attr);
	int not_shown = 0, total_outlen = 0, shown = 0;
	int outlen = 0, idx, start, type;
	u32 ale_entry[ALE_ENTRY_WORDS];

	start = ale->raw_show_next;

	for (idx = start; (idx < ale->params.ale_entries) &&
				(PAGE_SIZE > total_outlen); idx++) {
		cpsw_ale_read(ale, idx, ale_entry);
		outlen = cpsw_ale_dump_entry_raw(idx, ale_entry,
					buf + total_outlen,
					PAGE_SIZE - total_outlen);
		if (outlen == 0) {
			type = cpsw_ale_get_entry_type(ale_entry);
			if (type != ALE_TYPE_FREE) {
				++not_shown;
				break;
			}
		} else {
			total_outlen += outlen;
			++shown;
		}
	}

	/* update next show index */
	if (idx >= ale->params.ale_entries)
		ale->raw_show_next = 0;
	else
		ale->raw_show_next = idx;

	if (PAGE_SIZE > total_outlen + 32)
		total_outlen += snprintf(buf + total_outlen,
			PAGE_SIZE - total_outlen,
			"[%d..%d]: %d entries%s\n",
			start, idx - 1, shown, not_shown ? ", +" : "");

	return total_outlen;
}

static ssize_t cpsw_ale_table_raw_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct cpsw_ale *ale = table_raw_attr_to_ale(attr);
	unsigned long end;

	if (kstrtoul(buf, 0, &end) == 0) {
		/* set start-show-index command */
		ale->raw_show_next = (int)end;
		if (ale->raw_show_next >= ale->params.ale_entries)
			ale->raw_show_next = 0;
		return count;
	}

	/* add or delete command */
	return cpsw_ale_table_store_proc(ale, buf, count);
}
DEVICE_ATTR(ale_table_raw, S_IRUGO | S_IWUSR,
	cpsw_ale_table_raw_show, cpsw_ale_table_raw_store);

static void cpsw_ale_timer(unsigned long arg)
{
	struct cpsw_ale *ale = (struct cpsw_ale *)arg;

	cpsw_ale_control_set(ale, 0, ALE_AGEOUT, 1);

	if (ale->ageout) {
		ale->timer.expires = jiffies + ale->ageout;
		add_timer(&ale->timer);
	}
}

int cpsw_ale_set_ageout(struct cpsw_ale *ale, int ageout)
{
	del_timer_sync(&ale->timer);
	ale->ageout = ageout * HZ;
	if (ale->ageout) {
		ale->timer.expires = jiffies + ale->ageout;
		add_timer(&ale->timer);
	}
	return 0;
}

void cpsw_ale_start(struct cpsw_ale *ale)
{
	u32 rev;
	int ret;

	rev = __raw_readl(ale->params.ale_regs + ALE_IDVER);
	dev_dbg(ale->params.dev, "initialized cpsw ale revision %d.%d\n",
		ALE_VERSION_MAJOR(rev), ALE_VERSION_MINOR(rev));
	cpsw_ale_control_set(ale, 0, ALE_ENABLE, 1);
	cpsw_ale_control_set(ale, 0, ALE_CLEAR, 1);

	ale->ale_control_attr = dev_attr_ale_control;
	ret = device_create_file(ale->params.dev, &ale->ale_control_attr);
	WARN_ON(ret < 0);

	ale->ale_table_attr = dev_attr_ale_table;
	ret = device_create_file(ale->params.dev, &ale->ale_table_attr);
	WARN_ON(ret < 0);

	ale->ale_table_raw_attr = dev_attr_ale_table_raw;
	ret = device_create_file(ale->params.dev, &ale->ale_table_raw_attr);
	WARN_ON(ret < 0);

	init_timer(&ale->timer);
	ale->timer.data	    = (unsigned long)ale;
	ale->timer.function = cpsw_ale_timer;
	if (ale->ageout) {
		ale->timer.expires = jiffies + ale->ageout;
		add_timer(&ale->timer);
	}
}

void cpsw_ale_stop(struct cpsw_ale *ale)
{
	del_timer_sync(&ale->timer);
	device_remove_file(ale->params.dev, &ale->ale_table_attr);
	device_remove_file(ale->params.dev, &ale->ale_control_attr);
	device_remove_file(ale->params.dev, &ale->ale_table_raw_attr);
}

struct cpsw_ale *cpsw_ale_create(struct cpsw_ale_params *params)
{
	struct cpsw_ale *ale;

	ale = kzalloc(sizeof(*ale), GFP_KERNEL);
	if (!ale)
		return NULL;

	ale->params = *params;
	ale->ageout = ale->params.ale_ageout * HZ;

	return ale;
}

int cpsw_ale_destroy(struct cpsw_ale *ale)
{
	if (!ale)
		return -EINVAL;
	cpsw_ale_stop(ale);
	cpsw_ale_control_set(ale, 0, ALE_ENABLE, 0);
	kfree(ale);
	return 0;
}
