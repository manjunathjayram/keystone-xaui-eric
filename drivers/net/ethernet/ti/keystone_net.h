/*
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
#ifndef __KEYSTONE_NECTP_H__
#define __KEYSTONE_NECTP_H__

#include <linux/skbuff.h>
#include <linux/if_vlan.h>
#include <linux/ethtool.h>
#include <linux/if_ether.h>
#include <linux/netdevice.h>
#include <linux/keystone-dma.h>
#include <linux/interrupt.h>

/* Maximum Ethernet frame size supported by Keystone switch */
#define NETCP_MAX_FRAME_SIZE	9504
#define NETCP_MAX_MCAST_ADDR	16

#define SGMII_LINK_MAC_MAC_AUTONEG	0
#define SGMII_LINK_MAC_PHY		1
#define SGMII_LINK_MAC_MAC_FORCED	2
#define SGMII_LINK_MAC_FIBER		3
#define SGMII_LINK_MAC_PHY_NO_MDIO	4
#define XGMII_LINK_MAC_PHY		10
#define XGMII_LINK_MAC_MAC_FORCED	11

int serdes_init(void);
void serdes_init_6638_156p25Mhz(void);
int keystone_sgmii_reset(void __iomem *sgmii_ofs, int port);
int keystone_sgmii_link_status(void __iomem *sgmii_ofs, int ports);
int keystone_sgmii_get_port_link(void __iomem *sgmii_ofs, int port);
int keystone_sgmii_config(void __iomem *sgmii_ofs,
			  int port, u32 interface);

struct netcp_device;

enum netcp_rx_state {
	RX_STATE_INTERRUPT,
	RX_STATE_SCHEDULED,
	RX_STATE_POLL,
	RX_STATE_TEARDOWN,
	RX_STATE_INVALID,
};

enum netcp_tx_state {
	TX_STATE_INVALID,
	TX_STATE_INTERRUPT,
	TX_STATE_SCHEDULED,
	TX_STATE_POLL,
};

struct netcp_tx_pipe {
	struct netcp_priv		*netcp_priv;
	struct dma_chan			*dma_channel;
	const char			*dma_chan_name;
	u8				 dma_psflags;
	u8				 filler1;
	u16				 dma_queue;
	unsigned int			 dma_queue_depth;
	unsigned int			 dma_pause_threshold;
	unsigned int			 dma_resume_threshold;
	atomic_t			 dma_poll_count;
	struct tasklet_struct		 dma_poll_tasklet;
	enum netcp_tx_state		 dma_poll_state;
};

#define ADDR_NEW	BIT(0)
#define ADDR_VALID	BIT(1)

enum netcp_addr_type {
	ADDR_ANY,
	ADDR_DEV,
	ADDR_UCAST,
	ADDR_MCAST,
	ADDR_BCAST
};

struct netcp_addr {
	struct netcp_priv	*netcp;
	unsigned char		 addr[MAX_ADDR_LEN];
	enum netcp_addr_type	 type;
	unsigned int		 flags;
	struct list_head	 node;
};

struct netcp_priv {
	spinlock_t			 lock;
	struct netcp_device		*netcp_device;
	struct platform_device		*pdev;
	struct net_device		*ndev;
	struct napi_struct		 napi;
	struct device			*dev;
	int				 cpsw_port;
	u32				 msg_enable;
	struct net_device_stats		 stats;
	int				 rx_packet_max;

	struct dma_chan			*rx_channel;
	const char			*rx_chan_name;

	u32				 link_state;

	enum netcp_rx_state		 rx_state;
	struct list_head		 module_head;
	struct list_head		 interface_list;
	struct list_head		 txhook_list_head;
	struct list_head		 rxhook_list_head;
	struct list_head		 addr_list;

	/* PktDMA configuration data */
	u32				 rx_queue_depths[KEYSTONE_QUEUES_PER_CHAN];
	u32				 rx_buffer_sizes[KEYSTONE_QUEUES_PER_CHAN];
};

#define NETCP_SGLIST_SIZE	(MAX_SKB_FRAGS + 2)
#define	NETCP_PSDATA_LEN	16
struct netcp_packet {
	struct scatterlist		 sg[NETCP_SGLIST_SIZE];
	int				 sg_ents;
	struct sk_buff			*skb;
	u32				 epib[4];
	u32				 psdata[NETCP_PSDATA_LEN];
	unsigned int			 psdata_len;
	struct netcp_priv		*netcp;
	dma_cookie_t			 cookie;
	struct netcp_tx_pipe		*tx_pipe;
};

static inline int netcp_prepend_psdata(struct netcp_packet *p_info, u32 *data, unsigned len)
{
	if ((len + p_info->psdata_len) > NETCP_PSDATA_LEN)
		return -ENOBUFS;
	p_info->psdata_len += len;

	memcpy(&p_info->psdata[NETCP_PSDATA_LEN - p_info->psdata_len],
			data, len * sizeof(u32));
	return 0;
}

struct netcp_module {
	const char		*name;
	struct module		*owner;
	struct list_head	 module_list;
	struct list_head	 interface_list;

	/* probe/remove: called once per NETCP instance */
	int			(*probe)(struct netcp_device *netcp_device,
					 struct device *device,
					 struct device_node *node,
					 void **inst_priv);
	int			(*remove)(struct netcp_device *netcp_device,
					  void *inst_priv);

	/* attach/release: called once per network interface */
	int			(*attach)(void *inst_priv, struct net_device *ndev,
					  void **intf_priv);
	int			(*release)(void *intf_priv);

	int			(*open)(void *intf_priv, struct net_device *ndev);
	int			(*close)(void *intf_priv, struct net_device *ndev);
	int			(*add_addr)(void *intf_priv,
					    struct netcp_addr *naddr);
	int			(*del_addr)(void *intf_priv,
					    struct netcp_addr *naddr);
	int			(*add_vid)(void *intf_priv, int vid);
	int			(*del_vid)(void *intf_priv, int vid);
	int			(*ioctl)(void *intf_priv, struct ifreq *req,
					 int cmd);
};

int netcp_register_module(struct netcp_module *module);
void netcp_unregister_module(struct netcp_module *module);

u32 netcp_get_streaming_switch(struct netcp_device *netcp_device, int port);
u32 netcp_set_streaming_switch(struct netcp_device *netcp_device,
				int port, u32 new_value);

int netcp_create_interface(struct netcp_device *netcp_device,
			   struct net_device **ndev_p,
			   const char *ifname_proto,
			   int tx_queues, int rx_queues,
			   int cpsw_port);
void netcp_delete_interface(struct netcp_device *netcp_device,
			    struct net_device *ndev);

int netcp_txpipe_init(struct netcp_tx_pipe *tx_pipe,
		struct netcp_priv *netcp_priv,
		const char *chan_name,
		int queue_depth);
int netcp_txpipe_open(struct netcp_tx_pipe *tx_pipe);
int netcp_txpipe_close(struct netcp_tx_pipe *tx_pipe);

struct dma_chan *netcp_get_rx_chan(struct netcp_priv *priv);
struct dma_chan *netcp_get_tx_chan(struct netcp_priv *priv);

u32 *netcp_push_psdata(struct netcp_packet *p_info, unsigned words);
int netcp_align_psdata(struct netcp_packet *p_info, unsigned word_align);

typedef int netcp_hook_rtn(int order, void *data, struct netcp_packet *packet);

int netcp_register_txhook(struct netcp_priv *netcp_priv, int order,
		netcp_hook_rtn *hook_rtn, void *hook_data);
int netcp_unregister_txhook(struct netcp_priv *netcp_priv, int order,
		netcp_hook_rtn *hook_rtn, void *hook_data);
int netcp_register_rxhook(struct netcp_priv *netcp_priv, int order,
		netcp_hook_rtn *hook_rtn, void *hook_data);
int netcp_unregister_rxhook(struct netcp_priv *netcp_priv, int order,
		netcp_hook_rtn *hook_rtn, void *hook_data);

void *netcp_device_find_module(struct netcp_device *netcp_device,
		const char *name);
void xge_serdes_init_156p25Mhz(void);
int keystone_pcsr_config(void __iomem *pcsr_ofs, int port, u32 interface);
#endif
