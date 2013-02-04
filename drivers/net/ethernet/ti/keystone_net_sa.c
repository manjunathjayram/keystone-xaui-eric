/*
 * Copyright (C) 2012 Texas Instruments Incorporated
 * Authors: Sandeep Nair <sandeep_n@ti.com>
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

#include <linux/io.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/interrupt.h>
#include <linux/byteorder/generic.h>
#include <linux/platform_device.h>
#include <linux/keystone-dma.h>
#include <linux/errqueue.h>

#include "keystone_net.h"
#include "keystone_pasahost.h"

struct sa_device {
	struct netcp_device		*netcp_device;
	struct net_device		*net_device;		/* FIXME */
	struct device			*dev;
	const char			*tx_chan_name;
	u32				 tx_queue_depth;
	struct netcp_tx_pipe		 tx_pipe;
	struct device_node		*node;
	u32				 multi_if;
};

struct ipsecmgr_mod_sa_ctx_info {
	u32 word0;
	u32 word1;
	u16 flow_id;
};

/**
 * Used to update the destination information within swInfo[2]
 */

#define SA_SWINFO_UPDATE_DEST_INFO(info, queueID, flowID) \
{ \
	(info[0]) |= 0x40000000L; \
	(info[2]) = ((queueID)) | (((flowID) & 0xFF) << 16) | \
	((info[2]) & 0xFF000000L); \
}

#define	SA_TXHOOK_ORDER	30

static int sa_tx_hook(int order, void *data, struct netcp_packet *p_info)
{
	struct sa_device *sa_dev = data;
	u16 offset, len, ihl;
	u32 *psdata, *swinfo;
	const struct iphdr *iph;
	struct ipsecmgr_mod_sa_ctx_info *ctx_info =
			(struct ipsecmgr_mod_sa_ctx_info *)p_info->skb->sp;

	if (!ctx_info)
		return 0;

	iph = ip_hdr(p_info->skb);

	if (iph->version != IPVERSION)
		return 0;

	ihl = iph->ihl * 4;

	if (iph->protocol == IPPROTO_UDP) {
		/* UDP encapsulation for IPSec NAT-T */
		offset = (ulong)(skb_network_header(p_info->skb) -
			p_info->skb->data) + ihl + sizeof(struct udphdr);
		len = ntohs(iph->tot_len) - ihl - sizeof(struct udphdr);
	} else if (iph->protocol == IPPROTO_ESP) {
		offset = (ulong)(skb_network_header(p_info->skb) -
			p_info->skb->data) + ihl;
		len = ntohs(iph->tot_len) - ihl;
	} else {
	    return 0;
	}

	psdata = netcp_push_psdata(p_info, (2 * sizeof(u32)));
	if (!psdata)
		return -ENOMEM;

	psdata[0] = PASAHO_SINFO_FORMAT_CMD(offset, len);
	psdata[1] = 0;
	swinfo = &p_info->epib[1];
	swinfo[0] = ctx_info->word0;
	swinfo[1] = ctx_info->word1;
	SA_SWINFO_UPDATE_DEST_INFO(swinfo, p_info->tx_pipe->dma_queue,
			ctx_info->flow_id);

	p_info->tx_pipe = &sa_dev->tx_pipe;
	kfree(ctx_info);
	p_info->skb->sp = NULL;
	return 0;
}

static int sa_close(void *intf_priv, struct net_device *ndev)
{
	struct sa_device *sa_dev = intf_priv;
	struct netcp_priv *netcp_priv = netdev_priv(ndev);

	netcp_unregister_txhook(netcp_priv, SA_TXHOOK_ORDER, sa_tx_hook, sa_dev);

	netcp_txpipe_close(&sa_dev->tx_pipe);

	return 0;
}

static int sa_open(void *intf_priv, struct net_device *ndev)
{
	struct sa_device *sa_dev = intf_priv;
	struct netcp_priv *netcp_priv = netdev_priv(ndev);
	int ret;

	/* Open the SA IPSec data transmit channel */
	ret = netcp_txpipe_open(&sa_dev->tx_pipe);
	if (ret)
		goto fail;

	netcp_register_txhook(netcp_priv, SA_TXHOOK_ORDER, sa_tx_hook, sa_dev);
	return 0;

fail:
	sa_close(intf_priv, ndev);
	return ret;
}

static int sa_attach(void *inst_priv, struct net_device *ndev, void **intf_priv)
{
	struct netcp_priv *netcp = netdev_priv(ndev);
	struct sa_device *sa_dev = inst_priv;
	char node_name[24];

	snprintf(node_name, sizeof(node_name), "interface-%d",
		 (sa_dev->multi_if) ? (netcp->cpsw_port - 1) : 0);

	if (of_find_property(sa_dev->node, node_name, NULL)) {
		sa_dev->net_device = ndev;
		*intf_priv = sa_dev;

		netcp_txpipe_init(&sa_dev->tx_pipe, netdev_priv(ndev),
				  sa_dev->tx_chan_name, sa_dev->tx_queue_depth);

		sa_dev->tx_pipe.dma_psflags = netcp->cpsw_port;

		return 0;
	} else
		return -ENODEV;
}

static int sa_release(void *intf_priv)
{
	struct sa_device *sa_dev = intf_priv;

	printk("%s() called for interface %s\n", __func__, sa_dev->net_device->name);
	sa_dev->net_device = NULL;
	return 0;
}

static int sa_remove(struct netcp_device *netcp_device, void *inst_priv)
{
	struct sa_device *sa_dev = inst_priv;
	kfree(sa_dev);
	return 0;
}

static int sa_probe(struct netcp_device *netcp_device,
		    struct device *dev,
		    struct device_node *node,
		    void **inst_priv)
{
	struct sa_device *sa_dev;
	int ret = 0;

	if (!node) {
		dev_err(dev, "device tree info unavailable\n");
		return -ENODEV;
	}

	sa_dev = devm_kzalloc(dev, sizeof(struct sa_device), GFP_KERNEL);
	if (!sa_dev) {
		dev_err(dev, "memory allocation failed\n");
		return -ENOMEM;
	}
	*inst_priv = sa_dev;
	sa_dev->dev = dev;

	sa_dev->node = node;

	if (of_find_property(node, "multi-interface", NULL))
		sa_dev->multi_if = 1;

	ret = of_property_read_string(node, "tx-channel", &sa_dev->tx_chan_name);
	if (ret < 0) {
		dev_err(dev, "missing \"tx-channel\" parameter, err %d\n", ret);
		sa_dev->tx_chan_name = "satx";
	}
	dev_dbg(dev, "tx-channel \"%s\"\n", sa_dev->tx_chan_name);

	ret = of_property_read_u32(node, "tx_queue_depth",
				   &sa_dev->tx_queue_depth);
	if (ret < 0) {
		dev_err(dev, "missing tx_queue_depth parameter, err %d\n", ret);
		sa_dev->tx_queue_depth = 32;
	}
	dev_dbg(dev, "tx_queue_depth %u\n", sa_dev->tx_queue_depth);

	return 0;
}

static struct netcp_module sa_module = {
	.name		= "keystone-sa",
	.owner		= THIS_MODULE,
	.probe		= sa_probe,
	.open		= sa_open,
	.close		= sa_close,
	.remove		= sa_remove,
	.attach		= sa_attach,
	.release	= sa_release,
};

static int __init keystone_sa_init(void)
{
	return netcp_register_module(&sa_module);
}
module_init(keystone_sa_init);

static void __exit keystone_sa_exit(void)
{
	netcp_unregister_module(&sa_module);
}
module_exit(keystone_sa_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Sandeep Nair <sandeep_n@ti.com>");
MODULE_DESCRIPTION("IPSec driver for Keystone devices");

