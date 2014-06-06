#
# Makefile for the TI network device drivers.
#

obj-$(CONFIG_TLAN) += tlan.o
obj-$(CONFIG_CPMAC) += cpmac.o
obj-$(CONFIG_TI_DAVINCI_EMAC) += davinci_emac.o
obj-$(CONFIG_TI_DAVINCI_MDIO) += davinci_mdio.o
obj-$(CONFIG_TI_DAVINCI_CPDMA) += davinci_cpdma.o
obj-$(CONFIG_TI_CPSW) += ti_cpsw.o
ti_cpsw-y := cpsw_ale.o cpsw.o cpts.o
obj-$(CONFIG_TI_KEYSTONE_NET) += keystone_net.o
obj-$(CONFIG_TI_KEYSTONE_PA) += keystone_pa.o
obj-$(CONFIG_TI_KEYSTONE_QOS) += keystone_qos.o
obj-$(CONFIG_TI_KEYSTONE_NET_SA) += keystone_net_sa.o
keystone_net-y += cpsw_ale.o cpts.o	\
		  keystone_ethss.o	\
		  keystone_sgmii.o	\
		  keystone_net_core.o	\
		  keystone_serdes.o
obj-$(CONFIG_TI_KEYSTONE_XGE) += keystone_xge.o
obj-$(CONFIG_TI_KEYSTONE_XGE_MDIO) += keystone_xgemdio.o
keystone_xge-y += keystone_xgess.o	\
		  keystone_xgepcsr.o
