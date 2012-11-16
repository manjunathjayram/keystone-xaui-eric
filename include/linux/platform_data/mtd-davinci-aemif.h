/*
 * TI DaVinci AEMIF support
 *
 * Copyright 2010 (C) Texas Instruments, Inc. http://www.ti.com/
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */
#ifndef _MACH_DAVINCI_AEMIF_H
#define _MACH_DAVINCI_AEMIF_H

#define NRCSR_OFFSET		0x00
#define AWCCR_OFFSET		0x04
#define A1CR_OFFSET		0x10

#define ACR_ASIZE_MASK		0x3
#define ACR_EW_MASK		BIT(30)
#define ACR_SS_MASK		BIT(31)
#define ASIZE_16BIT		1

struct davinci_aemif_cs_data {
	u8	cs;
	u16	wstrobe;
	u16	rstrobe;
	u8	wsetup;
	u8	whold;
	u8	rsetup;
	u8	rhold;
	u8	ta;
	u8	enable_ss;
	u8	enable_ew;
	u8	asize;
};

struct davinci_aemif_pdata {
	u8	num_cs;
	struct davinci_aemif_cs_data cs_data[4];
};

/* API to Get current Asynchrnous emif bus parameters */
struct davinci_aemif_cs_data *davinci_aemif_get_abus_params(unsigned int cs);

/* API to Set current Asynchrnous emif bus parameters */
int davinci_aemif_set_abus_params(unsigned int cs,
			struct davinci_aemif_cs_data *data);

#endif
