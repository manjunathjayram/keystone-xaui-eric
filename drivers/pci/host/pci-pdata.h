/*
 * Copyright 2013 Texas Instruments, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* platform specific setup for k2 serdes */
int k2_pcie_platform_setup(void *pdata, struct device_node *);

/* keystone pcie pdata configurations */
struct keystone_pcie_pdata {
	int (*setup)(void *pdata, struct device_node *);
	int en_link_train;
};

