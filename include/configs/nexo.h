/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Boundary Devices Nitrogen6_max
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include "mx6_common.h"

#define CONFIG_MACH_TYPE	3778

#define CONFIG_MXC_UART_BASE UART3_BASE

#define CONFIG_IMX_HDMI
#define CONFIG_SYS_FSL_USDHC_NUM	2
#define CONFIG_USB_MAX_CONTROLLER_COUNT 2
#define BD_I2C_MASK	7

#define CONFIG_ENV_IS_IN_MMC
/* 1MB after beginning of disk (uboot ends around 0.5MB right now) */
#define CONFIG_ENV_OFFSET		(1 * 1024 * 1024)
#define CONFIG_SYS_MMC_ENV_DEV		0

/* Network */
#define CONFIG_FEC_MXC
#define CONFIG_MII
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_XCV_TYPE		RGMII
#define CONFIG_ETHPRIME			"FEC"
#define CONFIG_FEC_MXC_PHYADDR		3
#define CONFIG_PHYLIB
#define CONFIG_PHY_MICREL
#define CONFIG_PHY_MICREL_KSZ9031

#include "boundary.h"
#define CONFIG_EXTRA_ENV_SETTINGS BD_BOUNDARY_ENV_SETTINGS \
	"serverip=192.168.1.100\0" \
	"ethaddr=00:19:b8:00:00:03\0" \
	"uload=setenv autoload 0;" \
		"dhcp;"\
		"nfs 0x12000000 192.168.1.100:/srv/aosp/uboot/u-boot.imx;"\
		"\0" \
	"uburn=mmc dev 0;" \
		"mmc write 0x12000000 0x2 0x700;"\
		"\0" \
	"u=run uload; run uburn" \
		"\0" \
	"kload=setenv autoload 0;" \
		"dhcp;"\
		"nfs 0x13000000 192.168.1.100:/srv/aosp/boot/imx6qp-nexo.dtb;" \
		"fdt addr 0x13000000;" \
		"setenv fdt_high 0xffffffff;" \
		"nfs 0x13800000 192.168.1.100:/srv/aosp/boot/uramdisk.img;"\
		"nfs 0x10800000 192.168.1.100:/srv/aosp/boot/zImage"\
		"\0" \
	"kboot=" \
		"setenv bootargs console=ttymxc2,115200 vmalloc=128M consoleblank=0 rootwait androidboot.hardware=freescale androidboot.bootdev=mmcblk1 androidboot.serialno=00 cma=448M;" \
		"bootz 0x10800000 0x13800000 0x13000000" \
		"\0" \
	"k=run kload; run kboot" \
		"\0" \

#endif	       /* __CONFIG_H */
