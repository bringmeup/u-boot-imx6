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
/* change when needed, DEVICE 1 is the 'BIG' SD card slot */
#define CONFIG_SYS_MMC_ENV_DEV	0
#define NEXO_UBOOT_MMC_DEV		"0"
#define NEXO_OS_MMC_DEV			"0"
#define NEXO_NFS_SRV_ADDRESS	"192.168.1.100"

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
	"serverip="NEXO_NFS_SRV_ADDRESS"\0" \
	"nexohostip="NEXO_NFS_SRV_ADDRESS"\0" \
	"ethaddr=00:19:b8:00:00:03\0" \
	"a_zImage=0x10800000\0" \
	"a_fdt=0x13000000\0" \
	"a_ramdisk=0x13800000\0" \
	"a_uboot=0x12000000\0" \
	"devos="NEXO_OS_MMC_DEV"\0" \
	"devuboot="NEXO_UBOOT_MMC_DEV"\0" \
	"nexo=" \
		"setenv bootargs " \
			"'console='${console}',115200 vmalloc=128M "\
			"consoleblank=0 rootwait androidboot.hardware=freescale "\
			"androidboot.bootdev=mmcblk'${devos}' androidboot.serialno=d1500bc69 " \
			"wlcore.mac='${wlmac}' androidboot.wlan.mac='${wlmac}' " \
			"androidboot.btmacaddr='${wlmac}' cma=448M';" \
		"setenv kload " \
			"'setenv autoload 0; dhcp;" \
			"nfs '${a_fdt} ${nexohostip}':/srv/aosp/boot/imx6qp-nexo.dtb;" \
			"fdt addr '${a_fdt}';" \
			"setenv fdt_high 0xffffffff;" \
			"nfs '${a_ramdisk} ${nexohostip}':/srv/aosp/boot/uramdisk.img;"\
			"nfs '${a_zImage} ${nexohostip}':/srv/aosp/boot/zImage';"\
		"setenv kboot " \
			"'bootz '${a_zImage} ${a_ramdisk} ${a_fdt};" \
		"setenv uload " \
			"'setenv autoload 0; dhcp;" \
			"nfs '${a_uboot} ${nexohostip}':/srv/aosp/uboot/u-boot.imx';"\
		"setenv uburn " \
			"'mmc dev '${devuboot}'; mmc write '${a_uboot}' 0x2 0x700';"\
		"\0" \
	"n=run nexo" \
		"\0" \
	"ul=run nexo; run uload" \
		"\0" \
	"ub=run nexo; run uburn" \
		"\0" \
	"u=run nexo; run uload; run uburn" \
		"\0" \
	"kl=run nexo; run kload" \
		"\0" \
	"kb=run nexo; run kboot" \
		"\0" \
	"k=run nexo; run kload; run kboot" \
		"\0" \



#endif	       /* __CONFIG_H */
