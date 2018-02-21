/*
 * Copyright (C) 2010-2013 Freescale Semiconductor, Inc.
 * Copyright (C) 2013, Boundary Devices <info@boundarydevices.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/sys_proto.h>
#include <malloc.h>
#include <asm/arch/mx6-pins.h>
#include <linux/errno.h>
#include <asm/gpio.h>
#include <asm/imx-common/boot_mode.h>
#include <asm/imx-common/fbpanel.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/imx-common/sata.h>
#include <asm/imx-common/spi.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <splash.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/mxc_hdmi.h>
#include <i2c.h>
#include <input.h>
#include <usb/ehci-ci.h>
#include "../common/bd_common.h"
#include "../common/padctrl.h"

DECLARE_GLOBAL_DATA_PTR;

#define BUTTON_PAD_CTRL	(PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define I2C_PAD_CTRL	(PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |	\
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)

#define RGB_PAD_CTRL	PAD_CTL_DSE_120ohm

#define SPI_PAD_CTRL	(PAD_CTL_HYS | PAD_CTL_SPEED_MED |	\
	PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

#define CAN_PAD_CTRL   (PAD_CTL_PUS_100K_UP | \
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |\
	PAD_CTL_HYS | PAD_CTL_SRE_FAST)

#define UART_PAD_CTRL	(PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_HYS | PAD_CTL_SRE_FAST)

#define USDHC_PAD_CTRL	(PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_HYS | PAD_CTL_SRE_FAST)

#define AUD_PAD_CTRL  (PAD_CTL_PUS_47K_UP | \
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm | \
	PAD_CTL_HYS | PAD_CTL_SRE_FAST)

static const iomux_v3_cfg_t init_pads[] = {
	/* bt_rfkill */
#define GP_BT_RFKILL_RESET	IMX_GPIO_NR(2, 5)
	IOMUX_PAD_CTRL(NANDF_D5__GPIO2_IO05, WEAK_PULLDN),

	/* ENET pads that don't change for PHY reset */
#define GPIRQ_ENET_PHY		IMX_GPIO_NR(1, 28)
	IOMUX_PAD_CTRL(ENET_TX_EN__GPIO1_IO28, WEAK_PULLUP),
#define GP_RGMII_PHY_RESET	IMX_GPIO_NR(1, 25)
	IOMUX_PAD_CTRL(ENET_CRS_DV__GPIO1_IO25, WEAK_PULLUP),

	IOMUX_PAD_CTRL(ENET_MDIO__ENET_MDIO, PAD_CTRL_ENET_MDIO),
	IOMUX_PAD_CTRL(ENET_MDC__ENET_MDC, PAD_CTRL_ENET_MDC),
	IOMUX_PAD_CTRL(ENET_REF_CLK__ENET_TX_CLK, PAD_CTRL_ENET_TX),
	IOMUX_PAD_CTRL(RGMII_TXC__RGMII_TXC, PAD_CTRL_ENET_TX),
	IOMUX_PAD_CTRL(RGMII_TD0__RGMII_TD0, PAD_CTRL_ENET_TX),
	IOMUX_PAD_CTRL(RGMII_TD1__RGMII_TD1, PAD_CTRL_ENET_TX),
	IOMUX_PAD_CTRL(RGMII_TD2__RGMII_TD2, PAD_CTRL_ENET_TX),
	IOMUX_PAD_CTRL(RGMII_TD3__RGMII_TD3, PAD_CTRL_ENET_TX),
	IOMUX_PAD_CTRL(RGMII_TX_CTL__RGMII_TX_CTL, PAD_CTRL_ENET_TX),


	// camera
	IOMUX_PAD_CTRL(GPIO_3__CCM_CLKO2, OUTPUT_40OHM),	/* mclk, MIPI_XCLK*/

	/* PWM4 - Backlight on LVDS connector: J6 */
#define GP_BACKLIGHT_LVDS	IMX_GPIO_NR(1, 9)
	IOMUX_PAD_CTRL(GPIO_9__GPIO1_IO09, WEAK_PULLDN),

	/* reg_wlan_en */
#define GP_REG_WLAN_EN		IMX_GPIO_NR(6, 8)
	IOMUX_PAD_CTRL(NANDF_ALE__GPIO6_IO08, WEAK_PULLDN),

	/* UART1, SD3_D6 and SD3_D7 on schematic, not connected
	IOMUX_PAD_CTRL(SD3_DAT6__GPIO6_IO18, WEAK_PULLDN),
	IOMUX_PAD_CTRL(SD3_DAT7__GPIO6_IO17, WEAK_PULLDN),*/
	/* UART1, CSI0_D10 and CSI0_D11 on schematic, touch panel, resistors not mounted */
	IOMUX_PAD_CTRL(CSI0_DAT10__UART1_TX_DATA, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(CSI0_DAT11__UART1_RX_DATA, UART_PAD_CTRL),

	/* UART2, EIM_D26 and EIM_D27 on schematic, not connected
	IOMUX_PAD_CTRL(EIM_D26__GPIO3_IO26, WEAK_PULLDN),
	IOMUX_PAD_CTRL(EIM_D27__GPIO3_IO27, WEAK_PULLDN),*/
	/* UART2, SD4_D4..SD4_D7 on schematic, Bluetooth HCI*/
	IOMUX_PAD_CTRL(SD4_DAT7__UART2_TX_DATA, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT6__UART2_CTS_B, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT5__UART2_RTS_B, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT4__UART2_RX_DATA, UART_PAD_CTRL),

	/* UART3, EIM_D24 and EIM_D25 on schematic, console */
#ifndef CONFIG_SILENT_UART
	IOMUX_PAD_CTRL(EIM_D24__UART3_TX_DATA, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_D25__UART3_RX_DATA, UART_PAD_CTRL),
#else
	IOMUX_PAD_CTRL(EIM_D24__GPIO3_IO26, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(EIM_D25__GPIO3_IO27, UART_PAD_CTRL),
#endif
	/* UART4, EIM_D26 and EIM_D27 on schematic, not connected
	IOMUX_PAD_CTRL(EIM_D26__GPIO3_IO26, WEAK_PULLDN),
	IOMUX_PAD_CTRL(EIM_D27__GPIO3_IO27, WEAK_PULLDN),*/
	/* UART4, CSI0_D12 and CSI0_D13 on schematic, connected to st485ebdr - RS485, Tukan? */
	IOMUX_PAD_CTRL(CSI0_DAT12__UART4_TX_DATA, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(CSI0_DAT13__UART4_RX_DATA, UART_PAD_CTRL),
	/* UART4, manual CTS, as Tukan needs manual CS, hence setting as GPIO */
#define GP_TUKAN_CS           IMX_GPIO_NR(6, 3)
	IOMUX_PAD_CTRL(CSI0_DAT17__GPIO6_IO03, WEAK_PULLDN),

		/* UART5, CSI0_D12 and CSI0_D13 on schematic,  */
	IOMUX_PAD_CTRL(CSI0_DAT14__UART5_TX_DATA, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(CSI0_DAT15__UART5_RX_DATA, UART_PAD_CTRL),

	/* CAN1, GPIO_07 and GPIO_08 on schematic, MC2562-E CAN controller */
#define GP_CAN_SBY           IMX_GPIO_NR(1, 6)
	IOMUX_PAD_CTRL(GPIO_6__GPIO1_IO06, OUTPUT_40OHM),
#define GP_LED           IMX_GPIO_NR(1, 7)
	IOMUX_PAD_CTRL(GPIO_7__GPIO1_IO07, OUTPUT_40OHM),
//	IOMUX_PAD_CTRL(GPIO_7__FLEXCAN1_TX, CAN_PAD_CTRL),
	IOMUX_PAD_CTRL(GPIO_8__FLEXCAN1_RX, CAN_PAD_CTRL),

	/* USB HOST
	 * (power supplied by http://ww1.microchip.com/downloads/en/DeviceDoc/mic20xx.pdf chipset)
	 */
#define GP_REG_USBHOST		IMX_GPIO_NR(3, 31)
	IOMUX_PAD_CTRL(EIM_D31__USB_H1_PWR, WEAK_PULLDN),
	IOMUX_PAD_CTRL(EIM_D30__USB_H1_OC, WEAK_PULLUP),

	/* USB OTG
	 * (power supplied by http://ww1.microchip.com/downloads/en/DeviceDoc/mic20xx.pdf chipset)
	 */
#define GP_REG_USBOTG		IMX_GPIO_NR(4, 15)
	IOMUX_PAD_CTRL(KEY_ROW4__USB_OTG_PWR, WEAK_PULLDN),
	IOMUX_PAD_CTRL(KEY_COL4__USB_OTG_OC, WEAK_PULLUP),

	/* AUD4 - AudioCodec */
	IOMUX_PAD_CTRL(SD2_DAT0__AUD4_RXD, AUD_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_DAT1__AUD4_TXFS, AUD_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_DAT2__AUD4_TXD, AUD_PAD_CTRL),
	IOMUX_PAD_CTRL(SD2_DAT3__AUD4_TXC, AUD_PAD_CTRL),
	/* AudioCodec, 12MHz clock via Clock Controller Module*/
	IOMUX_PAD_CTRL(GPIO_0__CCM_CLKO1, OUTPUT_40OHM),

	/* USDHC1 - sdcard (external) */
	IOMUX_PAD_CTRL(SD1_CLK__SD1_CLK, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD1_CMD__SD1_CMD, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD1_DAT0__SD1_DATA0, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD1_DAT1__SD1_DATA1, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD1_DAT2__SD1_DATA2, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD1_DAT3__SD1_DATA3, USDHC_PAD_CTRL),

	/* USDHC3 - sdcard (bootable) */
	IOMUX_PAD_CTRL(SD3_CLK__SD3_CLK, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_CMD__SD3_CMD, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT0__SD3_DATA0, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT1__SD3_DATA1, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT2__SD3_DATA2, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD3_DAT3__SD3_DATA3, USDHC_PAD_CTRL),
#define GP_USDHC3_RESET		IMX_GPIO_NR(7, 8)
	IOMUX_PAD_CTRL(SD3_RST__SD3_RESET, WEAK_PULLUP),
#define GP_USDHC3_VSELECT	IMX_GPIO_NR(6, 14)
	IOMUX_PAD_CTRL(NANDF_CS1__SD3_VSELECT, OUTPUT_40OHM),

	/* USDHC4 - WiFi */
	IOMUX_PAD_CTRL(SD4_CLK__SD4_CLK, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_CMD__SD4_CMD, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT0__SD4_DATA0, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT1__SD4_DATA1, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT2__SD4_DATA2, USDHC_PAD_CTRL),
	IOMUX_PAD_CTRL(SD4_DAT3__SD4_DATA3, USDHC_PAD_CTRL),

	IOMUX_PAD_CTRL(NANDF_CS1__SD3_VSELECT, OUTPUT_40OHM),
};

static const struct i2c_pads_info i2c_pads[] = {
	/* I2C1, TouchPanel? */
	I2C_PADS_INFO_ENTRY(I2C1, CSI0_DAT9, 5, 27, CSI0_DAT8, 5, 26, I2C_PAD_CTRL),
	/* I2C3, AudioCodec */
	I2C_PADS_INFO_ENTRY(I2C3, EIM_D17, 3, 17, EIM_D18, 3, 18, I2C_PAD_CTRL),
};
#define I2C_BUS_CNT	2

#ifdef CONFIG_USB_EHCI_MX6
int board_ehci_hcd_init(int port)
{
	if (port) {
		/* Reset USB hub */
		/* no HUB to reset
		gpio_set_value(GP_USB_HUB_RESET, 0);
		mdelay(2);
		gpio_set_value(GP_USB_HUB_RESET, 1);
		*/
	}

	return 0;
}

int board_ehci_power(int port, int on)
{
	if (port)
		return 0;
	gpio_set_value(GP_REG_USBOTG, on);
	return 0;
}

#endif

#ifdef CONFIG_FSL_ESDHC
struct fsl_esdhc_cfg board_usdhc_cfg[] = {
	{.esdhc_base = USDHC3_BASE_ADDR, .bus_width = 4,
			.gp_cd = 0,
			.gp_reset = GP_USDHC3_RESET },
	{.esdhc_base = USDHC1_BASE_ADDR, .bus_width = 4,
			.gp_cd = 0},
};
#endif

#ifdef CONFIG_MXC_SPI
int board_spi_cs_gpio(unsigned bus, unsigned cs)
{
//	if (bus == 0 && cs == 0)
//		return GP_ECSPI1_NOR_CS;
//	if (cs >> 8)
//		return (cs >> 8);
	return -1;
}
#endif

#ifdef CONFIG_CMD_FBPANEL
void board_enable_lvds(const struct display_info_t *di, int enable)
{
	gpio_direction_output(GP_BACKLIGHT_LVDS, enable);
}

static const struct display_info_t displays[] = {
	VD_AUO_FHD_15IN_12V(LVDS, NULL, 0, 0x00)

	/* fusion7 specific touchscreen */
	//VD_FUSION7(LCD, fbp_detect_i2c, 2, 0x10),
};
#define display_cnt	ARRAY_SIZE(displays)
#else
#define displays	NULL
#define display_cnt	0
#endif

static const unsigned short gpios_out_low[] = {
	GP_RGMII_PHY_RESET, /* low disables the Ethernet */
	GP_BACKLIGHT_LVDS, /* PWM output for LVDs connector */
	GP_REG_WLAN_EN, /* low disables WiFi */
	GP_BT_RFKILL_RESET, /* low disables BT */
	GP_REG_USBOTG, /* LOW disables OTG power */
	GP_REG_USBHOST, /* LOW disables HOST power */
	GP_CAN_SBY, /* low ENABLES CAN controller */
	GP_LED, /* low turns ON LED between CANH and CANL */
};

static const unsigned short gpios_out_high[] = {
	GP_USDHC3_VSELECT /* high=3.3v */,
};

static const unsigned short gpios_in[] = {
	GPIRQ_ENET_PHY,
	GP_USDHC3_RESET,
};

int board_early_init_f(void)
{
	set_gpios_in(gpios_in, ARRAY_SIZE(gpios_in));
	set_gpios(gpios_out_high, ARRAY_SIZE(gpios_out_high), 1);
	set_gpios(gpios_out_low, ARRAY_SIZE(gpios_out_low), 0);
	SETUP_IOMUX_PADS(init_pads);

	return 0;
}

int board_init(void)
{
	common_board_init(i2c_pads, I2C_BUS_CNT, IOMUXC_GPR1_OTG_ID_GPIO1,
			displays, display_cnt, 0);

	gpio_set_value(GP_LED, 0);
	mdelay(200);
	gpio_set_value(GP_LED, 1);
	mdelay(200);
	gpio_set_value(GP_LED, 0);
	mdelay(200);
	gpio_set_value(GP_LED, 1);
	mdelay(200);
	gpio_set_value(GP_LED, 0);
	mdelay(200);
	gpio_set_value(GP_LED, 1);
	mdelay(200);
	gpio_set_value(GP_LED, 0);
	mdelay(200);
	gpio_set_value(GP_LED, 1);
	mdelay(200);
	gpio_set_value(GP_LED, 0);
	mdelay(200);
	gpio_set_value(GP_LED, 1);
	mdelay(200);
	gpio_set_value(GP_LED, 0);
	mdelay(200);
	gpio_set_value(GP_LED, 1);
	mdelay(200);

	return 0;
}

#ifndef CONFIG_SYS_BOARD
const char *board_get_board_type(void)
{
	return "nexo";
}
#endif

const struct button_key board_buttons[] = {
	{NULL, 0, 0, 0},
};


#ifdef CONFIG_CMD_BMODE
const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"mmc0",	MAKE_CFGVAL(0x40, 0x30, 0x00, 0x00)},
	{"mmc1",	MAKE_CFGVAL(0x40, 0x38, 0x00, 0x00)},
	{NULL,		0},
};
#endif
