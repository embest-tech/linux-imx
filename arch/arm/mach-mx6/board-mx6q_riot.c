/*
 * Copyright (C) 2012-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/i2c.h>
#include <linux/i2c/pca953x.h>
#include <linux/ata.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/regulator/consumer.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/pwm_backlight.h>
#include <linux/fec.h>
#include <linux/memblock.h>
#include <linux/gpio.h>
#include <linux/etherdevice.h>
//#include <linux/power/riot_battery.h>
#include <linux/regulator/anatop-regulator.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/mfd/max17135.h>
#include <linux/mfd/wm8994/pdata.h>
#include <linux/mfd/wm8994/gpio.h>
#include <sound/wm8962.h>
#include <linux/mfd/mxc-hdmi-core.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/mxc_dvfs.h>
#include <mach/memory.h>
#include <mach/iomux-mx6q.h>
#include <mach/imx-uart.h>
#include <mach/viv_gpu.h>
//#include <mach/ahci_sata.h>
#include <mach/ipu-v3.h>
#include <mach/mxc_hdmi.h>
#include <mach/mxc_asrc.h>
#include <mach/mipi_dsi.h>

#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include "usb.h"
#include "devices-imx6q.h"
#include "crm_regs.h"
#include "cpu_op-mx6.h"
#include "board-mx6dl_riot.h"

#define RIOT_USR_DEF_GRN_LED	IMX_GPIO_NR(1, 1)
#define RIOT_VOLUME_DN	IMX_GPIO_NR(1, 5)
#define RIOT_MICROPHONE_DET	IMX_GPIO_NR(1, 9)
#define RIOT_ACCL_INT	IMX_GPIO_NR(1, 18)
#define RIOT_MIPICSI_PWN	IMX_GPIO_NR(1, 19)
#define RIOT_MIPICSI_RST	IMX_GPIO_NR(1, 20)
#define RIOT_RGMII_RST	IMX_GPIO_NR(1, 25)
#define RIOT_RGMII_INT	IMX_GPIO_NR(1, 26)
#define RIOT_CHARGE_UOK_B	IMX_GPIO_NR(1, 27)
#define RIOT_DISP0_PWR_EN	IMX_GPIO_NR(1, 30)

#define RIOT_SD3_CD		IMX_GPIO_NR(7, 0)
#define RIOT_SD3_WP		IMX_GPIO_NR(7, 1)
#define RIOT_SD2_CD		IMX_GPIO_NR(1, 4)
#define RIOT_SD2_WP		IMX_GPIO_NR(1, 2)
#define RIOT_CHARGE_DOK_B	IMX_GPIO_NR(2, 24)
#define RIOT_GPS_RESET	IMX_GPIO_NR(2, 28)
#define RIOT_SENSOR_EN	IMX_GPIO_NR(2, 31)

#define RIOT_GPS_EN	IMX_GPIO_NR(3, 0)
#define RIOT_DISP0_RST_B	IMX_GPIO_NR(3, 8)
#define RIOT_CHARGE_CHG_2_B	IMX_GPIO_NR(3, 13)
#define RIOT_CHARGE_FLT_2_B	IMX_GPIO_NR(3, 14)
#define RIOT_BAR0_INT	IMX_GPIO_NR(3, 15)
#define RIOT_eCOMPASS_INT	IMX_GPIO_NR(3, 16)
#define RIOT_GPS_PPS		IMX_GPIO_NR(3, 18)
#define RIOT_CSI0_RST              IMX_GPIO_NR(3, 19)
#define RIOT_CSI0_PWD              IMX_GPIO_NR(3, 20)
#define RIOT_USB_OTG_PWR	IMX_GPIO_NR(3, 22)
#define RIOT_CHARGE_CHG_1_B	IMX_GPIO_NR(3, 23)
#define RIOT_TS_INT		IMX_GPIO_NR(3, 26)
#define RIOT_POWER_OFF	IMX_GPIO_NR(3, 29)

#define RIOT_CAN1_STBY	IMX_GPIO_NR(4, 5)
#define RIOT_ECSPI1_CS0  IMX_GPIO_NR(4, 9)
#define RIOT_CODEC_PWR_EN	IMX_GPIO_NR(4, 10)
#define RIOT_HDMI_CEC_IN	IMX_GPIO_NR(4, 11)
#define RIOT_PCIE_DIS_B	IMX_GPIO_NR(4, 14)

#define RIOT_DI0_D0_CS	IMX_GPIO_NR(5, 0)
#define RIOT_PCIE_WAKE_B	IMX_GPIO_NR(5, 20)

#define RIOT_CAP_TCH_INT1	IMX_GPIO_NR(6, 7)
#define RIOT_CAP_TCH_INT0	IMX_GPIO_NR(6, 8)
#define RIOT_DISP_RST_B	IMX_GPIO_NR(6, 11)
#define RIOT_LED_PWN          IMX_GPIO_NR(6, 15)
#define RIOT_CABC_EN1	IMX_GPIO_NR(6, 16)
#define RIOT_AUX_3V15_EN	IMX_GPIO_NR(6, 9)
#define RIOT_DISP0_WR_REVB	IMX_GPIO_NR(6, 9)
#define RIOT_AUX_5V_EN	IMX_GPIO_NR(6, 10)
#define RIOT_DI1_D0_CS	IMX_GPIO_NR(6, 31)

#define RIOT_HEADPHONE_DET	IMX_GPIO_NR(7, 8)
#define RIOT_PCIE_RST_B_REVB	IMX_GPIO_NR(7, 12)
#define RIOT_PMIC_INT_B	IMX_GPIO_NR(7, 13)
#define RIOT_PFUZE_INT	IMX_GPIO_NR(7, 13)

#define RIOT_EPDC_SDDO_0	IMX_GPIO_NR(2, 22)
#define RIOT_EPDC_SDDO_1	IMX_GPIO_NR(3, 10)
#define RIOT_EPDC_SDDO_2	IMX_GPIO_NR(3, 12)
#define RIOT_EPDC_SDDO_3	IMX_GPIO_NR(3, 11)
#define RIOT_EPDC_SDDO_4	IMX_GPIO_NR(2, 27)
#define RIOT_EPDC_SDDO_5	IMX_GPIO_NR(2, 30)
#define RIOT_EPDC_SDDO_6	IMX_GPIO_NR(2, 23)
#define RIOT_EPDC_SDDO_7	IMX_GPIO_NR(2, 26)
#define RIOT_EPDC_SDDO_8	IMX_GPIO_NR(2, 24)
#define RIOT_EPDC_SDDO_9	IMX_GPIO_NR(3, 15)
#define RIOT_EPDC_SDDO_10	IMX_GPIO_NR(3, 16)
#define RIOT_EPDC_SDDO_11	IMX_GPIO_NR(3, 23)
//#define RIOT_EPDC_SDDO_12	IMX_GPIO_NR(3, 19)
#define RIOT_EPDC_SDDO_13	IMX_GPIO_NR(3, 13)
#define RIOT_EPDC_SDDO_14	IMX_GPIO_NR(3, 14)
#define RIOT_EPDC_GDCLK	IMX_GPIO_NR(2, 17)
#define RIOT_EPDC_GDSP	IMX_GPIO_NR(2, 16)
#define RIOT_EPDC_GDOE	IMX_GPIO_NR(6, 6)
#define RIOT_EPDC_GDRL	IMX_GPIO_NR(5, 4)
#define RIOT_EPDC_SDCLK	IMX_GPIO_NR(3, 31)
#define RIOT_EPDC_SDOEZ	IMX_GPIO_NR(3, 30)
#define RIOT_EPDC_SDOED	IMX_GPIO_NR(3, 26)
#define RIOT_EPDC_SDOE	IMX_GPIO_NR(3, 27)
#define RIOT_EPDC_SDLE	IMX_GPIO_NR(3, 1)
#define RIOT_EPDC_SDCLKN	IMX_GPIO_NR(3, 0)
#define RIOT_EPDC_SDSHR	IMX_GPIO_NR(2, 29)
#define RIOT_EPDC_PWRCOM	IMX_GPIO_NR(2, 28)
#define RIOT_EPDC_PWRSTAT	IMX_GPIO_NR(2, 21)
#define RIOT_EPDC_PWRCTRL0	IMX_GPIO_NR(2, 20)
#define RIOT_EPDC_PWRCTRL1	IMX_GPIO_NR(2, 19)
#define RIOT_EPDC_PWRCTRL2	IMX_GPIO_NR(2, 18)
#define RIOT_EPDC_BDR0	IMX_GPIO_NR(3, 2)
#define RIOT_EPDC_BDR1	IMX_GPIO_NR(3, 3)
#define RIOT_EPDC_SDCE0	IMX_GPIO_NR(3, 4)
#define RIOT_EPDC_SDCE1	IMX_GPIO_NR(3, 5)
#define RIOT_EPDC_SDCE2	IMX_GPIO_NR(3, 6)
#define RIOT_EPDC_SDCE3	IMX_GPIO_NR(3, 7)
#define RIOT_EPDC_SDCE4	IMX_GPIO_NR(3, 8)
#define RIOT_EPDC_PMIC_INT	IMX_GPIO_NR(2, 25)
#define RIOT_EPDC_VCOM	IMX_GPIO_NR(3, 17)
#define RIOT_SYS_LED		IMX_GPIO_NR(3, 28)
#define RIOT_USER_LED		IMX_GPIO_NR(5, 2)

#ifdef CONFIG_MX6_ENET_IRQ_TO_GPIO
#define MX6_ENET_IRQ		IMX_GPIO_NR(1, 6)
#define IOMUX_OBSRV_MUX1_OFFSET	0x3c
#define OBSRV_MUX1_MASK		0x3f
#define OBSRV_MUX1_ENET_IRQ	0x9
#endif

static int caam_enabled;

extern char *gp_reg_id;
extern char *soc_reg_id;
extern char *pu_reg_id;

static const struct esdhc_platform_data mx6q_riot_sd2_data __initconst = {
	.cd_gpio = RIOT_SD2_CD,
	.wp_gpio = RIOT_SD2_WP,
	.keep_power_at_suspend = 1,
//	.support_8bit = 1,
//	.delay_line = 0,
	.cd_type = ESDHC_CD_CONTROLLER,
};

static const struct esdhc_platform_data mx6q_riot_sd3_data __initconst = {
	.cd_gpio = RIOT_SD3_CD,
	.wp_gpio = RIOT_SD3_WP,
	.keep_power_at_suspend = 1,
//	.support_8bit = 1,
//	.delay_line = 0,
	.cd_type = ESDHC_CD_CONTROLLER,
};

static const struct esdhc_platform_data mx6q_riot_sd4_data __initconst = {
	.always_present = 1,
	.keep_power_at_suspend = 1,
//	.support_8bit = 1,
//	.delay_line = 0,
	.cd_type = ESDHC_CD_PERMANENT,
};

static const struct anatop_thermal_platform_data
	mx6q_riot_anatop_thermal_data __initconst = {
		.name = "anatop_thermal",
};

static inline void mx6q_riot_init_uart(void)
{
	imx6q_add_imx_uart(0, NULL);
	imx6q_add_imx_uart(1, NULL);
	imx6q_add_imx_uart(2, NULL);
}

static int mx6q_riot_fec_phy_init(struct phy_device *phydev)
{
	unsigned short val;

	/* Ar8031 phy SmartEEE feature cause link status generates glitch,
	 * which cause ethernet link down/up issue, so disable SmartEEE
	 */
	phy_write(phydev, 0xd, 0x3);
	phy_write(phydev, 0xe, 0x805d);
	phy_write(phydev, 0xd, 0x4003);
	val = phy_read(phydev, 0xe);
	val &= ~(0x1 << 8);
	phy_write(phydev, 0xe, val);

	/* To enable AR8031 ouput a 125MHz clk from CLK_25M */
	phy_write(phydev, 0xd, 0x7);
	phy_write(phydev, 0xe, 0x8016);
	phy_write(phydev, 0xd, 0x4007);
	val = phy_read(phydev, 0xe);

	val &= 0xffe3;
	val |= 0x18;
	phy_write(phydev, 0xe, val);

	/* Introduce tx clock delay */
	phy_write(phydev, 0x1d, 0x5);
	val = phy_read(phydev, 0x1e);
	val |= 0x0100;
	phy_write(phydev, 0x1e, val);

	/*check phy power*/
	val = phy_read(phydev, 0x0);

	if (val & BMCR_PDOWN)
		phy_write(phydev, 0x0, (val & ~BMCR_PDOWN));

	return 0;
}

static struct fec_platform_data fec_data __initdata = {
	.init = mx6q_riot_fec_phy_init,
	.phy = PHY_INTERFACE_MODE_RGMII,
#ifdef CONFIG_MX6_ENET_IRQ_TO_GPIO
	.gpio_irq = MX6_ENET_IRQ,
#endif
};

static int mx6q_riot_spi_cs[] = {
	RIOT_ECSPI1_CS0,
};

static const struct spi_imx_master mx6q_riot_spi_data __initconst = {
	.chipselect     = mx6q_riot_spi_cs,
	.num_chipselect = ARRAY_SIZE(mx6q_riot_spi_cs),
};

#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_MTD_M25P80_MODULE)
static struct mtd_partition imx6_riot_spi_nor_partitions[] = {
	{
	 .name = "bootloader",
	 .offset = 0,
	 .size = 0x00100000,
	},
	{
	 .name = "kernel",
	 .offset = MTDPART_OFS_APPEND,
	 .size = MTDPART_SIZ_FULL,
	},
};

static struct flash_platform_data imx6_riot__spi_flash_data = {
	.name = "m25p80",
	.parts = imx6_riot_spi_nor_partitions,
	.nr_parts = ARRAY_SIZE(imx6_riot_spi_nor_partitions),
	.type = "sst25vf016b",
};
#endif

static struct spi_board_info imx6_riot_spi_nor_device[] __initdata = {
#if defined(CONFIG_MTD_M25P80)
	{
		.modalias = "m25p80",
		.max_speed_hz = 20000000, /* max spi clock (SCK) speed in HZ */
		.bus_num = 0,
		.chip_select = 0,
		.platform_data = &imx6_riot__spi_flash_data,
	},
#endif
};

static void spi_device_init(void)
{
	spi_register_board_info(imx6_riot_spi_nor_device,
				ARRAY_SIZE(imx6_riot_spi_nor_device));
}

static struct imx_ssi_platform_data mx6_riot_ssi_pdata = {
	.flags = IMX_SSI_DMA | IMX_SSI_SYN,
};

static struct mxc_audio_platform_data mx6_riot_audio_data;

static int sab_sgtl5000_init(void)
{
        struct clk *clko;
        int rate;

	printk("----sab_sgtl5000_init\n");

        clko = clk_get(NULL, "clko_clk");
        if (IS_ERR(clko))
                pr_err("can't get CLKO clock.\n");

        rate = clk_round_rate(clko, 24000000);
        mx6_riot_audio_data.sysclk = rate;
        clk_set_rate(clko, rate);
        clk_enable(clko);

        printk("----enable clko_clk.\n");

	return 0;
}

static struct platform_device mx6_riot_audio_device = {
	.name = "imx-sgtl5000",
};

static struct mxc_audio_platform_data mx6_riot_audio_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 3,
        .init = sab_sgtl5000_init,
        .hp_gpio = -1,
};


static void mx6q_csi0_cam_powerdown(int powerdown)
{
        printk("----mx6_csi0_cam_powerdown %d\n", powerdown);

        if (powerdown)
                gpio_set_value(RIOT_CSI0_PWD, 1);
        else
                gpio_set_value(RIOT_CSI0_PWD, 0);

        msleep(2);
}

static void mx6q_csi0_io_init(void)
{
        printk("----mx6_csi0_io_init\n");

	mxc_iomux_v3_setup_multiple_pads(mx6dl_riot_csi0_sensor_pads,
                        ARRAY_SIZE(mx6dl_riot_csi0_sensor_pads));

        /* Camera power down */
        gpio_request(RIOT_CSI0_PWD, "cam-pwdn");
        gpio_direction_output(RIOT_CSI0_PWD, 1);
        msleep(1);
        gpio_set_value(RIOT_CSI0_PWD, 0);

        /* Camera reset */
        gpio_request(RIOT_CSI0_RST, "cam-reset");
        gpio_direction_output(RIOT_CSI0_RST, 1);

        gpio_set_value(RIOT_CSI0_RST, 0);
        msleep(10);
        gpio_set_value(RIOT_CSI0_RST, 1);

        /* For MX6Q:
         * GPR1 bit19 and bit20 meaning:
         * Bit19:       0 - Enable mipi to IPU1 CSI0
         *                      virtual channel is fixed to 0
         *              1 - Enable parallel interface to IPU1 CSI0
         * Bit20:       0 - Enable mipi to IPU2 CSI1
         *                      virtual channel is fixed to 3
         *              1 - Enable parallel interface to IPU2 CSI1
         * IPU1 CSI1 directly connect to mipi csi2,
         *      virtual channel is fixed to 1
         * IPU2 CSI0 directly connect to mipi csi2,
         *      virtual channel is fixed to 2
         *
         * For MX6DL:
         * GPR13 bit 0-2 IPU_CSI0_MUX
         *   000 MIPI_CSI0
         *   100 IPU CSI0
         */
        if (cpu_is_mx6q())
                mxc_iomux_set_gpr_register(1, 19, 1, 1);
        else if (cpu_is_mx6dl())
                mxc_iomux_set_gpr_register(13, 0, 3, 4);
}

static struct fsl_mxc_camera_platform_data camera_data = {
	.mclk = 24000000,
	.mclk_source = 0,
	.csi = 0,
	.io_init = mx6q_csi0_io_init,
	.pwdn = mx6q_csi0_cam_powerdown,
};

static void mx6q_mipi_powerdown(int powerdown)
{
	if (powerdown)
		gpio_set_value(RIOT_MIPICSI_PWN, 1);
	else
		gpio_set_value(RIOT_MIPICSI_PWN, 0);

	msleep(2);
}

static void mx6q_mipi_sensor_io_init(void)
{
	mxc_iomux_v3_setup_multiple_pads(mx6dl_riot_mipi_sensor_pads,
		ARRAY_SIZE(mx6dl_riot_mipi_sensor_pads));

	/* Camera reset */
	gpio_request(RIOT_MIPICSI_RST, "cam-reset");
	gpio_direction_output(RIOT_MIPICSI_RST, 1);

	/* Camera power down */
	gpio_request(RIOT_MIPICSI_PWN, "cam-pwdn");
	gpio_direction_output(RIOT_MIPICSI_PWN, 1);
	msleep(5);
	gpio_set_value(RIOT_MIPICSI_PWN, 0);
	msleep(5);
	gpio_set_value(RIOT_MIPICSI_RST, 0);
	msleep(1);
	gpio_set_value(RIOT_MIPICSI_RST, 1);
	msleep(5);
	gpio_set_value(RIOT_MIPICSI_PWN, 1);

	/*for mx6dl, mipi virtual channel 1 connect to csi 1*/
	if (cpu_is_mx6dl())
		mxc_iomux_set_gpr_register(13, 3, 3, 1);
}

static struct fsl_mxc_camera_platform_data mipi_csi2_data = {
	.mclk = 24000000,
	.mclk_source = 0,
	.csi = 1,
	.io_init = mx6q_mipi_sensor_io_init,
	.pwdn = mx6q_mipi_powerdown,
};

static struct imxi2c_platform_data mx6q_riot_i2c0_data = {
	.bitrate = 400000,
};

static struct imxi2c_platform_data mx6q_riot_i2c1_data = {
        .bitrate = 400000,
};

static struct imxi2c_platform_data mx6q_riot_i2c2_data = {
        .bitrate = 400000,
};

static struct imxi2c_platform_data mx6q_riot_i2c3_data = {
        .bitrate = 400000,
};

static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
        {
                I2C_BOARD_INFO("sgtl5000", 0x0a),
        },
};

static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("mxc_hdmi_i2c", 0x50),
	},
	{
		I2C_BOARD_INFO("ov5640_mipi", 0x3c),
		.platform_data = (void *)&mipi_csi2_data,
	},
};

static struct i2c_board_info mxc_i2c2_board_info[] __initdata = {
};

static struct i2c_board_info mxc_i2c3_board_info[] __initdata = {
        {
                I2C_BOARD_INFO("ov2656", 0x30),
                .platform_data = (void *)&camera_data,
        },
};

static void imx6q_riot_usbotg_vbus(bool on)
{
	if (on)
		gpio_set_value(RIOT_USB_OTG_PWR, 0);
	else
		gpio_set_value(RIOT_USB_OTG_PWR, 1);
}

static void imx6q_riot_host1_vbus(bool on)
{
}

static void __init imx6q_riot_init_usb(void)
{
	int ret = 0;

	imx_otg_base = MX6_IO_ADDRESS(MX6Q_USB_OTG_BASE_ADDR);
	/* disable external charger detect,
	 * or it will affect signal quality at dp .
	 */
	ret = gpio_request(RIOT_USB_OTG_PWR, "usb-pwr");
	if (ret) {
		pr_err("failed to get GPIO RIOT_USB_OTG_PWR: %d\n",
			ret);
		return;
	}
	gpio_direction_output(RIOT_USB_OTG_PWR, 1);

	mxc_iomux_set_gpr_register(1, 13, 1, 0);

	mx6_set_otghost_vbus_func(imx6q_riot_usbotg_vbus);
	mx6_set_host1_vbus_func(imx6q_riot_host1_vbus);

}

#if 0
static void mx6q_riot_flexcan0_switch(int enable)
{
	if (enable) {
		gpio_set_value(RIOT_CAN1_STBY, 1);
	} else {
		gpio_set_value(RIOT_CAN1_STBY, 0);
	}
}

static const struct flexcan_platform_data
	mx6q_riot_flexcan0_pdata __initconst = {
	.transceiver_switch = mx6q_riot_flexcan0_switch,
};
#endif

static struct viv_gpu_platform_data imx6q_gpu_pdata __initdata = {
	.reserved_mem_size = SZ_128M,
};

static struct imx_asrc_platform_data imx_asrc_data = {
	.channel_bits = 4,
	.clk_map_ver = 2,
};

static void mx6_reset_mipi_dsi(void)
{
}

static struct mipi_dsi_platform_data mipi_dsi_pdata = {
	.ipu_id		= 0,
	.disp_id	= 1,
	.lcd_panel	= "TRULY-WVGA",
	.reset		= mx6_reset_mipi_dsi,
};

static struct ipuv3_fb_platform_data riot_fb_data[] = {
        { /* fb0 */
        .disp_dev = "ldb",
        .interface_pix_fmt = IPU_PIX_FMT_RGB666,
        .mode_str = "LDB-XGA",
        .default_bpp = 32,
        .int_clk = false,
        }, {
        .disp_dev = "hdmi",
        .interface_pix_fmt = IPU_PIX_FMT_RGB24,
        .mode_str = "1920x1080M@60",
        .default_bpp = 32,
        .int_clk = false,
        },
};

static void hdmi_init(int ipu_id, int disp_id)
{
	int hdmi_mux_setting;

	if ((ipu_id > 1) || (ipu_id < 0)) {
		pr_err("Invalid IPU select for HDMI: %d. Set to 0\n", ipu_id);
		ipu_id = 0;
	}

	if ((disp_id > 1) || (disp_id < 0)) {
		pr_err("Invalid DI select for HDMI: %d. Set to 0\n", disp_id);
		disp_id = 0;
	}

	/* Configure the connection between IPU1/2 and HDMI */
	hdmi_mux_setting = 2*ipu_id + disp_id;

	/* GPR3, bits 2-3 = HDMI_MUX_CTL */
	mxc_iomux_set_gpr_register(3, 2, 2, hdmi_mux_setting);

	/* Set HDMI event as SDMA event2 while Chip version later than TO1.2 */
	if (hdmi_SDMA_check())
		mxc_iomux_set_gpr_register(0, 0, 1, 1);
}

/* On mx6x riot board i2c2 iomux with hdmi ddc,
 * the pins default work at i2c2 function,
 when hdcp enable, the pins should work at ddc function */

static void hdmi_enable_ddc_pin(void)
{
	mxc_iomux_v3_setup_multiple_pads(mx6dl_riot_hdmi_ddc_pads,
		ARRAY_SIZE(mx6dl_riot_hdmi_ddc_pads));
}

static void hdmi_disable_ddc_pin(void)
{
	mxc_iomux_v3_setup_multiple_pads(mx6dl_riot_i2c2_pads,
		ARRAY_SIZE(mx6dl_riot_i2c2_pads));
}

static struct fsl_mxc_hdmi_platform_data hdmi_data = {
	.init = hdmi_init,
	.enable_pins = hdmi_enable_ddc_pin,
	.disable_pins = hdmi_disable_ddc_pin,
};

static struct fsl_mxc_hdmi_core_platform_data hdmi_core_data = {
	.ipu_id = 0,
	.disp_id = 1,
};

static struct fsl_mxc_lcd_platform_data lcdif_data = {
	.ipu_id = 0,
	.disp_id = 0,
	.default_ifmt = IPU_PIX_FMT_RGB565,
};

static void ldb_init(void)
{
        int ret;

        printk("----ldb_init\n");

        ret = gpio_request(RIOT_LED_PWN, "led_pwn");
        if (ret) {
                pr_err("failed to get GPIO RIOT_LED_PWN: %d\n",
                        ret);
                return;
        }

        gpio_direction_output(RIOT_LED_PWN, 1);
}

static struct fsl_mxc_ldb_platform_data ldb_data = {
	.ipu_id = 0,
	.disp_id = 0,
	.ext_ref = 1,
	.mode = LDB_SEP0,
	.sec_ipu_id = 0,
	.sec_disp_id = 1,
};

static struct imx_ipuv3_platform_data ipu_data[] = {
	{
	.rev = 4,
	.csi_clk[0] = "clko_clk",
	.bypass_reset = false,
	}, {
	.rev = 4,
	.csi_clk[0] = "clko_clk",
	.bypass_reset = false,
	},
};

static struct fsl_mxc_capture_platform_data capture_data[] = {
	{
		.csi = 0,
		.ipu = 0,
		.mclk_source = 0,
		.is_mipi = 0,
	}, {
		.csi = 1,
		.ipu = 0,
		.mclk_source = 0,
		.is_mipi = 1,
	},
};


static void riot_suspend_enter(void)
{
	/* suspend preparation */
	/* Disable AUX 5V */
	gpio_set_value(RIOT_AUX_5V_EN, 0);
}

static void riot_suspend_exit(void)
{
	/* resume restore */
	/* Enable AUX 5V */
	gpio_set_value(RIOT_AUX_5V_EN, 1);
}
static const struct pm_platform_data mx6q_riot_pm_data __initconst = {
	.name = "imx_pm",
	.suspend_enter = riot_suspend_enter,
	.suspend_exit = riot_suspend_exit,
};

static struct regulator_consumer_supply riot_vmmc_consumers[] = {
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.1"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.2"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.3"),
};

static struct regulator_init_data riot_vmmc_init = {
	.num_consumer_supplies = ARRAY_SIZE(riot_vmmc_consumers),
	.consumer_supplies = riot_vmmc_consumers,
};

static struct fixed_voltage_config riot_vmmc_reg_config = {
	.supply_name		= "vmmc",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &riot_vmmc_init,
};

static struct platform_device riot_vmmc_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 3,
	.dev	= {
		.platform_data = &riot_vmmc_reg_config,
	},
};

#ifdef CONFIG_SND_SOC_SGTL5000

static struct regulator_consumer_supply sgtl5000_sabrelite_consumer_vdda = {
        .supply = "VDDA",
        .dev_name = "0-000a",
};

static struct regulator_consumer_supply sgtl5000_sabrelite_consumer_vddio = {
        .supply = "VDDIO",
        .dev_name = "0-000a",
};

static struct regulator_consumer_supply sgtl5000_sabrelite_consumer_vddd = {
        .supply = "VDDD",
        .dev_name = "0-000a",
};

static struct regulator_init_data sgtl5000_sabrelite_vdda_reg_initdata = {
        .num_consumer_supplies = 1,
        .consumer_supplies = &sgtl5000_sabrelite_consumer_vdda,
};

static struct regulator_init_data sgtl5000_sabrelite_vddio_reg_initdata = {
        .num_consumer_supplies = 1,
        .consumer_supplies = &sgtl5000_sabrelite_consumer_vddio,
};

static struct regulator_init_data sgtl5000_sabrelite_vddd_reg_initdata = {
        .num_consumer_supplies = 1,
        .consumer_supplies = &sgtl5000_sabrelite_consumer_vddd,
};

static struct fixed_voltage_config sgtl5000_sabrelite_vdda_reg_config = {
        .supply_name            = "VDDA",
        .microvolts             = 2500000,
        .gpio                   = -1,
        .init_data              = &sgtl5000_sabrelite_vdda_reg_initdata,
};

static struct fixed_voltage_config sgtl5000_sabrelite_vddio_reg_config = {
        .supply_name            = "VDDIO",
        .microvolts             = 3300000,
        .gpio                   = -1,
        .init_data              = &sgtl5000_sabrelite_vddio_reg_initdata,
};

static struct fixed_voltage_config sgtl5000_sabrelite_vddd_reg_config = {
        .supply_name            = "VDDD",
        .microvolts             = 0,
        .gpio                   = -1,
        .init_data              = &sgtl5000_sabrelite_vddd_reg_initdata,
};

static struct platform_device sgtl5000_sabrelite_vdda_reg_devices = {
        .name   = "reg-fixed-voltage",
        .id     = 0,
        .dev    = {
                .platform_data = &sgtl5000_sabrelite_vdda_reg_config,
        },
};

static struct platform_device sgtl5000_sabrelite_vddio_reg_devices = {
        .name   = "reg-fixed-voltage",
        .id     = 1,
        .dev    = {
                .platform_data = &sgtl5000_sabrelite_vddio_reg_config,
        },
};

static struct platform_device sgtl5000_sabrelite_vddd_reg_devices = {
        .name   = "reg-fixed-voltage",
        .id     = 2,
        .dev    = {
                .platform_data = &sgtl5000_sabrelite_vddd_reg_config,
        },
};

#endif /* CONFIG_SND_SOC_SGTL5000 */

static int __init imx6q_init_audio(void)
{
        mxc_register_device(&mx6_riot_audio_device,
                            &mx6_riot_audio_data);
        imx6q_add_imx_ssi(1, &mx6_riot_ssi_pdata);
#ifdef  CONFIG_SND_SOC_SGTL5000
        platform_device_register(&sgtl5000_sabrelite_vdda_reg_devices);
        platform_device_register(&sgtl5000_sabrelite_vddio_reg_devices);
        platform_device_register(&sgtl5000_sabrelite_vddd_reg_devices);
#endif

	return 0;
}

#if 0
#ifndef CONFIG_IMX_PCIE
static void pcie_3v3_power(void)
{
	/* disable PCIE_3V3 first */
	gpio_request(RIOT_PCIE_PWR_EN, "pcie_3v3_en");
	gpio_direction_output(RIOT_PCIE_PWR_EN, 0);
	mdelay(10);
	/* enable PCIE_3V3 again */
	gpio_set_value(RIOT_PCIE_PWR_EN, 1);
	gpio_free(RIOT_PCIE_PWR_EN);
}

static void pcie_3v3_reset(void)
{
	/* reset miniPCIe */
	gpio_request(RIOT_PCIE_RST_B_REVB, "pcie_reset_rebB");
	gpio_direction_output(RIOT_PCIE_RST_B_REVB, 0);
	/* The PCI Express Mini CEM specification states that PREST# is
	deasserted minimum 1ms after 3.3vVaux has been applied and stable*/
	mdelay(1);
	gpio_set_value(RIOT_PCIE_RST_B_REVB, 1);
	gpio_free(RIOT_PCIE_RST_B_REVB);
}
#endif
#endif

#if defined(CONFIG_LEDS_TRIGGER) || defined(CONFIG_LEDS_GPIO)
static struct gpio_led imx6q_gpio_leds[] = {
        {
                .name                   = "sys_led",
                .default_trigger        = "heartbeat",
                .gpio                   = RIOT_SYS_LED,
                .active_low             = true,
        },
        {
                .name                   = "user_led",
                .gpio                   = RIOT_USER_LED,
                .active_low             = true,
        },
};

static struct gpio_led_platform_data imx6q_gpio_leds_data = {
	.leds		= imx6q_gpio_leds,
	.num_leds	= ARRAY_SIZE(imx6q_gpio_leds),
};

static struct platform_device imx6q_gpio_led_device = {
	.name		= "leds-gpio",
	.id		= -1,
	.num_resources  = 0,
	.dev		= {
		.platform_data = &imx6q_gpio_leds_data,
	}
};

static void __init imx6q_add_device_gpio_leds(void)
{
	platform_device_register(&imx6q_gpio_led_device);
}
#else
static void __init imx6q_add_device_gpio_leds(void) {}
#endif

#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
#define GPIO_BUTTON(gpio_num, ev_code, act_low, descr, wake, debounce)	\
{								\
	.gpio		= gpio_num,				\
	.type		= EV_KEY,				\
	.code		= ev_code,				\
	.active_low	= act_low,				\
	.desc		= "btn " descr,				\
	.wakeup		= wake,					\
	.debounce_interval = debounce,				\
}

static struct gpio_keys_button imx6q_buttons[] = {
//	GPIO_BUTTON(RIOT_VOLUME_UP, KEY_VOLUMEUP, 1, "volume-up", 0, 1),
	GPIO_BUTTON(RIOT_VOLUME_DN, KEY_VOLUMEDOWN, 1, "volume-down", 0, 1),
	GPIO_BUTTON(RIOT_POWER_OFF, KEY_POWER, 1, "power", 1, 1),
};

static struct gpio_keys_platform_data imx6q_button_data = {
	.buttons	= imx6q_buttons,
	.nbuttons	= ARRAY_SIZE(imx6q_buttons),
};

static struct platform_device imx6q_button_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.num_resources  = 0,
	.dev		= {
		.platform_data = &imx6q_button_data,
	}
};

static void __init imx6q_add_device_buttons(void)
{
	platform_device_register(&imx6q_button_device);
}
#else
static void __init imx6q_add_device_buttons(void) {}
#endif

/* Backlight PWM for lvds*/
static struct platform_pwm_backlight_data mx6_riot_pwm_backlight_data4 = {
        .pwm_id                 = 3,
        .max_brightness         = 255,
        .dft_brightness         = 128,
        .pwm_period_ns          = 50000,
};

static struct mxc_dvfs_platform_data riot_dvfscore_data = {
	.reg_id = "VDDCORE",
	.soc_id	= "VDDSOC",
	.clk1_id = "cpu_clk",
	.clk2_id = "gpc_dvfs_clk",
	.gpc_cntr_offset = MXC_GPC_CNTR_OFFSET,
	.ccm_cdcr_offset = MXC_CCM_CDCR_OFFSET,
	.ccm_cacrr_offset = MXC_CCM_CACRR_OFFSET,
	.ccm_cdhipr_offset = MXC_CCM_CDHIPR_OFFSET,
	.prediv_mask = 0x1F800,
	.prediv_offset = 11,
	.prediv_val = 3,
	.div3ck_mask = 0xE0000000,
	.div3ck_offset = 29,
	.div3ck_val = 2,
	.emac_val = 0x08,
	.upthr_val = 25,
	.dnthr_val = 9,
	.pncthr_val = 33,
	.upcnt_val = 10,
	.dncnt_val = 10,
	.delay_time = 80,
};

static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
}

static struct mipi_csi2_platform_data mipi_csi2_pdata = {
	.ipu_id	 = 0,
	.csi_id = 1,
	.v_channel = 0,
	.lanes = 2,
	.dphy_clk = "mipi_pllref_clk",
	.pixel_clk = "emi_clk",
};

static int __init caam_setup(char *__unused)
{
	caam_enabled = 1;
	return 1;
}
early_param("caam", caam_setup);

#define SNVS_LPCR 0x38
static void mx6_snvs_poweroff(void)
{

	void __iomem *mx6_snvs_base =  MX6_IO_ADDRESS(MX6Q_SNVS_BASE_ADDR);
	u32 value;
	value = readl(mx6_snvs_base + SNVS_LPCR);
	/*set TOP and DP_EN bit*/
	writel(value | 0x60, mx6_snvs_base + SNVS_LPCR);
}

#if 0
static const struct imx_pcie_platform_data mx6_riot_pcie_data __initconst = {
	.pcie_pwr_en	= RIOT_PCIE_PWR_EN,
	.pcie_rst	= RIOT_PCIE_RST_B_REVB,
	.pcie_wake_up	= RIOT_PCIE_WAKE_B,
	.pcie_dis	= RIOT_PCIE_DIS_B,
};
#endif

/*!
 * Board specific initialization.
 */
static void __init mx6_riot_board_init(void)
{
	int i;
	int ret;
	struct clk *clko, *clko2;
	struct clk *new_parent;
	int rate;

	mxc_iomux_v3_setup_multiple_pads(mx6dl_riot_pads,
		ARRAY_SIZE(mx6dl_riot_pads));

#ifdef CONFIG_FEC_1588
	/* Set GPIO_16 input for IEEE-1588 ts_clk and RMII reference clock
	 * For MX6 GPR1 bit21 meaning:
	 * Bit21:       0 - GPIO_16 pad output
	 *              1 - GPIO_16 pad input
	 */
	 mxc_iomux_set_gpr_register(1, 21, 1, 1);
#endif

	gp_reg_id = riot_dvfscore_data.reg_id;
	soc_reg_id = riot_dvfscore_data.soc_id;
	mx6q_riot_init_uart();

	imx6q_add_mxc_hdmi_core(&hdmi_core_data);

	imx6q_add_ipuv3(0, &ipu_data[0]);
	for (i = 0; i < 2 && i < ARRAY_SIZE(riot_fb_data); i++)
		imx6q_add_ipuv3fb(i, &riot_fb_data[i]);

	imx6q_add_vdoa();
	imx6q_add_mipi_dsi(&mipi_dsi_pdata);
	imx6q_add_lcdif(&lcdif_data);
	imx6q_add_ldb(&ldb_data);
	imx6q_add_v4l2_output(0);
	imx6q_add_v4l2_capture(0, &capture_data[0]);
	imx6q_add_v4l2_capture(1, &capture_data[1]);
	imx6q_add_mipi_csi2(&mipi_csi2_pdata);
	imx6q_add_imx_snvs_rtc();

	if (1 == caam_enabled)
		imx6q_add_imx_caam();

	imx6q_add_device_gpio_leds();

	ldb_init();

	imx6q_add_imx_i2c(0, &mx6q_riot_i2c0_data);
	imx6q_add_imx_i2c(1, &mx6q_riot_i2c1_data);
	imx6q_add_imx_i2c(2, &mx6q_riot_i2c2_data);
	imx6q_add_imx_i2c(3, &mx6q_riot_i2c3_data);
	i2c_register_board_info(0, mxc_i2c0_board_info,
			ARRAY_SIZE(mxc_i2c0_board_info));
	i2c_register_board_info(1, mxc_i2c1_board_info,
			ARRAY_SIZE(mxc_i2c1_board_info));
	i2c_register_board_info(2, mxc_i2c2_board_info,
			ARRAY_SIZE(mxc_i2c2_board_info));
        i2c_register_board_info(3, mxc_i2c3_board_info,
                        ARRAY_SIZE(mxc_i2c3_board_info));

	ret = gpio_request(RIOT_PFUZE_INT, "pFUZE-int");
	if (ret) {
		printk(KERN_ERR"request pFUZE-int error!!\n");
		return;
	} else {
		gpio_direction_input(RIOT_PFUZE_INT);
		mx6q_riot_init_pfuze100(RIOT_PFUZE_INT);
	}

	/* SPI */
	imx6q_add_ecspi(0, &mx6q_riot_spi_data);
	spi_device_init();

	imx6q_add_mxc_hdmi(&hdmi_data);

	imx6q_add_anatop_thermal_imx(1, &mx6q_riot_anatop_thermal_data);
	imx6_init_fec(fec_data);
#ifdef CONFIG_MX6_ENET_IRQ_TO_GPIO
	/* Make sure the IOMUX_OBSRV_MUX1 is set to ENET_IRQ. */
	mxc_iomux_set_specialbits_register(IOMUX_OBSRV_MUX1_OFFSET,
		OBSRV_MUX1_ENET_IRQ, OBSRV_MUX1_MASK);
#endif

	imx6q_add_pm_imx(0, &mx6q_riot_pm_data);

	/* Move sd4 to first because sd4 connect to emmc.
	   Mfgtools want emmc is mmcblk0 and other sd card is mmcblk1.
	*/
	imx6q_add_sdhci_usdhc_imx(3, &mx6q_riot_sd4_data);
	imx6q_add_sdhci_usdhc_imx(1, &mx6q_riot_sd2_data);
	imx6q_add_sdhci_usdhc_imx(2, &mx6q_riot_sd3_data);
	imx_add_viv_gpu(&imx6_gpu_data, &imx6q_gpu_pdata);
	imx6q_riot_init_usb();

	imx6q_add_vpu();
	imx6q_init_audio();
	platform_device_register(&riot_vmmc_reg_devices);
	imx_asrc_data.asrc_core_clk = clk_get(NULL, "asrc_clk");
	imx_asrc_data.asrc_audio_clk = clk_get(NULL, "asrc_serial_clk");
	imx6q_add_asrc(&imx_asrc_data);

	imx6q_add_mxc_pwm(0);
	imx6q_add_mxc_pwm(1);
	imx6q_add_mxc_pwm(2);
	imx6q_add_mxc_pwm(3);
	imx6q_add_mxc_pwm_backlight(3, &mx6_riot_pwm_backlight_data4);

	imx6q_add_otp();
	imx6q_add_viim();
	imx6q_add_imx2_wdt(0, NULL);
	imx6q_add_dma();

	imx6q_add_dvfs_core(&riot_dvfscore_data);
//	imx6q_add_device_buttons();

	imx6q_add_hdmi_soc();
	imx6q_add_hdmi_soc_dai();

	if (cpu_is_mx6dl()) {
		imx6dl_add_imx_pxp();
		imx6dl_add_imx_pxp_client();
	}
	/*
	ret = gpio_request_array(mx6q_riot_flexcan_gpios,
			ARRAY_SIZE(mx6q_riot_flexcan_gpios));
	if (ret)
		pr_err("failed to request flexcan1-gpios: %d\n", ret);
	else
		imx6q_add_flexcan0(&mx6q_riot_flexcan0_pdata);
	*/

	clko2 = clk_get(NULL, "clko2_clk");
	if (IS_ERR(clko2))
		pr_err("can't get CLKO2 clock.\n");

	new_parent = clk_get(NULL, "osc_clk");
	if (!IS_ERR(new_parent)) {
		clk_set_parent(clko2, new_parent);
		clk_put(new_parent);
	}
	rate = clk_round_rate(clko2, 24000000);
	clk_set_rate(clko2, rate);
	clk_enable(clko2);

	printk("----enable clko2\n");

	/* Camera and audio use osc clock */
	clko = clk_get(NULL, "clko_clk");
	if (!IS_ERR(clko))
		clk_set_parent(clko, clko2);

#if 0
#ifndef CONFIG_IMX_PCIE
	/* enable pcie 3v3 power without pcie driver */
	pcie_3v3_power();
	mdelay(10);
	pcie_3v3_reset();
#endif
#endif
	/* Register charger chips */
//	platform_device_register(&riot_max8903_charger_1);
	pm_power_off = mx6_snvs_poweroff;
	imx6q_add_busfreq();

	/* Add PCIe RC interface support */
//	imx6q_add_pcie(&mx6_riot_pcie_data);

	imx6_add_armpmu();
	imx6q_add_perfmon(0);
	imx6q_add_perfmon(1);
	imx6q_add_perfmon(2);
}

extern void __iomem *twd_base;
static void __init mx6_riot_timer_init(void)
{
	struct clk *uart_clk;
#ifdef CONFIG_LOCAL_TIMERS
	twd_base = ioremap(LOCAL_TWD_ADDR, SZ_256);
	BUG_ON(!twd_base);
#endif
	mx6_clocks_init(32768, 24000000, 0, 0);

	uart_clk = clk_get_sys("imx-uart.0", NULL);
	early_console_setup(UART2_BASE_ADDR, uart_clk);
}

static struct sys_timer mx6_riot_timer = {
	.init   = mx6_riot_timer_init,
};

static void __init mx6q_riot_reserve(void)
{
#if defined(CONFIG_MXC_GPU_VIV) || defined(CONFIG_MXC_GPU_VIV_MODULE)
	phys_addr_t phys;

	if (imx6q_gpu_pdata.reserved_mem_size) {
		phys = memblock_alloc_base(imx6q_gpu_pdata.reserved_mem_size,
					   SZ_4K, SZ_1G);
		memblock_remove(phys, imx6q_gpu_pdata.reserved_mem_size);
		imx6q_gpu_pdata.reserved_mem_base = phys;
	}
#endif
}

/*
 * initialize __mach_desc_MX6Q_RIOT data structure.
 */
MACHINE_START(MX6Q_RIOT, "Freescale i.MX 6Solo RIOT Board")
	/* Maintainer: Freescale Semiconductor, Inc. */
	.boot_params = MX6_PHYS_OFFSET + 0x100,
	.fixup = fixup_mxc_board,
	.map_io = mx6_map_io,
	.init_irq = mx6_init_irq,
	.init_machine = mx6_riot_board_init,
	.timer = &mx6_riot_timer,
	.reserve = mx6q_riot_reserve,
MACHINE_END
