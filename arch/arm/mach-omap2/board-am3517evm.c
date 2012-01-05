/*
 * linux/arch/arm/mach-omap2/board-am3517evm.c
 *
 * Copyright (C) 2009 Texas Instruments Incorporated
 * Author: Ranjith Lohithakshan <ranjithl@ti.com>
 *
 * Based on mach-omap2/board-omap3evm.c
 * Modified: Daren Yeo <dyeo@mtiemail.com> for custom AM3517 board
 * with support for WLAN-device reset based on board-zoom3.c
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as  published by the
 * Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/i2c/tsc2004.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/input.h>
#include <linux/tca6416_keypad.h>
#include <linux/davinci_emac.h>
#include <linux/i2c/pca953x.h>
#include <linux/regulator/machine.h>
#include <linux/can/platform/ti_hecc.h>
#include <linux/delay.h>

#include <mach/hardware.h>
#include <mach/am35xx.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/control.h>
#include <plat/usb.h>
#include <plat/display.h>
#include <plat/gpmc.h>
#include <plat/nand.h>
#ifdef CONFIG_MMC_EMBEDDED_SDIO
#include <plat/wifi_tiwlan.h>
#endif

#include <linux/spi/spi.h>
#include <plat/mcspi.h>



#include <media/tvp514x.h>
#include <media/ti-media/vpfe_capture.h>

#include "mmc-am3517evm.h"
#include "mux.h"

#include "mux_NDU.h"		// DCY
#include "pinmux_NDU.h"		// DCY

#include <linux/i2c-gpio.h>	// DCY

static struct omap2_mcspi_device_config lcd_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel	= 1,	/* 0: slave, 1: master */
};

// DCY
static struct omap2_mcspi_device_config ad7606_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel	= 1,	/* 0: slave, 1: master */
};



struct spi_board_info ndu_spi_board_info[] = {
	[0] = {
		.modalias		= "spidev",
		.bus_num		= 1,
		.chip_select		= 0,
		.max_speed_hz		= 1500000,
		.controller_data	= &lcd_mcspi_config,
	},
	[1] = {
		.modalias		= "spidev",
		.bus_num		= 2,
		.chip_select		= 0,
		.max_speed_hz		= 1500000,
		.controller_data	= &ad7606_mcspi_config,
	},

};


#define GPMC_CS0_BASE  0x60
#define GPMC_CS_SIZE   0x30

#define NAND_BLOCK_SIZE        SZ_128K

static struct mtd_partition am3517evm_nand_partitions[] = {
/* All the partition sizes are listed in terms of NAND block size */
{
	   .name           = "xloader-nand",
	   .offset         = 0,
	   .size           = 4*(SZ_128K),
	   .mask_flags     = MTD_WRITEABLE
},
{
	   .name           = "uboot-nand",
	   .offset         = MTDPART_OFS_APPEND,
	   .size           = 14*(SZ_128K),
	   .mask_flags     = MTD_WRITEABLE
},
{
	   .name           = "params-nand",
	   .offset         = MTDPART_OFS_APPEND,
	   .size           = 2*(SZ_128K)
},
{
	   .name           = "linux-nand",
	   .offset         = MTDPART_OFS_APPEND,
	   .size           = 40*(SZ_128K)
},
{
	   .name           = "jffs2-nand",
	   .size           = MTDPART_SIZ_FULL,
	   .offset         = MTDPART_OFS_APPEND,
},
};

static struct omap_nand_platform_data am3517evm_nand_data = {
	   .parts          = am3517evm_nand_partitions,
	   .nr_parts       = ARRAY_SIZE(am3517evm_nand_partitions),
	   .nand_setup     = NULL,
	   .dma_channel    = -1,           /* disable DMA in OMAP NAND driver */
	   .dev_ready      = NULL,
};

static struct resource am3517evm_nand_resource = {
	   .flags          = IORESOURCE_MEM,
};

static struct platform_device am3517evm_nand_device = {
	   .name           = "omap2-nand",
	   .id             = 0,
	   .dev            = {
					   .platform_data  = &am3517evm_nand_data,
	   },
	   .num_resources  = 1,
	   .resource       = &am3517evm_nand_resource,
};

void __init am3517evm_flash_init(void)
{
	   u8 cs = 0;
	   u8 nandcs = GPMC_CS_NUM + 1;
	   u32 gpmc_base_add = OMAP34XX_GPMC_VIRT;

	   while (cs < GPMC_CS_NUM) {
			   u32 ret = 0;
			   ret = gpmc_cs_read_reg(cs, GPMC_CS_CONFIG1);

			   if ((ret & 0xC00) == 0x800) {
					   /* Found it!! */
					   if (nandcs > GPMC_CS_NUM)
							   nandcs = cs;
			   }
			   cs++;
	   }
	   if (nandcs > GPMC_CS_NUM) {
			   printk(KERN_INFO "NAND: Unable to find configuration "
					   " in GPMC\n ");
			   return;
	   }

	   if (nandcs < GPMC_CS_NUM) {
			   am3517evm_nand_data.cs   = nandcs;
			   am3517evm_nand_data.gpmc_cs_baseaddr = (void *)(gpmc_base_add +
									   GPMC_CS0_BASE + nandcs*GPMC_CS_SIZE);
			   am3517evm_nand_data.gpmc_baseaddr   = (void *) (gpmc_base_add);

			   if (platform_device_register(&am3517evm_nand_device) < 0)
					   printk(KERN_ERR "Unable to register NAND device\n");

	   }
}



#define AM35XX_EVM_PHY_MASK		(0xF)
#define AM35XX_EVM_MDIO_FREQUENCY    	(1000000)

static struct emac_platform_data am3517_evm_emac_pdata = {
	.phy_mask       = AM35XX_EVM_PHY_MASK,
	.mdio_max_freq  = AM35XX_EVM_MDIO_FREQUENCY,
	.rmii_en        = 1,
};

static int __init eth_addr_setup(char *str)
{
	int i;

	if(str == NULL)
		return 0;
	for(i = 0; i <  ETH_ALEN; i++)
		am3517_evm_emac_pdata.mac_addr[i] = simple_strtol(&str[i*3],
							(char **)NULL, 16);
	return 1;
}

/* Get MAC address from kernel boot parameter eth=AA:BB:CC:DD:EE:FF */
__setup("eth=", eth_addr_setup);

static struct resource am3517_emac_resources[] = {
	{
		.start  = AM35XX_IPSS_EMAC_BASE,
		.end    = AM35XX_IPSS_EMAC_BASE + 0x3FFFF,
		.flags  = IORESOURCE_MEM,
	},
	{
		.start  = INT_35XX_EMAC_C0_RXTHRESH_IRQ,
		.end    = INT_35XX_EMAC_C0_RXTHRESH_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.start  = INT_35XX_EMAC_C0_RX_PULSE_IRQ,
		.end    = INT_35XX_EMAC_C0_RX_PULSE_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.start  = INT_35XX_EMAC_C0_TX_PULSE_IRQ,
		.end    = INT_35XX_EMAC_C0_TX_PULSE_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.start  = INT_35XX_EMAC_C0_MISC_PULSE_IRQ,
		.end    = INT_35XX_EMAC_C0_MISC_PULSE_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device am3517_emac_device = {
	.name           = "davinci_emac",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(am3517_emac_resources),
	.resource       = am3517_emac_resources,
};




static void am3517_enable_ethernet_int(void)
{
	u32 regval;

	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = (regval | AM35XX_CPGMAC_C0_RX_PULSE_CLR |
		AM35XX_CPGMAC_C0_TX_PULSE_CLR | AM35XX_CPGMAC_C0_MISC_PULSE_CLR |
		AM35XX_CPGMAC_C0_RX_THRESH_CLR );
	omap_ctrl_writel(regval,AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
}

static void am3517_disable_ethernet_int(void)
{
	u32 regval;

	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = (regval | AM35XX_CPGMAC_C0_RX_PULSE_CLR |
		AM35XX_CPGMAC_C0_TX_PULSE_CLR);
	omap_ctrl_writel(regval,AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
}

void am3517_evm_ethernet_init(struct emac_platform_data *pdata)
{
	unsigned int regval;

	pdata->ctrl_reg_offset          = AM35XX_EMAC_CNTRL_OFFSET;
	pdata->ctrl_mod_reg_offset      = AM35XX_EMAC_CNTRL_MOD_OFFSET;
	pdata->ctrl_ram_offset          = AM35XX_EMAC_CNTRL_RAM_OFFSET;
	pdata->mdio_reg_offset          = AM35XX_EMAC_MDIO_OFFSET;
	pdata->ctrl_ram_size            = AM35XX_EMAC_CNTRL_RAM_SIZE;
	pdata->version                  = EMAC_VERSION_2;
	pdata->hw_ram_addr              = AM35XX_EMAC_HW_RAM_ADDR;
	pdata->interrupt_enable 	= am3517_enable_ethernet_int;
	pdata->interrupt_disable 	= am3517_disable_ethernet_int;
	am3517_emac_device.dev.platform_data     = pdata;
	platform_device_register(&am3517_emac_device);

	regval = omap_ctrl_readl(AM35XX_CONTROL_IP_SW_RESET);
	regval = regval & (~(AM35XX_CPGMACSS_SW_RST));
	omap_ctrl_writel(regval,AM35XX_CONTROL_IP_SW_RESET);
	regval = omap_ctrl_readl(AM35XX_CONTROL_IP_SW_RESET);

	return ;
}



/*
 * RTC - S35390A
 */
#define	GPIO_RTCS35390A_IRQ	55
static struct i2c_board_info __initdata am3517evm_i2c1_boardinfo[] =
{
	{
		I2C_BOARD_INFO("s35390a", 0x30),
		.type		= "s35390a",
	},
};
static struct i2c_board_info __initdata am3517evm_i2c3_boardinfo[] =
{
	{
		I2C_BOARD_INFO("LTC1760", 0x14),
	},
};


static int __init am3517_evm_i2c_init(void)
{
	omap_register_i2c_bus(1, 400, NULL, 0);
	omap_register_i2c_bus(2, 400, NULL, 0);
	omap_register_i2c_bus(3, 100, NULL, 0);	// DCY - 100 for slower Battery / Charger
	return 0;
}

#ifdef CONFIG_WLAN_1273
void wlan_1273_reset()
{
	/* Pulse the WLAN_EN and BT_EN pins high, then low to place in low-power mode */
	if (gpio_request(OMAP_AM3517EVM_WIFI_PMENA_GPIO, "WLAN_EN") != 0)
		pr_err("GPIO %i request failed\n", OMAP_AM3517EVM_WIFI_PMENA_GPIO);
	gpio_direction_output(OMAP_AM3517EVM_WIFI_PMENA_GPIO, 1);

	if (gpio_request(OMAP_AM3517EVM_BT_EN_GPIO, "BT_EN") != 0)
		pr_err("GPIO %i request failed\n", OMAP_AM3517EVM_BT_EN_GPIO);
	gpio_direction_output(OMAP_AM3517EVM_BT_EN_GPIO, 1);

	if (gpio_request(OMAP_AM3517EVM_BT_WAKEUP_GPIO, "BT_WAKEUP") != 0)
		pr_err("GPIO %i request failed\n", OMAP_AM3517EVM_BT_WAKEUP_GPIO);
	gpio_direction_output(OMAP_AM3517EVM_BT_WAKEUP_GPIO, 1);

	if (gpio_request(OMAP_AM3517EVM_BT_HOST_WAKE_GPIO, "BT_HOST_WAKE") != 0)
		pr_err("GPIO %i request failed\n", OMAP_AM3517EVM_BT_HOST_WAKE_GPIO);
	gpio_direction_input(OMAP_AM3517EVM_BT_HOST_WAKE_GPIO);

	if (gpio_request(OMAP_AM3517EVM_FM_EN_GPIO, "FM_EN") != 0)
		pr_err("GPIO %i request failed\n", OMAP_AM3517EVM_FM_EN_GPIO);
	gpio_direction_output(OMAP_AM3517EVM_FM_EN_GPIO, 1);

	if (gpio_request(OMAP_AM3517EVM_WIFI_LVLSHFT_GPIO, "LVLSHFT_OE") != 0)
		pr_err("GPIO %i request failed\n", OMAP_AM3517EVM_WIFI_LVLSHFT_GPIO);
	gpio_direction_output(OMAP_AM3517EVM_WIFI_LVLSHFT_GPIO, 0);

	printk("sleeping\n");
	msleep(10);

	gpio_set_value( OMAP_AM3517EVM_WIFI_LVLSHFT_GPIO, 0 );
	gpio_set_value( OMAP_AM3517EVM_WIFI_PMENA_GPIO, 0 );
	gpio_set_value( OMAP_AM3517EVM_BT_EN_GPIO, 0 );
	gpio_set_value( OMAP_AM3517EVM_FM_EN_GPIO, 0 );

	msleep(1);

	gpio_set_value( OMAP_AM3517EVM_WIFI_PMENA_GPIO, 1 );
	gpio_set_value( OMAP_AM3517EVM_BT_WAKEUP_GPIO, 1 );
	msleep(10);
	gpio_set_value( OMAP_AM3517EVM_BT_EN_GPIO, 1 );

	/* export the BT pins */
	gpio_export(OMAP_AM3517EVM_BT_EN_GPIO, 0);
	gpio_export(OMAP_AM3517EVM_BT_WAKEUP_GPIO, 0);
	gpio_export(OMAP_AM3517EVM_BT_HOST_WAKE_GPIO, 0);



	return;
}
#else
#define wlan_1273_reset() do { } while (0)
#endif


/*
 * Board initialization
 */
static struct omap_board_config_kernel am3517_evm_config[] __initdata = {
};

static struct platform_device *am3517_evm_devices[] __initdata = {
};

static void __init am3517_evm_init_irq(void)
{
	omap_board_config = am3517_evm_config;
	omap_board_config_size = ARRAY_SIZE(am3517_evm_config);

	omap2_init_common_hw(NULL, NULL, NULL, NULL, NULL);
	omap_init_irq();
	omap_gpio_init();
}

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata =
{
	MUX_AM3517EVM()		// DCY   from pinmux_NDU.h, generated with PinMux Utility
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux	NULL
#endif

static struct am3517_hsmmc_info mmc[] = {
	{
		.mmc            = 1,
		.wires          = 4,
		.gpio_cd        = 127,
		.gpio_wp        = 126,
	},
	{
		.mmc            = 2,
		.wires          = 4,
		.gpio_cd        = -EINVAL,
		.gpio_wp        = -EINVAL,
	},
	{}      /* Terminator */
};

static void wlan_mux_init()
{
	omap_mux_init_signal("sdmmc2_clk", OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc2_cmd", OMAP_PIN_INPUT_PULLUP);	
	omap_mux_init_signal("sdmmc2_dat0", OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc2_dat1", OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc2_dat2", OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc2_dat3", OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc2_dat4.sdmmc2_dir_dat0", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("sdmmc2_dat5.sdmmc2_dir_dat1", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("sys_boot4.sdmmc2_dir_dat2", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("sys_boot5.sdmmc2_dir_dat3", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("sdmmc2_dat6.sdmmc2_dir_cmd", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("sdmmc2_dat7.sdmmc2_clkin", OMAP_PIN_INPUT);

	omap_mux_init_gpio(OMAP_AM3517EVM_WIFI_PMENA_GPIO, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(OMAP_AM3517EVM_WIFI_IRQ_GPIO, OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_gpio(OMAP_AM3517EVM_WIFI_LVLSHFT_GPIO, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(OMAP_AM3517EVM_BT_EN_GPIO, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(OMAP_AM3517EVM_BT_WAKEUP_GPIO, OMAP_PIN_OUTPUT); 
	omap_mux_init_gpio(OMAP_AM3517EVM_BT_HOST_WAKE_GPIO, OMAP_PIN_INPUT);
	omap_mux_init_gpio(OMAP_AM3517EVM_FM_EN_GPIO, OMAP_PIN_OUTPUT);

	omap_mux_init_signal("uart2_cts.uart2_cts", OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("uart2_rts.uart2_rts", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("uart2_tx.uart2_tx", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("uart2_rx.uart2_rx", OMAP_PIN_INPUT);

	omap_mux_init_signal("mcbsp3_fsx.mcbsp3_fsx", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("mcbsp3_clkx.mcbsp3_clkx", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("mcbsp3_dr.mcbsp3_dr", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("mcbsp3_dx.mcbsp3_dx", OMAP_PIN_OUTPUT);


}

static void __init am3517_evm_init(void)
{

	am3517_evm_i2c_init();

printk("Setup Board Mux (table size=%d)\n", sizeof(board_mux));	// DCY
	omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);
	platform_add_devices(am3517_evm_devices,
				ARRAY_SIZE(am3517_evm_devices));

#ifdef CONFIG_WLAN_1273
	wlan_1273_reset();
#endif
	omap_serial_init();
	am3517evm_flash_init();
	usb_musb_init();

	// SPI Bus Info
	spi_register_board_info(ndu_spi_board_info,
				ARRAY_SIZE(ndu_spi_board_info));

	/* RTC - S35390A */
	omap_mux_init_gpio(55, OMAP_PIN_INPUT_PULLUP);
	if (gpio_request(GPIO_RTCS35390A_IRQ, "rtcs35390a-irq") < 0)
		printk(KERN_WARNING "failed to request GPIO#%d\n",
				GPIO_RTCS35390A_IRQ);
	if (gpio_direction_input(GPIO_RTCS35390A_IRQ))
		printk(KERN_WARNING "GPIO#%d cannot be configured as "
				"input\n", GPIO_RTCS35390A_IRQ);
	am3517evm_i2c1_boardinfo[1].irq = gpio_to_irq(GPIO_RTCS35390A_IRQ);

	i2c_register_board_info(1, am3517evm_i2c1_boardinfo,
				ARRAY_SIZE(am3517evm_i2c1_boardinfo));

	/* Battery Charger - LTC1760 on I2C-3*/
	i2c_register_board_info(3, am3517evm_i2c3_boardinfo, ARRAY_SIZE(am3517evm_i2c3_boardinfo));

	/*Ethernet*/
	am3517_evm_ethernet_init(&am3517_evm_emac_pdata);

	wlan_mux_init();

	/* MMC init function */
	am3517_mmc_init(mmc);

}

static void __init am3517_evm_map_io(void)
{
	omap2_set_globals_343x();
	omap2_map_common_io();
}

MACHINE_START(OMAP3517EVM, "OMAP3517/AM3517 EVM")
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xd8000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= am3517_evm_map_io,
	.init_irq	= am3517_evm_init_irq,
	.init_machine	= am3517_evm_init,
	.timer		= &omap_timer,
MACHINE_END
