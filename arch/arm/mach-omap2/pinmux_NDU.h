/*
* (C) Copyright 2012
* Texas Instruments, <www.ti.com>
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License as
* published by the Free Software Foundation; either version 2 of
* the License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place, Suite 330, Boston,
* MA 02111-1307 USA
*/

#ifndef _PINMUX_H_
#define _PINMUX_H_

/*
 * M0 - Mux mode 0
 * M1 - Mux mode 1
 * M2 - Mux mode 2
 * M3 - Mux mode 3
 * M4 - Mux mode 4
 * M5 - Mux mode 5
 * M6 - Mux mode 6
 * M7 - Mux mode 7
 * IDIS - Input disabled
 * IEN - Input enabled
 * PD - Active-mode pull-down enabled
 * PU - Active-mode pull-up enabled
 * PI - Active-mode pull inhibited
 */

#define MUX_AM3517EVM() \
MUX_VAL(CONTROL_PADCONF_CCDC_DATA0, (IEN | PD | M0 )) /* ccdc_data0 */\
MUX_VAL(CONTROL_PADCONF_CCDC_DATA1, (IEN | PD | M0 )) /* ccdc_data1 */\
MUX_VAL(CONTROL_PADCONF_CCDC_DATA2, (IEN | PD | M0 )) /* ccdc_data2 */\
MUX_VAL(CONTROL_PADCONF_CCDC_DATA3, (IDIS | PD | M4 )) /* gpio_102 */\
MUX_VAL(CONTROL_PADCONF_CCDC_DATA4, (IDIS | PD | M4 )) /* gpio_103 */\
MUX_VAL(CONTROL_PADCONF_CCDC_DATA5, (IDIS | PD | M4 )) /* gpio_104 */\
MUX_VAL(CONTROL_PADCONF_CCDC_DATA6, (IEN | PD | M0 )) /* ccdc_data6 */\
MUX_VAL(CONTROL_PADCONF_CCDC_DATA7, (IEN | PD | M0 )) /* ccdc_data7 */\
MUX_VAL(CONTROL_PADCONF_CCDC_FIELD, (IDIS | PD | M2 )) /* uart4_tx */\
MUX_VAL(CONTROL_PADCONF_CCDC_HD, (IDIS | PD | M4 )) /* gpio_96 */\
MUX_VAL(CONTROL_PADCONF_CCDC_PCLK, (IDIS | PD | M4 )) /* gpio_94 */\
MUX_VAL(CONTROL_PADCONF_CCDC_VD, (IDIS | PD | M4 )) /* gpio_97 */\
MUX_VAL(CONTROL_PADCONF_CCDC_WEN, (IEN | PD | M2 )) /* uart4_rx */\
MUX_VAL(CONTROL_PADCONF_DSS_ACBIAS, (IDIS | PD | M0 )) /* dss_acbias */\
MUX_VAL(CONTROL_PADCONF_DSS_DATA0, (IDIS | PI | M4 )) /* gpio_70 */\
MUX_VAL(CONTROL_PADCONF_DSS_DATA1, (IEN | PD | M4 )) /* gpio_71 */\
MUX_VAL(CONTROL_PADCONF_DSS_DATA2, (IEN | PD | M4 )) /* gpio_72 */\
MUX_VAL(CONTROL_PADCONF_DSS_DATA3, (IEN | PD | M4 )) /* gpio_73 */\
MUX_VAL(CONTROL_PADCONF_DSS_DATA4, (IEN | PD | M4 )) /* gpio_74 */\
MUX_VAL(CONTROL_PADCONF_DSS_DATA5, (IEN | PD | M4 )) /* gpio_75 */\
MUX_VAL(CONTROL_PADCONF_DSS_DATA6, (IEN | PD | M4 )) /* gpio_76 */\
MUX_VAL(CONTROL_PADCONF_DSS_DATA7, (IEN | PD | M4 )) /* gpio_77 */\
MUX_VAL(CONTROL_PADCONF_DSS_DATA8, (IEN | PD | M4 )) /* gpio_78 */\
MUX_VAL(CONTROL_PADCONF_DSS_DATA9, (IEN | PD | M4 )) /* gpio_79 */\
MUX_VAL(CONTROL_PADCONF_DSS_DATA10, (IEN | PD | M4 )) /* gpio_80 */\
MUX_VAL(CONTROL_PADCONF_DSS_DATA11, (IEN | PD | M4 )) /* gpio_81 */\
MUX_VAL(CONTROL_PADCONF_DSS_DATA12, (IEN | PD | M4 )) /* gpio_82 */\
MUX_VAL(CONTROL_PADCONF_DSS_DATA13, (IEN | PD | M4 )) /* gpio_83 */\
MUX_VAL(CONTROL_PADCONF_DSS_DATA14, (IEN | PD | M4 )) /* gpio_84 */\
MUX_VAL(CONTROL_PADCONF_DSS_DATA15, (IEN | PD | M4 )) /* gpio_85 */\
MUX_VAL(CONTROL_PADCONF_DSS_DATA16, (IDIS | PD | M0 )) /* dss_data16 */\
MUX_VAL(CONTROL_PADCONF_DSS_DATA17, (IDIS | PD | M0 )) /* dss_data17 */\
MUX_VAL(CONTROL_PADCONF_DSS_DATA18, (IDIS | PD | M0 )) /* dss_data18 */\
MUX_VAL(CONTROL_PADCONF_DSS_DATA19, (IDIS | PD | M0 )) /* dss_data19 */\
MUX_VAL(CONTROL_PADCONF_DSS_DATA20, (IDIS | PD | M0 )) /* dss_data20 */\
MUX_VAL(CONTROL_PADCONF_DSS_DATA21, (IDIS | PD | M0 )) /* dss_data21 */\
MUX_VAL(CONTROL_PADCONF_DSS_DATA22, (IDIS | PD | M0 )) /* dss_data22 */\
MUX_VAL(CONTROL_PADCONF_DSS_DATA23, (IDIS | PD | M0 )) /* dss_data23 */\
MUX_VAL(CONTROL_PADCONF_DSS_HSYNC, (IEN | PU | M4 )) /* gpio_67 */\
MUX_VAL(CONTROL_PADCONF_DSS_PCLK, (IEN | PU | M4 )) /* gpio_66 */\
MUX_VAL(CONTROL_PADCONF_DSS_VSYNC, (IDIS | PU | M0 )) /* dss_vsync */\
MUX_VAL(CONTROL_PADCONF_ETK_CLK_ES2, (IDIS | PI | M3 )) /* hsusb1_stp */\
MUX_VAL(CONTROL_PADCONF_ETK_CTL_ES2, (IDIS | PI | M3 )) /* hsusb1_clk */\
MUX_VAL(CONTROL_PADCONF_ETK_D0_ES2, (IEN | PD | M3 )) /* hsusb1_data0 */\
MUX_VAL(CONTROL_PADCONF_ETK_D1_ES2, (IEN | PD | M3 )) /* hsusb1_data1 */\
MUX_VAL(CONTROL_PADCONF_ETK_D2_ES2, (IEN | PD | M3 )) /* hsusb1_data2 */\
MUX_VAL(CONTROL_PADCONF_ETK_D3_ES2, (IEN | PD | M3 )) /* hsusb1_data7 */\
MUX_VAL(CONTROL_PADCONF_ETK_D4_ES2, (IEN | PD | M3 )) /* hsusb1_data4 */\
MUX_VAL(CONTROL_PADCONF_ETK_D5_ES2, (IEN | PD | M3 )) /* hsusb1_data5 */\
MUX_VAL(CONTROL_PADCONF_ETK_D6_ES2, (IEN | PD | M3 )) /* hsusb1_data6 */\
MUX_VAL(CONTROL_PADCONF_ETK_D7_ES2, (IEN | PD | M3 )) /* hsusb1_data3 */\
MUX_VAL(CONTROL_PADCONF_ETK_D8_ES2, (IEN | PD | M3 )) /* hsusb1_dir */\
MUX_VAL(CONTROL_PADCONF_ETK_D9_ES2, (IEN | PD | M3 )) /* hsusb1_nxt */\
MUX_VAL(CONTROL_PADCONF_ETK_D10_ES2, (IDIS | PD | M7 )) /* hw_dbg12 */\
MUX_VAL(CONTROL_PADCONF_ETK_D11_ES2, (IDIS | PD | M7 )) /* hw_dbg13 */\
MUX_VAL(CONTROL_PADCONF_ETK_D12_ES2, (IDIS | PD | M7 )) /* hw_dbg14 */\
MUX_VAL(CONTROL_PADCONF_ETK_D13_ES2, (IDIS | PD | M7 )) /* hw_dbg15 */\
MUX_VAL(CONTROL_PADCONF_ETK_D14_ES2, (IDIS | PD | M7 )) /* hw_dbg16 */\
MUX_VAL(CONTROL_PADCONF_ETK_D15_ES2, (IDIS | PD | M7 )) /* hw_dbg17 */\
MUX_VAL(CONTROL_PADCONF_GPMC_A1, (IDIS | PD | M0 )) /* gpmc_a1 */\
MUX_VAL(CONTROL_PADCONF_GPMC_A2, (IDIS | PD | M0 )) /* gpmc_a2 */\
MUX_VAL(CONTROL_PADCONF_GPMC_A3, (IDIS | PD | M0 )) /* gpmc_a3 */\
MUX_VAL(CONTROL_PADCONF_GPMC_A4, (IDIS | PD | M0 )) /* gpmc_a4 */\
MUX_VAL(CONTROL_PADCONF_GPMC_A5, (IDIS | PD | M0 )) /* gpmc_a5 */\
MUX_VAL(CONTROL_PADCONF_GPMC_A6, (IDIS | PU | M0 )) /* gpmc_a6 */\
MUX_VAL(CONTROL_PADCONF_GPMC_A7, (IDIS | PU | M0 )) /* gpmc_a7 */\
MUX_VAL(CONTROL_PADCONF_GPMC_A8, (IDIS | PU | M0 )) /* gpmc_a8 */\
MUX_VAL(CONTROL_PADCONF_GPMC_A9, (IDIS | PU | M0 )) /* gpmc_a9 */\
MUX_VAL(CONTROL_PADCONF_GPMC_A10, (IDIS | PU | M0 )) /* gpmc_a10 */\
MUX_VAL(CONTROL_PADCONF_GPMC_CLK, (IDIS | PI | M0 )) /* gpmc_clk */\
MUX_VAL(CONTROL_PADCONF_GPMC_D0, (IEN | PU | M0 )) /* gpmc_d0 */\
MUX_VAL(CONTROL_PADCONF_GPMC_D1, (IEN | PU | M0 )) /* gpmc_d1 */\
MUX_VAL(CONTROL_PADCONF_GPMC_D2, (IEN | PU | M0 )) /* gpmc_d2 */\
MUX_VAL(CONTROL_PADCONF_GPMC_D3, (IEN | PU | M0 )) /* gpmc_d3 */\
MUX_VAL(CONTROL_PADCONF_GPMC_D4, (IEN | PU | M0 )) /* gpmc_d4 */\
MUX_VAL(CONTROL_PADCONF_GPMC_D5, (IEN | PU | M0 )) /* gpmc_d5 */\
MUX_VAL(CONTROL_PADCONF_GPMC_D6, (IEN | PU | M0 )) /* gpmc_d6 */\
MUX_VAL(CONTROL_PADCONF_GPMC_D7, (IEN | PU | M0 )) /* gpmc_d7 */\
MUX_VAL(CONTROL_PADCONF_GPMC_D8, (IEN | PU | M0 )) /* gpmc_d8 */\
MUX_VAL(CONTROL_PADCONF_GPMC_D9, (IEN | PU | M0 )) /* gpmc_d9 */\
MUX_VAL(CONTROL_PADCONF_GPMC_D10, (IEN | PU | M0 )) /* gpmc_d10 */\
MUX_VAL(CONTROL_PADCONF_GPMC_D11, (IEN | PU | M0 )) /* gpmc_d11 */\
MUX_VAL(CONTROL_PADCONF_GPMC_D12, (IEN | PU | M0 )) /* gpmc_d12 */\
MUX_VAL(CONTROL_PADCONF_GPMC_D13, (IEN | PU | M0 )) /* gpmc_d13 */\
MUX_VAL(CONTROL_PADCONF_GPMC_D14, (IEN | PU | M0 )) /* gpmc_d14 */\
MUX_VAL(CONTROL_PADCONF_GPMC_D15, (IEN | PU | M0 )) /* gpmc_d15 */\
MUX_VAL(CONTROL_PADCONF_GPMC_NADV_ALE, (IDIS | PI | M0 )) /* gpmc_nadv_ale */\
MUX_VAL(CONTROL_PADCONF_GPMC_NBE0_CLE, (IDIS | PI | M0 )) /* gpmc_nbe0_cle */\
MUX_VAL(CONTROL_PADCONF_GPMC_NBE1, (IDIS | PD | M0 )) /* gpmc_nbe1 */\
MUX_VAL(CONTROL_PADCONF_GPMC_NCS0, (IDIS | PI | M0 )) /* gpmc_ncs0 */\
MUX_VAL(CONTROL_PADCONF_GPMC_NCS1, (IDIS | PI | M0 )) /* gpmc_ncs1 */\
MUX_VAL(CONTROL_PADCONF_GPMC_NCS2, (IDIS | PU | M0 )) /* gpmc_ncs2 */\
MUX_VAL(CONTROL_PADCONF_GPMC_NCS3, (IDIS | PU | M0 )) /* gpmc_ncs3 */\
MUX_VAL(CONTROL_PADCONF_GPMC_NCS4, (IEN | PU | M4 )) /* gpio_55 */\
MUX_VAL(CONTROL_PADCONF_GPMC_NCS5, (IDIS | PU | M0 )) /* gpmc_ncs5 */\
MUX_VAL(CONTROL_PADCONF_GPMC_NCS6, (IDIS | PI | M4 )) /* gpio_57 */\
MUX_VAL(CONTROL_PADCONF_GPMC_NCS7, (IDIS | PU | M0 )) /* gpmc_ncs7 */\
MUX_VAL(CONTROL_PADCONF_GPMC_NOE, (IDIS | PI | M0 )) /* gpmc_noe */\
MUX_VAL(CONTROL_PADCONF_GPMC_NWE, (IDIS | PI | M0 )) /* gpmc_nwe */\
MUX_VAL(CONTROL_PADCONF_GPMC_NWP, (IDIS | PI | M0 )) /* gpmc_nwp */\
MUX_VAL(CONTROL_PADCONF_GPMC_WAIT0, (IEN | PU | M0 )) /* gpmc_wait0 */\
MUX_VAL(CONTROL_PADCONF_GPMC_WAIT1, (IEN | PU | M0 )) /* gpmc_wait1 */\
MUX_VAL(CONTROL_PADCONF_GPMC_WAIT2, (IEN | PU | M0 )) /* gpmc_wait2 */\
MUX_VAL(CONTROL_PADCONF_GPMC_WAIT3, (IEN | PU | M0 )) /* gpmc_wait3 */\
MUX_VAL(CONTROL_PADCONF_HDQ_SIO, (IEN | PU | M0 )) /* hdq_sio */\
MUX_VAL(CONTROL_PADCONF_HECC1_RXD, (IDIS | PD | M4 )) /* gpio_131 */\
MUX_VAL(CONTROL_PADCONF_HECC1_TXD, (IEN | PI | M4 )) /* gpio_130 */\
MUX_VAL(CONTROL_PADCONF_I2C1_SCL, (IEN | PU | M0 )) /* i2c1_scl */\
MUX_VAL(CONTROL_PADCONF_I2C1_SDA, (IEN | PU | M0 )) /* i2c1_sda */\
MUX_VAL(CONTROL_PADCONF_I2C2_SCL, (IEN | PU | M0 )) /* i2c2_scl */\
MUX_VAL(CONTROL_PADCONF_I2C2_SDA, (IEN | PU | M0 )) /* i2c2_sda */\
MUX_VAL(CONTROL_PADCONF_I2C3_SCL, (IEN | PI | M0 )) /* i2c3_scl */\
MUX_VAL(CONTROL_PADCONF_I2C3_SDA, (IEN | PI | M0 )) /* i2c3_sda */\
MUX_VAL(CONTROL_PADCONF_JTAG_EMU0, (IEN | PU | M0 )) /* jtag_emu0 */\
MUX_VAL(CONTROL_PADCONF_JTAG_EMU1, (IEN | PU | M0 )) /* jtag_emu1 */\
MUX_VAL(CONTROL_PADCONF_JTAG_NTRST, (IEN | PD | M0 )) /* jtag_ntrst */\
MUX_VAL(CONTROL_PADCONF_JTAG_RTCK, (IDIS | PI | M0 )) /* jtag_rtck */\
MUX_VAL(CONTROL_PADCONF_JTAG_TCK, (IEN | PD | M0 )) /* jtag_tck */\
MUX_VAL(CONTROL_PADCONF_JTAG_TDI, (IEN | PU | M0 )) /* jtag_tdi */\
MUX_VAL(CONTROL_PADCONF_JTAG_TDO, (IDIS | PI | M0 )) /* jtag_tdo */\
MUX_VAL(CONTROL_PADCONF_JTAG_TMS, (IEN | PU | M0 )) /* jtag_tms_tmsc */\
MUX_VAL(CONTROL_PADCONF_MCBSP_CLKS, (IEN | PU | M4 )) /* gpio_160 */\
MUX_VAL(CONTROL_PADCONF_MCBSP1_CLKR, (IEN | PD | M0 )) /* mcbsp1_clkr */\
MUX_VAL(CONTROL_PADCONF_MCBSP1_CLKX, (IEN | PD | M0 )) /* mcbsp1_clkx */\
MUX_VAL(CONTROL_PADCONF_MCBSP1_DR, (IEN | PD | M0 )) /* mcbsp1_dr */\
MUX_VAL(CONTROL_PADCONF_MCBSP1_DX, (IDIS | PD | M0 )) /* mcbsp1_dx */\
MUX_VAL(CONTROL_PADCONF_MCBSP1_FSR, (IEN | PD | M0 )) /* mcbsp1_fsr */\
MUX_VAL(CONTROL_PADCONF_MCBSP1_FSX, (IEN | PD | M0 )) /* mcbsp1_fsx */\
MUX_VAL(CONTROL_PADCONF_MCBSP2_CLKX, (IEN | PD | M0 )) /* mcbsp2_clkx */\
MUX_VAL(CONTROL_PADCONF_MCBSP2_DR, (IEN | PI | M4 )) /* gpio_118 */\
MUX_VAL(CONTROL_PADCONF_MCBSP2_DX, (IEN | PI | M4 )) /* gpio_119 */\
MUX_VAL(CONTROL_PADCONF_MCBSP2_FSX, (IEN | PD | M4 )) /* gpio_116 */\
MUX_VAL(CONTROL_PADCONF_MCBSP3_CLKX, (IEN | PD | M0 )) /* mcbsp3_clkx */\
MUX_VAL(CONTROL_PADCONF_MCBSP3_DR, (IEN | PD | M0 )) /* mcbsp3_dr */\
MUX_VAL(CONTROL_PADCONF_MCBSP3_DX, (IDIS | PD | M0 )) /* mcbsp3_dx */\
MUX_VAL(CONTROL_PADCONF_MCBSP3_FSX, (IEN | PD | M0 )) /* mcbsp3_fsx */\
MUX_VAL(CONTROL_PADCONF_MCBSP4_CLKX, (IEN | PD | M0 )) /* mcbsp4_clkx */\
MUX_VAL(CONTROL_PADCONF_MCBSP4_DR, (IEN | PD | M0 )) /* mcbsp4_dr */\
MUX_VAL(CONTROL_PADCONF_MCBSP4_DX, (IDIS | PD | M0 )) /* mcbsp4_dx */\
MUX_VAL(CONTROL_PADCONF_MCBSP4_FSX, (IEN | PD | M0 )) /* mcbsp4_fsx */\
MUX_VAL(CONTROL_PADCONF_MCSPI1_CLK, (IDIS | PD | M0 )) /* mcspi1_clk */\
MUX_VAL(CONTROL_PADCONF_MCSPI1_CS0, (IEN | PU | M0 )) /* mcspi1_cs0 */\
MUX_VAL(CONTROL_PADCONF_MCSPI1_CS1, (IDIS | PD | M4 )) /* gpio_175 */\
MUX_VAL(CONTROL_PADCONF_MCSPI1_CS2, (IDIS | PD | M4 )) /* gpio_176 */\
MUX_VAL(CONTROL_PADCONF_MCSPI1_CS3, (IDIS | PD | M4 )) /* gpio_177 */\
MUX_VAL(CONTROL_PADCONF_MCSPI1_SIMO, (IDIS | PD | M0 )) /* mcspi1_simo */\
MUX_VAL(CONTROL_PADCONF_MCSPI1_SOMI, (IEN | PU | M0 )) /* mcspi1_somi */\
MUX_VAL(CONTROL_PADCONF_MCSPI2_CLK, (IEN | PD | M4 )) /* gpio_178 */\
MUX_VAL(CONTROL_PADCONF_MCSPI2_CS0, (IEN | PU | M4 )) /* gpio_181 */\
MUX_VAL(CONTROL_PADCONF_MCSPI2_CS1, (IDIS | PD | M0 )) /* mcspi2_cs1 */\
MUX_VAL(CONTROL_PADCONF_MCSPI2_SIMO, (IEN | PD | M1 )) /* gpt9_pwm_evt */\
MUX_VAL(CONTROL_PADCONF_MCSPI2_SOMI, (IEN | PD | M4 )) /* gpio_180 */\
MUX_VAL(CONTROL_PADCONF_MMC1_CLK, (IEN | PD | M4 )) /* gpio_120 */\
MUX_VAL(CONTROL_PADCONF_MMC1_CMD, (IEN | PD | M4 )) /* gpio_121 */\
MUX_VAL(CONTROL_PADCONF_MMC1_DAT0, (IEN | PD | M1 )) /* mcspi2_clk */\
MUX_VAL(CONTROL_PADCONF_MMC1_DAT1, (IEN | PD | M0 )) /* mmc1_dat1 */\
MUX_VAL(CONTROL_PADCONF_MMC1_DAT2, (IEN | PD | M1 )) /* mcspi2_somi */\
MUX_VAL(CONTROL_PADCONF_MMC1_DAT3, (IDIS | PD | M1 )) /* mcspi2_cs0 */\
MUX_VAL(CONTROL_PADCONF_MMC1_DAT4, (IEN | PD | M4 )) /* gpio_126 */\
MUX_VAL(CONTROL_PADCONF_MMC1_DAT5, (IEN | PD | M4 )) /* gpio_127 */\
MUX_VAL(CONTROL_PADCONF_MMC1_DAT6, (IEN | PD | M0 )) /* mmc1_dat6 */\
MUX_VAL(CONTROL_PADCONF_MMC1_DAT7, (IEN | PD | M0 )) /* mmc1_dat7 */\
MUX_VAL(CONTROL_PADCONF_MMC2_CLK, (IDIS | PU | M0 )) /* mmc2_clk */\
MUX_VAL(CONTROL_PADCONF_MMC2_CMD, (IEN | PU | M0 )) /* mmc2_cmd */\
MUX_VAL(CONTROL_PADCONF_MMC2_DAT0, (IEN | PU | M0 )) /* mmc2_dat0 */\
MUX_VAL(CONTROL_PADCONF_MMC2_DAT1, (IEN | PU | M0 )) /* mmc2_dat1 */\
MUX_VAL(CONTROL_PADCONF_MMC2_DAT2, (IEN | PU | M0 )) /* mmc2_dat2 */\
MUX_VAL(CONTROL_PADCONF_MMC2_DAT3, (IEN | PU | M0 )) /* mmc2_dat3 */\
MUX_VAL(CONTROL_PADCONF_MMC2_DAT4, (IDIS | PD | M1 )) /* mmc2_dir_dat0 */\
MUX_VAL(CONTROL_PADCONF_MMC2_DAT5, (IDIS | PD | M1 )) /* mmc2_dir_dat1 */\
MUX_VAL(CONTROL_PADCONF_MMC2_DAT6, (IDIS | PD | M1 )) /* mmc2_dir_cmd */\
MUX_VAL(CONTROL_PADCONF_MMC2_DAT7, (IEN | PD | M1 )) /* mmc2_clkin */\
MUX_VAL(CONTROL_PADCONF_RMII_50MHZ_CLK, (IEN | PU | M0 )) /* rmii_50mhz_clk */\
MUX_VAL(CONTROL_PADCONF_RMII_CRS_DV, (IEN | PU | M0 )) /* rmii_crs_dv */\
MUX_VAL(CONTROL_PADCONF_RMII_MDIO_CLK, (IDIS | PU | M0 )) /* rmii_mdio_clk */\
MUX_VAL(CONTROL_PADCONF_RMII_MDIO_DATA, (IEN | PU | M0 )) /* rmii_mdio_data */\
MUX_VAL(CONTROL_PADCONF_RMII_RXD0, (IEN | PU | M0 )) /* rmii_rxd0 */\
MUX_VAL(CONTROL_PADCONF_RMII_RXD1, (IEN | PU | M0 )) /* rmii_rxd1 */\
MUX_VAL(CONTROL_PADCONF_RMII_RXER, (IEN | PU | M0 )) /* rmii_rxer */\
MUX_VAL(CONTROL_PADCONF_RMII_TXD0, (IDIS | PU | M0 )) /* rmii_txd0 */\
MUX_VAL(CONTROL_PADCONF_RMII_TXD1, (IDIS | PU | M0 )) /* rmii_txd1 */\
MUX_VAL(CONTROL_PADCONF_RMII_TXEN, (IDIS | PU | M0 )) /* rmii_txen */\
MUX_VAL(CONTROL_PADCONF_SDRC_CKE0, (IEN | PD | M7 )) /* sdrc_cke0_safe */\
MUX_VAL(CONTROL_PADCONF_SDRC_CLK, (IDIS | PI | M0 )) /* sdrc_clk */\
MUX_VAL(CONTROL_PADCONF_SDRC_D0, (IEN | PI | M0 )) /* sdrc_d0 */\
MUX_VAL(CONTROL_PADCONF_SDRC_D1, (IEN | PI | M0 )) /* sdrc_d1 */\
MUX_VAL(CONTROL_PADCONF_SDRC_D2, (IEN | PI | M0 )) /* sdrc_d2 */\
MUX_VAL(CONTROL_PADCONF_SDRC_D3, (IEN | PI | M0 )) /* sdrc_d3 */\
MUX_VAL(CONTROL_PADCONF_SDRC_D4, (IEN | PI | M0 )) /* sdrc_d4 */\
MUX_VAL(CONTROL_PADCONF_SDRC_D5, (IEN | PI | M0 )) /* sdrc_d5 */\
MUX_VAL(CONTROL_PADCONF_SDRC_D6, (IEN | PI | M0 )) /* sdrc_d6 */\
MUX_VAL(CONTROL_PADCONF_SDRC_D7, (IEN | PI | M0 )) /* sdrc_d7 */\
MUX_VAL(CONTROL_PADCONF_SDRC_D8, (IEN | PI | M0 )) /* sdrc_d8 */\
MUX_VAL(CONTROL_PADCONF_SDRC_D9, (IEN | PI | M0 )) /* sdrc_d9 */\
MUX_VAL(CONTROL_PADCONF_SDRC_D10, (IEN | PI | M0 )) /* sdrc_d10 */\
MUX_VAL(CONTROL_PADCONF_SDRC_D11, (IEN | PI | M0 )) /* sdrc_d11 */\
MUX_VAL(CONTROL_PADCONF_SDRC_D12, (IEN | PI | M0 )) /* sdrc_d12 */\
MUX_VAL(CONTROL_PADCONF_SDRC_D13, (IEN | PI | M0 )) /* sdrc_d13 */\
MUX_VAL(CONTROL_PADCONF_SDRC_D14, (IEN | PI | M0 )) /* sdrc_d14 */\
MUX_VAL(CONTROL_PADCONF_SDRC_D15, (IEN | PI | M0 )) /* sdrc_d15 */\
MUX_VAL(CONTROL_PADCONF_SDRC_D16, (IEN | PI | M0 )) /* sdrc_d16 */\
MUX_VAL(CONTROL_PADCONF_SDRC_D17, (IEN | PI | M0 )) /* sdrc_d17 */\
MUX_VAL(CONTROL_PADCONF_SDRC_D18, (IEN | PI | M0 )) /* sdrc_d18 */\
MUX_VAL(CONTROL_PADCONF_SDRC_D19, (IEN | PI | M0 )) /* sdrc_d19 */\
MUX_VAL(CONTROL_PADCONF_SDRC_D20, (IEN | PI | M0 )) /* sdrc_d20 */\
MUX_VAL(CONTROL_PADCONF_SDRC_D21, (IEN | PI | M0 )) /* sdrc_d21 */\
MUX_VAL(CONTROL_PADCONF_SDRC_D22, (IEN | PI | M0 )) /* sdrc_d22 */\
MUX_VAL(CONTROL_PADCONF_SDRC_D23, (IEN | PI | M0 )) /* sdrc_d23 */\
MUX_VAL(CONTROL_PADCONF_SDRC_D24, (IEN | PI | M0 )) /* sdrc_d24 */\
MUX_VAL(CONTROL_PADCONF_SDRC_D25, (IEN | PI | M0 )) /* sdrc_d25 */\
MUX_VAL(CONTROL_PADCONF_SDRC_D26, (IEN | PI | M0 )) /* sdrc_d26 */\
MUX_VAL(CONTROL_PADCONF_SDRC_D27, (IEN | PI | M0 )) /* sdrc_d27 */\
MUX_VAL(CONTROL_PADCONF_SDRC_D28, (IEN | PI | M0 )) /* sdrc_d28 */\
MUX_VAL(CONTROL_PADCONF_SDRC_D29, (IEN | PI | M0 )) /* sdrc_d29 */\
MUX_VAL(CONTROL_PADCONF_SDRC_D30, (IEN | PI | M0 )) /* sdrc_d30 */\
MUX_VAL(CONTROL_PADCONF_SDRC_D31, (IEN | PI | M0 )) /* sdrc_d31 */\
MUX_VAL(CONTROL_PADCONF_SDRC_DQS0N, (IEN | PI | M0 )) /* sdrc_dqs0n */\
MUX_VAL(CONTROL_PADCONF_SDRC_DQS0, (IEN | PI | M0 )) /* sdrc_dqs0p */\
MUX_VAL(CONTROL_PADCONF_SDRC_DQS1N, (IEN | PI | M0 )) /* sdrc_dqs1n */\
MUX_VAL(CONTROL_PADCONF_SDRC_DQS1, (IEN | PI | M0 )) /* sdrc_dqs1p */\
MUX_VAL(CONTROL_PADCONF_SDRC_DQS2N, (IEN | PI | M0 )) /* sdrc_dqs2n */\
MUX_VAL(CONTROL_PADCONF_SDRC_DQS2, (IEN | PI | M0 )) /* sdrc_dqs2p */\
MUX_VAL(CONTROL_PADCONF_SDRC_DQS3N, (IEN | PI | M0 )) /* sdrc_dqs3n */\
MUX_VAL(CONTROL_PADCONF_SDRC_DQS3, (IEN | PI | M0 )) /* sdrc_dqs3p */\
MUX_VAL(CONTROL_PADCONF_STRBEN_DLY0, (IEN | PI | M0 )) /* sdrc_strben_dly0 */\
MUX_VAL(CONTROL_PADCONF_STRBEN_DLY1, (IEN | PI | M0 )) /* sdrc_strben_dly1 */\
MUX_VAL(CONTROL_PADCONF_SYS_32K, (IEN | PI | M0 )) /* sys_32k */\
MUX_VAL(CONTROL_PADCONF_SYS_BOOT0, (IEN | PI | M0 )) /* sys_boot0 */\
MUX_VAL(CONTROL_PADCONF_SYS_BOOT1, (IEN | PI | M0 )) /* sys_boot1 */\
MUX_VAL(CONTROL_PADCONF_SYS_BOOT2, (IEN | PI | M0 )) /* sys_boot2 */\
MUX_VAL(CONTROL_PADCONF_SYS_BOOT3, (IEN | PI | M0 )) /* sys_boot3 */\
MUX_VAL(CONTROL_PADCONF_SYS_BOOT4, (IDIS | PI | M1 )) /* mmc2_dir_dat2 */\
MUX_VAL(CONTROL_PADCONF_SYS_BOOT5, (IDIS | PI | M1 )) /* mmc2_dir_dat3 */\
MUX_VAL(CONTROL_PADCONF_SYS_BOOT6, (IEN | PI | M0 )) /* sys_boot6 */\
MUX_VAL(CONTROL_PADCONF_SYS_BOOT7, (IEN | PI | M0 )) /* sys_boot7 */\
MUX_VAL(CONTROL_PADCONF_SYS_BOOT8, (IEN | PI | M0 )) /* sys_boot8 */\
MUX_VAL(CONTROL_PADCONF_SYS_CLKOUT1, (IDIS | PD | M0 )) /* sys_clkout1 */\
MUX_VAL(CONTROL_PADCONF_SYS_CLKOUT2, (IDIS | PD | M0 )) /* sys_clkout2 */\
MUX_VAL(CONTROL_PADCONF_SYS_CLKREQ, (IEN | PI | M0 )) /* sys_clkreq */\
MUX_VAL(CONTROL_PADCONF_SYS_NIRQ, (IEN | PU | M4 )) /* gpio_0 */\
MUX_VAL(CONTROL_PADCONF_SYS_NRESWARM, (IDIS | PU | M4 )) /* gpio_30 */\
MUX_VAL(CONTROL_PADCONF_UART1_CTS, (IDIS | PD | M4 )) /* gpio_150 */\
MUX_VAL(CONTROL_PADCONF_UART1_RTS, (IEN | PD | M4 )) /* gpio_149 */\
MUX_VAL(CONTROL_PADCONF_UART1_RX, (IEN | PD | M0 )) /* uart1_rx */\
MUX_VAL(CONTROL_PADCONF_UART1_TX, (IDIS | PD | M0 )) /* uart1_tx */\
MUX_VAL(CONTROL_PADCONF_UART2_CTS, (IEN | PU | M4 )) /* gpio_144 */\
MUX_VAL(CONTROL_PADCONF_UART2_RTS, (IDIS | PU | M0 )) /* uart2_rts */\
MUX_VAL(CONTROL_PADCONF_UART2_RX, (IEN | PU | M0 )) /* uart2_rx */\
MUX_VAL(CONTROL_PADCONF_UART2_TX, (IDIS | PU | M0 )) /* uart2_tx */\
MUX_VAL(CONTROL_PADCONF_UART3_CTS_RCTX, (IEN | PU | M4 )) /* gpio_163 */\
MUX_VAL(CONTROL_PADCONF_UART3_RTS_SD, (IEN | PU | M4 )) /* gpio_164 */\
MUX_VAL(CONTROL_PADCONF_UART3_RX_IRRX, (IEN | PU | M0 )) /* uart3_rx_irrx */\
MUX_VAL(CONTROL_PADCONF_UART3_TX_IRTX, (IDIS | PU | M0 )) /* uart3_tx_irtx */\
MUX_VAL(CONTROL_PADCONF_USB0_DRVBUS, (IDIS | PD | M0 )) /* usb0_drvvbus */\

#endif
