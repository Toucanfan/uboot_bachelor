/*
 * board.c
 *
 * Board functions for TI AM335X based boards
 *
 * Copyright (C) 2011, Texas Instruments, Incorporated - http://www.ti.com/
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <errno.h>
#include <spl.h>
#include <serial.h>
#include <asm/arch/cpu.h>
#include <asm/arch/hardware.h>
#include <asm/arch/omap.h>
#include <asm/arch/ddr_defs.h>
#include <asm/arch/clock.h>
#include <asm/arch/clk_synthesizer.h>
#include <asm/arch/gpio.h>
#include <asm/arch/mmc_host_def.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/mem.h>
#include <asm/io.h>
#include <asm/emif.h>
#include <asm/gpio.h>
#include <asm/omap_sec_common.h>
#include <i2c.h>
#include <miiphy.h>
#include <cpsw.h>
#include <power/tps65217.h>
#include <power/tps65910.h>
#include <environment.h>
#include <watchdog.h>
#include <environment.h>
#include <libfdt.h>
#include <fdt_support.h>

#include "../common/board_detect.h"
#include "board.h"

/* Micron MT41K256M16HA-125E / Alliance Memory AS4C256M16D3 */
#define AS4C256M16D3_EMIF_READ_LATENCY 0x100007
#define AS4C256M16D3_EMIF_TIM1     0x0AAAD4DB
#define AS4C256M16D3_EMIF_TIM2     0x266B7FDA
#define AS4C256M16D3_EMIF_TIM3     0x501F867F
#define AS4C256M16D3_EMIF_SDCFG        0x61C05332
#define AS4C256M16D3_EMIF_SDREF        0xC30
#define AS4C256M16D3_ZQ_CFG        0x50074BE4
#define AS4C256M16D3_RATIO         0x40 //from script
#define AS4C256M16D3_INVERT_CLKOUT     0x0 //from script
#define AS4C256M16D3_RD_DQS        0x03a //from script
#define AS4C256M16D3_WR_DQS        0x03b //from script
#define AS4C256M16D3_PHY_WR_DATA       0x074 //from script
#define AS4C256M16D3_PHY_FIFO_WE       0x0a6 //from script
#define AS4C256M16D3_IOCTRL_VALUE      0x18B

#define EMIF_OCP_CONFIG_BACHELOR_BOARD 0x00141414


DECLARE_GLOBAL_DATA_PTR;

static struct ctrl_dev *cdev = (struct ctrl_dev *)CTRL_DEVICE_BASE;

#ifndef CONFIG_DM_SERIAL
struct serial_device *default_serial_console(void)
{
	return &eserial1_device;
}
#endif /* CONFIG_DM_SERIAL */

#ifndef CONFIG_SKIP_LOWLEVEL_INIT

static const struct ddr_data ddr3_beagleblack_data = {
	.datardsratio0 = AS4C256M16D3_RD_DQS,
	.datawdsratio0 = AS4C256M16D3_WR_DQS,
	.datafwsratio0 = AS4C256M16D3_PHY_FIFO_WE,
	.datawrsratio0 = AS4C256M16D3_PHY_WR_DATA,
};


static const struct cmd_control ddr3_beagleblack_cmd_ctrl_data = {
	.cmd0csratio = AS4C256M16D3_RATIO,
	.cmd0iclkout = AS4C256M16D3_INVERT_CLKOUT,

	.cmd1csratio = AS4C256M16D3_RATIO,
	.cmd1iclkout = AS4C256M16D3_INVERT_CLKOUT,

	.cmd2csratio = AS4C256M16D3_RATIO,
	.cmd2iclkout = AS4C256M16D3_INVERT_CLKOUT,
};


static struct emif_regs ddr3_beagleblack_emif_reg_data = {
	.sdram_config = AS4C256M16D3_EMIF_SDCFG,
	.ref_ctrl = AS4C256M16D3_EMIF_SDREF,
	.sdram_tim1 = AS4C256M16D3_EMIF_TIM1,
	.sdram_tim2 = AS4C256M16D3_EMIF_TIM2,
	.sdram_tim3 = AS4C256M16D3_EMIF_TIM3,
	.ocp_config = EMIF_OCP_CONFIG_BACHELOR_BOARD,
	.zq_config = AS4C256M16D3_ZQ_CFG,
	.emif_ddr_phy_ctlr_1 = AS4C256M16D3_EMIF_READ_LATENCY,
};


#ifdef CONFIG_SPL_OS_BOOT
int spl_start_uboot(void)
{
	/* break into full u-boot on 'c' */
	if (serial_tstc() && serial_getc() == 'c')
		return 1;

#ifdef CONFIG_SPL_ENV_SUPPORT
	env_init();
	env_relocate_spec();
	if (getenv_yesno("boot_os") != 1)
		return 1;
#endif /* CONFIG_SPL_ENV_SUPPORT */

	return 0;
}
#endif /* CONFIG_SPL_OS_BOOT */

#define OSC	(V_OSCK/1000000)
const struct dpll_params dpll_ddr = {
		266, OSC-1, 1, -1, -1, -1, -1};
const struct dpll_params dpll_ddr_evm_sk = {
		303, OSC-1, 1, -1, -1, -1, -1};
const struct dpll_params dpll_ddr_bone_black = {
		400, OSC-1, 1, -1, -1, -1, -1};

const struct dpll_params *get_dpll_ddr_params(void)
{
	return &dpll_ddr_bone_black;
}

void set_uart_mux_conf(void)
{
	enable_uart0_pin_mux();
}

void set_mux_conf_regs(void)
{
	enable_board_pin_mux();
}

const struct ctrl_ioregs ioregs_bonelt = {
	.cm0ioctl		= AS4C256M16D3_IOCTRL_VALUE,
	.cm1ioctl		= AS4C256M16D3_IOCTRL_VALUE,
	.cm2ioctl		= AS4C256M16D3_IOCTRL_VALUE,
	.dt0ioctl		= AS4C256M16D3_IOCTRL_VALUE,
	.dt1ioctl		= AS4C256M16D3_IOCTRL_VALUE,
};

void sdram_init(void)
{
	config_ddr(400, &ioregs_bonelt,
	   &ddr3_beagleblack_data,
	   &ddr3_beagleblack_cmd_ctrl_data,
	   &ddr3_beagleblack_emif_reg_data, 0);
}
#endif /* CONFIG_SKIP_LOWLEVEL_INIT */

/*
 * Basic board specific setup.  Pinmux has been handled already.
 */
int board_init(void)
{
#if defined(CONFIG_HW_WATCHDOG)
	hw_watchdog_init();
#endif

	gd->bd->bi_boot_params = CONFIG_SYS_SDRAM_BASE + 0x100;

	return 0;
}

#ifdef CONFIG_BOARD_LATE_INIT
int board_late_init(void)
{
#if !defined(CONFIG_SPL_BUILD)
	uint8_t mac_addr[6];
	uint32_t mac_hi, mac_lo;
#endif

#if !defined(CONFIG_SPL_BUILD)
	/* try reading mac address from efuse */
	mac_lo = readl(&cdev->macid0l);
	mac_hi = readl(&cdev->macid0h);
	mac_addr[0] = mac_hi & 0xFF;
	mac_addr[1] = (mac_hi & 0xFF00) >> 8;
	mac_addr[2] = (mac_hi & 0xFF0000) >> 16;
	mac_addr[3] = (mac_hi & 0xFF000000) >> 24;
	mac_addr[4] = mac_lo & 0xFF;
	mac_addr[5] = (mac_lo & 0xFF00) >> 8;

	if (!getenv("ethaddr")) {
		printf("<ethaddr> not set. Validating first E-fuse MAC\n");

		if (is_valid_ethaddr(mac_addr))
			eth_setenv_enetaddr("ethaddr", mac_addr);
	}

	mac_lo = readl(&cdev->macid1l);
	mac_hi = readl(&cdev->macid1h);
	mac_addr[0] = mac_hi & 0xFF;
	mac_addr[1] = (mac_hi & 0xFF00) >> 8;
	mac_addr[2] = (mac_hi & 0xFF0000) >> 16;
	mac_addr[3] = (mac_hi & 0xFF000000) >> 24;
	mac_addr[4] = mac_lo & 0xFF;
	mac_addr[5] = (mac_lo & 0xFF00) >> 8;

	if (!getenv("eth1addr")) {
		if (is_valid_ethaddr(mac_addr))
			eth_setenv_enetaddr("eth1addr", mac_addr);
	}
#endif /* !defined(CONFIG_SPL_BUILD) */

	return 0;
}
#endif /* CONFIG_BOARD_LATE_INIT */

int board_fit_config_name_match(const char *name) {
	return -1;
}
