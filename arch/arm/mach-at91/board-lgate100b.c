/*
 * linux/arch/arm/mach-at91/board-lgate100b.c
 *
 *  Copyright (C) 2005 SAN People
 *  Copyright (C) 2006 Atmel
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/at73c213.h>
#include <linux/clk.h>
#include <linux/i2c/at24.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>

#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/irq.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <mach/hardware.h>
#include <mach/board.h>
#include <mach/gpio.h>
#include <mach/at91sam9_smc.h>
#include <mach/at91_shdwc.h>

#include "sam9_smc.h"
#include "generic.h"


static void __init ek_map_io(void)
{
	/* Initialize processor: 18.432 MHz crystal */
	at91sam9260_initialize(18432000);

	/* DGBU on ttyS0. (Rx & Tx only) */
	at91_register_uart(0, 0, 0);

    if ((system_rev & 0xf) >= 9) // is this Lgate-50 with modem port?
    {
    	/* USART0 on ttyS1. (Rx, Tx, CTS, RTS, DTR, DSR, DCD, RI) */
		at91_register_uart(AT91SAM9260_ID_US0, 1, ATMEL_UART_CTS | ATMEL_UART_RTS
			   | ATMEL_UART_DTR | ATMEL_UART_DSR | ATMEL_UART_DCD
			   | ATMEL_UART_RI);

 		/* USART1 on ttyS2. (Rx, Tx, RTS, CTS) */
		at91_register_uart(AT91SAM9260_ID_US1, 2, ATMEL_UART_CTS | ATMEL_UART_RTS);
	}
//	/* USART3 on ttyS3. (Rx, Tx) */
//	at91_register_uart(AT91SAM9260_ID_US2, 3, 0);

	/* set serial console to ttyS0 (ie, DBGU) */
	at91_set_serial_console(0);
}

static void __init ek_init_irq(void)
{
	at91sam9260_init_interrupts(NULL);
}


/*
 * USB Host port
 */
static struct at91_usbh_data __initdata ek_usbh_data = {
	.ports		= 2,
};

/*
 * USB Device port
 */
static struct at91_udc_data __initdata ek_udc_data = {
	.vbus_pin	= AT91_PIN_PC5,
	.pullup_pin	= 0,		/* pull-up driven by UDC */
};

/*
 * Compact Flash (via Expansion Connector)
 */
static struct at91_cf_data __initdata ek_cf_data = {
	// .irq_pin	= ... user defined
	// .det_pin	= ... user defined
	// .vcc_pin	= ... user defined
	// .rst_pin	= ... user defined
	.chipselect	= 4,
};

/*
 * Audio
 */
static struct at73c213_board_info at73c213_data = {
	.ssc_id		= 0,
	.shortname	= "AT91SAM9260-EK external DAC",
};

#if defined(CONFIG_SND_AT73C213) || defined(CONFIG_SND_AT73C213_MODULE)
static void __init at73c213_set_clk(struct at73c213_board_info *info)
{
	struct clk *pck0;
	struct clk *plla;

	pck0 = clk_get(NULL, "pck0");
	plla = clk_get(NULL, "plla");

	/* AT73C213 MCK Clock */
	at91_set_B_periph(AT91_PIN_PC1, 0);	/* PCK0 */

	clk_set_parent(pck0, plla);
	clk_put(plla);

	info->dac_clk = pck0;
}
#else
static void __init at73c213_set_clk(struct at73c213_board_info *info) {}
#endif

/*
 * SPI devices.
 */
static struct spi_board_info ek_spi_devices[] = {
	{	/* lcd */
		.modalias	= "ssd1805",
		.chip_select	= 2,  // was 0 in hw rev 0, moved for serial boot flash
		.max_speed_hz	= 5 * 1000 * 1000,
		.bus_num	= 0,
	},
#if 0 // some day this may be the energy chips if the pic is removed
	{	/* lcd */
		.modalias	= "spidev",
		.chip_select	= 0,
		.max_speed_hz	= 5 * 1000 * 1000,
		.bus_num	= 1,
	},
#endif	
#if !defined(CONFIG_MMC_AT91)
	{	/* DataFlash chip */
		.modalias	= "mtd_dataflash",
		.chip_select	= 1,
		.max_speed_hz	= 15 * 1000 * 1000,
		.bus_num	= 0,
	},
#if defined(CONFIG_MTD_AT91_DATAFLASH_CARD)
	{	/* DataFlash card */
		.modalias	= "mtd_dataflash",
		.chip_select	= 0,
		.max_speed_hz	= 15 * 1000 * 1000,
		.bus_num	= 0,
	},
#endif
#endif
#if defined(CONFIG_SND_AT73C213) || defined(CONFIG_SND_AT73C213_MODULE)
	{	/* AT73C213 DAC */
		.modalias	= "at73c213",
		.chip_select	= 0,
		.max_speed_hz	= 10 * 1000 * 1000,
		.bus_num	= 1,
		.mode		= SPI_MODE_1,
		.platform_data	= &at73c213_data,
	},
#endif
};


/*
 * MACB Ethernet device
 */
static struct at91_eth_data __initdata ek_macb_data = {
	.phy_irq_pin	= AT91_PIN_PA7,
	.is_rmii	= 1,
};


/*
 * NAND flash
 */
static struct mtd_partition __initdata ek_nand_partition[] = {
	{
		.name	= "Bootstrap",
		.offset	= 0,
		.size	= SZ_4M,
	},
	{
		.name	= "Partition 1",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= 28 * SZ_1M,
	},
	{
		.name	= "Partition 2",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= MTDPART_SIZ_FULL,
	},
};

static struct mtd_partition * __init nand_partitions(int size, int *num_partitions)
{
	*num_partitions = ARRAY_SIZE(ek_nand_partition);
	return ek_nand_partition;
}

static struct atmel_nand_data __initdata ek_nand_data = {
	.ale		= 21,
	.cle		= 22,
//	.det_pin	= ... not connected
	.rdy_pin	= AT91_PIN_PC13,
	.enable_pin	= AT91_PIN_PC14,
	.partition_info	= nand_partitions,
#if defined(CONFIG_MTD_NAND_ATMEL_BUSWIDTH_16)
	.bus_width_16	= 1,
#else
	.bus_width_16	= 0,
#endif
};
#ifdef CONFIG_ARCH_AT91SAM9260
static struct sam9_smc_config __initdata ek_nand_smc_config = {
	.ncs_read_setup		= 0,
	.nrd_setup		= 1,
	.ncs_write_setup	= 0,
	.nwe_setup		= 1,

	.ncs_read_pulse		= 3,
	.nrd_pulse		= 3,
	.ncs_write_pulse	= 3,
	.nwe_pulse		= 3,

	.read_cycle		= 5,
	.write_cycle		= 5,

	.mode			= AT91_SMC_READMODE | AT91_SMC_WRITEMODE | AT91_SMC_EXNWMODE_DISABLE,
	.tdf_cycles		= 2,
};
#else // CONFIG_AT91SAM9G20
static struct sam9_smc_config __initdata ek_nand_smc_config = {
	.ncs_read_setup		= 0,
	.nrd_setup		= 2,
	.ncs_write_setup	= 0,
	.nwe_setup		= 2,

	.ncs_read_pulse		= 4,
	.nrd_pulse		= 4,
	.ncs_write_pulse	= 4,
	.nwe_pulse		= 4,

	.read_cycle		= 7,
	.write_cycle		= 7,

	.mode			= AT91_SMC_READMODE | AT91_SMC_WRITEMODE | AT91_SMC_EXNWMODE_DISABLE,
	.tdf_cycles		= 3,
};
#endif
static void __init ek_add_device_nand(void)
{
	/* setup bus-width (8 or 16) */
	if (ek_nand_data.bus_width_16)
		ek_nand_smc_config.mode |= AT91_SMC_DBW_16;
	else
		ek_nand_smc_config.mode |= AT91_SMC_DBW_8;

	/* configure chip-select 3 (NAND) */
	sam9_smc_configure(3, &ek_nand_smc_config);

	at91_add_device_nand(&ek_nand_data);
}


/*
 * MCI (SD/MMC)
 */
static struct at91_mmc_data __initdata ek_mmc_data = {
	.slot_b		= 1,
	.wire4		= 1,
//	.det_pin	= ... not connected
//	.wp_pin		= ... not connected
//	.vcc_pin	= ... not connected
};


/*
 * LEDs
 */
static struct gpio_led ek_leds[] = {
	{	/* "bottom" led */
		.name			= "red",
		.gpio			= AT91_PIN_PB23,
		.default_trigger	= "none",
	},
	{	/* "top" led */
		.name			= "green",
		.gpio			= AT91_PIN_PB20,
		.default_trigger	= "none",
	},
	{	/* backlight */
		.name			= "backlight",
		.gpio			= AT91_PIN_PA8,
		.default_trigger	= "backlight",
	},
	{	/* USB power */
		.name			= "USBPower",
		.gpio			= AT91_PIN_PB24,
		.default_trigger	= "USBPower",
	},
	{	/* EOP power*/
		.name			= "EOPPower",
		.gpio			= AT91_PIN_PB25,
		.default_trigger	= "EOPPower",
	}
};


/*
 * I2C devices
 */
static struct at24_platform_data at24c512 = {
	.byte_len	= SZ_512K / 8,
	.page_size	= 128,
	.flags		= AT24_FLAG_ADDR16,
};

static struct i2c_board_info __initdata ek_i2c_devices[] = {
	{
		I2C_BOARD_INFO("24c512", 0x50),
		.platform_data = &at24c512,
	},
	/* more devices can be added using expansion connectors */
};




static void __init lgate_gpio_init(void)
{
	// configure hardware rev bits as inputs with no pullups
	at91_set_gpio_input(AT91_PIN_PC7, 0);
	at91_set_gpio_input(AT91_PIN_PC8, 0);
	at91_set_gpio_input(AT91_PIN_PC9, 0);
	at91_set_gpio_input(AT91_PIN_PC10, 0);
	
	// configure configuration bits as inputs with weak pullups
	at91_set_gpio_input(AT91_PIN_PC29, 1);
	at91_set_gpio_input(AT91_PIN_PC30, 1);
	at91_set_gpio_input(AT91_PIN_PC31, 1);
	
	// configure display enabled input, with pullup
	at91_set_gpio_input(AT91_PIN_PA6, 1);
	// configure display rotate input, with pullup
	at91_set_gpio_input(AT91_PIN_PA5, 1);
	
}
/*
 * read the hw rev 
 */
static void lgate_get_hw_rev(void)
{
	u16 value = 0;
	
	value |= at91_get_gpio_value(AT91_PIN_PC7);
	value |= at91_get_gpio_value(AT91_PIN_PC8) << 1;
	value |= at91_get_gpio_value(AT91_PIN_PC9) << 2;
	value |= at91_get_gpio_value(AT91_PIN_PC10) << 3;
	
	if (value >= 8)	
    {
		// Configuration bits in upper byte of system rev	
		value |= at91_get_gpio_value(AT91_PIN_PC29) << 8; // not defined yet 
		value |= at91_get_gpio_value(AT91_PIN_PC30) << 9; // Energy chips present
		value |= at91_get_gpio_value(AT91_PIN_PC31) << 10; // ZigBee Present
    }
	system_rev = value;
}


static void __init ek_board_init(void)
{
	lgate_gpio_init();
    // get hw rev bits	
	// these show up in /proc/cpuinfo
	lgate_get_hw_rev();
	/* Serial */
	at91_add_device_serial();
	/* USB Host */
	at91_add_device_usbh(&ek_usbh_data);
	/* USB Device */
	at91_add_device_udc(&ek_udc_data);
	/* NAND */
	ek_add_device_nand();
	/* Ethernet */
	at91_add_device_eth(&ek_macb_data);
	/* MMC */
	at91_add_device_mmc(0, &ek_mmc_data);
	/* I2C */
	at91_add_device_i2c(ek_i2c_devices, ARRAY_SIZE(ek_i2c_devices));
	/* Compact Flash */
	at91_add_device_cf(&ek_cf_data);
	/* SSC (to AT73C213) */
	at73c213_set_clk(&at73c213_data);
	at91_add_device_ssc(AT91SAM9260_ID_SSC, ATMEL_SSC_TX);
	/* LEDs */
    if ((system_rev & 0xf) >= 9) // Lgate-50 Rev B?
    {
       ek_leds[0].gpio = AT91_PIN_PB19;  //Red LED moved to free modem control lines
       ek_leds[3].gpio = AT91_PIN_PB30;  //USB PWR moved to free modem control lines
       ek_leds[4].gpio = AT91_PIN_PB31;  //EOP PWR moved to free modem control lines
	}
	at91_gpio_leds(ek_leds, ARRAY_SIZE(ek_leds));
//	/* shutdown controller, wakeup button (5 msec low) */
//	at91_sys_write(AT91_SHDW_MR, AT91_SHDW_CPTWK0_(10) | AT91_SHDW_WKMODE0_LOW
//				| AT91_SHDW_RTTWKEN);

				
	system_serial_low = 0x600DF00D;
	system_serial_high = 0xDEADBEEF;
	/* SPI */
	at91_add_device_spi(ek_spi_devices, ARRAY_SIZE(ek_spi_devices));
}
MACHINE_START(AT91SAM9G20EK, "Atmel AT91SAM9G20 Locus Energy")
	/* Maintainer: Atmel */
	.phys_io	= AT91_BASE_SYS,
	.io_pg_offst	= (AT91_VA_BASE_SYS >> 18) & 0xfffc,
	.boot_params	= AT91_SDRAM_BASE + 0x100,
	.timer		= &at91sam926x_timer,
	.map_io		= ek_map_io,
	.init_irq	= ek_init_irq,
	.init_machine	= ek_board_init,
MACHINE_END
