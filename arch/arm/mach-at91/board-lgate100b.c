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
#include <linux/i2c-gpio.h>
#include <linux/i2c/at24.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/mtd/mtd.h>
#include <linux/spi/flash.h>

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
#include <mach/at91_rstc.h>

#include "sam9_smc.h"
#include "generic.h"

static void __init lgate_gpio_init(void)
{
	// configure hardware rev bits as inputs with no pullups
	at91_set_gpio_input(AT91_PIN_PC7, 0);
	at91_set_gpio_input(AT91_PIN_PC8, 0);
	at91_set_gpio_input(AT91_PIN_PC9, 0);
	at91_set_gpio_input(AT91_PIN_PC10, 0);
	
	// debug board option jumper 
	at91_set_gpio_input(AT91_PIN_PC25, 0);
	
	// configure configuration bits as inputs with no pullups
	at91_set_gpio_input(AT91_PIN_PC29, 0);
	at91_set_gpio_input(AT91_PIN_PC30, 0);
	at91_set_gpio_input(AT91_PIN_PC31, 0);
	
	// configure display enabled input, with pullup
	at91_set_gpio_input(AT91_PIN_PA6, 1);
	// configure display rotate input, with pullup
	at91_set_gpio_input(AT91_PIN_PA5, 1);
	
}
/*
 * read the hw rev 
 */
 // 0 - Lgate-101 Rev A & B
 // 1 - Lgate-101 Rev C & D
 
 // 4 - Lgate -90 Rev A
 
 // 8 - Lgate - 50 Rev A
 // 9 - Lgate - 50 Rev B
 static u16 lgate_type;
static void lgate_get_hw_rev(void)
{
	u16 value = 0;
	
	value |= at91_get_gpio_value(AT91_PIN_PC7);
	value |= at91_get_gpio_value(AT91_PIN_PC8) << 1;
	value |= at91_get_gpio_value(AT91_PIN_PC9) << 2;
	value |= at91_get_gpio_value(AT91_PIN_PC10) << 3;
	
	lgate_type = value;
	
	if (lgate_type >= 4)	
    {
		// Configuration bits in upper byte of system rev	
		value |= at91_get_gpio_value(AT91_PIN_PC29) << 8; // cell Present  
		value |= at91_get_gpio_value(AT91_PIN_PC30) << 9; // Energy chips present
		value |= at91_get_gpio_value(AT91_PIN_PC31) << 10; // ZigBee Present
		value |= at91_get_gpio_value(AT91_PIN_PC25) << 11; // Debug board option jumper 1 = not present
    }
	system_rev = value;
}

/*
 * LEDs
 */
#define RED_LED_INDEX 0
#define USB_PWR_INDEX 5
#define EOP_PWR_INDEX 6
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
	{	// yellow led
		.name			= "yellow",
		.gpio			= AT91_PIN_PA10,
		.default_trigger	= "none",
	},
	{	//blue led
		.name			= "blue",
		.gpio			= AT91_PIN_PA11,
		.default_trigger	= "none",
	},
	{	/* USB power */
		.name			= "USBPower",
		.gpio			= AT91_PIN_PB24,
		.default_trigger	= "none",
	},
	{	// EOP power also cell power on Lgate 90
		.name			= "EOPPower",
		.gpio			= AT91_PIN_PB25,
		.default_trigger	= "none",
	},
	{	//cell reset
		.name			= "CellReset",
		.gpio			= AT91_PIN_PC26,
		.default_trigger	= "none",
	},
};

extern struct i2c_gpio_platform_data i2c_pdata;

static void __init ek_map_io(void)
{
	/* Initialize processor: 18.432 MHz crystal */
	at91sam9260_initialize(18432000);

	/* DGBU on ttyS0. (Rx & Tx only) */
	at91_register_uart(0, 0, 0);

	lgate_gpio_init();
    // get hw rev bits	
	// these show up in /proc/cpuinfo
	lgate_get_hw_rev();

    if (lgate_type >= 4) // Lgate-50 & Lgate-90
    {
       ek_leds[RED_LED_INDEX].gpio = AT91_PIN_PB19;  //Red LED moved to free modem control lines
       ek_leds[USB_PWR_INDEX].gpio = AT91_PIN_PB30;  //USB PWR moved to free modem control lines
       ek_leds[EOP_PWR_INDEX].gpio = AT91_PIN_PB31;  //EOP PWR moved to free modem control lines
    	/* USART0 on ttyS1. (Rx, Tx, CTS, RTS, DTR, DSR, DCD, RI)  mode on Lgate 50*/
		at91_register_uart(AT91SAM9260_ID_US0, 1, ATMEL_UART_CTS | ATMEL_UART_RTS
			   | ATMEL_UART_DTR | ATMEL_UART_DSR | ATMEL_UART_DCD
			   | ATMEL_UART_RI);

 		/* USART1 on ttyS2. (Rx, Tx, RTS, CTS)  zigbee lgate-50 */
		at91_register_uart(AT91SAM9260_ID_US1, 2, ATMEL_UART_CTS | ATMEL_UART_RTS);
		/* USART2 on ttyS3. (Rx, Tx)  */ 
		at91_register_uart(AT91SAM9260_ID_US2, 3, 0);
		
		if (lgate_type == 4 || lgate_type == 5) // Lgate-90 only
		{
			/* USART3 on ttyS4. (Rx, Tx)  RS-485*/
			at91_set_gpio_output(AT91_PIN_PC27, 0); // RX not eanble, therfore a 0 is enable
			at91_register_uart(AT91SAM9260_ID_US3, 4, ATMEL_UART_RTS); // RTS controls TX enable
			/* USART4 on ttyS5. (Rx, Tx)  Energy Chip RS-232 ??*/
			at91_register_uart(AT91SAM9260_ID_US4, 5, 0);
			if (lgate_type == 4) // Lgate-50  Rev A only
			{
				// swap pins since RTC is wired backwards and eeprom not needed for now
				i2c_pdata.sda_pin = AT91_PIN_PA24;
				i2c_pdata.scl_pin = AT91_PIN_PA23;
			}
			
			// config ADC
			at91_set_B_periph(AT91_PIN_PC0, 0); // channel 0 used for temp in Lgate-90
		}
    }

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
//static struct at91_udc_data __initdata ek_udc_data = {
//	.vbus_pin	= AT91_PIN_PC5,
//	.pullup_pin	= 0,		/* pull-up driven by UDC */
//};

/*
 * SPI devices.
 */
static struct mtd_partition spi_boot_flash_partitions[] = {
	{
		.name = "bootstrap(spi)",
		.size = 0x0001000,
		.offset = 0,
		.mask_flags = MTD_CAP_ROM, // RO for now
	}, 
	{
		.name = "u-boot(spi)",
		.size = 0x0040000,
		.offset = 0x0010000,
		.mask_flags = MTD_CAP_ROM,
	}
};

static struct flash_platform_data spi_boot_flash_data = {
	.name = "m25p80",
	.parts = spi_boot_flash_partitions,
	.nr_parts = ARRAY_SIZE(spi_boot_flash_partitions),
	.type = "at25df041a",
};
static struct spi_board_info ek_spi_devices[] = {
	{
		/* the modalias must be the same as spi device driver name */
		.modalias = "m25p80", /* Name of spi_driver for this device */
		.chip_select = 1, 
		.max_speed_hz = 25000000,     /* max spi clock (SCK) speed in HZ */
		.bus_num = 0, /* Framework bus number */
		.platform_data = &spi_boot_flash_data,
//		.controller_data = &spi_flash_chip_info,
		.mode = SPI_MODE_3,
	},
	{	/* lcd */
		.modalias	= "ssd1805",
		.chip_select	= 2,  // was 0 in hw rev 0, moved for serial boot flash
		.max_speed_hz	= 5 * 1000 * 1000,
		.bus_num	= 0,
	},
	{	/* energy chip 1*/
		.modalias	= "spidev",
		.chip_select	= 0,
		.max_speed_hz	= 1 * 1000 * 1000,
		.bus_num	= 1,
		.mode = SPI_MODE_0,
	},
};


/*
 * MACB Ethernet device
 */
static struct at91_eth_data __initdata ek_macb_data = {
	.phy_irq_pin	= AT91_PIN_PA7,
	.is_rmii	= 1,
};


/*
 * NAND flash, this partition table is not used.  The partition table is passed in from U-boot as startup parameters
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
	.bus_width_16	= 0,
};

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
 * I2C devices
 */
static struct at24_platform_data at24aa02 = {
	.byte_len	= 2048 / 8,
	.page_size	= 8,
};

static struct i2c_board_info __initdata ek_i2c_devices[] = {
	{
		I2C_BOARD_INFO("24c02", 0x50),
		.platform_data = &at24aa02,
	},
	{
		I2C_BOARD_INFO("ds1374", 0x68),
	},
};


static void __init ek_board_init(void)
{

	/* Serial */
	at91_add_device_serial();
	/* I2C */
	at91_add_device_i2c(ek_i2c_devices, ARRAY_SIZE(ek_i2c_devices));
	/* USB Host */
	at91_add_device_usbh(&ek_usbh_data);
//	/* USB Device */
//	at91_add_device_udc(&ek_udc_data);
	/* NAND */
	ek_add_device_nand();
	/* Ethernet */
	at91_add_device_eth(&ek_macb_data);
	/* LEDs */     
	at91_gpio_leds(ek_leds, ARRAY_SIZE(ek_leds));
				
	system_serial_low = 0x600DF00D;
	system_serial_high = 0xDEADBEEF;
	/* SPI */
	at91_add_device_spi(ek_spi_devices, ARRAY_SIZE(ek_spi_devices));
	
//	printk(KERN_INFO "AT91 RSTC_SR: %8x\n", at91_sys_read(AT91_RSTC_SR));
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
