/*
 * include/configs/ccm2200.h
 *
 * Copyright (C) 2006 by Weiss-Electronic GmbH.
 * Copyright (C) 2010 by SWARCO Traffic Systems GmbH.
 * All rights reserved.
 *
 * @author:     Guido Classen <guido.classen@swarco.de>
 * @descr:      Configuation settings for the CCM2200 board.
 * @references: [1] at91rm9200dk.h
 *
 * See file CREDITS for list of people who contributed to this
 * project.
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
 *
 * @par Modification History:
 *   2006-04-26 gc: initial version (partly derived from at91rm9200dk.h by
 *                  Rick Bronson)
 */

#ifndef __CONFIG_H
#define __CONFIG_H

//#define CONFIG_AT91_LEGACY

/****************************************************************************
 * Constants for CCM2200 memory map
 ****************************************************************************/

/* CS0: NOR-Flash */
#define CCM2200_NOR_FLASH_CS		0
#define CCM2200_NOR_FLASH_PHYS          0x10000000
#define CCM2200_NOR_FLASH_SIZE          (16*1024*1024)

/* CS1: SD-RAM */
#define CCM2200_SDRAM_PHYS		0x20000000
#define CCM2200_SDRAM_SIZE		(32*1024*1024)

/* CS2: SRAM */
#define CCM2200_SRAM_CS                 2
#define CCM2200_SRAM_PHYS               0x30000000
#define CCM2200_SRAM_SIZE               (2*1024*1024)

/* CS3: NAND-Flash */
#define CCM2200_NAND_FLASH_CS           3
#define CCM2200_NAND_FLASH_PHYS         0x40000000
#define CCM2200_NAND_FLASH_SIZE         0x8000

/* CS4: external quad UART */
#define CCM2200_QUAD_UART_CS            4
#define CCM2200_QUAD_UART_PHYS          0x50000000
#define CCM2200_QUAD_UART_SIZE          0x20

/* CS5: digital output */
#define CCM2200_DIG_OUT_CS              5
#define CCM2200_DIG_OUT_PHYS            0x60000000
#define CCM2200_DIG_OUT_SIZE            0x1

/* CS6: extension board */
#define CCM2200_EXT_A_CS                6
#define CCM2200_EXT_A_PHYS              0x70000000

/* CS7: extension board */
#define CCM2200_EXT_B_CS                7
#define CCM2200_EXT_B_PHYS              0x80000000

#define CONFIG_CCM2200_ENV_IN_EEPROM         1

#define CONFIG_MISC_INIT_R		1	/* we have a misc_init_r() function */
#define BOARD_LATE_INIT                 1       /* Enables initializations before jumping to main loop */
#define CONFIG_SHOW_BOOT_PROGRESS       1               /* Show boot progress on LEDs   */


/****************************************************************************
 * general AT91RM9200 configuration (taken from at91rm9200dk.h)
 ****************************************************************************/

/* ARM asynchronous clock */
//#define AT91C_MAIN_CLOCK      184320000
//#define AT91C_MAIN_CLOCK      179712000
//#define AT91C_MAIN_CLOCK      165888000
#define AT91C_MAIN_CLOCK        158515200       /* from 18.432 MHz crystal (18432000 / ... * ...) */
//#define AT91C_MAIN_CLOCK      152064000
//#define AT91C_MAIN_CLOCK      138240000
//#define AT91C_MAIN_CLOCK      110592000

#define AT91C_MASTER_CLOCK	(AT91C_MAIN_CLOCK/2)	/* peripheral clock (AT91C_MASTER_CLOCK / 2) */

#define AT91_SLOW_CLOCK		32768	/* slow clock */

#define CONFIG_ARM920T		1	/* This is an ARM920T Core	*/
#define CONFIG_AT91RM9200	1	/* It's an Atmel AT91RM9200 SoC	*/
#define CONFIG_CCM2200		1	/* on an CCM2200 Board		*/
#undef  CONFIG_USE_IRQ			/* we don't need IRQ/FIQ stuff	*/
#define USE_920T_MMU		1

#define CONFIG_CMDLINE_TAG	1	/* enable passing of ATAGs	*/
#define CONFIG_SETUP_MEMORY_TAGS 1
#define CONFIG_INITRD_TAG	1

#ifndef CONFIG_SKIP_LOWLEVEL_INIT
#define CONFIG_SYS_USE_MAIN_OSCILLATOR		1
/* flash */
#define CONFIG_SYS_EBI_CFGR_VAL	0x00000000
//#define CONFIG_SYS_SMC_CSR0_VAL	0x00003284 /* 16bit, 2 TDF, 4 WS */
#define CONFIG_SYS_SMC_CSR0_VAL	0x0000328b /* CS0: Flash, 16bit, 2 TDF, 11 WS, BAT=1 */

/* clocks */
//#define CONFIG_SYS_PLLAR_VAL	0x20263E04 /* 179.712000 MHz for PCK */
#define CONFIG_SYS_PLLBR_VAL	0x10483E0E /* 48.054857 MHz (divider by 2 for USB) */
#define CONFIG_SYS_MCKR_VAL	0x00000102 /* PCK/2 = MCK Master Clock = 79.257600MHz from PLLA */

/* sdram */
#define CONFIG_SYS_PIOC_ASR_VAL	0xFFFF0000 /* Configure PIOC as peripheral (D16/D31) */
#define CONFIG_SYS_PIOC_BSR_VAL	0x00000000
#define CONFIG_SYS_PIOC_PDR_VAL	0xFFFF0000
#define CONFIG_SYS_EBI_CSA_VAL	0x00000002 /* CS1=CONFIG_SYS_SDRAM */
/* #define CONFIG_SYS_SDRC_CR_VAL	0x2188c155 /\* set up the CONFIG_SYS_SDRAM *\/ */
#define CONFIG_SYS_SDRC_CR_VAL	0x3399c255 /* set up the SDRAM, taken from UNC90 test */
				   /* COL=9, RAS=12, 4 Banks, CL=2, TWR=4, TRC=8*/

#define CONFIG_SYS_SDRAM	CCM2200_SDRAM_PHYS
#define CONFIG_SYS_SDRAM1	(CCM2200_SDRAM_PHYS + 0x80)
#define CONFIG_SYS_SDRAM_VAL	0x00000000 /* value written to CONFIG_SYS_SDRAM */
#define CONFIG_SYS_SDRC_MR_VAL	0x00000002 /* Precharge All */
#define CONFIG_SYS_SDRC_MR_VAL1	0x00000004 /* refresh */
#define CONFIG_SYS_SDRC_MR_VAL2	0x00000003 /* Load Mode Register */
#define CONFIG_SYS_SDRC_MR_VAL3	0x00000000 /* Normal Mode */
#define CONFIG_SYS_SDRC_TR_VAL	0x000002E0 /* Write refresh rate */
#else
#define CONFIG_SKIP_RELOCATE_UBOOT
#endif	/* CONFIG_SKIP_LOWLEVEL_INIT */

#define CONFIG_BAUDRATE			38400 /* 115200 */

/*
 * Hardware drivers
 */

/* define one of these to choose the DBGU, USART0  or USART1 as console */
#define CONFIG_AT91RM9200_USART
/* 2010-11-17 gc: ATMEL_USART isn't currently working on AT91RM9200 */
/* #define CONFIG_ATMEL_USART */
#undef  CONFIG_DBGU
#undef CONFIG_USART0
#undef CONFIG_USART1
#define CONFIG_USART2           /* 2006-07-04 gc: USART2 */

#undef	CONFIG_HWFLOW			/* don't include RTS/CTS flow control support	*/

#undef	CONFIG_MODEM_SUPPORT		/* disable modem initialization stuff */

//#define CONFIG_BOOTDELAY      3


/*
 * BOOTP options
 */
#define CONFIG_BOOTP_BOOTFILESIZE
#define CONFIG_BOOTP_BOOTPATH
#define CONFIG_BOOTP_GATEWAY
#define CONFIG_BOOTP_HOSTNAME


/****************************************************************************
 * configuration of available u-boot commands
 ****************************************************************************/
/*
 * Command line configuration.
 */
#include <config_cmd_default.h>

/* I2C commands */
#define CONFIG_CMD_I2C
/* network commands */
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_MII
/* NAND support */
#define CONFIG_CMD_NAND
#define CONFIG_CMD_NAND_LOCK_UNLOCK
/* UBI support (2010-11-18 gc: disable at moment, makes uboot to large >256k) */
/* #define CONFIG_CMD_UBI */
#ifdef CONFIG_CMD_UBI
#define CONFIG_CMD_MTDPARTS
#define	CONFIG_RBTREE
#define CONFIG_MTD_DEVICE		/* needed for mtdparts commands */
#define CONFIG_MTD_PARTITIONS
#endif

/* comand source replace to old autoscr command! */
#define CONFIG_CMD_SOURCE


/*
 * Size of malloc() pool
 */
#ifdef CONFIG_CMD_UBI
// 2010-11-18 gc: larger malloc len for ubi support
#define	CONFIG_SYS_MALLOC_LEN	(((CONFIG_ENV_SIZE+0x3ff)&~0x3ff) + 512*1024)
#else
#define CONFIG_SYS_MALLOC_LEN	(((CONFIG_ENV_SIZE+0x3ff)&~0x3ff) + 128*1024)
#endif

#define CONFIG_SYS_GBL_DATA_SIZE	128	/* size in bytes reserved for initial data */

#include <asm/arch/hardware.h>	/* needed for port definitions */

#define CONFIG_NR_DRAM_BANKS 1
#define PHYS_SDRAM		CCM2200_SDRAM_PHYS
#define PHYS_SDRAM_SIZE		CCM2200_SDRAM_SIZE

#define CONFIG_SYS_MEMTEST_START		PHYS_SDRAM
#define CONFIG_SYS_MEMTEST_END			CONFIG_SYS_MEMTEST_START + PHYS_SDRAM_SIZE - 262144


/* CONFIG_NET_MULTI => use u-boot's new network driver API */
#define CONFIG_NET_MULTI		1
#define CONFIG_NET_RETRY_COUNT		20
/* must not be set on CCM2200! */
#undef CONFIG_AT91C_USE_RMII

#define CONFIG_MII
#ifdef CONFIG_NET_MULTI
/****************************************************************************
 * network configuration (at91_emac.c driver)
 ****************************************************************************/
#define CONFIG_NET_MULTI		1
#define CONFIG_DRIVER_AT91EMAC		1
/* CCM2200 LXT971 PHY is configured at address 1 */
#define CONFIG_DRIVER_AT91EMAC_PHYADDR	1
#define CONFIG_SYS_RX_ETH_BUFFER	8
#define CONFIG_MII

#else
/****************************************************************************
 * network configuration (old at91rm9200/ether driver)
 ****************************************************************************/
/* this driver only supports 10Base-T Half-Duplex! */
#define CONFIG_DRIVER_ETHER		1
#endif



/* AC Characteristics */
/* DLYBS = tCSS = 250ns min and DLYBCT = tCSH = 250ns */
/* 
 * #define DATAFLASH_TCSS	(0xC << 16)
 * #define DATAFLASH_TCHS	(0x1 << 24)
 */

#undef CONFIG_HAS_DATAFLASH
/* 
 * #define CONFIG_SYS_SPI_WRITE_TOUT		(5*CONFIG_SYS_HZ)
 * #define CONFIG_SYS_MAX_DATAFLASH_BANKS		2
 * #define CONFIG_SYS_MAX_DATAFLASH_PAGES		16384
 * #define CONFIG_SYS_DATAFLASH_LOGIC_ADDR_CS0	0xC0000000	/\* Logical adress for CS0 *\/
 * #define CONFIG_SYS_DATAFLASH_LOGIC_ADDR_CS3	0xD0000000	/\* Logical adress for CS3 *\/
 */
/****************************************************************************
 * CCM2200 flash organisation
 ****************************************************************************/
#define CONFIG_SYS_FLASH_BASE			PHYS_FLASH_1
#define CONFIG_SYS_FLASH_CFI	/* The flash is CFI compatible  */

#define CONFIG_SYS_MAX_FLASH_BANKS		1
#define CONFIG_SYS_MAX_FLASH_SECT		256

#define PHYS_FLASH_1			CCM2200_NOR_FLASH_PHYS
#define PHYS_FLASH_SIZE			CCM2200_NOR_FLASH_SIZE  /* 2 megs main flash */

/* use this to switch on / off common CFI driver 
 *
 * CFI driver seams to be slower, but handles waiting until flash writing 
 * ready correctly!
 * AMD AM29LV160B (2MB) don't support buffered write!
 */
#define CONFIG_FLASH_CFI_DRIVER		/* Use common CFI driver        */

#ifdef CONFIG_FLASH_CFI_DRIVER
/* use hardware protection		*/
#define CONFIG_SYS_FLASH_PROTECTION		1
/* use buffered writes (20x faster)	*/
#define CONFIG_SYS_FLASH_USE_BUFFER_WRITE	1
/* print 'E' for empty sector on flinfo */
#define CONFIG_SYS_FLASH_EMPTY_INFO		1


/* We use u-boot common CFI driver for CCM2200 NOR flash */
#else

#define CONFIG_SYS_FLASH_ERASE_TOUT		(2*CONFIG_SYS_HZ) /* Timeout for Flash Erase */
#define CONFIG_SYS_FLASH_WRITE_TOUT		(2*CONFIG_SYS_HZ) /* Timeout for Flash Write */
#endif	/* CONFIG_FLASH_CFI_DRIVER */


#undef	CONFIG_ENV_IS_IN_DATAFLASH
/* #define CONFIG_ENV_OVERWRITE	1 */

#ifdef CONFIG_CCM2200_ENV_IN_EEPROM
#define CONFIG_ENV_IS_IN_EEPROM            1

#define CONFIG_SYS_REDUNDAND_ENVIRONMENT   1

/* environment starts at address 0x100, the first
 * 256 bytes of EEPROM are unused,
 * redundant copy starts at address 0x1000, the last
 * 256 bytes of EEPROM are also unused at present
 */
#define CONFIG_ENV_OFFSET                  0x0100
#ifdef CONFIG_SYS_REDUNDAND_ENVIRONMENT
/* 2010-12-01 gc: enable redundant environment */
#define CONFIG_ENV_SIZE                    0xF00
#define CONFIG_ENV_OFFSET_REDUND	   0x1000
//#define CONFIG_SYS_REDUNDAND_ENV_RECOVER	1
#define CONFIG_ENV_OFFSET_OLD              0x100
#define CONFIG_ENV_SIZE_OLD                0x1000
#else
#define CONFIG_ENV_SIZE                    0x1000
#endif

#else  /* CONFIG_CCM2200_ENV_IN_EEPROM */

#define CONFIG_ENV_IS_IN_FLASH		1
#ifdef CONFIG_SKIP_LOWLEVEL_INIT
#define CONFIG_ENV_ADDR			(PHYS_FLASH_1 + 0xe000)  /* between boot.bin and u-boot.bin.gz */
#define CONFIG_ENV_SIZE			0x2000 
#else
#define CONFIG_ENV_ADDR			(PHYS_FLASH_1 + 0x60000)  /* after u-boot.bin */
#define CONFIG_ENV_SIZE			0x10000 /* sectors are 64K here */
#endif	/* CONFIG_SKIP_LOWLEVEL_INIT */
#endif  /* CONFIG_CCM2200_ENV_IN_EEPROM */


/* #define CONFIG_SYS_LOAD_ADDR		0x21000000  /\* default load address *\/ */
#define CONFIG_SYS_LOAD_ADDR            0x20100000  /* default load address */

#ifdef CONFIG_SKIP_LOWLEVEL_INIT
#define CONFIG_SYS_BOOT_SIZE		0x6000 /* 24 KBytes */
#define CONFIG_SYS_U_BOOT_BASE		(PHYS_FLASH_1 + 0x10000)
#define CONFIG_SYS_U_BOOT_SIZE		0x10000 /* 64 KBytes */
#else
#define CONFIG_SYS_BOOT_SIZE		0x00 /* 0 KBytes */
#define CONFIG_SYS_U_BOOT_BASE		PHYS_FLASH_1
#define CONFIG_SYS_U_BOOT_SIZE		0x60000 /* 384 KBytes */
#endif	/* CONFIG_SKIP_LOWLEVEL_INIT */

#define CONFIG_SYS_BAUDRATE_TABLE	{ 115200, 19200, 38400, 57600, 9600 }

/* #define CONFIG_SYS_PROMPT		"U-Boot> "	/\* Monitor Command Prompt *\/ */
#define CONFIG_SYS_CBSIZE		256		/* Console I/O Buffer Size */
#define CONFIG_SYS_MAXARGS		16		/* max number of command args */
#define CONFIG_SYS_PBSIZE		(CONFIG_SYS_CBSIZE+sizeof(CONFIG_SYS_PROMPT)+16) /* Print Buffer Size */

#define CONFIG_SYS_HZ 1000

/*
 * AT91C_TC0_CMR is implicitly set to
 * AT91C_TC_TIMER_DIV1_CLOCK
 */
#define CONFIG_SYS_HZ_CLOCK (AT91C_MASTER_CLOCK/2)

#define CONFIG_STACKSIZE	(32*1024)	/* regular stack */
#define CONFIG_STACKSIZE_IRQ		(4 * 1024) /* Unsure if to big or to small*/
#define CONFIG_STACKSIZE_FIQ		(4 * 1024) /* Unsure if to big or to small*/

#ifdef CONFIG_USE_IRQ
#error CONFIG_USE_IRQ not supported
#endif

/****************************************************************************
 * CCM2200 specific configuration
 ****************************************************************************/

#undef CONFIG_SYS_PLLAR_VAL

#if AT91C_MAIN_CLOCK == 110592000

  #define CONFIG_SYS_PLLAR_VAL             0x20173F04      /* 110,59 MHz for PLLA */

#elif AT91C_MAIN_CLOCK == 138240000
   
  #define CONFIG_SYS_PLLAR_VAL             0x201D3F04      /* 138,24 MHz for PLLA */

#elif AT91C_MAIN_CLOCK == 152064000       
   
  #define CONFIG_SYS_PLLAR_VAL             0x20203F04      /* 119,8 MHz for PLLA */

#elif AT91C_MAIN_CLOCK == 158515200       
   
  #define CONFIG_SYS_PLLAR_VAL             0x202A3F05      /* 158,5 MHz for PLLA */
  
#elif AT91C_MAIN_CLOCK == 165888000       

  #define CONFIG_SYS_PLLAR_VAL             0x20233F04      /* 165,88 MHz for PLLA */

#elif AT91C_MAIN_CLOCK == 179712000       

//#define CONFIG_SYS_PLLAR_VAL             0x20263F04      /* 179,712 MHz for PLLA */
  #define CONFIG_SYS_PLLAR_VAL             0x2026BF04      /* 179,712 MHz for PLLA, 2006/11/03 df*/

#elif AT91C_MAIN_CLOCK == 184320000       

  #define CONFIG_SYS_PLLAR_VAL             0x2027BF04      /* 184,320 MHz for PLLA */
  
#else

#error unsupported AT91C_MAIN_CLOCK value!

#endif      



#define CONFIG_SYS_PROMPT                      "CCM2200 # "    /* Monitor Command Prompt */
#define CONFIG_IDENT_STRING     " SWARCO Traffic Systems GmbH"
#ifdef CONFIG_SYS_REDUNDAND_ENVIRONMENT
#define CONFIG_BOARD_VERSION	"-redu"
#endif

/****************************************************************************
 * CCM2200 external watchdog MAX6751
 ****************************************************************************/
#define CONFIG_HW_WATCHDOG              1


//#define CONFIG_SKIP_LOWLEVEL_INIT


/*
 * I2C-Bus (taken from eb_cpux9k2.h)
 */

#define CONFIG_SYS_I2C_SPEED		50000
#define CONFIG_SYS_I2C_SLAVE		0 		/* not used */

#undef CONFIG_HARD_I2C

#ifndef CONFIG_HARD_I2C
 /*
  * 2006-06-13 gc: SOFT_I2C seems to bee more reliable than HARD_I2C
  * on AT91RM9200 platform
  */
#define CONFIG_SOFT_I2C

/* Software  I2C driver configuration */

/* 
 * 2010-11-05 gc: fixed to use permament open drain mode (conforming
 * to I2C spec)
 */

#define CONFIG_SYS_I2C_INIT_BOARD	1 
//#define I2C_INIT	i2c_init_board();

#define I2C_SOFT_DECLARATIONS 	at91_pio_t *pio = (at91_pio_t *) AT91_PIO_BASE;
#define I2C_ACTIVE	do {} while (0)
/* we using open drain mode, so set port to "1" for tri-state */
#define I2C_TRISTATE	writel(AT91_PMX_AA_TWD, &pio->pioa.sodr);
#define I2C_READ	((readl(&pio->pioa.pdsr) & AT91_PMX_AA_TWD) != 0)

#define I2C_SDA(bit)						\
	if (bit)						\
		writel(AT91_PMX_AA_TWD, &pio->pioa.sodr);	\
	else							\
		writel(AT91_PMX_AA_TWD, &pio->pioa.codr);
#define I2C_SCL(bit)						\
	if (bit)						\
		writel(AT91_PMX_AA_TWCK, &pio->pioa.sodr);	\
	else							\
		writel(AT91_PMX_AA_TWCK, &pio->pioa.codr);

/* #define I2C_DELAY	udelay(2500000/CONFIG_SYS_I2C_SPEED) */
#define I2C_DELAY	udelay(5)	/* 1/4 I2C clock duration */
//#define I2C_DELAY	udelay(3)	/* 1/4 I2C clock duration */

#endif	/* CONFIG_HARD_I2C */

/* I2C-RTC */

#ifdef CONFIG_CMD_DATE
#define CONFIG_RTC_DS1337
#define CONFIG_SYS_I2C_RTC_ADDR	0x68
#endif

/* I2C-EEPROM */

#define CONFIG_SYS_I2C_EEPROM_ADDR_LEN	2
#define CONFIG_SYS_I2C_EEPROM_ADDR	0x50
/* The  has a PAGE WRITE up to 32 bytes, so we set this to 2^5 */
#define	CONFIG_SYS_EEPROM_PAGE_WRITE_BITS  5
/* The ST/Motorola 24C64 has 32 byte page write mode using last 5 bits
 * of the address  */
#define CONFIG_SYS_EEPROM_PAGE_WRITE_BITS 5 
/* and takes up to 10 msec (for worst case set delay to 20...) */
#define CONFIG_SYS_EEPROM_PAGE_WRITE_DELAY_MS	20   

/* #define CONFIG_SOFT_I2C_READ_REPEATED_START */


/* FLASH organization */

/* NAND */

#define CONFIG_SYS_NAND_MAX_CHIPS	1
#define CONFIG_SYS_MAX_NAND_DEVICE	1
#define CONFIG_SYS_NAND_BASE		CCM2200_NAND_FLASH_PHYS
#define CONFIG_SYS_NAND_DBW_8		1

#define CONFIG_SYS_64BIT_VSPRINTF	1
#define CONFIG_NAND_LOCK

#define CCM2200_NAND_FLASH_DAT  (0x0000) /* Data Read/Write */
#define CCM2200_NAND_FLASH_ADR  (0x2000) /* Adr Write */
#define CCM2200_NAND_FLASH_CMD  (0x4000) /* Cmd Write */

/* testweise nand_atmel driver */
#define CONFIG_NAND_ATMEL
/* our ALE is AD21 */
#define CONFIG_SYS_NAND_MASK_ALE	CCM2200_NAND_FLASH_ADR
/* our CLE is AD22 */
#define CONFIG_SYS_NAND_MASK_CLE	CCM2200_NAND_FLASH_CMD
#undef CONFIG_SYS_NAND_ENABLE_PIN	
/* not necessary, we have an own at91_nand_ready() function for CCM2200 */
/* #define CONFIG_SYS_NAND_READY_PIN	AT91_PIO_PORTA, 19 */



/************************************************************
 * CCM2200 status led 
 ************************************************************/
#define CONFIG_STATUS_LED		1 /* enable status led driver */
#define CONFIG_BOARD_SPECIFIC_LED	1

#ifdef   CONFIG_STATUS_LED



#define STATUS_LED_BIT          (1 << 0)                /* LED[0] */
#define STATUS_LED_STATE        STATUS_LED_BLINKING
#define STATUS_LED_BOOT_STATE   STATUS_LED_OFF
#define STATUS_LED_PERIOD       (CONFIG_SYS_HZ / 2)
#define STATUS_LED_ACTIVE       1
#define STATUS_LED_BOOT         0                       /* boot LED */

#define STATUS_LED_BIT1         (1 << 1)                /* LED[1] */
#define STATUS_LED_STATE1       STATUS_LED_OFF
#define STATUS_LED_PERIOD1      (CONFIG_SYS_HZ / 2)
#define STATUS_LED_ACTIVE       1
#define STATUS_LED_RED          1                       /* Error LED */

#define	STATUS_LED_BIT2		(1 << 2)                /* LED[2] */
#define	STATUS_LED_STATE2	STATUS_LED_OFF
#define	STATUS_LED_PERIOD2	(CONFIG_SYS_HZ / 2)            /* ca. 1 Hz */
#define	STATUS_LED_YELLOW	2                       /* info LED */

#define	STATUS_LED_BIT3		(1 << 3)                /* LED[3] */
#define	STATUS_LED_STATE3	STATUS_LED_OFF
#define	STATUS_LED_PERIOD3	(CONFIG_SYS_HZ / 2)            /* ca. 1 Hz */
#define	STATUS_LED_GREEN	3                       /* Info LED */


#ifndef __ASSEMBLY__

/* 
 * now in status_led.h
 * typedef unsigned led_id_t;
 * extern void __led_init (led_id_t mask, int state);
 * extern void __led_set (led_id_t mask, int state);
 * extern void __led_toggle (led_id_t mask);
 */

#endif
#endif /* CONFIG_STATUS_LED */

/****************************************************************************
 * USB-Configuration
 ****************************************************************************/
#define CONFIG_CMD_USB
#define CONFIG_USB_ATMEL	1
#define CONFIG_USB_OHCI_NEW	1
//#define CONFIG_USB_OHCI		1
#define CONFIG_DOS_PARTITION	1
#define CONFIG_USB_STORAGE	1
#define CONFIG_CMD_FAT		1

//2006-06-04 gc: this is valid for UNC90, but not for CCM2200
//#define CONFIG_AT91C_PQFP_UHPBUG        1  /* We are not a pqfp chip... but this solves the CCM2200 hardware bug (missing pull-down resistors on the un-connected USB interface) */

#undef CONFIG_SYS_USB_OHCI_BOARD_INIT

#define CONFIG_SYS_USB_OHCI_CPU_INIT		1
#define CONFIG_SYS_USB_OHCI_REGS_BASE		AT91_USB_HOST_BASE
#define CONFIG_SYS_USB_OHCI_SLOT_NAME		"at91rm9200"
#define CONFIG_SYS_USB_OHCI_MAX_ROOT_PORTS	15

/************************************************************
 * CCM2200 default environment settings 
 ************************************************************/

/* 2006-04-28 gc: Macros for updating and booting NAND-Flash */
#define DEFAULT_NAND_ENV_MACROS                                                                                                                                  \
        "update_rootfs_tftp_nand=tftp $(loadaddr) $(nand_rimg); nand erase clean; nand write.jffs2 $(loadaddr) 00000000 $(filesize)\0" \
        "s_bootargs_nand=setenv root /dev/mtdblock5;setenv rootfstype jffs2\0"                   \
        "boot_nand=run s_bootargs_nand; run gfk boot_linux\0"


#if 0
/* 2007-02-09 gc: removed settings for rootfs in NOR Flash
 *                (only usable on boards with 16MB NOR Flash
 */
#define DEFAULT_NOR_ROOTFS_MACROS                                       \
        "cfr=erase 10340000 1063ffff\0"					\
        "gur=usb reset;fatload usb 0 $(loadaddr) $(rimg)\0"		\
        "gtr=tftp $(loadaddr) $(rimg)\0"				\
        "smtd=setenv root /dev/mtdblock3;setenv rootfstype jffs2\0"     \
        "update_rootfs_usb=run gur cfr wfr\0"				\
        "rimg=rootfs-ccm2200.jffs2\0"					\
        "wfr=cp.b $(loadaddr) 10340000 $(filesize)\0"                   \
         "update_rootfs_tftp=run gtr cfr wfr\0"				
#endif

#define CONFIG_EXTRA_ENV_SETTINGS					\
        "ipaddr=192.168.42.30\0"					\
        "serverip=192.168.42.1\0"					\
        "netmask=255.255.255.0\0"					\
        "dhcp=no\0"							\
        "link=auto\0"							\
        "loadaddr=20100000\0"						\
        "kimg=uImage-ccm2200dev.bin\0"					\
        "uimg=u-boot-ccm2200dev.bin\0"					\
        "npath=/exports/nfsroot-ccm2200dev\0"				\
        "snfs=setenv root nfs;setenv bootargs_ext nfsroot=$(serverip):$(npath)\0"    \
        "gtk=tftp $(loadaddr) $(kimg)\0"				\
        "gtu=tftp $(loadaddr) $(uimg)\0"				\
        "gfk=cp.w 10040000 $(loadaddr) 180000\0"			\
        "guk=usb reset;fatload usb 0 $(loadaddr) $(kimg)\0"		\
        "gub=usb reset;fatload usb 0 $(loadaddr) ccm2200.bin\0"		\
        "guu=usb reset;fatload usb 0 $(loadaddr) $(uimg)\0"		\
        "wfk=cp.b $(loadaddr) 10040000 $(filesize)\0"                   \
        "wfu=cp.b $(loadaddr) 10000000 $(filesize)\0"                   \
        "cfk=erase 10040000 1033ffff\0"					\
        "cfu=protect off 1:0-7;erase 10000000 1003ffff\0"		\
        "boot_flash=run smtd gfk boot_linux\0"				\
        "boot_net=run snfs gtk boot_linux\0"				\
        "boot_usb=usb start;run gub;source\0"				\
        "update_kernel_tftp=run gtk cfk wfk\0"				\
        "update_uboot_tftp=run gtu cfu wfu\0"				\
        "update_kernel_usb=run guk cfk wfk\0"				\
        "update_uboot_usb=run guu cfu wfu\0"				\
        "bootcmd_update=run boot_usb\0"					\
	"cpuclock=ccm2200 clock 5 43 1 2\0"				\
        "set_bootargs=setenv bootargs ip=$(ipaddr):$(serverip)::$(netmask)::eth0:off root=$(root) rootfstype=$(rootfstype) $(bootargs_ext)\0"                   \
	"boot_linux=run set_bootargs cpuclock;bootm\0"				\
        DEFAULT_NAND_ENV_MACROS						\
        ""


/* Default environment settings */
#define CONFIG_BOOTARGS                 ""
#define CONFIG_BOOTCOMMAND              "run boot_nand"
/* enable preboot variable      */
#define CONFIG_PREBOOT			"nand unlock" 

/* 2006-07-27 gc: set bootdelay to 0 to increase booting speed */
#define CONFIG_BOOTDELAY                0
//#define CONFIG_BOOTDELAY                2
/* If this option is defined, you can stop the autoboot process by
 * hitting a key even in that case when "bootdelay" has been set to
 * 0. You can set "bootdelay" to a negative value to prevent the check
 * for console input.
 */
#define CONFIG_ZERO_BOOTDELAY_CHECK     1


/* Testweise */
// 2010-11-12 gc: @todo print_cpuinfo() must be implemented for at91rm9200
//#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DISPLAY_BOARDINFO

#endif	

 
