/*
 * board/ccm2200/ccm2200.c
 *
 * Copyright (C) 2006 by Weiss-Electronic GmbH.
 * Copyright (C) 2010 by SWARCO Traffic Systems GmbH.
 * All rights reserved.
 *
 * @author:     Guido Classen <guido.classen@swarco.de>
 * @descr:      Initialization of the CCM2200 board.
 * @references: [1] unc90/unc90.c
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
 *   2006-04-28 gc: release 1.01: machine-type will be read from environemnt
 *                                string machine_arch_type
 *   2006-04-26 gc: release 1
 *   2006-03-29 FS-Forth: initial version
 */

#include <common.h>
#include <command.h>
#include <nand.h>		     /* nand_info[0] */
#include <linux/string.h>            /* strcmp */
#include <asm/arch/hardware.h>
#include <asm/io.h>
#include <asm/arch/at91_pio.h>
#include <asm/arch/at91_pmc.h>
#include <asm/arch/at91_util.h>
#include <asm/arch/at91rm9200.h>


/**************************************************************************
 * checkboard() (called if CONFIG_DISPLAY_BOARDINFO set)
 **************************************************************************/
int checkboard (void)
{
	puts("Board: CCM2200\n");
	return 0;
}


/*****************************************************************************
 * Software I2C initialization
 ****************************************************************************/

#ifdef CONFIG_SOFT_I2C
static void i2c_send_start(void)
{
	I2C_SOFT_DECLARATIONS	/* intentional without ';' */

	I2C_DELAY;
	I2C_SDA(1);
	I2C_ACTIVE;
	I2C_DELAY;
	I2C_SCL(1);
	I2C_DELAY;
	I2C_SDA(0);
	I2C_DELAY;
}

void i2c_init_board(void)
{
	const u32 pins = AT91_PMX_AA_TWD | AT91_PMX_AA_TWCK;
	at91_pmc_t *pmc = (at91_pmc_t *) AT91_PMC_BASE;
	I2C_SOFT_DECLARATIONS	/* intentional without ';' */
	int j;

	writel(1 << AT91_ID_PIOA, &pmc->pcer);
        writel(pins, &pio->pioa.ifdr);	/* disable glitch filter */
	writel(pins, &pio->pioa.idr);	/* disable interrupt for this PINs */
	writel(pins, &pio->pioa.pudr);	/* disable internal pull up */
	writel(pins, &pio->pioa.per);	/* configure GPIO to PIO controller */
	writel(pins, &pio->pioa.sodr);	/* output 1 */
	writel(pins, &pio->pioa.mder);	/* multi drive enable (open drain) */
	writel(pins, &pio->pioa.oer);	/* enable output (open drain mode) */

	/* 2010-11-08 gc: modified I2C bus reset. To prevent
	 * corruption of serial eeprom content clock out only as many
	 * clock cylces as needed until slave release the i2c_dat wire.
	 * Then reset slave's state machines using a START symbol
	 */
        /* __led_set(0x0ff0, 1); */
	__led_set(0x0001, 1);

	I2C_SCL(1);
	I2C_SDA(1);
#ifdef	I2C_INIT
	I2C_INIT;
#endif
	I2C_TRISTATE;

	j = 0;
	/* check if i2c_dat is stuck to low by any slave and send
	 * clocks to slave until it releases i2c_dat to high
	 */
	while ((I2C_READ == 0) && j < 100) {
		I2C_SCL(0);
		I2C_DELAY;
		I2C_DELAY;
		I2C_SCL(1);
		I2C_DELAY;
		I2C_DELAY;
		++j;
	}

	/* send 9 start symbols to reset slaves */
	for(j = 0; j < 9; j++) {
		i2c_send_start();
		I2C_DELAY;
		I2C_DELAY;
		I2C_DELAY;
		I2C_DELAY;
	}
	/* don't send stop at initialization (EEPROM may write
	 * corrupted data on stop cycle */
	/* send_stop(); 
	 */
	I2C_TRISTATE;
}

#endif

#define pio ((at91_pio_t *) AT91_PIO_BASE)

/*************************************************************************
 *  hw_watchdog_reset
 *
 *	This routine is called to reset (keep alive) the watchdog timer
 *
 ************************************************************************/
#if defined(CONFIG_HW_WATCHDOG)
static struct at91_pio_pins watchdog_trigger = {
        &pio->pioa, 1 << 21
};

ulong get_timer_raw(void);


/* we can not call udelay from hw_watchdog_reset() (endless recursion) */
static long udelay_loop_dummy;
static void udelay_loop (unsigned long usec)
{
	int j;
	
	for (j=0; j < 28*usec; ++j)
	{
		udelay_loop_dummy = 1+j;
	}
}


void hw_watchdog_reset(void)
{
        static ulong last_reset = 0;
      
	ulong current_time = get_timer_raw();
        /* do reset only once all 500msec */
        if ((ulong) (current_time - last_reset) 
	    > (ulong) (CONFIG_SYS_HZ_CLOCK / 2)) {
                
                /* assert reset signal to external watchdog */
                at91_pio_clear_all_pins(&watchdog_trigger);
		udelay_loop(5);
                at91_pio_set_all_pins(&watchdog_trigger);
                /* blink with LED to indicate the running watchdog */
                __led_toggle(0x000c);
                last_reset = current_time;
        }
}
#endif

int ccm2200_change_master_clock_freq(int css, int div, int pres );

#ifdef CONFIG_STATUS_LED
# include <status_led.h>
#endif /* CONFIG_STATUS_LED */

// 2006-03-31 gc: 
static int current_led_state;
#define CCM2200_PIOD_LED_MASK           0x0000ffff
#define CCM2200_PIOD_LED_SHIFT          0

static inline void set_led(unsigned led_output)
{
        current_led_state = led_output;
	writel(led_output, &pio->piod.codr);
	writel(~led_output & 0xffff, &pio->piod.sodr);
}


void __led_init(led_id_t mask, int state)
{
	/* configure LEDs, keep it off */
	writel(CCM2200_PIOD_LED_MASK, &pio->piod.sodr);  /* set output high */
	writel(CCM2200_PIOD_LED_MASK, &pio->piod.oer);   /* set to output */
	writel(CCM2200_PIOD_LED_MASK, &pio->piod.per);   /* enable PIO function */
	writel(CCM2200_PIOD_LED_MASK, &pio->piod.pudr);  /* disable pullups */
        current_led_state = 0;
        __led_set(mask, state);
}

void __led_set(led_id_t mask, int state)
{
        mask &= CCM2200_PIOD_LED_MASK;
        if (state) {
		writel(mask, &pio->piod.codr);
                current_led_state |= mask;
        } else {
		writel(mask, &pio->piod.sodr);
                current_led_state &= ~mask;
        }
}

void __led_toggle(led_id_t mask)
{
        current_led_state ^= mask & CCM2200_PIOD_LED_MASK; 
        set_led(current_led_state);
}


/* SCONF0: PIOA27 */
/* SCONF1: PIOA28 */
#define CCM2200_PIOA_SCONF0_1_MASK      (1<<27 | 1<<28)
#define CCM2200_PIOA_SCONF0_1_SHIFT     27

/* preliminary for testing */
/* SCONF2: PIOC5 */
#define CCM2200_PIOC_SCONF2_MASK        (1<<5)
#define CCM2200_PIOC_SCONF2_SHIFT       (5-2)

/* preliminary for testing */
/* SCONF3: PB28 */
#define CCM2200_PIOB_SCONF3_MASK        (1<<28)
#define CCM2200_PIOB_SCONF3_SHIFT       (28-3)

/**
 * misc_init_r: - misc initialisation routines
 *
 * @return: sconf0...sconf3 in bits 0...3
 *          a DIP switch in on position is indicated by a 0 bit!
 */
static inline unsigned ccm2200_get_sconf_input(void)
{
        return ( (readl(&pio->pioa.pdsr) & CCM2200_PIOA_SCONF0_1_MASK) 
                 >> CCM2200_PIOA_SCONF0_1_SHIFT )
                | ( (readl(&pio->pioc.pdsr) & CCM2200_PIOC_SCONF2_MASK) 
                   >> CCM2200_PIOC_SCONF2_SHIFT )
                | ( (readl(&pio->piob.pdsr) & CCM2200_PIOB_SCONF3_MASK) 
                   >> CCM2200_PIOB_SCONF3_SHIFT );
}



#ifdef CONFIG_SHOW_BOOT_PROGRESS

/**
 * show_boot_progress: - indicate state of the boot process
 *
 * @param status: Status number - see README for details.
 *
 * We use 5 LEDs from the CCM2200 frontpanel for the boot progress
 * four from them displaying the status code in binary form,
 * the fifth is used to indicate error conditions (negative status code)
 */
void show_boot_progress (int status)
{
        int status_code = status;

#if 0
        if (status == 15) {
                printf( "changing master clock frequency for linux...\n" );
	
		if (ccm2200_change_master_clock(5, 43,	1, 2) != 0) {
                        printf("ccm2200_change_master_clock() error\n");
                }
        }
#endif

#if 0
        if (status == 15) {
                printf( "changing master clock frequency for linux...\n" );
	
                if (ccm2200_change_master_clock_freq(
                            AT91C_PMC_CSS_PLLA_CLK,
                            CCM2200_MSTR_CLK_DIV_LINUX - 1,
                            AT91C_PMC_PRES_CLK ) != 0 ) {
                        printf("change_master_clock_freq() error\n");
                }
        }
#endif

        if (status_code < 0) {
                status_code = - status_code;
        }

        //__led_set(0xf000, status_code << 12);
  
	if ( status < 0 ) {
#if defined(CONFIG_STATUS_LED) && defined(STATUS_LED_RED)
		int i;
		/* 2010-11-18 gc: STATUS_LED_BLINKING ist currently
		 * not implemented on ARM */
		/* status_led_set(STATUS_LED_RED, STATUS_LED_BLINKING); */
                for (i=0; i<10; i++) {
			printf ("*** i=(%d) ***\n", i);
                        status_led_set(STATUS_LED_RED, STATUS_LED_OFF);
                        udelay(300000);
                        status_led_set(STATUS_LED_RED, STATUS_LED_ON);
                        udelay(300000);
                }
#endif /* CONFIG_STATUS_LED && STATUS_LED_RED */
		printf ("*** Error show_boot_progress(%d) ***\n", status);
	}

}
#endif

/*****************************************************************************
 * change master clock frequency
 ****************************************************************************/

#define	MAX_LOOPS_FOR_MCKRDY	100000000
#define AT91_SLOW_CLOCK         32768
#define AT91_MAIN_CLOCK         18432000
static unsigned long at91_master_clock = AT91C_MASTER_CLOCK;
static unsigned long at91_cpu_clock    = AT91C_MAIN_CLOCK;

/* atmel_usart.c callback function */
unsigned long get_mck_clk_rate(void)
{
	return at91_master_clock;
}

void serial_setbrg(void);	/* in atmel_usart.c */
static void ccm2200_update_uart_clock(void)
{
	serial_setbrg();
}

/* 2010-11-16 gc: missing definitions for AT91RM9200 @todo find some! */
#define AT91C_CKGR_DIVA		((unsigned int) 0xFF  <<  0)	/* (CKGR) Divider Selected */
#define AT91C_CKGR_MULA		((unsigned int) 0x7FF << 16)	/* (CKGR) PLL A Multiplier */
#define AT91C_CKGR_OUTA_0	((unsigned int) 0x0   << 14)	/* (CKGR) Please refer to the PLLA datasheet */
#define AT91C_CKGR_OUTA_1	((unsigned int) 0x1   << 14)	/* (CKGR) Please refer to the PLLA datasheet */
#define AT91C_CKGR_OUTA_2	((unsigned int) 0x2   << 14)	/* (CKGR) Please refer to the PLLA datasheet */
#define AT91C_CKGR_OUTA_3	((unsigned int) 0x3   << 14)	/* (CKGR) Please refer to the PLLA datasheet */

#define AT91C_CKGR_DIVB		((unsigned int) 0xFF  <<  0)	/* (CKGR) Divider Selected */
#define AT91C_CKGR_MULB		((unsigned int) 0x7FF << 16)	/* (CKGR) PLL B Multiplier */
#define AT91C_CKGR_SRCA		((unsigned int) 0x1   << 29)	/* (CKGR) PLL A Source */

static void measure_and_set_clocks(int print)
{
	at91_pmc_t *pmc = (at91_pmc_t *) AT91_PMC_BASE;
	//AT91PS_PMC pmc = (AT91PS_PMC) AT91C_BASE_PMC;
	//AT91PS_CKGR ckgr = (AT91PS_CKGR) AT91C_BASE_CKGR;
	unsigned long mainf_counter = readl(&pmc->mcfr) & AT91_PMC_MCFR_MAINF_MASK;
	unsigned long main_clock = mainf_counter * (AT91_SLOW_CLOCK / 16);
	unsigned long main_clock_measured;
	unsigned long plla_clock;
	unsigned long pllb_clock;

	unsigned long master_clock;
	unsigned long cpu_clock;

	main_clock_measured = main_clock;

	if (main_clock_measured > (unsigned long) (AT91_MAIN_CLOCK * 0.99)
	    && main_clock_measured < (unsigned long) (AT91_MAIN_CLOCK * 1.01)) {
		main_clock = AT91_MAIN_CLOCK;
	}


	{
		unsigned div_a = readl(&pmc->pllar) & AT91C_CKGR_DIVA;
		unsigned mul_a = (readl(&pmc->pllar) & AT91C_CKGR_MULA) >> 16;
			
		if (div_a == 0 || mul_a == 0) {
			plla_clock = 0;
		} else {
			plla_clock = (((unsigned long long) main_clock)
				      *  (mul_a+1) / div_a);
		}
	}
			
	{
		unsigned div_b = readl(&pmc->pllbr) & AT91C_CKGR_DIVB;
		unsigned mul_b = (readl(&pmc->pllbr) & AT91C_CKGR_MULB) >> 16;
		
		if (div_b == 0 || mul_b == 0) {
			pllb_clock = 0;
		} else {
			pllb_clock = (((unsigned long long) main_clock)
				      *  (mul_b+1) / div_b);
		}
	}

	switch (readl(&pmc->mckr) & AT91_PMC_CSS) {
	case AT91_PMC_MCKR_CSS_SLOW:
		master_clock = AT91_SLOW_CLOCK;
		break;

	case AT91_PMC_MCKR_CSS_MAIN:
		master_clock = main_clock;
		break;

	case AT91_PMC_MCKR_CSS_PLLA:
		master_clock = plla_clock;
		break;

	case AT91_PMC_MCKR_CSS_PLLB:
		master_clock = pllb_clock;
		break;

	default:
		master_clock = 0;
	}


	switch (readl(&pmc->mckr) & AT91_PMC_MCKR_PRES_MASK) {
	default:
	case AT91_PMC_MCKR_PRES_1:
		break;
	case AT91_PMC_MCKR_PRES_2:
		master_clock >>= 1;
		break;

	case AT91_PMC_MCKR_PRES_4:
		master_clock >>= 2;
		break;

	case AT91_PMC_MCKR_PRES_8:
		master_clock >>= 3;
		break;

	case AT91_PMC_MCKR_PRES_16:
		master_clock >>= 4;
		break;

	case AT91_PMC_MCKR_PRES_32:
		master_clock >>= 5;
		break;

	case AT91_PMC_MCKR_PRES_64:
		master_clock >>= 6;
		break;
	}

	cpu_clock = master_clock;

	switch (readl(&pmc->mckr) & AT91_PMC_MCKR_MDIV_MASK) {
	default:
	case AT91_PMC_MCKR_MDIV_1:
		break;

	case AT91_PMC_MCKR_MDIV_2:
		master_clock >>= 1;
		break;

	case AT91_PMC_MCKR_MDIV_3:
		master_clock /= 3;
		break;

	case AT91_PMC_MCKR_MDIV_4:
		master_clock >>= 2;
		break;
	}

	/* 2006-10-17 gc: save measured clock values in global variables */
	at91_master_clock = master_clock;
	at91_cpu_clock   = cpu_clock;
	ccm2200_update_uart_clock();

	if (print) {
		printf("CCM2200 Main   Clock: %9lu Hz (measured: %9lu Hz)\n", 
		       main_clock, main_clock_measured);
		printf("        PLLA   Clock: %9lu Hz (PLLAR: 0x%08x)\n", 
		       plla_clock, readl(&pmc->pllar));
		printf("        PLLB   Clock: %9lu Hz\n", pllb_clock);
		printf("        CPU    Clock: %9lu Hz\n", cpu_clock);
		printf("        Master Clock: %9lu Hz\n", master_clock);
	}
}


/* switch to clock source */
static inline void at91_pmc_master_clock_select(u32 clock_source)
{
	register at91_pmc_t *pmc = (at91_pmc_t *) AT91_PMC_BASE;
	u32 mckr_old = readl(&pmc->mckr);
	/* Write MCKR in two accesses only if css value has changed!
	 * See AT91RM9200 errata sheet, errata 26-28.
	 */
		
	clock_source &= AT91_PMC_MCKR_CSS_MASK;
	if ((mckr_old & AT91_PMC_MCKR_CSS_MASK) != clock_source) {
		unsigned timeout = 1000;

		/* Write MCKR in two accesses. See AT91RM9200 errata sheet */
/* 		pmc->PMC_MCKR = mckr_old & AT91C_PMC_MDIV; */
/* 		pmc->PMC_MCKR = (mckr_old & (AT91C_PMC_MDIV|AT91C_PMC_PRES)) */
/* 			| clock_source; */

		writel((mckr_old & ~AT91_PMC_MCKR_CSS_MASK) | clock_source, 
		       &pmc->mckr);

		while (!(readl(&pmc->sr) & AT91_PMC_IXR_MCKRDY)) {
			udelay(50);
			if (!--timeout) break;
		}
	}
}

static inline void at91_pmc_master_clock_prescaler(u32 prescaler)
{
	register at91_pmc_t *pmc = (at91_pmc_t *) AT91_PMC_BASE;
	u32 mckr_old = readl(&pmc->mckr);
	/* Write MCKR in two accesses only if css value has changed!
	 * See AT91RM9200 errata sheet, errata 26-28.
	 */
		
	prescaler &= AT91_PMC_MCKR_PRES_MASK;
	if ((mckr_old & AT91_PMC_MCKR_PRES_MASK) != prescaler) {
		unsigned timeout = 1000;

		writel((mckr_old & ~AT91_PMC_MCKR_PRES_MASK) | prescaler,
		       &pmc->mckr);
			
		while (!(readl(&pmc->sr) & AT91_PMC_IXR_MCKRDY)) {
			udelay(50);
			if (!--timeout) break;
		}
	}
}

static inline void at91_pmc_master_clock_divider(u32 divider)
{
	register at91_pmc_t *pmc = (at91_pmc_t *) AT91_PMC_BASE;
	u32 mckr_old = readl(&pmc->mckr);
	/* Write MCKR in two accesses only if css value has changed!
	 * See AT91RM9200 errata sheet, errata 26-28.
	 */
		
	divider &= AT91_PMC_MCKR_MDIV_MASK;
	if ((mckr_old & AT91_PMC_MCKR_MDIV_MASK) != divider) {
		unsigned timeout = 1000;
		writel((mckr_old & ~AT91_PMC_MCKR_MDIV_MASK) 
		       | divider, &pmc->mckr);

		/* MCKRDY may not set, see errata 27 => use timeout */
		while (!(readl(&pmc->sr) & AT91_PMC_IXR_MCKRDY)) {
			udelay(50);
			if (!--timeout) break;
		}
	}
}


static inline void at91_ckgr_plla(u32 div, u32 mul)
{
	register at91_pmc_t *pmc = (at91_pmc_t *) AT91_PMC_BASE;
//	register AT91S_CKGR *ckgr = (AT91S_CKGR*) AT91C_BASE_CKGR;
	unsigned div_a = readl(&pmc->pllar) & AT91C_CKGR_DIVA;
	unsigned mul_a = (readl(&pmc->pllar) & AT91C_CKGR_MULA) >> 16;
	unsigned long plla_clock = (((unsigned long long) AT91_MAIN_CLOCK)
				    *  mul / div);

	--mul;
	div &= AT91C_CKGR_DIVA;

	if (div_a != div || mul != mul_a) {
		u32 pllar;
		unsigned timeout = 5000;

		mul <<= 16;
		mul &= AT91C_CKGR_MULA;

		/* bit 29 is always set */
		pllar = div | mul | AT91C_CKGR_SRCA;
		if (plla_clock > 155000000) 
			pllar |= AT91C_CKGR_OUTA_2;
		else
			pllar |= AT91C_CKGR_OUTA_0;

		pllar |= 63 << 8; /* AT91C_CKGR_PLLACOUNT */

		writel(pllar, &pmc->pllar);

		/* wait till pll ready */
		while (!(readl(&pmc->sr) & AT91_PMC_IXR_LOCKA)) {
			udelay(50);
			if (!--timeout) break;
		}
	}
}


#if 0
int ccm2200_change_master_clock_freq(int css, int div, int pres )
{
	//AT91PS_PMC pmc = (AT91PS_PMC) AT91C_BASE_PMC;
	if( ( pres < 0 || pres > 7 ) 
            || ( css < 0 || pres > 3 ) 
            || ( div < 0 || div > 3 ) )
		return 1;
	
	// Write MCKR in two accesses. See AT91RM9200 errata sheet
	//pmc->PMC_MCKR = (div << 8);
	//pmc->PMC_MCKR = css | (pres << 2) | (div << 8);

/* 	while (!(pmc->PMC_SR & AT91C_PMC_MCKRDY)) */
/*                 ; */

	at91_pmc_master_clock_select(AT91C_PMC_CSS_MAIN_CLK);

	at91_pmc_master_clock_divider(div << 8);
	at91_pmc_master_clock_prescaler(AT91C_PMC_PRES_CLK);

	udelay(1000);
	at91_pmc_master_clock_select(AT91C_PMC_CSS_PLLA_CLK);

	measure_and_set_clocks(0);
	return 0;
}
#endif


int ccm2200_change_master_clock(unsigned long div_pll,
				unsigned long mul_pll, 
				unsigned long pre_master, 
				unsigned long div_master)
{
	u32 prescaler, divider;
	if (div_pll < 1 || div_pll > 255 || mul_pll < 2 || mul_pll > 2047)
		return -1;

	switch (pre_master) {
	case 1:
		prescaler = AT91_PMC_MCKR_PRES_1;
		break;

	case 2:
		prescaler = AT91_PMC_MCKR_PRES_2;
		break;

	case 4:
		prescaler = AT91_PMC_MCKR_PRES_4;
		break;

	case 8:
		prescaler = AT91_PMC_MCKR_PRES_8;
		break;

	case 16:
		prescaler = AT91_PMC_MCKR_PRES_16;
		break;

	case 32:
		prescaler = AT91_PMC_MCKR_PRES_32;
		break;

	case 64:
		prescaler = AT91_PMC_MCKR_PRES_64;
		break;
	default:
		return -1;
	}

	switch (div_master) {

	case 1:
		divider = AT91_PMC_MCKR_MDIV_1;
		break;
	case 2:
		divider = AT91_PMC_MCKR_MDIV_2;
		break;
	case 3:
		divider = AT91_PMC_MCKR_MDIV_3;
		break;
	case 4:
		divider = AT91_PMC_MCKR_MDIV_4;
		break;
	default:
		return -1;
	}

	printf("\r");		
//	at91_pmc_master_clock_select(AT91C_PMC_CSS_MAIN_CLK);
//	at91_pmc_master_clock_select(AT91C_PMC_CSS_SLOW_CLK);
	at91_pmc_master_clock_prescaler(prescaler);
	at91_pmc_master_clock_divider(divider);
	at91_ckgr_plla(div_pll, mul_pll);
	udelay(1000);

//	at91_pmc_master_clock_select(AT91C_PMC_CSS_PLLA_CLK);
		
	measure_and_set_clocks(0);
	return 0;
}


/*****************************************************************************
 * initialization ethernet phy
 ****************************************************************************/

static void ccm2200_enable_ethernet_phy( void )
{
        register at91_port_t *p_pio = &pio->piob;

	/* Set MII_PWRDWN port pin to output with low level */
	/* UNC90:   MII_PWRDWN = PA4 */
	/* CCM2200: MII_PWRDWN = PB22 */
	writel(1<<22, &p_pio->codr);	/* clear output data */
	writel(1<<22, &p_pio->oer);	/* Enable Output */ 
	writel(1<<22, &p_pio->per);	/* Select PIO function */
}	

#ifndef CONFIG_DRIVER_AT91EMAC
#if defined(CONFIG_DRIVER_ETHER) && defined(CONFIG_CMD_NET)

#include <at91rm9200_net.h>
unsigned int lxt972_IsPhyConnected (AT91PS_EMAC p_mac);
unsigned char lxt972_GetLinkSpeed (AT91PS_EMAC p_mac);
unsigned char lxt972_InitPhy (AT91PS_EMAC p_mac);
unsigned char lxt972_AutoNegotiate (AT91PS_EMAC p_mac, int *status);

/*
 * Name:
 *	at91rm9200_GetPhyInterface
 * Description:
 *	Initialise the interface functions to the PHY
 * Arguments:
 *	None
 * Return value:
 *	None
 */
void at91rm9200_GetPhyInterface(AT91PS_PhyOps p_phyops)
{
	p_phyops->Init = lxt972_InitPhy;
	p_phyops->IsPhyConnected = lxt972_IsPhyConnected;
	p_phyops->GetLinkSpeed = lxt972_GetLinkSpeed;
	p_phyops->AutoNegotiate = lxt972_AutoNegotiate;
}

#endif  /* defined(CONFIG_DRIVER_ETHER) && defined(CONFIG_CMD_NET) */
#endif	/* ndef CONFIG_DRIVER_AT91EMAC */

#ifdef CONFIG_DRIVER_AT91EMAC
#include <netdev.h>
int board_eth_init(bd_t *bis)
{
	/* DECLARE_GLOBAL_DATA_PTR; */
	int rc = 0;
	rc = at91emac_register(bis, 0);
	/* 
         * if (rc)
	 * 	eth_init(gd->bd);
         */
	return rc;
}
#endif


/* ------------------------------------------------------------------------- */
/*
 * Miscelaneous platform dependent initialisations
 */


/**
 * misc_init_r: - misc initialisation routines
 *
 * @return: 0 in case of success
 */
int misc_init_r(void)
{
	return 0;
}


/* workaround (called by board_late_init) */
int auto_update(void) 
{
	if ((ccm2200_get_sconf_input() & 0x01 ) == 0) {
#ifndef CONFIG_SYS_HUSH_PARSER
                if (run_command (getenv ("bootcmd_update"), 0) < 0) {
                        return -1;
                }
#else
                if (parse_string_outer(getenv("bootcmd_update"),
                                       FLAG_PARSE_SEMICOLON 
                                       | FLAG_EXIT_FROM_LOOP) != 0 ) {
                        return -1;
                }
#endif
        }
        return 0;
}


static int init_ccm2200_sram(void)
{
        /*
         * Setup static memory controller chip select for SRAM
         *
         */
        static const struct at91_smc_cs_info sram_cs_config = {
                .chip_select          = CCM2200_SRAM_CS,
                .wait_states          = 15, /* SRAM needs at least 3 wait states! */
                .data_float_time      = 2,
                .byte_access_type     = AT91_BAT_16_BIT,
                .data_bus_width       = AT91_DATA_BUS_WIDTH_16,
                .data_read_protocol   = AT91_DRP_EARLY, //AT91_DRP_STANDARD,
                .address_to_cs_setup  = AT91_ACSS_STANDARD,
                .rw_setup             = 2, /* SRAM needs 1 cycle rw_setup! */
                .rw_hold              = 2  /* SRAM needs 1 cycle rw_hold! */
        };
        if ( at91_config_smc_cs( &sram_cs_config ) != 0 ) {
	    printf( "Unable to configure SRAM chip select signal\n" );
            return -1;
	}
	return 0;
}

/**
 * board_init: - setup some data structures
 *
 * @return: 0 in case of success
 */
int board_init (void)
{
	DECLARE_GLOBAL_DATA_PTR;

        __led_init(0x8008, 0x8008);

#if defined(CONFIG_HW_WATCHDOG)
        at91_pio_config_output_pins(&watchdog_trigger);
#endif

        ccm2200_enable_ethernet_phy();

	/* Enable Ctrlc */
	console_init_f ();

	init_ccm2200_sram();
	{
		int ccm2200_init_nand_chipselect(void);
		ccm2200_init_nand_chipselect();
	}

	/* memory and cpu-speed are setup before relocation */
	/* so we do _nothing_ here */

        /* 2006-04-26 gc: we have no registered a own arch number 1014 for
         *                the CCM2200 :-)
         *
         *                IMPORTANT: This arch number will be given to the
         *                linux kernel using the arm register R1
         *
         *                The linux kernel uses this arch number to select
         *                the proper board initailisation module.
         *                see linux/arch/arm/mach-at91rm9200/board-ccm2200.c
         *                For booting an UNC90 kernel it must be set to 701!
         *
         * 2006-04-28 gc: architecture type can now be overwritten using
         *                u-boot variable "machine_arch_type"
         *
         */

        /* taken from linux/include/asm-arm/mach-types.h */
#define MACH_TYPE_AT91RM9200           251
#define MACH_TYPE_UNC90                701
#define MACH_TYPE_CCM2200              1014

	//gd->bd->bi_arch_number = MACH_TYPE_AT91RM9200;
	// gd->bd->bi_arch_number = MACH_TYPE_UNC90;
        gd->bd->bi_arch_number = MACH_TYPE_CCM2200;
	/* adress of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;
	
	return 0;
}

void flash_preinit(void)
{
	/* Nothing to do... */
}


static u8 compute_sdram_nr_banks( void )
{
	vu_long *bank_addr1, *bank_addr2;
	ulong bank1_old, pattern_bank1, pattern_bank2, read_bank1, read_bank2;
		
        pattern_bank2 = 0xaaaa5555;
        pattern_bank1 = 0xa5a5a5a5;
        
	bank_addr1 = (ulong *)PHYS_SDRAM;
	bank_addr2 = (ulong *)(PHYS_SDRAM + PHYS_SDRAM_SIZE);
	
	bank1_old =  *bank_addr1;	/* save old value */
	*bank_addr1 = pattern_bank1;	/* write pattern to 1st bank */
	*bank_addr2 = pattern_bank2;	/* write pattern to 2nd bank */

	read_bank1 = *bank_addr1;	/* read from 1st bank */
	read_bank2 = *bank_addr2;	/* read from 2nd bank */
	
	*bank_addr1 = bank1_old;	/* write back old value */

	if( read_bank2 == pattern_bank2 ) {
		if( pattern_bank1 != read_bank1 )
			return 1;	/* only one bank */
		else	
			return 2;	/* both banks */
	}
	 
	return 1;	/* only one bank */

}


/**
 * dram_init: - setup dynamic RAM
 *
 * @return: 0 in case of success
 */
int dram_init (void)
{
	DECLARE_GLOBAL_DATA_PTR;

	gd->bd->bi_dram[0].start = PHYS_SDRAM;
	gd->bd->bi_dram[0].size = PHYS_SDRAM_SIZE;
	
	// Depending on the number of banks... adjust the SDRAM size
	gd->bd->bi_dram[0].size *= compute_sdram_nr_banks();
	
	return 0;
}


#if defined(BOARD_LATE_INIT)
#include <net.h>


/* 2006-04-28 gc: convert string to lower case (ASCII only) */
static void strtolower(char *dest, unsigned dest_length, const char *src)
{
        char *dest_end = dest + dest_length - 1;
        *dest_end = '\0';
        for (;*src && dest != dest_end; ++src, ++dest) {
                if (*src >= 'A' && *src <= 'Z')
                        *dest = *src | 0x20;
                else 
                        *dest = *src;
        }
        *dest = '\0';
}


int board_late_init( )
{
	at91_pmc_t *pmc = (at91_pmc_t *) AT91_PMC_BASE;

	//char flashsize[32];
	//char flashpartsize[32];

        /* define pointer gd */
	DECLARE_GLOBAL_DATA_PTR;

	//printf("board_late_init()\n");
	measure_and_set_clocks(1);

	
	/* added for use in u-boot macros */ 
/* 	sprintf(flashsize, "%x", PHYS_FLASH_SIZE); */
/* 	sprintf(flashpartsize, "%x", PHYS_FLASH_SIZE - 0x340000); */
/* 	setenv("flashsize", flashsize); */
/* 	setenv("flashpartsize", flashpartsize);  */

	/* set rootfs image filename according installed NAND flash page size */
	switch (nand_info[0].writesize /* oobblock */) {
	case 512:
                setenv("nand_rimg", "rootfs-ccm2200-sp-nand.jffs2");
		break;
	case 2048:
                setenv("nand_rimg", "rootfs-ccm2200-lp-nand.jffs2");
		break;
	}

        /* 2006-04-28 gc: try to read machine-type from environment */
        {
                const char *env_str = getenv("machine_arch_type");
                
                if (env_str) {
                        char machine_arch_type_str[32];
                        unsigned long machine_arch_type_value;
                        strtolower(machine_arch_type_str,
                                   sizeof(machine_arch_type_str)/
                                   sizeof(machine_arch_type_str[0]),
                                   env_str);

                        if (!strcmp(machine_arch_type_str, 
                                        "unc90")) {
                                machine_arch_type_value = MACH_TYPE_UNC90;
                        } else if (!strcmp(machine_arch_type_str, 
                                        "ccm2200")) {
                                machine_arch_type_value = MACH_TYPE_CCM2200;
                        } else {
                                machine_arch_type_value 
                                        = simple_strtoul(machine_arch_type_str,
                                                         NULL, 0);
                        }
                        gd->bd->bi_arch_number = machine_arch_type_value;
                }

                printf( "machine_arch_type for linux kernel set to: "
                        "%lu %s\n",  (unsigned long)gd->bd->bi_arch_number,
                        gd->bd->bi_arch_number == MACH_TYPE_UNC90 ?
                        "(MACH_TYPE_UNC90)" : 
                        gd->bd->bi_arch_number == MACH_TYPE_CCM2200 ?
                        "(MACH_TYPE_CCM2200)" : "");
                
        }

#ifdef CONFIG_DRIVER_ETHER
	printf("initializing eth\n");
        ccm2200_enable_ethernet_phy();

	// Initial delay due to the phy PWDOWN line has been set to 0...
	udelay( 400 );

	if( eth_init( gd->bd ) < 0 )
		printf( "Error initializing ethernet inteface\n" );
#endif

	/* configure here the port pins of the AT91RM9200 */
	/* some things may be removed here later */

	/* enable all PIO clocks. PIO inputs need clock running */
	writel((1 << AT91_ID_PIOA)
	       | (1 << AT91_ID_PIOB)
	       | (1 << AT91_ID_PIOC)
	       | (1 << AT91_ID_PIOD), &pmc->pcer);

#if 0
        /* 2006-03-31 gc:  */
        if (ccm2200_get_sconf_input() & 0x01) {
          int i;
          static unsigned short array[] = {
            //0x8000,
            //0x4000 
            0x1248,
            0x03a0,
            0x0a30,
            0x8421,
            0x4422,
            0x2244,
            /* 
             * 0x0000,
             * 0xffff,
             * 0x0000            
             */
          };            
          unsigned status = 0;
          unsigned short *ptr = array,
            *end = array + sizeof(array)/sizeof(array[0]);

          /* 
           * while (!tstc()) {
           *   for (i = 0; i < 14; ++i) {
           *     set_led((1<<i) | *ptr );
           *     udelay_masked(50000);
           *   }
           *   if (++ptr == end) { ptr = array; }
           * }
           */
          i = 0;
          while (!tstc()) {
            udelay_masked(14000);
            
            if ((i % 5) == 0) {
              if (++ptr == end) { ptr = array; }
            } 

            if ((i % 43) == 0) {
              status ^= 0x8000;
            }
            ++i;
            set_led(*ptr | status);

          }
        }

        set_led(0x9669); /* test pattern */
#endif

	/* configure opto isolated ouputs */
	/* attention: avoid contention, open drain needed!! */
/*	*AT91C_PIOD_SODR = 0x0000FFFF; */ /* set output high */
	
        auto_update();
	return 0;

}
#endif


void set_default_env(void);

int load_default_environment(void)
{
	char ethaddr[64];
	char serialnum[256];

	int have_ethaddr;
	int have_serialnum;
	have_ethaddr = getenv_f("ethaddr", ethaddr, sizeof(ethaddr));
	have_serialnum = getenv_f("serial#", serialnum, sizeof(serialnum));

	printf("\n*** RESET ENVIRONMENT ***\n");

	set_default_env();

	if (have_ethaddr > 0) {
		setenv("ethaddr", ethaddr);
	}
	if (have_serialnum > 0) {
		setenv("serial#", serialnum);
	}
	return 0;
}


int do_ccm2200(cmd_tbl_t * cmdtp, int flag, int argc, char *const argv[])
{
	const char *cmd;

	if (argc < 2) {
			printf("Usage:\n%s\n", cmdtp->usage);
			
			return 1;
	}

	cmd = argv[1];

	if (!strcmp(cmd, "clock")) {
		if (argc != 6) {
			printf("Usage:\n ccm2200 clock "
                               "DIV_PLL MUL_PLL PRE_MAS DIV_MAS\n");
			return 1;
		} else {
			int result = 0;
			unsigned long div_pll 
				= simple_strtoul(argv[2], NULL, 0);
			unsigned long mul_pll 
				= simple_strtoul(argv[3], NULL, 0);
			unsigned long pre_master 
				= simple_strtoul(argv[4], NULL, 0);
			unsigned long div_master 
				= simple_strtoul(argv[5], NULL, 0);
			printf("switching clock to: div_pll:%lu "
			       "mul_pll:%lu pre_mas:%lu div_mas:%lu\n",
			       div_pll, mul_pll, 
			       pre_master, div_master);
			if (ccm2200_change_master_clock(div_pll, mul_pll,
							pre_master, div_master)
			    != 0) {
				printf("Switching clock failed! "
				       "Clock parameters may be wrong!\n");
				result = 1;
			} else {
				measure_and_set_clocks(1);
			}
			return result;
		}
	} else if (!strcmp(cmd, "defaultenv")) {
		load_default_environment();
		return 0;
	} else if (!strcmp(cmd, "led")) {
		if (argc != 4) {
			printf("Usage:\n ccm2200 led "
                               "<mask> <value>\n");
			return 1;
		} else {
			unsigned mask  = simple_strtoul(argv[2], NULL, 0);
			unsigned value = simple_strtoul(argv[3], NULL, 0);
			__led_set(mask, value);
			return 0;
		}
	}
	return 1;
}

U_BOOT_CMD(ccm2200, 6, 1, do_ccm2200,
	"ccm2200   - CCM2200 board specific commands\n",
	"clock DIV_PLL MUL_PLL PRE_MAS DIV_MAS\n"
	"defaultenv\n"
	"led <mask> <value>\n"
	);
