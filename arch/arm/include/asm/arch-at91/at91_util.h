/*
 * include/asm-arm/arch-at91rm9200/at91_util.h
 *
 * Helper functions for configuration of AT91RM9200 peripherals
 *
 * Guido Classen <guido.classen@swarco.de>
 * Copyright (C) 2006 by Weiss-Electronic GmbH.
 * Copyright (C) 2010 by SWARCO Traffic Systems GmbH.
 * All rights reserved.
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
 *
 *  @par Modification History:
 *     2006-05-05 gc: initial version
 */

#ifndef __ASM_ARCH_AT91_UTIL_H
#define __ASM_ARCH_AT91_UTIL_H

#include <asm/hardware.h>
#include <asm/arch/at91_pio.h>

/****************************************************************************
 * define some constants for easer usage of at91_set_gpio_direction() 
 ****************************************************************************/

#define GPIO_OUTPUT     0
#define GPIO_INPUT      1
#define GPIO_NO_PULLUP  0
#define GPIO_USE_PULLUP 1
#define GPIO_INIT_0     0
#define GPIO_INIT_1     1


/****************************************************************************
 * generic AT91RM9200 specific functions
 ****************************************************************************/
struct at91_smc_cs_info
{
        int chip_select;              /* number of chip select pin */
        int wait_states;              /* wait states 0...128 (0 = no wait
                                       * states) */
        int data_float_time;          /* 0...15 wait states after memory read
                                           cycle */
        enum byte_access_type {       /* only valid for
                                       * data_bus_width=16bit */
                AT91_BAT_8_BIT  = 0,  /* dummy value for 8 bit device
                                       * (ignored) */
                AT91_BAT_TWO_8_BIT = 0,     /* chip select is connected to
                                             * two/four 8-bit devices */
                AT91_BAT_16_BIT = 1   /* chip select is connected toone 16 bit
                                       * device */
        } byte_access_type;

        enum data_bus_width {
                AT91_DATA_BUS_WIDTH_8 = 2,  /* 8 bit data bus */
                AT91_DATA_BUS_WIDTH_16 = 1, /* 16 bit data bus */
        } data_bus_width;
        enum data_read_protocol {
                AT91_DRP_STANDARD = 0,      /* standard data read protocol */
                AT91_DRP_EARLY    = 1,      /* early data read protocol */
        } data_read_protocol;

        enum address_to_cs_setup {
                AT91_ACSS_STANDARD   = 0,   /* standard: address asserted at
                                             * the beginning of the access and
                                             * deasserted at the end */
                AT91_ACSS_1_CYCLE    = 1,   /* one cycle less at the beginning
                                             * and end */
                AT91_ACSS_2_CYCLES   = 2,   /* two cycles less at the
                                             * beginning and end */
                AT91_ACSS_3_CYCLES   = 3    /* three cycles less at the
                                             * beginning and end */
        } address_to_cs_setup;
        int rw_setup;                       /* 0...7 number of read/write
                                                 setup cycles */
        int rw_hold;                        /* 0...7 number of read/write hold
                                                 cycles */
};


/* 
 * Configure the static memory controller chip select register using
 * values in struct at91_smc_cs_info
 */
int at91_config_smc_cs( const struct at91_smc_cs_info *info );

/* 
 * #undef AT91_PIOA
 * #undef AT91_PIOB
 * #undef AT91_PIOC
 * #undef AT91_PIOD
 * 
 * #define AT91_PIOA               ((AT91S_PIO*) AT91C_BASE_PIOA)
 * #define AT91_PIOB               ((AT91S_PIO*) AT91C_BASE_PIOB)
 * #define AT91_PIOC               ((AT91S_PIO*) AT91C_BASE_PIOC)
 * #define AT91_PIOD               ((AT91S_PIO*) AT91C_BASE_PIOD)
 */


/* describe 1...32 pins of a PIO port */
struct at91_pio_pins {
        //AT91S_PIO* pio;
        at91_port_t *pio;
        unsigned signal_bit;
};

static inline
void at91_pio_clear_all_pins(const struct at91_pio_pins *pins)
{
        //pins->pio->PIO_CODR = pins->signal_bit; /* set pin to 0 */
        pins->pio->codr = pins->signal_bit; /* set pin to 0 */
}

static inline
void at91_pio_set_all_pins(const struct at91_pio_pins *pins)
{
        //pins->pio->PIO_SODR = pins->signal_bit; /* set pin to 1 */
        pins->pio->sodr = pins->signal_bit; /* set pin to 1 */
}

static inline
void at91_pio_assign_pins(const struct at91_pio_pins *pins, unsigned value)
{
        //pins->pio->PIO_SODR = value & pins->signal_bit; /* set pin to 1 */
        pins->pio->sodr = value & pins->signal_bit; /* set pin to 1 */
}

static inline
int at91_pio_get_pins(const struct at91_pio_pins *pins)
{
        //return pins->pio->PIO_PDSR & pins->signal_bit;
        return pins->pio->pdsr & pins->signal_bit;
}

static inline
void at91_pio_config_output_pins(const struct at91_pio_pins *pins)
{
        //register AT91S_PIO *pio = pins->pio;
        register at91_port_t *pio = pins->pio;
        register unsigned signal_bit = pins->signal_bit;
        
        /* 
         * pio->PIO_IDR = signal_bit; /\* disable interrupt *\/
         * pio->PIO_PPUDR = signal_bit; /\* disable pull up *\/
         * pio->PIO_SODR = signal_bit; /\* set pin to 1  *\/
         * pio->PIO_OER  = signal_bit; /\* enable output *\/
         * pio->PIO_PER = signal_bit; /\* assign gpio to pin! *\/
         */
        pio->idr = signal_bit; /* disable interrupt */
        pio->pudr = signal_bit; /* disable pull up */
        pio->sodr = signal_bit; /* set pin to 1  */
        pio->oer  = signal_bit; /* enable output */
        pio->per = signal_bit; /* assign gpio to pin! */
}

static inline
void at91_pio_config_input_pins(const struct at91_pio_pins *pins)
{
        //register AT91S_PIO *pio = pins->pio;
        register at91_port_t *pio = pins->pio;
        register unsigned signal_bit = pins->signal_bit;
        
        /* 
         * pio->PIO_IDR = signal_bit; /\* disable interrupt *\/
         * pio->PIO_PPUDR = signal_bit; /\* disable pull up *\/
         * pio->PIO_ODR  = signal_bit; /\* disable output *\/
         * pio->PIO_PER = signal_bit; /\* assign gpio to pin! *\/
         */
        pio->idr = signal_bit; /* disable interrupt */
        pio->pudr = signal_bit; /* disable pull up */
        pio->odr  = signal_bit; /* disable output */
        pio->per = signal_bit; /* assign gpio to pin! */
}

/* enable open drain output (multi drive) */
static inline
void at91_pio_enable_open_drain_pins(const struct at91_pio_pins *pins)
{
        //pins->pio->PIO_MDER = pins->signal_bit;
        pins->pio->mder = pins->signal_bit;
}

/* disable open drain output (multi drive) */
static inline
void at91_pio_disable_open_drain_pins(const struct at91_pio_pins *pins)
{
        //pins->pio->PIO_MDDR = pins->signal_bit;
        pins->pio->mddr = pins->signal_bit;
}

#endif /* __ASM_ARCH_BOARD_CCM2200_H */

/*
 *Local Variables:
 * mode: c
 * c-file-style: "linux"
 * End:
 */
