/*
 * cpu/arm920t/at91rm9200/at91_util.h
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
 *     2006-05-23 gc: initial version
 */

#include <common.h>

#include <asm/io.h>
#include <asm/arch/hardware.h>
#include <asm/arch/at91rm9200.h>
#include <asm/arch/at91_mc.h>
#include <asm/arch/at91_util.h>

#define AT91C_PIO_PC10		(1 << 10)
#define AT91C_PC10_NCS4_CFCS	(AT91C_PIO_PC10) //  Compact Flash Chip Select
#define AT91C_PIO_PC11		(1 << 11)
#define AT91C_PC11_NCS5_CFCE1	(AT91C_PIO_PC11) //  Chip Select 5 / Compact Flash Chip Enable 1
#define AT91C_PIO_PC12		(1 << 12)
#define AT91C_PC12_NCS6_CFCE2	(AT91C_PIO_PC12) //  Chip Select 6 / Compact Flash Chip Enable 2
#define AT91C_PIO_PC13		(1 << 13)
#define AT91C_PC13_NCS7		(AT91C_PIO_PC13) //  Chip Select 7

#define pio ((at91_pio_t *) AT91_PIO_BASE)
/* 
 * Configure the static memory controller chip select register using
 * values in struct at91_smc_cs_info
 */
int at91_config_smc_cs( register const struct at91_smc_cs_info *info )
{
	unsigned int mask = 0;
	at91_mc_t *mc = (at91_mc_t *)AT91_MC_BASE;

        /* check parameters */
        if ( info->chip_select < 0 || info->chip_select > 7 
             || info->wait_states < 0 || info->wait_states > 128  
             || info->data_float_time < 0 || info->data_float_time > 15
             || (info->byte_access_type != AT91_BAT_TWO_8_BIT
                 && info->byte_access_type != AT91_BAT_16_BIT)
             || (info->data_read_protocol != AT91_DRP_STANDARD
                 && info->data_read_protocol != AT91_DRP_EARLY)
             || info->address_to_cs_setup < AT91_ACSS_STANDARD
             || info->address_to_cs_setup > AT91_ACSS_3_CYCLES
             || info->rw_setup < 0 || info->rw_setup > 7
             || info->rw_hold  < 0 || info->rw_hold > 7)
        {
                return -1;
        }

        // configure gpios, if necessary
        if( info->chip_select > 3 )
        {
                switch( info->chip_select )
                {
                case 4:	mask = AT91C_PC10_NCS4_CFCS;	break;
                case 5:	mask = AT91C_PC11_NCS5_CFCE1;	break;
                case 6:	mask = AT91C_PC12_NCS6_CFCE2;	break;
                case 7:	mask = AT91C_PC13_NCS7;		break;
                default: mask = 0;
                }

                /* select peripheral a function */
		writel(mask, &pio->pioc.asr);
                /* disable pio controller and enable peripheral */
		writel(mask, &pio->pioc.pdr);
        }

        /* write the new configuration to SMC chip select register */
	writel(mask, &pio->pioc.pdr);

	writel((info->wait_states > 0 
		? AT91_SMC_CSR_NWS((info->wait_states-1)) 
		| AT91_SMC_CSR_WSEN 
		: 0)
	       | AT91_SMC_CSR_TDF(info->data_float_time) 
	       | (info->byte_access_type == AT91_BAT_16_BIT 
		  ? AT91_SMC_CSR_BAT_16 : 0) 
	       | ((int)info->data_bus_width << 13) 
	       | (info->data_read_protocol == AT91_DRP_EARLY 
		  ? AT91_SMC_CSR_DRP : 0) 
	       | ((int)info->address_to_cs_setup << 16)
	       | AT91_SMC_CSR_RWSETUP(info->rw_setup)
	       | AT91_SMC_CSR_RWHOLD(info->rw_hold),
	       &mc->smc.csr[info->chip_select]);
	
	return 0;
}
