/*
 * board/ccm2200/ccm2200.c
 *
 * Copyright (C) 2006 by Weiss-Electronic GmbH.
 * Copyright (C) 2010 by SWARCO Traffic Systems GmbH.
 * All rights reserved.
 *
 * @author:     Guido Classen <guido.classen@swarco.de>
 * @descr:      NAND Flash support
 * @references: 
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
 *   2010-11-16 gc: migration to generic atmel_nand.c driver
 *   2006-09-21 gc: support for non legacy NAND subsystem
 *   2006-04-25 gc: initial version
 */

#include <common.h>

#include <asm/io.h>             /* readl() */

#include <asm/arch/at91_util.h>
#include <asm/arch-at91/at91rm9200.h>   /* AT91_PIO_BASE */
#include <asm/arch/at91_pio.h>     /* at91_pio_t */
#include <asm/arch/at91_pmc.h>     /* at91_pmc_t */

#if defined(CONFIG_CMD_NAND) 

/*****************************************************************************
 * common routines for legacy and new NAND support
 ****************************************************************************/

/*
 * Setup static memory controller chip select for NAND Flash
 *
 */
int ccm2200_init_nand_chipselect(void)
{
        static const struct at91_smc_cs_info nand_cs_config = {
                .chip_select          = CCM2200_NAND_FLASH_CS,
                .wait_states          = 5,
                .data_float_time      = 2,
                .byte_access_type     = AT91_BAT_8_BIT,
                .data_bus_width       = AT91_DATA_BUS_WIDTH_8,
                .data_read_protocol   = AT91_DRP_STANDARD,
                .address_to_cs_setup  = AT91_ACSS_STANDARD,
                .rw_setup             = 1,
                .rw_hold              = 1
        };

        if ( at91_config_smc_cs( &nand_cs_config ) != 0 )
	{
	    printf( "Unable to configure NAND flash chip select signal\n" );
            return -1;
	}

        /* enable PIOA's clock so we can read the NAND ready/_busy PIN
         * using GPIO PA19 
         */
        {
                at91_pmc_t *pmc = (at91_pmc_t *) AT91_PMC_BASE; \
                writel(1 << AT91_ID_PIOA, &pmc->pcer);         \
        }
        return 0;
}

#endif // defined(CONFIG_CMD_NAND) 

/*
 *Local Variables:
 * mode: c
 * c-file-style: "linux"
 * End:
 */



