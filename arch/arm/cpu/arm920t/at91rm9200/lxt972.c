/*
 *
 * (C) Copyright 2003
 * Author : Hamid Ikdoumi (Atmel)
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
 */

/*
 * Adapted for KwikByte KB920x board: 22APR2005
 */

#include <common.h>
#include <at91rm9200_net.h>
#include <net.h>
#include <miiphy.h>
#include <lxt971a.h>

#ifdef CONFIG_DRIVER_ETHER

#if defined(CONFIG_CMD_NET)
static unsigned short phy_address = 0;
/*
 * Name:
 *	lxt972_IsPhyConnected
 * Description:
 *	Reads the 2 PHY ID registers
 * Arguments:
 *	p_mac - pointer to AT91S_EMAC struct
 * Return value:
 *	TRUE - if id read successfully
 *	FALSE- if error
 */
unsigned int lxt972_IsPhyConnected (AT91PS_EMAC p_mac)
{
	unsigned short Id1, Id2;

	for (phy_address = 0; phy_address < 32; phy_address++) {
	at91rm9200_EmacEnableMDIO (p_mac);
		at91rm9200_EmacReadPhy(p_mac, (phy_address << 5) | PHY_PHYIDR1,
				       &Id1);
		at91rm9200_EmacReadPhy(p_mac, (phy_address << 5) | PHY_PHYIDR2,
				       &Id2);
	at91rm9200_EmacDisableMDIO (p_mac);

		printf("lxt972_IsPhyConnected: id1=%d, id2=%d\n", Id1, Id2);
		if ((Id1 == PHY_LXT971A/* (0x0013) */) 
		    && ((Id2  & 0xFFF0) == 0x78E0)) {
			printf("found Cortina LXT971 / LXT972 phy at addr=%d\n",
			       phy_address);
		return TRUE;
		}
	}

	return FALSE;
}

/*
 * Name:
 *	lxt972_GetLinkSpeed
 * Description:
 *	Link parallel detection status of MAC is checked and set in the
 *	MAC configuration registers
 * Arguments:
 *	p_mac - pointer to MAC
 * Return value:
 *	TRUE - if link status set succesfully
 *	FALSE - if link status not set
 */
UCHAR lxt972_GetLinkSpeed (AT91PS_EMAC p_mac)
{
	unsigned short stat1;

	if (!at91rm9200_EmacReadPhy (p_mac, (phy_address << 5) | PHY_LXT971_STAT2, &stat1))
		return FALSE;

	if (!(stat1 & PHY_LXT971_STAT2_LINK))	/* link status up? */
		return FALSE;

	if (stat1 & PHY_LXT971_STAT2_100BTX) {

		if (stat1 & PHY_LXT971_STAT2_DUPLEX_MODE) {

			/*set Emac for 100BaseTX and Full Duplex  */
			p_mac->EMAC_CFG |= AT91C_EMAC_SPD | AT91C_EMAC_FD;
		} else {

			/*set Emac for 100BaseTX and Half Duplex  */
			p_mac->EMAC_CFG = (p_mac->EMAC_CFG &
					~(AT91C_EMAC_SPD | AT91C_EMAC_FD))
					| AT91C_EMAC_SPD;
		}

		return TRUE;

	} else {

		if (stat1 & PHY_LXT971_STAT2_DUPLEX_MODE) {

			/*set MII for 10BaseT and Full Duplex  */
			p_mac->EMAC_CFG = (p_mac->EMAC_CFG &
					~(AT91C_EMAC_SPD | AT91C_EMAC_FD))
					| AT91C_EMAC_FD;
		} else {

			/*set MII for 10BaseT and Half Duplex  */
			p_mac->EMAC_CFG &= ~(AT91C_EMAC_SPD | AT91C_EMAC_FD);
		}

		return TRUE;
	}

	return FALSE;
}


UCHAR lxt972_AutoNegotiate (AT91PS_EMAC p_mac, int *status);
/*
 * Name:
 *	lxt972_InitPhy
 * Description:
 *	MAC starts checking its link by using parallel detection and
 *	Autonegotiation and the same is set in the MAC configuration registers
 * Arguments:
 *	p_mac - pointer to struct AT91S_EMAC
 * Return value:
 *	TRUE - if link status set succesfully
 *	FALSE - if link status not set
 */
UCHAR lxt972_InitPhy (AT91PS_EMAC p_mac)
{
	UCHAR ret = TRUE;
	int dummy;
	printf("lxt972_InitPhy()\n");

	at91rm9200_EmacEnableMDIO (p_mac);

	if (!lxt972_GetLinkSpeed (p_mac)) {
		/* Try another time */
		ret = lxt972_GetLinkSpeed (p_mac);
	}

	/* Disable PHY Interrupts */
	at91rm9200_EmacWritePhy (p_mac, (phy_address << 5) | PHY_LXT971_INT_ENABLE, 0);

	at91rm9200_EmacDisableMDIO (p_mac);

	/* lxt972_AutoNegotiate() (or PhyOps.AutoNegotiate() seems
	 * never be called at all, so we call it once here */
	lxt972_AutoNegotiate(p_mac, &dummy);
	return (ret);
}


/*
 * Name:
 *	lxt972_AutoNegotiate
 * Description:
 *	MAC Autonegotiates with the partner status of same is set in the
 *	MAC configuration registers
 * Arguments:
 *	dev - pointer to struct net_device
 * Return value:
 *	TRUE - if link status set successfully
 *	FALSE - if link status not set
 */
UCHAR lxt972_AutoNegotiate (AT91PS_EMAC p_mac, int *status)
{
	unsigned short value;
	int i;

	printf("lxt972_AutoNegotiate\n");
	at91rm9200_EmacEnableMDIO (p_mac);

	/* Write Autonegotion advertisement register */
	value = PHY_ANLPAR_TX | PHY_ANLPAR_TXFD 
		| PHY_ANLPAR_10 | PHY_ANLPAR_10FD
		| PHY_ANLPAR_PSB_802_3;

	if (!at91rm9200_EmacWritePhy (p_mac, (phy_address << 5) | PHY_ANAR, &value))
		return FALSE;

	/* Set lxt972 control register */
	if (!at91rm9200_EmacReadPhy (p_mac, (phy_address << 5) | PHY_BMCR, &value))
		return FALSE;

	/* Restart Auto_negotiation  */
	value |= PHY_BMCR_AUTON | PHY_BMCR_RST_NEG;
	if (!at91rm9200_EmacWritePhy (p_mac, (phy_address << 5) | PHY_BMCR, &value))
		return FALSE;

	for (i = 0; i < 50; i++) {

		/*check AutoNegotiate complete */
		//udelay (10000);
		udelay (100000);
		at91rm9200_EmacReadPhy(p_mac, (phy_address << 5) | PHY_BMSR, &value);
		printf("PHY-SR: %04x, %d\n", value, (value & PHY_BMSR_AUTN_COMP) != 0);
		if (value & PHY_BMSR_AUTN_COMP) {
			at91rm9200_EmacDisableMDIO (p_mac);
	return (lxt972_GetLinkSpeed (p_mac));
		}
	}

	at91rm9200_EmacDisableMDIO (p_mac);
	printf("autonegation failed\n");
	return FALSE;
}

#endif

#endif	/* CONFIG_DRIVER_ETHER */
