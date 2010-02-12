//==========================================================================
//
//      if_dm9000.c
//
//	Davicom DM9000 ethernet driver
//
//==========================================================================
// ####ECOSGPLCOPYRIGHTBEGIN####                                            
// -------------------------------------------                              
// This file is part of eCos, the Embedded Configurable Operating System.   
// Copyright (C) 2003, 2004 Free Software Foundation, Inc.                  
//
// eCos is free software; you can redistribute it and/or modify it under    
// the terms of the GNU General Public License as published by the Free     
// Software Foundation; either version 2 or (at your option) any later      
// version.                                                                 
//
// eCos is distributed in the hope that it will be useful, but WITHOUT      
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or    
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License    
// for more details.                                                        
//
// You should have received a copy of the GNU General Public License        
// along with eCos; if not, write to the Free Software Foundation, Inc.,    
// 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.            
//
// As a special exception, if other files instantiate templates or use      
// macros or inline functions from this file, or you compile this file      
// and link it with other works to produce a work based on this file,       
// this file does not by itself cause the resulting work to be covered by   
// the GNU General Public License. However the source code for this file    
// must still be made available in accordance with section (3) of the GNU   
// General Public License v2.                                               
//
// This exception does not invalidate any other reasons why a work based    
// on this file might be covered by the GNU General Public License.         
// -------------------------------------------                              
// ####ECOSGPLCOPYRIGHTEND####                                              
//#####DESCRIPTIONBEGIN####
//
// Author(s):    msalter
// Contributors: msalter
// Date:         2004-03-18
// Purpose:      
// Description:  hardware driver for Davicom DM9000 NIC
// Notes:
//
// modify: morgan 
// date:   2010-02-12
//####DESCRIPTIONEND####
//
//==========================================================================

#include <pkgconf/system.h>
#include <pkgconf/io_eth_drivers.h>
#include <pkgconf/devs_eth_davicom_dm9000.h>

#include <cyg/infra/cyg_type.h>
#include <cyg/infra/cyg_ass.h>
#include <cyg/hal/hal_arch.h>
#include <cyg/hal/hal_cache.h>
#include <cyg/hal/hal_intr.h>
#include <cyg/hal/hal_endian.h>
#include <cyg/infra/diag.h>
#include <cyg/hal/hal_if.h>
#include <cyg/hal/drv_api.h>
#include <cyg/io/eth/netdev.h>
#include <cyg/io/eth/eth_drv.h>

#include <dm9000_info.h>

#ifdef CYGPKG_REDBOOT
#include <pkgconf/redboot.h>
#include <redboot.h>
#include <flash_config.h>
#endif

#include CYGDAT_DEVS_ETH_DAVICOM_DM9000_INL

#define DM9000_PKT_MAX 1536

//#define DEBUG
//#define DEBUG_DUMP

//
// Control and Status register offsets
//
#define DM_NCR      0x00
#define DM_NSR      0x01
#define DM_TCR      0x02
#define DM_TSRI     0x03
#define DM_TSRII    0x04
#define DM_RCR      0x05
#define DM_RSR      0x06
#define DM_ROCR     0x07
#define DM_BPTR     0x08
#define DM_FCTR     0x09
#define DM_FCR      0x0a
#define DM_EPCR     0x0b
#define DM_EPAR     0x0c
#define DM_EPDRL    0x0d
#define DM_EPDRH    0x0e
#define DM_WCR      0x0f
#define DM_PAR      0x10
#define DM_MAR      0x16
#define DM_GPCR     0x1e
#define DM_GPR      0x1f
#define DM_TRPAL    0x22
#define DM_TRPAH    0x23
#define DM_RWPAL    0x24
#define DM_RWPAH    0x25
#define DM_VIDL     0x28
#define DM_VIDH     0x29
#define DM_PIDL     0x2a
#define DM_PIDH     0x2b
#define DM_CHIPR    0x2c
#define DM_SMCR     0x2f
#define DM_MRCMDX   0xf0
#define DM_MRCMD    0xf2
#define DM_MDRAL    0xf4
#define DM_MDRAH    0xf5
#define DM_MWCMDX   0xf6
#define DM_MWCMD    0xf8
#define DM_MDWAL    0xfa
#define DM_MDWAH    0xfb
#define DM_TXPLL    0xfc
#define DM_TXPLH    0xfd
#define DM_ISR      0xfe
#define DM_IMR      0xff

// NCR (Network Control Register)
#define NCR_EXT_PHY   (1 << 7)     // 1 ==> external PHY, 0 ==> internal
#define NCR_WAKEEN    (1 << 6)     // enable wakeup events
#define NCR_FCOL      (1 << 4)     // force collision mode (test)
#define NCR_FDX       (1 << 3)     // full duplex (read-only for internal phy)
#define NCR_LBK_NOR   (0 << 1)     // loopback off
#define NCR_LBK_MAC   (1 << 1)     // MAC loopback
#define NCR_LBK_PHY   (2 << 1)     // PHY loopback
#define NCR_RST       (1 << 0)     // Reset (auto-clears after 10us)

// NSR (Network Status Register)
#define NSR_SPEED     (1 << 7)     // 0 = 100Mbps, 1 = 10Mbps
#define NSR_LINKST    (1 << 6)     // link status (1 = okay)
#define NSR_WAKEST    (1 << 5)     // wake status (clear by read)
#define NSR_TX2END    (1 << 3)     // TX packet 2 complete
#define NSR_TX1END    (1 << 2)     // TX packet 1 complete
#define NSR_RXOV      (1 << 1)     // RX overflow

// TCR (TX Control Register)
#define TCR_TJDIS     (1 << 6)     // TX jabber disable
#define TCR_EXCECM    (1 << 5)     // 0 = abort after 15 collisions
#define TCR_PAD_DIS2  (1 << 4)
#define TCR_CRC_DIS2  (1 << 3)
#define TCR_PAD_DIS1  (1 << 2)
#define TCR_CRC_DIS1  (1 << 1)
#define TCR_TXREQ     (1 << 0)

// TSR (TX Status Register)
#define TSR_TJTO      (1 << 7)
#define TSR_LC        (1 << 6)
#define TSR_NC        (1 << 5)
#define TSR_LCOL      (1 << 4)
#define TSR_COL       (1 << 3)
#define TSR_EC        (1 << 2)

// RCR (RX Control Register)
#define RCR_WTDIS     (1 << 6)
#define RCR_DIS_LONG  (1 << 5)
#define RCR_DIS_CRC   (1 << 4)
#define RCR_ALL       (1 << 3)
#define RCR_RUNT      (1 << 2)
#define RCR_PRMSC     (1 << 1)
#define RCR_RXEN      (1 << 0)

// RSR (RX Status Register)
#define RSR_RF        (1 << 7)
#define RSR_MF        (1 << 6)
#define RSR_LCS       (1 << 5)
#define RSR_RWTO      (1 << 4)
#define RSR_PLE       (1 << 3)
#define RSR_AE        (1 << 2)
#define RSR_CE        (1 << 1)
#define RSR_FOE       (1 << 0)

// FCR (Flow Control Register)
#define FCR_TXPO      (1 << 7)
#define FCR_TXPF      (1 << 6)
#define FCR_TXPEN     (1 << 5)
#define FCR_BKPA      (1 << 4)
#define FCR_BKPM      (1 << 3)
#define FCR_RXPS      (1 << 2)
#define FCR_RXPCS     (1 << 1)
#define FCR_FLCE      (1 << 0)

// EPCR (EEPROM & PHY Control Register)
#define EPCR_REEP     (1 << 5)
#define EPCR_WEP      (1 << 4)
#define EPCR_EPOS     (1 << 3)
#define EPCR_ERPRR    (1 << 2)
#define EPCR_ERPRW    (1 << 1)
#define EPCR_ERRE     (1 << 0)

// WCR (Wakeup Control Register)
#define WCR_LINKEN    (1 << 5)
#define WCR_SAMPLEEN  (1 << 4)
#define WCR_MAGICEN   (1 << 3)
#define WCR_LINKST    (1 << 2)
#define WCR_SAMPLEST  (1 << 1)
#define WCR_MAGIGST   (1 << 0)

// SMCR (Special Mode Control Register)
#define SMCR_SM_EN    (1 << 7)
#define SMCR_FLC      (1 << 2)
#define SMCR_FB1      (1 << 1)
#define SMCR_FB0      (1 << 0)

// ISR (Interrupt Status Register)
#define ISR_IOMODE_16 (0 << 6)
#define ISR_IOMODE_32 (1 << 6)
#define ISR_IOMODE_8  (2 << 6)
#define ISR_ROOS      (1 << 3)
#define ISR_ROS       (1 << 2)
#define ISR_PTS       (1 << 1)
#define ISR_PRS       (1 << 0)

// IMR (Interrupt Mask Register)
#define IMR_PAR       (1 << 7)
#define IMR_ROOM      (1 << 3)
#define IMR_ROM       (1 << 2)
#define IMR_PTM       (1 << 1)
#define IMR_PRM       (1 << 0)

/* PHY registers */
#define PHY_BMCR      0x00
#define PHY_BMSR      0x01
#define PHY_ANAR      0x04
#define PHY_SCSR      0x11         

/* PHY BMCR (Basic Mode Control Register) */
#define PHY_BMCR_AUTO_NEG_EN    (1 << 12)
#define PHY_BMCR_AUTO_NEG_START (1 << 12)

/* PHY BMSR (Basic Mode Status Register) */
#define PHY_BMSR_AUTO_NEG_COMPLETE (1 << 5)

// Read one datum from 8-bit bus
static int read_data_8(struct dm9000 *p, cyg_uint8 *dest)
{
    HAL_READ_UINT8(p->io_data, *dest);
    return 1;
}

// Read one datum from 16-bit bus 
static int read_data_16(struct dm9000 *p, cyg_uint8 *dest)
{
    cyg_uint16 val;

    HAL_READ_UINT16(p->io_data, val);
    memcpy(dest, &val, 2);
    return 2;
}

// Read one datum from 32-bit bus 
static int read_data_32(struct dm9000 *p, cyg_uint8 *dest)
{
    cyg_uint32 val;

    HAL_READ_UINT32(p->io_data, val);
    memcpy(dest, &val, 4);
    return 4;
}


// Write one datum to 8-bit bus
static int write_data_8(struct dm9000 *p, cyg_uint8 *src)
{
    HAL_WRITE_UINT8(p->io_data, *src);
    return 1;
}

// Write one datum to 16-bit bus 
static int write_data_16(struct dm9000 *p, cyg_uint8 *src)
{
    cyg_uint16 val;

    memcpy(&val, src, 2);
    HAL_WRITE_UINT16(p->io_data, val);
    return 2;
}

// Write one datum to 32-bit bus 
static int write_data_32(struct dm9000 *p, cyg_uint8 *src)
{
    cyg_uint32 val;

    memcpy(&val, src, 4);
    HAL_WRITE_UINT32(p->io_data, val);
    return 4;
}



// Return one byte from DM9000 register
static cyg_uint8 getreg(struct dm9000 *p, cyg_uint8 reg)
{
    cyg_uint8 val;
    HAL_WRITE_UINT8(p->io_addr, reg);
    HAL_READ_UINT8(p->io_data, val);
    return val;
}

// Write one byte to DM9000 register
static void putreg(struct dm9000 *p, cyg_uint8 reg, cyg_uint8 val)
{
    HAL_WRITE_UINT8(p->io_addr, reg);
    HAL_WRITE_UINT8(p->io_data, val);
}

// Read a word from EEPROM
static cyg_uint16 eeprom_read(struct dm9000 *p, int offset)
{
    putreg(p, DM_EPAR, offset);
    putreg(p, DM_EPCR, EPCR_ERPRR);
    while (getreg(p, DM_EPCR) & EPCR_ERRE)
	;
    CYGACC_CALL_IF_DELAY_US(8000);
    putreg(p, DM_EPCR, 0);
    return getreg(p, DM_EPDRL) | (getreg(p, DM_EPDRH) << 8);
}

// Write a word to EEPROM
static void eeprom_write(struct dm9000 *p, int offset, cyg_uint16 val)
{
    putreg(p, DM_EPAR, offset);
    putreg(p, DM_EPDRH, val >> 8);
    putreg(p, DM_EPDRL, val);
    putreg(p, DM_EPCR, EPCR_WEP | EPCR_ERPRW);
    while (getreg(p, DM_EPCR) & EPCR_ERRE)
	;
    CYGACC_CALL_IF_DELAY_US(8000);
    putreg(p, DM_EPCR, 0);
}

// Reload info from EEPROM
static void eeprom_reload(struct dm9000 *p)
{
    putreg(p, DM_EPCR, EPCR_REEP);
    while (getreg(p, DM_EPCR) & EPCR_ERRE)
	;
    CYGACC_CALL_IF_DELAY_US(8000);
    putreg(p, DM_EPCR, 0);
}


// Read a word from PHY
static cyg_uint16 phy_read(struct dm9000 *p, int offset)
{
    putreg(p, DM_EPAR, 0x40 + offset);
    putreg(p, DM_EPCR, EPCR_EPOS | EPCR_ERPRR);
    CYGACC_CALL_IF_DELAY_US(200);
    putreg(p, DM_EPCR, 0);
    return getreg(p, DM_EPDRL) | (getreg(p, DM_EPDRH) << 8);
}

// Write a word to PHY
static void phy_write(struct dm9000 *p, int offset, cyg_uint16 val)
{
    putreg(p, DM_EPAR, 0x40 + offset);
    putreg(p, DM_EPDRL, val);
    putreg(p, DM_EPDRH, val >> 8);
    putreg(p, DM_EPCR, EPCR_EPOS | EPCR_ERPRW);
    CYGACC_CALL_IF_DELAY_US(500);
    putreg(p, DM_EPCR, 0);
}


static void init_phy(struct dm9000 *p)
{
    int t = 0;
    cyg_uint16 r;

    /* power on PHY */
    putreg(p, DM_GPCR, 0x01);
    putreg(p, DM_GPR,  0x00);

    phy_write(p, PHY_ANAR, 0x1e1); // Advertise 10/100 half/full duplex w/CSMA
    phy_write(p, PHY_BMCR, PHY_BMCR_AUTO_NEG_EN | PHY_BMCR_AUTO_NEG_START);
   //phy_write(p, PHY_ANAR, 0x101); // for 100 full duplex 
   //phy_write(p, PHY_BMCR, 0x3100);
    /* wait for autonegotiation to complete */
    do {
        CYGACC_CALL_IF_DELAY_US(1000);
        r = phy_read(p, PHY_BMSR);
    } while (!(r & PHY_BMSR_AUTO_NEG_COMPLETE) && t++ < 2000);
}


static inline void dm9000_reset(struct dm9000 *p)
{
    putreg(p, DM_NCR, NCR_RST);
    CYGACC_CALL_IF_DELAY_US(100);
}

static int initialize_nic(struct dm9000 *priv)
{
    int i,lnk;

    dm9000_reset(priv);

    switch (getreg(priv, DM_ISR) >> 6) {
    case 0:
	priv->read_data = read_data_16;
	priv->write_data = write_data_16;
	priv->buswidth = 2;
	break;
    case 1:
	priv->read_data = read_data_32;
	priv->write_data = write_data_32;
	priv->buswidth = 4;
	break;
    case 2:
	priv->read_data = read_data_8;
	priv->write_data = write_data_8;
	priv->buswidth = 1;
	break;
    default:
	diag_printf("Unknown DM9000 bus i/f.\n");
	return 0;
    }

    init_phy(priv);

    putreg(priv, DM_TCR, 0);        /* TX Polling clear */
    putreg(priv, DM_BPTR, 0x3f);    /* Less 3Kb, 200us */
    putreg(priv, DM_FCTR, 0x38);   /* Flow Control : High/Low Water */
    putreg(priv, DM_FCR, 0xff);
    putreg(priv, DM_SMCR, 0);      /* Special Mode */
    putreg(priv, DM_NSR, NSR_WAKEST | NSR_TX1END | NSR_TX2END);  /* clear TX status */ // set 0x2c
    putreg(priv, DM_ISR, ISR_ROOS | ISR_ROS | ISR_PTS | ISR_PRS); /* Clear interrupt status */
    
    // set MAC address
    for (i = 0; i < 6; i++)
	putreg(priv, DM_PAR + i, priv->mac_address[i]);

    // clear multicast table except for broadcast address
    for (i = 0; i < 6; i++)
	putreg(priv, DM_MAR + i, 0x00);
    putreg(priv, DM_MAR + 6, 0x00);
    putreg(priv, DM_MAR + 7, 0x80);
	
	/* see what we've got */
	lnk = phy_read(priv, PHY_SCSR) >> 12;
	diag_printf("operating at ");
	switch (lnk) {
	case 1:
		diag_printf("10M half duplex ");
		break;
	case 2:
		diag_printf("10M full duplex ");
		break;
	case 4:
		diag_printf("100M half duplex ");
		break;
	case 8:
		diag_printf("100M full duplex ");
		break;
	case 15:
		diag_printf("Auto Negotiation"); 
		break;
	default:
		diag_printf("unknown: %d ", lnk);
		break;
	}
	diag_printf("mode\n");

    return 1;
}

#ifdef CYGINT_IO_ETH_INT_SUPPORT_REQUIRED
static cyg_uint32 dm9000_isr(cyg_vector_t vector, cyg_addrword_t data)
{
    struct eth_drv_sc *sc = (struct eth_drv_sc *)data;
    struct dm9000 *priv = (struct dm9000 *)sc->driver_private;

    cyg_drv_interrupt_mask(priv->interrupt);
    cyg_drv_interrupt_acknowledge(priv->interrupt);

    return CYG_ISR_HANDLED | CYG_ISR_CALL_DSR;
}
#endif


// ------------------------------------------------------------------------
//
//  API Function : dm9000_init
//
// ------------------------------------------------------------------------
static bool
dm9000_init(struct cyg_netdevtab_entry * ndp)
{
    struct eth_drv_sc *sc;
    struct dm9000 *priv;
    int i;
    unsigned id;
    unsigned short u16tab[64];

    sc = (struct eth_drv_sc *)ndp->device_instance;
    priv = (struct dm9000 *)sc->driver_private;

    priv->sc = sc;

#ifdef CYG_HAL_DM9000_PRESENT
    if (!CYG_HAL_DM9000_PRESENT())
	return 0;
#endif

    id = getreg(priv, DM_VIDL);
    id |= getreg(priv, DM_VIDH) << 8;
    id |= getreg(priv, DM_PIDL) << 16;
    id |= getreg(priv, DM_PIDH) << 24;

    if (id != 0x90000A46)
	return 0;

#ifdef CYGINT_IO_ETH_INT_SUPPORT_REQUIRED
    cyg_drv_interrupt_create(priv->interrupt,
                             0,
                             (cyg_addrword_t)sc,
                             dm9000_isr,
                             eth_drv_dsr,
                             &priv->interrupt_handle,
                             &priv->interrupt_object);
    cyg_drv_interrupt_attach(priv->interrupt_handle);
    cyg_drv_interrupt_acknowledge(priv->interrupt);
    cyg_drv_interrupt_unmask(priv->interrupt);
#endif // !CYGPKG_IO_ETH_DRIVERS_STAND_ALONE

    for (i = 0; i < 64; i++)
	u16tab[i] = eeprom_read(priv, i);

    u16tab[3] &= ~0xc;
    u16tab[3] |= 4;
    u16tab[6] &= 0xfe00;
    u16tab[6] |= 6;


#if 1   // for morgan test
    eeprom_write(priv, 6, u16tab[6]);
    eeprom_write(priv, 3, u16tab[3]);
	
	u16tab[0] = 0x12;
	u16tab[1] = 0x34;
	u16tab[2] = 0x56;	
	eeprom_write(priv, 0, u16tab[0]);
	eeprom_write(priv, 1, u16tab[1]);
	eeprom_write(priv, 2, u16tab[2]);
#endif

    eeprom_reload(priv);
#if 0  // morgan
    do {
	for (i = 0; i < 64; i++)
	    u16tab[i] = eeprom_read(priv, i);
    } while ((u16tab[0] | u16tab[1] | u16tab[2]) == 0);
	u16tab[0] = 0x12;
	u16tab[1] = 0x34;
	u16tab[2] = 0x56;		
	
    priv->mac_address[0] = u16tab[0];
    priv->mac_address[1] = u16tab[0] >> 8;
    priv->mac_address[2] = u16tab[1];
    priv->mac_address[3] = u16tab[1] >> 8;
    priv->mac_address[4] = u16tab[2];
    priv->mac_address[5] = u16tab[2] >> 8;
#endif
    if (!initialize_nic(priv))
	return 0;

    // Initialize upper level driver
    (sc->funs->eth_drv->init)(sc, &(priv->mac_address[0]) );
    return 1;
}

// ------------------------------------------------------------------------
//
//  API Function : dm9000_start
//
// ------------------------------------------------------------------------
static void 
dm9000_start( struct eth_drv_sc *sc, unsigned char *enaddr, int flags )
{
    struct dm9000 *priv = (struct dm9000 *)sc->driver_private;

    // turn on receiver
    putreg(priv, DM_RCR, RCR_DIS_LONG | RCR_DIS_CRC | RCR_RXEN);

    // unmask interrupt
    putreg(priv, DM_IMR, IMR_PAR | IMR_PTM | IMR_PRM);

    priv->active = 1;
}

// ------------------------------------------------------------------------
//
//  API Function : dm9000_stop
//
// ------------------------------------------------------------------------
static void
dm9000_stop( struct eth_drv_sc *sc )
{
    struct dm9000 *priv = (struct dm9000 *)sc->driver_private;

    // turn on receiver
    putreg(priv, DM_RCR, 0);

    // mask interrupts
    putreg(priv, DM_IMR, IMR_PAR);

    priv->active = 0;
}


// ------------------------------------------------------------------------
//
//  API Function : dm9000_recv
//
// ------------------------------------------------------------------------
static void 
dm9000_recv( struct eth_drv_sc *sc, struct eth_drv_sg *sg_list, int sg_len )
{
    struct dm9000 *priv = (struct dm9000 *)sc->driver_private;
    struct eth_drv_sg *sg = sg_list;
    cyg_uint8   tmpbuf[4];
    char *p;
    int len, total_len, nread, n, leftover;

    total_len = priv->rxlen;
    nread = leftover = 0;

//    diag_printf("dm9000_recv: total_len=%d\n", total_len);

    do {
	p = (char *)sg->buf;
	len = sg->len;

//	diag_printf("recv: buf=%p len=%d to_read=%d, leftover=%d\n", p, len, total_len - nread, leftover);

	if ((nread + len) > total_len)
	    len = total_len - nread;

	if (leftover) {
	    if (leftover <= len) {
		memcpy(p, tmpbuf + (sizeof(tmpbuf) - leftover), leftover);
		p += leftover;
		len -= leftover;
		nread += leftover;
		leftover = 0;
	    } else {
		memcpy(p, tmpbuf + (sizeof(tmpbuf) - leftover), len);
		leftover -= len;
		p += len;
		nread += len;
		len = 0;
	    }
	}

	while (len >= sizeof(tmpbuf)) {
	    n = priv->read_data(priv, p);
	    nread += n;
	    len -= n;
	    p += n;
	}

	while (len > 0) {
	    n = priv->read_data(priv, tmpbuf);
	    if (n <= len) {
		memcpy(p, tmpbuf, n);
		len -= n;
		nread += n;
		p += n;
	    } else {
		memcpy(p, tmpbuf, len);
		nread += len;
		leftover = n - len;
		len = 0;
	    } 
	}
	
	++sg;
    } while (nread < total_len);

#ifdef DEBUG_DUMP
    for (sg = sg_list; sg < (sg_list + sg_len); sg++) {
	diag_printf("\n");
	diag_dump_buf(sg->buf, sg->len);
    }
#endif
}

// ------------------------------------------------------------------------
//
//  API Function : dm9000_can_send
//
// ------------------------------------------------------------------------
static int 
dm9000_can_send(struct eth_drv_sc *sc)
{
    struct dm9000 *priv = (struct dm9000 *)sc->driver_private;

    if (!priv->active || priv->txbusy || priv->reset_pending)
	return 0;

    return 1;
}


// ------------------------------------------------------------------------
//
//  API Function : dm9000_send
//
// ------------------------------------------------------------------------
static void 
dm9000_send(struct eth_drv_sc *sc,
	    struct eth_drv_sg *sg_list, int sg_len,
	    int total_len, unsigned long key)
{
    struct dm9000 *priv = (struct dm9000 *)sc->driver_private;
    struct eth_drv_sg *sg;
    cyg_uint8 tmpbuf[4];
    int i, len, n, save_len, tail_extra;
    char *p;

#ifdef DEBUG
    diag_printf("dm9000_send: NCR[%02x] NSR[%02x] TRPA[%04x]\n",
                getreg(priv, DM_NCR), getreg(priv, DM_NSR),
                getreg(priv, DM_TRPAL) | (getreg(priv, DM_TRPAH) << 8)
        );
#endif
#ifdef DEBUG_DUMP
    for (sg = sg_list; sg < (sg_list + sg_len); sg++) {
	diag_printf("\n");
	diag_dump_buf(sg->buf, sg->len);
    }
#endif

    priv->txbusy = 1;

    sg = sg_list;
    save_len = total_len;
    tail_extra = 0;

    /* Disable all interrupts */
    putreg(priv, DM_IMR, IMR_PAR);

    HAL_WRITE_UINT8(priv->io_addr, DM_MWCMD);

    while (total_len > 0) {
	len = sg->len;
	if (len > total_len)
	    len = total_len;
	p = (char *)sg->buf;

        /* write any left over partial words by combining them with the start
         * of this sg block */
        if (tail_extra) {
            int head_extra = sizeof(tmpbuf) - tail_extra;
            memcpy(tmpbuf + tail_extra, p, head_extra);
            p += head_extra;
            len -= head_extra;
	    for (i = 0; i < sizeof(tmpbuf) && total_len > 0; i += n) {
		n = priv->write_data(priv, tmpbuf + i);
		total_len -= n;
	    }
            tail_extra = 0;
        }

        /* write out whole words */
        while (len >= priv->buswidth) {
	    n = priv->write_data(priv, p);
            len -= n;
            total_len -= n;
            p += n;
        }

        /* if we have some left over partial words... */
        if (len > 0) {
            /* combine them with the next sg block if available */
            if (total_len > len ) {
                tail_extra = len;
                memcpy(tmpbuf, p, tail_extra);
            } else {
                /* otherwise just write this last partial word */
                n = priv->write_data(priv, p);
                total_len -= n;
            }
        }
        sg++;
    }

    priv->txkey = key;

    putreg(priv, DM_TXPLL, save_len);
    putreg(priv, DM_TXPLH, save_len >> 8);

    putreg(priv, DM_TCR, TCR_TXREQ);

    /* Re-enable interrupt */
    putreg(priv, DM_IMR, IMR_PAR | IMR_PTM | IMR_PRM);
}

// ------------------------------------------------------------------------
//
//  API Function : dm9000_poll
//
// ------------------------------------------------------------------------
static void
dm9000_poll(struct eth_drv_sc *sc)
{
    struct dm9000 *priv = (struct dm9000 *)sc->driver_private;
    cyg_uint8 status, rxstat;
    cyg_uint16 pkt_stat, pkt_len;
    int i;

    // mask interrupts
    putreg(priv, DM_IMR, IMR_PAR);

    // get and clear staus
    status = getreg(priv, DM_ISR);
    putreg(priv, DM_ISR, status);

    // check for rx done
    if (1 /*status & ISR_PRS*/) {
        cyg_uint8 hdr[4]; /* 4 byte Rx pkt hdr */

        getreg(priv, DM_MRCMDX); /* dummy read */

        HAL_READ_UINT8(priv->io_data, rxstat);

	// check for packet ready
	if (rxstat == 1) {
            HAL_WRITE_UINT8(priv->io_addr, DM_MRCMD);
            for (i = 0; i < 4;)
                i += priv->read_data(priv, hdr + i);

            pkt_stat = hdr[0] | (hdr[1] << 8);
            pkt_len  = hdr[2] | (hdr[3] << 8);

#ifdef DEBUG
	    diag_printf("pkt_stat=%04x pkt_len=%04x\n", pkt_stat, pkt_len);
#endif

	    if (pkt_len < 0x40) {
		diag_printf("packet too short: %d (0x%04x)\n", pkt_len, pkt_len);
		i = 0;
		while (i < pkt_len)
		    i += priv->read_data(priv, hdr);
	    } else if (pkt_len > 1536) {
		priv->reset_pending = 1;
		diag_printf("packet too long: %d (0x%04x)\n", pkt_len, pkt_len);
	    } else if (pkt_stat & 0xbf00) {
		diag_printf("bad packet status: 0x%04x\n", pkt_stat);
		i = 0;
		while (i < pkt_len)
		    i += priv->read_data(priv, hdr);
	    } else {
		// receive packet
		priv->rxlen = pkt_len;
		(sc->funs->eth_drv->recv)(sc, pkt_len);
	    }

	} else if (rxstat > 1) {
	    // this should never happen.
	    diag_printf("unknown rxstat byte: %d\n", rxstat);
	    priv->reset_pending = 1;
	}
    }


    // check transmit status
    if (status & ISR_PTS) {
	cyg_uint8 txstat;

	txstat = getreg(priv, DM_NSR);

	if (txstat & (NSR_TX1END | NSR_TX2END)) {
	    if (txstat & NSR_TX1END)
		txstat = getreg(priv, DM_TSRI);
	    else
		txstat = getreg(priv, DM_TSRII);

	    if (txstat & TSR_COL) {
		// collision
	    }

	    if (getreg(priv, DM_TRPAL) & 3) {
		// NIC bug detected. Need to reset.
		//priv->reset_pending = 1;  //morgan test
		//diag_printf("NIC collision bug detected!\n");
	    }

	    (sc->funs->eth_drv->tx_done)(sc, priv->txkey, 0);
	    priv->txbusy = 0;
	}
    }

    if (priv->reset_pending && !priv->txbusy) {
	initialize_nic(priv);

	// turn on receiver
	putreg(priv, DM_RCR, RCR_DIS_LONG | RCR_DIS_CRC | RCR_RXEN);

	priv->reset_pending = 0;
    }

    // unmask interrupts
    putreg(priv, DM_IMR, IMR_PAR | IMR_PTM | IMR_PRM);
}


// ------------------------------------------------------------------------
//
//  API Function : dm9000_deliver
//
// ------------------------------------------------------------------------
static void
dm9000_deliver(struct eth_drv_sc *sc)
{
    struct dm9000 *priv = (struct dm9000 *)sc->driver_private;

    dm9000_poll(sc);

#ifdef CYGINT_IO_ETH_INT_SUPPORT_REQUIRED
    cyg_drv_interrupt_unmask(priv->interrupt);
#endif
}

// ------------------------------------------------------------------------
//
//  API Function : dm9000_int_vector
//
// ------------------------------------------------------------------------
static int
dm9000_int_vector(struct eth_drv_sc *sc)
{
    struct dm9000 *priv = (struct dm9000 *)sc->driver_private;

    return priv->interrupt;
}


// ------------------------------------------------------------------------
//
//  API Function : dm9000_ioctl
//
// ------------------------------------------------------------------------
static int
dm9000_ioctl(struct eth_drv_sc *sc, unsigned long key,
	  void *data, int data_length)
{
    struct dm9000 *priv = (struct dm9000 *)sc->driver_private;
    cyg_uint8 *esa = (cyg_uint8 *)data;
    int i;

    switch (key) {
#ifdef ETH_DRV_GET_MAC_ADDRESS
    case ETH_DRV_GET_MAC_ADDRESS:
        memcpy(esa, priv->mac_address, sizeof(priv->mac_address));
        return 0;
#endif
#ifdef ETH_DRV_SET_MAC_ADDRESS
    case ETH_DRV_SET_MAC_ADDRESS:
        for (i = 0; i < sizeof(priv->mac_address);  i++) {
            priv->mac_address[i] = esa[i];
            putreg(priv, DM_PAR + i, priv->mac_address[i]);
        }
#if defined(CYGSEM_DEVS_ETH_DAVICOM_DM9000_WRITE_EEPROM)
        for (i = 0; i < sizeof(priv->mac_address) / 2; i++)
            eeprom_write(priv, i, priv->mac_address[2*i] | (priv->mac_address[2*i+1] << 8));
#else
        diag_printf("dm9000: eeprom write disabled\n");
#endif
        return 0;
#endif
    }

    return -1;
}

// ------------------------------------------------------------------------
// EOF if_dm9000.c
