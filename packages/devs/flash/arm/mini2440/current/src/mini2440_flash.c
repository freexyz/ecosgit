//==========================================================================
//
//      mini2440_flash.c
//
//      Flash programming for AMD/SST Flash device on FriendlyARM MINI2440 board
//
//==========================================================================
// ####ECOSGPLCOPYRIGHTBEGIN####                                            
// -------------------------------------------                              
// This file is part of eCos, the Embedded Configurable Operating System.   
// Copyright (C) 2009 Free Software Foundation, Inc.
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
//==========================================================================
//#####DESCRIPTIONBEGIN####
//
// Author(s):		T.C. Chiu <TCChiu@sq.com.tw>
// Contributors:	T.C. Chiu <TCChiu@sq.com.tw>
// Date:		2009-12-28
// Purpose:
// Description:
//
//####DESCRIPTIONEND####
//
//==========================================================================

//--------------------------------------------------------------------------
// Device properties
#include <pkgconf/devs_flash_arm_mini2440.h>
#include <cyg/hal/mini2440.h>

// The FriendlyARM MINI2440 has either one SST39VF1601 part or one AM29LV160D part
#define CYGNUM_FLASH_INTERLEAVE		(1)
#define CYGNUM_FLASH_SERIES		(1)
#define CYGNUM_FLASH_WIDTH		(16)
#define CYGNUM_FLASH_BASE		MINI2440_FLASH_VIRT_BASE

//--------------------------------------------------------------------------
// Platform specific extras
#define CYGPKG_DEVS_FLASH_SST_39VF1601

//--------------------------------------------------------------------------
// Now include the driver code
#ifdef CYGINT_DEVS_FLASH_SST_39VFXXX_REQUIRED
#include "cyg/io/flash_sst_39vfxxx.inl"

static const cyg_flash_block_info_t	cyg_flash_sst_block_info[1] = {
	{ FLASH_BLOCK_SIZE, FLASH_NUM_REGIONS*CYGNUM_FLASH_SERIES }
};

CYG_FLASH_DRIVER(
	cyg_flash_sst_flashdev,
	&cyg_sst_funs,
	0,				// Flags
	CYGNUM_FLASH_BASE,		// Start
	CYGNUM_FLASH_BASE + (FLASH_BLOCK_SIZE*FLASH_NUM_REGIONS*CYGNUM_FLASH_SERIES) - 1,	// End
	1,				// Number of block infos
	cyg_flash_sst_block_info,
	NULL				// priv
);
#endif

#ifdef CYGINT_DEVS_FLASH_AMD_AM29XXXXX_REQUIRED
#include "cyg/io/flash_am29xxxxx.inl"
#endif
// ------------------------------------------------------------------------

// ------------------------------------------------------------------------
// EOF mini2440_flash.c