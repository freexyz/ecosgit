#ifndef CYGONCE_HAL_MINI2440_PLATFORM_PLF_MMAP_H
#define CYGONCE_HAL_MINI2440_PLATFORM_PLF_MMAP_H
/*=============================================================================
// ####ECOSGPLCOPYRIGHTBEGIN####                                            
// -------------------------------------------                              
// This file is part of eCos, the Embedded Configurable Operating System.   
// Copyright (C) 2010 Free Software Foundation, Inc.
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
//=============================================================================
//#####DESCRIPTIONBEGIN####
//
// Author(s):    shen
// Contributors: shen
// Date:         2010-01-21
// Purpose:      Samsung MINI2440 series platform-specific memory mapping macros
// Description:  Macros to convert a cached, virtual address to
//               1) an uncached adddress for the same memory (if available)
//               2) the equivalent physical address for giving to external
//               hardware eg. DMA engines.
//               
//               NB: this mapping is expressed here statically, independent
//               of the actual mapping installed in the MMAP table.  So if
//               someone changes that, or its initialisation is changed,
//               then this module must change.  This is intended to be
//               efficient at a cost of generality.  It is also intended to
//               be called with constants (such as &my_struct) as input
//               args, so that all the work can be done at compile time,
//               with optimization on.
//
// Usage:        #include <cyg/hal/hal_cache.h>
//		 (which includes this file itself)
//
//####DESCRIPTIONEND####
//
//===========================================================================*/

#include <cyg/hal/hal_misc.h>

// Get the pagesize for a particular virtual address:

// This does not depend on the vaddr.
#define HAL_MM_PAGESIZE( vaddr, pagesize ) CYG_MACRO_START      \
    (pagesize) = SZ_1M;                                         \
CYG_MACRO_END

// Get the physical address from a virtual address:

// Only RAM and ROM are mapped; we just pass through all other values,
// rather than detecting nonexistent areas here.

#define HAL_VIRT_TO_PHYS_ADDRESS( vaddr, paddr ) CYG_MACRO_START           \
    cyg_uint32 _v_ = (cyg_uint32)(vaddr);                                  \
    if ( 64 * SZ_1M > _v_ )         /* 64Mb of SDRAM Bank 0 from 0-64Mb */ \
        _v_ += 0x300u * SZ_1M;                                             \
    else if ( 0x800u * SZ_1M > _v_ ) /* Space between RAM and mapped ROM */\
        /* no change */ ;                                                  \
    else if ( 0x802u * SZ_1M > _v_ ) /* Mapped boot ROM size 2Mb */       \
        _v_ -= 0x800u * SZ_1M;                                             \
    else                            /* Rest of it */                       \
        /* no change */ ;                                                  \
    (paddr) = _v_;                                                         \
CYG_MACRO_END

// Get the virtual address for a physical address:

// Only RAM and ROM are mapped; we just pass through all other values,
// rather than detecting nonexistent areas here.

#define HAL_PHYS_TO_VIRT_ADDRESS( paddr, vaddr ) CYG_MACRO_START           \
    cyg_uint32 _p_ = (cyg_uint32)(paddr);                                  \
    if ( 2 * SZ_1M > _p_ )         /* 2Mb Boot ROM mapped to 0x800Mb */  \
        _p_ += 0x800u * SZ_1M;                                             \
    else if ( 0x300u * SZ_1M > _p_ ) /* Space between ROM and SDRAM */     \
        /* no change */ ;                                                  \
    else if ( 0x340u * SZ_1M > _p_ ) /* Raw RAM size 64Mb */               \
        _p_ -= 0x300u * SZ_1M;                                             \
    else                            /* Rest of it */                       \
        /* no change */ ;                                                  \
    (vaddr) = _p_ ;                                                        \
CYG_MACRO_END

// Get a non-cacheable address for accessing the same store as a virtual
// (assumed cachable) address:

// Only RAM is mapped: ROM is only available cachable, everything else is
// not cachable anyway.

#define HAL_VIRT_TO_UNCACHED_ADDRESS( vaddr, uaddr ) CYG_MACRO_START       \                                               \
    (uaddr) = (vaddr);                                                        \
CYG_MACRO_END

//---------------------------------------------------------------------------
#endif // CYGONCE_HAL_MINI2440_PLATFORM_PLF_MMAP_H
// EOF plf_mmap.h
