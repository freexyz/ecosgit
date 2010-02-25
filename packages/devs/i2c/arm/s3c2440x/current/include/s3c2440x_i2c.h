#ifndef CYGONCE_I2C_S3C2440X_H
#define CYGONCE_I2C_S3C2440X_H
//==========================================================================
//
//      s3c2440x_i2c.h
//
//      I2C driver for Samsung S3C2440x ARM processors
//
//==========================================================================
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
//==========================================================================
//#####DESCRIPTIONBEGIN####
//
// Author(s):     shen
// Contributors:  shen
// Date:          2010-01-14
// Description:   I2C driver for S3C2440x
//####DESCRIPTIONEND####
//==========================================================================


//==========================================================================
//                               INCLUDES
//==========================================================================
#include <cyg/infra/cyg_type.h>
#include <cyg/hal/drv_api.h>

typedef enum cyg_s3c2440x_i2c_mode {
    CYG_S3C2440X_I2C_MODE_INVALID   = 0x00,
    CYG_S3C2440X_I2C_MODE_TX        = 0x01,
    CYG_S3C2440X_I2C_MODE_RX        = 0x02
} cyg_s3c2440x_i2c_mode;

//==========================================================================
// Single I2C bus sepecififc data
//==========================================================================
typedef struct cyg_s3c2440x_i2c_extra {

    cyg_uint8        i2c_mode;
    cyg_uint32       i2c_count;
    const cyg_uint8* i2c_txbuf;
    cyg_uint8*       i2c_rxbuf;
    cyg_bool         i2c_rx_start;
    cyg_bool         i2c_rxnak;

    cyg_uint8       i2c_completed;

    cyg_drv_mutex_t  i2c_lock; // For synchronizing between DSR and foreground
    cyg_drv_cond_t   i2c_wait;
    cyg_handle_t     i2c_interrupt_handle;// For initializing the interrupt
    cyg_interrupt    i2c_interrupt_data;
} cyg_s3c2440x_i2c_extra;


//==========================================================================
// I2C driver interface
//==========================================================================
externC void        cyg_s3c2440x_i2c_init(struct cyg_i2c_bus*);
externC cyg_uint32  cyg_s3c2440x_i2c_tx(const cyg_i2c_device*, 
                                       cyg_bool, const cyg_uint8*, 
                                       cyg_uint32, cyg_bool);
externC cyg_uint32  cyg_s3c2440x_i2c_rx(const cyg_i2c_device*, 
                                       cyg_bool, cyg_uint8*, 
                                       cyg_uint32, cyg_bool, cyg_bool);
externC void        cyg_s3c2440x_i2c_stop(const cyg_i2c_device*);

//==========================================================================
// I2C bus declaration macros
//=========================================================================
# define CYG_S3C2440X_I2C_BUS(_name_)			                 \
  static cyg_s3c2440x_i2c_extra _name_ ## _extra = {                     \
  i2c_count    :  0,                                                    \
  i2c_txbuf    :  NULL,                                                 \
  i2c_rxbuf    :  NULL,                                                 \
  i2c_completed     :  1                                                \
  } ;                                                                   \
  CYG_I2C_BUS(_name_,                                                   \
              cyg_s3c2440x_i2c_init,                                     \
              cyg_s3c2440x_i2c_tx,                                       \
              cyg_s3c2440x_i2c_rx,                                       \
              cyg_s3c2440x_i2c_stop,                                     \
              (void*) & ( _name_ ## _extra)) ;


//-----------------------------------------------------------------------------
#endif // #endif CYGONCE_I2C_S3C2440X_H
