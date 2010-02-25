//==========================================================================
//
//      s3c2440x_i2c.c
//
//      I2C driver for S3C2440x
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
// Author(s):    shen
// Contributors: shen
// Date:         2010-01-14  
// Description:  I2C driver for motorola coldfire processor
//              
//####DESCRIPTIONEND####
//
//==========================================================================


//==========================================================================
//                                 INCLUDES
//==========================================================================
#include <pkgconf/system.h>
#include <pkgconf/devs_i2c_arm_s3c2440x.h>
#include <cyg/hal/hal_s3c2440x.h>  
#include <cyg/infra/cyg_type.h>
#include <cyg/infra/cyg_ass.h>
#include <cyg/infra/diag.h>
#include <cyg/io/i2c.h>
#include <cyg/io/s3c2440x_i2c.h>
#include <cyg/hal/hal_arch.h>
#include <cyg/hal/hal_io.h>
#include <cyg/hal/hal_intr.h>
#include <cyg/hal/hal_var_ints.h>
#include <cyg/hal/drv_api.h>

//
// According to the Users Manual the S3C2440x I2C module is very
// similar to the I2C module of the Philips 8xC552/556 controllers. I
// guess it is used in other Philips/NXP controllers, too. Using these
// macros should make it easier to split off the common parts of the
// driver once it's necessary.
//

# define    I2C_BASE(_extra_)       (cyg_uint8*)HAL_S3C2440X_I2C_SINGLETON_BASE
# define    I2C_ISRVEC(_extra_)     CYGNUM_HAL_INTERRUPT_IIC
# define    I2C_ISRPRI(_extra_)     0//HAL_S3C2440X_I2C_SINGLETON_ISRPRI


static void Delay(int time);   
   
   
  
#if CYGPKG_DEVS_I2C_ARM_S3C2440X_DEBUG_LEVEL > 0
   #define debug1_printf(args...) diag_printf(args)
#else
   #define debug1_printf(args...)
#endif
#if CYGPKG_DEVS_I2C_ARM_S3C2440X_DEBUG_LEVEL > 1
   #define debug2_printf(args...) diag_printf(args)
#else
   #define debug2_printf(args...)
#endif



//==========================================================================
// The ISR does the actual work. It is not that much work to justify
// putting it in the DSR, and it is also not clear whether this would
// even work.  If an error occurs we try to leave the bus in the same
// state as we would if there was no error.
//==========================================================================
static cyg_uint32 s3c2440x_i2c_isr(cyg_vector_t vec, cyg_addrword_t data)
{
	cyg_s3c2440x_i2c_extra* extra = (cyg_s3c2440x_i2c_extra*)data;
	cyg_uint32              result  = CYG_ISR_HANDLED;
	
  	cyg_uint32 iicSt, i;   
	cyg_uint32 dr;     
       
	HAL_READ_UINT32(IICSTAT, iicSt);    
      
    	switch (extra->i2c_mode){       
    	case CYG_S3C2440X_I2C_MODE_TX:   
		//Arbitration status flag, Last-received bit status flag
		if(iicSt & 0x09){	
		    extra->i2c_count++;
		    result              = CYG_ISR_HANDLED | CYG_ISR_CALL_DSR;    
		    break;       
		} 
	
    		//check the data is already transmited or not, if it's finished transfer the "P" signal.
		if (extra->i2c_count == 0) {   
		    result              = CYG_ISR_HANDLED | CYG_ISR_CALL_DSR;    
		    break;       
            	}   
   
            	HAL_WRITE_UINT32(IICDS, *extra->i2c_txbuf);
		extra->i2c_txbuf++;
		extra->i2c_count--;
		
            	// SDA line should take some times for data transfer after data inputing to the register "IICDS".     
            	for (i = 0; i < 10; i++);      
   
            	HAL_WRITE_UINT32(IICCON, 0xaf);      // recover the i2c process    
            	break;   
           
   
        case CYG_S3C2440X_I2C_MODE_RX:    
		if(extra->i2c_rx_start == true){
		    //Arbitration status flag, Last-received bit status flag
		    if(iicSt & 0x09){	
			result              = CYG_ISR_HANDLED | CYG_ISR_CALL_DSR;    
			break;       
		    } 
		
		    if((extra->i2c_count == 1) && extra->i2c_rxnak)
			HAL_WRITE_UINT32(IICCON, 0x2f); //Resumes IIC operation with NOACK.  
		    else 
			HAL_WRITE_UINT32(IICCON, 0xaf); //Resumes IIC operation with ACK	
			
		    extra->i2c_rx_start = false;	    
		} else {
		  
		    if((extra->i2c_count == 1) && extra->i2c_rxnak) {
			//Arbitration status flag
			if(iicSt & 0x08){	
			    result              = CYG_ISR_HANDLED | CYG_ISR_CALL_DSR;    
			    break;       
			} 		      
		    } else {		      
			//Arbitration status flag, Last-received bit status flag
			if(iicSt & 0x09){	
			    result              = CYG_ISR_HANDLED | CYG_ISR_CALL_DSR;    
			    break;       
			} 		      
		    }
    
		    HAL_READ_UINT32(IICDS, dr);
		    *(extra->i2c_rxbuf) = (cyg_uint8)dr;
		    extra->i2c_rxbuf++;
		    extra->i2c_count--;
		    
		    if(extra->i2c_count == 0) {		  
			  result              = CYG_ISR_HANDLED | CYG_ISR_CALL_DSR;
			  break;
		    }
		    
		    if((extra->i2c_count == 1) && extra->i2c_rxnak)
			HAL_WRITE_UINT32(IICCON, 0x2f); //Resumes IIC operation with NOACK.  
		    else 
			HAL_WRITE_UINT32(IICCON, 0xaf); //Resumes IIC operation with ACK	
		    
		}          
              
           	break;
           	
	default:
		// Invalid state? Some kind of spurious interrupt? Just ignore
		// it.
		CYG_FAIL("I2C spurious interrupt");	  
		break;         
	}
	
	cyg_drv_interrupt_acknowledge(vec);
	return result;	
}


//==========================================================================
// DSR signals data
//==========================================================================
static void
s3c2440x_i2c_dsr(cyg_vector_t vec, cyg_ucount32 count, cyg_addrword_t data)
{
    cyg_s3c2440x_i2c_extra*  extra   = (cyg_s3c2440x_i2c_extra*)data;
    extra->i2c_completed    = 1;
    cyg_drv_cond_signal(&(extra->i2c_wait));
}



//==========================================================================
// Initialize driver & hardware state
//==========================================================================
void cyg_s3c2440x_i2c_init(struct cyg_i2c_bus *bus)
{

    cyg_s3c2440x_i2c_extra* extra = (cyg_s3c2440x_i2c_extra*)bus->i2c_extra;
    cyg_uint32		ctl;
	
    cyg_drv_mutex_init(&extra->i2c_lock);
    cyg_drv_cond_init(&extra->i2c_wait, &extra->i2c_lock);
    cyg_drv_interrupt_create(I2C_ISRVEC(extra),
                             I2C_ISRPRI(extra),
                             (cyg_addrword_t) extra,
                             &s3c2440x_i2c_isr,
                             &s3c2440x_i2c_dsr,
                             &(extra->i2c_interrupt_handle),
                             &(extra->i2c_interrupt_data));
    cyg_drv_interrupt_attach(extra->i2c_interrupt_handle);	
	
	//GPEUP  |= 0xc000;       // disable put up
	//GPECON |= 0xa0000000;   // GPE15:IICSDA , GPE14:IICSCL    	
	HAL_READ_UINT32(GPEUP, ctl); 
	HAL_WRITE_UINT32(GPEUP, ctl | 0xc000); 
	HAL_READ_UINT32(GPECON, ctl); 
	HAL_WRITE_UINT32(GPECON, ctl | 0xa0000000); 
   
	//INTMSK &= ~(BIT_IIC);
	// Interrupts can now be safely unmasked
	HAL_INTERRUPT_UNMASK(I2C_ISRVEC(extra));
    
	/* bit[7] = 1, enable acknowledge  
	* bit[6] = 0, IICCLK = PCLK/16  
	* bit[5] = 1, enable interrupt  
	* bit[3:0] = 0xf, Tx clock = IICCLK/16  
	* PCLK = 50MHz, IICCLK = 3.125MHz, Tx Clock = 0.195MHz  
	*/   
	HAL_WRITE_UINT32(IICCON, (1<<7) | (0<<6) | (1<<5) | (0xf));  // 0xaf    
   
	HAL_WRITE_UINT32(IICADD, 0x10);     // S3C24xx slave address = [7:1]    
	HAL_WRITE_UINT32(IICSTAT, 0x10);    // enable transmit or recive mode(Rx/Tx)     
  
}


//==========================================================================
// transmit a buffer to a device
//==========================================================================
cyg_uint32 cyg_s3c2440x_i2c_tx(const cyg_i2c_device *dev, 
                              cyg_bool              send_start, 
                              const cyg_uint8      *tx_data, 
                              cyg_uint32            count, 
                              cyg_bool              send_stop)
{

    cyg_s3c2440x_i2c_extra* extra = 
                           (cyg_s3c2440x_i2c_extra*)dev->i2c_bus->i2c_extra;
			   
    extra->i2c_mode  = CYG_S3C2440X_I2C_MODE_TX;   	// transmit operation   		   
    extra->i2c_txbuf = tx_data;
    extra->i2c_count = count;
    
    if(send_start) 
    {
	HAL_WRITE_UINT32(IICDS, dev->i2c_address << 1);   
	HAL_WRITE_UINT32(IICSTAT, 0xf0);         //MasTx,Start
    } 
    else 
    {
        HAL_WRITE_UINT8(IICDS, *(extra->i2c_txbuf));
        extra->i2c_txbuf++;
	extra->i2c_count--;
    }

    HAL_WRITE_UINT32(IICCON, 0xaf);	//Resumes IIC operation. 

    extra->i2c_completed  = 0;
	
    //
    // the isr will do most of the work, and the dsr will signal when an
    // error occured or the transfer finished
    //
    cyg_drv_mutex_lock(&extra->i2c_lock);
    cyg_drv_dsr_lock();
    cyg_drv_interrupt_unmask(I2C_ISRVEC(extra));
    
    while(extra->i2c_completed == 0)
    {
        cyg_drv_cond_wait(&extra->i2c_wait);
    }
    
    cyg_drv_interrupt_mask(I2C_ISRVEC(extra));
    cyg_drv_dsr_unlock();
    cyg_drv_mutex_unlock(&extra->i2c_lock);
    
    if (send_stop) {
      HAL_WRITE_UINT32(IICSTAT, 0xd0);  //Stop MasTx condition 
      HAL_WRITE_UINT32(IICCON, 0xaf);	//Resumes IIC operation. 
      Delay(10);                        //Wait until stop condtion is in effect.      
      
      extra->i2c_mode     = CYG_S3C2440X_I2C_MODE_INVALID;      
    }
    
    extra->i2c_txbuf = NULL;	
	
    // tx() should return the number of bytes actually transmitted.
    // ISR() increments extra->count after a failure, which leads to
    // an edge condition when send_start and there is no acknowledgment
    // of the address byte.
    if (extra->i2c_count > count) {
        return 0;
    }
    return count - extra->i2c_count;
}

 
//==========================================================================
// receive into a buffer from a device
//==========================================================================
cyg_uint32 cyg_s3c2440x_i2c_rx(const cyg_i2c_device *dev,
                              cyg_bool              send_start,
                              cyg_uint8            *rx_data,
                              cyg_uint32            count,
                              cyg_bool              send_nak,
                              cyg_bool              send_stop)
{

    cyg_s3c2440x_i2c_extra* extra = 
                           (cyg_s3c2440x_i2c_extra*)dev->i2c_bus->i2c_extra;    
			   
    extra->i2c_mode  = CYG_S3C2440X_I2C_MODE_RX;   // recive operation   		   			   
    extra->i2c_count = count;
    extra->i2c_rxbuf = rx_data;   
    extra->i2c_rxnak = send_nak;
  
    if(send_start) 
    {
	HAL_WRITE_UINT32(IICDS, (dev->i2c_address << 1) | 0x01);   
	HAL_WRITE_UINT32(IICSTAT, 0xb0);        //MasRx,Start
	HAL_WRITE_UINT32(IICCON, 0xaf); 	//Resumes IIC operation with ACK
	
	extra->i2c_rx_start = true;
    } else {
	if (count == 0) {
	    return 0;
	}
	
	if((extra->i2c_count == 1) && extra->i2c_rxnak)
	    HAL_WRITE_UINT32(IICCON, 0x2f); //Resumes IIC operation with NOACK.  
	else 
	    HAL_WRITE_UINT32(IICCON, 0xaf); //Resumes IIC operation with ACK	
	    
	extra->i2c_rx_start = false;	    
    }


    extra->i2c_completed  = 0;
     
    //
    // the isr will do most of the work, and the dsr will signal when an
    // error occured or the transfer finished
    //
    cyg_drv_mutex_lock(&extra->i2c_lock);
    cyg_drv_dsr_lock();
    cyg_drv_interrupt_unmask(I2C_ISRVEC(extra));

    while(extra->i2c_completed == 0)
    {
        cyg_drv_cond_wait(&extra->i2c_wait);
    }
    
    cyg_drv_interrupt_mask(I2C_ISRVEC(extra));
    cyg_drv_dsr_unlock();
    cyg_drv_mutex_unlock(&extra->i2c_lock);
    
    
    if (send_stop) {
      HAL_WRITE_UINT32(IICSTAT, 0x90);  //Stop MasRx condition 
      HAL_WRITE_UINT32(IICCON, 0xaf);	//Resumes IIC operation. 
      Delay(10);                        //Wait until stop condtion is in effect.      
      
      extra->i2c_mode     = CYG_S3C2440X_I2C_MODE_INVALID;      
    }

    extra->i2c_rxbuf = NULL;
    
    if (extra->i2c_count > count) {
        return 0;
    }    
    
    return count - extra->i2c_count;
}


//==========================================================================
//  generate a STOP
//==========================================================================
void cyg_s3c2440x_i2c_stop(const cyg_i2c_device *dev)
{
  
    cyg_s3c2440x_i2c_extra* extra = 
                           (cyg_s3c2440x_i2c_extra*)dev->i2c_bus->i2c_extra;

    if (extra->i2c_mode == CYG_S3C2440X_I2C_MODE_TX) {
      HAL_WRITE_UINT32(IICSTAT, 0xd0);  //Stop MasTx condition 
    } else  if (extra->i2c_mode == CYG_S3C2440X_I2C_MODE_RX){
      HAL_WRITE_UINT32(IICSTAT, 0x90);  //Stop MasRx condition       
    }
    
    HAL_WRITE_UINT32(IICCON, 0xaf);	//Resumes IIC operation. 
    Delay(10);                          //Wait until stop condtion is in effect.          
       
    extra->i2c_mode     = CYG_S3C2440X_I2C_MODE_INVALID;
  
}


/*  
 * delay function  
 */   
static void Delay(int time)   
{
	for (; time > 0; time--);   
} 

//---------------------------------------------------------------------------
// eof i2c_s3c2440x.c
