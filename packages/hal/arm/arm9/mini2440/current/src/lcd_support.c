//==========================================================================
//
//        Lcd_support.c
//
//        FriendlyARM MINI2440 - LCD support routines
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
// Date:          2010-01-05
// Description:   Simple LCD support
//####DESCRIPTIONEND####

#include <pkgconf/hal.h>
#include <pkgconf/system.h>
#include CYGBLD_HAL_BOARD_H

#include <cyg/infra/diag.h>
#include <cyg/hal/hal_io.h>       // IO macros
#include <cyg/hal/hal_if.h>       // Virtual vector support
#include <cyg/hal/hal_arch.h>     // Register state info
#include <cyg/hal/hal_intr.h>     // HAL interrupt macros

#include <cyg/hal/hal_s3c2440x.h>  // Board definitions
#include <cyg/hal/lcd_support.h>
#include <cyg/hal/plf_mmap.h>

#include <string.h>

#ifdef CYGOPT_MINI2440_LCD_SHOW_LOGO
#include "logo.h"
#endif


#ifdef CYGPKG_ISOINFRA
# include <pkgconf/isoinfra.h>
# ifdef CYGINT_ISO_STDIO_FORMATTED_IO
#  include <stdio.h>  // sscanf
# endif
#endif

#include CYGHWR_MEMORY_LAYOUT_H
#define LCD_FRAMEBUFFER CYGMEM_REGION_lcd
static cyg_uint32 lcd_framebuffer;

#ifndef FALSE
#define FALSE 0
#define TRUE  1
#endif

// Physical dimensions of LCD display
#define DISPLAY_WIDTH  	240
#define DISPLAY_HEIGHT 	320

// Logical layout
#define LCD_WIDTH  	DISPLAY_WIDTH
#define LCD_HEIGHT 	DISPLAY_HEIGHT
#define LCD_DEPTH  	16

#define USE_RGB565
#ifdef USE_RGB565
#define RGB_RED(x)   	(((x)&0x1F)<<11)
#define RGB_GREEN(x) 	(((x)&0x3F)<<5)
#define RGB_BLUE(x)  	((x)&0x1F)
#else
#define RGB_RED(x)   	(((x)&0x0F)<<11)
#define RGB_GREEN(x) 	(((x)&0x0F)<<6)
#define RGB_BLUE(x)  	(((x)&0x0F)<<1)
#endif

// Physical screen info
//static int lcd_depth  = LCD_DEPTH;  // Should be 1, 2, or 4
static int lcd_bpp;
static int lcd_height = LCD_HEIGHT;

// Black on light blue
static int bg = RGB_RED(0x17) | RGB_GREEN(0x17) | RGB_BLUE(0x1F);


//NEC35
#define MVAL		(13)
#define MVAL_USED 	(0)		//0=each frame   1=rate by MVAL
#define INVVDEN		(1)		//0=normal       1=inverted
#define BSWP		(0)		//Byte swap control
#define HWSWP		(1)		//Half word swap control

#define M5D(n) 		((n) & 0x1fffff)	// To get lower 21bits

#define LCD_XSIZE 	(240)	
#define LCD_YSIZE 	(320)

#define SCR_XSIZE 	(240)
#define SCR_YSIZE 	(320)

#define HOZVAL		(LCD_XSIZE-1)
#define LINEVAL		(LCD_YSIZE-1)

#define VBPD		(1)
#define VFPD		(5)
#define VSPW		(1)
#define HBPD		(36)
#define HFPD		(19)
#define HSPW		(5)

#define CLKVAL		(4)


// Compute the location for a pixel within the framebuffer
static cyg_uint32 *
lcd_fb(int row, int col)
{
    return (cyg_uint32 *)(LCD_FRAMEBUFFER+(((row*DISPLAY_WIDTH)+col)*2));
}

void
lcd_on(short enable)
{
#ifdef CYGOPT_MINI2440_LCD_SHOW_LOGO

    cyg_uint32 *fb_row0, *fb_rown, dly;
    cyg_uint16 *log_row0, *log_rown, i, x, y;

#endif   
  
    cyg_uint32 ctl;
    
#ifdef CYGOPT_MINI2440_LCD_SHOW_LOGO

    // fill logo background
    fb_row0 = lcd_fb(0, 0);
    fb_rown = lcd_fb(lcd_height, 0);
    while (fb_row0 != fb_rown) {
        *fb_row0++ = 0x0;
    }
    
    // show logo
    log_row0 = (cyg_uint16*)lcd_fb((lcd_height-LOGO_HEIGHT)>>1, (LCD_WIDTH-LOGO_WIDTH)>>1);

    i = y= 0;
    
    while (y < LOGO_HEIGHT) {
      
	log_rown = log_row0;
	
	x = 0;
	while (x++ < LOGO_WIDTH) {
	    *log_rown++ = logo[i++];
	}
	
	log_row0 += LCD_WIDTH;
	y++;
    }
    
#endif    
 
    HAL_READ_UINT32(LCDCON1, ctl);
    
    if(enable == TRUE){
	HAL_WRITE_UINT32(LCDCON1, ctl | 1); // ENVID=ON
    } else {
	HAL_WRITE_UINT32(LCDCON1, ctl & 0x3fffe); // ENVID Off
    }

#ifdef CYGOPT_MINI2440_LCD_SHOW_LOGO

    dly = 0x1000000;
    while(dly--){}

#endif   
    
}

// Initialize LCD hardware

void
lcd_init(int depth)
{
    cyg_uint32 ctl;
    cyg_uint32 invpwren, pwren;
      
    HAL_VIRT_TO_PHYS_ADDRESS(LCD_FRAMEBUFFER, lcd_framebuffer);	
    //Lcd_Port_Init
    HAL_WRITE_UINT32(GPCUP, 0xffffffff); // Disable Pull-up register
    HAL_WRITE_UINT32(GPCCON, 0xaaaa02a8); //Initialize VD[7:0],VM,VFRAME,VLINE,VCLK

    HAL_WRITE_UINT32(GPDUP, 0xffffffff); // Disable Pull-up register
    HAL_WRITE_UINT32(GPDCON, 0xaaaaaaaa); //Initialize VD[15:8]    
    
    //LCD_Init
    HAL_WRITE_UINT32(LCDCON1, (CLKVAL<<8)|(1<<7)|(3<<5)|(12<<1)|0);
    // TFT LCD panel,16bpp TFT,ENVID=off
    HAL_WRITE_UINT32(LCDCON2, (VBPD<<24)|(LINEVAL<<14)|(VFPD<<6)|(VSPW));
    HAL_WRITE_UINT32(LCDCON3, (HBPD<<19)|(HOZVAL<<8)|(HFPD));
    HAL_WRITE_UINT32(LCDCON4, (MVAL<<8)|(HSPW));
    HAL_WRITE_UINT32(LCDCON5, (1<<11) | (1<<10) | (1<<9) | (1<<8) | (0<<7) | (0<<6)
				| (1<<3)  |(BSWP<<1) | (HWSWP));
    HAL_WRITE_UINT32(LCDSADDR1, (((cyg_uint32)lcd_framebuffer>>22)<<21)|M5D((cyg_uint32)lcd_framebuffer>>1));
    HAL_WRITE_UINT32(LCDSADDR2, M5D( ((cyg_uint32)lcd_framebuffer+(SCR_XSIZE*LCD_YSIZE*2))>>1 ));
    HAL_WRITE_UINT32(LCDSADDR3, (((SCR_XSIZE-LCD_XSIZE)/1)<<11)|(LCD_XSIZE/1));
    HAL_READ_UINT32(LCDINTMSK, ctl);
    HAL_WRITE_UINT32(LCDINTMSK, ctl | 3); // MASK LCD Sub Interrupt
    HAL_READ_UINT32(LPCSEL, ctl);    
    HAL_WRITE_UINT32(LPCSEL, ctl & ~((1<<4)|1)); // Disable LCC3600, LPC3600
    HAL_WRITE_UINT32(TPAL, 0); // Disable Temp Palette 
	
    //GPG4 is setted as LCD_PWREN
    HAL_READ_UINT32(GPGUP, ctl);    
    HAL_WRITE_UINT32(GPGUP,ctl | (1<<4)); // Pull-up disable
    HAL_READ_UINT32(GPGCON, ctl); 
    HAL_WRITE_UINT32(GPGCON, ctl | (3<<8)); //GPG4=LCD_PWREN
    
    //Enable LCD POWER ENABLE Function
    invpwren 	= 0;
    pwren	= 1;
    HAL_READ_UINT32(LCDCON5, ctl);     
    HAL_WRITE_UINT32(LCDCON5, ctl & ((~(1<<3))|(pwren<<3)));   // PWREN
    HAL_READ_UINT32(LCDCON5, ctl);     
    HAL_WRITE_UINT32(LCDCON5, ctl & ((~(1<<5))|(invpwren<<5)));   // INVPWREN

    lcd_bpp = LCD_DEPTH;    	
}

// Get information about the frame buffer
int
lcd_getinfo(struct lcd_info *info)
{
    if (lcd_bpp == 0) {
        return 0;  // LCD not initialized
    }
    info->width = DISPLAY_WIDTH;
    info->height = DISPLAY_HEIGHT;
    info->bpp = lcd_bpp;
    info->fb = (void*)lcd_framebuffer;
    info->rlen = DISPLAY_WIDTH * 2;
    info->type = FB_TRUE_RGB565;
    return 1; // Information valid
}

// Clear screen
void
lcd_clear(void)
{
    cyg_uint32 *fb_row0, *fb_rown;
    cyg_uint32 _bg = (bg<<16)|bg;

    fb_row0 = lcd_fb(0, 0);
    fb_rown = lcd_fb(lcd_height, 0);
    while (fb_row0 != fb_rown) {
        *fb_row0++ = _bg;
    }
}



