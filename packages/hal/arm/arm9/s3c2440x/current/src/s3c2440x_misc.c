//==========================================================================
//
//      s3c2440x_misc.c
//
//      HAL misc platform support code for Samsung S3C2440x
//
//==========================================================================
// ####ECOSGPLCOPYRIGHTBEGIN####
// -------------------------------------------
// This file is part of eCos, the Embedded Configurable Operating System.
// Copyright (C) 1998, 1999, 2000, 2001, 2002 Free Software Foundation, Inc.
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
// Date:		2009-12-22
// Purpose:		HAL platform support
// Description:		Implementations of HAL platform interfaces
//
//####DESCRIPTIONEND####
//
//==========================================================================

#include <pkgconf/hal.h>
#include <pkgconf/system.h>
#include CYGBLD_HAL_PLATFORM_H

#include <cyg/infra/cyg_type.h>		// base types
#include <cyg/infra/cyg_trac.h>		// tracing macros
#include <cyg/infra/cyg_ass.h>		// assertion macros

#include <cyg/hal/hal_io.h>		// IO macros
#include <cyg/hal/hal_arch.h>		// register state info
#include <cyg/hal/hal_diag.h>
#include <cyg/hal/hal_intr.h>		// interrupt names
#include <cyg/hal/hal_cache.h>
#include <cyg/hal/hal_s3c2440x.h>	// platform specifics

#include <cyg/infra/diag.h>		// diag_printf


// -------------------------------------------------------------------------
// Use Timer4 for system clock
void hal_clock_initialize(cyg_uint32 period)
{
	cyg_uint32	temp;

	// Configure the Prescaler1
	HAL_READ_UINT32(TCFG0, temp);
	temp &= ~(0xff<<8);
	temp |=  (CYGNUM_HAL_ARM_S3C2440X_TIMER_PRESCALE<<8);
	HAL_WRITE_UINT32(TCFG0, temp);

	// Configure the MUX to select the 1/2 divider
	HAL_READ_UINT32(TCFG1, temp);
	temp &= ~(0xf<<16);
	temp |=  (0x0<<16);
	HAL_WRITE_UINT32(TCFG1, temp);

	// Set up the Timer4 for period
	HAL_WRITE_UINT32(TCNTB4, period);

	// Start Timer4
	HAL_READ_UINT32(TCON, temp);
	temp &= ~(0xf << 20);
	HAL_WRITE_UINT32(TCON, (temp|(6<<20)));
	HAL_WRITE_UINT32(TCON, (temp|(5<<20)));

	// Unmask Timer4 interrupt, need not be done here
//	HAL_INTERRUPT_CONFIGURE(CYGNUM_HAL_INTERRUPT_RTC, 1, 1);
//	HAL_INTERRUPT_UNMASK(CYGNUM_HAL_INTERRUPT_RTC);
}

// This routine is called during a clock interrupt.
void hal_clock_reset(cyg_uint32 vector, cyg_uint32 period)
{
    // Do nothing
}

// Read the current value of the clock, returning the number of hardware
// "ticks" that have occurred (i.e. how far away the current value is from
// the start)

// Note: The "contract" for this function is that the value is the number
// of hardware clocks that have happened since the last interrupt (i.e.
// when it was reset).  This value is used to measure interrupt latencies.
// However, since the hardware counter runs freely, this routine computes
// the difference between the current clock period and the number of hardware
// ticks left before the next timer interrupt.
void hal_clock_read(cyg_uint32 *pvalue)
{
	cyg_int32	clock_val;

	// Read Timer4's current value
	HAL_READ_UINT32(TCNTO4, clock_val);
	*pvalue = CYGNUM_HAL_RTC_PERIOD - (clock_val & 0xFFFF);	// Note: counter is only 16 bits
								// and decreases
}

// Delay for some number of micro-seconds
void hal_delay_us(cyg_int32 usecs)
{
	cyg_uint32	ticks = 0;

	// Divide by 1000000 in two steps to preserve precision.
	cyg_uint32	wait_ticks = (((PCLK/100000)*usecs)/CYGNUM_HAL_ARM_S3C2440X_TIMER_PRESCALE/2/10);
	cyg_int32	val, prev, diff;

	// Read Timer4's current value
	HAL_READ_UINT32(TCNTO4, prev);
	prev &= 0xFFFF;
	while (ticks < wait_ticks) {
		while (true) {
			// Read Timer4's current value
			HAL_READ_UINT32(TCNTO4, val);
			val &= 0xFFFF;
			diff = prev - val;
			if (diff != 0) {
				if(diff < 0) {
					diff += (CYGNUM_HAL_RTC_PERIOD+1);
				}
				break;	// atleast 1 tick has passed
			}
		}
		prev   = val;
		ticks += diff;
	}
}


// -------------------------------------------------------------------------
// This routine is called to respond to a hardware interrupt (IRQ).  It
// should interrogate the hardware and return the IRQ vector number.
int hal_IRQ_handler(void)
{
	cyg_uint32	ior;

	HAL_READ_UINT32(INTOFFSET, ior);
	return (int) ior;
}

//----------------------------------------------------------------------------
// Interrupt control
void hal_interrupt_mask(int vector)
{
	cyg_uint32	imr;

	CYG_ASSERT(vector <= CYGNUM_HAL_ISR_MAX &&
		   vector >= CYGNUM_HAL_ISR_MIN , "Invalid vector");

	HAL_READ_UINT32(INTMSK, imr);
	imr |= (1<<vector);
	HAL_WRITE_UINT32(INTMSK, imr);
}

void hal_interrupt_unmask(int vector)
{
	cyg_uint32	imr;

	CYG_ASSERT(vector <= CYGNUM_HAL_ISR_MAX &&
		   vector >= CYGNUM_HAL_ISR_MIN , "Invalid vector");

	HAL_READ_UINT32(INTMSK, imr);
	imr &= ~(1<<vector);
	HAL_WRITE_UINT32(INTMSK, imr);
}

void hal_interrupt_acknowledge(int vector)
{
	cyg_uint32	ipr;

	CYG_ASSERT(vector <= CYGNUM_HAL_ISR_MAX &&
		   vector >= CYGNUM_HAL_ISR_MIN , "Invalid vector");

	HAL_WRITE_UINT32(SRCPND, (1<<vector));
	HAL_READ_UINT32(INTPND, ipr);
	HAL_WRITE_UINT32(INTPND, ipr);
}

void hal_interrupt_configure(int vector, int level, int up)
{
	CYG_ASSERT(vector <= CYGNUM_HAL_ISR_MAX &&
		   vector >= CYGNUM_HAL_ISR_MIN, "Invalid vector");
}

void hal_interrupt_set_level(int vector, int level)
{
	CYG_ASSERT(vector <= CYGNUM_HAL_ISR_MAX &&
		   vector >= CYGNUM_HAL_ISR_MIN, "Invalid vector");
}


//-----------------------------------------------------------------------------
// End of s3c2440x_misc.c
