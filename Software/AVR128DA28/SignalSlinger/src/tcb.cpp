/*
 *  MIT License
 *
 *  Copyright (c) 2026 DigitalConfections
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 */

/*
 * Timer-counter helper functions used by periodic firmware services.
 *
 * This module contains support functions for:
 * - configuring the TCB timers used for periodic tasks and timeout tracking
 * - providing a lightweight millisecond delay primitive
 * - stopping those timers when the device enters sleep
 *
 * Higher-level scheduling policy and ISR-side work belong elsewhere.
 */

#include <tcb.h>
#include <atomic.h>
#include "globals.h"

static volatile uint32_t g_ms_counter = 0;

/**
 * Read the shared millisecond countdown atomically.
 *
 * @return Current millisecond countdown value.
 */
static uint32_t ms_counter_read_atomic(void)
{
	uint32_t value;
	ENTER_CRITICAL(tcb_ms_counter_read);
	value = g_ms_counter;
	EXIT_CRITICAL(tcb_ms_counter_read);
	return value;
}

/**
 * Update the shared millisecond countdown atomically.
 *
 * @param value New countdown value to store.
 */
static void ms_counter_write_atomic(uint32_t value)
{
	ENTER_CRITICAL(tcb_ms_counter_write);
	g_ms_counter = value;
	EXIT_CRITICAL(tcb_ms_counter_write);
}

/**
 * Initialize the timer-counter peripherals used by periodic firmware services.
 *
 * TCB0 services coarse periodic foreground tasks, TCB1 drives LED timing, and
 * TCB2 provides timeout ticks for I2C and serial-related logic.
 *
 * @return Initialization status code, with 0 indicating success.
 */
int8_t TIMERB_init()
{
	/* TCB0: periodic tasks not requiring precise timing. */

TCB0.INTCTRL = 1 << TCB_CAPT_bp   /* Capture or Timeout: enabled */
| 0 << TCB_OVF_bp; /* OverFlow Interrupt: disabled */

// Set TOP
TCB0.CCMP = 0x9C40;

TCB0.CTRLA = TCB_CLKSEL_DIV2_gc     /* CLK_PER */
| 1 << TCB_ENABLE_bp   /* Enable: enabled */
| 0 << TCB_RUNSTDBY_bp /* Run Standby: disabled */
| 0 << TCB_SYNCUPD_bp  /* Synchronize Update: disabled */
| 0 << TCB_CASCADE_bp; /* Cascade Two Timer/Counters: disabled */

TCB0.INTFLAGS = (TCB_CAPT_bm | TCB_OVF_bm); /* Clear flag */

/* TCB1: LED timer. */
TCB1.INTCTRL = 1 << TCB_CAPT_bp   /* Capture or Timeout: enabled */
| 0 << TCB_OVF_bp; /* OverFlow Interrupt: disabled */

// Set TOP
TCB1.CCMP = 0xC350;

TCB1.CTRLA = TCB_CLKSEL_DIV2_gc     /* CLK_PER */
| 1 << TCB_ENABLE_bp   /* Enable: enabled */
| 0 << TCB_RUNSTDBY_bp /* Run Standby: disabled */
| 0 << TCB_SYNCUPD_bp  /* Synchronize Update: disabled */
| 0 << TCB_CASCADE_bp; /* Cascade Two Timer/Counters: disabled */

TCB1.INTFLAGS = (TCB_CAPT_bm | TCB_OVF_bm); /* Clear flag */


/* TCB2: timeout tick source for I2C and serial idle tracking. */

CPUINT.LVL1VEC = 30; /* Set to level 1 - highest priority interrupt */
TCB2.INTCTRL = 1 << TCB_CAPT_bp   /* Capture or Timeout: enabled */
| 0 << TCB_OVF_bp; /* OverFlow Interrupt: disabled */

// Set TOP
TCB2.CCMP = 0x5DC0; /* Approximate 1 ms interrupts */

TCB2.CTRLA = TCB_CLKSEL_DIV2_gc     /* CLK_PER */
| 1 << TCB_ENABLE_bp   /* Enable: enabled */
| 0 << TCB_RUNSTDBY_bp /* Run Standby: disabled */
| 0 << TCB_SYNCUPD_bp  /* Synchronize Update: disabled */
| 0 << TCB_CASCADE_bp; /* Cascade Two Timer/Counters: disabled */

TCB2.INTFLAGS = (TCB_CAPT_bm | TCB_OVF_bm); /* Clear flag */
CPUINT.LVL1VEC = 30; /* Set to level 1 - highest priority interrupt */

/********************************************************************************/

	return 0;
}


static bool delay_initialized = false;

/**
 * Start, update, or stop the shared millisecond delay timer.
 *
 * Passing a nonzero delay starts or refreshes the countdown. Passing 0 stops the
 * timer and clears the shared delay state.
 *
 * @param delayValue Delay in milliseconds, or 0 to reset the delay timer.
 * @return true while the delay is still active, false when it has expired or been reset.
 */
bool util_delay_ms(uint32_t delayValue)
{
	static uint32_t countdownValue=0;
	static bool counting = false;
	
	if(!delay_initialized)
	{
		/* Lazily configure TCA0 the first time delay service is requested. */
		delay_initialized = true;
		
		TCA0.SINGLE.CTRLA = 0x00; /* Disable TCA0 */
		TCA0.SINGLE.PER = 0x5DC0; /* Set interrupt period */
		TCA0.SINGLE.INTCTRL = 0x01; /* Overflow interrupt enable */
		TCA0.SINGLE.CTRLA = 0x01; /* Enable TCA0 */
		TCA0.SINGLE.INTFLAGS = (TCA_SINGLE_OVF_bm | TCA_SINGLE_CMP0_bm | TCA_SINGLE_CMP1_bm | TCA_SINGLE_CMP2_bm); /* Clear all interrupt flags */
	}
	
	if(delayValue)
	{
		if(counting)
		{
			/* Once the shared counter reaches zero, the requested delay has expired. */
			if(!ms_counter_read_atomic())
			{
 				TCA0.SINGLE.INTCTRL = 0 << TCA_SINGLE_OVF_bp; /* OverFlow Interrupt: disabled */
//				TCA0.SINGLE.CTRLA = 0x00; /* Disable TCA0 */
				counting = false;
				countdownValue = 0;
				return(false); /* time expired */
			}
			else if(delayValue != countdownValue) /* countdown delay changed while counting */
			{
				/* Refresh the countdown if the caller changes the requested delay mid-flight. */
 				TCA0.SINGLE.INTCTRL = 0 << TCA_SINGLE_OVF_bp; /* OverFlow Interrupt: disabled */
// 				TCA0.SINGLE.CTRLA = 0x00; /* Disable TCA0 */
				TCA0.SINGLE.CNT = 0x0000;
				countdownValue = delayValue;
				ms_counter_write_atomic(delayValue);
				TCA0.SINGLE.INTCTRL = 1 << TCA_SINGLE_OVF_bp; /* OverFlow Interrupt: enabled */
 				TCA0.SINGLE.CTRLA = 0x01; /* Enable TCA0 */
				counting = true;
				return(true);
			}
		}
		else if(delayValue != countdownValue) /* New delay value received */
		{
 			TCA0.SINGLE.INTCTRL = 0 << TCA_SINGLE_OVF_bp; /* OverFlow Interrupt: disabled */
// 			TCA0.SINGLE.CTRLA = 0x00; /* Disable TCA0 */
 			TCA0.SINGLE.CNT = 0x0000;
			countdownValue = delayValue;
			ms_counter_write_atomic(delayValue);
 			TCA0.SINGLE.INTCTRL = 1 << TCA_SINGLE_OVF_bp; /* OverFlow Interrupt: enabled */
 			TCA0.SINGLE.CTRLA = 0x01; /* Enable TCA0 */
			counting = true;
		}
	}
	else /* Shut down the counter */
	{
		/* Reset the shared delay service completely when the caller passes zero. */
 		TCA0.SINGLE.INTCTRL = 0 << TCA_SINGLE_OVF_bp; /* OverFlow Interrupt: disabled */
		TCA0.SINGLE.CTRLA = 0x00; /* Disable TCA0 */
		delay_initialized = false;
		counting = false;
		countdownValue = 0;
		ms_counter_write_atomic(0);
		return(false); /* timer reset */
	}
	
 	TCA0.SINGLE.INTCTRL = 1 << TCA_SINGLE_OVF_bp; /* OverFlow Interrupt: enabled */	
	return(true);
}

/**
 * TCA0 overflow ISR backing the shared millisecond delay service.
 */
ISR(TCA0_OVF_vect)
{
	uint8_t x = TCA0.SINGLE.INTFLAGS;
	
	if(x & TCA_SINGLE_OVF_bm)
	{
		if(g_ms_counter) g_ms_counter--;
	}
	
	TCA0.SINGLE.INTFLAGS = (TCA_SINGLE_OVF_bm | TCA_SINGLE_CMP0_bm | TCA_SINGLE_CMP1_bm | TCA_SINGLE_CMP2_bm); /* Clear all interrupt flags */
}

/**
 * TCB2 ISR backing I2C/serial timeout tracking and receive-idle detection.
 */
ISR(TCB2_INT_vect)
{
	uint8_t x = TCB2.INTFLAGS;
	
	if(x & TCB_CAPT_bm)
	{
		if(g_i2c0_timeout_ticks) g_i2c0_timeout_ticks--;
		if(g_serial_timeout_ticks) g_serial_timeout_ticks--;
		serialbus_rx_idle_tick();
	}
	
	TCB2.INTFLAGS = (TCB_CAPT_bm | TCB_OVF_bm);
}


/**
 * Stop the TCB timers before entering a low-power sleep state.
 *
 * @return Initialization status code, with 0 indicating success.
 */
int8_t TIMERB_sleep()
{
	TCB0.INTCTRL = 0;   /* Capture or Timeout: disable interrupts */
	TCB0.CTRLA = 0; /* Disable timer */

	/********************************************************************************/

	TCB1.INTCTRL = 0; /* OverFlow Interrupt: disabled */
	TCB1.CTRLA = 0; /* Disable timer */

	/********************************************************************************/

	TCB2.INTCTRL = 0; /* OverFlow Interrupt: disabled */
	TCB2.CTRLA = 0; /* Disable timer */

	return 0;
}
