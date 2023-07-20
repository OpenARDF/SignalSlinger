/*
 *  MIT License
 *
 *  Copyright (c) 2022 DigitalConfections
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

#include <tcb.h>

static uint32_t g_ms_counter = 0;
extern volatile uint16_t g_i2c0_timeout_ticks;
extern volatile uint16_t g_serial_timeout_ticks;

/**
 * \brief Initialize tcb interface
 *
 * \return Initialization status.
 */
int8_t TIMERB_init()
{
/**
TB0: Periodic tasks not requiring precise timing. Rate = 300 Hz
*/

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

/********************************************************************************/
/** 
LED Timer
*/
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


/********************************************************************************/
/** 
I2C Timeout Flag Timer
*/

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

bool util_delay_ms(uint32_t delayValue)
{
	static uint32_t countdownValue=0;
	static bool counting = false;
	
	if(!delay_initialized)
	{
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
			if(!g_ms_counter)
			{
 				TCA0.SINGLE.INTCTRL = 0 << TCA_SINGLE_OVF_bp; /* OverFlow Interrupt: disabled */
//				TCA0.SINGLE.CTRLA = 0x00; /* Disable TCA0 */
				counting = false;
				countdownValue = 0;
				return(false); /* time expired */
			}
			else if(delayValue != countdownValue) /* countdown delay changed while counting */
			{
 				TCA0.SINGLE.INTCTRL = 0 << TCA_SINGLE_OVF_bp; /* OverFlow Interrupt: disabled */
// 				TCA0.SINGLE.CTRLA = 0x00; /* Disable TCA0 */
 				TCA0.SINGLE.CNT = 0x0000;
				countdownValue = delayValue;
				g_ms_counter = delayValue;
 				TCA0.SINGLE.INTCTRL = 1 << TCA_SINGLE_OVF_bp; /* OverFlow Interrupt: enabled */
 				TCA0.SINGLE.CTRLA = 0x01; /* Enable TCA0 */
				return(false);
			}
		}
		else if(delayValue != countdownValue) /* New delay value received */
		{
 			TCA0.SINGLE.INTCTRL = 0 << TCA_SINGLE_OVF_bp; /* OverFlow Interrupt: disabled */
// 			TCA0.SINGLE.CTRLA = 0x00; /* Disable TCA0 */
 			TCA0.SINGLE.CNT = 0x0000;
			countdownValue = delayValue;
			g_ms_counter = delayValue;
 			TCA0.SINGLE.INTCTRL = 1 << TCA_SINGLE_OVF_bp; /* OverFlow Interrupt: enabled */
 			TCA0.SINGLE.CTRLA = 0x01; /* Enable TCA0 */
			counting = true;
		}
	}
	else /* Shut down the counter */
	{
 		TCA0.SINGLE.INTCTRL = 0 << TCA_SINGLE_OVF_bp; /* OverFlow Interrupt: disabled */
		TCA0.SINGLE.CTRLA = 0x00; /* Disable TCA0 */
		delay_initialized = false;
		counting = false;
		countdownValue = 0;
		g_ms_counter = 0;
		return(false); /* timer reset */
	}
	
 	TCA0.SINGLE.INTCTRL = 1 << TCA_SINGLE_OVF_bp; /* OverFlow Interrupt: enabled */	
	return(true);
}

ISR(TCA0_OVF_vect)
{
	uint8_t x = TCA0.SINGLE.INTFLAGS;
	
	if(x & TCA_SINGLE_OVF_bm)
	{
		if(g_ms_counter) g_ms_counter--;
	}
	
	TCA0.SINGLE.INTFLAGS = (TCA_SINGLE_OVF_bm | TCA_SINGLE_CMP0_bm | TCA_SINGLE_CMP1_bm | TCA_SINGLE_CMP2_bm); /* Clear all interrupt flags */
}

ISR(TCB2_INT_vect)
{
	uint8_t x = TCB2.INTFLAGS;
	
	if(x & TCB_CAPT_bm)
	{
		if(g_i2c0_timeout_ticks) g_i2c0_timeout_ticks--;
		if(g_serial_timeout_ticks) g_serial_timeout_ticks--;
	}
	
	TCB2.INTFLAGS = (TCB_CAPT_bm | TCB_OVF_bm);
}


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

