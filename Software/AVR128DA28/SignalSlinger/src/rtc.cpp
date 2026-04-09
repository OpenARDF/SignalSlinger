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
 * Real-time-clock configuration helpers.
 *
 * This module contains support functions for:
 * - initializing the RTC from the primary or backup 32 kHz source
 * - applying and persisting RTC calibration values
 * - resetting the RTC counter while preserving the selected clock source
 *
 * Time interpretation and scheduled-event policy belong elsewhere.
 */

#include <ccp.h>
#include "defs.h"
#include "rtc.h"
#include "tcb.h"
#include "eeprommanager.h"
#include "globals.h"
static bool use_backup_clock = false;
uint16_t g_clock_calibration = EEPROM_CLOCK_CALIBRATION_DEFAULT;

/**
 * Initialize the RTC using the default calibration value and primary clock source.
 */
void RTC_init(void)
{
	RTC_init(EEPROM_CLOCK_CALIBRATION_DEFAULT);
}

/**
 * Reset the RTC count to zero while preserving the active clock-source selection.
 */
void RTC_zero(void)
{
	if(use_backup_clock)
	{
		RTC_init_backup(g_clock_calibration);
	}
	else
	{
		RTC_init(g_clock_calibration);
	}
}

/**
 * Initialize the RTC using the supplied calibration value and primary clock source.
 *
 * @param cal Calibration period value to apply to the RTC.
 */
void RTC_init(uint16_t cal)
{
	g_clock_calibration = cal;
	util_delay_ms(0);
	while((RTC.STATUS > 0) && util_delay_ms(500))
	{ /* Wait for all registers to be synchronized */
	}
	// Compare
	RTC.CMP = 0x00;

	// Count
	RTC.CNT = 0x00;

	// Period
	RTC.PER = CLAMP(32757u, cal, 32777u);

	// Clock selection: XOSC32K
	RTC.CLKSEL = 0x02;

	// CMP disabled; OVF enabled;
	RTC.INTCTRL = 0x01;

	// RUNSTDBY enabled; PRESCALER DIV1; CORREN disabled; RTCEN enabled;
	RTC.CTRLA = 0x81;
	RTC.DBGCTRL = 0x01; /* Run in debug mode */

	util_delay_ms(0);
	while((RTC.PITCTRLA > 0) && util_delay_ms(500))
	{ /* Wait for all registers to be synchronized */
	}
	// PI disabled;
	RTC.PITINTCTRL = 0x00;
}

/**
 * Read back the currently programmed RTC calibration period.
 *
 * @return RTC period register value.
 */
uint16_t RTC_get_cal(void)
{
	return RTC.PER;
}

/**
 * Initialize the RTC using the default calibration value and backup 32 kHz source.
 */
void RTC_init_backup(void)
{
	RTC_init_backup(EEPROM_CLOCK_CALIBRATION_DEFAULT);
}

/**
 * Initialize the RTC using the supplied calibration value and backup 32 kHz source.
 *
 * @param cal Calibration period value to apply to the RTC.
 */
void RTC_init_backup(uint16_t cal)
{
	g_clock_calibration = cal;
	use_backup_clock = true;

	/* Keep the internal 32 kHz oscillator running when the backup clock path is selected. */
	ccp_write_io((void *)&(CLKCTRL.OSC32KCTRLA),
	             1 << CLKCTRL_RUNSTDBY_bp /* Run standby: enabled */);

	util_delay_ms(0);
	while((RTC.STATUS > 0) && util_delay_ms(500))
	{ /* Wait for all registers to be synchronized */
	}
	// Compare
	RTC.CMP = 0x00;

	// Count
	RTC.CNT = 0x00;

	// Period
	RTC.PER = CLAMP(32757u, cal, 32777u);

	// Clock selection: OSC32K
	RTC.CLKSEL = 0x00;

	// CMP disabled; OVF enabled;
	RTC.INTCTRL = 0x01;

	// RUNSTDBY disabled; PRESCALER DIV1; CORREN disabled; RTCEN enabled;
	RTC.CTRLA = 0x81;
	RTC.DBGCTRL = 0x01; /* Run in debug mode */

	util_delay_ms(0);
	while((RTC.PITCTRLA > 0) && util_delay_ms(500))
	{ /* Wait for all registers to be synchronized */
	}
	// PI disabled;
	RTC.PITINTCTRL = 0x00;
}

/**
 * Reinitialize the RTC with a new calibration value and persist it to EEPROM.
 *
 * @param cal Calibration period value to apply to the RTC.
 */
void RTC_set_calibration(uint16_t cal)
{
	if(use_backup_clock)
	{
		RTC_init_backup(cal);
	}
	else
	{
		RTC_init(cal);
	}

	g_clock_calibration = cal;
	/* Persist the selected calibration so future boots reuse the same clock trim. */
	g_ee_mgr.updateEEPROMVar(Clock_calibration, (void *)&g_clock_calibration);
}
