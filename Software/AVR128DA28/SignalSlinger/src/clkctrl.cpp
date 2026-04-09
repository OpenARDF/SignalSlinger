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
 * Clock-controller initialization helpers.
 *
 * This module contains support functions for:
 * - configuring the external 32 kHz clock source
 * - setting up the high-frequency oscillator used as the system clock
 *
 * Runtime clock-policy changes and peripheral timing decisions belong elsewhere.
 */

#include <clkctrl.h>
#include <ccp.h>
/**
 * Initialize the device clock controller for the firmware's expected clock sources.
 *
 * @return Initialization status code, with 0 indicating success.
 */
int8_t CLKCTRL_init()
{
	/* Enable the external 32 kHz source used by timekeeping-related peripherals. */
	ccp_write_io((void *)&(CLKCTRL.XOSC32KCTRLA),CLKCTRL_CSUT_1K_gc /* 1k cycles */
	| 1 << CLKCTRL_ENABLE_bp   /* Enable: enabled */
	| 1 << CLKCTRL_RUNSTDBY_bp /* Run standby: enabled */
	| 1 << CLKCTRL_SEL_bp      /* Select external clock: XTAL32K1 input only */
	| 0 << CLKCTRL_LPMODE_bp /* Low-Power Mode: disabled */);

	// 	ccp_write_io((void *)&(CLKCTRL.OSC32KCTRLA),
	// 			1 << CLKCTRL_RUNSTDBY_bp /* Run standby: enabled */);

	// ccp_write_io((void*)&(CLKCTRL.MCLKCTRLB),CLKCTRL_PDIV_2X_gc /* 2 */
	//		 | 0 << CLKCTRL_PEN_bp /* Prescaler enable: disabled */);

	// ccp_write_io((void*)&(CLKCTRL.PLLCTRLA),0 << CLKCTRL_RUNSTDBY_bp /* Run Standby: disabled */
	//		 | CLKCTRL_MULFAC_DISABLE_gc /* 1 */
	//		 | 0 << CLKCTRL_SOURCE_bp /* Select Source for PLL: disabled */);

	/* Configure the high-frequency oscillator as the primary fast clock source. */
	ccp_write_io((void*)&(CLKCTRL.OSCHFCTRLA),CLKCTRL_FRQSEL_24M_gc /* 4 */
	| 1 << CLKCTRL_AUTOTUNE_bp /* Auto-Tune enable: enabled */
	| 0 << CLKCTRL_RUNSTDBY_bp /* Run standby: disabled */);

	// ccp_write_io((void*)&(CLKCTRL.MCLKCTRLA),CLKCTRL_CLKSEL_OSCHF_gc /* Internal high-frequency oscillator */
	//		 | 0 << CLKCTRL_CLKOUT_bp /* System clock out: disabled */);

	// ccp_write_io((void*)&(CLKCTRL.MCLKLOCK),0 << CLKCTRL_LOCKEN_bp /* lock enable: disabled */);

	return 0;
}
