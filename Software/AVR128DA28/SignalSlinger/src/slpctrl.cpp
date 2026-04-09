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
 * Sleep-controller configuration helpers.
 *
 * This module contains support functions for:
 * - initializing the MCU sleep controller
 * - switching between supported hardware sleep modes
 *
 * Sleep policy and wakeup sequencing belong elsewhere.
 */

#include <slpctrl.h>

/**
 * Initialize the MCU sleep controller with sleep disabled.
 *
 * @return Initialization status code, with 0 indicating success.
 */
int8_t SLPCTRL_init()
{

	SLPCTRL.CTRLA = 0 << SLPCTRL_SEN_bp; /* Sleep enable: disabled */
//			 | SLPCTRL_SMODE_IDLE_gc;  /* Idle mode */
//			 | SLPCTRL_SMODE_STDBY_gc; /* Standby Mode */
//			 | SLPCTRL_SMODE_PDOWN_gc; /* Power-down Mode */

	return 0;
}

/**
 * Select the hardware sleep mode used when sleep is later enabled.
 *
 * @param setmode Sleep-mode value to apply to the controller register.
 */
void SLPCTRL_set_sleep_mode(SLPCTRL_SMODE_t setmode)
{
	SLPCTRL.CTRLA = (SLPCTRL.CTRLA & ~SLPCTRL_SMODE_gm) | (setmode & SLPCTRL_SMODE_gm);
}
