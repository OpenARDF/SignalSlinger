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


#ifndef TCB_H_INCLUDED
#define TCB_H_INCLUDED

#include <compiler.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize the timer-counter peripherals used by periodic firmware services.
 *
 * @return Initialization status code, with 0 indicating success.
 */
int8_t TIMERB_init();

/**
 * Start, update, or stop the shared millisecond delay timer.
 *
 * Passing a nonzero delay starts or refreshes the countdown. Passing 0 stops the
 * timer and clears the shared delay state.
 *
 * @param delayValue Delay in milliseconds, or 0 to reset the delay timer.
 * @return true while the delay is still active, false when it has expired or been reset.
 */
bool util_delay_ms(uint32_t delayValue);

/**
 * Stop the TCB timers before entering a low-power sleep state.
 *
 * @return Initialization status code, with 0 indicating success.
 */
int8_t TIMERB_sleep();

#ifdef __cplusplus
}
#endif

#endif /* TCB_H_INCLUDED */
