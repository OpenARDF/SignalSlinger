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


#ifndef __RTC_H__
#define __RTC_H__

/**
 * Initialize the RTC using the default calibration value and primary clock source.
 */
void RTC_init(void);

/**
 * Initialize the RTC using the supplied calibration value and primary clock source.
 *
 * @param cal Calibration period value to apply to the RTC.
 */
void RTC_init(uint16_t cal);

/**
 * Initialize the RTC using the default calibration value and backup 32 kHz source.
 */
void RTC_init_backup(void);

/**
 * Initialize the RTC using the supplied calibration value and backup 32 kHz source.
 *
 * @param cal Calibration period value to apply to the RTC.
 */
void RTC_init_backup(uint16_t cal);

/**
 * Reinitialize the RTC with a new calibration value and persist it to EEPROM.
 *
 * @param cal Calibration period value to apply to the RTC.
 */
void RTC_set_calibration(uint16_t cal);

/**
 * Read back the currently programmed RTC calibration period.
 *
 * @return RTC period register value.
 */
uint16_t RTC_get_cal(void);

/**
 * Reset the RTC count to zero while preserving the active clock-source selection.
 */
void RTC_zero(void);

#endif //__RTC_H__
