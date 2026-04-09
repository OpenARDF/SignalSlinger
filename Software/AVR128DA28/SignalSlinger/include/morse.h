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
 * Morse-code generation helpers shared across the firmware.
 *
 * This module contains support functions for:
 * - incremental Morse timing and carrier gating
 * - estimating message duration from code speed
 * - tracking which subsystem currently owns the Morse generator
 *
 * RF keying, throttling, and transmission scheduling belong elsewhere.
 */

#ifndef MORSE_H_
#define MORSE_H_

#include "defs.h"

#define PROCESSSOR_CLOCK_HZ			(8000000L)
#define WPM_TO_MS_PER_DOT(w)		(1200/(w))
#define THROTTLE_VAL_FROM_WPM(w)	(PROCESSSOR_CLOCK_HZ / 8000000L) * ((7042 / (w)) / 10)

typedef enum{
	NO_CALLER,
	CALLER_AUTOMATED_EVENT,
	CALLER_MANUAL_TRANSMISSIONS
} callerID_t;

/**
 * Load or advance the incremental Morse encoder.
 *
 * Pass a non-null string to start encoding a new message. Thereafter, call the
 * function once per Morse element interval with `s == NULL` to advance the
 * internal state machine; each call returns whether the CW carrier should be on
 * for that element.
 *
 * @param s Message to load, or NULL to advance the current transmission.
 * @param repeating Optional in/out flag controlling whether the message repeats.
 * @param finished Optional output flag set when the current message has finished.
 * @param caller Identifier for the subsystem using the Morse generator.
 * @return true when the carrier should be keyed during the current element.
 */
bool makeMorse(char* s, bool* repeating, bool* finished, callerID_t caller);

/**
 * Report which subsystem most recently claimed the Morse generator.
 *
 * @return The last caller identifier passed to `makeMorse()`.
 */
callerID_t lastMorseCaller(void);

/**
 * Estimate how long a message will take to send at a given code speed.
 *
 * @param str Null-terminated text to evaluate.
 * @param spd Code speed in words per minute.
 * @return Estimated transmit time in milliseconds.
 */
uint16_t timeRequiredToSendStrAtWPM(char* str, uint16_t spd);

#endif /* MORSE_H_ */
