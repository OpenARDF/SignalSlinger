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
 *
 * util.h
 *
 * Various helper functions with no better home.
 *
 */


#ifndef UTIL_H_
#define UTIL_H_

#include "defs.h"
#include "timeutil.h"
#include <time.h>

/**
 * Calculate the signed difference between two time values.
 *
 * @param a First time value in seconds.
 * @param b Second time value in seconds.
 * @return The result of a - b, accounting for unsigned types.
 */
int32_t timeDif(time_t a, time_t b);

/**
 * Determine whether a string is composed solely of digit characters.
 *
 * @param s Pointer to a null-terminated string to inspect.
 * @return true if every character in the string is a numeric digit; otherwise false.
 */
bool only_digits(char *s);

/**
 * Format a frequency value in Hz as a human-readable string.
 *
 * @param result Buffer to receive the formatted string.
 * @param freq   Frequency in Hz to format.
 * @return true on failure, false on success.
 */
bool frequencyString(char* result, uint32_t freq);

/**
 * Parse a numeric frequency string into a Frequency_Hz value and normalized text.
 *
 * The function infers whether the input is expressed in MHz, kHz, or Hz by
 * comparing its magnitude against the transmitter's valid frequency range.
 * On success it also rewrites `str` into normalized "####.# kHz" text.
 *
 * @param str    Mutable string containing the frequency to parse.
 * @param result Pointer to store the parsed frequency in Hz.
 * @return true on failure, false on success.
 */
bool frequencyVal(char* str, Frequency_Hz* result);

/**
 * Convert a fox identifier to descriptive text.
 *
 * @param str Buffer to receive the description.
 * @param fox Enum value identifying the fox type.
 * @return true on failure, false on success.
 */
bool fox2Text(char* str, Fox_t fox);

/**
 * Convert an event identifier to descriptive text.
 *
 * @param str Buffer to receive the description.
 * @param evt Enum value identifying the event type.
 * @return true on failure, false on success.
 */
bool event2Text(char* str, Event_t evt);

/**
 * Convert a functionality identifier to descriptive text.
 *
 * @param str Buffer to receive the description.
 * @param fun Enum value identifying the desired function.
 * @return true on failure, false on success.
 */
bool function2Text(char* str, Function_t fun);

/**
 * Split a floating-point value into integer and fractional components.
 *
 * @param value        Input floating-point value to split.
 * @param integerPart  Output pointer for the signed integer portion.
 * @param fractionPart Output pointer for the first decimal digit (0-9).
 * @return true on error, false on success.
 */
bool float_to_parts_signed(float value,
                           int16_t  *integerPart,
                           uint16_t *fractionPart);


#endif  /* UTIL_H_ */
