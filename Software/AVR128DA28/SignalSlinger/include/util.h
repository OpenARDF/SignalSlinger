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

/***********************************************************************************************
 *  Print Formatting Utility Functions
 ************************************************************************************************/

/**
 * Convert a time string of the form "yyyy-mm-ddThh:mm:ss" to seconds since 1900.
 *
 * @param s Pointer to a null-terminated string containing the timestamp.
 * @return Seconds elapsed since 1 January 1900.
 */
uint32_t convertTimeStringToEpoch(char * s);

/**
 * Parse a timestamp string formatted as "yyyy-mm-ddThh:mm:ssZ" into a tm structure.
 *
 * @param s   Pointer to the input string (with optional trailing 'Z').
 * @param ltm Pointer to a tm structure to populate.
 * @return true on error, false on success.
 */
bool mystrptime(char* s, struct tm* ltm);

/**
 * Format a frequency value in Hz as a human-readable string.
 *
 * @param result Buffer to receive the formatted string.
 * @param freq   Frequency in Hz to format.
 * @return true on failure, false on success.
 */
bool frequencyString(char* result, uint32_t freq);

/**
 * Parse a textual frequency representation into a Frequency_Hz value.
 *
 * @param str    String containing the frequency (may include units).
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