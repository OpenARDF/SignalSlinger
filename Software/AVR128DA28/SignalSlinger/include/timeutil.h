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
 *
 */

/*
 * Time parsing and formatting helpers shared across the firmware.
 *
 * This module contains support functions for:
 * - converting between firmware timestamps, epoch values, and `tm` fields
 * - normalizing partially specified date/time strings
 * - formatting times for display and command handling
 *
 * RTC access and system-time ownership belong elsewhere.
 */

#ifndef TIMEUTIL_H_
#define TIMEUTIL_H_

#include "defs.h"
#include <time.h>

/**
 * Convert an ISO-like timestamp string to the module's epoch representation.
 *
 * The input is parsed using `mystrptime()`, which accepts
 * "yyyy-mm-ddThh:mm:ss" and the same form with a trailing `Z`.
 *
 * @param s Pointer to a null-terminated string containing the timestamp.
 * @return Parsed epoch value, or 0 if the string cannot be parsed.
 */
uint32_t convertTimeStringToEpoch(char *s);

/**
 * Format an epoch value as "ddd dd-mon-yyyy hh:mm:ss".
 *
 * The helper preserves the historical epoch convention used by this firmware,
 * including the adjustment applied to values stored relative to 1900.
 *
 * @param epoch Epoch value to format.
 * @param buf Destination buffer for the formatted string.
 * @param size Capacity of `buf` in bytes.
 * @return Pointer to `buf`.
 */
char *convertEpochToTimeString(time_t epoch, char *buf, size_t size);

/**
 * Convert a compact "YYMMDDhhmmss" timestamp string to an epoch value.
 *
 * @param error Optional output flag set to true when conversion fails.
 * @param datetime Timestamp string in compact firmware format.
 * @return Parsed epoch value, or 0 on failure.
 */
time_t String2Epoch(bool *error, char *datetime);

/**
 * Complete or offset a timestamp and return normalized "YYMMDDhhmmss" text.
 *
 * Depending on the input, the function can:
 * - accept an already complete 12-digit compact timestamp
 * - expand shorter digit-only partial timestamps using the current local date/time
 * - apply signed time offsets such as "+0530", "+1:30", "+5h", or "-2d"
 *
 * @param partialString Partial compact timestamp or signed offset string.
 * @param currentEpoch Optional current time override used as the reference clock.
 * @return Pointer to a static normalized timestamp buffer, or `null` on error.
 */
char *completeTimeString(const char *partialString, time_t *currentEpoch);

/**
 * Parse an ISO-like timestamp string into a `tm` structure.
 *
 * The accepted form is "yyyy-mm-ddThh:mm:ss" with an optional trailing `Z`.
 * A shortened "yyyy-mm-ddThh:mm" form is also accepted.
 *
 * @param s   Pointer to the input string (with optional trailing 'Z').
 * @param ltm Pointer to a tm structure to populate.
 * @return true on error, false on success.
 */
bool mystrptime(char *s, struct tm *ltm);

#endif  /* TIMEUTIL_H_ */
