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
 * timeutil.h
 *
 * Time parsing and formatting helpers shared across the firmware.
 *
 */

#ifndef TIMEUTIL_H_
#define TIMEUTIL_H_

#include "defs.h"
#include <time.h>

/**
 * Convert a time string of the form "yyyy-mm-ddThh:mm:ss" to seconds since 1900.
 *
 * @param s Pointer to a null-terminated string containing the timestamp.
 * @return Seconds elapsed since 1 January 1900.
 */
uint32_t convertTimeStringToEpoch(char *s);

char *convertEpochToTimeString(time_t epoch, char *buf, size_t size);

time_t String2Epoch(bool *error, char *datetime);

/**
 * @brief Completes or offsets a timestamp and returns a 12-character time string.
 *
 * This function accepts either:
 *  - A partial timestamp string (up to 10 characters) representing YYMMDDhhmm,
 *    in which case missing leading fields are filled using the current local time.
 *
 *  - An offset string beginning with '+', such as "+5h", "+0500", "+2d", "+7m",
 *    "+1M", or "+1y". These are interpreted as offsets from the current local
 *    time in hours, minutes, days, months, or years. Multi-digit "+HHMM" format
 *    (e.g., "+0530") is interpreted as +5 hours 30 minutes.
 *
 * The returned timestamp always consists of 12 characters in the form:
 *
 *        YYMMDDhhmmSS
 *
 * with seconds (SS) always set to "00". The caller must supply `buf`, which
 * must be large enough to hold at least 13 bytes (12 characters + null terminator).
 *
 * @param partialString  A partial YYMMDDhhmm string, or an offset string.
 *
 * @return A pointer to the completed 12-character timestamp stored in `buf`.
 */
char *completeTimeString(const char *partialString, time_t *currentEpoch);

/**
 * Parse a timestamp string formatted as "yyyy-mm-ddThh:mm:ssZ" into a tm structure.
 *
 * @param s   Pointer to the input string (with optional trailing 'Z').
 * @param ltm Pointer to a tm structure to populate.
 * @return true on error, false on success.
 */
bool mystrptime(char *s, struct tm *ltm);

#endif  /* TIMEUTIL_H_ */
