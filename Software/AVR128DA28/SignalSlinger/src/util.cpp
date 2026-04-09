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
 * Utility helpers shared across the firmware.
 *
 * This module contains small, stateless support functions for:
 * - time and numeric formatting/parsing
 * - enum-to-text conversion
 * - compact display-oriented value splitting
 *
 * Hardware control, ISR coordination, and event-engine logic belong elsewhere.
 */

#include "util.h"
#include "transmitter.h"
#include <avr/eeprom.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <stdint-gcc.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __INT16_MAX__
#undef INT16_MAX
#define INT16_MAX __INT16_MAX__
#undef INT16_MIN
#define INT16_MIN (-INT16_MAX - 1)
#endif

/**
 * Copy a textual description from a lookup table when an enum value is valid.
 *
 * The helper centralizes the "index in range and entry is non-null" check used
 * by the public enum-to-text wrappers below. A true return value means the
 * caller supplied an unsupported enum value or the table intentionally leaves
 * that slot unmapped.
 *
 * @param str Destination buffer for the selected text.
 * @param index Table index derived from the enum value.
 * @param count Number of entries available in the table.
 * @param table Lookup table containing string pointers.
 * @return true if the lookup cannot be satisfied, false on success.
 */
static bool lookupEnumText(char *str, uint8_t index, uint8_t count, const char *const *table)
{
	if((index >= count) || !table[index])
	{
		return true;
	}

	/* The wrappers guarantee that the destination buffer is large enough for these literals. */
	strcpy(str, table[index]);
	return false;
}

/* Human-readable names for Function_t values.
 * Index 0 is intentionally unmapped because the corresponding enum value is not
 * presented as a selectable user-facing mode.
 */
static const char *const g_function_text[] =
    {
        NULL,
        "QRP Transmitter",
        "Radio Orienteering",
        "Signal Generator"};

/* Human-readable names for Fox_t values in enum order. */
static const char *const g_fox_text[USE_CURRENT_FOX] =
    {
        "Beacon \"MO\"",
        "Classic Fox 1 \"MOE\"",
        "Classic Fox 2 \"MOI\"",
        "Classic Fox 3 \"MOS\"",
        "Classic Fox 4 \"MOH\"",
        "Classic Fox 5 \"MO5\"",
        "Spectator \"S\"",
        "Sprint Slow 1 \"ME\"",
        "Sprint Slow 2 \"MI\"",
        "Sprint Slow 3 \"MS\"",
        "Sprint Slow 4 \"MH\"",
        "Sprint Slow 5 \"M5\"",
        "Sprint Fast 1 \"OE\"",
        "Sprint Fast 2 \"OI\"",
        "Sprint Fast 3 \"OS\"",
        "Sprint Fast 4 \"OH\"",
        "Sprint Fast 5 \"O5\"",
        "Foxoring \"Low Freq\" Fox",
        "Foxoring \"Medium Freq\" Fox",
        "Foxoring \"High Freq\" Fox",
        "Frequency Test Beacon"};

/* Human-readable names for Event_t values in enum order. */
static const char *const g_event_text[EVENT_NUMBER_OF_EVENTS] =
    {
        "None Set",
        "Classic",
        "Sprint",
        "Foxoring",
        "Blind ARDF"};

/**
 * Return the signed difference between two time values without using `difftime()`.
 *
 * This firmware uses `time_t` in environments where the toolchain's `difftime()`
 * behavior has been questionable for unsigned representations. This helper keeps
 * the intent explicit by always computing `a - b` and returning a signed result.
 *
 * @param a Minuend time value.
 * @param b Subtrahend time value.
 * @return Signed difference in seconds, preserving the direction of the delta.
 */
int32_t timeDif(time_t a, time_t b)
{
	int32_t dif;

	/* Split the cases so the subtraction stays well-defined even if time_t is unsigned. */
	if(a > b)
		dif = a - b;
	else
		dif = -(b - a);

	return dif;
}

/**
 * Report whether every character in a string is an ASCII digit.
 *
 * The function walks the string until the terminating null byte and returns
 * false as soon as a non-digit is encountered. An empty string returns true,
 * because the loop never finds a character that violates the rule.
 *
 * @param s Pointer to the null-terminated string to inspect.
 * @return true if all characters are digits or the string is empty, false otherwise.
 */
bool only_digits(char *s)
{
	while(*s)
	{
		/* Stop on the first character that is outside '0'..'9'. */
		if(isdigit(*s++) == 0)
		{
			return (false);
		}
	}

	return (true);
}

/**
 * Format a validated frequency in Hz as normalized display text.
 *
 * This helper accepts only frequencies inside the transmitter's supported range
 * and expresses them in "####.# kHz" form. The fractional digit is derived from
 * the hundreds-of-Hz place, so the display stays consistent with the firmware's
 * one-decimal-place UI conventions.
 *
 * @param result Buffer to receive the formatted text.
 * @param freq Frequency in Hz to normalize for display.
 * @return true on null-buffer or out-of-range input, false on success.
 */
bool frequencyString(char *result, uint32_t freq)
{
	bool failure = true;

	if(!result)
	{
		return (failure);
	}

	if((freq >= TX_MINIMUM_FREQUENCY) && (freq <= TX_MAXIMUM_FREQUENCY)) // Accept only a Hz value to be expressed in kHz
	{
		/* Preserve one decimal place by reusing the hundreds-of-Hz digit. */
		uint32_t frac = (freq % 1000) / 100;
		sprintf(result, "%lu.%1lu kHz", freq / 1000, frac);

		failure = false;
	}

	return (failure);
}

/**
 * Parse a numeric frequency string and normalize it to Hz and display text.
 *
 * The input is interpreted by magnitude rather than by explicit units:
 * - values in the transmitter's MHz range are treated as MHz
 * - values in the transmitter's kHz range are treated as kHz
 * - values in the expected raw-Hz range are treated as Hz
 *
 * On success the function performs two outputs:
 * - `result`, when non-null, receives the frequency rounded up to the next Hz
 * - `str` is rewritten in normalized "####.# kHz" form for later display
 *
 * @param str Pointer to the mutable numeric string to parse and normalize.
 * @param result Pointer to a Frequency_Hz destination, or NULL if only normalization is needed.
 * @return true on parse/range failure, false on success.
 */
bool frequencyVal(char *str, Frequency_Hz *result)
{
	bool failure = true;
	float mhzMin, mhzMax, khzMin, khzMax, hzMin, hzMax;

	/* Build comparison ranges once so the same transmitter limits drive all accepted formats. */
	mhzMin = (float)TX_MINIMUM_FREQUENCY / 1000000.;
	mhzMax = (float)TX_MAXIMUM_FREQUENCY / 1000000.;
	khzMin = (float)TX_MINIMUM_FREQUENCY / 1000.;
	khzMax = (float)TX_MAXIMUM_FREQUENCY / 1000.;
	hzMin = 3500000.;
	hzMax = 4000000.;

	if(!str)
	{
		return (failure);
	}

	float f = atof(str);

	/* Infer units from magnitude because command paths provide plain numeric text. */
	if((f > mhzMin) && (f < mhzMax))
	{
		f *= 1000000.;
		failure = false;
	}
	else if((f > khzMin) && (f < khzMax))
	{
		f *= 1000.;
		failure = false;
	}
	else if((f > hzMin) && (f < hzMax))
	{
		failure = false;
	}

	if(!failure)
	{
		/* Store an integer-Hz value, then rewrite the original buffer in a consistent display form. */
		Frequency_Hz temp = (Frequency_Hz)ceilf(f);
		if(result)
			*result = temp;
		sprintf(str, "%4.1f kHz", (double)f);
	}

	return (failure);
}

/**
 * Translate a Function_t enum value into the corresponding user-facing label.
 *
 * @param str Buffer to receive the textual description.
 * @param fun Enumerated function value to describe.
 * @return true if the function value is not mapped to display text, false otherwise.
 */
bool function2Text(char *str, Function_t fun)
{
	return lookupEnumText(str, (uint8_t)fun, (uint8_t)(sizeof(g_function_text) / sizeof(g_function_text[0])), g_function_text);
}

/**
 * Translate a Fox_t enum value into the corresponding user-facing label.
 *
 * @param str Buffer to receive the textual description.
 * @param fox Enumerated fox value to describe.
 * @return true if the fox value is not mapped to display text, false otherwise.
 */
bool fox2Text(char *str, Fox_t fox)
{
	return lookupEnumText(str, (uint8_t)fox, USE_CURRENT_FOX, g_fox_text);
}

/**
 * Translate an Event_t enum value into the corresponding user-facing label.
 *
 * @param str Buffer to receive the textual description.
 * @param evt Enumerated event value to describe.
 * @return true if the event value is not mapped to display text, false otherwise.
 */
bool event2Text(char *str, Event_t evt)
{
	return lookupEnumText(str, (uint8_t)evt, EVENT_NUMBER_OF_EVENTS, g_event_text);
}

/**
 * Split a floating-point value into a signed integer part and one decimal digit.
 *
 * The integer portion keeps the original sign, while the fractional output is
 * always returned as a non-negative single decimal digit suitable for compact
 * display formatting. Values that are NaN, infinite, or outside the `int16_t`
 * range are rejected.
 *
 * @param value Input value to split.
 * @param integerPart Destination for the signed integer portion.
 * @param fractionPart Destination for the first decimal digit, in the range 0..9.
 * @return true on invalid pointers or unrepresentable input, false on success.
 */
bool float_to_parts_signed(float value,
                           int16_t *integerPart,
                           uint16_t *fractionPart)
{
	/* Both output pointers are required because success always produces a full split result. */
	if(integerPart == NULL || fractionPart == NULL)
		return true;

	/* Special floating-point values cannot be represented meaningfully in this format. */
	if(isnanf(value) || isinff(value))
		return true;

	/* modff preserves the sign on both parts, which lets negative inputs round-trip cleanly. */
	float int_part_f;
	float frac_part_f = modff(value, &int_part_f);

	/* Reject values whose whole-number portion would overflow the signed destination type. */
	if(int_part_f < (float)INT16_MIN || int_part_f > (float)INT16_MAX)
		return true;

	/* Convert the fractional portion to one decimal digit for compact text output. */
	float scaled = roundf(fabsf(frac_part_f) * 10.0f);
	if(scaled > 9.)
		scaled = 9.; /* Clamp rare round-up cases so the fractional field stays one digit. */

	*integerPart = (int16_t)int_part_f;
	*fractionPart = (uint16_t)scaled;

	return false;
}
