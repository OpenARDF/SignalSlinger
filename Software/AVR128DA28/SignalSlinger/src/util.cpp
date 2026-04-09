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

static bool lookupEnumText(char *str, uint8_t index, uint8_t count, const char *const *table)
{
	if((index >= count) || !table[index])
	{
		return true;
	}

	strcpy(str, table[index]);
	return false;
}

static const char *const g_function_text[] =
    {
        NULL,
        "QRP Transmitter",
        "Radio Orienteering",
        "Signal Generator"};

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

static const char *const g_event_text[EVENT_NUMBER_OF_EVENTS] =
    {
        "None Set",
        "Classic",
        "Sprint",
        "Foxoring",
        "Blind ARDF"};

/**
 * Returns a-b
 * It appears difftime might not be handling subtraction of unsigned arguments correctly with current compiler. This function avoids any problems.
 */
int32_t timeDif(time_t a, time_t b)
{
	int32_t dif; // = difftime(now, g_event_start_epoch); // returns arg1 - arg2
	if(a > b)
		dif = a - b;
	else
		dif = -(b - a);

	return dif;
}

/**
 * Checks a string to see if it contains only numerical characters
 */
bool only_digits(char *s)
{
	while(*s)
	{
		if(isdigit(*s++) == 0)
		{
			return (false);
		}
	}

	return (true);
}

/**
 * Convert a frequency string to a proper Hz value and string format based on assumptions
 * related to the size and decimal properties of the number contained in the string.
 * result = pointer to a character sting to hold the frequency string
 * freq = the frequency value to be represented as a string
 * Returns 1 if an error is detected
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
 * Translate a Function_t enum value into a descriptive string.
 *
 * @param str Buffer to receive the textual description.
 * @param fun Enumerated function value to describe.
 * @return true if the function is unrecognized, false otherwise.
 */
bool function2Text(char *str, Function_t fun)
{
	return lookupEnumText(str, (uint8_t)fun, (uint8_t)(sizeof(g_function_text) / sizeof(g_function_text[0])), g_function_text);
}

/**
 * Translate a Fox_t enum value into a descriptive string.
 *
 * @param str Buffer to receive the textual description.
 * @param fox Enumerated fox value to describe.
 * @return true if the fox value is unrecognized, false otherwise.
 */
bool fox2Text(char *str, Fox_t fox)
{
	return lookupEnumText(str, (uint8_t)fox, USE_CURRENT_FOX, g_fox_text);
}

/**
 * Translate an Event_t enum value into a descriptive string.
 *
 * @param str Buffer to receive the textual description.
 * @param evt Enumerated event value to describe.
 * @return true if the event value is unrecognized, false otherwise.
 */
bool event2Text(char *str, Event_t evt)
{
	return lookupEnumText(str, (uint8_t)evt, EVENT_NUMBER_OF_EVENTS, g_event_text);
}

/*-------------------------------------------------------------
 *  float_to_parts_signed
 *  ---------------------
 *  Split a float (positive or negative) into:
 *      - characteristic : signed integer part      (-32768 to 32767)
 *      - mantissa       : first decimal digit      (0 to 9)
 *
 *  Returns:  false  ? success
 *            true   ? error   (bad args, NaN/Inf, out of range)
 *------------------------------------------------------------*/
bool float_to_parts_signed(float value,
                           int16_t *integerPart,   /* signed  */
                           uint16_t *fractionPart) /* unsigned */
{
	/* pointer validity */
	if(integerPart == NULL || fractionPart == NULL)
		return true;

	/* reject NaN or +/-Inf */
	if(isnanf(value) || isinff(value))
		return true;

	/* split into integer and fractional parts */
	float int_part_f;
	float frac_part_f = modff(value, &int_part_f); /* both carry the sign of value */

	/* range-check integer part for int16_t */
	if(int_part_f < (float)INT16_MIN || int_part_f > (float)INT16_MAX)
		return true;

	/* scale fractional part to 1 decimal place, keep it non-negative */
	float scaled = roundf(fabsf(frac_part_f) * 10.0f);
	if(scaled > 9.)
		scaled = 9.; /* avoid potential "10" for the fractional part */

	/* commit results */
	*integerPart = (int16_t)int_part_f; /* may be negative */
	*fractionPart = (uint16_t)scaled;   /* always positive */

	return false; /* success */
}
