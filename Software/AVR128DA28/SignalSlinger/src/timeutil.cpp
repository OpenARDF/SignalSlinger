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
 * Time parsing and formatting helpers shared across the firmware.
 *
 * This module contains support functions for:
 * - converting between firmware timestamps, epoch values, and `tm` fields
 * - normalizing partially specified date/time strings
 * - formatting times for display and command handling
 *
 * RTC access and system-time ownership belong elsewhere.
 */

#include "util.h"
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef DATE_STRING_SUPPORT_ENABLED

static uint8_t char2bcd(char c[]);
static uint8_t bcd2dec(uint8_t val);
static time_t epoch_from_ltm(tm *ltm);

/**
 * Parse an ISO-like timestamp string into a `tm` structure.
 *
 * The accepted form is "yyyy-mm-ddThh:mm:ss" with an optional trailing `Z`.
 * A shortened "yyyy-mm-ddThh:mm" form is also accepted. The function fills the
 * `tm` fields needed by the rest of this module, including `tm_yday`.
 *
 * @param s Pointer to the input string to parse.
 * @param ltm Destination `tm` structure.
 * @return true on parse or range error, false on success.
 */
bool mystrptime(char *s, struct tm *ltm)
{
	const int month_days[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
	char temp[6];
	char str[21];
	int hold;
	bool isleap;
	char *ptr0;
	char *ptr1;
	bool noSeconds = false;

	/* Work on a bounded local copy because the parser inserts temporary terminators. */
	strncpy(str, s, 21); // "yyyy-mm-ddThh:mm:ssZ\0" <- maximum length null-terminated string

	ptr1 = strchr(str, 'Z');
	if(ptr1)
		*ptr1 = '\0'; // erase the 'Z' character at the end if one exists

	noSeconds = (strlen(str) < 17);

	ptr1 = strchr(str, '-');
	*ptr1 = '\0';
	strncpy(temp, str, 5);
	++ptr1;
	hold = atoi(temp);
	isleap = is_leap_year(hold);
	hold -= 1900;
	if((hold < 0) || (hold > 200))
		return true;
	ltm->tm_year = hold;

	ptr0 = ptr1;

	ptr1 = strchr(ptr0, '-');
	*ptr1 = '\0';
	strncpy(temp, ptr0, 3);
	++ptr1;
	hold = atoi(temp) - 1;
	if((hold > 11) || (hold < 0))
		return true;
	ltm->tm_mon = hold;

	ptr0 = ptr1;

	ptr1 = strchr(ptr0, 'T');
	*ptr1 = '\0';
	strncpy(temp, ptr0, 3);
	++ptr1;
	hold = atoi(temp);
	if((hold > 31) || (hold < 1))
		return true;
	ltm->tm_mday = hold;

	ptr0 = ptr1;

	ltm->tm_yday = 0;
	for(int i = 0; i < ltm->tm_mon; i++)
	{
		ltm->tm_yday += month_days[i];
		if((i == 1) && isleap)
			ltm->tm_yday++;
	}

	ltm->tm_yday += (ltm->tm_mday - 1);

	ptr1 = strchr(ptr0, ':');
	*ptr1 = '\0';
	strncpy(temp, ptr0, 3);
	++ptr1;
	hold = atoi(temp);
	if((hold > 23) || (hold < 0))
		return true;
	ltm->tm_hour = hold;

	ptr0 = ptr1;

	if(noSeconds)
	{
		strncpy(temp, ptr0, 3);
		hold = atoi(temp);
		if(hold > 59)
			return true;
		ltm->tm_min = hold;
	}
	else
	{
		ptr1 = strchr(ptr0, ':');
		*ptr1 = '\0';
		strncpy(temp, ptr0, 3);
		++ptr1;
		hold = atoi(temp);
		if(hold > 59)
			return true;
		ltm->tm_min = hold;

		hold = atoi(ptr1);
		if(hold > 59)
			return true;
		ltm->tm_sec = hold;
	}

	return false;
}

/**
 * Format an epoch value as "ddd dd-mon-yyyy hh:mm:ss".
 *
 * This module stores times using a historical 1900-based convention in some
 * paths. Values at or above the 30-year offset are adjusted before calling the
 * C library so the resulting text matches the firmware's expected calendar date.
 *
 * @param epoch Epoch value to format.
 * @param buf Destination buffer for the formatted string.
 * @param size Capacity of `buf` in bytes.
 * @return Pointer to `buf`.
 */
#define THIRTY_YEARS 946684800
char *convertEpochToTimeString(time_t epoch, char *buf, size_t size)
{
	struct tm ts;
	time_t t = epoch;

	if(epoch >= THIRTY_YEARS)
	{
		t = epoch - THIRTY_YEARS;
	}

	/* Format local time without a timezone suffix because command paths do not use one. */
	ts = *localtime(&t);
	strftime(buf, size, "%a %d-%b-%Y %H:%M:%S", &ts);
	return buf;
}

/**
 * Convert a three-letter month abbreviation into a 1-based month number.
 *
 * @param mon Month abbreviation such as "Jan".
 * @return Month number in the range 1..12, or 0 if the text is unrecognized.
 */
static int parseMonth(const char *mon)
{
	static const char *months[] =
	    {
	        "Jan", "Feb", "Mar", "Apr", "May", "Jun",
	        "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};

	for(int i = 0; i < 12; i++)
		if(months[i][0] == mon[0] && months[i][1] == mon[1] && months[i][2] == mon[2])
			return i + 1;

	return 0;
}

/**
 * Extract compact numeric date/time fields from `convertEpochToTimeString()` output.
 *
 * The expected input format is "ddd dd-mon-yyyy hh:mm:ss".
 *
 * @param tbuf Formatted time string to decode.
 * @param yy Destination for the two-digit year.
 * @param mon Destination for the 1-based month.
 * @param dd Destination for the day of month.
 * @param hh Destination for hours.
 * @param mm Destination for minutes.
 * @param ss Destination for seconds.
 */
static void parseTimeFields(const char *tbuf,
                            int *yy, int *mon, int *dd,
                            int *hh, int *mm, int *ss)
{
	*dd = (tbuf[4] - '0') * 10 + (tbuf[5] - '0');

	char monStr[4];
	monStr[0] = tbuf[7];
	monStr[1] = tbuf[8];
	monStr[2] = tbuf[9];
	monStr[3] = 0;
	*mon = parseMonth(monStr);

	int yyyy =
	    (tbuf[11] - '0') * 1000 + (tbuf[12] - '0') * 100 +
	    (tbuf[13] - '0') * 10 + (tbuf[14] - '0');

	*yy = yyyy % 100;

	*hh = (tbuf[16] - '0') * 10 + (tbuf[17] - '0');
	*mm = (tbuf[19] - '0') * 10 + (tbuf[20] - '0');
	*ss = (tbuf[22] - '0') * 10 + (tbuf[23] - '0');
}

/**
 * Write a compact timestamp in firmware "YYMMDDhhmmss" form.
 *
 * @param buf Destination buffer with room for 13 bytes.
 * @param yy Two-digit year.
 * @param mon 1-based month.
 * @param dd Day of month.
 * @param hh Hour.
 * @param mm Minute.
 * @param ss Second.
 */
static void formatTimestamp(char *buf,
                            int yy, int mon, int dd,
                            int hh, int mm, int ss)
{
	snprintf(buf, 13, "%02d%02d%02d%02d%02d%02d",
	         yy, mon, dd, hh, mm, ss);
}

/**
 * Complete or offset a timestamp and return normalized "YYMMDDhhmmss" text.
 *
 * Depending on the input, the function can:
 * - accept an already complete 12-digit compact timestamp
 * - expand shorter digit-only partial timestamps using the current local date/time
 * - apply signed time offsets such as "+0530", "+1:30", "+5h", or "-2d"
 *
 * The return value points to a static internal buffer, so each call overwrites
 * the result of the previous one.
 *
 * @param partialString Partial compact timestamp or signed offset string.
 * @param currentEpoch Optional current time override used as the reference clock.
 * @return Pointer to a static normalized timestamp buffer, or `null` on error.
 */
char *completeTimeString(const char *partialString, time_t *currentEpoch)
{
	static char buf[13];
	buf[0] = '\0';

	if(!partialString)
		return null;

	/* Use the supplied epoch when valid; otherwise fall back to the local runtime clock. */
	time_t epoch = time(null); // Use clock time by default

	if(currentEpoch)
	{
		if(*currentEpoch > MINIMUM_VALID_EPOCH)
		{
			epoch = *currentEpoch;
		}
	}

	if(epoch < MINIMUM_VALID_EPOCH)
		return null;

	/* Start from the current local calendar components so partial strings can fill them in. */
	char tbuf[40];
	convertEpochToTimeString(epoch, tbuf, sizeof(tbuf));

	int yy, mon, dd, hh, mm, ss;
	parseTimeFields(tbuf, &yy, &mon, &dd, &hh, &mm, &ss);

	size_t n = strlen(partialString);

	if(currentEpoch)
	{
		/* Signed offset inputs are only accepted when the caller provides a reference epoch. */
		if(partialString[0] == '+' || partialString[0] == '-')
		{
			int sign = (partialString[0] == '+') ? 1 : -1;
			const char *p = partialString + 1;
			time_t value = atol(p);
			time_t newEpoch = epoch;
			const char *colon = strchr(p, ':');

			/* Support four-digit offsets like +0130 for one hour and thirty minutes. */
			if(isdigit(p[0]) && isdigit(p[1]) &&
			   isdigit(p[2]) && isdigit(p[3]) && p[4] == '\0')
			{
				int H = (p[0] - '0') * 10 + (p[1] - '0');
				int M = (p[2] - '0') * 10 + (p[3] - '0');

				newEpoch += sign * (H * 3600 + M * 60);
			}
			else if(colon)
			{
				size_t hour_len = (size_t)(colon - p);
				const char *minutes_str = colon + 1;
				size_t minute_len = strlen(minutes_str);
				int H = 0;
				int M = 0;
				bool valid = true;

				if(strchr(minutes_str, ':') || (hour_len > 3) || (minute_len == 0) || (minute_len > 2))
				{
					return null;
				}

				for(size_t i = 0; i < hour_len; i++)
				{
					if(!isdigit((unsigned char)p[i]))
					{
						valid = false;
						break;
					}

					H = (H * 10) + (p[i] - '0');
				}

				for(size_t i = 0; valid && (i < minute_len); i++)
				{
					if(!isdigit((unsigned char)minutes_str[i]))
					{
						valid = false;
						break;
					}

					M = (M * 10) + (minutes_str[i] - '0');
				}

				if(!valid || (M > 59))
				{
					return null;
				}

				newEpoch += sign * (H * 3600 + M * 60);
			}
			else
			{
				/* Support compact unit-suffixed offsets such as +5h, -30m, and +2d. */
				while(*p && isdigit(*p))
					p++;

				char unit = *p;

				switch(unit)
				{
					case 'h':
					case 'H': // HOURS
						newEpoch += sign * (value * 3600);
						break;

					case 'm':
					case 'M': // MINUTES (new rule)
						newEpoch += sign * (value * 60);
						break;

					case 'd':
					case 'D': // DAYS
						newEpoch += sign * (value * 86400);
						break;

					default:
						/* Unknown units intentionally leave the epoch unchanged. */
						break;
				}
			}

			/* Convert the adjusted epoch back to compact text using the same formatter path. */
			convertEpochToTimeString(newEpoch, tbuf, sizeof(tbuf));
			parseTimeFields(tbuf, &yy, &mon, &dd, &hh, &mm, &ss);

			/* Offset-based commands normalize seconds to zero. */
			formatTimestamp(buf, yy, mon, dd, hh, mm, 0);

			return buf;
		}
	}

	if(!only_digits((char *)partialString))
	{
		return null;
	}

	int AB = (partialString[0] - '0') * 10 + (partialString[1] - '0');
	int CD = (partialString[2] - '0') * 10 + (partialString[3] - '0');
	int EF = (partialString[4] - '0') * 10 + (partialString[5] - '0');
	int GH = (partialString[6] - '0') * 10 + (partialString[7] - '0');
	int IJ = (partialString[8] - '0') * 10 + (partialString[9] - '0');
	int KL = (partialString[10] - '0') * 10 + (partialString[11] - '0');

	/* A fully specified compact timestamp can pass through after basic field validation. */
	if(n == 12)
	{
		if(((AB >= 25) && (AB <= 99)) && ((CD >= 1) && (CD <= 12)) && ((EF >= 1) && (EF <= 31)) && (GH < 24) && (IJ < 60) && (KL < 60)) // A valid year (until 2100) and month
		{
			memcpy(buf, partialString, 12);
			buf[12] = '\0';
			return buf;
		}

		return null;
	}

	/* Six-digit partial timestamps are disambiguated between hhmmss and ddhhmm. */
	if(n == 6)
	{
		if(EF > 59)
			return null;

		/* Favor hhmmss when the leading hour has not yet passed today. */
		if(AB >= hh)
		{
			if((AB < 24) && (CD < 60))
			{
				formatTimestamp(buf, yy, mon, dd, AB, CD, EF);
				return buf;
			}
		}

		if(AB >= dd) // If greater than or equal to today, assume ddhhmm
		{
			if(((AB >= 1) && (AB <= 31)) && (CD < 24))
			{
				formatTimestamp(buf, yy, mon, AB, CD, EF, 0);
				return buf;
			}
		}

		return null; // Otherwise, it is an error
	}

	/* Eight-digit partial timestamps prefer ddhhmmss over mmddhhmm when both fit. */
	if(n == 8)
	{
		if(GH > 59)
			return null;

		bool matchMonth = (AB != mon);
		bool matchDay = (AB >= dd);

		/* If both interpretations fit, treat the first pair as day-of-month. */
		if(matchDay)
		{
			if(((AB >= 1) && (AB <= 31)) && (CD < 24) && (EF < 60))
			{
				formatTimestamp(buf, yy, mon, AB, CD, EF, GH);
				return buf;
			}
		}

		if(matchMonth) // assume mmddhhmm
		{
			if(((AB >= 1) && (AB <= 12)) && ((CD >= 1) && (CD <= 31)) && (EF < 24))
			{
				formatTimestamp(buf, yy, AB, CD, EF, GH, 0);
				return buf;
			}
		}

		return null;
	}

	/* Ten-digit partial timestamps choose between yymmddhhmm and mmddhhmmss. */
	if(n == 10)
	{
		if(IJ > 59) // If invalid minutes or seconds
		{
			return null;
		}

		if(((AB >= 25) && (AB <= 99)) && ((CD >= 1) && (CD <= 12)) && ((EF >= 1) && (EF <= 31)) && (GH < 24)) // A valid year (until 2100) and month
		{
			formatTimestamp(buf,
			                AB,
			                CD,
			                EF,
			                GH,
			                IJ,
			                0);
			return buf;
		}
		else if(((AB >= 1) && (AB <= 12)) && ((CD >= 1) && (CD <= 31)) && (EF < 24) && (GH < 60)) // A valid month
		{
			formatTimestamp(buf,
			                yy,
			                AB,
			                CD,
			                EF,
			                GH,
			                IJ);
			return buf;
		}

		return null;
	}

	return null;
}

/**
 * Convert a compact "YYMMDDhhmmss" timestamp string to an epoch value.
 *
 * @param error Optional output flag set to true when conversion fails.
 * @param datetime Timestamp string in compact firmware format.
 * @return Parsed epoch value, or 0 on failure.
 */
time_t String2Epoch(bool *error, char *datetime)
{
	time_t epoch = 0;
	uint8_t data[7] = {0, 0, 0, 0, 0, 0, 0};

	struct tm ltm = {0, 0, 0, 0, 0, 0, 0, 0, 0};
	int16_t year = 100; /* start at 100 years past 1900 */
	uint8_t month;
	uint8_t date;
	uint8_t hours;
	uint8_t minutes;
	uint8_t seconds;

	if(datetime) /* String format "YYMMDDhhmmss" */
	{
		/* Convert each two-digit field through BCD because the RTC-facing utilities use that form. */
		data[0] = char2bcd(&datetime[10]); /* seconds in BCD */
		data[1] = char2bcd(&datetime[8]);  /* minutes in BCD */
		data[2] = char2bcd(&datetime[6]);  /* hours in BCD */
		/* data[3] =  not used */
		data[4] = char2bcd(&datetime[4]); /* day of month in BCD */
		data[5] = char2bcd(&datetime[2]); /* month in BCD */
		data[6] = char2bcd(&datetime[0]); /* 2-digit year in BCD */

		hours = bcd2dec(data[2]); /* Must be calculated here */

		year += (int16_t)bcd2dec(data[6]);
		ltm.tm_year = year; /* year since 1900 */

		year += 1900; /* adjust year to calendar year */

		month = bcd2dec(data[5]);
		ltm.tm_mon = month - 1; /* mon 0 to 11 */

		date = bcd2dec(data[4]);
		ltm.tm_mday = date; /* month day 1 to 31 */

		ltm.tm_yday = 0;
		for(uint8_t mon = 1; mon < month; mon++) /* months from 1 to 11 (excludes partial month) */
		{
			ltm.tm_yday += month_length(year, mon);
			;
		}

		ltm.tm_yday += (ltm.tm_mday - 1);

		seconds = bcd2dec(data[0]);
		minutes = bcd2dec(data[1]);

		ltm.tm_hour = hours;
		ltm.tm_min = minutes;
		ltm.tm_sec = seconds;

		epoch = epoch_from_ltm(&ltm);
	}

	if(error)
	{
		*error = (epoch == 0);
	}

	return (epoch);
}

/**
 * Convert a Binary-Coded Decimal byte to its decimal value.
 *
 * @param val Input BCD value.
 * @return Decimal value represented by `val`.
 */
static uint8_t bcd2dec(uint8_t val)
{
	uint8_t result = 10 * (val >> 4) + (val & 0x0F);
	return (result);
}

/**
 * Convert two ASCII digits to a packed Binary-Coded Decimal byte.
 *
 * @param c Pointer to the first of two digit characters.
 * @return Packed BCD value.
 */
static uint8_t char2bcd(char c[])
{
	uint8_t result = (c[1] - '0') + ((c[0] - '0') << 4);
	return (result);
}

/**
 * Convert a populated `tm` structure to an epoch value.
 *
 * @param ltm Pointer to the `tm` structure to convert.
 * @return Epoch value represented by `ltm`.
 */
static time_t epoch_from_ltm(tm *ltm)
{
	time_t epoch = ltm->tm_sec + ltm->tm_min * 60 + ltm->tm_hour * 3600L + ltm->tm_yday * 86400L +
	               (ltm->tm_year - 70) * 31536000L + ((ltm->tm_year - 69) / 4) * 86400L -
	               ((ltm->tm_year - 1) / 100) * 86400L + ((ltm->tm_year + 299) / 400) * 86400L;

	return (epoch);
}

/**
 * Convert an ISO-like timestamp string to the module's epoch representation.
 *
 * The input is parsed using `mystrptime()`, which accepts
 * "yyyy-mm-ddThh:mm:ss" and the same form with a trailing `Z`.
 *
 * @param s Pointer to a null-terminated string containing the timestamp.
 * @return Parsed epoch value, or 0 if the string cannot be parsed.
 */
uint32_t convertTimeStringToEpoch(char *s)
{
	unsigned long result = 0;
	struct tm ltm = {0};

	if(!mystrptime(s, &ltm))
	{
		result = ltm.tm_sec + ltm.tm_min * 60 + ltm.tm_hour * 3600L + ltm.tm_yday * 86400L +
		         (ltm.tm_year - 70) * 31536000L + ((ltm.tm_year - 69) / 4) * 86400L -
		         ((ltm.tm_year - 1) / 100) * 86400L + ((ltm.tm_year + 299) / 400) * 86400L;
	}

	return result;
}

#endif //  DATE_STRING_SUPPORT_ENABLED
