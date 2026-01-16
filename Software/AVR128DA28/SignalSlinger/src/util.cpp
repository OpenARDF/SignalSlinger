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
# undef INT16_MAX
# define INT16_MAX __INT16_MAX__
# undef INT16_MIN
# define INT16_MIN (-INT16_MAX - 1)
#endif

uint8_t char2bcd(char c[]);
uint8_t bcd2dec(uint8_t val);
time_t epoch_from_ltm(tm *ltm);

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


/***********************************************************************************************
 *  Print Formatting Utility Functions
 ************************************************************************************************/

#ifdef DATE_STRING_SUPPORT_ENABLED

/**
 * Returns parsed time structure from a string of format "yyyy-mm-ddThh:mm:ssZ"
 */
bool mystrptime(char* s, struct tm* ltm) {
  const int month_days[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  char temp[6];
  char str[21];
  int hold;
  bool isleap;
  char *ptr0;
  char *ptr1;
  bool noSeconds = false;

  strncpy(str, s, 21);  // "yyyy-mm-ddThh:mm:ssZ\0" <- maximum length null-terminated string

  ptr1 = strchr(str, 'Z');
  if(ptr1) *ptr1 = '\0'; // erase the 'Z' character at the end if one exists

  noSeconds = (strlen(str) < 17);

  ptr1 = strchr(str, '-');
  *ptr1 = '\0';
  strncpy(temp, str, 5);
  ++ptr1;
  hold = atoi(temp);
  isleap = is_leap_year(hold);
  hold -= 1900;
  if((hold < 0) || (hold > 200)) return true;
  ltm->tm_year = hold;

  ptr0 = ptr1;

  ptr1 = strchr(ptr0, '-');
  *ptr1 = '\0';
  strncpy(temp, ptr0, 3);
  ++ptr1;
  hold = atoi(temp) - 1;
  if((hold > 11) || (hold < 0)) return true;
  ltm->tm_mon = hold;

  ptr0 = ptr1;

  ptr1 = strchr(ptr0, 'T');
  *ptr1 = '\0';
  strncpy(temp, ptr0, 3);
  ++ptr1;
  hold = atoi(temp);
  if((hold > 31) || (hold < 1)) return true;
  ltm->tm_mday = hold;

  ptr0 = ptr1;

  ltm->tm_yday = 0;
  for(int i=0; i<ltm->tm_mon; i++)
  {
    ltm->tm_yday += month_days[i];
    if((i==1) && isleap) ltm->tm_yday++;
  }

  ltm->tm_yday += (ltm->tm_mday - 1);

  ptr1 = strchr(ptr0, ':');
  *ptr1 = '\0';
  strncpy(temp, ptr0, 3);
  ++ptr1;
  hold = atoi(temp);
  if((hold > 23) || (hold < 0)) return true;
  ltm->tm_hour = hold;

  ptr0 = ptr1;

  if(noSeconds)
  {
	  strncpy(temp, ptr0, 3);
	  hold = atoi(temp);
	  if(hold > 59) return true;
	  ltm->tm_min = hold;
  }
  else
  {
	  ptr1 = strchr(ptr0, ':');
	  *ptr1 = '\0';
	  strncpy(temp, ptr0, 3);
	  ++ptr1;
	  hold = atoi(temp);
	  if(hold > 59) return true;
	  ltm->tm_min = hold;

	  hold = atoi(ptr1);
	  if(hold > 59) return true;
	  ltm->tm_sec = hold;
  }

  return false;
}


/*
 * Converts an epoch time (seconds since 1900) to a human-readable string with format "ddd dd-mon-yyyy hh:mm:ss zzz"
 * @param epoch - the epoch time to convert
 * @param buf - a buffer to store the resulting string
 * @param size - size of the buffer
 * @return pointer to the formatted time string
 */
#define THIRTY_YEARS 946684800
char* convertEpochToTimeString(time_t epoch, char* buf, size_t size)
 {
    struct tm  ts;
	time_t t = epoch;
	
	if(epoch >= THIRTY_YEARS)
	{
		t = epoch - THIRTY_YEARS;
	}

    // Format time, "ddd dd-mon-yyyy hh:mm:ss zzz"
    ts = *localtime(&t);
    strftime(buf, size, "%a %d-%b-%Y %H:%M:%S", &ts);
   return buf;
 }

// ==========================
// Helper: month abbreviation
// ==========================
static int parseMonth(const char* mon)
{
	static const char* months[] =
	{
		"Jan","Feb","Mar","Apr","May","Jun",
		"Jul","Aug","Sep","Oct","Nov","Dec"
	};

	for (int i = 0; i < 12; i++)
	if (months[i][0]==mon[0] && months[i][1]==mon[1] && months[i][2]==mon[2])
	return i+1;

	return 0;
}

// ======================================
// Helper: parse convertEpochToTimeString
// ======================================
static void parseTimeFields(const char* tbuf,
int* yy, int* mon, int* dd,
int* hh, int* mm, int* ss)
{
	*dd = (tbuf[4]-'0')*10 + (tbuf[5]-'0');

	char monStr[4];
	monStr[0]=tbuf[7]; monStr[1]=tbuf[8]; monStr[2]=tbuf[9]; monStr[3]=0;
	*mon = parseMonth(monStr);

	int yyyy =
	(tbuf[11]-'0')*1000 + (tbuf[12]-'0')*100 +
	(tbuf[13]-'0')*10   + (tbuf[14]-'0');

	*yy = yyyy % 100;

	*hh = (tbuf[16]-'0')*10 + (tbuf[17]-'0');
	*mm = (tbuf[19]-'0')*10 + (tbuf[20]-'0');
	*ss = (tbuf[22]-'0')*10 + (tbuf[23]-'0');
}

// ========================================
// Helper: produce 12-char YYMMDDhhmmss
// ========================================
static void formatTimestamp(char* buf,
int yy, int mon, int dd,
int hh, int mm, int ss)
{
	snprintf(buf, 13, "%02d%02d%02d%02d%02d%02d",
	yy, mon, dd, hh, mm, ss);
}

// =====================================================
// MAIN FUNCTION — Complete, Refactored, With New Rules
// =====================================================
char* completeTimeString(const char* partialString, time_t* currentEpoch)
{
	static char buf[13];
	buf[0]='\0';

	if (!partialString)
	return null;

	// --- validate clock ---
	time_t epoch = time(null); // Use clock time by default
	
	if(currentEpoch)
	{
		if(*currentEpoch > MINIMUM_VALID_EPOCH)
		{
			epoch = *currentEpoch;
		}
	}
	
	if (epoch < MINIMUM_VALID_EPOCH)
	return null;

	// --- extract now fields ---
	char tbuf[40];
	convertEpochToTimeString(epoch, tbuf, sizeof(tbuf));

	int yy, mon, dd, hh, mm, ss;
	parseTimeFields(tbuf, &yy, &mon, &dd, &hh, &mm, &ss);

	size_t n = strlen(partialString);

	// ============================
	// OFFSET MODE
	// ============================
	if(currentEpoch)
	{
		// ============================
		// OFFSET MODE: + / - (h, m, d)
		// ============================
		if (partialString[0] == '+' || partialString[0] == '-')
		{
			int sign = (partialString[0] == '+') ? 1 : -1;
			const char* p = partialString + 1;
			time_t value = atol(p);
			time_t newEpoch = epoch;

			// ---- Case: ±HHMM  (4 digits) ----
			if (isdigit(p[0]) && isdigit(p[1]) &&
			isdigit(p[2]) && isdigit(p[3]) && p[4] == '\0')
			{
				int H = (p[0]-'0')*10 + (p[1]-'0');
				int M = (p[2]-'0')*10 + (p[3]-'0');

				newEpoch += sign * (H * 3600 + M * 60);
			}
			else
			{
				// ---- Case: ±N(unit) ----
				// Where unit = h/H (hours), m/M (minutes), d/D (days)
				while (*p && isdigit(*p))
				p++;

				char unit = *p;

				switch (unit)
				{
					case 'h': case 'H':     // HOURS
					newEpoch += sign * (value * 3600);
					break;

					case 'm': case 'M':     // MINUTES (new rule)
					newEpoch += sign * (value * 60);
					break;

					case 'd': case 'D':     // DAYS
					newEpoch += sign * (value * 86400);
					break;

					default:
					// Unknown unit ? do nothing
					break;
				}
			}

			// ---- Recompute resulting timestamp ----
			convertEpochToTimeString(newEpoch, tbuf, sizeof(tbuf));
			parseTimeFields(tbuf, &yy, &mon, &dd, &hh, &mm, &ss);

			// Seconds always "00" for offsets
			formatTimestamp(buf, yy, mon, dd, hh, mm, 0);

			return buf;
		}
	}
	
	if(!only_digits((char*)partialString))
	{
		return null;
	}
	
	int AB = (partialString[0]-'0')*10 + (partialString[1]-'0');
	int CD = (partialString[2]-'0')*10 + (partialString[3]-'0');
	int EF = (partialString[4]-'0')*10 + (partialString[5]-'0');
	int GH = (partialString[6]-'0')*10 + (partialString[7]-'0');
	int IJ = (partialString[8]-'0')*10 + (partialString[9]-'0');
	int KL = (partialString[10]-'0')*10 + (partialString[11]-'0');

	// ============================
	// 12 char ? copy as-is
	// ============================
	if (n == 12)
	{
		if(((AB >= 25) && (AB <= 99)) && ((CD >= 1) && (CD <= 12)) && ((EF >= 1) && (EF <= 31)) && (GH < 24) && (IJ < 60) && (KL < 60)) // A valid year (until 2100) and month
		{
			memcpy(buf, partialString, 12);
			buf[12]='\0';
			return buf;
		}
		
		return null;
	}

	// =====================================================
	// NEW RULE: 6-character (ABxxxx) DAY/HOUR priority
	// =====================================================
	if (n == 6)
	{
		if(EF > 59) return null;

		// If hour is in the future, assume hhmmss
		if (AB >= hh)
		{
			if((AB < 24) && (CD < 60))
			{
 				formatTimestamp(buf, yy, mon, dd, AB, CD, EF);
 				return buf;
			}
		}

		if (AB >= dd) // If greater than or equal to today, assume ddhhmm
		{
			if(((AB >= 1) && (AB <= 31)) && (CD < 24))
			{
				formatTimestamp(buf, yy, mon, AB, CD, EF, 0);
				return buf;				
			}
		}
		
		return null; // Otherwise, it is an error
	}

	// =====================================================
	// NEW RULE: 8-character partial (AB CD EF GH)
	// day overrides month
	// =====================================================
	if (n == 8)
	{		
		if(GH > 59) return null;

		bool matchMonth = (AB != mon);
		bool matchDay   = (AB >= dd);

		// If both ? day wins: assume ddhhmmss
		if (matchDay)
		{
			if(((AB >= 1) && (AB <= 31)) && (CD < 24) && (EF < 60))
			{
				formatTimestamp(buf,yy,mon,AB,CD,EF,GH);
				return buf;
			}
		}

		if (matchMonth) // assume mmddhhmm
		{
			if(((AB >= 1) && (AB <= 12)) && ((CD >= 1) && (CD <= 31)) && (EF < 24))
			{
				formatTimestamp(buf,yy,AB,CD,EF,GH,0);
				return buf;
			}
		}
		
		return null;
	}

	// =====================================================
	// NEW RULE: 10-character partial
	// month overrides year if both match prefix
	// =====================================================
	if (n == 10)
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




/*
 * Converts a datetime string into an epoch value
 * @param error - pointer to an error flag, set to 1 if an error occurs
 * @param datetime - character string in the format "YYMMDDhhmmss"
 * @return epoch value representing the given date and time, or current RTC time if datetime is null
 */
time_t String2Epoch(bool *error, char *datetime)
{
	time_t epoch = 0;
	uint8_t data[7] = { 0, 0, 0, 0, 0, 0, 0 };

	struct tm ltm = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	int16_t year = 100;                 /* start at 100 years past 1900 */
	uint8_t month;
	uint8_t date;
	uint8_t hours;
	uint8_t minutes;
	uint8_t seconds;

	if(datetime)                            /* String format "YYMMDDhhmmss" */
	{
		data[0] = char2bcd(&datetime[10]);  /* seconds in BCD */
		data[1] = char2bcd(&datetime[8]);   /* minutes in BCD */
		data[2] = char2bcd(&datetime[6]);   /* hours in BCD */
		/* data[3] =  not used */
		data[4] = char2bcd(&datetime[4]);   /* day of month in BCD */
		data[5] = char2bcd(&datetime[2]);   /* month in BCD */
		data[6] = char2bcd(&datetime[0]);   /* 2-digit year in BCD */

		hours = bcd2dec(data[2]); /* Must be calculated here */

		year += (int16_t)bcd2dec(data[6]);
		ltm.tm_year = year;                         /* year since 1900 */

		year += 1900;                               /* adjust year to calendar year */

		month = bcd2dec(data[5]);
		ltm.tm_mon = month - 1;                     /* mon 0 to 11 */

		date = bcd2dec(data[4]);
		ltm.tm_mday = date;                         /* month day 1 to 31 */

		ltm.tm_yday = 0;
		for(uint8_t mon = 1; mon < month; mon++)    /* months from 1 to 11 (excludes partial month) */
		{
			ltm.tm_yday += month_length(year, mon);;
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

	return(epoch);
}



/*
 * Function: bcd2dec
 * -----------------
 * This function converts a Binary-Coded Decimal (BCD) value to a decimal value.
 * In BCD, each digit is represented by a 4-bit binary value. This function extracts
 * the tens and units portions of the BCD value and calculates the equivalent decimal value.

 * Parameters:
 *  - val: A uint8_t value representing a number in BCD format.

 * Return:
 *  - A uint8_t value representing the equivalent decimal value.

 * Example:
 *  If val = 0x25 (BCD for 25), the function will return 25 in decimal format.
 *
 * Conversion Steps:
 *  - The tens digit is extracted by shifting the upper 4 bits to the right (val >> 4).
 *  - The units digit is extracted by taking the lower 4 bits (val & 0x0F).
 *  - The final decimal result is calculated as: (10 * tens) + units.
 */
uint8_t bcd2dec(uint8_t val)
{
	uint8_t result = 10 * (val >> 4) + (val & 0x0F);
	return( result);
}

/*
 * Converts a value from decimal to Binary Coded Decimal (BCD)
 * @param val - the decimal value to convert (0-99)
 * @return the value converted to BCD
 */
uint8_t dec2bcd(uint8_t val)
{
	uint8_t result = val % 10;
	result |= (val / 10) << 4;
	return (result);
}

/*
 * Converts a character array to BCD format
 * @param c - a character array representing a number
 * @return the value converted to BCD
 */
uint8_t char2bcd(char c[])
{
	uint8_t result = (c[1] - '0') + ((c[0] - '0') << 4);
	return( result);
}

/*
 * Converts a tm struct to an epoch time (seconds since 1970)
 * @param ltm - a pointer to a tm struct with time information
 * @return epoch value representing the given local time
 */
time_t epoch_from_ltm(tm *ltm)
{
	time_t epoch = ltm->tm_sec + ltm->tm_min * 60 + ltm->tm_hour * 3600L + ltm->tm_yday * 86400L +
	(ltm->tm_year - 70) * 31536000L + ((ltm->tm_year - 69) / 4) * 86400L -
	((ltm->tm_year - 1) / 100) * 86400L + ((ltm->tm_year + 299) / 400) * 86400L;

	return(epoch);
}

/**
 * Converts a string of format "yyyy-mm-ddThh:mm:ss" to seconds since 1900
 */
uint32_t convertTimeStringToEpoch(char * s)
{
  unsigned long result = 0;
  struct tm ltm = {0};

  if (!mystrptime(s, &ltm)) {
    result = ltm.tm_sec + ltm.tm_min*60 + ltm.tm_hour*3600L + ltm.tm_yday*86400L +
    (ltm.tm_year-70)*31536000L + ((ltm.tm_year-69)/4)*86400L -
    ((ltm.tm_year-1)/100)*86400L + ((ltm.tm_year+299)/400)*86400L;
  }

  return result;
}

#endif //  DATE_STRING_SUPPORT_ENABLED

/** 
 * Checks a string to see if it contains only numerical characters
 */
bool only_digits(char *s)
{
	while(*s)
	{
		if(isdigit(*s++) == 0)
		{
			return( false);
		}
	}

	return( true);
}


/** 
 * Convert a frequency string to a proper Hz value and string format based on assumptions 
 * related to the size and decimal properties of the number contained in the string.
 * result = pointer to a character sting to hold the frequency string
 * freq = the frequency value to be represented as a string
 * Returns 1 if an error is detected
 */
bool frequencyString(char* result, uint32_t freq)
{
	bool failure = true;
	
	if(!result)
	{
		return(failure);
	}
	
	if((freq >= TX_MINIMUM_FREQUENCY) && (freq <= TX_MAXIMUM_FREQUENCY)) // Accept only a Hz value to be expressed in kHz
	{
		uint32_t frac = (freq % 1000)/100;		
		sprintf(result, "%lu.%1lu kHz", freq/1000, frac);
		
		failure = false;
	}
	
	return(failure);	
}

/** 
 * Convert a frequency string to a proper Hz value and string format based on assumptions 
 * related to the size and decimal properties of the number contained in the string.
 * str = pointer to a string containing the frequency string
 * result = pointer to a Frequency_Hz variable to hold the frequency in Hz
 * Returns 1 if an error is detected
 */
bool frequencyVal(char* str, Frequency_Hz* result)
{
	bool failure = true;
	float mhzMin, mhzMax, khzMin, khzMax, hzMin, hzMax;
	
	mhzMin = (float)TX_MINIMUM_FREQUENCY/1000000.;
	mhzMax = (float)TX_MAXIMUM_FREQUENCY/1000000.;
	khzMin = (float)TX_MINIMUM_FREQUENCY/1000.;
	khzMax = (float)TX_MAXIMUM_FREQUENCY/1000.;
	hzMin = 3500000.;
	hzMax = 4000000.;
	
	if(!str)
	{
		return(failure);
	}
		
	float f = atof(str);
		
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
		Frequency_Hz temp = (Frequency_Hz)ceilf(f);
		if(result) *result = temp;
		sprintf(str, "%4.1f kHz", (double)f);
	}
	
	return(failure);	
}

/**
 * Translate a Function_t enum value into a descriptive string.
 *
 * @param str Buffer to receive the textual description.
 * @param fun Enumerated function value to describe.
 * @return true if the function is unrecognized, false otherwise.
 */
bool function2Text(char* str, Function_t fun)
{
	bool failure = false;
	
	switch(fun)
	{
		case Function_ARDF_TX:
		{
			sprintf(str, "Radio Orienteering");
		}
		break;
		
		case Function_QRP_TX:
		{
			sprintf(str, "QRP Transmitter");
		}
		break;
		
		case Function_Signal_Gen:
		{
			sprintf(str, "Signal Generator");
		}
		break;
		
		default:
		{
			failure = true;
		}		
	}
	
	return failure;
}

/**
 * Translate a Fox_t enum value into a descriptive string.
 *
 * @param str Buffer to receive the textual description.
 * @param fox Enumerated fox value to describe.
 * @return true if the fox value is unrecognized, false otherwise.
 */
bool fox2Text(char* str, Fox_t fox)
{
	bool failure = false;
	
	switch(fox)
	{
		case BEACON:
		{
			sprintf(str, "Beacon \"MO\"");
		}
		break;
		
		case FOX_1:
		{
			sprintf(str, "Classic Fox 1 \"MOE\"");
		}
		break;
		
		case FOX_2:
		{
			sprintf(str, "Classic Fox 2 \"MOI\"");
		}
		break;
		
		case FOX_3:
		{
			sprintf(str, "Classic Fox 3 \"MOS\"");
		}
		break;
		
		case FOX_4:
		{
			sprintf(str, "Classic Fox 4 \"MOH\"");
		}
		break;
		
		case FOX_5:
		{
			sprintf(str, "Classic Fox 5 \"MO5\"");
		}
		break;
		
		case SPECTATOR:
		{
			sprintf(str, "Spectator \"S\"");
		}
		break;
		
		case SPRINT_S1:
		{
			sprintf(str, "Sprint Slow 1 \"ME\"");
		}
		break;
		
		case SPRINT_S2:
		{
			sprintf(str, "Sprint Slow 2 \"MI\"");
		}
		break;
		
		case SPRINT_S3:
		{
			sprintf(str, "Sprint Slow 3 \"MS\"");
		}
		break;
		
		case SPRINT_S4:
		{
			sprintf(str, "Sprint Slow 4 \"MH\"");
		}
		break;
		
		case SPRINT_S5:
		{
			sprintf(str, "Sprint Slow 5 \"M5\"");
		}
		break;
		
		case SPRINT_F1:
		{
			sprintf(str, "Sprint Fast 1 \"OE\"");
		}
		break;
		
		case SPRINT_F2:
		{
			sprintf(str, "Sprint Fast 2 \"OI\"");
		}
		break;
		
		case SPRINT_F3:
		{
			sprintf(str, "Sprint Fast 3 \"OS\"");
		}
		break;
		
		case SPRINT_F4:
		{
			sprintf(str, "Sprint Fast 4 \"OH\"");
		}
		break;
		
		case SPRINT_F5:
		{
			sprintf(str, "Sprint Fast 5 \"O5\"");
		}
		break;
		
		case FOXORING_FOX1:
		{
			sprintf(str, "Foxoring \"Low Freq\" Fox");
		}
		break;
		
		case FOXORING_FOX2:
		{
			sprintf(str, "Foxoring \"Medium Freq\" Fox");
		}
		break;
		
		case FOXORING_FOX3:
		{
			sprintf(str, "Foxoring \"High Freq\" Fox");
		}
		break;
		
		case FREQUENCY_TEST_BEACON:
		{
			sprintf(str, "Frequency Test Beacon");
		}
		break;
		
		default:
		{
			failure = true;
		}
		break;
	}
	
	return(failure);
}


/**
 * Translate an Event_t enum value into a descriptive string.
 *
 * @param str Buffer to receive the textual description.
 * @param evt Enumerated event value to describe.
 * @return true if the event value is unrecognized, false otherwise.
 */
bool event2Text(char* str, Event_t evt)
{
	bool failure = false;
	
	switch(evt)
	{
		case EVENT_CLASSIC:
		{
			sprintf(str, "Classic");
		}
		break;
		
		case EVENT_SPRINT:
		{
			sprintf(str, "Sprint");
		}
		break;
		
		case EVENT_FOXORING:
		{
			sprintf(str, "Foxoring");
		}
		break;
		
		case EVENT_BLIND_ARDF:
		{
			sprintf(str, "Blind ARDF");
		}
		break;
		
		case EVENT_NONE:
		{
			sprintf(str, "None Set");
		}
		break;
		
		default:
		{
			failure = true;
		}
		break;
	}
	
	return(failure);
}



/*-------------------------------------------------------------
 *  float_to_parts_signed
 *  ---------------------
 *  Split a float (positive or negative) into:
 *      – characteristic : signed integer part      (-32768 … 32767)
 *      – mantissa       : first decimal digit   (0 … 9)
 *
 *  Returns:  false  ? success
 *            true   ? error   (bad args, NaN/Inf, out of range)
 *------------------------------------------------------------*/
bool float_to_parts_signed(float value,
                           int16_t  *integerPart,   /* signed  */
                           uint16_t *fractionPart)         /* unsigned */
{
    /* pointer validity */
    if (integerPart == NULL || fractionPart == NULL)
        return true;

    /* reject NaN or ±Inf */
    if (isnanf(value) || isinff(value))
        return true;

    /* split into integer and fractional parts */
    float int_part_f;
    float frac_part_f = modff(value, &int_part_f);   /* both carry the sign of value */

    /* range-check integer part for int16_t */
    if (int_part_f < (float)INT16_MIN || int_part_f > (float)INT16_MAX)
        return true;

    /* scale fractional part to 1 decimal place, keep it non-negative */
    float scaled = roundf(fabsf(frac_part_f) * 10.0f);

    if (scaled > (float)UINT16_MAX)          /* should never happen, but guard anyway */
        return true;

    /* commit results */
    *integerPart = (int16_t)int_part_f;   /* may be negative */
    *fractionPart = (uint16_t)scaled;      /* always positive */

    return false;   /* success */
}
