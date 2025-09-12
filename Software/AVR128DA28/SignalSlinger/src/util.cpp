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
