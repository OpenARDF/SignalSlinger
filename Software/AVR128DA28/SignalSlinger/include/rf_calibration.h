/*
 * Pure RF-calibration parsing and conversion helpers.
 *
 * Keeping this arithmetic independent from AVR peripherals makes the serial
 * command contract host-testable while the transmitter module remains the sole
 * owner of applying values to the Si5351.
 */

#ifndef SIGNALSLINGER_RF_CALIBRATION_H
#define SIGNALSLINGER_RF_CALIBRATION_H

#include <stdint.h>

#define RF_CALIBRATION_DEFAULT_FREQUENCY_HZ 3600000UL
#define RF_CALIBRATION_MIN_FREQUENCY_HZ 8000UL
#define RF_CALIBRATION_MAX_FREQUENCY_HZ 160000000UL
#define RF_CALIBRATION_MAX_CORRECTION_PPB 1000000L

static inline bool rfCalibrationFrequencyIsValid(uint32_t frequency_hz)
{
	return frequency_hz >= RF_CALIBRATION_MIN_FREQUENCY_HZ &&
	       frequency_hz <= RF_CALIBRATION_MAX_FREQUENCY_HZ;
}

static inline bool rfCalibrationCorrectionIsValid(int32_t correction_ppb)
{
	return correction_ppb >= -RF_CALIBRATION_MAX_CORRECTION_PPB &&
	       correction_ppb <= RF_CALIBRATION_MAX_CORRECTION_PPB;
}

/* The existing Si5351 safety policy uses 100 Hz steps above 1 MHz. Normalize
 * the stored calibration selection to the frequency the library will program. */
static inline uint32_t rfCalibrationNormalizeFrequencyHz(uint32_t frequency_hz)
{
	if(frequency_hz > 999999UL)
		return (frequency_hz / 100UL) * 100UL;

	return frequency_hz;
}

/* Divide with symmetric nearest-integer rounding so positive and negative
 * operator offsets have matching behavior. */
static inline int64_t rfCalibrationRoundedDivide(int64_t numerator, int64_t denominator)
{
	if(numerator >= 0)
		return (numerator + denominator / 2) / denominator;

	return -((-numerator + denominator / 2) / denominator);
}

/* The Si5351 library adds positive correction to its assumed crystal
 * frequency, which moves the generated output downward. The operator-facing
 * offset therefore has the opposite sign from the persisted correction. */
static inline bool rfCalibrationCorrectionForOffsetHz(uint32_t frequency_hz,
	                                                   int32_t offset_hz,
	                                                   int32_t *correction_ppb)
{
	if(!correction_ppb || !rfCalibrationFrequencyIsValid(frequency_hz))
		return false;

	int64_t correction = rfCalibrationRoundedDivide(-(int64_t)offset_hz * 1000000000LL,
	                                                frequency_hz);
	if(correction < -RF_CALIBRATION_MAX_CORRECTION_PPB ||
	   correction > RF_CALIBRATION_MAX_CORRECTION_PPB)
		return false;

	*correction_ppb = (int32_t)correction;
	return true;
}

static inline int32_t rfCalibrationOffsetHzForCorrection(uint32_t frequency_hz,
	                                                      int32_t correction_ppb)
{
	if(!rfCalibrationFrequencyIsValid(frequency_hz) ||
	   !rfCalibrationCorrectionIsValid(correction_ppb))
		return 0;

	return (int32_t)rfCalibrationRoundedDivide(-(int64_t)correction_ppb * frequency_hz,
	                                           1000000000LL);
}

static inline char rfCalibrationAsciiUpper(char value)
{
	return (value >= 'a' && value <= 'z') ? (char)(value - ('a' - 'A')) : value;
}

static inline bool rfCalibrationSuffixEquals(const char *text, const char *expected)
{
	while(*text && *expected)
	{
		if(rfCalibrationAsciiUpper(*text++) != *expected++)
			return false;
	}

	return *text == '\0' && *expected == '\0';
}

/* Match the established FRE command convention across the Si5351 range:
 * whole numbers are kHz and decimal numbers are MHz. Explicit K/KHZ and M/MHZ
 * suffixes remain accepted for serial-client compatibility, but raw-Hz carrier
 * input is intentionally unsupported because calibration offsets provide the
 * required Hz granularity. */
static inline bool rfCalibrationParseFrequencyHz(const char *text, uint32_t *frequency_hz)
{
	if(!text || !frequency_hz || !*text)
		return false;

	const char *cursor = text;
	uint64_t whole = 0;
	bool have_whole_digit = false;
	while(*cursor >= '0' && *cursor <= '9')
	{
		have_whole_digit = true;
		whole = whole * 10U + (uint8_t)(*cursor - '0');
		if(whole > UINT32_MAX)
			return false;
		++cursor;
	}
	if(!have_whole_digit)
		return false;

	uint64_t fraction = 0;
	uint64_t fraction_scale = 1;
	bool have_fraction = false;
	if(*cursor == '.')
	{
		++cursor;
		while(*cursor >= '0' && *cursor <= '9')
		{
			if(fraction_scale >= 1000000000ULL)
				return false;
			have_fraction = true;
			fraction = fraction * 10U + (uint8_t)(*cursor - '0');
			fraction_scale *= 10U;
			++cursor;
		}
		if(!have_fraction)
			return false;
	}

	uint32_t multiplier;
	if(*cursor)
	{
		if(rfCalibrationSuffixEquals(cursor, "K") ||
		   rfCalibrationSuffixEquals(cursor, "KHZ"))
			multiplier = 1000UL;
		else if(rfCalibrationSuffixEquals(cursor, "M") ||
		        rfCalibrationSuffixEquals(cursor, "MHZ"))
			multiplier = 1000000UL;
		else
			return false;
	}
	else
	{
		multiplier = have_fraction ? 1000000UL : 1000UL;
	}

	uint64_t scaled = whole * multiplier;
	if(have_fraction)
		scaled += (fraction * multiplier + fraction_scale / 2U) / fraction_scale;
	if(scaled > UINT32_MAX || !rfCalibrationFrequencyIsValid((uint32_t)scaled))
		return false;

	*frequency_hz = (uint32_t)scaled;
	return true;
}

static inline bool rfCalibrationParseOffsetHz(const char *text, int32_t *offset_hz)
{
	if(!text || !offset_hz || !*text)
		return false;

	const char *cursor = text;
	bool negative = false;
	if(*cursor == '+' || *cursor == '-')
	{
		negative = *cursor == '-';
		++cursor;
	}
	if(*cursor < '0' || *cursor > '9')
		return false;

	uint64_t magnitude = 0;
	while(*cursor >= '0' && *cursor <= '9')
	{
		magnitude = magnitude * 10U + (uint8_t)(*cursor - '0');
		if(magnitude > (uint64_t)INT32_MAX + (negative ? 1U : 0U))
			return false;
		++cursor;
	}
	if(*cursor != '\0')
		return false;

	*offset_hz = negative ? (int32_t)(-(int64_t)magnitude) : (int32_t)magnitude;
	return true;
}

#endif /* SIGNALSLINGER_RF_CALIBRATION_H */
