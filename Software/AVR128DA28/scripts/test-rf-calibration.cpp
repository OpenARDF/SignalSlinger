#include <cassert>
#include <cstdint>
#include <limits>

#include "rf_calibration.h"

static void expectFrequency(const char *text, uint32_t expected_hz)
{
	uint32_t actual_hz = 0;
	assert(rfCalibrationParseFrequencyHz(text, &actual_hz));
	assert(actual_hz == expected_hz);
}

int main()
{
	expectFrequency("3600", 3600000UL);
	expectFrequency("3.6", 3600000UL);
	expectFrequency("15000", 15000000UL);
	expectFrequency("3600K", 3600000UL);
	expectFrequency("3600KHZ", 3600000UL);
	expectFrequency("3.6M", 3600000UL);
	expectFrequency("3.600001MHz", 3600001UL);
	expectFrequency("8", RF_CALIBRATION_MIN_FREQUENCY_HZ);
	expectFrequency("0.008", RF_CALIBRATION_MIN_FREQUENCY_HZ);
	expectFrequency("8K", RF_CALIBRATION_MIN_FREQUENCY_HZ);
	expectFrequency("160000", RF_CALIBRATION_MAX_FREQUENCY_HZ);
	expectFrequency("160.0", RF_CALIBRATION_MAX_FREQUENCY_HZ);
	expectFrequency("160M", RF_CALIBRATION_MAX_FREQUENCY_HZ);
	assert(rfCalibrationNormalizeFrequencyHz(1000099UL) == 1000000UL);
	assert(rfCalibrationNormalizeFrequencyHz(999999UL) == 999999UL);

	uint32_t frequency_hz = 0;
	assert(!rfCalibrationParseFrequencyHz("7", &frequency_hz));
	assert(!rfCalibrationParseFrequencyHz("0.007999", &frequency_hz));
	assert(!rfCalibrationParseFrequencyHz("7.999K", &frequency_hz));
	assert(!rfCalibrationParseFrequencyHz("160.000001M", &frequency_hz));
	assert(!rfCalibrationParseFrequencyHz("3600000", &frequency_hz));
	assert(!rfCalibrationParseFrequencyHz("-3600000", &frequency_hz));
	assert(!rfCalibrationParseFrequencyHz("3.6MB", &frequency_hz));
	assert(!rfCalibrationParseFrequencyHz("", &frequency_hz));

	int32_t offset_hz = 0;
	assert(rfCalibrationParseOffsetHz("+15", &offset_hz) && offset_hz == 15);
	assert(rfCalibrationParseOffsetHz("-15", &offset_hz) && offset_hz == -15);
	assert(rfCalibrationParseOffsetHz("2147483647", &offset_hz) &&
	       offset_hz == std::numeric_limits<int32_t>::max());
	assert(rfCalibrationParseOffsetHz("-2147483648", &offset_hz) &&
	       offset_hz == std::numeric_limits<int32_t>::min());
	assert(!rfCalibrationParseOffsetHz("2147483648", &offset_hz));
	assert(!rfCalibrationParseOffsetHz("15Hz", &offset_hz));

	int32_t correction_ppb = 0;
	assert(rfCalibrationCorrectionForOffsetHz(3600000UL, 15, &correction_ppb));
	assert(correction_ppb == -4167);
	assert(rfCalibrationOffsetHzForCorrection(3600000UL, correction_ppb) == 15);
	assert(rfCalibrationCorrectionForOffsetHz(3600000UL, -15, &correction_ppb));
	assert(correction_ppb == 4167);
	assert(rfCalibrationOffsetHzForCorrection(3600000UL, correction_ppb) == -15);

	assert(rfCalibrationCorrectionForOffsetHz(10000000UL, 1, &correction_ppb));
	assert(correction_ppb == -100);
	assert(rfCalibrationCorrectionForOffsetHz(3600000UL, 3600, &correction_ppb));
	assert(correction_ppb == -RF_CALIBRATION_MAX_CORRECTION_PPB);
	assert(!rfCalibrationCorrectionForOffsetHz(3600000UL, 3601, &correction_ppb));

	return 0;
}
