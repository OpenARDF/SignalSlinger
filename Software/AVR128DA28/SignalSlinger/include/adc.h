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
 * ADC channel selection, conversion control, and measurement helpers.
 *
 * This module contains support functions for:
 * - selecting ADC input sources used by the firmware
 * - running single-shot or free-running conversions
 * - converting raw ADC readings into voltage or temperature values
 *
 * Sampling policy and interpretation of those measurements belong elsewhere.
 */

#ifndef ADC_H_
#define ADC_H_

#define FREE_RUNNING true
#define SINGLE_CONVERSION false

typedef enum
{
	ADCInternalBatteryVoltage,
	ADCExternalBatteryVoltage,
	ADC12VRegulatedVoltage,
	ADCTXAdjustableVoltage,
	ADCTemperature,
	ADCShutdown
} ADC_Active_Channel_t;

typedef enum
{
	ADC_NOT_INITIALIZED,
	ADC_FREE_RUN_INITIALIZED,
	ADC_SINGLE_CONVERSION_INITIALIZED
} ADC_Init_t;

/**
 * Select the active ADC input channel, initializing the ADC for single conversion if needed.
 *
 * @param chan ADC channel selection to apply.
 */
void ADC0_setADCChannel(ADC_Active_Channel_t chan);

/**
 * Start an ADC conversion when the ADC subsystem is initialized.
 */
void ADC0_startConversion(void);

/**
 * Report whether the most recent ADC conversion has completed.
 *
 * @return true when the ADC result-ready flag is set.
 */
bool ADC0_conversionDone(void);

/**
 * Read the current ADC result register.
 *
 * @return Raw ADC conversion result.
 */
int ADC0_read(void);

/**
 * Convert a raw ADC temperature reading to degrees Celsius.
 *
 * @param adc_reading Raw ADC result from the temperature sensor channel.
 * @return Temperature in degrees Celsius.
 */
float temperatureCfromADC(uint16_t adc_reading);

/**
 * Check whether a temperature reading falls inside the firmware's valid range.
 *
 * @param temperatureC Temperature in degrees Celsius.
 * @return true when the reading is considered valid.
 */
bool isValidTemp(float temperatureC);

/**
 * Perform a single conversion on the requested voltage channel and scale it to volts.
 *
 * @param chan ADC channel to sample.
 * @return Measured voltage, or 0 when the conversion does not complete in time.
 */
float readVoltage(ADC_Active_Channel_t chan);

/**
 * Initialize the ADC subsystem and its supporting port/reference configuration.
 *
 * @param freerun true to enable free-running conversions; false for single-shot operation.
 */
void ADC0_SYSTEM_init(bool freerun);

/**
 * Shut down the ADC subsystem and mark it uninitialized.
 */
void ADC0_SYSTEM_shutdown(void);

#endif /* ADC_H_ */
