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

#include "defs.h"
#include "adc.h"
#include <avr/io.h>
#include <stdbool.h>
#include <driver_init.h>
#include <compiler.h>

static void PORT_init(void);
static void VREF0_init(void);
static void ADC0_init(bool freerun);

/* AVR128DA temperature measurements require >=25 us initialization and >=28 us
 * sample length (DS40002183, Temperature Measurement). At 24 MHz / 64,
 * 16 initialization clocks provide 42.7 us and 11 sample clocks provide 29.3 us.
 * Use hardware timing so periodic conversions still complete asynchronously. */
static constexpr uint32_t ADC_CLOCK_HZ = F_CPU / 64UL;
static constexpr uint8_t TEMPERATURE_SAMPLE_CLOCKS = (28UL * ADC_CLOCK_HZ + 999999UL) / 1000000UL;
static_assert(16UL * 1000000UL >= 25UL * ADC_CLOCK_HZ, "ADC initialization delay is too short");

ADC_Init_t g_adc_initialization = ADC_NOT_INITIALIZED;
/* Public peripheral initialization does not know the next requested input. */
static volatile bool adcChannelPrepared = false;

/**
 * Select the active ADC input channel, initializing the ADC for single conversion if needed.
 *
 * @param chan ADC channel selection to apply.
 */
void ADC0_setADCChannel(ADC_Active_Channel_t chan)
{
	uint8_t mux;
	switch(chan)
	{
		case ADCInternalBatteryVoltage: mux = ADC_MUXPOS_AIN0_gc; break;
		case ADCExternalBatteryVoltage: mux = ADC_MUXPOS_AIN1_gc; break;
		case ADC12VRegulatedVoltage: mux = ADC_MUXPOS_AIN4_gc; break;
		case ADCTXAdjustableVoltage: mux = ADC_MUXPOS_AIN5_gc; break;
		case ADCTemperature: mux = ADC_MUXPOS_TEMPSENSE_gc; break;
		default:
			ADC0_SYSTEM_shutdown();
			return;
	}

	if(g_adc_initialization != ADC_SINGLE_CONVERSION_INITIALIZED || !adcChannelPrepared)
	{
		/* Select the input BEFORE enabling the ADC/reference. With INITDLY,
		 * changing MUX afterward can leave the first result on the old channel.
		 * Stop any prior free-running conversion before reconfiguration, too. */
		ADC0_SYSTEM_shutdown();
		ADC0.MUXPOS = mux;
		ADC0_SYSTEM_init(SINGLE_CONVERSION);
	}
	else
	{
		ADC0.MUXPOS = mux;
	}

	/* Restore the short voltage acquisition after leaving the temperature input. */
	ADC0.SAMPCTRL = (chan == ADCTemperature) ? TEMPERATURE_SAMPLE_CLOCKS : 0;
	adcChannelPrepared = true;
}

/**
 * Start an ADC conversion when the ADC subsystem is initialized.
 */
void ADC0_startConversion(void)
{
	if(g_adc_initialization != ADC_NOT_INITIALIZED)
	{
		ADC0.INTCTRL = 0x00;          /* Disable interrupt */
		ADC0.INTFLAGS = ADC_RESRDY_bm; /* A previous result cannot complete this request. */
		ADC0.COMMAND = ADC_STCONV_bm; /* Start conversion */
	}
}

/**
 * Report whether the most recent ADC conversion has completed.
 *
 * @return true when the ADC result-ready flag is set.
 */
bool ADC0_conversionDone(void)
{
	/* Check if the conversion is done  */
	return (ADC0.INTFLAGS & ADC_RESRDY_bm);
}

/**
 * Read the current ADC result register.
 *
 * @return Raw ADC conversion result.
 */
int ADC0_read()
{
	return ADC0.RES; /* Reading the result also clears the interrupt flag */
}

/**
 * Acquire one foreground sample, returning false on timeout or invalid input.
 * Callers must exclude the periodic ADC service while this blocking read owns
 * the converter. Abort any displaced conversion before selecting the input.
 */
static bool readSingleConversion(ADC_Active_Channel_t chan, uint16_t *result)
{
	ADC0_SYSTEM_shutdown();
	ADC0_setADCChannel(chan);
	if(g_adc_initialization != ADC_SINGLE_CONVERSION_INITIALIZED)
	{
		return false;
	}
	ADC0_startConversion();

	for(uint16_t remaining = 10000; remaining > 0; --remaining)
	{
		if(ADC0_conversionDone())
		{
			*result = ADC0_read();
			return true;
		}
	}

	/* Do not accept a stale result or leave a timed-out conversion running. */
	ADC0_SYSTEM_shutdown();
	return false;
}

/**
 * Perform a single conversion on the requested voltage channel and scale it to volts.
 *
 * @param chan ADC channel to sample.
 * @return Measured voltage, or 0 when the conversion does not complete in time.
 */
float readVoltage(ADC_Active_Channel_t chan)
{
	uint16_t adc_reading;
	if(readSingleConversion(chan, &adc_reading))
	{
		return (0.00725 * (float)adc_reading) + 0.05;
	}
	return 0;
}

/**
 * Perform a single conversion on the temperature sensor channel and convert it to C.
 *
 * @return Measured temperature in degrees Celsius, or an invalid reading when the conversion times out.
 */
float readTemperature(void)
{
	uint16_t adc_reading;
	if(readSingleConversion(ADCTemperature, &adc_reading))
	{
		return temperatureCfromADC(adc_reading);
	}
	return MINIMUM_VALID_TEMP - 1.;
}

/**
 * Check whether a temperature reading falls inside the firmware's valid range.
 *
 * @param temperatureC Temperature in degrees Celsius.
 * @return true when the reading is considered valid.
 */
bool isValidTemp(float temperatureC)
{
	return ((temperatureC > MINIMUM_VALID_TEMP) && (temperatureC < MAXIMUM_VALID_TEMP));
}

/**
 * Convert a raw ADC temperature reading to degrees Celsius.
 *
 * @param adc_reading Raw ADC result from the temperature sensor channel.
 * @return Temperature in degrees Celsius.
 */
float temperatureCfromADC(uint16_t adc_reading)
{
	uint16_t sigrow_offset = SIGROW.TEMPSENSE1; // Read unsigned value from signature row
	uint16_t sigrow_slope = SIGROW.TEMPSENSE0;  // Read unsigned value from signature row
	int32_t sensor_delta = (int32_t)sigrow_offset - (int32_t)adc_reading;
	float temperature_in_K = ((float)sensor_delta * (float)sigrow_slope) / 4096.0f;

	return (temperature_in_K - 273.15f);
}

/**
 * Configure the ADC input pin used for internal battery-voltage measurement.
 */
static void PORT_init(void)
{
	/* Disable interrupt and digital input buffer on PD0 */
	PORTD.PIN0CTRL &= ~PORT_ISC_gm;
	PORTD.PIN0CTRL |= PORT_ISC_INPUT_DISABLE_gc;

	/* Disable pull-up resistor */
	PORTD.PIN0CTRL &= ~PORT_PULLUPEN_bm;
}

/**
 * Configure the ADC voltage-reference source.
 */
static void VREF0_init(void)
{
	VREF.ADC0REF = VREF_REFSEL_2V048_gc; /* Internal 2.048V reference */
}

/**
 * Apply the core ADC register configuration for either free-running or single-shot use.
 *
 * @param freerun true to enable free-running conversions; false for single-shot operation.
 */
static void ADC0_init(bool freerun)
{
	ADC0.CTRLC = ADC_PRESC_DIV64_gc; /* 24 MHz / 64 = 375 kHz ADC clock */
	ADC0.CTRLD = ADC_INITDLY_DLY16_gc;
	ADC0.SAMPCTRL = 0;

	if(freerun)
	{
		ADC0.CTRLA = ADC_ENABLE_bm         /* ADC Enable: enabled */
		             | ADC_RESSEL_12BIT_gc /* 12-bit mode */
		             | ADC_FREERUN_bm;     /* Enable Free-Run mode */

		ADC0.INTCTRL = 0x01; /* Enable interrupt */

		ADC0.COMMAND = ADC_STCONV_bm; /* Start conversion */
		g_adc_initialization = ADC_FREE_RUN_INITIALIZED;
	}
	else
	{
		ADC0.CTRLA |= ADC_ENABLE_bm; /* ADC Enable: enabled; 12-bit mode is default */
		ADC0.INTCTRL = 0x00;         /* Disable interrupt */
		g_adc_initialization = ADC_SINGLE_CONVERSION_INITIALIZED;
	}
}

/**
 * Initialize the ADC subsystem and its supporting port/reference configuration.
 *
 * @param freerun true to enable free-running conversions; false for single-shot operation.
 */
void ADC0_SYSTEM_init(bool freerun)
{
	ADC0_SYSTEM_shutdown();
	PORT_init();
	VREF0_init();
	ADC0_init(freerun);
}

/**
 * Shut down the ADC subsystem and mark it uninitialized.
 */
void ADC0_SYSTEM_shutdown(void)
{
	ADC0.INTCTRL = 0x00;              /* Disable interrupt */
	ADC0.CTRLA = ADC_RESSEL_12BIT_gc; /* Turn off ADC leaving 12-bit resolution set */
	g_adc_initialization = ADC_NOT_INITIALIZED;
	adcChannelPrepared = false;
}

/**
 * ADC result-ready ISR used by the free-running conversion mode.
 */
ISR(ADC0_RESRDY_vect)
{
	ADC0_read();
	ADC0.INTCTRL = 0x00; /* disable ADC interrupt */
}
