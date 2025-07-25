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


#ifndef ADC_H_
#define ADC_H_

	
#define FREE_RUNNING true
#define SINGLE_CONVERSION false

typedef enum {
	ADCInternalBatteryVoltage,
	ADCExternalBatteryVoltage,
	ADC12VRegulatedVoltage,
	ADCTXAdjustableVoltage,
	ADCTemperature,
	ADCShutdown
} ADC_Active_Channel_t;

typedef enum {
	ADC_NOT_INITIALIZED,
	ADC_FREE_RUN_INITIALIZED,
	ADC_SINGLE_CONVERSION_INITIALIZED
} ADC_Init_t;

void ADC0_setADCChannel(ADC_Active_Channel_t chan);
void ADC0_startConversion(void);
bool ADC0_conversionDone(void);
int ADC0_read(void);
float temperatureC(void);
float temperatureCfromADC(uint16_t adc_reading);
bool isValidTemp(float temperatureC);
float readVoltage(ADC_Active_Channel_t chan);
void ADC0_SYSTEM_init(bool freerun);
void ADC0_SYSTEM_shutdown(void);


#endif /* ADC_H_ */