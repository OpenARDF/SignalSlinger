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


#include "binio.h"
#include "port.h"
#include "defs.h"
#include "atmel_start_pins.h"
#include "adc.h"

extern ADC_Init_t g_adc_initialization;

uint8_t portDpinReadings[3];
uint8_t portDdebounced;

// default constructor
binio::binio()
{
	portDdebounced = 0;
} //binio

// default destructor
binio::~binio()
{
} //~binio


// This function is called approximately each 1/60 to 1/30 sec.
void debounce(void)
{
	// Move previously sampled raw input bits one step down the line.
	portDpinReadings[2] = portDpinReadings[1];
	portDpinReadings[1] = portDpinReadings[0];

	// Sample new raw input bits from PORT_A.
	portDpinReadings[0] = PORTD_get_port_level();

	// Debounce output bits using low-pass filtering.
	portDdebounced = portDdebounced ^ (
	(portDdebounced ^ portDpinReadings[0])
	& (portDdebounced ^ portDpinReadings[1])
	& (portDdebounced ^ portDpinReadings[2]));
}

uint8_t portDdebouncedVals(void)
{
	return portDdebounced;
}


void BINIO_init(void)
{
	/* PORTA *************************************************************************************/
// 	PORTA_set_pin_dir(unusedA0, PORT_DIR_OUT);
// 	PORTA_set_pin_level(unusedA0, HIGH);
	
 	PORTA_set_pin_dir(RF_OUT_ENABLE, PORT_DIR_OUT);
	PORTA_set_pin_level(RF_OUT_ENABLE, LOW);

	PORTA_set_pin_dir(WIFI_MODULE_DETECT, PORT_DIR_IN); /* Detect presence of Huzzah module */
	PORTA_set_pin_pull_mode(WIFI_MODULE_DETECT, PORT_PULL_OFF);

 	PORTA_set_pin_dir(TO_WIFI_RX, PORT_DIR_OUT);

 	PORTA_set_pin_dir(TO_WIFI_TX, PORT_DIR_IN);
	PORTA_set_pin_pull_mode(TO_WIFI_TX, PORT_PULL_UP);

	PORTA_set_pin_dir(WIFI_ENABLE, PORT_DIR_OUT);
	PORTA_set_pin_level(WIFI_ENABLE, LOW);
	
	PORTA_set_pin_dir(V3V3_PWR_ENABLE, PORT_DIR_OUT);
	PORTA_set_pin_level(V3V3_PWR_ENABLE, LOW);
	
	PORTA_set_pin_dir(WIFI_RESET, PORT_DIR_OUT);
	PORTA_set_pin_level(WIFI_RESET, LOW);
	
	/* PORTC *************************************************************************************/
	
	PORTC_set_pin_dir(SERIAL_TX, PORT_DIR_OUT);
	PORTC_set_pin_dir(SERIAL_RX, PORT_DIR_IN);
// 	PORTC_set_pin_dir(SI5351_SDA, PORT_DIR_OUT);
// 	PORTC_set_pin_dir(SI5351_SCL, PORT_DIR_IN);
	
	/* PORTD *************************************************************************************/
	PORTD_set_pin_dir(VBAT_IN, PORT_DIR_IN);

	PORTD_set_pin_dir(VDIV_ENABLE, PORT_DIR_OUT);
	PORTD_set_pin_level(VDIV_ENABLE, LOW);

	PORTD_set_pin_dir(LED_RED, PORT_DIR_OUT);
	PORTD_set_pin_level(LED_RED, LOW);

	PORTD_set_pin_dir(SWITCH, PORT_DIR_IN);
	PORTD_set_pin_pull_mode(SWITCH, PORT_PULL_UP);
	PORTD_pin_set_isc(SWITCH, PORT_ISC_BOTHEDGES_gc);
	
	g_adc_initialization = ADC_NOT_INITIALIZED; /* Reset ADC configuration */

	/* PORTF *************************************************************************************/
// 	PORTF_set_pin_dir(X32KHZ_SQUAREWAVE, PORT_DIR_OFF);	
}
