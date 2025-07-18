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
uint8_t portApinReadings[3];
uint8_t portAdebounced;
void setExtBatLSEnable(bool state);
void v3V3_enable(bool state);
void boost_enable(bool state);

// default constructor
binio::binio()
{
	portDdebounced = 0;
	portAdebounced = 0;
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
	
	portApinReadings[2] = portApinReadings[1];
	portApinReadings[1] = portApinReadings[0];

	// Sample new raw input bits
	portDpinReadings[0] = PORTD_get_port_level();
	portApinReadings[0] = PORTA_get_port_level();

	// Debounce output bits using low-pass filtering.
	portDdebounced = portDdebounced ^ (
	(portDdebounced ^ portDpinReadings[0])
	& (portDdebounced ^ portDpinReadings[1])
	& (portDdebounced ^ portDpinReadings[2]));
	
	portAdebounced = portAdebounced ^ (
	(portAdebounced ^ portApinReadings[0])
	& (portAdebounced ^ portApinReadings[1])
	& (portAdebounced ^ portApinReadings[2]));
}

uint8_t portDdebouncedVals(void)
{
	return portDdebounced;
}

uint8_t portAdebouncedVals(void)
{
	return portAdebounced;
}


void BINIO_init(void)
{
	/* PORTA *************************************************************************************/
 	PORTA_set_pin_dir(CHARGE_AUX_ENABLE, PORT_DIR_OUT);
 	PORTA_set_pin_level(CHARGE_AUX_ENABLE, HIGH);
	
 	PORTA_set_pin_dir(FET_DRIVER_ENABLE, PORT_DIR_OUT);
	PORTA_set_pin_level(FET_DRIVER_ENABLE, LOW);

	PORTA_set_pin_dir(POWER_ENABLE, PORT_DIR_OUT); /* Enables/latches battery power to +VSW */
	PORTA_set_pin_level(POWER_ENABLE, HIGH);

 	PORTA_set_pin_dir(PADDLE_DIT, PORT_DIR_IN);
	PORTA_set_pin_pull_mode(PADDLE_DIT, PORT_PULL_OFF);

 	PORTA_set_pin_dir(PADDLE_DAH, PORT_DIR_IN);
	PORTA_set_pin_pull_mode(PADDLE_DAH, PORT_PULL_OFF);

	PORTA_set_pin_dir(STRAIGHTKEY, PORT_DIR_OUT);
	PORTA_set_pin_level(STRAIGHTKEY, LOW);
	
	PORTA_set_pin_dir(V3V3_PWR_ENABLE, PORT_DIR_OUT);
	PORTA_set_pin_level(V3V3_PWR_ENABLE, LOW);
	
	PORTA_set_pin_dir(BOOST_PWR_ENABLE, PORT_DIR_OUT);
	PORTA_set_pin_level(BOOST_PWR_ENABLE, LOW);
	
	/* PORTC *************************************************************************************/
	
	PORTC_set_pin_dir(SERIAL_TX, PORT_DIR_OUT);
	PORTC_set_pin_dir(SERIAL_RX, PORT_DIR_IN);
// 	PORTC_set_pin_dir(SI5351_SDA, PORT_DIR_OUT);
// 	PORTC_set_pin_dir(SI5351_SCL, PORT_DIR_IN);
	
	/* PORTD *************************************************************************************/
	PORTD_set_pin_dir(VBAT_INT, PORT_DIR_IN);
	PORTD_set_pin_pull_mode(VBAT_INT, PORT_PULL_OFF);
	PORTD_set_pin_dir(VBAT_EXT, PORT_DIR_IN);
	PORTD_set_pin_pull_mode(VBAT_EXT, PORT_PULL_OFF);

	PORTD_set_pin_dir(LED_RED, PORT_DIR_OUT);
	PORTD_set_pin_level(LED_RED, LOW);

	PORTD_set_pin_dir(SWITCH, PORT_DIR_IN);
	PORTD_set_pin_pull_mode(SWITCH, PORT_PULL_UP);
	PORTD_pin_set_isc(SWITCH, PORT_ISC_FALLING_gc);
	
	PORTD_set_pin_dir(LED_GREEN, PORT_DIR_OUT);
	PORTD_set_pin_level(LED_GREEN, LOW);

	g_adc_initialization = ADC_NOT_INITIALIZED; /* Reset ADC configuration */

	/* PORTF *************************************************************************************/
// 	PORTF_set_pin_dir(X32KHZ_SQUAREWAVE, PORT_DIR_OFF);	
}


static volatile bool driverCallerStates[NUMBER_OF_LS_CONTROLLERS] = {OFF, OFF};
/**
 State machine to keep track of multiple controllers of the FET driver load switch. This ensures that the switch is ON if any of the controllers has turned it on.
 */
bool setFETDriverLoadSwitch(bool onoff, hardwareResourceClients sender)
{
	switch(sender)
	{
		case INTERNAL_BATTERY_CHARGING: // Not used
		{
// 			driverCallerStates[INTERNAL_BATTERY_CHARGING] = onoff;
		}
		break;
		
		case TRANSMITTER:
		{
			driverCallerStates[TRANSMITTER] = onoff;
		}
		break;
		
		case INITIALIZE_LS:
		{
			driverCallerStates[INTERNAL_BATTERY_CHARGING] = onoff;
			driverCallerStates[TRANSMITTER] = onoff;
		}
		break;
		
		default:
		break;
	}
	
	if(!driverCallerStates[INTERNAL_BATTERY_CHARGING] && !driverCallerStates[TRANSMITTER])
	{
		fet_driver(OFF);
		return OFF;
	}

	fet_driver(ON);
	return(ON);
}


static volatile bool chargeLScallerStates[NUMBER_OF_LS_CONTROLLERS] = {OFF, OFF};
/**
 State machine to keep track of multiple controllers of the external battery load switch. This ensures that the switch is ON if any of the controllers has turned it on.
 */
bool setExtBatLoadSwitch(bool onoff, hardwareResourceClients sender)
{
	
	switch(sender)
	{
		case INTERNAL_BATTERY_CHARGING:
		{
			chargeLScallerStates[INTERNAL_BATTERY_CHARGING] = onoff;
		}
		break;
		
		case TRANSMITTER:
		{
			chargeLScallerStates[TRANSMITTER] = onoff;
		}
		break;
		
		case INITIALIZE_LS:
		{
			chargeLScallerStates[INTERNAL_BATTERY_CHARGING] = onoff;
			chargeLScallerStates[TRANSMITTER] = onoff;
		}
		break;
		
		default:
		break;
	}
	
	if(!chargeLScallerStates[INTERNAL_BATTERY_CHARGING] && !chargeLScallerStates[TRANSMITTER])
	{
		setExtBatLSEnable(OFF);
		return OFF;
	}

	setExtBatLSEnable(ON);
	return(ON);
}

static volatile bool SignalGeneratorCallerStates[NUMBER_OF_LS_CONTROLLERS] = {OFF, OFF};
/**
 State machine to keep track of multiple controllers of the signal generator (VDD). This ensures that VDD is ON if any of the controllers has turned it on.
 */
bool setSignalGeneratorEnable(bool onoff, hardwareResourceClients sender)
{	
	switch(sender)
	{
		case INTERNAL_BATTERY_CHARGING:
		{
			SignalGeneratorCallerStates[INTERNAL_BATTERY_CHARGING] = onoff;
		}
		break;
		
		case TRANSMITTER:
		{
			SignalGeneratorCallerStates[TRANSMITTER] = onoff;
		}
		break;
		
		case INITIALIZE_LS:
		{
			SignalGeneratorCallerStates[INTERNAL_BATTERY_CHARGING] = onoff;
			SignalGeneratorCallerStates[TRANSMITTER] = onoff;
		}
		break;
		
		default:
		break;
	}
	
	if(!SignalGeneratorCallerStates[INTERNAL_BATTERY_CHARGING] && !SignalGeneratorCallerStates[TRANSMITTER])
	{
		v3V3_enable(OFF);
		return OFF;
	}

	v3V3_enable(ON);
	return(ON);
}


/**
 State machine to keep track of multiple controllers of the boost regulator. This ensures that the resource is ON if any of the clients has turned it on.
 */
bool setBoostEnable(bool onoff)
{		
	boost_enable(onoff);
	return(onoff);
}

	
void fet_driver(bool state)
{
	if(state == ON)
	{
		PORTA_set_pin_level(FET_DRIVER_ENABLE, HIGH);
		driverCallerStates[TRANSMITTER] = ON;
	}
	else
	{
		PORTA_set_pin_level(FET_DRIVER_ENABLE, LOW);
		driverCallerStates[TRANSMITTER] = OFF;
	}
}
	
void setExtBatLSEnable(bool state)
{
	if(state == ON)
	{
		PORTA_set_pin_level(CHARGE_AUX_ENABLE, HIGH);
	}
	else
	{
		PORTA_set_pin_level(CHARGE_AUX_ENABLE, LOW);
	}
}

void v3V3_enable(bool state)
{
	if(state == ON)
	{
		PORTA_set_pin_level(V3V3_PWR_ENABLE, HIGH);
	}
	else
	{
		PORTA_set_pin_level(V3V3_PWR_ENABLE, LOW);
	}
}

void boost_enable(bool state)
{
	if(state == ON)
	{
		PORTA_set_pin_level(BOOST_PWR_ENABLE, HIGH);
	}
	else
	{
		PORTA_set_pin_level(BOOST_PWR_ENABLE, LOW);
	}
}

