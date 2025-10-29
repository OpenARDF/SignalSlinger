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


#include <string.h>
#include <stdlib.h>
#include <avr/eeprom.h>
#include "transmitter.h"
#include "port.h"
#include "binio.h"
#include "tcb.h"

static volatile bool g_tx_initialized = false;
volatile Frequency_Hz g_80m_frequency = EEPROM_FREQUENCY_DEFAULT;
volatile uint16_t g_80m_power_level_mW = EEPROM_TX_80M_POWER_MW_DEFAULT;
volatile Frequency_Hz g_rtty_offset = EEPROM_RTTY_OFFSET_FREQUENCY_DEFAULT;

static volatile bool g_drain_voltage_enabled = false;
static volatile bool g_transmitter_keyed = false;
static volatile bool g_disable_transmissions = false;

uint16_t g_80m_power_table[16] = DEFAULT_80M_POWER_TABLE;
extern volatile bool g_device_enabled;
volatile bool g_enable_boost_regulator = false;
volatile bool g_enable_external_battery_control = true;

/**
 */
EC init_transmitter(bool leave_clock_off);
EC init_transmitter(Frequency_Hz freq, bool leave_clock_off);

/**
 */
 void shutdown_transmitter(void);
	
/**
 */
 void restart_transmitter(void);


/*
 * This function sets the VFO frequency (CLK0 of the Si5351) based on the intended frequency passed in by the parameter (freq),
 * and the VFO configuration in effect. The VFO  frequency might be above or below the intended  frequency, depending on the VFO
 * configuration setting in effect for the radio band of the frequency.
 * Update the oscillator with a new transmit frequency.  The caller
 * may request that the clock remain disabled after programming.
 */
	bool txSetFrequency(Frequency_Hz *freq, bool leaveClockOff)
	{
		bool err = true;

		if(!freq) return(err);
		
		if(g_tx_initialized)
		{
			if((*freq < TX_MAXIMUM_FREQUENCY) && (*freq > TX_MINIMUM_FREQUENCY))    /* 80m */
			{
				if(!si5351_set_freq(*freq, TX_CLOCK_HF_0, leaveClockOff))
				{
					err = false;
				}
			}
		}

		g_80m_frequency = *freq;
		
// 		si5351_set_freq(*freq, TX_CLOCK_VHF, leaveClockOff); /* Test for quadrature */

		return(err);
	}
	
/* Globally enable or disable all RF output.  When disabled the
 * transmitter is powered down to conserve energy.
 */
	void setDisableTransmissions(bool disabled)
	{
		if(disabled)
		{
			g_disable_transmissions = true;
			powerToTransmitter(OFF);
		}
		else
		{
			g_disable_transmissions = false;
		}
	}
	
	bool getDisableTransmissions(void)
	{
		return(g_disable_transmissions);
	}

/* Apply or remove power from the RF chain and related control lines.
 * When enabling, the Si5351 and related peripherals are reinitialized.
 */
	EC powerToTransmitter(bool state)
	{
		EC result = ERROR_CODE_NO_ERROR;
		
		if(g_disable_transmissions)
		{
			si5351_shutdown_comms();
			setBoostEnable(OFF);
			setFETDriverLoadSwitch(OFF, TRANSMITTER);
			setSignalGeneratorEnable(OFF, TRANSMITTER);
			g_tx_initialized = false;
			g_transmitter_keyed = false;
		}
		else
		{
			if(!state)
			{
				si5351_shutdown_comms();
			}
			
			if(g_enable_external_battery_control) setExtBatLoadSwitch(state, TRANSMITTER);
			setSignalGeneratorEnable(state, TRANSMITTER);
			if(g_enable_boost_regulator) setBoostEnable(state);
			setFETDriverLoadSwitch(state, TRANSMITTER);
						
			if(state)
			{
				int tries = 10;
				
				util_delay_ms(0);
				while(util_delay_ms(100));
				while(tries && (init_transmitter(g_80m_frequency, true) != ERROR_CODE_NO_ERROR))
				{
					--tries;
				}
				
				if(!tries)
				{
					result = ERROR_CODE_RF_OSCILLATOR_ERROR;
				}
				
				si5351_start_comms();
			}
			else
			{
				g_tx_initialized = false;
				g_transmitter_keyed = false;
			}
		}

		return(result);
	}
// 	
// 	void txKeyDown(bool key)
// 	{
// 		if(g_tx_initialized)
// 		{
// 			int tries = 10;
// 			while(--tries && (key != keyTransmitter(key)));
// 		}
// 	}
	
// 	bool txConfirmRFisOff(void)
// 	{
// 		if(g_tx_initialized)
// 		{
// 			int tries = 5;
// 		
// 			while(--tries && (si5351_get_phase(SI5351_CLK0, null) != ERROR_CODE_NO_ERROR)); /* confirm oscillator comms are working */
// 		
// 			if(tries > 0) // oscillator appears to be working
// 			{
// 				tries = 5;
// 				while(--tries && g_transmitter_keyed) // if the transmitter is keyed, key it off
// 				{
// 					keyTransmitter(OFF);
// 				}
// 			}
// 			else // failed to communicate with SI5351
// 			{
// 				tries = 5;
// 				while(--tries && (si5351_clock_enable(TX_CLOCK_HF_0, SI5351_CLK_DISABLED) != ERROR_CODE_NO_ERROR))
// 				{
// 					shutdown_transmitter();
// 					restart_transmitter();
// 				}
// 			
// 				if(tries <= 0) // failed to disable the RF generator
// 				{
// 					tries = 5;
// 				
// 					g_tx_initialized = false;
// 					while(--tries && !g_tx_initialized)
// 					{
// 						init_transmitter(true);
// 						keyTransmitter(OFF);
// 					}
// 				
// 					if(tries < 1)
// 					{
// 						return(true); // assume transmitter might be stuck in key down state
// 					}
// 				}
// 			}
// 		}
// 		
// 		return(g_transmitter_keyed);
// 	}

	
	bool keyTransmitter(bool on)
	{
		if(g_tx_initialized)
		{			
			int tries = 5;
			
			if(on)
			{
				if(!g_transmitter_keyed)
				{
					fet_driver(ON);
					while(--tries && (si5351_clock_enable(TX_CLOCK_HF_0, SI5351_CLK_ENABLED) != ERROR_CODE_NO_ERROR))
					{
						shutdown_transmitter();
						restart_transmitter();
					}
					
					if(tries)
					{
						g_transmitter_keyed = true;
					}
				}
			}
			else
			{
				if(g_transmitter_keyed)
				{
					while(--tries && (si5351_clock_enable(TX_CLOCK_HF_0, SI5351_CLK_DISABLED) != ERROR_CODE_NO_ERROR))
					{
						shutdown_transmitter();
						restart_transmitter();
					}
					
					if(tries)
					{
						g_transmitter_keyed = false;
					}
				}
			}
		}
		else
		{
			g_transmitter_keyed = false;
		}
		
		return(g_transmitter_keyed);
	}

// 	uint16_t txGetPowerMw(void)
// 	{
// 		return( g_80m_power_level_mW);
// 	}
	
	bool txIsInitialized(void)
	{
		return g_tx_initialized;
	}

	void shutdown_transmitter(void)
	{
		si5351_shutdown_comms();	
		powerToTransmitter(OFF);
	}
	
	void restart_transmitter(void)
	{
		powerToTransmitter(ON);
		si5351_start_comms();
	}
	
	EC init_transmitter(Frequency_Hz freq, bool leave_clock_off)
	{
		EC code;
		g_80m_frequency = freq;
		code = init_transmitter(leave_clock_off);
		
		return code;
	}
	
	EC init_transmitter(bool leave_clock_off)
	{
		EC code = ERROR_CODE_NO_ERROR;

		int tries = 5;

		while(--tries && (si5351_init(SI5351_CRYSTAL_LOAD_6PF, 0) != ERROR_CODE_NO_ERROR)); /* Initialize SI5351 */

		if(!tries) // SI5351 initialization failed
		{
			return(ERROR_CODE_RF_OSCILLATOR_ERROR);
		}

// 		if((err = si5351_init(SI5351_CRYSTAL_LOAD_6PF, 0)))
// 		{
// 			return(ERROR_CODE_RF_OSCILLATOR_ERROR);
// 		}

		tries = 5;

		while(--tries && ((code = si5351_drive_strength(TX_CLOCK_HF_0, SI5351_DRIVE_2MA)) != ERROR_CODE_NO_ERROR)); /* Initialize SI5351 */

		if(!tries) // SI5351 drive strength failed
		{
			return(ERROR_CODE_RF_OSCILLATOR_ERROR);
		}

// 		if((code = si5351_drive_strength(TX_CLOCK_HF_0, SI5351_DRIVE_2MA)))
// 		{
// 			return( code);
// 		}
		

		tries = 5;

		while(--tries && ((code = si5351_clock_enable(TX_CLOCK_HF_0, SI5351_CLK_DISABLED)) != ERROR_CODE_NO_ERROR)); /* Initialize SI5351 */

		if(!tries) // SI5351 drive strength failed
		{
			return(ERROR_CODE_RF_OSCILLATOR_ERROR);
		}
		else
		{
			g_tx_initialized = true;
		}

// 		if((code = si5351_clock_enable(TX_CLOCK_HF_0, SI5351_CLK_DISABLED)))
// 		{
// 			return( code);
// 		}
		
// 		if((code = si5351_set_phase(TX_CLOCK_HF_0, 50)))
// 		{
// 			return( code);
// 		}
// 
// 		if((code = si5351_drive_strength(TX_CLOCK_VHF, SI5351_DRIVE_2MA)))
// 		{
// 			return( code);
// 		}
// 
// 		if((code = si5351_clock_enable(TX_CLOCK_VHF, SI5351_CLK_DISABLED)))
// 		{
// 			return( code);
// 		}
// 
// 		if((code = si5351_set_phase(TX_CLOCK_VHF, 0)))
// 		{
// 			return( code);
// 		}
// 

		tries = 5;

		while(--tries && (txSetFrequency((Frequency_Hz*)&g_80m_frequency, leave_clock_off))); /* Initialize SI5351 */

		if(!tries) // SI5351 drive strength failed
		{
			return(ERROR_CODE_RF_OSCILLATOR_ERROR);
		}

// 		err = txSetFrequency((Frequency_Hz*)&g_80m_frequency, leave_clock_off);
// 		if(!err)
// 		{
// 			g_tx_initialized = true;
// 		}

		return( code);
	}
