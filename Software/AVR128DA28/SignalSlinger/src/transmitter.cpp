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
#include "globals.h"

static const uint8_t TX_POWER_ON_INIT_RETRY_COUNT = 10;
static const uint8_t TX_CONTROL_RETRY_COUNT = 5;
static const uint16_t TX_POWER_SETTLE_DELAY_MS = 100;

static volatile bool g_tx_initialized = false;
volatile Frequency_Hz g_80m_frequency = EEPROM_FREQUENCY_DEFAULT;
volatile uint16_t g_80m_power_level_mW = EEPROM_TX_80M_POWER_MW_DEFAULT;
volatile Frequency_Hz g_rtty_offset = EEPROM_RTTY_OFFSET_FREQUENCY_DEFAULT;

static volatile bool g_drain_voltage_enabled = false;
static volatile bool g_transmitter_keyed = false;
static volatile bool g_disable_transmissions = false;

volatile bool g_enable_boost_regulator = false;

/*
 * Runtime role select for the shared auxiliary switch:
 * - HW_TARGET_3_5 defined: battery control and fan control are separate outputs,
 *   so this flag affects only the external-battery/charging path.
 * - HW_TARGET_3_5 not defined: the board has one shared auxiliary switched output;
 *   true routes that output to external-battery/charging support, false repurposes
 *   the same output for temperature-controlled fan drive.
 */
volatile bool g_enable_external_battery_control = true;

/**
 */
bool init_transmitter(bool leave_clock_off);
bool init_transmitter(Frequency_Hz freq, bool leave_clock_off);

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

	if(!freq)
		return (err);

	if(!g_tx_initialized)
	{
		g_80m_frequency = *freq;
		return false;
	}

	if((*freq < TX_MAXIMUM_FREQUENCY) && (*freq > TX_MINIMUM_FREQUENCY)) /* 80m */
	{
		if(!si5351_set_freq(*freq, TX_CLOCK_HF_0, leaveClockOff))
		{
			g_80m_frequency = *freq;
			err = false;
		}
	}

	return (err);
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
	return (g_disable_transmissions);
}

/* Apply or remove power from the RF chain and related control lines.
 * When enabling, the Si5351 and related peripherals are reinitialized.
 */
bool powerToTransmitter(bool state)
{
	bool success = true;

	/* BAT X 2 disables the internal RF chain but still uses the external-control
	 * switch to power any attached external device with the same event timing. */
	if(g_enable_external_battery_control)
		setExtBatLoadSwitch(state, TRANSMITTER);

	if(g_disable_transmissions)
	{
		si5351_shutdown_comms();
#ifdef HW_TARGET_3_5
#else
		setBoostEnable(OFF);
#endif
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

		setSignalGeneratorEnable(state, TRANSMITTER);

#ifdef HW_TARGET_3_5
#else
		if(g_enable_boost_regulator)
			setBoostEnable(state);
#endif
		setFETDriverLoadSwitch(state, TRANSMITTER);

		if(state)
		{
			int tries = TX_POWER_ON_INIT_RETRY_COUNT;

			util_delay_ms(0);
			while(util_delay_ms(TX_POWER_SETTLE_DELAY_MS))
				;
			while(tries && !init_transmitter(g_80m_frequency, true))
			{
				--tries;
			}

			if(!tries)
			{
				success = false;
			}

			si5351_start_comms();
		}
		else
		{
			g_tx_initialized = false;
			g_transmitter_keyed = false;
		}
	}

	return (success);
}

bool keyTransmitter(bool on)
{
	if(g_tx_initialized)
	{
		int tries = TX_CONTROL_RETRY_COUNT;

		if(on)
		{
			if(!g_transmitter_keyed)
			{
				fet_driver(ON);
				while(--tries && !si5351_clock_enable(TX_CLOCK_HF_0, SI5351_CLK_ENABLED))
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
				while(--tries && !si5351_clock_enable(TX_CLOCK_HF_0, SI5351_CLK_DISABLED))
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

	return (g_transmitter_keyed);
}

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

bool init_transmitter(Frequency_Hz freq, bool leave_clock_off)
{
	g_80m_frequency = freq;
	return init_transmitter(leave_clock_off);
}

bool init_transmitter(bool leave_clock_off)
{
	int tries = TX_CONTROL_RETRY_COUNT;

	while(--tries && si5351_init(SI5351_CRYSTAL_LOAD_6PF, 0))
		; /* Initialize SI5351 */

	if(!tries) // SI5351 initialization failed
	{
		return false;
	}

	tries = TX_CONTROL_RETRY_COUNT;

	while(--tries && !si5351_drive_strength(TX_CLOCK_HF_0, SI5351_DRIVE_2MA))
		; /* Initialize SI5351 */

	if(!tries) // SI5351 drive strength failed
	{
		return false;
	}

	tries = TX_CONTROL_RETRY_COUNT;

	while(--tries && !si5351_clock_enable(TX_CLOCK_HF_0, SI5351_CLK_DISABLED))
		; /* Initialize SI5351 */

	if(!tries) // SI5351 drive strength failed
	{
		return false;
	}
	else
	{
		g_tx_initialized = true;
	}

	tries = TX_CONTROL_RETRY_COUNT;

	while(--tries && (txSetFrequency((Frequency_Hz *)&g_80m_frequency, leave_clock_off)))
		; /* Initialize SI5351 */

	if(!tries) // SI5351 drive strength failed
	{
		return false;
	}

	return true;
}
