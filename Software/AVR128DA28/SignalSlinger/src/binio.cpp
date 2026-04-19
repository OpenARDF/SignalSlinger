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
 * Board-level digital I/O and shared hardware-resource control.
 *
 * This module contains support functions for:
 * - configuring GPIO directions, pull-ups, and default output states
 * - debouncing sampled switch inputs
 * - arbitrating shared load-switch style resources between firmware clients
 *
 * Higher-level event logic and policy decisions belong elsewhere.
 */

#include "binio.h"
#include "port.h"
#include "defs.h"
#include "atmel_start_pins.h"
#include "adc.h"
#include "globals.h"

/* Three-sample histories and debounced snapshots for the polled input ports. */
uint8_t portDpinReadings[3];
uint8_t portDdebounced;
uint8_t portApinReadings[3];
uint8_t portAdebounced;

/* Low-level output helpers used internally by the shared arbitration wrappers. */
void v3V3_enable(bool state);
void setExtBatLSEnable(bool state);
#ifdef HW_TARGET_3_5
#else
void boost_enable(bool state);
#endif

/**
 * Sample raw input ports and update the debounced input snapshots.
 *
 * The debounce filter uses three successive samples to suppress brief input
 * chatter while still allowing the foreground loop to poll inexpensive GPIO.
 */
void debounce(void)
{
	/* Shift the existing raw samples so the newest reading can be inserted at index 0. */
	portDpinReadings[2] = portDpinReadings[1];
	portDpinReadings[1] = portDpinReadings[0];

	portApinReadings[2] = portApinReadings[1];
	portApinReadings[1] = portApinReadings[0];

	/* Capture the most recent raw port levels. */
	portDpinReadings[0] = PORTD_get_port_level();
	portApinReadings[0] = PORTA_get_port_level();

	/* Update debounced bits only when the recent samples consistently agree. */
	portDdebounced = portDdebounced ^ ((portDdebounced ^ portDpinReadings[0]) & (portDdebounced ^ portDpinReadings[1]) & (portDdebounced ^ portDpinReadings[2]));

	portAdebounced = portAdebounced ^ ((portAdebounced ^ portApinReadings[0]) & (portAdebounced ^ portApinReadings[1]) & (portAdebounced ^ portApinReadings[2]));
}

/**
 * Return the latest debounced snapshot of PORTD input bits.
 *
 * @return Debounced PORTD level bitmap.
 */
uint8_t portDdebouncedVals(void)
{
	return portDdebounced;
}

/**
 * Qualify the main front-panel switch using the debounce path.
 *
 * The function seeds the debounce history from the current raw port state so it
 * can make a one-shot switch decision without relying on older samples.
 *
 * @return true when the debounced switch input is in the closed/active state.
 */
bool debouncedSwitchIsClosed(void)
{
	uint8_t current_portd = PORTD_get_port_level();
	uint8_t current_porta = PORTA_get_port_level();

	/* Seed the existing debounce history from the current raw levels so a
	 * one-shot switch qualification is not polluted by stale runtime samples.
	 */
	portDpinReadings[0] = current_portd;
	portDpinReadings[1] = current_portd;
	portDpinReadings[2] = current_portd;
	portDdebounced = current_portd;

	portApinReadings[0] = current_porta;
	portApinReadings[1] = current_porta;
	portApinReadings[2] = current_porta;
	portAdebounced = current_porta;

	debounce();
	return (!(portDdebounced & (1 << SWITCH)));
}

/**
 * Initialize board-level GPIO directions, pull modes, and default output states.
 *
 * This should be called during firmware startup before any higher-level module
 * assumes ownership of LEDs, switches, keyed outputs, or load-switch resources.
 */
void BINIO_init(void)
{
	/* PORTA *************************************************************************************/
	PORTA_set_pin_dir(AUX_SWITCH_ENABLE, PORT_DIR_OUT);
	PORTA_set_pin_level(AUX_SWITCH_ENABLE, HIGH);

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

#ifdef HW_TARGET_3_5
	PORTA_set_pin_dir(COOLING_FAN_ENABLE, PORT_DIR_OUT);
	PORTA_set_pin_level(COOLING_FAN_ENABLE, LOW);
#else
	PORTA_set_pin_dir(BOOST_PWR_ENABLE, PORT_DIR_OUT);
	PORTA_set_pin_level(BOOST_PWR_ENABLE, LOW);
#endif

	/* PORTC *************************************************************************************/

	PORTC_set_pin_dir(SERIAL_TX, PORT_DIR_OUT);
	PORTC_set_pin_dir(SERIAL_RX, PORT_DIR_IN);
	PORTC_pin_set_isc(SERIAL_RX, PORT_ISC_INTDISABLE_gc);

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

/* Per-client state for the FET-driver shared load switch. */
static volatile bool driverCallerStates[NUMBER_OF_LS_CONTROLLERS] = {OFF, OFF};

/**
 * Record the most recent requested state for a shared hardware client.
 *
 * @param callerStates Per-client state array for the resource being arbitrated.
 * @param onoff Desired state for the caller.
 * @param sender Caller whose slot should be updated.
 * @param trackInternalBatteryCharging Whether the internal-battery slot is valid for this resource.
 */
static void updateLoadSwitchCallerState(volatile bool *callerStates,
                                        bool onoff,
                                        hardwareResourceClients sender,
                                        bool trackInternalBatteryCharging)
{
	switch(sender)
	{
		case INTERNAL_BATTERY_CHARGING:
		{
			if(trackInternalBatteryCharging)
			{
				callerStates[INTERNAL_BATTERY_CHARGING] = onoff;
			}
		}
		break;

		case TRANSMITTER:
		{
			callerStates[TRANSMITTER] = onoff;
		}
		break;

		case INITIALIZE_LS:
		{
			callerStates[INTERNAL_BATTERY_CHARGING] = onoff;
			callerStates[TRANSMITTER] = onoff;
		}
		break;

		default:
			break;
	}
}

/**
 * Report whether any tracked client currently requires a shared resource.
 *
 * @param callerStates Per-client state array for the resource being arbitrated.
 * @return true when at least one client is enabled.
 */
static bool anyLoadSwitchCallerEnabled(const volatile bool *callerStates)
{
	return callerStates[INTERNAL_BATTERY_CHARGING] || callerStates[TRANSMITTER];
}

/**
 * Update a caller's request and return the resulting arbitrated resource state.
 *
 * @param callerStates Per-client state array for the resource being arbitrated.
 * @param onoff Desired state for the caller.
 * @param sender Caller whose slot should be updated.
 * @param trackInternalBatteryCharging Whether the internal-battery slot is valid for this resource.
 * @return true when the resource should remain enabled after arbitration.
 */
static bool getArbitratedLoadSwitchState(volatile bool *callerStates,
                                         bool onoff,
                                         hardwareResourceClients sender,
                                         bool trackInternalBatteryCharging)
{
	updateLoadSwitchCallerState(callerStates, onoff, sender, trackInternalBatteryCharging);
	return anyLoadSwitchCallerEnabled(callerStates);
}

/**
 * Request or release the FET-driver load switch on behalf of a firmware client.
 *
 * @param onoff Desired client state.
 * @param sender Client requesting the change.
 * @return The effective shared output state after arbitration.
 */
bool setFETDriverLoadSwitch(bool onoff, hardwareResourceClients sender)
{
	bool state = getArbitratedLoadSwitchState(driverCallerStates, onoff, sender, false);
	fet_driver(state);
	return state;
}

/* Per-client state for the external-battery auxiliary switch. */
static volatile bool chargeLScallerStates[NUMBER_OF_LS_CONTROLLERS] = {OFF, OFF};

/**
 * Re-apply the current external-battery load-switch arbitration state.
 *
 * @param client Caller identifier, retained for API compatibility.
 * @return The effective shared output state after arbitration is re-applied.
 */
bool setExtBatLoadSwitch(hardwareResourceClients client)
{
	(void)client;
	return setExtBatLoadSwitch(OFF, RE_APPLY_LS_STATE);
}

/**
 * Request or release the external-battery auxiliary switch on behalf of a client.
 *
 * @param onoff Desired client state.
 * @param sender Client requesting the change.
 * @return The effective shared output state after arbitration.
 */
bool setExtBatLoadSwitch(bool onoff, hardwareResourceClients sender)
{
	bool state = getArbitratedLoadSwitchState(chargeLScallerStates, onoff, sender, true);
	setExtBatLSEnable(state);
	return state;
}

/* Per-client state for the signal-generator supply rail. */
static volatile bool SignalGeneratorCallerStates[NUMBER_OF_LS_CONTROLLERS] = {OFF, OFF};

/**
 * Request or release the signal-generator supply on behalf of a firmware client.
 *
 * @param onoff Desired client state.
 * @param sender Client requesting the change.
 * @return The effective shared output state after arbitration.
 */
bool setSignalGeneratorEnable(bool onoff, hardwareResourceClients sender)
{
	bool state = getArbitratedLoadSwitchState(SignalGeneratorCallerStates, onoff, sender, true);
	v3V3_enable(state);
	return state;
}

#ifdef HW_TARGET_3_5
#else
/**
 * Drive the legacy boost-regulator enable output.
 *
 * @param onoff Desired boost-enable state.
 * @return The same state that was requested.
 */
bool setBoostEnable(bool onoff)
{
	boost_enable(onoff);
	return (onoff);
}
#endif

/**
 * Drive the FET-driver enable pin directly.
 *
 * This is the low-level pin-control primitive used by the shared arbitration
 * wrapper above.
 *
 * @param state Desired pin state.
 */
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

/**
 * Read back the FET-driver enable output state.
 *
 * @return true when the FET-driver enable output is asserted.
 */
bool get_fet_driver(void)
{
	return (PORTA_get_pin_level(FET_DRIVER_ENABLE) != LOW);
}

/* Historical name retained: on legacy non-HW_TARGET_3_5 hardware this drives the
 * shared auxiliary switch, which may be assigned at runtime to battery support or fan control. */
/**
 * Drive the external-battery auxiliary switch output directly.
 *
 * @param state Desired auxiliary-switch state.
 */
void setExtBatLSEnable(bool state)
{
	if(state == ON)
	{
		PORTA_set_pin_level(AUX_SWITCH_ENABLE, HIGH);
	}
	else
	{
		PORTA_set_pin_level(AUX_SWITCH_ENABLE, LOW);
	}
}

/**
 * Read back the external-battery auxiliary output state.
 *
 * @return true when the auxiliary switch output is asserted.
 */
bool getExtBatLSEnable(void)
{
	return (PORTA_get_pin_level(AUX_SWITCH_ENABLE) != LOW);
}

/**
 * Drive the signal-generator 3.3 V enable output directly.
 *
 * @param state Desired rail-enable state.
 */
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

/**
 * Read back the signal-generator 3.3 V enable output state.
 *
 * @return true when the shared 3.3 V rail enable output is asserted.
 */
bool get_V3V3_enable(void)
{
	return (PORTA_get_pin_level(V3V3_PWR_ENABLE) != LOW);
}

#ifdef HW_TARGET_3_5
/**
 * Drive the dedicated cooling-fan auxiliary output.
 *
 * @param state Desired fan-switch state.
 */
void setCoolingFanLSEnable(bool state)
{
	if(state == ON)
	{
		PORTA_set_pin_level(COOLING_FAN_ENABLE, ON);
	}
	else
	{
		PORTA_set_pin_level(COOLING_FAN_ENABLE, OFF);
	}
}

/**
 * Read back the dedicated cooling-fan auxiliary output state.
 *
 * @return true when the fan-switch output is asserted.
 */
bool getCoolingFanLSEnable(void)
{
	return (PORTA_get_pin_level(COOLING_FAN_ENABLE) != LOW);
}

#else
/**
 * Drive the legacy boost-regulator enable output directly.
 *
 * @param state Desired boost-enable state.
 */
void boost_enable(bool state)
{
	if(state == ON)
	{
		PORTA_set_pin_level(BOOST_PWR_ENABLE, ON);
	}
	else
	{
		PORTA_set_pin_level(BOOST_PWR_ENABLE, OFF);
	}
}
#endif
