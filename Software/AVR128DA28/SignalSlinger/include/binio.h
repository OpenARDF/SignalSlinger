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

#ifndef __BINIO_H__
#define __BINIO_H__

#include "defs.h"

/* PORTA *************************************************************************************/
#ifdef HW_TARGET_3_5
#define COOLING_FAN_ENABLE 7
#else
#define BOOST_PWR_ENABLE 7
#endif

#define STRAIGHTKEY 6 /* Straight key */
#define PADDLE_DAH 5  /* Paddle Dah */
#define PADDLE_DIT 4  /* Paddle Dit */
#define POWER_ENABLE 3
#define V3V3_PWR_ENABLE 2
#define FET_DRIVER_ENABLE 1
/*
 * Hardware topology note:
 * - When HW_TARGET_3_5 is defined, the board has separate switched outputs for
 *   external-battery control (AUX_SWITCH_ENABLE below) and cooling-fan control
 *   (COOLING_FAN_ENABLE above).
 * - When HW_TARGET_3_5 is not defined, the board has only one auxiliary switched
 *   output on this pin. At runtime that shared output is used either for
 *   external-battery/charging support or as a temperature-controlled fan switch.
 */
#define AUX_SWITCH_ENABLE 0

/* PORTC *************************************************************************************/
#define SI5351_SCL 3
#define SI5351_SDA 2
#define SERIAL_RX 1
#define SERIAL_TX 0

/* PORTD *************************************************************************************/
#define unusedD7 7
#define DAC_OUTPUT 6
#define unusedD5 5
#define LED_GREEN 4
#define SWITCH 3
#define LED_RED 2
#define VBAT_EXT 1
#define VBAT_INT 0

/* PORTF *************************************************************************************/
#define unusedF6 6
#define X32KHZ_SQUAREWAVE 0
#define unusedF1 1

enum hardwareResourceClients
{
	INTERNAL_BATTERY_CHARGING,
	TRANSMITTER,
	NUMBER_OF_LS_CONTROLLERS,
	INITIALIZE_LS,
	RE_APPLY_LS_STATE
};

/**
 * Initialize board-level GPIO directions, pull modes, and default output states.
 *
 * This should be called during firmware startup before any higher-level module
 * assumes ownership of LEDs, switches, keyed outputs, or load-switch resources.
 */
void BINIO_init(void);

/**
 * Sample raw input ports and update the debounced input snapshots.
 *
 * The debounce filter uses a short history window, so callers should invoke this
 * periodically from a timer-driven context rather than in a tight polling loop.
 */
void debounce(void);

/**
 * Return the latest debounced snapshot of PORTD input bits.
 *
 * @return Debounced PORTD level bitmap.
 */
uint8_t portDdebouncedVals(void);

/**
 * Qualify the main front-panel switch using the debounce path.
 *
 * The function seeds the debounce history from the current raw port state so it
 * can make a one-shot switch decision without relying on older samples.
 *
 * @return true when the debounced switch input is in the closed/active state.
 */
bool debouncedSwitchIsClosed(void);

/**
 * Request or release the FET-driver load switch on behalf of a firmware client.
 *
 * @param onoff Desired client state.
 * @param sender Client requesting the change.
 * @return The effective shared output state after arbitration.
 */
bool setFETDriverLoadSwitch(bool onoff, hardwareResourceClients sender);

/**
 * Drive the FET-driver enable pin directly.
 *
 * This is the low-level pin-control primitive used by the shared arbitration
 * wrapper above.
 *
 * @param state Desired pin state.
 */
void fet_driver(bool state);

/**
 * Read back the FET-driver enable output state.
 *
 * @return true when the FET-driver enable output is asserted.
 */
bool get_fet_driver(void);

/**
 * Read back the signal-generator 3.3 V enable output state.
 *
 * @return true when the shared 3.3 V rail enable output is asserted.
 */
bool get_V3V3_enable(void);

/**
 * Re-apply the current external-battery load-switch arbitration state.
 *
 * @param client Caller identifier, retained for API compatibility.
 * @return The effective shared output state after arbitration is re-applied.
 */
bool setExtBatLoadSwitch(hardwareResourceClients client);

/**
 * Request or release the external-battery auxiliary switch on behalf of a client.
 *
 * @param onoff Desired client state.
 * @param sender Client requesting the change.
 * @return The effective shared output state after arbitration.
 */
bool setExtBatLoadSwitch(bool onoff, hardwareResourceClients sender);

/**
 * Request or release the signal-generator supply on behalf of a firmware client.
 *
 * @param onoff Desired client state.
 * @param sender Client requesting the change.
 * @return The effective shared output state after arbitration.
 */
bool setSignalGeneratorEnable(bool onoff, hardwareResourceClients sender);

#ifdef HW_TARGET_3_5
/**
 * Drive the dedicated cooling-fan auxiliary output.
 *
 * @param onoff Desired fan-switch state.
 */
void setCoolingFanLSEnable(bool onoff);

/**
 * Read back the dedicated cooling-fan auxiliary output state.
 *
 * @return true when the fan-switch output is asserted.
 */
bool getCoolingFanLSEnable(void);
#else
/**
 * Drive the legacy boost-regulator enable output.
 *
 * @param onoff Desired boost-enable state.
 * @return The same state that was requested.
 */
bool setBoostEnable(bool onoff);
#endif

/**
 * Read back the external-battery auxiliary output state.
 *
 * @return true when the auxiliary switch output is asserted.
 */
bool getExtBatLSEnable(void);

#endif //__BINIO_H__
