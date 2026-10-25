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
 * EEPROM persistence support for firmware configuration.
 *
 * This module defines the EEPROM layout and the helper that synchronizes
 * runtime configuration values between EEPROM and the shared global state.
 *
 * Low-level EEPROM register access lives here; higher-level event logic and
 * hardware behavior continue to live in their own feature modules.
 */

#ifndef __EEPROMMANAGER_H__
#define __EEPROMMANAGER_H__

#include "defs.h"

#include <stddef.h>
#include <time.h>
#include <avr/eeprom.h>

/**
 * Declare the on-device EEPROM layout used by the firmware.
 *
 * The field order defines the persistent storage offsets, so the struct serves
 * as both documentation and the canonical layout for the matching enum below.
 * The compact v2 layout contains only active settings. Fixed regions following
 * it hold session history and hardware fault records, so later additions must
 * not grow this structure without an explicit EEPROM map migration.
 */
struct __attribute__((packed)) EE_prom
{
	uint16_t eeprom_initialization_flag;
	time_t event_start_epoch;
	time_t event_finish_epoch;
	char pattern_text[MAX_PATTERN_TEXT_LENGTH + 2];
	char foxoring_pattern_text[MAX_PATTERN_TEXT_LENGTH + 2];
	char stationID_text[MAX_PATTERN_TEXT_LENGTH + 2];
	Fox_t fox_setting_classic;
	Fox_t fox_setting_sprint;
	Fox_t fox_setting_foxoring;
	uint8_t utc_offset;
	uint16_t rf_power;
	uint8_t id_codespeed;
	uint8_t pattern_codespeed;
	uint8_t foxoring_pattern_codespeed;
	int16_t off_air_seconds;
	int16_t on_air_seconds;
	int16_t ID_period_seconds;
	int16_t intra_cycle_delay_time;
	Event_t event_setting;
	uint32_t frequency;
	uint32_t frequency_low;
	uint32_t frequency_med;
	uint32_t frequency_high;
	uint32_t frequency_beacon;
	float voltage_threshold;
	uint16_t clock_calibration;
	int32_t si5351_correction_ppb;
	uint8_t days_to_run;
	int8_t thermal_shutdown_threshold;
	uint8_t thermal_shutdown_enabled_marker;
	float hottest_ever_temperature;
	uint8_t function;
	uint8_t enable_boost_regulator;
	uint8_t enable_external_battery_control;
	uint8_t device_enabled;
	uint32_t calibration_frequency_hz;
};

/**
 * Identify EEPROM variables by byte offset within EE_prom.
 *
 * Each enum value is derived from offsetof(EE_prom, field) so callers can use
 * a symbolic name when reading or writing EEPROM without manually maintaining
 * a separate list of numeric offsets.
 */
typedef enum
{
#define EEPROM_OFFSET(field) offsetof(EE_prom, field)
	Eeprom_initialization_flag = EEPROM_OFFSET(eeprom_initialization_flag),
	Event_start_epoch = EEPROM_OFFSET(event_start_epoch),
	Event_finish_epoch = EEPROM_OFFSET(event_finish_epoch),
	Pattern_text = EEPROM_OFFSET(pattern_text),
	Foxoring_pattern_text = EEPROM_OFFSET(foxoring_pattern_text),
	StationID_text = EEPROM_OFFSET(stationID_text),
	Fox_setting_classic = EEPROM_OFFSET(fox_setting_classic),
	Fox_setting_sprint = EEPROM_OFFSET(fox_setting_sprint),
	Fox_setting_foxoring = EEPROM_OFFSET(fox_setting_foxoring),
	Utc_offset = EEPROM_OFFSET(utc_offset),
	RF_Power = EEPROM_OFFSET(rf_power),
	Id_codespeed = EEPROM_OFFSET(id_codespeed),
	Pattern_Code_Speed = EEPROM_OFFSET(pattern_codespeed),
	Foxoring_Pattern_Code_Speed = EEPROM_OFFSET(foxoring_pattern_codespeed),
	Off_Air_Seconds = EEPROM_OFFSET(off_air_seconds),
	On_Air_Seconds = EEPROM_OFFSET(on_air_seconds),
	ID_Period_Seconds = EEPROM_OFFSET(ID_period_seconds),
	Intra_Cycle_Delay_Seconds = EEPROM_OFFSET(intra_cycle_delay_time),
	Event_setting = EEPROM_OFFSET(event_setting),
	Frequency = EEPROM_OFFSET(frequency),
	Frequency_Low = EEPROM_OFFSET(frequency_low),
	Frequency_Med = EEPROM_OFFSET(frequency_med),
	Frequency_Hi = EEPROM_OFFSET(frequency_high),
	Frequency_Beacon = EEPROM_OFFSET(frequency_beacon),
	Voltage_threshold = EEPROM_OFFSET(voltage_threshold),
	Clock_calibration = EEPROM_OFFSET(clock_calibration),
	Si5351_Correction = EEPROM_OFFSET(si5351_correction_ppb),
	Days_to_run = EEPROM_OFFSET(days_to_run),
	Thermal_Shutdown_Threshold = EEPROM_OFFSET(thermal_shutdown_threshold),
	Thermal_Shutdown_Enabled_Marker = EEPROM_OFFSET(thermal_shutdown_enabled_marker),
	Hottest_Ever_Temperature = EEPROM_OFFSET(hottest_ever_temperature),
	Function = EEPROM_OFFSET(function),
	Enable_Boost_Regulator = EEPROM_OFFSET(enable_boost_regulator),
	Enable_External_Battery_Control = EEPROM_OFFSET(enable_external_battery_control),
	Device_Enabled = EEPROM_OFFSET(device_enabled),
	Calibration_Frequency = EEPROM_OFFSET(calibration_frequency_hz)
#undef EEPROM_OFFSET
} EE_var_t;

/* Existing settings never move within EEPROM. New settings must be appended
 * inside the reserved settings region rather than inserted between these
 * fields; changing any assertion requires an explicit map migration. */
static_assert(Eeprom_initialization_flag == 0, "EEPROM initialization marker moved");
static_assert(Event_start_epoch == 2 && Event_finish_epoch == 6, "EEPROM schedule fields moved");
static_assert(Pattern_text == 10 && Foxoring_pattern_text == 32 && StationID_text == 54,
              "EEPROM text fields moved");
static_assert(Fox_setting_classic == 76 && Fox_setting_sprint == 77 && Fox_setting_foxoring == 78,
              "EEPROM fox settings moved");
static_assert(Event_setting == 93, "EEPROM event setting moved");
static_assert(Frequency == 94 && Frequency_Low == 98 && Frequency_Med == 102 &&
                  Frequency_Hi == 106 && Frequency_Beacon == 110,
              "EEPROM frequency settings moved");
static_assert(Voltage_threshold == 114 && Clock_calibration == 118 && Si5351_Correction == 120,
              "EEPROM calibration settings moved");
static_assert(Days_to_run == 124 && Thermal_Shutdown_Threshold == 125 &&
                  Thermal_Shutdown_Enabled_Marker == 126 && Hottest_Ever_Temperature == 127,
              "EEPROM safety settings moved");
static_assert(Function == 131 && Enable_Boost_Regulator == 132 &&
                  Enable_External_Battery_Control == 133 && Device_Enabled == 134,
              "EEPROM operating settings moved");
static_assert(Calibration_Frequency == 135, "EEPROM calibration frequency moved");

/**
 * Load, initialize, and save the firmware's persisted configuration values.
 *
 * The manager bridges between the EEPROM layout above and the runtime globals
 * used by the rest of the firmware.
 */
class EepromManager
{
	/*variables */
  public:
  protected:
  private:
	/*functions */
  public:
	EepromManager() {}

	/* Default EEPROM image used to define the stored layout and erased defaults. */
	static const struct EE_prom ee_vars;

	/**
	 * Initialize EEPROM and globals with firmware defaults when storage is blank.
	 *
	 * The function checks the stored initialization flag and, when it is absent,
	 * writes the current firmware defaults to EEPROM and mirrors them into the
	 * runtime globals.
	 *
	 * @return true if EEPROM was freshly initialized, false if it was already valid.
	 */
	bool initializeEEPROMVars(void);

	/**
	 * Load persisted configuration values from EEPROM into runtime globals.
	 *
	 * The function validates the EEPROM initialization flag before loading values.
	 * A false return indicates success; a true return means the EEPROM contents
	 * should be treated as unavailable or uninitialized.
	 *
	 * @return true on failure or uninitialized EEPROM, false on success.
	 */
	bool readNonVols(void);

	/**
	 * Write one persisted value to EEPROM using the correct storage width.
	 *
	 * The caller supplies an EE_var_t offset along with a pointer to the source
	 * value. The function dispatches to the matching byte, word, dword, float,
	 * or string write helper based on the selected variable.
	 *
	 * @param v   EEPROM variable identifier to update.
	 * @param val Pointer to the new value to store.
	 */
	void updateEEPROMVar(EE_var_t v, void *val);

	/**
	 * Save all persisted runtime globals back to EEPROM.
	 *
	 * Each field is written through updateEEPROMVar(), which allows the lower
	 * helpers to skip EEPROM writes when the stored value is already current.
	 */
	void saveAllEEPROM();

  protected:
  private:
	EepromManager(const EepromManager &c);
	EepromManager &operator=(const EepromManager &c);
}; /*EepromManager */

#endif /*__EEPROMMANAGER_H__ */
