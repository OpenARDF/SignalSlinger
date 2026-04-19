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
 * Reserved padding fields intentionally preserve space for future expansion
 * without shifting the offsets of existing persisted values.
 */
struct EE_prom
{
	uint16_t eeprom_initialization_flag;
	uint32_t reserved_1;
	time_t event_start_epoch;
	uint32_t reserved_2;
	time_t event_finish_epoch;
	uint32_t reserved_3;
	char pattern_text[MAX_PATTERN_TEXT_LENGTH + 2];
	uint32_t reserved_4;
	char foxoring_pattern_text[MAX_PATTERN_TEXT_LENGTH + 2];
	uint32_t reserved_5;
	char stationID_text[MAX_PATTERN_TEXT_LENGTH + 2];
	uint32_t reserved_6;
	uint8_t reserved_unlock_code[MAX_UNLOCK_CODE_LENGTH + 2];
	uint32_t reserved_7;
	Fox_t fox_setting_none;
	uint32_t reserved_8;
	Fox_t fox_setting_classic;
	uint32_t reserved_9;
	Fox_t fox_setting_sprint;
	uint32_t reserved_10;
	Fox_t fox_setting_foxoring;
	uint32_t reserved_11;
	Fox_t fox_setting_blind;
	uint32_t reserved_12;
	uint8_t utc_offset;
	uint32_t reserved_13;
	Frequency_Hz reserved_rtty_offset;
	uint32_t reserved_14;
	uint16_t rf_power;
	uint32_t reserved_15;
	uint8_t id_codespeed;
	uint32_t reserved_16;
	uint8_t pattern_codespeed;
	uint32_t reserved_17;
	uint8_t foxoring_pattern_codespeed;
	uint32_t reserved_18;
	int16_t off_air_seconds;
	uint32_t reserved_19;
	int16_t on_air_seconds;
	uint32_t reserved_20;
	int16_t ID_period_seconds;
	uint32_t reserved_21;
	int16_t intra_cycle_delay_time;
	uint32_t reserved_22;
	Event_t event_setting;
	uint32_t reserved_23;
	uint32_t frequency;
	uint32_t reserved_24;
	uint32_t frequency_low;
	uint32_t reserved_25;
	uint32_t frequency_med;
	uint32_t reserved_26;
	uint32_t frequency_high;
	uint32_t reserved_27;
	uint32_t frequency_beacon;
	uint32_t reserved_28;
	bool reserved_master_setting;
	uint32_t reserved_29;
	float voltage_threshold;
	uint32_t reserved_30;
	uint16_t clock_calibration;
	uint32_t reserved_31;
	uint8_t days_to_run;
	uint32_t reserved_32;
	uint16_t reserved_i2c_failure_count;
	uint32_t reserved_33;
	uint8_t function;
	uint32_t reserved_34;
	uint8_t enable_boost_regulator;
	uint32_t reserved_35;
	uint8_t enable_external_battery_control;
	uint32_t reserved_36;
	uint8_t device_enabled;
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
	Reserved_1 = EEPROM_OFFSET(reserved_1),
	Event_start_epoch = EEPROM_OFFSET(event_start_epoch),
	Reserved_2 = EEPROM_OFFSET(reserved_2),
	Event_finish_epoch = EEPROM_OFFSET(event_finish_epoch),
	Reserved_3 = EEPROM_OFFSET(reserved_3),
	Pattern_text = EEPROM_OFFSET(pattern_text),
	Reserved_4 = EEPROM_OFFSET(reserved_4),
	Foxoring_pattern_text = EEPROM_OFFSET(foxoring_pattern_text),
	Reserved_5 = EEPROM_OFFSET(reserved_5),
	StationID_text = EEPROM_OFFSET(stationID_text),
	Reserved_6 = EEPROM_OFFSET(reserved_6),
	Reserved_Unlock_Code = EEPROM_OFFSET(reserved_unlock_code),
	Reserved_7 = EEPROM_OFFSET(reserved_7),
	Fox_setting_none = EEPROM_OFFSET(fox_setting_none),
	Reserved_8 = EEPROM_OFFSET(reserved_8),
	Fox_setting_classic = EEPROM_OFFSET(fox_setting_classic),
	Reserved_9 = EEPROM_OFFSET(reserved_9),
	Fox_setting_sprint = EEPROM_OFFSET(fox_setting_sprint),
	Reserved_10 = EEPROM_OFFSET(reserved_10),
	Fox_setting_foxoring = EEPROM_OFFSET(fox_setting_foxoring),
	Reserved_11 = EEPROM_OFFSET(reserved_11),
	Fox_setting_blind = EEPROM_OFFSET(fox_setting_blind),
	Reserved_12 = EEPROM_OFFSET(reserved_12),
	Utc_offset = EEPROM_OFFSET(utc_offset),
	Reserved_13 = EEPROM_OFFSET(reserved_13),
	Reserved_RTTY_Offset = EEPROM_OFFSET(reserved_rtty_offset),
	Reserved_14 = EEPROM_OFFSET(reserved_14),
	RF_Power = EEPROM_OFFSET(rf_power),
	Reserved_15 = EEPROM_OFFSET(reserved_15),
	Id_codespeed = EEPROM_OFFSET(id_codespeed),
	Reserved_16 = EEPROM_OFFSET(reserved_16),
	Pattern_Code_Speed = EEPROM_OFFSET(pattern_codespeed),
	Reserved_17 = EEPROM_OFFSET(reserved_17),
	Foxoring_Pattern_Code_Speed = EEPROM_OFFSET(foxoring_pattern_codespeed),
	Reserved_18 = EEPROM_OFFSET(reserved_18),
	Off_Air_Seconds = EEPROM_OFFSET(off_air_seconds),
	Reserved_19 = EEPROM_OFFSET(reserved_19),
	On_Air_Seconds = EEPROM_OFFSET(on_air_seconds),
	Reserved_20 = EEPROM_OFFSET(reserved_20),
	ID_Period_Seconds = EEPROM_OFFSET(ID_period_seconds),
	Reserved_21 = EEPROM_OFFSET(reserved_21),
	Intra_Cycle_Delay_Seconds = EEPROM_OFFSET(intra_cycle_delay_time),
	Reserved_22 = EEPROM_OFFSET(reserved_22),
	Event_setting = EEPROM_OFFSET(event_setting),
	Reserved_23 = EEPROM_OFFSET(reserved_23),
	Frequency = EEPROM_OFFSET(frequency),
	Reserved_24 = EEPROM_OFFSET(reserved_24),
	Frequency_Low = EEPROM_OFFSET(frequency_low),
	Reserved_25 = EEPROM_OFFSET(reserved_25),
	Frequency_Med = EEPROM_OFFSET(frequency_med),
	Reserved_26 = EEPROM_OFFSET(reserved_26),
	Frequency_Hi = EEPROM_OFFSET(frequency_high),
	Reserved_27 = EEPROM_OFFSET(reserved_27),
	Frequency_Beacon = EEPROM_OFFSET(frequency_beacon),
	Reserved_28 = EEPROM_OFFSET(reserved_28),
	Reserved_Master_Setting = EEPROM_OFFSET(reserved_master_setting),
	Reserved_29 = EEPROM_OFFSET(reserved_29),
	Voltage_threshold = EEPROM_OFFSET(voltage_threshold),
	Reserved_30 = EEPROM_OFFSET(reserved_30),
	Clock_calibration = EEPROM_OFFSET(clock_calibration),
	Reserved_31 = EEPROM_OFFSET(reserved_31),
	Days_to_run = EEPROM_OFFSET(days_to_run),
	Reserved_32 = EEPROM_OFFSET(reserved_32),
	Reserved_I2C_Failure_Count = EEPROM_OFFSET(reserved_i2c_failure_count),
	Reserved_33 = EEPROM_OFFSET(reserved_33),
	Function = EEPROM_OFFSET(function),
	Reserved_34 = EEPROM_OFFSET(reserved_34),
	Enable_Boost_Regulator = EEPROM_OFFSET(enable_boost_regulator),
	Reserved_35 = EEPROM_OFFSET(reserved_35),
	Enable_External_Battery_Control = EEPROM_OFFSET(enable_external_battery_control),
	Reserved_36 = EEPROM_OFFSET(reserved_36),
	Device_Enabled = EEPROM_OFFSET(device_enabled)
#undef EEPROM_OFFSET
} EE_var_t;

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
