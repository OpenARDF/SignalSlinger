/*
 *  MIT License
 *
 *  Copyright (c) 2021 DigitalConfections
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

#ifndef __EEPROMMANAGER_H__
#define __EEPROMMANAGER_H__

#include "defs.h"

#include <stddef.h>
#include <time.h>
#include <avr/eeprom.h>

struct EE_prom
{
	uint16_t eeprom_initialization_flag;
	uint32_t guard4_1;
	time_t event_start_epoch;
	uint32_t guard4_2;
	time_t event_finish_epoch;
	uint32_t guard4_3;
	char pattern_text[MAX_PATTERN_TEXT_LENGTH + 2];
	uint32_t guard4_4;
	char foxoring_pattern_text[MAX_PATTERN_TEXT_LENGTH + 2];
	uint32_t guard4_5;
	char stationID_text[MAX_PATTERN_TEXT_LENGTH + 2];
	uint32_t guard4_6;
	uint8_t unlockCode[MAX_UNLOCK_CODE_LENGTH + 2];
	uint32_t guard4_7;
	Fox_t fox_setting_none;
	uint32_t guard4_8;
	Fox_t fox_setting_classic;
	uint32_t guard4_9;
	Fox_t fox_setting_sprint;
	uint32_t guard4_10;
	Fox_t fox_setting_foxoring;
	uint32_t guard4_11;
	Fox_t fox_setting_blind;
	uint32_t guard4_12;
	uint8_t utc_offset;
	uint32_t guard4_13;
	Frequency_Hz rtty_offset;
	uint32_t guard4_14;
	uint16_t rf_power;
	uint32_t guard4_15;
	uint8_t id_codespeed;
	uint32_t guard4_16;
	uint8_t pattern_codespeed;
	uint32_t guard4_17;
	uint8_t foxoring_pattern_codespeed;
	uint32_t guard4_18;
	int16_t off_air_seconds;
	uint32_t guard4_19;
	int16_t on_air_seconds;
	uint32_t guard4_20;
	int16_t ID_period_seconds;
	uint32_t guard4_21;
	int16_t intra_cycle_delay_time;
	uint32_t guard4_22;
	Event_t event_setting;
	uint32_t guard4_23;
	uint32_t frequency;
	uint32_t guard4_24;
	uint32_t frequency_low;
	uint32_t guard4_25;
	uint32_t frequency_med;
	uint32_t guard4_26;
	uint32_t frequency_high;
	uint32_t guard4_27;
	uint32_t frequency_beacon;
	uint32_t guard4_28;
	bool master_setting;
	uint32_t guard4_29;
	float voltage_threshold;
	uint32_t guard4_30;
	uint16_t clock_calibration;
	uint32_t guard4_31;
	uint8_t days_to_run;
	uint32_t guard4_32;
	uint16_t i2c_failure_count;
	uint32_t guard4_33;
	uint8_t function;
	uint32_t guard4_34;
	uint8_t enable_boost_regulator;
	uint32_t guard4_35;
	uint8_t enable_external_battery_control;
	uint32_t guard4_36;
	uint8_t device_enabled;
};

typedef enum
{
#define EEPROM_OFFSET(field) offsetof(EE_prom, field)
	Eeprom_initialization_flag = EEPROM_OFFSET(eeprom_initialization_flag),
	Guard4_1 = EEPROM_OFFSET(guard4_1),
	Event_start_epoch = EEPROM_OFFSET(event_start_epoch),
	Guard4_2 = EEPROM_OFFSET(guard4_2),
	Event_finish_epoch = EEPROM_OFFSET(event_finish_epoch),
	Guard4_3 = EEPROM_OFFSET(guard4_3),
	Pattern_text = EEPROM_OFFSET(pattern_text),
	Guard4_4 = EEPROM_OFFSET(guard4_4),
	Foxoring_pattern_text = EEPROM_OFFSET(foxoring_pattern_text),
	Guard4_5 = EEPROM_OFFSET(guard4_5),
	StationID_text = EEPROM_OFFSET(stationID_text),
	Guard4_6 = EEPROM_OFFSET(guard4_6),
	UnlockCode = EEPROM_OFFSET(unlockCode),
	Guard4_7 = EEPROM_OFFSET(guard4_7),
	Fox_setting_none = EEPROM_OFFSET(fox_setting_none),
	Guard4_8 = EEPROM_OFFSET(guard4_8),
	Fox_setting_classic = EEPROM_OFFSET(fox_setting_classic),
	Guard4_9 = EEPROM_OFFSET(guard4_9),
	Fox_setting_sprint = EEPROM_OFFSET(fox_setting_sprint),
	Guard4_10 = EEPROM_OFFSET(guard4_10),
	Fox_setting_foxoring = EEPROM_OFFSET(fox_setting_foxoring),
	Guard4_11 = EEPROM_OFFSET(guard4_11),
	Fox_setting_blind = EEPROM_OFFSET(fox_setting_blind),
	Guard4_12 = EEPROM_OFFSET(guard4_12),
	Utc_offset = EEPROM_OFFSET(utc_offset),
	Guard4_13 = EEPROM_OFFSET(guard4_13),
	RTTY_offset = EEPROM_OFFSET(rtty_offset),
	Guard4_14 = EEPROM_OFFSET(guard4_14),
	RF_Power = EEPROM_OFFSET(rf_power),
	Guard4_15 = EEPROM_OFFSET(guard4_15),
	Id_codespeed = EEPROM_OFFSET(id_codespeed),
	Guard4_16 = EEPROM_OFFSET(guard4_16),
	Pattern_Code_Speed = EEPROM_OFFSET(pattern_codespeed),
	Guard4_17 = EEPROM_OFFSET(guard4_17),
	Foxoring_Pattern_Code_Speed = EEPROM_OFFSET(foxoring_pattern_codespeed),
	Guard4_18 = EEPROM_OFFSET(guard4_18),
	Off_Air_Seconds = EEPROM_OFFSET(off_air_seconds),
	Guard4_19 = EEPROM_OFFSET(guard4_19),
	On_Air_Seconds = EEPROM_OFFSET(on_air_seconds),
	Guard4_20 = EEPROM_OFFSET(guard4_20),
	ID_Period_Seconds = EEPROM_OFFSET(ID_period_seconds),
	Guard4_21 = EEPROM_OFFSET(guard4_21),
	Intra_Cycle_Delay_Seconds = EEPROM_OFFSET(intra_cycle_delay_time),
	Guard4_22 = EEPROM_OFFSET(guard4_22),
	Event_setting = EEPROM_OFFSET(event_setting),
	Guard4_23 = EEPROM_OFFSET(guard4_23),
	Frequency = EEPROM_OFFSET(frequency),
	Guard4_24 = EEPROM_OFFSET(guard4_24),
	Frequency_Low = EEPROM_OFFSET(frequency_low),
	Guard4_25 = EEPROM_OFFSET(guard4_25),
	Frequency_Med = EEPROM_OFFSET(frequency_med),
	Guard4_26 = EEPROM_OFFSET(guard4_26),
	Frequency_Hi = EEPROM_OFFSET(frequency_high),
	Guard4_27 = EEPROM_OFFSET(guard4_27),
	Frequency_Beacon = EEPROM_OFFSET(frequency_beacon),
	Guard4_28 = EEPROM_OFFSET(guard4_28),
	Master_setting = EEPROM_OFFSET(master_setting),
	Guard4_29 = EEPROM_OFFSET(guard4_29),
	Voltage_threshold = EEPROM_OFFSET(voltage_threshold),
	Guard4_30 = EEPROM_OFFSET(guard4_30),
	Clock_calibration = EEPROM_OFFSET(clock_calibration),
	Guard4_31 = EEPROM_OFFSET(guard4_31),
	Days_to_run = EEPROM_OFFSET(days_to_run),
	Guard4_32 = EEPROM_OFFSET(guard4_32),
	I2C_failure_count = EEPROM_OFFSET(i2c_failure_count),
	Guard4_33 = EEPROM_OFFSET(guard4_33),
	Function = EEPROM_OFFSET(function),
	Guard4_34 = EEPROM_OFFSET(guard4_34),
	Enable_Boost_Regulator = EEPROM_OFFSET(enable_boost_regulator),
	Guard4_35 = EEPROM_OFFSET(guard4_35),
	Enable_External_Battery_Control = EEPROM_OFFSET(enable_external_battery_control),
	Guard4_36 = EEPROM_OFFSET(guard4_36),
	Device_Enabled = EEPROM_OFFSET(device_enabled)
#undef EEPROM_OFFSET
} EE_var_t;

class EepromManager
{
	/*variables */
  public:
  protected:
  private:
	/*functions */
  public:
	EepromManager() {}

	static const struct EE_prom ee_vars;

	bool initializeEEPROMVars(void);
	bool readNonVols(void);
	void updateEEPROMVar(EE_var_t v, void *val);
	void saveAllEEPROM();

  protected:
  private:
	EepromManager(const EepromManager &c);
	EepromManager &operator=(const EepromManager &c);
}; /*EepromManager */

#endif /*__EEPROMMANAGER_H__ */
