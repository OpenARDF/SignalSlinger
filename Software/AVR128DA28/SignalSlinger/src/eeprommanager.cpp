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
 * This module owns the concrete EEPROM image, the low-level typed read/write
 * helpers, and the routines that load defaults from firmware constants or
 * restore persisted values into runtime globals.
 *
 * It should contain storage and synchronization logic, not higher-level event
 * scheduling or radio behavior.
 */

#include "defs.h"
#include "eeprommanager.h"
#include "serialbus.h"
#include "i2c.h"
#include "transmitter.h"
#include "globals.h"
#include "shared_state.h"
#include <avr/pgmspace.h>
#include <string.h>

/* Canonical EEPROM image used both for layout definition and default contents. */
const struct EE_prom EEMEM EepromManager::ee_vars =
    {
        0x00,                                         // 	uint16_t eeprom_initialization_flag;
        0x00000000,                                   // 	reserved
        0x00000000,                                   // 	time_t event_start_epoch;
        0x00000000,                                   // 	reserved
        0x00000000,                                   //  time_t event_finish_epoch;
        0x00000000,                                   // 	reserved
        "\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0", // 	char pattern_text[MAX_PATTERN_TEXT_LENGTH + 2];
        0x00000000,                                   // 	reserved
        "\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0", // 	char foxoring_pattern_text[MAX_PATTERN_TEXT_LENGTH + 2];
        0x00000000,                                   // 	reserved
        "\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0", // char stationID_text[MAX_PATTERN_TEXT_LENGTH + 2];
        0x00000000,                                   // 	reserved
        "\0\0\0\0\0\0\0\0\0",                         // 	uint8_t reserved_unlock_code[MAX_UNLOCK_CODE_LENGTH + 2];
        0x00000000,                                   // 	reserved
        (Fox_t)0x00,                                  // Fox_t fox_setting_none;
        0x00000000,                                   // 	reserved
        (Fox_t)0x00,                                  // Fox_t fox_setting_classic;
        0x00000000,                                   // 	reserved
        (Fox_t)0x00,                                  // Fox_t fox_setting_sprint;
        0x00000000,                                   // 	reserved
        (Fox_t)0x00,                                  // Fox_t fox_setting_foxoring;
        0x00000000,                                   // 	reserved
        (Fox_t)0x00,                                  // Fox_t fox_setting_blind;
        0x00000000,                                   // 	reserved
        0x00,                                         // 	uint8_t utc_offset;
        0x00000000,                                   // 	reserved
        0x00000000,                                   // 	uint32_t reserved_rtty_offset;
        0x00000000,                                   // 	reserved
        0x0000,                                       // 	uint16_t rf_power;
        0x00000000,                                   // 	reserved
        0x00,                                         // 	uint8_t id_codespeed;
        0x00000000,                                   // 	reserved
        0x00,                                         // 	uint8_t pattern_codespeed;
        0x00000000,                                   // 	reserved
        0x00,                                         // 	uint8_t foxoring_pattern_codespeed;
        0x00000000,                                   // 	reserved
        0x0000,                                       // 	int16_t off_air_seconds;
        0x00000000,                                   // 	reserved
        0x0000,                                       // 	int16_t on_air_seconds;
        0x00000000,                                   // 	reserved
        0x0000,                                       // 	int16_t ID_period_seconds;
        0x00000000,                                   // 	reserved
        0x0000,                                       // 	int16_t intra_cycle_delay_time;
        0x00000000,                                   // 	reserved
        (Event_t)0x00,                                // Event_t event_setting;
        0x00000000,                                   // 	reserved
        0x00000000,                                   // uint32_t frequency;
        0x00000000,                                   // 	reserved
        0x00000000,                                   // uint32_t frequency_low;
        0x00000000,                                   // 	reserved
        0x00000000,                                   //	uint32_t frequency_med;
        0x00000000,                                   // 	reserved
        0x00000000,                                   // uint32_t frequency_hi;
        0x00000000,                                   // 	reserved
        0x00000000,                                   // uint32_t frequency_beacon;
        0x00000000,                                   // 	reserved
        0x00,                                         // 	uint8_t reserved_master_setting;
        0x00000000,                                   // 	reserved
        0x00000000,                                   //  float voltage_threshold
        0x00000000,                                   // 	reserved
        0x0000,                                       //  uint16_t clock_calibration
        0x00000000,                                   //  reserved
        0x00,                                         //  uint8_t days_to_run
        0x00000000,                                   //  reserved
        0x00,                                         // int8_t thermal_shutdown_threshold;
        0x00,                                         // uint8_t reserved_thermal_shutdown_threshold_padding;
        0x00000000,                                   //  float hottest_ever_temperature
        0x00,                                         //  uint8_t function
        0x00000000,                                   //  reserved
        0x00,                                         //  uint8_t enable_boost_regulator
        0x00000000,                                   //  reserved
        0x00,                                         //  uint8_t enable_external_battery_control
        0x00000000,                                   //  reserved
        0x00                                          //  uint8_t device_enabled
};

typedef uint16_t eeprom_addr_t;

/**
 * Write one byte to EEPROM through the AVR mapped EEPROM window.
 *
 * The helper waits for any prior EEPROM operation to finish, enables erase and
 * write for the next access, performs the store, and then clears the command.
 *
 * @param index Byte offset into EEPROM.
 * @param in    Value to store.
 */
void avr_eeprom_write_byte(eeprom_addr_t index, uint8_t in)
{
	while(NVMCTRL.STATUS & NVMCTRL_EEBUSY_bm)
		;
	_PROTECTED_WRITE_SPM(NVMCTRL.CTRLA, NVMCTRL_CMD_EEERWR_gc);
	*(uint8_t *)(eeprom_addr_t)(MAPPED_EEPROM_START + index) = in;
	_PROTECTED_WRITE_SPM(NVMCTRL.CTRLA, NVMCTRL_CMD_NONE_gc);
}

/**
 * Write a 16-bit value to EEPROM through the AVR mapped EEPROM window.
 *
 * @param index Byte offset into EEPROM.
 * @param in    Value to store.
 */
void avr_eeprom_write_word(eeprom_addr_t index, uint16_t in)
{
	while(NVMCTRL.STATUS & NVMCTRL_EEBUSY_bm)
		;
	_PROTECTED_WRITE_SPM(NVMCTRL.CTRLA, NVMCTRL_CMD_EEERWR_gc);
	*(uint16_t *)(eeprom_addr_t)(MAPPED_EEPROM_START + index) = in;
	_PROTECTED_WRITE_SPM(NVMCTRL.CTRLA, NVMCTRL_CMD_NONE_gc);
}

/**
 * Write a 32-bit value to EEPROM through the AVR mapped EEPROM window.
 *
 * @param index Byte offset into EEPROM.
 * @param in    Value to store.
 */
void avr_eeprom_write_dword(eeprom_addr_t index, uint32_t in)
{
	while(NVMCTRL.STATUS & NVMCTRL_EEBUSY_bm)
		;
	_PROTECTED_WRITE_SPM(NVMCTRL.CTRLA, NVMCTRL_CMD_EEERWR_gc);
	*(uint32_t *)(eeprom_addr_t)(MAPPED_EEPROM_START + index) = in;
	_PROTECTED_WRITE_SPM(NVMCTRL.CTRLA, NVMCTRL_CMD_NONE_gc);
}

/**
 * Write a float value to EEPROM through the AVR mapped EEPROM window.
 *
 * @param index Byte offset into EEPROM.
 * @param in    Value to store.
 */
void avr_eeprom_write_float(eeprom_addr_t index, float in)
{
	while(NVMCTRL.STATUS & NVMCTRL_EEBUSY_bm)
		;
	_PROTECTED_WRITE_SPM(NVMCTRL.CTRLA, NVMCTRL_CMD_EEERWR_gc);
	*(float *)(eeprom_addr_t)(MAPPED_EEPROM_START + index) = in;
	_PROTECTED_WRITE_SPM(NVMCTRL.CTRLA, NVMCTRL_CMD_NONE_gc);
}

/**
 * Read one byte from EEPROM at the supplied offset.
 *
 * @param index Byte offset into EEPROM.
 * @return Stored byte value.
 */
static uint8_t avr_eeprom_read_byte_at(eeprom_addr_t index)
{
	return eeprom_read_byte((const uint8_t *)(eeprom_addr_t)index);
}

/**
 * Read a 16-bit value from EEPROM at the supplied offset.
 *
 * @param index Byte offset into EEPROM.
 * @return Stored 16-bit value.
 */
static uint16_t avr_eeprom_read_word_at(eeprom_addr_t index)
{
	return eeprom_read_word((const uint16_t *)(eeprom_addr_t)index);
}

/**
 * Read a 32-bit value from EEPROM at the supplied offset.
 *
 * @param index Byte offset into EEPROM.
 * @return Stored 32-bit value.
 */
static uint32_t avr_eeprom_read_dword_at(eeprom_addr_t index)
{
	return eeprom_read_dword((const uint32_t *)(eeprom_addr_t)index);
}

/**
 * Read a float value from EEPROM at the supplied offset.
 *
 * @param index Byte offset into EEPROM.
 * @return Stored floating-point value.
 */
static float avr_eeprom_read_float_at(eeprom_addr_t index)
{
	return eeprom_read_float((const float *)(eeprom_addr_t)index);
}

/**
 * Write a byte only when the stored EEPROM value differs.
 *
 * Skipping unchanged writes reduces EEPROM wear during routine saves.
 *
 * @param index Byte offset into EEPROM.
 * @param value New byte value to store.
 */
static void avr_eeprom_write_byte_if_changed(eeprom_addr_t index, uint8_t value)
{
	if(value != avr_eeprom_read_byte_at(index))
	{
		avr_eeprom_write_byte(index, value);
	}
}

/**
 * Write a 16-bit value only when the stored EEPROM value differs.
 *
 * @param index Byte offset into EEPROM.
 * @param value New 16-bit value to store.
 */
static void avr_eeprom_write_word_if_changed(eeprom_addr_t index, uint16_t value)
{
	if(value != avr_eeprom_read_word_at(index))
	{
		avr_eeprom_write_word(index, value);
	}
}

/**
 * Write a 32-bit value only when the stored EEPROM value differs.
 *
 * @param index Byte offset into EEPROM.
 * @param value New 32-bit value to store.
 */
static void avr_eeprom_write_dword_if_changed(eeprom_addr_t index, uint32_t value)
{
	if(value != avr_eeprom_read_dword_at(index))
	{
		avr_eeprom_write_dword(index, value);
	}
}

/**
 * Write a float value only when the stored EEPROM value differs.
 *
 * @param index Byte offset into EEPROM.
 * @param value New floating-point value to store.
 */
static void avr_eeprom_write_float_if_changed(eeprom_addr_t index, float value)
{
	if(value != avr_eeprom_read_float_at(index))
	{
		avr_eeprom_write_float(index, value);
	}
}

/**
 * Write a bounded C string to EEPROM only where bytes differ.
 *
 * Characters are written until a null terminator or max_length is reached. The
 * helper also ensures the stored string is explicitly null-terminated when the
 * new value becomes shorter than the previous contents.
 *
 * @param index Byte offset of the first character in EEPROM.
 * @param value Null-terminated string to store.
 * @param max_length Maximum number of non-terminator characters to write.
 */
static void avr_eeprom_write_string_if_changed(eeprom_addr_t index, const char *value, uint8_t max_length)
{
	eeprom_addr_t current_index = index;
	uint8_t count = 0;

	while(value[count] && (count < max_length))
	{
		uint8_t current_char = (uint8_t)value[count];
		if(current_char != avr_eeprom_read_byte_at(current_index))
		{
			avr_eeprom_write_byte(current_index, current_char);
		}

		++count;
		++current_index;
	}

	/* Trim any leftover stored suffix when the new string ends earlier. */
	if(avr_eeprom_read_byte_at(current_index))
	{
		avr_eeprom_write_byte(current_index, 0);
	}
}

/**
 * Read a bounded EEPROM string into a RAM buffer.
 *
 * EEPROM bytes containing 0xFF are treated as erased and converted to a normal
 * null terminator so callers do not inherit unterminated garbage text.
 *
 * @param dst Destination buffer in RAM.
 * @param dst_size Size of the destination buffer in bytes.
 * @param index Byte offset of the first stored character in EEPROM.
 * @param max_length Maximum number of non-terminator characters to read.
 */
static void avr_eeprom_read_string(char *dst, size_t dst_size, eeprom_addr_t index, uint8_t max_length)
{
	if(!dst || !dst_size)
	{
		return;
	}

	size_t limit = MIN(dst_size - 1, (size_t)max_length);
	for(size_t i = 0; i < limit; i++)
	{
		char c = (char)avr_eeprom_read_byte_at(index + (eeprom_addr_t)i);
		/* Treat erased EEPROM bytes as end-of-string instead of exposing 0xFF. */
		if((uint8_t)c == 0xFF)
		{
			c = '\0';
		}

		dst[i] = c;
		if(!c)
		{
			return;
		}
	}

	dst[limit] = '\0';
}

/**
 * Write an initial C string to EEPROM without first comparing existing bytes.
 *
 * This helper is used during first-time EEPROM initialization, when the module
 * is intentionally populating blank storage with firmware defaults.
 *
 * @param index Byte offset of the first character in EEPROM.
 * @param value Null-terminated string to store.
 * @param max_length Maximum number of non-terminator characters to write.
 */
static void avr_eeprom_initialize_string(eeprom_addr_t index, const char *value, uint8_t max_length)
{
	uint8_t count = 0;

	while(value[count] && (count < max_length))
	{
		avr_eeprom_write_byte(index + count, (uint8_t)value[count]);
		++count;
	}

	avr_eeprom_write_byte(index + count, '\0');
}

/**
 * Fill a contiguous EEPROM range with zero bytes.
 *
 * @param index Byte offset of the first byte to clear.
 * @param length Number of bytes to clear.
 */
static void avr_eeprom_write_zero_bytes(eeprom_addr_t index, uint8_t length)
{
	for(uint8_t i = 0; i < length; i++)
	{
		avr_eeprom_write_byte(index + i, 0);
	}
}

/**
 * Read a Fox_t value from EEPROM and clamp it to a valid event-specific range.
 *
 * @param index EEPROM offset containing the stored fox value.
 * @param max_value Highest valid fox value for the current event mode.
 * @return Clamped fox selection.
 */
static Fox_t avr_eeprom_read_clamped_fox(eeprom_addr_t index, Fox_t max_value)
{
	return (Fox_t)CLAMP(BEACON, avr_eeprom_read_byte_at(index), max_value);
}

/**
 * Upgrade known older EEPROM layouts in place to the current version.
 *
 * Version 0x0131 predates both new thermal fields, so migration seeds the
 * thermal shutdown threshold and hottest-ever temperature with defaults.
 * Version 0x0132 already contains the threshold field, so it only needs the
 * hottest-ever value initialized. In both cases the version marker is written
 * last so a partial migration can be retried safely on the next boot.
 *
 * @param initialization_flag Stored EEPROM layout version.
 * @return true when a known prior layout was migrated, false otherwise.
 */
static bool migrateEEPROMLayoutIfNeeded(uint16_t initialization_flag)
{
	if(initialization_flag == EEPROM_INITIALIZED_FLAG_V0131)
	{
		avr_eeprom_write_byte(Thermal_Shutdown_Threshold, (uint8_t)EEPROM_THERMAL_SHUTDOWN_THRESHOLD_DEFAULT);
		avr_eeprom_write_byte(Thermal_Shutdown_Threshold + 1, 0);
		avr_eeprom_write_float(Hottest_Ever_Temperature, EEPROM_PROCESSOR_MAX_EVER_TEMPERATURE_DEFAULT);
		avr_eeprom_write_word(Eeprom_initialization_flag, EEPROM_INITIALIZED_FLAG);
		return true;
	}

	if(initialization_flag == EEPROM_INITIALIZED_FLAG_V0132)
	{
		avr_eeprom_write_float(Hottest_Ever_Temperature, EEPROM_PROCESSOR_MAX_EVER_TEMPERATURE_DEFAULT);
		avr_eeprom_write_word(Eeprom_initialization_flag, EEPROM_INITIALIZED_FLAG);
		return true;
	}

	return false;
}

/**
 * Persist one named runtime value to EEPROM.
 *
 * The enum value identifies both the EEPROM offset and the storage width, so
 * this dispatcher selects the matching typed helper without exposing raw
 * offsets or NVM access details to callers.
 *
 * @param v EEPROM variable to update.
 * @param val Pointer to the source value.
 */
void EepromManager::updateEEPROMVar(EE_var_t v, void *val)
{
	if(!val)
	{
		return;
	}

	switch(v)
	{
		case StationID_text:
		case Pattern_text:
		case Foxoring_pattern_text:
			avr_eeprom_write_string_if_changed((eeprom_addr_t)v, (const char *)val, MAX_PATTERN_TEXT_LENGTH);
			break;

		case Id_codespeed:
		case Pattern_Code_Speed:
		case Foxoring_Pattern_Code_Speed:
		case Fox_setting_none:
		case Fox_setting_classic:
		case Fox_setting_sprint:
		case Fox_setting_foxoring:
		case Fox_setting_blind:
		case Event_setting:
		case Utc_offset:
		case Days_to_run:
		case Thermal_Shutdown_Threshold:
		case Function:
		case Enable_Boost_Regulator:
		case Enable_External_Battery_Control:
		case Device_Enabled:
			avr_eeprom_write_byte_if_changed((eeprom_addr_t)v, *(const uint8_t *)val);
			break;

		case Frequency:
		case Frequency_Low:
		case Frequency_Med:
		case Frequency_Hi:
		case Frequency_Beacon:
		case Event_start_epoch:
		case Event_finish_epoch:
			avr_eeprom_write_dword_if_changed((eeprom_addr_t)v, *(const uint32_t *)val);
			break;

		case RF_Power:
		case Off_Air_Seconds:
		case On_Air_Seconds:
		case ID_Period_Seconds:
		case Intra_Cycle_Delay_Seconds:
		case Eeprom_initialization_flag:
		case Clock_calibration:
			avr_eeprom_write_word_if_changed((eeprom_addr_t)v, *(const uint16_t *)val);
			break;

		case Voltage_threshold:
		case Hottest_Ever_Temperature:
			avr_eeprom_write_float_if_changed((eeprom_addr_t)v, *(const float *)val);
			break;

		default:
			break;
	}
}

/**
 * Save all persisted runtime globals back to EEPROM.
 *
 * Each field is routed through updateEEPROMVar(), which in turn uses the
 * write-if-changed helpers so routine saves avoid unnecessary EEPROM wear.
 */
void EepromManager::saveAllEEPROM(void)
{
	updateEEPROMVar(Id_codespeed, (void *)&g_evteng_id_codespeed);
	updateEEPROMVar(Fox_setting_none, (void *)&g_fox[EVENT_NONE]);
	updateEEPROMVar(Fox_setting_classic, (void *)&g_fox[EVENT_CLASSIC]);
	updateEEPROMVar(Fox_setting_sprint, (void *)&g_fox[EVENT_SPRINT]);
	updateEEPROMVar(Fox_setting_foxoring, (void *)&g_fox[EVENT_FOXORING]);
	updateEEPROMVar(Fox_setting_blind, (void *)&g_fox[EVENT_BLIND_ARDF]);
	updateEEPROMVar(Event_setting, (void *)&g_event);
	updateEEPROMVar(Frequency, (void *)&g_frequency);
	updateEEPROMVar(Frequency_Low, (void *)&g_frequency_low);
	updateEEPROMVar(Frequency_Med, (void *)&g_frequency_med); /* use for sprint slow */
	updateEEPROMVar(Frequency_Hi, (void *)&g_frequency_hi);   /* use for sprint high - will also need sprint spectator and beacon */
	updateEEPROMVar(Frequency_Beacon, (void *)&g_frequency_beacon);
	updateEEPROMVar(Event_start_epoch, (void *)&g_event_start_epoch);
	updateEEPROMVar(Event_finish_epoch, (void *)&g_event_finish_epoch);
	updateEEPROMVar(Utc_offset, (void *)&g_utc_offset);
	updateEEPROMVar(Pattern_text, (void *)g_messages_text[PATTERN_TEXT]);
	updateEEPROMVar(Foxoring_pattern_text, (void *)g_messages_text[FOXORING_PATTERN_TEXT]);
	updateEEPROMVar(StationID_text, (void *)g_messages_text[STATION_ID]);
	updateEEPROMVar(RF_Power, (void *)&g_80m_power_level_mW);
	updateEEPROMVar(Pattern_Code_Speed, (void *)&g_evteng_pattern_codespeed);
	updateEEPROMVar(Foxoring_Pattern_Code_Speed, (void *)&g_foxoring_pattern_codespeed);
	updateEEPROMVar(Off_Air_Seconds, (void *)&g_evteng_off_air_seconds);
	updateEEPROMVar(On_Air_Seconds, (void *)&g_evteng_on_air_seconds);
	updateEEPROMVar(ID_Period_Seconds, (void *)&g_evteng_ID_period_seconds);
	updateEEPROMVar(Intra_Cycle_Delay_Seconds, (void *)&g_evteng_intra_cycle_delay_time);
	updateEEPROMVar(Voltage_threshold, (void *)&g_internal_voltage_low_threshold);
	updateEEPROMVar(Clock_calibration, (void *)&g_clock_calibration);
	updateEEPROMVar(Days_to_run, (void *)&g_days_to_run);
	updateEEPROMVar(Thermal_Shutdown_Threshold, (void *)&g_thermal_shutdown_threshold);
	updateEEPROMVar(Hottest_Ever_Temperature, (void *)&g_processor_max_ever_temperature);
	updateEEPROMVar(Function, (void *)&g_function);
	updateEEPROMVar(Enable_Boost_Regulator, (void *)&g_enable_boost_regulator);
	updateEEPROMVar(Enable_External_Battery_Control, (void *)&g_enable_external_battery_control);
	updateEEPROMVar(Device_Enabled, (void *)&g_device_enabled);
}

/**
 * Load persisted configuration values from EEPROM into the runtime globals.
 *
 * The function succeeds only when the EEPROM initialization flag matches the
 * expected marker. Individual values are clamped or normalized as needed before
 * being published into the shared global state.
 *
 * @return true when EEPROM is uninitialized or invalid, false on success.
 */
bool EepromManager::readNonVols(void)
{
	bool failure = true;
	uint16_t initialization_flag = avr_eeprom_read_word_at(Eeprom_initialization_flag);

	if(initialization_flag == EEPROM_INITIALIZED_FLAG) /* EEPROM is up to date */
	{
		g_isMaster = EEPROM_MASTER_SETTING_DEFAULT;
		g_evteng_id_codespeed = CLAMP(MIN_CODE_SPEED_WPM, avr_eeprom_read_byte_at(Id_codespeed), MAX_CODE_SPEED_WPM);
		g_event = (Event_t)avr_eeprom_read_byte_at(Event_setting);
		g_frequency = CLAMP(TX_MINIMUM_FREQUENCY, avr_eeprom_read_dword_at(Frequency), TX_MAXIMUM_FREQUENCY);
		g_frequency_low = CLAMP(TX_MINIMUM_FREQUENCY, avr_eeprom_read_dword_at(Frequency_Low), TX_MAXIMUM_FREQUENCY);
		g_frequency_med = CLAMP(TX_MINIMUM_FREQUENCY, avr_eeprom_read_dword_at(Frequency_Med), TX_MAXIMUM_FREQUENCY);
		g_frequency_hi = CLAMP(TX_MINIMUM_FREQUENCY, avr_eeprom_read_dword_at(Frequency_Hi), TX_MAXIMUM_FREQUENCY);
		g_frequency_beacon = CLAMP(TX_MINIMUM_FREQUENCY, avr_eeprom_read_dword_at(Frequency_Beacon), TX_MAXIMUM_FREQUENCY);
		g_enable_boost_regulator = (bool)avr_eeprom_read_byte_at(Enable_Boost_Regulator);
		fox_setting_slot_write_atomic(EVENT_NONE, avr_eeprom_read_clamped_fox(Fox_setting_none, SPRINT_F5));
		fox_setting_slot_write_atomic(EVENT_CLASSIC, avr_eeprom_read_clamped_fox(Fox_setting_classic, FOX_5));
		fox_setting_slot_write_atomic(EVENT_SPRINT, avr_eeprom_read_clamped_fox(Fox_setting_sprint, SPRINT_F5));
		fox_setting_slot_write_atomic(EVENT_FOXORING, avr_eeprom_read_clamped_fox(Fox_setting_foxoring, FREQUENCY_TEST_BEACON));
		fox_setting_slot_write_atomic(EVENT_BLIND_ARDF, avr_eeprom_read_clamped_fox(Fox_setting_blind, FOX_5));
		g_event_start_epoch = avr_eeprom_read_dword_at(Event_start_epoch);
		g_event_finish_epoch = avr_eeprom_read_dword_at(Event_finish_epoch);
		g_utc_offset = (int8_t)avr_eeprom_read_byte_at(Utc_offset);

		/* Stage string reads in local buffers before publishing them atomically. */
		char pattern_text_local[MAX_PATTERN_TEXT_LENGTH + 2] = {0};
		char foxoring_pattern_text_local[MAX_PATTERN_TEXT_LENGTH + 2] = {0};
		char station_id_text_local[MAX_PATTERN_TEXT_LENGTH + 2] = {0};
		avr_eeprom_read_string(pattern_text_local, sizeof(pattern_text_local), Pattern_text, MAX_PATTERN_TEXT_LENGTH);
		messages_text_slot_publish_atomic(PATTERN_TEXT, pattern_text_local);

		avr_eeprom_read_string(foxoring_pattern_text_local, sizeof(foxoring_pattern_text_local), Foxoring_pattern_text, MAX_PATTERN_TEXT_LENGTH);
		messages_text_slot_publish_atomic(FOXORING_PATTERN_TEXT, foxoring_pattern_text_local);

		avr_eeprom_read_string(station_id_text_local, sizeof(station_id_text_local), StationID_text, MAX_PATTERN_TEXT_LENGTH);
		messages_text_slot_publish_atomic(STATION_ID, station_id_text_local);

		g_80m_power_level_mW = CLAMP(MIN_RF_POWER_MW, avr_eeprom_read_word_at(RF_Power), MAX_TX_POWER_80M_MW);

		g_evteng_pattern_codespeed = CLAMP(MIN_CODE_SPEED_WPM, avr_eeprom_read_byte_at(Pattern_Code_Speed), MAX_CODE_SPEED_WPM);
		g_foxoring_pattern_codespeed = CLAMP(MIN_CODE_SPEED_WPM, avr_eeprom_read_byte_at(Foxoring_Pattern_Code_Speed), MAX_CODE_SPEED_WPM);

		g_evteng_off_air_seconds = CLAMP(0, (int16_t)avr_eeprom_read_word_at(Off_Air_Seconds), 3600);
		g_evteng_on_air_seconds = CLAMP(0, (int16_t)avr_eeprom_read_word_at(On_Air_Seconds), 3600);
		g_evteng_ID_period_seconds = CLAMP(0, (int16_t)avr_eeprom_read_word_at(ID_Period_Seconds), 3600);
		g_evteng_intra_cycle_delay_time = CLAMP(0, (int16_t)avr_eeprom_read_word_at(Intra_Cycle_Delay_Seconds), 3600);

		g_internal_voltage_low_threshold = CLAMP(3.0, avr_eeprom_read_float_at(Voltage_threshold), 4.1);

		g_clock_calibration = avr_eeprom_read_word_at(Clock_calibration);

		g_days_to_run = avr_eeprom_read_byte_at(Days_to_run);

		g_thermal_shutdown_threshold =
		    CLAMP(THERMAL_SHUTDOWN_THRESHOLD_MIN_C,
		          (int8_t)avr_eeprom_read_byte_at(Thermal_Shutdown_Threshold),
		          THERMAL_SHUTDOWN_THRESHOLD_MAX_C);

		float hottest_ever_temperature = avr_eeprom_read_float_at(Hottest_Ever_Temperature);
		g_processor_max_ever_temperature =
		    (isValidTemp(hottest_ever_temperature) && (hottest_ever_temperature >= EEPROM_PROCESSOR_MAX_EVER_TEMPERATURE_DEFAULT))
		        ? hottest_ever_temperature
		        : EEPROM_PROCESSOR_MAX_EVER_TEMPERATURE_DEFAULT;

		g_function = (Function_t)avr_eeprom_read_byte_at(Function);

		g_enable_external_battery_control = (bool)avr_eeprom_read_byte_at(Enable_External_Battery_Control);

		g_device_enabled = (bool)avr_eeprom_read_byte_at(Device_Enabled);

		failure = false;
	}

	return (failure);
}

/**
 * Populate blank EEPROM with firmware defaults and mirror them into RAM.
 *
 * The function runs only when the EEPROM initialization flag is missing. After
 * writing defaults, it stores the marker so later boots can load values through
 * readNonVols() instead of reinitializing storage.
 *
 * @return true if initialization work was performed, false if EEPROM was already initialized.
 */
bool EepromManager::initializeEEPROMVars(void)
{
	bool init = false;
	uint16_t initialization_flag = avr_eeprom_read_word_at(Eeprom_initialization_flag);

	if(initialization_flag == EEPROM_INITIALIZED_FLAG)
	{
		return init;
	}

	if(migrateEEPROMLayoutIfNeeded(initialization_flag))
	{
		return init;
	}

	{
		/* First-time initialization writes defaults to both RAM globals and EEPROM. */
		g_evteng_id_codespeed = EEPROM_ID_CODE_SPEED_DEFAULT;
		avr_eeprom_write_byte(Id_codespeed, g_evteng_id_codespeed);

		g_isMaster = EEPROM_MASTER_SETTING_DEFAULT;
		avr_eeprom_write_byte(Reserved_Master_Setting, 0);

		fox_setting_slot_write_atomic(EVENT_NONE, EEPROM_FOX_SETTING_NONE_DEFAULT);
		avr_eeprom_write_byte(Fox_setting_none, (uint8_t)g_fox[EVENT_NONE]);

		fox_setting_slot_write_atomic(EVENT_CLASSIC, EEPROM_FOX_SETTING_CLASSIC_DEFAULT);
		avr_eeprom_write_byte(Fox_setting_classic, (uint8_t)g_fox[EVENT_CLASSIC]);

		fox_setting_slot_write_atomic(EVENT_SPRINT, EEPROM_FOX_SETTING_SPRINT_DEFAULT);
		avr_eeprom_write_byte(Fox_setting_sprint, (uint8_t)g_fox[EVENT_SPRINT]);

		fox_setting_slot_write_atomic(EVENT_FOXORING, EEPROM_FOX_SETTING_FOXORING_DEFAULT);
		avr_eeprom_write_byte(Fox_setting_foxoring, (uint8_t)g_fox[EVENT_FOXORING]);

		fox_setting_slot_write_atomic(EVENT_BLIND_ARDF, EEPROM_FOX_SETTING_BLIND_DEFAULT);
		avr_eeprom_write_byte(Fox_setting_blind, (uint8_t)g_fox[EVENT_BLIND_ARDF]);

		g_event = EEPROM_EVENT_SETTING_DEFAULT;
		avr_eeprom_write_byte(Event_setting, (uint8_t)g_event);

		g_frequency = EEPROM_FREQUENCY_DEFAULT;
		avr_eeprom_write_dword(Frequency, g_frequency);

		g_frequency_low = EEPROM_FREQUENCY_LOW_DEFAULT;
		avr_eeprom_write_dword(Frequency_Low, g_frequency_low);

		g_frequency_med = EEPROM_FREQUENCY_MED_DEFAULT;
		avr_eeprom_write_dword(Frequency_Med, g_frequency_med);

		g_frequency_hi = EEPROM_FREQUENCY_HI_DEFAULT;
		avr_eeprom_write_dword(Frequency_Hi, g_frequency_hi);

		g_frequency_beacon = EEPROM_FREQUENCY_BEACON_DEFAULT;
		avr_eeprom_write_dword(Frequency_Beacon, g_frequency_beacon);

		g_event_start_epoch = EEPROM_START_EPOCH_DEFAULT;
		avr_eeprom_write_dword(Event_start_epoch, g_event_start_epoch);

		g_event_finish_epoch = EEPROM_FINISH_EPOCH_DEFAULT;
		avr_eeprom_write_dword(Event_finish_epoch, g_event_finish_epoch);

		g_utc_offset = EEPROM_UTC_OFFSET_DEFAULT;
		avr_eeprom_write_byte(Utc_offset, (uint8_t)g_utc_offset);

		messages_text_slot_clear_atomic(STATION_ID);
		avr_eeprom_write_byte(StationID_text, 0);

		avr_eeprom_initialize_string(Pattern_text, EEPROM_FOX_PATTERN_DEFAULT, MAX_PATTERN_TEXT_LENGTH);
		messages_text_slot_publish_atomic(PATTERN_TEXT, EEPROM_FOX_PATTERN_DEFAULT);

		avr_eeprom_initialize_string(Foxoring_pattern_text, EEPROM_FOXORING_PATTERN_DEFAULT, MAX_PATTERN_TEXT_LENGTH);
		messages_text_slot_publish_atomic(FOXORING_PATTERN_TEXT, EEPROM_FOXORING_PATTERN_DEFAULT);

		avr_eeprom_write_zero_bytes(Reserved_Unlock_Code, MAX_UNLOCK_CODE_LENGTH + 2);
		avr_eeprom_write_dword(Reserved_RTTY_Offset, 0);

		g_80m_power_level_mW = EEPROM_TX_80M_POWER_MW_DEFAULT;
		avr_eeprom_write_word(RF_Power, g_80m_power_level_mW);

		g_evteng_pattern_codespeed = EEPROM_PATTERN_CODE_SPEED_DEFAULT;
		avr_eeprom_write_byte(Pattern_Code_Speed, g_evteng_pattern_codespeed);

		g_foxoring_pattern_codespeed = EEPROM_FOXORING_PATTERN_CODESPEED_DEFAULT;
		avr_eeprom_write_byte(Foxoring_Pattern_Code_Speed, g_foxoring_pattern_codespeed);

		g_evteng_off_air_seconds = EEPROM_OFF_AIR_TIME_DEFAULT;
		avr_eeprom_write_word(Off_Air_Seconds, g_evteng_off_air_seconds);

		g_evteng_on_air_seconds = EEPROM_ON_AIR_TIME_DEFAULT;
		avr_eeprom_write_word(On_Air_Seconds, g_evteng_on_air_seconds);

		g_evteng_ID_period_seconds = EEPROM_ID_TIME_INTERVAL_DEFAULT;
		avr_eeprom_write_word(ID_Period_Seconds, g_evteng_ID_period_seconds);

		g_evteng_intra_cycle_delay_time = EEPROM_INTRA_CYCLE_DELAY_TIME_DEFAULT;
		avr_eeprom_write_word(Intra_Cycle_Delay_Seconds, g_evteng_intra_cycle_delay_time);

		g_internal_voltage_low_threshold = EEPROM_INT_BATTERY_LOW_THRESHOLD_V;
		avr_eeprom_write_float(Voltage_threshold, g_internal_voltage_low_threshold);

		g_clock_calibration = EEPROM_CLOCK_CALIBRATION_DEFAULT;
		avr_eeprom_write_word(Clock_calibration, g_clock_calibration);

		g_days_to_run = 1;
		avr_eeprom_write_byte(Days_to_run, g_days_to_run);

		g_thermal_shutdown_threshold = EEPROM_THERMAL_SHUTDOWN_THRESHOLD_DEFAULT;
		avr_eeprom_write_byte(Thermal_Shutdown_Threshold, (uint8_t)g_thermal_shutdown_threshold);

		g_processor_max_ever_temperature = EEPROM_PROCESSOR_MAX_EVER_TEMPERATURE_DEFAULT;
		avr_eeprom_write_float(Hottest_Ever_Temperature, g_processor_max_ever_temperature);

		g_function = EEPROM_FUNCTION_DEFAULT;
		avr_eeprom_write_byte(Function, (uint8_t)g_function);

		g_enable_boost_regulator = false;
		avr_eeprom_write_byte(Enable_Boost_Regulator, (uint8_t)g_enable_boost_regulator);

		g_enable_external_battery_control = true;
		avr_eeprom_write_byte(Enable_External_Battery_Control, (uint8_t)g_enable_external_battery_control);

		g_device_enabled = false;
		avr_eeprom_write_byte(Device_Enabled, (uint8_t)g_device_enabled);

		/* Mark EEPROM as valid only after the default image has been fully written. */
		avr_eeprom_write_word(Eeprom_initialization_flag, EEPROM_INITIALIZED_FLAG);

		init = true;
	}

	return (init);
}
