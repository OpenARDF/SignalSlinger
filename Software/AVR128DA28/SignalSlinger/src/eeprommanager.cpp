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

#include "serial_latency.h"
#include "defs.h"
#include "eeprommanager.h"
#include "serialbus.h"
#include "i2c.h"
#include "transmitter.h"
#include "globals.h"
#include "shared_state.h"
#include "thermal_shutdown.h"
#include "session_history.h"
#include "hardware_fault_history.h"
#include <avr/pgmspace.h>
#include <string.h>

/* Canonical EEPROM image used both for layout definition and default contents. */
const struct EE_prom EEMEM EepromManager::ee_vars = {};

static_assert(sizeof(EE_prom) == 135, "Compact EEPROM settings layout changed unexpectedly");

/* Exact v0x0136 layout. It is retained only long enough to migrate installed
 * units; new code never writes this padded map. AVR data types are byte-aligned. */
struct __attribute__((packed)) LegacyEEPromV0136
{
	uint16_t eeprom_initialization_flag; uint32_t reserved_1; time_t event_start_epoch;
	uint32_t reserved_2; time_t event_finish_epoch; uint32_t reserved_3;
	char pattern_text[MAX_PATTERN_TEXT_LENGTH + 2]; uint32_t reserved_4;
	char foxoring_pattern_text[MAX_PATTERN_TEXT_LENGTH + 2]; uint32_t reserved_5;
	char stationID_text[MAX_PATTERN_TEXT_LENGTH + 2]; uint32_t reserved_6;
	uint8_t reserved_unlock_code[MAX_UNLOCK_CODE_LENGTH + 2]; uint32_t reserved_7;
	Fox_t fox_setting_none; uint32_t reserved_8; Fox_t fox_setting_classic;
	uint32_t reserved_9; Fox_t fox_setting_sprint; uint32_t reserved_10;
	Fox_t fox_setting_foxoring; uint32_t reserved_11; Fox_t fox_setting_blind;
	uint32_t reserved_12; uint8_t utc_offset; uint32_t reserved_13;
	Frequency_Hz reserved_rtty_offset; uint32_t reserved_14; uint16_t rf_power;
	uint32_t reserved_15; uint8_t id_codespeed; uint32_t reserved_16;
	uint8_t pattern_codespeed; uint32_t reserved_17; uint8_t foxoring_pattern_codespeed;
	uint32_t reserved_18; int16_t off_air_seconds; uint32_t reserved_19;
	int16_t on_air_seconds; uint32_t reserved_20; int16_t ID_period_seconds;
	uint32_t reserved_21; int16_t intra_cycle_delay_time; uint32_t reserved_22;
	Event_t event_setting; uint32_t reserved_23; uint32_t frequency;
	uint32_t reserved_24; uint32_t frequency_low; uint32_t reserved_25;
	uint32_t frequency_med; uint32_t reserved_26; uint32_t frequency_high;
	uint32_t reserved_27; uint32_t frequency_beacon; uint32_t reserved_28;
	bool reserved_master_setting; uint32_t reserved_29; float voltage_threshold;
	uint32_t reserved_30; uint16_t clock_calibration; int32_t si5351_correction_ppb;
	uint8_t days_to_run; uint32_t reserved_32; int8_t thermal_shutdown_threshold;
	uint8_t thermal_shutdown_enabled_marker; float hottest_ever_temperature;
	uint8_t function; uint32_t reserved_34; uint8_t enable_boost_regulator;
	uint32_t reserved_35; uint8_t enable_external_battery_control;
	uint32_t reserved_36; uint8_t device_enabled;
};
static_assert(sizeof(LegacyEEPromV0136) == 288, "Legacy EEPROM layout must remain readable");

typedef uint16_t eeprom_addr_t;

/* EEPROM access is foreground/startup work. Polling busy permits already-enabled
 * interrupts to run, unlike a mapped-memory access while EEPROM is busy, which
 * stalls the CPU. Never change the caller's interrupt state here. ISR code must
 * continue to queue persistence work for the foreground rather than call us. */
static void avr_eeprom_wait_ready(void)
{
	while(NVMCTRL.STATUS & NVMCTRL_EEBUSY_bm)
		;
}

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
	SERIAL_LATENCY_SCOPE(LAT_EE_BYTE);
	avr_eeprom_wait_ready();
	_PROTECTED_WRITE_SPM(NVMCTRL.CTRLA, NVMCTRL_CMD_EEERWR_gc);
	*(volatile uint8_t *)(eeprom_addr_t)(MAPPED_EEPROM_START + index) = in;
	_PROTECTED_WRITE_SPM(NVMCTRL.CTRLA, NVMCTRL_CMD_NONE_gc);
}

/* Preserve the AVR object representation and ascending byte order. Each byte
 * waits for its predecessor through the existing writer; the final byte remains
 * asynchronous, as before. Every reader also waits before accessing EEPROM. */
static void avr_eeprom_write_bytes(eeprom_addr_t index, const uint8_t *bytes, uint8_t count)
{
	for(uint8_t i = 0; i < count; ++i)
		avr_eeprom_write_byte(index + i, bytes[i]);
}

/**
 * Write a 16-bit value to EEPROM through the AVR mapped EEPROM window.
 *
 * @param index Byte offset into EEPROM.
 * @param in    Value to store.
 */
void avr_eeprom_write_word(eeprom_addr_t index, uint16_t in)
{
	SERIAL_LATENCY_SCOPE(LAT_EE_WORD);
	avr_eeprom_write_bytes(index, (const uint8_t *)&in, sizeof(in));
}

/**
 * Write a 32-bit value to EEPROM through the AVR mapped EEPROM window.
 *
 * @param index Byte offset into EEPROM.
 * @param in    Value to store.
 */
void avr_eeprom_write_dword(eeprom_addr_t index, uint32_t in)
{
	SERIAL_LATENCY_SCOPE(LAT_EE_DWORD);
	avr_eeprom_write_bytes(index, (const uint8_t *)&in, sizeof(in));
}

/**
 * Write a float value to EEPROM through the AVR mapped EEPROM window.
 *
 * @param index Byte offset into EEPROM.
 * @param in    Value to store.
 */
void avr_eeprom_write_float(eeprom_addr_t index, float in)
{
	SERIAL_LATENCY_SCOPE(LAT_EE_FLOAT);
	avr_eeprom_write_bytes(index, (const uint8_t *)&in, sizeof(in));
}

/**
 * Read one byte from EEPROM at the supplied offset.
 *
 * @param index Byte offset into EEPROM.
 * @return Stored byte value.
 */
static uint8_t avr_eeprom_read_byte_at(eeprom_addr_t index)
{
	SERIAL_LATENCY_SCOPE(LAT_EE_READ);
	avr_eeprom_wait_ready();
	return eeprom_read_byte((const uint8_t *)(eeprom_addr_t)index);
}

/* Fixed EEPROM v2 regions. Mission-critical settings occupy the first region,
 * including 25 reserved bytes for append-only growth. Bounded logs follow it;
 * neither a setting addition nor log growth may move another region silently. */
static const uint16_t EEPROM_SETTINGS_REGION_BYTES = 160;
static_assert(sizeof(EE_prom) <= EEPROM_SETTINGS_REGION_BYTES, "EEPROM settings exceeded their fixed region");
static const uint16_t SESSION_JOURNAL_START = EEPROM_SETTINGS_REGION_BYTES;
static const uint8_t SESSION_JOURNAL_SLOTS = 7;
static const uint16_t HARDWARE_FAULT_HISTORY_START = SESSION_JOURNAL_START + SESSION_JOURNAL_SLOTS * sizeof(SessionRecord);
static const uint8_t HARDWARE_FAULT_HISTORY_SLOTS = HARDWARE_FAULT_MAX;
static_assert(HARDWARE_FAULT_HISTORY_START + HARDWARE_FAULT_HISTORY_SLOTS * sizeof(HardwareFaultRecord) <= EEPROM_SIZE,
              "EEPROM v2 regions exceed physical storage");

uint8_t sessionHistoryCapacity() { return SESSION_JOURNAL_SLOTS; }

static SessionRecord readSessionSlot(uint8_t slot)
{
	SessionRecord record;
	uint8_t* bytes = (uint8_t*)&record;
	for(uint8_t i = 0; i < sizeof(record); ++i)
		bytes[i] = avr_eeprom_read_byte_at(SESSION_JOURNAL_START + slot * sizeof(record) + i);
	return record;
}

uint8_t readSessionHistory(SessionRecord* records, uint8_t capacity)
{
	uint8_t count = 0;
	for(uint8_t slot = 0; slot < SESSION_JOURNAL_SLOTS; ++slot)
	{
		SessionRecord record = readSessionSlot(slot);
		if(!sessionRecordValid(record) || !capacity) continue;
		if(count == capacity)
		{
			if(!sessionSequenceAfter(record.sequence, records[0].sequence)) continue;
			memmove(records, records + 1, (count - 1) * sizeof(record));
			--count;
		}
		uint8_t pos = count;
		while(pos && sessionSequenceAfter(records[pos - 1].sequence, record.sequence))
		{
			records[pos] = records[pos - 1]; --pos;
		}
		records[pos] = record; ++count;
	}
	return count;
}

void appendSessionHistory(SessionRecord record)
{
	SERIAL_LATENCY_SCOPE(LAT_HISTORY);
	uint8_t next = 0;
	uint32_t latest = 0;
	bool found = false;
	for(uint8_t slot = 0; slot < SESSION_JOURNAL_SLOTS; ++slot)
	{
		SessionRecord previous = readSessionSlot(slot);
		if(sessionRecordValid(previous) && (!found || sessionSequenceAfter(previous.sequence, latest)))
		{
			latest = previous.sequence; next = (slot + 1) % SESSION_JOURNAL_SLOTS; found = true;
		}
	}
	if(sessionRecordValid(readSessionSlot(next))) record.flags |= SESSION_HISTORY_GAP;
	record.sequence = found ? latest + 1 : 1;
	record.magic = 0xa7; record.version = 1;
	record.checksum = sessionRecordChecksum(record);
	uint16_t offset = SESSION_JOURNAL_START + next * sizeof(record);
	const uint8_t* bytes = (const uint8_t*)&record;
	avr_eeprom_write_byte(offset + offsetof(SessionRecord, magic), 0);
	for(uint8_t i = 0; i < sizeof(record); ++i)
		if(i != offsetof(SessionRecord, magic)) avr_eeprom_write_byte(offset + i, bytes[i]);
	avr_eeprom_write_byte(offset + offsetof(SessionRecord, magic), record.magic);
}

uint8_t hardwareFaultHistoryCapacity() { return HARDWARE_FAULT_HISTORY_SLOTS; }

static HardwareFaultRecord readHardwareFaultSlot(uint8_t slot)
{
	HardwareFaultRecord record;
	uint8_t *bytes = (uint8_t *)&record;
	for(uint8_t i = 0; i < sizeof(record); ++i)
		bytes[i] = avr_eeprom_read_byte_at(HARDWARE_FAULT_HISTORY_START + slot * sizeof(record) + i);
	return record;
}

uint8_t readHardwareFaultHistory(HardwareFaultRecord *records, uint8_t capacity)
{
	uint8_t count = 0;
	for(uint8_t slot = 0; slot < HARDWARE_FAULT_HISTORY_SLOTS && count < capacity; ++slot)
	{
		HardwareFaultRecord record = readHardwareFaultSlot(slot);
		if(hardwareFaultRecordValid(record)) records[count++] = record;
	}
	return count;
}

bool recordHardwareFault(uint8_t code, uint32_t detected_at, bool time_valid)
{
	if(code < HARDWARE_FAULT_RTC || code > HARDWARE_FAULT_MAX) return false;
	if((time_valid && detected_at == 0) || (!time_valid && detected_at != 0)) return false;
	uint8_t slot = code - 1;
	if(hardwareFaultRecordValid(readHardwareFaultSlot(slot))) return false;

	HardwareFaultRecord record = {};
	record.detected_at = time_valid ? detected_at : 0;
	record.code = code;
	record.flags = time_valid ? HARDWARE_FAULT_TIME_VALID : 0;
	record.magic = 0x48;
	record.version = 1;
	record.checksum = hardwareFaultRecordChecksum(record);
	uint16_t offset = HARDWARE_FAULT_HISTORY_START + slot * sizeof(record);
	const uint8_t *bytes = (const uint8_t *)&record;
	avr_eeprom_write_byte(offset + offsetof(HardwareFaultRecord, magic), 0);
	for(uint8_t i = 0; i < sizeof(record); ++i)
		if(i != offsetof(HardwareFaultRecord, magic)) avr_eeprom_write_byte(offset + i, bytes[i]);
	avr_eeprom_write_byte(offset + offsetof(HardwareFaultRecord, magic), record.magic);
	return true;
}

void clearHardwareFaultHistory()
{
	for(uint8_t slot = 0; slot < HARDWARE_FAULT_HISTORY_SLOTS; ++slot)
		avr_eeprom_write_byte(HARDWARE_FAULT_HISTORY_START + slot * sizeof(HardwareFaultRecord) +
		                          offsetof(HardwareFaultRecord, magic),
		                      0);
}

/**
 * Read a 16-bit value from EEPROM at the supplied offset.
 *
 * @param index Byte offset into EEPROM.
 * @return Stored 16-bit value.
 */
static uint16_t avr_eeprom_read_word_at(eeprom_addr_t index)
{
	SERIAL_LATENCY_SCOPE(LAT_EE_READ);
	avr_eeprom_wait_ready();
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
	SERIAL_LATENCY_SCOPE(LAT_EE_READ);
	avr_eeprom_wait_ready();
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
	SERIAL_LATENCY_SCOPE(LAT_EE_READ);
	avr_eeprom_wait_ready();
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
 * Decode a current-layout EEPROM event byte into a valid runtime event.
 *
 * @param stored_event Raw byte read from EEPROM.
 * @return Valid event, or the firmware default if the byte is out of range.
 */
static Event_t currentEEPROMEventOrDefault(uint8_t stored_event)
{
	Event_t event = (Event_t)stored_event;
	return EVENT_IS_VALID(event) ? event : EEPROM_EVENT_SETTING_DEFAULT;
}

/**
 * Decode an EEPROM event byte saved by firmware versions that still used zero
 * as an explicit "no event" selector.
 *
 * @param stored_event Raw legacy byte read from EEPROM.
 * @return Valid current-layout event, defaulting old none/invalid values to Classic.
 */
static Event_t legacyEEPROMEventOrDefault(uint8_t stored_event)
{
	switch(stored_event)
	{
		case 1:
			return EVENT_CLASSIC;

		case 2:
			return EVENT_SPRINT;

		case 3:
			return EVENT_FOXORING;

		case 4:
			return EEPROM_EVENT_SETTING_DEFAULT;

		default:
			return EEPROM_EVENT_SETTING_DEFAULT;
	}
}

static bool legacyEEPROMVersionKnown(uint16_t version)
{
	return version >= EEPROM_INITIALIZED_FLAG_V0131 && version <= EEPROM_INITIALIZED_FLAG_V0136;
}

/* Move an installed unit from the padded 288-byte map to the compact v2 map.
 * Every old setting and all seven journal slots are copied to RAM before the
 * first overlapping write. The new version marker is committed last. */
static bool migrateEEPROMLayoutIfNeeded(uint16_t initialization_flag)
{
	if(!legacyEEPROMVersionKnown(initialization_flag)) return false;

	LegacyEEPromV0136 legacy = {};
	uint8_t *legacy_bytes = (uint8_t *)&legacy;
	for(uint16_t i = 0; i < sizeof(legacy); ++i) legacy_bytes[i] = avr_eeprom_read_byte_at(i);

	SessionRecord history[SESSION_JOURNAL_SLOTS] = {};
	uint8_t history_count = 0;
	for(uint8_t slot = 0; slot < SESSION_JOURNAL_SLOTS; ++slot)
	{
		SessionRecord record;
		uint8_t *bytes = (uint8_t *)&record;
		for(uint8_t i = 0; i < sizeof(record); ++i)
			bytes[i] = avr_eeprom_read_byte_at(sizeof(LegacyEEPromV0136) + slot * sizeof(record) + i);
		if(!sessionRecordValid(record)) continue;
		uint8_t pos = history_count;
		while(pos && sessionSequenceAfter(history[pos - 1].sequence, record.sequence))
		{
			history[pos] = history[pos - 1];
			--pos;
		}
		history[pos] = record;
		++history_count;
	}

	if(initialization_flag == EEPROM_INITIALIZED_FLAG_V0131)
	{
		legacy.thermal_shutdown_threshold = EEPROM_THERMAL_SHUTDOWN_THRESHOLD_DEFAULT;
		legacy.thermal_shutdown_enabled_marker = thermalShutdownMarkerForEnabled(EEPROM_THERMAL_SHUTDOWN_ENABLED_DEFAULT);
		legacy.hottest_ever_temperature = EEPROM_PROCESSOR_MAX_EVER_TEMPERATURE_DEFAULT;
	}
	else if(initialization_flag == EEPROM_INITIALIZED_FLAG_V0132)
	{
		legacy.hottest_ever_temperature = EEPROM_PROCESSOR_MAX_EVER_TEMPERATURE_DEFAULT;
	}
	if(initialization_flag <= EEPROM_INITIALIZED_FLAG_V0135)
		legacy.si5351_correction_ppb = EEPROM_SI5351_CALIBRATION_DEFAULT;

	Event_t migrated_event = initialization_flag >= EEPROM_INITIALIZED_FLAG_V0134
	                             ? currentEEPROMEventOrDefault((uint8_t)legacy.event_setting)
	                             : legacyEEPROMEventOrDefault((uint8_t)legacy.event_setting);

	/* Invalidate first; interruption therefore falls back to safe defaults and
	 * allows SerialSlinger's retained update backup to be restored. */
	avr_eeprom_write_word(Eeprom_initialization_flag, EEPROM_UNINITIALIZED);
#define COPY_LEGACY_BYTES(field) avr_eeprom_write_bytes(offsetof(EE_prom, field), (const uint8_t *)&legacy.field, sizeof(legacy.field))
	COPY_LEGACY_BYTES(event_start_epoch); COPY_LEGACY_BYTES(event_finish_epoch);
	COPY_LEGACY_BYTES(pattern_text); COPY_LEGACY_BYTES(foxoring_pattern_text); COPY_LEGACY_BYTES(stationID_text);
	COPY_LEGACY_BYTES(fox_setting_classic); COPY_LEGACY_BYTES(fox_setting_sprint); COPY_LEGACY_BYTES(fox_setting_foxoring);
	COPY_LEGACY_BYTES(utc_offset); COPY_LEGACY_BYTES(rf_power); COPY_LEGACY_BYTES(id_codespeed);
	COPY_LEGACY_BYTES(pattern_codespeed); COPY_LEGACY_BYTES(foxoring_pattern_codespeed);
	COPY_LEGACY_BYTES(off_air_seconds); COPY_LEGACY_BYTES(on_air_seconds); COPY_LEGACY_BYTES(ID_period_seconds);
	COPY_LEGACY_BYTES(intra_cycle_delay_time);
	avr_eeprom_write_byte(Event_setting, (uint8_t)migrated_event);
	COPY_LEGACY_BYTES(frequency); COPY_LEGACY_BYTES(frequency_low); COPY_LEGACY_BYTES(frequency_med);
	COPY_LEGACY_BYTES(frequency_high); COPY_LEGACY_BYTES(frequency_beacon);
	COPY_LEGACY_BYTES(voltage_threshold); COPY_LEGACY_BYTES(clock_calibration); COPY_LEGACY_BYTES(si5351_correction_ppb);
	COPY_LEGACY_BYTES(days_to_run); COPY_LEGACY_BYTES(thermal_shutdown_threshold);
	COPY_LEGACY_BYTES(thermal_shutdown_enabled_marker); COPY_LEGACY_BYTES(hottest_ever_temperature);
	COPY_LEGACY_BYTES(function); COPY_LEGACY_BYTES(enable_boost_regulator);
	COPY_LEGACY_BYTES(enable_external_battery_control); COPY_LEGACY_BYTES(device_enabled);
#undef COPY_LEGACY_BYTES

	for(uint8_t slot = 0; slot < SESSION_JOURNAL_SLOTS; ++slot)
		avr_eeprom_write_byte(SESSION_JOURNAL_START + slot * sizeof(SessionRecord) + offsetof(SessionRecord, magic), 0);
	for(uint8_t i = 0; i < history_count; ++i) appendSessionHistory(history[i]);
	clearHardwareFaultHistory();
	avr_eeprom_write_word(Eeprom_initialization_flag, EEPROM_INITIALIZED_FLAG);
	return true;
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
		case Fox_setting_classic:
		case Fox_setting_sprint:
		case Fox_setting_foxoring:
		case Utc_offset:
		case Days_to_run:
		case Thermal_Shutdown_Threshold:
		case Thermal_Shutdown_Enabled_Marker:
		case Function:
		case Enable_Boost_Regulator:
		case Enable_External_Battery_Control:
		case Device_Enabled:
			avr_eeprom_write_byte_if_changed((eeprom_addr_t)v, *(const uint8_t *)val);
			break;

		case Event_setting:
		{
			Event_t event = *(const Event_t *)val;
			if(EVENT_IS_VALID(event))
			{
				avr_eeprom_write_byte_if_changed((eeprom_addr_t)v, (uint8_t)event);
			}
			break;
		}

		case Frequency:
		case Frequency_Low:
		case Frequency_Med:
		case Frequency_Hi:
		case Frequency_Beacon:
		case Event_start_epoch:
		case Event_finish_epoch:
			avr_eeprom_write_dword_if_changed((eeprom_addr_t)v, *(const uint32_t *)val);
			break;

		case Si5351_Correction:
			avr_eeprom_write_dword_if_changed((eeprom_addr_t)v, (uint32_t)*(const int32_t *)val);
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
	updateEEPROMVar(Fox_setting_classic, (void *)&g_fox[EVENT_CLASSIC]);
	updateEEPROMVar(Fox_setting_sprint, (void *)&g_fox[EVENT_SPRINT]);
	updateEEPROMVar(Fox_setting_foxoring, (void *)&g_fox[EVENT_FOXORING]);
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
	int32_t si5351_correction_ppb = txGetCalibrationCorrectionPpb();
	updateEEPROMVar(Si5351_Correction, (void *)&si5351_correction_ppb);
	updateEEPROMVar(Days_to_run, (void *)&g_days_to_run);
	updateEEPROMVar(Thermal_Shutdown_Threshold, (void *)&g_thermal_shutdown_threshold);
	uint8_t thermal_shutdown_enabled_marker = thermalShutdownMarkerForEnabled(g_thermal_shutdown_enabled);
	updateEEPROMVar(Thermal_Shutdown_Enabled_Marker, (void *)&thermal_shutdown_enabled_marker);
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
		g_event = currentEEPROMEventOrDefault(avr_eeprom_read_byte_at(Event_setting));
		avr_eeprom_write_byte_if_changed(Event_setting, (uint8_t)g_event);
		g_frequency = CLAMP(TX_MINIMUM_FREQUENCY, avr_eeprom_read_dword_at(Frequency), TX_MAXIMUM_FREQUENCY);
		g_frequency_low = CLAMP(TX_MINIMUM_FREQUENCY, avr_eeprom_read_dword_at(Frequency_Low), TX_MAXIMUM_FREQUENCY);
		g_frequency_med = CLAMP(TX_MINIMUM_FREQUENCY, avr_eeprom_read_dword_at(Frequency_Med), TX_MAXIMUM_FREQUENCY);
		g_frequency_hi = CLAMP(TX_MINIMUM_FREQUENCY, avr_eeprom_read_dword_at(Frequency_Hi), TX_MAXIMUM_FREQUENCY);
		g_frequency_beacon = CLAMP(TX_MINIMUM_FREQUENCY, avr_eeprom_read_dword_at(Frequency_Beacon), TX_MAXIMUM_FREQUENCY);
		g_enable_boost_regulator = (bool)avr_eeprom_read_byte_at(Enable_Boost_Regulator);
		fox_setting_slot_write_atomic(EVENT_CLASSIC, avr_eeprom_read_clamped_fox(Fox_setting_classic, FOX_5));
		fox_setting_slot_write_atomic(EVENT_SPRINT, avr_eeprom_read_clamped_fox(Fox_setting_sprint, SPRINT_F5));
		fox_setting_slot_write_atomic(EVENT_FOXORING, avr_eeprom_read_clamped_fox(Fox_setting_foxoring, FREQUENCY_TEST_BEACON));
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

		int32_t si5351_correction_ppb = (int32_t)avr_eeprom_read_dword_at(Si5351_Correction);
		if(txRestoreCalibrationCorrectionPpb(si5351_correction_ppb))
		{
			si5351_correction_ppb = EEPROM_SI5351_CALIBRATION_DEFAULT;
			txRestoreCalibrationCorrectionPpb(si5351_correction_ppb);
			avr_eeprom_write_dword(Si5351_Correction, (uint32_t)si5351_correction_ppb);
		}

		g_days_to_run = avr_eeprom_read_byte_at(Days_to_run);

		uint8_t thermal_marker = avr_eeprom_read_byte_at(Thermal_Shutdown_Enabled_Marker);
		int8_t saved_threshold = (int8_t)avr_eeprom_read_byte_at(Thermal_Shutdown_Threshold);
		g_thermal_shutdown_threshold = thermalShutdownMarkerIsKnown(thermal_marker) &&
		                                  saved_threshold >= THERMAL_SHUTDOWN_THRESHOLD_MIN_C &&
		                                  saved_threshold <= THERMAL_SHUTDOWN_THRESHOLD_MAX_C
		                              ? saved_threshold : EEPROM_THERMAL_SHUTDOWN_THRESHOLD_DEFAULT;
		g_thermal_shutdown_enabled = thermalShutdownEnabledFromMarker(thermal_marker);
		if(!thermalShutdownMarkerIsKnown(thermal_marker))
		{
			avr_eeprom_write_byte(Thermal_Shutdown_Threshold, (uint8_t)g_thermal_shutdown_threshold);
			avr_eeprom_write_byte(Thermal_Shutdown_Enabled_Marker, THERMAL_SHUTDOWN_ENABLED_MARKER);
		}

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

		fox_setting_slot_write_atomic(EVENT_CLASSIC, EEPROM_FOX_SETTING_CLASSIC_DEFAULT);
		avr_eeprom_write_byte(Fox_setting_classic, (uint8_t)g_fox[EVENT_CLASSIC]);

		fox_setting_slot_write_atomic(EVENT_SPRINT, EEPROM_FOX_SETTING_SPRINT_DEFAULT);
		avr_eeprom_write_byte(Fox_setting_sprint, (uint8_t)g_fox[EVENT_SPRINT]);

		fox_setting_slot_write_atomic(EVENT_FOXORING, EEPROM_FOX_SETTING_FOXORING_DEFAULT);
		avr_eeprom_write_byte(Fox_setting_foxoring, (uint8_t)g_fox[EVENT_FOXORING]);

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

		txRestoreCalibrationCorrectionPpb(EEPROM_SI5351_CALIBRATION_DEFAULT);
		avr_eeprom_write_dword(Si5351_Correction, (uint32_t)EEPROM_SI5351_CALIBRATION_DEFAULT);

		g_days_to_run = 1;
		avr_eeprom_write_byte(Days_to_run, g_days_to_run);

		g_thermal_shutdown_threshold = EEPROM_THERMAL_SHUTDOWN_THRESHOLD_DEFAULT;
		avr_eeprom_write_byte(Thermal_Shutdown_Threshold, (uint8_t)g_thermal_shutdown_threshold);
		g_thermal_shutdown_enabled = EEPROM_THERMAL_SHUTDOWN_ENABLED_DEFAULT;
		avr_eeprom_write_byte(Thermal_Shutdown_Enabled_Marker,
		                      thermalShutdownMarkerForEnabled(g_thermal_shutdown_enabled));

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

		for(uint8_t slot = 0; slot < SESSION_JOURNAL_SLOTS; ++slot)
			avr_eeprom_write_byte(SESSION_JOURNAL_START + slot * sizeof(SessionRecord) + offsetof(SessionRecord, magic), 0);
		clearHardwareFaultHistory();

		/* Mark EEPROM as valid only after settings and persistent logs are initialized. */
		avr_eeprom_write_word(Eeprom_initialization_flag, EEPROM_INITIALIZED_FLAG);

		init = true;
	}

	return (init);
}
