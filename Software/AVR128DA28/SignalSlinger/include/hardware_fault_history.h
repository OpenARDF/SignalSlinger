#ifndef SIGNALSLINGER_HARDWARE_FAULT_HISTORY_H
#define SIGNALSLINGER_HARDWARE_FAULT_HISTORY_H

#include <stddef.h>
#include <stdint.h>

/* Stable values used both in EEPROM and on the serial protocol. */
enum HardwareFaultCode : uint8_t
{
	HARDWARE_FAULT_NONE = 0,
	HARDWARE_FAULT_RTC = 1,
	HARDWARE_FAULT_TRANSMITTER = 2,
	HARDWARE_FAULT_MAX = 8
};

enum HardwareFaultFlags : uint8_t
{
	HARDWARE_FAULT_TIME_VALID = 1
};

struct __attribute__((packed)) HardwareFaultRecord
{
	uint32_t detected_at;
	uint8_t code;
	uint8_t flags;
	uint8_t magic;
	uint8_t version;
	uint16_t checksum;
};
static_assert(sizeof(HardwareFaultRecord) == 10, "Persistent hardware fault record layout changed");

inline uint16_t hardwareFaultRecordChecksum(const HardwareFaultRecord &record)
{
	const uint8_t *bytes = (const uint8_t *)&record;
	uint16_t crc = 0xffff;
	for(size_t i = 0; i < offsetof(HardwareFaultRecord, checksum); ++i)
	{
		crc ^= (uint16_t)bytes[i] << 8;
		for(uint8_t bit = 0; bit < 8; ++bit)
			crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
	}
	return crc;
}

inline bool hardwareFaultRecordValid(const HardwareFaultRecord &record)
{
	return record.magic == 0x48 && record.version == 1 &&
	       record.code >= HARDWARE_FAULT_RTC && record.code <= HARDWARE_FAULT_MAX &&
	       (record.flags & ~HARDWARE_FAULT_TIME_VALID) == 0 &&
	       ((record.flags & HARDWARE_FAULT_TIME_VALID) ? record.detected_at != 0 : record.detected_at == 0) &&
	       record.checksum == hardwareFaultRecordChecksum(record);
}

uint8_t hardwareFaultHistoryCapacity();
uint8_t readHardwareFaultHistory(HardwareFaultRecord *records, uint8_t capacity);
bool recordHardwareFault(uint8_t code, uint32_t detected_at, bool time_valid);
void clearHardwareFaultHistory();

#endif
