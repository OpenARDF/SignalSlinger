#ifndef SIGNALSLINGER_SESSION_HISTORY_H
#define SIGNALSLINGER_SESSION_HISTORY_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

/* Wire values are stable. Pauses and resumes are history, not successful finishes. */
enum SessionAction : uint8_t { SESSION_NONE, SESSION_STARTED, SESSION_PAUSED, SESSION_RESUMED,
    SESSION_COMPLETED, SESSION_FINISHED_INTERRUPTED, SESSION_INTERRUPTED, SESSION_EXPIRED };
enum SessionReason : uint8_t { REASON_NONE, REASON_FINISH, REASON_THERMAL, REASON_USER,
    REASON_SETTINGS, REASON_RESET, REASON_CLOCK, REASON_SENSOR, REASON_DEVICE_DISABLED, REASON_TIMEOUT };
enum SessionFlags : uint8_t { SESSION_TIME_VALID = 1, SESSION_HAD_INTERRUPTION = 2,
    SESSION_SCHEDULED = 4, SESSION_HISTORY_GAP = 128 };

struct __attribute__((packed)) SessionRecord {
    uint32_t sequence;
    uint32_t schedule;
    uint32_t start;
    uint32_t finish;
    uint32_t timestamp;
    int16_t temperature;
    uint8_t action;
    uint8_t reason;
    uint8_t flags;
    uint8_t threshold;
    uint8_t magic;
    uint8_t version;
    uint16_t checksum;
};
static_assert(sizeof(SessionRecord) == 30, "Persistent session record layout changed");

inline uint16_t sessionRecordChecksum(const SessionRecord& record) {
    const uint8_t* bytes = (const uint8_t*)&record;
    uint16_t crc = 0xffff;
    for(size_t i = 0; i < offsetof(SessionRecord, checksum); ++i) {
        crc ^= (uint16_t)bytes[i] << 8;
        for(uint8_t bit = 0; bit < 8; ++bit)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}
inline bool sessionRecordValid(const SessionRecord& record) {
    return record.magic == 0xa7 && record.version == 1 && record.action >= SESSION_STARTED &&
           record.action <= SESSION_EXPIRED && record.reason <= REASON_TIMEOUT &&
           record.checksum == sessionRecordChecksum(record);
}
inline bool sessionSequenceAfter(uint32_t a, uint32_t b) { return (int32_t)(a - b) > 0; }
inline bool sessionActionActive(uint8_t action) {
    return action == SESSION_STARTED || action == SESSION_PAUSED || action == SESSION_RESUMED;
}

/* Calendar progress is independent of outcomes. Windows whose finish has passed
 * are never counted as future runs, even if they were interrupted or missed. */
inline uint8_t sessionDayIndex(uint32_t firstStart, uint32_t firstFinish, uint8_t days, uint32_t now) {
    if(!days || !firstStart || firstFinish <= firstStart) return days;
    uint8_t day = 0;
    while(day < days && (uint64_t)firstFinish + (uint64_t)day * 86400 <= now) ++day;
    return day;
}

/* Pure lifecycle policy; callers serialize access against the RTC ISR. */
struct SessionLifecycle {
    SessionRecord record = {};
    bool active() const { return sessionActionActive(record.action); }
    bool paused() const { return record.action == SESSION_PAUSED; }
    bool begin(uint32_t schedule, uint32_t start, uint32_t finish, bool scheduled) {
        if(active() && record.start == start && record.finish == finish) return false;
        uint8_t prior = record.start == start && record.finish == finish ?
                            (record.flags & SESSION_HAD_INTERRUPTION) : 0;
        record = {};
        record.schedule = schedule; record.start = start; record.finish = finish;
        record.flags = prior | (scheduled ? SESSION_SCHEDULED : 0);
        record.action = SESSION_STARTED;
        return true;
    }
    bool pause(uint8_t reason) {
        if(!active() || paused()) return false;
        record.action = SESSION_PAUSED; record.reason = reason;
        record.flags |= SESSION_HAD_INTERRUPTION;
        return true;
    }
    bool canResume(uint32_t now, bool safe, bool authorized) const {
        return paused() && safe && authorized && (record.flags & SESSION_SCHEDULED) &&
               now >= record.start && now < record.finish;
    }
    bool resume(uint32_t now, bool safe, bool authorized) {
        if(!canResume(now, safe, authorized)) return false;
        record.action = SESSION_RESUMED; record.reason = REASON_NONE;
        return true;
    }
    bool stop(uint8_t reason) {
        if(!active()) return false;
        bool wasPaused = paused();
        record.action = (reason == REASON_FINISH || reason == REASON_TIMEOUT) && !wasPaused ?
            ((record.flags & SESSION_HAD_INTERRUPTION) ? SESSION_FINISHED_INTERRUPTED : SESSION_COMPLETED) : SESSION_INTERRUPTED;
        if(reason != REASON_FINISH || !wasPaused) record.reason = reason;
        if(record.action == SESSION_INTERRUPTED) record.flags |= SESSION_HAD_INTERRUPTION;
        return true;
    }
};

/* Foreground EEPROM operations; never call these from an ISR. */
uint8_t readSessionHistory(SessionRecord* records, uint8_t capacity);
void appendSessionHistory(SessionRecord record);
uint8_t sessionHistoryCapacity();

#endif
