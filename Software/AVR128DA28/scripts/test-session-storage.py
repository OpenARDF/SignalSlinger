#!/usr/bin/env python3
"""Exercise the production EEPROM journal with simulated byte writes/power cuts."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parent.parent
source = (root / 'SignalSlinger/src/eeprommanager.cpp').read_text()
start = source.index('static const uint16_t SESSION_JOURNAL_START')
end = source.index('/**', source.index('void appendSessionHistory', start))
journal = source[start:end]
migration_start = source.index('uint8_t thermal_marker = avr_eeprom_read_byte_at')
migration_end = source.index('float hottest_ever_temperature', migration_start)
migration = source[migration_start:migration_end]
stubs = r'''
#include "session_history.h"
#include "thermal_shutdown.h"
#include <assert.h>
#include <stdexcept>
#include <array>
// Deliberately leave only the required minimum four slots in this fixture.
struct EE_prom { uint8_t settings[392]; };
#define EEPROM_SIZE 512
static std::array<uint8_t, EEPROM_SIZE> storage;
static int writes_left = -1;
static bool testing_migration = false;
static int8_t g_thermal_shutdown_threshold;
static bool g_thermal_shutdown_enabled;
enum { Thermal_Shutdown_Threshold=2, Thermal_Shutdown_Enabled_Marker=3 };
#define THERMAL_SHUTDOWN_THRESHOLD_MIN_C 30
#define THERMAL_SHUTDOWN_THRESHOLD_MAX_C 85
#define EEPROM_THERMAL_SHUTDOWN_THRESHOLD_DEFAULT 65
uint8_t avr_eeprom_read_byte_at(uint16_t offset) { return storage.at(offset); }
void avr_eeprom_write_byte(uint16_t offset, uint8_t value) {
    assert(testing_migration || offset >= sizeof(EE_prom));
    if(writes_left == 0) throw std::runtime_error("power cut");
    if(writes_left > 0) --writes_left;
    storage.at(offset) = value;
}
'''
tests = r'''
int main() {
    testing_migration = true;
    for(uint8_t marker : {0x00, 0xff, 0x37}) {
        storage.fill(0xff);
        storage[Thermal_Shutdown_Threshold] = 50;
        storage[Thermal_Shutdown_Enabled_Marker] = marker;
        migrate();
        assert(g_thermal_shutdown_enabled && g_thermal_shutdown_threshold == 65);
        assert(storage[Thermal_Shutdown_Enabled_Marker] == THERMAL_SHUTDOWN_ENABLED_MARKER);
        assert(storage[Thermal_Shutdown_Threshold] == 65);
    }
    storage[Thermal_Shutdown_Enabled_Marker] = THERMAL_SHUTDOWN_DISABLED_MARKER;
    storage[Thermal_Shutdown_Threshold] = 70; migrate();
    assert(!g_thermal_shutdown_enabled && g_thermal_shutdown_threshold == 70);
    storage[Thermal_Shutdown_Enabled_Marker] = THERMAL_SHUTDOWN_ENABLED_MARKER;
    storage[Thermal_Shutdown_Threshold] = 0xff; migrate();
    assert(g_thermal_shutdown_enabled && g_thermal_shutdown_threshold == 65);
    testing_migration = false;
    storage.fill(0xff);
    SessionRecord record = {}; record.action = SESSION_STARTED;
    SessionRecord result[16];
    assert(readSessionHistory(result, 16) == 0);
    for(int i = 0; i < 4; ++i) appendSessionHistory(record);
    auto before = storage;
    // Cut power before each write of an overwrite. Prior entries stay readable.
    for(int cut = 0; cut <= 31; ++cut) {
        storage = before; writes_left = cut;
        try { appendSessionHistory(record); } catch(const std::runtime_error&) {}
        unsigned count = readSessionHistory(result, 16);
        assert(count >= 3 && count <= 4);
        assert(result[count - 1].sequence == (cut >= 31 ? 5u : 4u));
        for(unsigned n = 0; n < count; ++n) assert(sessionRecordValid(result[n]));
    }
    storage = before; writes_left = -1;
    for(int i = 0; i < 100; ++i) appendSessionHistory(record);
    assert(readSessionHistory(result, 16) == 4);
    assert(result[0].sequence == 101 && result[3].sequence == 104);
    assert(result[3].flags & SESSION_HISTORY_GAP);
    assert(readSessionHistory(result, 1) == 1 && result[0].sequence == 104);
    // A checksum failure must never create a successful-looking stop.
    for(unsigned offset = sizeof(EE_prom); offset < EEPROM_SIZE; offset += sizeof(SessionRecord))
        storage.at(offset + offsetof(SessionRecord, action)) ^= 0x01;
    assert(readSessionHistory(result, 16) == 0);
}
'''
with tempfile.TemporaryDirectory(prefix='signalslinger-journal-') as temp:
    cpp = Path(temp) / 'journal.cpp'
    cpp.write_text(stubs + journal + '\nvoid migrate() {\n' + migration + '\n}\n' + tests)
    binary = Path(temp) / 'journal-test'
    subprocess.run(['c++', '-std=c++17', '-Wall', '-Wextra', '-Werror', '-I', str(root / 'SignalSlinger/include'), str(cpp), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
print('Session journal: legacy defaults, saved choices, power cuts, wraparound and CRC rejection passed.')
