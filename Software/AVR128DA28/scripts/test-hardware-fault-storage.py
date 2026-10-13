#!/usr/bin/env python3
"""Exercise production hardware-fault slots, CRCs, clearing, and torn writes."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parent.parent
source = (root / "SignalSlinger/src/eeprommanager.cpp").read_text()
main = (root / "SignalSlinger/main.cpp").read_text()
serialbus = (root / "SignalSlinger/src/serialbus.cpp").read_text()
start = source.index("uint8_t hardwareFaultHistoryCapacity")
end = source.index("/**\n * Read a 16-bit value", start)
implementation = source[start:end]

# Keep detection and clearing wired to the durable store. These assertions are
# intentionally source-level because the startup hardware probes are AVR-only.
record_start = main.index("static void recordDetectedHardwareFault(uint8_t code)", main.index("static void recordDetectedHardwareFault(uint8_t code)") + 1)
record_end = main.index("static void reportHardwareFaultHistory", record_start)
record_detection = main[record_start:record_end]
assert "code != HARDWARE_FAULT_RTC && timeIsSet()" in record_detection
assert "recordHardwareFault(code" in record_detection
assert "recordDetectedHardwareFault(HARDWARE_FAULT_RTC);" in main
assert "recordDetectedHardwareFault(HARDWARE_FAULT_TRANSMITTER);" in main
handler_start = main.index("case SB_MESSAGE_HARDWARE_FAULTS:")
handler_end = main.index("case SB_MESSAGE_UI_DIAGNOSTICS:", handler_start)
handler = main[handler_start:handler_end]
assert handler.index("clearHardwareFaultHistory();") < handler.index("recordDetectedHardwareFault(HARDWARE_FAULT_RTC)")
assert handler.index("recordDetectedHardwareFault(HARDWARE_FAULT_TRANSMITTER)") < handler.index("reportHardwareFaultHistory();")
send_start = serialbus.index("bool serialbus_send_text(char *text)")
send_start = serialbus.index("bool serialbus_send_text(char *text)", send_start + 1)
send_end = serialbus.index("/***********************************************************************************", send_start)
send = serialbus[send_start:send_end]
assert "strcmp(text, TEXT_TX_NOT_RESPONDING_TXT) == 0" in send
assert "recordHardwareFault(HARDWARE_FAULT_TRANSMITTER" in send
stubs = r'''
#include "hardware_fault_history.h"
#include <array>
#include <assert.h>
#include <stdexcept>
static const uint8_t HARDWARE_FAULT_HISTORY_SLOTS = HARDWARE_FAULT_MAX;
static const uint16_t HARDWARE_FAULT_HISTORY_START = 370;
static std::array<uint8_t, 512> storage;
static int writes_left = -1;
uint8_t avr_eeprom_read_byte_at(uint16_t offset) { return storage.at(offset); }
void avr_eeprom_write_byte(uint16_t offset, uint8_t value) {
    if(writes_left == 0) throw std::runtime_error("power cut");
    if(writes_left > 0) --writes_left;
    storage.at(offset) = value;
}
'''
tests = r'''
int main() {
    storage.fill(0xff);
    HardwareFaultRecord records[8] = {};
    assert(hardwareFaultHistoryCapacity() == 8);
    assert(readHardwareFaultHistory(records, 8) == 0);
    assert(recordHardwareFault(HARDWARE_FAULT_TRANSMITTER, 1788527000, true));
    assert(!recordHardwareFault(HARDWARE_FAULT_RTC, 0, true));
    assert(!recordHardwareFault(HARDWARE_FAULT_RTC, 1788527000, false));
    assert(readHardwareFaultHistory(records, 8) == 1);
    assert(records[0].code == HARDWARE_FAULT_TRANSMITTER);
    assert(records[0].detected_at == 1788527000);
    assert(records[0].flags == HARDWARE_FAULT_TIME_VALID);
    assert(!recordHardwareFault(HARDWARE_FAULT_TRANSMITTER, 1788529999, true));
    assert(readHardwareFaultHistory(records, 8) == 1);
    assert(records[0].detected_at == 1788527000);
    clearHardwareFaultHistory();
    assert(readHardwareFaultHistory(records, 8) == 0);

    assert(recordHardwareFault(HARDWARE_FAULT_TRANSMITTER, 1788529999, true));
    assert(recordHardwareFault(HARDWARE_FAULT_RTC, 0, false));
    assert(readHardwareFaultHistory(records, 8) == 2);
    assert(records[0].code == HARDWARE_FAULT_RTC && records[0].detected_at == 0 && records[0].flags == 0);
    assert(records[1].code == HARDWARE_FAULT_TRANSMITTER && records[1].detected_at == 1788529999);
    assert(readHardwareFaultHistory(records, 1) == 1 && records[0].code == HARDWARE_FAULT_RTC);

    auto before_clear = storage;
    clearHardwareFaultHistory();
    for(unsigned offset = 0; offset < storage.size(); ++offset) {
        bool fault_magic = false;
        for(unsigned slot = 0; slot < HARDWARE_FAULT_HISTORY_SLOTS; ++slot)
            fault_magic |= offset == HARDWARE_FAULT_HISTORY_START + slot * sizeof(HardwareFaultRecord) + offsetof(HardwareFaultRecord, magic);
        if(!fault_magic) assert(storage[offset] == before_clear[offset]);
    }
    assert(readHardwareFaultHistory(records, 8) == 0);
    assert(recordHardwareFault(HARDWARE_FAULT_TRANSMITTER, 1788531111, true));
    assert(readHardwareFaultHistory(records, 8) == 1 && records[0].detected_at == 1788531111);

    // Even a correctly checksummed version-1 record may not set undefined flags.
    HardwareFaultRecord invalid_flags = records[0];
    auto write_tx = [&](HardwareFaultRecord record) {
        record.checksum = hardwareFaultRecordChecksum(record);
        auto bytes = reinterpret_cast<const uint8_t*>(&record);
        unsigned tx_offset = HARDWARE_FAULT_HISTORY_START + (HARDWARE_FAULT_TRANSMITTER - 1) * sizeof(HardwareFaultRecord);
        for(unsigned i = 0; i < sizeof(record); ++i) storage[tx_offset + i] = bytes[i];
    };
    invalid_flags.flags = 2;
    write_tx(invalid_flags);
    assert(readHardwareFaultHistory(records, 8) == 0);

    HardwareFaultRecord invalid_time = invalid_flags;
    invalid_time.flags = HARDWARE_FAULT_TIME_VALID;
    invalid_time.detected_at = 0;
    write_tx(invalid_time);
    assert(readHardwareFaultHistory(records, 8) == 0);
    invalid_time.flags = 0;
    invalid_time.detected_at = 1788531111;
    write_tx(invalid_time);
    assert(readHardwareFaultHistory(records, 8) == 0);

    for(int cut = 0; cut <= 11; ++cut) {
        storage.fill(0xff); writes_left = cut;
        try { recordHardwareFault(HARDWARE_FAULT_RTC, 0, false); } catch(const std::runtime_error&) {}
        unsigned count = readHardwareFaultHistory(records, 8);
        assert(count == (cut >= 11 ? 1u : 0u));
    }
    assert(!recordHardwareFault(0, 0, false));
    assert(!recordHardwareFault(9, 0, false));
}
'''
with tempfile.TemporaryDirectory(prefix="signalslinger-faults-") as temp:
    cpp = Path(temp) / "faults.cpp"
    cpp.write_text(stubs + implementation + tests)
    binary = Path(temp) / "faults-test"
    subprocess.run(["c++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-I", str(root / "SignalSlinger/include"), str(cpp), "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
print("Hardware fault history: detection wiring, fixed slots, first detection, clear isolation, validation, CRC and power cuts passed.")
