#!/usr/bin/env python3
"""Run production EEPROM accessors against a busy-memory/interrupt model."""
from pathlib import Path
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parent.parent
source = (root / 'SignalSlinger/src/eeprommanager.cpp').read_text()
helpers = source[source.index('typedef uint16_t eeprom_addr_t;'):source.index('/* Append-only journal')]
for kind in ('word', 'dword', 'float'):
    match = re.search(r'static \w+ avr_eeprom_read_' + kind + r'_at\(.*?\n}', source, re.S)
    assert match
    helpers += '\n' + match.group()
helpers = helpers.replace('typedef uint16_t eeprom_addr_t;', 'typedef uintptr_t eeprom_addr_t;')
store = '*(volatile uint8_t *)(eeprom_addr_t)(MAPPED_EEPROM_START + index) = in;'
assert helpers.count(store) == 1
helpers = helpers.replace(store, 'mapped_write(index, in);')
model = r'''
#include <array>
#include <vector>
#include <cstring>
#include <cstdint>
#include <cassert>
#include <stdexcept>
#define SERIAL_LATENCY_SCOPE(id) ((void)0)
#define NVMCTRL_EEBUSY_bm 1
#define NVMCTRL_CMD_EEERWR_gc 2
#define NVMCTRL_CMD_NONE_gc 0
#define _PROTECTED_WRITE_SPM(reg, value) ((reg) = (value))
static std::array<uint8_t, 512> memory;
static std::vector<uintptr_t> addresses;
static unsigned pending_ticks, serviced_interrupts;
static bool interrupts_enabled;
static int writes_left = -1;
struct Status {
    operator uint8_t() const {
        if(!pending_ticks) return 0;
        --pending_ticks;
        if(interrupts_enabled) ++serviced_interrupts;
        return NVMCTRL_EEBUSY_bm;
    }
};
static struct { Status STATUS; uint8_t CTRLA; } NVMCTRL;
static void mapped_write(uintptr_t index, uint8_t value) {
    if(pending_ticks) throw std::runtime_error("mapped write while busy");
    assert(NVMCTRL.CTRLA == NVMCTRL_CMD_EEERWR_gc);
    if(writes_left == 0) throw std::runtime_error("power cut");
    if(writes_left > 0) --writes_left;
    addresses.push_back(index); memory.at(index) = value; pending_ticks = 3;
}
template<class T> static T read_memory(const T* address) {
    if(pending_ticks) throw std::runtime_error("mapped read while busy");
    uintptr_t index = reinterpret_cast<uintptr_t>(address);
    assert(index + sizeof(T) <= memory.size());
    T value; std::memcpy(&value, memory.data() + index, sizeof(value)); return value;
}
#define eeprom_read_byte read_memory<uint8_t>
#define eeprom_read_word read_memory<uint16_t>
#define eeprom_read_dword read_memory<uint32_t>
#define eeprom_read_float read_memory<float>
'''
tests = r'''
int main() {
    static_assert(sizeof(float) == 4, "IEEE binary32 fixture required");
    const uint32_t endian = 1;
    assert(*reinterpret_cast<const uint8_t*>(&endian) == 1); // AVR byte order.
    for(bool enabled : {false, true}) {
        interrupts_enabled = enabled; serviced_interrupts = 0; pending_ticks = 3;
        addresses.clear(); memory.fill(0xff);
        avr_eeprom_write_byte(5, 0xa5); assert(pending_ticks);
        assert(avr_eeprom_read_byte_at(5) == 0xa5);
        avr_eeprom_write_word(16, 0x1234); assert(pending_ticks);
        assert(avr_eeprom_read_word_at(16) == 0x1234);
        assert(memory[16] == 0x34 && memory[17] == 0x12);
        avr_eeprom_write_dword(32, 0x12345678); assert(pending_ticks);
        assert(avr_eeprom_read_dword_at(32) == 0x12345678);
        assert(memory[32] == 0x78 && memory[33] == 0x56 && memory[34] == 0x34 && memory[35] == 0x12);
        assert((addresses == std::vector<uintptr_t>{5,16,17,32,33,34,35}));
        for(uint32_t bits : {0x3f800000u, 0x80000000u, 0x7f800000u, 0x7fc12345u}) {
            float value; std::memcpy(&value, &bits, 4);
            avr_eeprom_write_float(48, value); assert(pending_ticks);
            float actual = avr_eeprom_read_float_at(48); uint32_t actual_bits;
            std::memcpy(&actual_bits, &actual, 4); assert(actual_bits == bits);
        }
        // Back-to-back multi-byte writes and an immediate different-size read.
        avr_eeprom_write_dword(64, 0xabcdef01);
        avr_eeprom_write_word(68, 0x2345);
        assert(avr_eeprom_read_byte_at(68) == 0x45);
        assert(avr_eeprom_read_dword_at(64) == 0xabcdef01);
        assert(interrupts_enabled == enabled);
        assert(enabled ? serviced_interrupts > 30 : serviced_interrupts == 0);
        assert(NVMCTRL.CTRLA == NVMCTRL_CMD_NONE_gc);
    }
    // Multi-byte settings retain the same ascending-byte partial-write shape.
    // Journal commit/checksum recovery is exercised by test-session-storage.py.
    for(int cut = 0; cut <= 4; ++cut) {
        pending_ticks = 0; memory.fill(0xff); addresses.clear(); writes_left = cut;
        try { avr_eeprom_write_dword(80, 0x12345678); }
        catch(const std::runtime_error& e) { assert(std::strcmp(e.what(), "power cut") == 0); }
        assert(addresses.size() == static_cast<unsigned>(cut));
        const uint8_t expected[] = {0x78,0x56,0x34,0x12};
        for(int i = 0; i < 4; ++i) assert(memory[80+i] == (i < cut ? expected[i] : 0xff));
    }
}
'''
with tempfile.TemporaryDirectory(prefix='signalslinger-eeprom-') as temp:
    cpp = Path(temp) / 'access.cpp'
    binary = Path(temp) / 'access-test'
    # Removing either readiness guard must fail: this test detects the original
    # read-after-write stall and missing inter-byte write waits independently.
    mutants = [helpers,
        helpers.replace('avr_eeprom_wait_ready();\n\treturn eeprom_read_', 'return eeprom_read_'),
        helpers.replace('avr_eeprom_wait_ready();\n\t_PROTECTED_WRITE_SPM', '_PROTECTED_WRITE_SPM')]
    for index, variant in enumerate(mutants):
        assert index == 0 or variant != helpers
        cpp.write_text(model + variant + tests)
        subprocess.run(['c++', '-std=c++17', '-Wall', '-Wextra', '-Werror', str(cpp), '-o', str(binary)], check=True)
        result = subprocess.run([str(binary)], capture_output=True)
        assert (result.returncode == 0) == (index == 0), result.stderr.decode()
print('EEPROM access: busy guards, byte order, float bits, interrupt state, partial writes and two regression mutants passed.')
