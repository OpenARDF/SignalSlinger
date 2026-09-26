#!/usr/bin/env python3
"""Exercise the production RF calibration transaction with a stubbed Si5351."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parent.parent
source = (root / "SignalSlinger/src/transmitter.cpp").read_text()
start = source.index("bool txSetCalibrationFrequency")
end = source.index("/**\n * Globally enable or disable", start)
runtime = source[start:end]

stubs = r'''
#include <cassert>
#include <cstdint>
#include "rf_calibration.h"
using Frequency_Hz = uint32_t;
enum { TX_CLOCK_HF_0 = 1 };
static bool g_tx_initialized = false;
static Frequency_Hz g_80m_frequency = 3520000UL;
static Frequency_Hz g_calibration_frequency_hz = RF_CALIBRATION_DEFAULT_FREQUENCY_HZ;
static Frequency_Hz g_active_calibration_frequency_hz = RF_CALIBRATION_DEFAULT_FREQUENCY_HZ;
static bool g_calibration_carrier_active = false;
static bool g_calibration_restore_initialized = false;
static bool g_calibration_restore_keyed = false;
static bool g_calibration_restore_valid = false;
static bool g_transmitter_keyed = false;
static int32_t stub_correction_ppb = 0;
static Frequency_Hz stub_programmed_frequency = g_80m_frequency;
static bool fail_next_set = false;
static bool fail_next_key = false;
static bool fail_next_power = false;
static unsigned key_transitions = 0;
static unsigned power_transitions = 0;
void si5351_set_correction(int32_t correction) { stub_correction_ppb = correction; }
int32_t si5351_get_correction() { return stub_correction_ppb; }
bool si5351_set_freq(Frequency_Hz frequency, int, bool) {
    if(fail_next_set) { fail_next_set = false; return true; }
    stub_programmed_frequency = frequency; return false;
}
bool keyTransmitter(bool on);
bool powerToTransmitter(bool on);
'''

tests = r'''
bool keyTransmitter(bool on) {
    if(fail_next_key) { fail_next_key = false; return g_transmitter_keyed; }
    ++key_transitions; g_transmitter_keyed = on; return g_transmitter_keyed;
}
bool powerToTransmitter(bool on) {
    ++power_transitions;
    if(fail_next_power) { fail_next_power = false; return false; }
    g_tx_initialized = on;
    g_transmitter_keyed = false;
    if(on) stub_programmed_frequency = g_80m_frequency;
    else {
        g_calibration_carrier_active = false;
        g_calibration_restore_valid = false;
    }
    return true;
}
int main() {
    assert(!txSetCalibrationFrequency(10000099UL));
    assert(txGetCalibrationFrequency() == 10000000UL);
    assert(stub_programmed_frequency == g_80m_frequency);
    assert(!txApplyCalibrationOffsetHz(10));
    assert(txCalibrationCarrierActive());
    assert(g_tx_initialized && g_transmitter_keyed);
    assert(stub_programmed_frequency == 10000000UL);
    assert(txGetCalibrationCorrectionPpb() == -1000);
    assert(txGetCalibrationOffsetHz() == 10);
    assert(power_transitions == 1 && key_transitions == 1);
    assert(!txExitCalibrationCarrier());
    assert(!g_tx_initialized && !g_transmitter_keyed && !txCalibrationCarrierActive());
    assert(stub_programmed_frequency == g_80m_frequency);
    assert(power_transitions == 2 && key_transitions == 2);

    key_transitions = power_transitions = 0;
    assert(!txRestoreCalibrationCorrectionPpb(-20000));
    assert(!txSetCalibrationFrequency(15000000UL));
    assert(!txStartCalibrationCarrier());
    assert(txCalibrationCarrierActive());
    assert(stub_programmed_frequency == 15000000UL);
    assert(txGetCalibrationCorrectionPpb() == -20000);
    assert(txGetCalibrationOffsetHz() == 300);
    assert(power_transitions == 1 && key_transitions == 1);
    assert(!txSetCalibrationFrequency(10000000UL));
    assert(!txStartCalibrationCarrier());
    assert(stub_programmed_frequency == 10000000UL);
    assert(txGetCalibrationCorrectionPpb() == -20000);
    assert(txGetCalibrationOffsetHz() == 200);
    assert(power_transitions == 1 && key_transitions == 3);
    assert(!txExitCalibrationCarrier());
    assert(!g_tx_initialized && !g_transmitter_keyed && !txCalibrationCarrierActive());
    assert(power_transitions == 2 && key_transitions == 4);

    g_tx_initialized = true;
    key_transitions = power_transitions = 0;
    assert(!txSetCalibrationFrequency(20000000UL));
    assert(!txApplyCalibrationOffsetHz(-20));
    assert(stub_programmed_frequency == 20000000UL);
    assert(txGetCalibrationCorrectionPpb() == 1000);
    assert(g_transmitter_keyed && key_transitions == 1 && power_transitions == 0);
    assert(!txExitCalibrationCarrier());
    assert(g_tx_initialized && !g_transmitter_keyed);
    assert(stub_programmed_frequency == g_80m_frequency && key_transitions == 2);

    g_transmitter_keyed = true;
    key_transitions = 0;
    assert(!txSetCalibrationFrequency(30000000UL));
    assert(!txApplyCalibrationOffsetHz(30));
    assert(g_transmitter_keyed && key_transitions == 2);
    assert(!txExitCalibrationCarrier());
    assert(g_tx_initialized && g_transmitter_keyed && key_transitions == 4);

    g_transmitter_keyed = false;
    assert(!txSetCalibrationFrequency(20000000UL));
    assert(!txApplyCalibrationOffsetHz(-20));
    const int32_t active_correction = txGetCalibrationCorrectionPpb();
    const Frequency_Hz active_frequency = stub_programmed_frequency;
    assert(!txSetCalibrationFrequency(30000000UL));
    fail_next_set = true;
    assert(txApplyCalibrationOffsetHz(30));
    assert(txCalibrationCarrierActive());
    assert(stub_programmed_frequency == active_frequency);
    assert(txGetCalibrationCorrectionPpb() == active_correction);
    assert(g_transmitter_keyed);

    fail_next_set = true;
    assert(txExitCalibrationCarrier());
    assert(txCalibrationCarrierActive());
    assert(stub_programmed_frequency == active_frequency);
    assert(g_transmitter_keyed);

    assert(!txExitCalibrationCarrier());
    assert(!txCalibrationCarrierActive());
    assert(stub_programmed_frequency == g_80m_frequency);

    g_tx_initialized = false;
    fail_next_power = true;
    assert(txApplyCalibrationOffsetHz(30));
    assert(!g_tx_initialized && !g_transmitter_keyed && !txCalibrationCarrierActive());

    const int32_t correction_before_key_failure = txGetCalibrationCorrectionPpb();
    fail_next_key = true;
    assert(txApplyCalibrationOffsetHz(30));
    assert(!g_tx_initialized && !g_transmitter_keyed && !txCalibrationCarrierActive());
    assert(txGetCalibrationCorrectionPpb() == correction_before_key_failure);

    assert(!txRestoreCalibrationCorrectionPpb(-1000000));
    assert(txRestoreCalibrationCorrectionPpb(-1000001));
}
'''

with tempfile.TemporaryDirectory(prefix="signalslinger-rf-calibration-") as temp:
    cpp = Path(temp) / "runtime.cpp"
    binary = Path(temp) / "runtime-test"
    cpp.write_text(stubs + runtime + tests)
    subprocess.run(
        [
            "c++",
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-I",
            str(root / "SignalSlinger/include"),
            str(cpp),
            "-o",
            str(binary),
        ],
        check=True,
    )
    subprocess.run([str(binary)], check=True)

print("RF calibration runtime: idle power-up, keyed carrier, rollback, exit and entry-state restoration passed.")
