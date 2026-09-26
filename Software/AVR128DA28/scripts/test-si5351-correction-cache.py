#!/usr/bin/env python3
"""Verify that a new Si5351 correction invalidates the cached PLLB plan."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parent.parent
source = (root / "SignalSlinger/src/si5351.cpp").read_text()
setter_start = source.index("void si5351_set_correction(int32_t corr)")
setter_end = source.index("/**\n * Return the currently configured", setter_start)
setter = source[setter_start:setter_end]

# The following set_freq branch is what turns an invalid cache into a fresh PLL
# calculation and register write. Keep the behavioral test tied to that contract.
set_freq_start = source.index("bool si5351_set_freq(")
set_freq_end = source.index("Frequency_Hz si5351_get_frequency", set_freq_start)
set_freq = source[set_freq_start:set_freq_end]
assert "(target_pll == SI5351_PLLA) || !freqVCOB" in set_freq
assert "set_pll(freq_VCO, target_pll)" in set_freq

harness = r'''
#include <cassert>
#include <cstdint>
using Frequency_Hz = uint32_t;
static int32_t g_si5351_ref_correction = 0;
static Frequency_Hz freqVCOB = 900000000UL;
'''

tests = r'''
int main() {
    si5351_set_correction(-20000);
    assert(g_si5351_ref_correction == -20000);
    assert(freqVCOB == 0);

    freqVCOB = 864000000UL;
    si5351_set_correction(0);
    assert(g_si5351_ref_correction == 0);
    assert(freqVCOB == 0);
}
'''

with tempfile.TemporaryDirectory(prefix="signalslinger-si5351-correction-") as temp:
    cpp = Path(temp) / "correction-cache.cpp"
    binary = Path(temp) / "correction-cache-test"
    cpp.write_text(harness + setter + tests)
    subprocess.run(
        [
            "c++",
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Werror",
            str(cpp),
            "-o",
            str(binary),
        ],
        check=True,
    )
    subprocess.run([str(binary)], check=True)

print("Si5351 correction cache: correction changes force the next PLLB frequency set to rebuild registers.")
