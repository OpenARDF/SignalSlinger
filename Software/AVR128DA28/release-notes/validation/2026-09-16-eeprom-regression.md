# EEPROM readiness fix: implementation and attached-device verification

Local firmware 2.0.5; HW 3.5, UID 42348279800036200109013300000000, bootloader BL0.13.
The final regression gates passed. The initial runner failure and its correction
are documented below. The attached unit retains the normal fixed firmware.
Original saved schedule and current host-local clock were restored, transmissions
were stopped, and optional tracing was disabled. Bench events remain in the bounded
session history; the original history is preserved in the pre-test snapshot.
At the time of this test run, no commit, push, tag or publication had occurred.

## Narrow production change

`SignalSlinger/src/eeprommanager.cpp` now polls EEPROM readiness before each read
and writes words, double words and floats through the existing byte writer,
waiting between bytes. Polling preserves the caller's interrupt state; already
enabled interrupts continue during the wait. EEPROM persistence remains startup
or foreground work; interrupt handlers queue session persistence for foreground
processing. No interrupts are forcibly enabled. The final byte remains asynchronous.

The EEPROM image/layout, stored representations, ascending byte order, journal
checksum and final magic-byte commit ordering are unchanged. The generated
EEPROM image is byte-identical to the pre-fix image (SHA-256
`1450f5c43bf9e37f9c245b3297d38763f710397a2f5938bdd75fe393479ba22f`).
No event scheduling, button policy, timer configuration, interrupt priorities, or thermal logic changed
in this fix. The scoped difference is retained in `eeprom-fix-only.patch`.

## Before and after on the same device

| Reproduction under continuous 9600-baud receive traffic | Before | Fixed diagnostic build |
| --- | --- | --- |
| Carrier to demo, three independent trials | One overrun per trial; timer gaps 10.83–11.51 ms | No overruns; maximum gap 2.014 ms |
| Stored finish-time write | One overrun; timer gap 10.254 ms | No overruns; maximum gap 2.014 ms |

EEPROM programming still takes approximately 10 ms per byte operation.
The fix prevents that wait from halting interrupt service. For example, the fixed
finish-time write still takes approximately 30 ms overall while the high-priority
timer continues at its ordinary approximately 2 ms interval. These are MCU timing
observations, not external waveform measurements.

Fourteen diagnostic captures passed: three carrier entries, three demo entries,
a finish-time write, idle operation, future carrier/demo entries, running demo
and scheduled Morse, scheduled start, and carrier entry/expiry. Maximum timer gap
across all captures: 2.014 ms. No overflow observations or UART overrun,
framing or parity errors were recorded.

The normal build separately passed seven continuous-traffic reproductions, and
all 24 hardware behavior cases using unpaced command bytes. Every recorded UART
snapshot in all four runs reported zero errors, including setup and restoration
snapshots. The runner verifies applied carrier/demo commands and finish-time
readback, so dropped test commands cannot masquerade as success.

## Automated validation

- `just check`: passed, including storage power-cut/CRC/ring recovery, scheduling,
  button policy/input, UART damaged-line rejection, thermal, battery, ADC and
  latency accounting checks.
- New actual-accessor busy-memory tests: read/write readiness, byte order,
  floating-point representations, enabled/disabled interrupt-state preservation,
  partial writes, and two deliberately broken readiness variants rejected.
- Normal HW 3.4 and HW 3.5 builds, plus diagnostic HW 3.5: zero compiler warnings.
- Generated AVR code reviewed: readiness polling precedes access, and multi-byte
  writers invoke the byte writer for each byte.
- Serial updater verified all 211 diagnostic-image pages and all 205 normal-image
  pages, then confirmed application 2.0.5/HW 3.5 after each flash.

## Normal-firmware behavior cases

1. PASS — hook limits and idle readback
2. PASS — timed short press and carrier cancellation
3. PASS — future carrier-demo-schedule sequence
4. PASS — future triple press clears temporary tests
5. PASS — active triple press advances one day
6. PASS — active off-air long hold cancels today
7. PASS — active on-air long hold cancels today
8. PASS — last-day hold stays canceled after serial wake
9. PASS — future long hold preserves schedule
10. PASS — preview hold release is ignored
11. PASS — timed triple press advances today
12. PASS — scheduled start during hold preserves today
13. PASS — active carrier long hold preserves phase
14. PASS — future demo long hold preserves schedule
15. PASS — 30-second carrier expiry restores phase
16. PASS — demo expiry restores future schedule
17. PASS — carrier expires while held without canceling today
18. PASS — expired schedule manual run is indefinite
19. PASS — clockless manual run and cancellation
20. PASS — natural off-air sleep and RTC wake
21. PASS — timed finish advances normally
22. PASS — manual run outlives expired duration
23. PASS — disabled equal-time schedule manual run
24. PASS — serial replies during an active event

## Retained initial failure and runner correction

The first normal-firmware run passed 18 cases, then the clockless-reset case
failed with a framing error and zero overruns. The startup banner contained the
expected UID, and the old runner incorrectly accepted that as an INF command
response while the device was waking. The runner now detects startup entering
sleep after its first wake pulse and requires a complete INF response (product,
software, bootloader and UID) before proceeding. No firmware change was needed.
The original failed run is retained in `ss-eeprom-normal-full/`.

The isolated retry passed with zero UART errors after reset (its initial snapshot
still contained the previous failed run's one framing error). A new host regression
test rejects banner-only replies, handles startup power-off, and bounds read-only
probe retries. The final full 24-case run was repeated from the beginning and is
retained in `ss-eeprom-normal-full-verified/`; all its UART snapshots were clean.

## Scope limits

The bench observes MCU state, GPIO/synthesizer readback and timing counters.
Physical button wake and contact bounce, RF frequency/power/waveform, visible LEDs,
standby current, real supply interruption and actual thermal trips were not measured.
Power-cut recovery tests are host simulations. HW 3.4 was built but not flashed or
physically tested. These limits remain explicit; they are not passed checks.

## Reproduction and evidence

The runner snapshots, raw timestamped serial logs, JSON results, build evidence,
firmware images, source snapshots and SHA-256 digests are retained beside this
report. The previous failing captures remain in `../serial-latency/`.

Use `just serial-eeprom-regression PORT UID NEW_OUTPUT_DIRECTORY` with the diagnostic
image. For normal firmware use `scripts/run-serial-latency.py` with
`--focus-demo --repeats 3 --require-clean --normal-image`; use `--restore-from`
with a saved pre-flash clock report. The full hardware suite uses
`--extended --byte-gap-ms 0`. No two processes may own the port concurrently.

### Firmware HEX SHA-256

- hw-3-5-latency: `a9afd57e387099bce66c4f61cd5b60a49fd8294f2b917c45eec569aa883c4b75`
- hw-3-5: `60b4f83232955c0bdf81e00f6db7b178823011b6cdc413664d12c06e487b2be2`
- hw-3-4: `5cc66886cc927ffcfd9e5cef765c4aeea48b133d77c06450f8f4163b9966027c`

Raw captures and source snapshots are retained locally under `Software/AVR128DA28/tmp/hardware-bench/2026-09-16/eeprom-fix/`. Generated binaries are not tracked source.
