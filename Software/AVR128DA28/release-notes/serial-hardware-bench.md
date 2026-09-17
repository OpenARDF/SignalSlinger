# Serial hardware regression bench

This bench runs production firmware on an identified attached **HW 3.5 Classic
Fox 3**, using its RTC, interrupt handlers, scheduler, serial transport, and GPIO
readback. It changes the RTC and schedule temporarily. It leaves transmissions
stopped, restores the original saved schedule and the computer's local wall clock,
and disables tracing on normal completion, assertion failure, or SIGTERM.

## Commands

Existing commands remain compatible: `UI S` reports UI/clone state, `UI P n`
queues a counted-button action directly to the foreground dispatcher, and `UI C`
clears clone UI latches. New commands extend that interface:

| Command | Function |
| --- | --- |
| `UI D` | Bench capability version, coherent state snapshot, LED/GPIO and synthesizer state, pending input, temperature freshness, runtime and last button-action/wake diagnostics. |
| `UI B 1-750` | Hold a simulated button closed for this many ordinary button-sampling periods, then release. A period is nominally 20 ms; 10 is a short press, 100 exercises hold preview, and 250 exceeds the normal long-hold threshold. |
| `UI T 1-30` | Clear and enable a 32-record RAM ring at the requested interval in RTC seconds. |
| `UI T 0` | Stop recording, preserving retained samples. |
| `UI T` | Freeze and print the ring in chronological order, ending with `UI T end`. |

`UI B` enters the normal button detector after physical debouncing. It exercises
press counting, hold preview, hold intent, long-hold thresholds, and foreground
handling. It does **not** simulate electrical bounce, the physical wake interrupt,
or hold-to-wake authorization. It refuses overlapping injection, a physically
closed button, sleeping/not-yet-authorized operation, pending sleep, master/clone
mode, and already pending button commands. It releases automatically and preserves
the pending release across standby so subsequent holds re-arm normally.

Synthetic input and timer tracing are off after reset; UART error counters start at zero. The hooks do not force transmitter
outputs, spoof temperature or RTC ticks, override thermal protection, or write
EEPROM. Normal user commands used to configure tests do write settings and session
history. Trace capture piggybacks on the existing one-second RTC interrupt, with
no serial output, ADC work, or extra scheduled wake-ups. A dump freezes recording;
it does not provide an endless live stream. At interval 10 it retains about five
minutes of samples; older samples are overwritten. Snapshots are scheduler/GPIO
observations, not calibrated RF, optical, or power measurements.

`UI D` also reports saturating UART overrun, framing, and parity error counts
since reset. Damaged lines are discarded through their terminator and produce an explicit
serial-receive error; a truncated setting must never execute. Clean lines retain
the existing parser behavior. The standard `just hardware-bench` recipe sends unpaced command bytes. The
underlying script retains its conservative 10 ms default for investigations;
`--byte-gap-ms 0` explicitly requests the unpaced regression run. Pacing was the initial diagnostic workaround for EEPROM-induced CPU stalls.
The EEPROM fix waits for readiness before reads and between multi-byte writes,
allowing already-enabled interrupts to continue. Use unpaced traffic and the
EEPROM regression gate below to verify the fix; paced success alone is insufficient. Errors are retained in evidence, never silently retried. The runner uses a kernel-exclusive serial handle on macOS
and stops on incomplete or error replies.

Snapshot/trace `at` uses device wall-clock epoch fields, **not UTC**. `onair` is the
signed slot countdown; `sleep` uses the existing runtime enum; `day` is zero-based;
`key` and `demo` are remaining 300-Hz timer ticks. `flags` is a bit mask:

| Bit value | Meaning |
| --- | --- |
| 1 | MCU is in the foreground standby loop |
| 2 | Event enabled |
| 4 | Event commenced |
| 8 | Manually launched event |
| 16 | Run until canceled |
| 32 | Last confirmed synthesizer key state |
| 64 | Synthesizer initialized |
| 128 / 256 | Red / green LED pin readback high |
| 512 | Thermal shutdown latched |
| 1024 | Current-day cancellation latched |
| 2048 | Wake authorization complete |

## Running

Use the existing serial update script after `just check` and the target firmware
build. Do not open a second serial owner. The runner checks the full requested UID,
hardware, bench capability, and Classic Fox 3 profile before changing settings.
It uses pyserial and records timestamped raw traffic plus structured case results.
Wake synchronization requires a complete INF command response; a startup banner
containing the device UID is insufficient. If startup enters sleep after the
first wake pulse, the runner sends another wake pulse before probing. Read-only
probe retries are bounded and logged; setting-changing commands are never replayed.

```
just hardware-bench /dev/cu.usbserial-DEVICE EXPECTED_FULL_UID /absolute/evidence/path
```

The recipe includes the longer quiet RTC sleep/wake and duration tests. Use the
script's `--case` option to reproduce one named scenario, or `--start-at` and
`--stop-before` to select a range. `--restore-from` accepts
a previously saved `CLK` report, useful when a firmware update reset the clock.
The original scheduled run must be stopped before dedicating a device to this
bench; this runner restores configuration and a stopped state, not prior live
execution or the prior history ring. Test timestamps use a fixed synthetic calendar
through the ordinary clock-setting protocol, while elapsed delays use real time.

On failure, read `results.json` and `serial.jsonl`; the restore result is explicit.
An abrupt host crash, power cut, or SIGKILL cannot execute software cleanup. For
fully unattended fault/soak work, add an independent hardware power/RF cutoff.
The framework does not erase history or alter calibration, callsign, frequency,
battery thresholds, thermal settings, or firmware version.

## Limits

Physical button wake and bounce, visible LED output, RF frequency/power/spectrum,
standby current, supply brownouts, and actual thermal trips require the corresponding
connected external instruments or fixture. Firmware pin/synthesizer readback is
valuable evidence but cannot establish those measurements. An unavailable instrument
is a coverage gap, never a passed test. A second device and HW 3.4 are required for
physical clone/mixed-board coverage. Both firmware targets still receive build checks.

## Optional serial latency diagnostic build

`just avr-latency-build-hw 3.5` builds a separate image in
`tmp/avr-release-relocated-hw-3-5-latency`. This enables
`SIGNALSLINGER_LATENCY_DIAGNOSTICS`; normal builds compile every timing scope out.
When first added, these hooks left the normal HW 3.5 image byte-for-byte
identical to its predecessor; subsequent functional fixes intentionally change
that image. The diagnostic image adds execution
overhead and must be identified separately in test evidence.

After installing that image with the existing serial updater:

| Command | Diagnostic behavior |
| --- | --- |
| `UI L 1` | Clear bounded timing/fault records and enable capture. |
| `UI L 0` | Disable capture, preserving records. |
| `UI L` | Freeze and report metrics plus the first 16 overflow observations. |

No timer configuration, interrupt priority, ADC, EEPROM, or scheduled wake-up is
added by the instrumentation itself. It reads the existing 32768-Hz RTC counter;
one tick is approximately 30.5 microseconds. Scope times wrap after one RTC
period (about one second), so this is a short-latency diagnostic, not a general
long-operation profiler. Captures must not span explicit clock resets. A scope's
elapsed time includes interrupts that preempt it; a long foreground operation is
not automatically evidence of interrupt starvation.

Metric IDs: 0 RTC handler, 1 TCB0 handler, 2 LED handler, 3 high-priority TCB2
handler, 4 key control, 5 I2C write, 6 I2C read, 7 EEPROM byte write, 8 EEPROM word
write, 9 EEPROM dword write, 10 EEPROM float write, 11 RX parser, 12 EEPROM read,
13 history append. `max` and `irq_max` are RTC ticks; `busy` counts entries while
EEPROM was busy; `over2ms` counts scopes lasting at least 66 ticks. TCB2's `gap`
is the longest entry-to-entry interval, normally approximately 2 ms in this build.
The scope-mask bit at position N denotes an active scope with ID N.

Fault observations require both an unread byte (`RXCIF`) and its overflow flag;
old RXDATAH error bits without RXCIF are stale, not additional overruns. Fault
records and the saturating UART error counters are observations, not a count of
individual lost bytes. `UI L 1` does not reset the UART counters.

`just serial-latency-bench PORT FULL_UID NEW_OUTPUT_DIRECTORY` runs the broader
capture sequence. The script also supports `--focus-demo --repeats 3` to repeat
the carrier-to-demo transition and capture a timed-setting write. It reuses the
hardware bench's identity checks, exclusive port, evidence log, and restoration.
The receive load consists of continuous comment text, avoiding a flood of queued
commands and replies. Selected captures prefix that text with an explicit button
or timer-setting command. A successful capture means the investigation completed;
read `latency.json` and the raw traffic to determine whether overruns occurred.
Restore the normal image and the current clock after the diagnostic session.

## EEPROM serial regression gate

`just serial-eeprom-regression PORT FULL_UID NEW_OUTPUT_DIRECTORY` repeats the
previously failing carrier-to-demo transition three times and changes a stored
finish time, under continuous 9600-baud receive traffic. It requires a diagnostic
image, zero UART overrun/framing/parity counters, no captured overflow events, and
a maximum high-priority timer service interval below 4 ms (normally about 2 ms).
It also verifies carrier/demo entry and the saved finish time, so a dropped
command cannot produce a false pass.

The underlying script accepts `--normal-image --require-clean` to exercise the
same traffic and state/error checks on ordinary firmware, omitting optional
timing commands. `--require-clean` can also be used for its broader sequence,
which covers scheduled-start and carrier-expiry transitions. Capturing evidence
without this option remains an investigation, not a zero-error regression pass.

The low-level `test-eeprom-access.py` test runs the production accessor bodies
against a busy-memory model, checking read/write readiness, ascending byte order,
floating-point bit preservation, enabled/disabled interrupt state, and partial
writes. Removing either readiness guard must fail the test. The separate journal
test covers power cuts, ring wraparound and checksum rejection; host simulation
does not substitute for physical supply-interruption testing.
