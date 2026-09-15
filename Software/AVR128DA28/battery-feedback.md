# External battery feedback

The green LED uses the most recent confirmed external-battery voltage. A healthy
battery can therefore produce steady green even while its power-control cable is
deliberately switched off. The existing internal-battery warning and charging
indications still apply. This is a voltage indication, not a capacity estimate.

Awake firmware uses the existing periodic ADC scheduler:

- A transmission or charging power-on requests a fresh external measurement after
  at least 10 ms of settling, even when the LEDs are dark.
- With visible LEDs and power already requested, external voltage is measured
  approximately once per second.
- With visible LEDs and managed external power disconnected, a temporary
  measurement client enables the battery, waits at least 10 ms, samples, and
  releases its request. This repeats approximately every ten seconds. It never
  enables the RF chain. Charging or transmission can retain power independently.
- LED timeout cancels temporary probes and disables the faster polling. Normal
  awake monitoring of an enabled supply keeps its approximately eight-second
  interval. The legacy timer constants are not changed for other services.
- With external battery control disabled, measurements never pulse the auxiliary
  output, which may be used for the HW 3.4 fan.

The periodic service completes a conversion on a subsequent 300 Hz timer tick,
so a normal probe lasts about 13 ms, with extra delay if another conversion is
in progress. A lost ADC
conversion releases the temporary request; a separate 100 ms request deadline
bounds a probe even when the ADC scheduler cannot start its conversion.

`BAT` reports the last confirmed external measurement rather than sampling the
intentionally disconnected jack. Its internal measurement and foreground
temperature reads temporarily own the ADC; any displaced periodic conversion is
retried, so results are not attributed to the wrong channel.

The ADC allows 16 ADC clocks for initialization after enabling its reference.
Temperature-channel acquisitions use 11 sample-length clocks at the 375 kHz ADC
clock (29.3 microseconds), meeting the AVR128DA sensor's 25-microsecond startup
and 28-microsecond sampling requirements. Voltage channels restore the short
sampling window. The input is selected before enabling the ADC/reference so
the initialization delay cannot retain the previous input for the first sample.
This prevents immediate `TMP` queries after ADC restart from
reporting false heat and changing the fan state. Sampling cadence and sleep
shutdown remain unchanged. `scripts/test-adc-temperature.py` exercises the
production driver across cold starts, channel switches, and shutdown.

Peripheral initialization leaves channel preparation pending. The shared channel
selector then establishes the requested input before enabling the ADC, even
when startup or standby charging code initialized it first. Blocking voltage and
temperature reads reuse one conversion helper: it aborts any displaced sample,
waits for a fresh result with a bounded counter, and disables the ADC on timeout.
Voltage failures return zero and temperature failures return an invalid value;
neither accepts the previous conversion as a successful sample. Every new
conversion clears the old result-ready flag. Callers retain their existing
interrupt guards and periodic-conversion restart notification.

The existing sleep loop, approximately 90-second sleep probe, charging decisions,
and wake schedule are unchanged. Awake probes are canceled before sleep and do
not wake the device or extend the LED timeout. No new settings are persisted.

The 10 ms settling time is an initial design allowance. Physical settling and RF
behavior on modified battery-control hardware still require bench verification.
Run `just test` for deterministic timing, arbitration, ADC recovery, and sleep
boundary checks, and `just avr-relocated-build-hw 3.4` / `3.5` for both firmware
targets.
