# Session recovery and history (EVT version 1)

SignalSlinger firmware 2.0.4 defaults thermal protection to enabled at 65 C. Legacy, erased and unknown EEPROM choice bytes migrate to that default. Explicit enabled (0xA5) and disabled (0x5A) choices saved by this firmware are preserved. Existing configuration offsets remain unchanged.

Scheduled sessions pause with RF off on overheating or loss of a fresh temperature reading. After overheating, they resume in the same window when both the latest and filtered temperatures are at or below threshold minus 5 C. Recovery also requires a fresh reading and normal schedule and user authorization. The original finish time and later days remain unchanged. Manual test sessions stop and require another user command. Normal Morse key-up and off-air slots are not session interruptions.

Calendar progress is separate from successful execution: passing a finish time reduces remaining windows but does not prove completion. A session that resumed after a pause finishes with interruptions. A session still paused at its finish is interrupted. A past window without an observed finish is expired, with completion unconfirmed.

`EVT` retains its older human-readable replies and adds:

```text
* Session state: v=1 action=2 reason=2 remaining=3 blocked=1
* Session history: v=1 count=1 capacity=7
* Session record: v=1 seq=42 base=1788526800 start=1788526800 finish=1788559200 at=1788527000 action=2 reason=2 flags=7 temp=651 limit=65
```

- Actions: 0 waiting/idle; 1 started; 2 paused; 3 resumed; 4 completed; 5 finished with interruptions; 6 interrupted; 7 expired.
- Reasons: 0 unavailable; 1 scheduled finish; 2 overheating; 3 user stop; 4 settings changed; 5 power loss/reset; 6 clock changed; 7 temperature unavailable; 8 transmissions disabled; 9 test timer finished.
- `remaining` counts current/future calendar windows, including a paused current window. It never counts successful days.
- `base`, `start`, `finish`, `at` encode the device wall-clock fields as epoch seconds. They are not UTC instants: decode their calendar fields without applying the phone or desktop timezone offset. `base` identifies the saved schedule; `start`/`finish` identify one daily window. Manual key tests use base zero. `at=0` with the valid-time flag clear means unknown time, especially for an unexpected power loss/reset. A reset can establish interruption but cannot establish the physical power-loss time.
- `temp` is the safety temperature in tenths of C (the higher of the fresh raw and filtered reading), or -32768 when unavailable; `limit` is C.
- Flag bits: 1 valid device time; 2 this window has an interruption; 4 scheduled session; 128 earlier history unavailable.
- The unused EEPROM tail holds seven recent 30-byte records on AVR128DA28. Sequence numbers wrap at 32 bits; chronological comparison uses signed differences. Each record has version and CRC checks, with its valid marker committed last. Rotation and RAM queue overflow mark history as incomplete. This is a recent history, not a permanent event archive.
- A boot following an active persisted record adds an interrupted/reset record with unknown time. No record claims continuous RF output: these are scheduler lifecycle observations, not an RF power measurement.

SerialSlinger accepts only version 1 records, deduplicates sequences within the connected device, clears history on identity changes and shows reasons and device times on Android and desktop. Older firmware continues to work; calendar-only results say expired with completion unconfirmed. Completion applies to the last recorded session, not all configured days. Full received records also remain in normal app logs.

Validation: `just test` covers thermal policy, legacy migration, the actual EEPROM journal with simulated power cuts, and production runtime/scheduler helpers under a deterministic clock. Hardware temperature/fan response and actual RF output still require device testing.

## Manual runs and button-wake diagnostics

An authorized button wake preserves an existing manual run's loaded window and
cycle countdown, independently of whether the saved calendar schedule has expired.
Indefinite runs also survive when Start equals Finish or the clock is unset. A wake
is not a new start: it must not extend the finish, restart the cycle, or revive a
canceled, disabled, or thermally stopped manual run. Calendar-driven launches retain
their existing scheduling behavior.

`EVT` also includes additive diagnostic lines in the normal received serial log:

```text
* Runtime: v=1 start=1789516800 finish=1789580208 onair=-180 sleep=1 enabled=1 commenced=1 manual=1 forever=0 day=3 days=3
* Button wake: v=1 count=1 at=1789547940
* Wake before: v=1 start=1789516800 finish=1789580208 onair=-180 sleep=2 enabled=1 commenced=1 manual=1 forever=0 day=3 days=3
* Wake after: v=1 start=1789516800 finish=1789580208 onair=-180 sleep=1 enabled=1 commenced=1 manual=1 forever=0 day=3 days=3
```

`Runtime` is the current event-engine state, separately from session history.
`Wake before` and `Wake after` capture event restoration at the end of the most
recent successful button-wake authorization, not the physical button edge.
`count` counts authorized button wakes since reset (saturating at 4294967295);
zero means no retained wake and omits both snapshots. `at` uses the same device
wall-clock encoding as session records, or zero when the clock is unset.

`onair` is the signed seconds countdown: negative before a transmit slot, positive
inside a slot, zero at its boundary. It does not establish measured RF output.
`sleep` is 0 waiting for scheduled start, 1 active/after-event mode, 2 between
transmissions, 3 indefinite sleep, or 4 forced power-off. `enabled` and `commenced`
are event-engine flags; `manual` identifies a user-launched run, and `forever`
identifies a run without a timed finish. `day` is the saved calendar's zero-based
progress index and `days` its configured count; exhausted calendar days do not
invalidate a separately launched manual run.

The last wake snapshots are bounded RAM diagnostics, lost on reset, and are only
printed when `EVT` is requested. Capturing them adds no EEPROM writes, ADC samples,
serial traffic, or extra wake-ups. Normal timer and serial wake paths are unchanged.
The existing `just test` session-runtime test now exercises the production calendar
and wake helpers together, with manual and scheduled controls, cancellation,
temperature stops, repeated wakes, and exact finish boundaries. Actual RF and
physical button timing remain bench checks.


## Button cancellation and expired schedules

A new synchronized manual run uses a saved duration only while a valid current or
future calendar window remains. An unset, invalid, or exhausted schedule cannot
limit that run: it runs until explicitly canceled, aligned to the clock when the
clock is valid, or relative to the manual start otherwise. Saved settings are not
erased. An already-running manual event keeps its original finish across wakes.

| Button operation | Result |
| --- | --- |
| Hold while asleep, or while awake with LEDs timed out | Authorize wake or revive LEDs; release before issuing another command. |
| Awake hold during an ordinary active calendar event, on-air or off-air | Cancel today's event, advance to the next configured day if present, and sleep. |
| Awake hold before a future event | Sleep with the schedule armed. |
| Awake hold during a temporary carrier or Morse demo | Stop the test, restore the calendar phase, and sleep without canceling today or explicitly advancing the day. |
| Three presses during an active calendar event, including a temporary test | Cancel today and advance to the next configured day if present. |
| Three presses before a future event | Clear temporary activity and re-arm the schedule. |
| Three presses during a manual run with no remaining schedule | Stop the manual run. |

The hold's temporary-test or future-schedule intent is captured at its first
debounced edge. A test expiring, or a scheduled start arriving, while the switch
remains closed must not convert that hold into a day cancellation. Normal calendar
expiry still applies: a test stopped at today's finish may reveal tomorrow's
window, but must not skip it. Restoring an active schedule also retains its
transmit phase; if its slot is already on-air or imminent, the requested sleep may
be brief and the RTC will wake the unit for that slot.

Single-press carrier/demo behavior remains unchanged. In particular, before a
future event the carrier, Morse demonstration, and return-to-schedule sequence
continues to use the existing button and timeout behavior.

`EVT` now also reports the last processed cancellation or scheduled-sleep command:

```text
* Button action: v=1 count=4 at=1789547940 action=1 day_before=0 day_after=0
```

Actions are 1 stop temporary test, 2 cancel current day, 3 cancel manual run,
4 re-arm schedule, and 5 sleep with schedule preserved. `count` saturates at
4294967295 and counts these recorded actions since reset; it is not a raw press
counter. `at` uses device wall-clock time, or zero with an unset clock. The day
indices are zero-based. The line is omitted until an action is recorded. Like the
wake snapshots, this is RAM-only, printed on request into the normal received
serial log, with no added EEPROM writes or periodic telemetry.

The production button detector, long-hold and three-press handlers, manual-start
setup, event activation, and off-air sleep decision are exercised by the host
runtime tests. Coverage includes on-air/off-air cancellation, last-day cancellation,
temporary-test expiry while held, a scheduled start while held, release requirements,
thermal blocking, repeated wakes, and expired/absent/invalid clock settings.
Hardware I/O is stubbed in these checks; physical debounce, standby current, LED
behavior, and RF output still require bench validation.
