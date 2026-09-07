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
- `base`, `start`, `finish`, `at` are Unix seconds from the device clock. `base` identifies the saved schedule; `start`/`finish` identify one daily window. Manual key tests use base zero. `at=0` with the valid-time flag clear means unknown time, especially for an unexpected power loss/reset. A reset can establish interruption but cannot establish the physical power-loss time.
- `temp` is the safety temperature in tenths of C (the higher of the fresh raw and filtered reading), or -32768 when unavailable; `limit` is C.
- Flag bits: 1 valid device time; 2 this window has an interruption; 4 scheduled session; 128 earlier history unavailable.
- The unused EEPROM tail holds seven recent 30-byte records on AVR128DA28. Sequence numbers wrap at 32 bits; chronological comparison uses signed differences. Each record has version and CRC checks, with its valid marker committed last. Rotation and RAM queue overflow mark history as incomplete. This is a recent history, not a permanent event archive.
- A boot following an active persisted record adds an interrupted/reset record with unknown time. No record claims continuous RF output: these are scheduler lifecycle observations, not an RF power measurement.

SerialSlinger accepts only version 1 records, deduplicates sequences within the connected device, clears history on identity changes and shows reasons and device times on Android and desktop. Older firmware continues to work; calendar-only results say expired with completion unconfirmed. Completion applies to the last recorded session, not all configured days. Full received records also remain in normal app logs.

Validation: `just test` covers thermal policy, legacy migration, the actual EEPROM journal with simulated power cuts, and production runtime/scheduler helpers under a deterministic clock. Hardware temperature/fan response and actual RF output still require device testing.
