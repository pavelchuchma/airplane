# Air-side diagnostic log over ESP-NOW (`.air.log`)

Status: **implemented** 2026-10-20, verified on the bench (both boards on
USB, `logs/2026-10-20_175342.cvlog`): the air boot lines buffered before
cocpitView connected arrived in order (`#0`..`#3`, no gaps), `flightlog
diag` output is identical to the live `.air.log`, and over 41 s the link
showed 0 % tx fail and 0 % command loss.

## Why

The air board's `#` diagnostics (`slow I2C: <sensor> took N ms`, sensor
death/recovery, gyro bias, IMU config lost, FAILSAFE transitions...) go to
the USB serial only. USB and the flight battery are never connected at the
same time, so every diagnostic about a battery-only problem is lost by
construction. The case that forced this: the ~1 s sensor-task stalls of
`doc/todo/002`, which occur only on battery and which the `slow I2C` line
was written to name — it has never run during a stall.

So the diagnostics travel over the radio as well, land in the `.cvlog`
(raw, like every other downlink payload) and in a plain-text `.air.log`
next to it that a human — or Claude — can read or `tail -f` directly.

## Wire: `LogMessage` (protocol v7)

```c
constexpr uint8_t MSG_LOG = 0x03;
constexpr size_t LOG_TEXT_MAX = 120;

struct __attribute__((packed)) LogMessageHeader {
  uint8_t msgId;        // MSG_LOG
  uint8_t version;      // PROTOCOL_VERSION
  uint16_t seq;         // per boot, from 0, assigned when the line is produced
  uint32_t timeBootMs;  // when the line was produced (not when it was sent)
  uint16_t dropped;     // lines lost to air-side buffer overflow since boot
};                      // followed by 1..LOG_TEXT_MAX bytes of text, no NUL
```

- Variable length: header (10 B) + text. One line per message, no `#`
  prefix and no newline on the wire. Python mirror in `protocol.py`
  (`decode_log()` returns `None` for anything malformed).
- `seq` gaps show lost lines; `dropped` tells buffer overflow on the air
  side apart from radio loss.
- `msgId`/`version` stay the first two bytes (the flightlog invariant).
- The ground bridge is protocol-agnostic and passes it as-is — no change.

## Air side

- `diag(fmt, ...)` replaces every `Serial.printf("# ...")` in
  `src/air/main.cpp`: prints `# <text>` on USB exactly as today and pushes
  the line into a ring buffer (32 × 120 B). Callable from `sensorTask`
  (core 0) and `loop()` (core 1); the buffer is guarded by its own spinlock
  (formatting happens outside it). Full buffer → the oldest line is dropped
  and `dropped++`.
- **Only `loop()` sends**, after the telemetry frame, and with a hard
  budget so diagnostics can never crowd the safety-critical link: at most
  one log frame per 100 ms (≤ 10 lines/s, a few % of airtime in LR mode).
- **Lines are sent only while commands arrive** (no failsafe): commands come
  from a running cocpitView, so without them nobody listens and the line
  would be lost. Boot lines (gyro bias, absent sensors) therefore wait in
  the buffer until cocpitView connects; their `timeBootMs` keeps them
  correctly placed in time.
- `txFailPct` keeps counting all sends (telemetry + log) — it is a link
  quality figure; the comment in `messages.h` says so.

## Battery-off switch for the todo 002 experiment

Build flag `-DAIR_NO_BATTERY`
(`PLATFORMIO_BUILD_FLAGS=-DAIR_NO_BATTERY pio run -e air -t upload`):
the battery ADC is never read, battery reports absent, and a `diag` line
says why. Same firmware otherwise, so a battery session with and without it
answers hypothesis 2 of todo 002 from the stall counts alone. Also
`updateBattery()` gets the same `reportSlow()` timing as the I2C reads.

## cocpitView

- `espnow_source.py`: `MSG_LOG` frames (current version) are printed to
  the console as `[air] <text>`. Logging needs nothing new — the payload
  is already a `DOWN` record.
- `.cvlog` header gains `log_struct` + `log_fields` (decode-by-header, as
  for telemetry); logs without them simply contain no log lines.
- `.air.log`: written by the FlightLog **writer thread** (which owns all
  file I/O), same stem as the `.cvlog`, opened with the first log line,
  flushed per line (low volume, `tail -f`-able). The lines are produced by
  one formatter (`flightlog.AirLogFormatter`) fed with the same records the
  writer writes, so `python -m cockpitview.flightlog diag FILE.cvlog`
  regenerates the identical text for any log — the `.cvlog` stays the
  source of truth, the `.air.log` is a derived view.
- Line format:

  ```
  2026-10-20T15:31:12.345Z  00:01:12.3  boot   98.765  #412  slow I2C: MS4525DO took 1004 ms
  ```

  UTC (anchored to the latest `link_window` event's UTC, so a laptop sleep
  does not skew it; before the first heartbeat to the header's
  `started_utc`) · position in the log as HH:MM:SS.s (the HUD replay clock,
  accepted by `--replay-start`) · air `time_boot_ms` · `seq` · text.
  Special lines: `--- air boot ---` when `seq` restarts,
  `[lost N lines (A buffer overflow on air, R radio)]` on a `seq` gap;
  duplicates (same `seq`) are skipped.
- `flightlog info` counts the log lines.

## Rules touched

- `PROTOCOL_VERSION` 6 → 7 on both sides (`messages.h`, `protocol.py`);
  air + cocpitView must be updated together, the ground bridge not.
- CLAUDE.md "Current state": the air USB serial is no longer the only home
  of the `#` diagnostics.

## Tests

- `protocol`: `LogMessage` round trip, malformed/oversized payloads → None.
- `flightlog`: formatter — boot marker, seq gap with/without overflow,
  duplicate, UTC anchoring; writer produces `.air.log` next to `.cvlog`
  identical to `diag` output; no log line → no `.air.log`.
- Bench: flash both boards, run cocpitView, check the boot lines arrive in
  `.air.log` with plausible `boot` times and no gaps.
