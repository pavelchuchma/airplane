# Plan: telemetry logging in cocpitView (`.cvlog`)

Status: **implemented** (2026-08-26) — `cockpitview/flightlog.py`, hooks in
`espnow_source.py` / `altimeter.py` / `__main__.py`, tests in
`cocpitView/tests/test_flightlog.py`. Bench-verified 2026-08-27 against the
air board on USB: the file appeared with the first frame (`serial_open`
from 30 s earlier retained ahead of it), 5996 frames in 120 s, PC-side dt
median 20.0 ms / p95 22.4 ms / max 41.9 ms (the `in_waiting` read works —
no 50 ms batching), no foreign-version frames, ~825 kB for 130 s.
Deviations from the text below: the loss event is named `serial_lost` (carries the
error text) rather than `serial_reconnect`; `to_dataframe(path, kind=...)`
covers both telemetry and commands.

## Goals

1. **Post-flight calibration** — e.g. pitot position error from GPS ground
   speed vs. IAS on reciprocal legs (`doc/parts/pitot-ms4525do.md`), battery
   offsets from raw `cellMv`, yaw drift vs. GPS course.
2. **Flight replay** into the HUD and QGC (not implemented now, but the format
   must not preclude it).
3. **Flight analysis** — later with joystick input: command → aircraft
   response correlation.

Non-goals: on-air (SD card) logging — the PC is the logging point because it
already sees everything the air side sends. **The emulator is not logged**:
it produces MAVLink straight from `flight_profile.py`, there is no wire frame
to record, and a log of synthetic data has no use.

## What to log: raw wire payloads, not MAVLink

Log **every payload the ground bridge delivers** (in `EspNowSource._read`,
right after `splitter.feed` yields it — before the `msgId` and
protocol-version filters) and **every command payload we send** (in
`_send_commands`). The container is protocol-agnostic like the bridge
itself: it records what crossed the serial link; the reader filters by
`payload[0]`. A future message type is logged for free without a format bump.

Why raw and not the MAVLink stream that goes to QGC:

- **Lossless.** The MAVLink stream is derived and lossy: NaN → 0
  (`gs_mps`, `alt_msl_m`, `hacc`), ATTITUDE is not published while the IMU is
  dead, GLOBAL_POSITION_INT only with a fix, `altimeter.py` rewrites
  `altitude_amsl`/`alt` using the runtime terrain grid, integer scaling
  (cm/s, cdeg, mm). Calibration wants what the air side actually measured:
  `iasMps`, `diffPressPa`, `speedMps`, `courseDeg`, `hAccM` as floats, plus
  the air-side `timeBootMs` (PC ↔ air clock drift comes for free).
- **Commands are free now and needed later** — the channels are already on
  the wire (neutral today), joystick input will land in the same struct.
- **Conversion is one-way trivial**: the whole MAVLink stream is a
  deterministic function of (TELEM records, the `baro_calibrated` event, the
  terrain grid) — raw → CSV / tlog / MAVLink reuses the existing `_publish_*`
  code. tlog → raw is impossible.
- Volume: 50 Hz × (11 + 90 B) + 50 Hz × (11 + 20 B) ≈ **6.6 kB/s ≈ 24 MB/h**,
  uncompressed. Negligible. No rotation: one file per app run, a forgotten
  overnight bench session costs ~600 MB.

Rejected alternatives:

- **CSV directly** — loses commands/events; two streams at different rates
  don't fit one CSV. CSV is the *export* of the converter instead.
- **tlog from the start** — lossy (above).
- **SQLite** — fsync per commit, overkill.
- **Parquet** — pyarrow dependency, not appendable → nothing survives a crash.
- **gzip while writing** — would work (GPS fields repeat between 5 Hz
  updates), but loses `hexdump`-ability; compress after landing if ever.

## File format: `.cvlog`, sequence of length-prefixed records

```
record = struct "<dBH"  (t_mono [s, double], type [u8], len [u16])  ||  payload[len]
```

| type | payload | when |
|---|---|---|
| `0x00 HEADER` | JSON (utf-8), see below | once, first record of the file |
| `0x01 DOWN` | raw downlink payload as delivered by the bridge (today: 90 B `TelemetryMessage`) | every CRC-valid frame from the bridge, whatever its `msgId`/version |
| `0x02 UP` | raw uplink payload (today: 20 B `CommandMessage`) | every command sent |
| `0x03 EVENT` | JSON `{"ev": "...", ...}` | PC-side state not reconstructible from the wire — see below |
| `0x04 MAVLINK` | outgoing MAVLink msgbuf | **reserved, never written** — everything MAVLink is derivable from the records above |

Events (the test is strictly "cannot be recomputed from the wire records" —
`failsafe` and `version_mismatch` were dropped from the list because they are
flag bits in the telemetry payload):

- `baro_calibrated` — `offset_m`, `field_elevation_m`, from `altimeter.py`
  at lock time. The only genuinely PC-side state that changes the output.
- `link_window` — 1 Hz, `telem_loss_pct`, `frames`, `splitter_dropped`,
  `utc`. Not for the percentages (exact loss comes from the 20 ms
  `timeBootMs` cadence in the TELEM records) but as a **liveness
  heartbeat**: in a file with no TELEM records it distinguishes "the app
  crashed" from "the link died". `utc` is there because `time.monotonic()`
  on macOS is `mach_absolute_time()`, which does not advance while the
  laptop sleeps — after a closed lid the header's `started_utc` no longer
  pairs `t_mono` with video/GPS time, a 1 Hz wall-clock sample does.
- `serial_open` / `serial_reconnect` — port name.

**`UP` records are written unconditionally at 50 Hz**, also while the
downlink is silent (plane off: ~6 MB per hour of neutral commands). The
alternative — log only on change plus a keepalive — was rejected: an
asymmetric outage (downlink dead, uplink alive, the plane flying on our
commands) is exactly when the exact timing of every sent command matters,
and a change-only scheme adds sender state that has to be reset on
reconnect. Disk is not the constraint.

Header JSON — the file is **self-describing**, so logs from different
`PROTOCOL_VERSION`s stay readable after future bumps (the reader decodes by
the struct format stored in the header, not by the current `protocol.py`):

```json
{"format": 1, "protocol_version": 4,
 "telemetry_struct": "<BBBBI10f2i6fbBBB3H", "telemetry_fields": ["msg_id", "version", "..."],
 "telemetry_flags": {"GPS_FIX": 1, "FAILSAFE": 2, "VERSION_MISMATCH": 4, "BARO_OK": 8, "PITOT_OK": 16, "IMU_OK": 32, "BATT_OK": 64},
 "command_struct": "<BBBB8h", "command_fields": ["msg_id", "version", "flags", "seq", "ch0", "..."],
 "started_utc": "2026-08-25T14:03:11Z", "t_mono0": 12345.678,
 "git": "8e31e1d", "git_dirty": false, "args": {"terrain": "...", "serial_port": null, "...": "..."}}
```

`git_dirty` matters: half the measurements happen with uncommitted changes,
and the hash alone would misidentify the era. `args` is `vars(args)` — it
includes the terrain grid path, on which the baro offset depends.

Timestamps: records carry `time.monotonic()` (reliable dt); the header
carries wall-clock UTC for pairing with video / GPS time. Air `timeBootMs` is
inside the payload. Records come from two threads (reader, sender), so
`t_mono` is **not strictly monotonic** across record types — the reader must
not assume it; `to_dataframe` sorts by `t`.

**Prerequisite fix in `_read`:** today `ser.read(256)` with a 1 s timeout
blocks until 256 bytes arrive — at 101 B/frame × 50 Hz that is ~2.5 frames
per return, so `now` is quantized to ~50 ms for a whole batch. Fine for
calibration (which uses `timeBootMs`), useless for replay and for
command → response correlation. Change it to `ser.read(max(1, ser.in_waiting))`
as part of this work (also cuts ~25 ms of HUD latency).

Crash resilience: the length prefix lets the reader consume up to the last
complete record and drop a truncated tail. No per-record CRC — the disk is
reliable, only truncation matters. The header is flushed immediately after
open, so even a crash within the first second leaves a readable file.

## Writing: queue + writer thread, bulk flush once per second

```
source thread (50 Hz) ──put(bytes)──▶ queue.SimpleQueue ──▶ writer thread
sender thread (50 Hz) ──put(bytes)──▶                          │ open(), bytearray buffer
hub / altimeter ──────put(event)───▶                          └─ every 1 s or ≥ 64 KiB: f.write(buf); f.flush()
```

- Producers only do `struct.pack` + `queue.put` — **no I/O on the 50 Hz
  path**, never blocks; `SimpleQueue` is thread-safe for multiple producers
  without a lock. `FlightLog` stamps `time.monotonic()` itself at call time.
- **All filesystem I/O lives in the writer thread, including `open()`.**
  The writer loop is `SimpleQueue.get(timeout=until_next_flush)`; it writes
  the buffer on timeout or when it exceeds 64 KiB. `f.flush()` yes, `fsync`
  no — the risk is an app crash, not a laptop power loss; a crash loses ≤ 1 s.
- **Lazy open on the first `DOWN` record.** The sender runs from the moment
  the serial port opens, so with the ground board plugged in and the plane
  off, 50 Hz `UP` records flow with nothing to log against. Rule: the writer
  opens the file when it dequeues the first `DOWN` record; `UP` records
  before that are dropped, and so are the periodic `link_window` events
  (an hour of waiting must not dump 3600 heartbeats behind the header);
  only `serial_open`/`serial_reconnect` are kept in a small list and
  written right after the header. Starting without any board leaves no
  file.
- On open, print `Logging to logs/<file>.cvlog` — the minimum visible
  confirmation that an always-on logger is actually writing. (A HUD `REC`
  indicator with the file size is a possible follow-up.)
- **Logging never takes the app down**: an `OSError` in the writer (disk
  full, unwritable directory) prints once and disables logging.
- `close()` is idempotent and `put` after `close()` is a no-op — the source
  threads are daemons and are still running while `main()`'s `finally`
  executes. `close()` enqueues a sentinel, joins the writer (2 s timeout)
  after a final flush.
- Location `cocpitView/logs/2026-08-25_140311.cvlog` (local time in the
  name, UTC in the header); add `cocpitView/logs/` to `.gitignore`.
- **Always on**; `--no-log` and `--log-dir` as switches — logging that has to
  be remembered never gets switched on at the field. `--no-log` yields a
  `FlightLog` in null mode (same API, every method a no-op) so call sites
  have no `if log:`.

## Code: one module `cockpitview/flightlog.py`

- `FlightLog` — writer (queue + thread); `downlink(payload)`,
  `uplink(payload)`, `event(name, **kw)`, `close()`. The instance is passed
  to `EspNowSource` and `BaroCalibrator` (`log=None` default → null mode).
- `read(path)` — generator of `(t, type, payload)`, handles truncation,
  refuses a `format` newer than it knows.
- `telemetry_frames(path)` — decodes `DOWN` records with
  `payload[0] == MSG_TELEMETRY` by the header struct into namedtuples (via
  `upgrade`); counts and skips frames whose version/length don't match.
  `commands(path)` likewise for `UP`. `to_dataframe(path)` → pandas with a
  `t` column, flags expanded to booleans, sorted by `t` (pandas is an
  optional analysis dependency, **not** in the app's `requirements.txt`;
  commands as a second frame, joined in analysis with `merge_asof`).
- CLI `python -m cockpitview.flightlog info|csv|events <file>` — `csv` is a
  stdlib export for calibration without pandas; `info` reports record
  counts, duration, skipped foreign-version frames and air-side reboots
  (`timeBootMs` jumping backwards — visible in the data, no event needed).

Hook points: `espnow_source.py` (`_read`: every splitter payload + the 1 Hz
window + `serial_open`; `_send_commands`: the command payload; `_run`:
`serial_reconnect`), one line in `altimeter.py` at calibration lock,
`__main__.py` for the switches and `close()`.

## Versioning: three independent layers

1. **Container format** (`format` in the header) — record layout `<dBH` and
   the meaning of record types. Independent of the wire protocol; bumped only
   if the log framing itself changes. The reader refuses a `format` newer
   than it knows.

2. **Wire message layout** (`PROTOCOL_VERSION`) — the header carries
   `telemetry_struct` / `telemetry_fields` (and the command counterparts),
   taken straight from `protocol._TELEMETRY.format` and
   `protocol.Telemetry._fields`; the flag map is generated by introspecting
   `protocol.TELEM_FLAG_*` — no duplication. The reader unpacks **by the
   header, not by the current `protocol.py`**, so a v4 log stays readable at
   v7. Covers added/removed/reordered fields and size changes.

   The header only describes the version the PC *expected*. Frames are logged
   before the version filter, so a file can contain frames of another version
   (mismatch session: older air firmware). Protocol invariant: `msgId,
   version` are always the first two bytes of every message and never move —
   the reader learns each frame's version without knowing its layout. A frame
   is decoded when `payload[1]` matches the header version **and** its length
   matches the struct; anything else is counted and reported by `info`
   ("312 frames v3 skipped"). A table of historical layouts for decoding
   mixed-version files is deliberately **not** built now — mismatch sessions
   are bench sessions, not flights, and no pre-v4 logs exist; add it the
   first time it is actually needed.

3. **Semantics without a layout change** — re-scaled field, a flag bit
   changing meaning, a sign fix (`kAxisSign`), yaw becoming GPS-corrected.
   Same struct, different numbers; the header cannot express that. Mitigation:
   - the header's flag map, `git` + `git_dirty` and `protocol_version`
     together identify the era;
   - **rule: fields in `protocol.py` are never renamed, only added** (wire
     order may change — the struct handles it; names are the API for analysis
     scripts and pandas columns, `df.ias_mps` must work across all logs).
     Record this rule in the `protocol.py` docstring;
   - all version-dependent fixes live in one function
     `upgrade(frame, version) -> Telemetry` (current namedtuple; missing
     fields → NaN). `to_dataframe` and the future `ReplaySource` go through
     it, so `TelemetryPublisher` only ever sees the current shape. Identity
     until the first semantic change.

## Implementation order

1. `protocol.py` — docstring note on field-name stability (no wire change,
   no bump).
2. `cockpitview/flightlog.py` — `FlightLog` (queue, writer thread, lazy open,
   null mode), `read`, `telemetry_frames`, `commands`, `upgrade`,
   `to_dataframe`, CLI.
3. `tests/test_flightlog.py` — round trip, truncated tail, foreign-version
   frame skipped, `--no-log` null mode. First tests in the repo, stdlib
   `unittest` (no pytest in the venv, not worth a dev dependency for this):
   `cd cocpitView && .venv/bin/python -m unittest discover tests`. They
   cover exactly the failure class that would otherwise show up after a
   lost flight.
4. `espnow_source.py` — `in_waiting` read; hooks listed above.
5. `altimeter.py` — `BaroCalibrator(terrain, log=None)`, one line at lock.
6. `__main__.py` — `--no-log`, `--log-dir` (default next to `data/`, like
   `--terrain`), `close()` in `finally`; `.gitignore`.
7. Bench check with the air board on USB (battery unplugged) →
   `python -m cockpitview.flightlog info` on the resulting file; then update
   `doc/architecture.md` (data flows) and `CLAUDE.md` current state.

## Follow-ups (not now, but the design keeps them cheap)

- **Replay** — done, 2026-09-10: `--replay file.cvlog`
  (`cockpitview/replay_source.py`, plan
  `doc/plans/2026-09-10-flightlog-replay.md`). `_publish_*` moved out of
  `EspNowSource` into `publisher.TelemetryPublisher` as foreseen, so the
  replay feeds the HUD **and** QGC through the current processing. An
  offline `cvlog → tlog` converter for QGC's "Replay Flight Data" (8 B
  big-endian µs + msgbuf per message) is still ~20 lines if ever needed.
- **HUD `REC` indicator** — `FlightLog.bytes_written` into the snapshot,
  so a full disk or an unwritable directory is visible in the field.
- **GPS vs. IAS calibration** — `df = flightlog.to_dataframe(...)`, filter
  `gs_mps` / `cog_deg` on two reciprocal legs (procedure in
  `doc/parts/pitot-ms4525do.md`), TAS ≈ mean GS, ratio to IAS → position
  error constant. Raw `diffPressPa` in the log (not only IAS) matters here.

## Decisions on the former open questions

- **No `0x04 MAVLINK` records** — the stream is derivable; `ReplaySource`
  produces a tlog whenever needed.
- **Emulator is not logged** — nothing on the wire to log, and no use for
  synthetic data.
- **`<dBH`** (double seconds) stays — easiest in numpy/pandas.
- `doc/architecture.md` and `CLAUDE.md` are updated at the end of step 7.
