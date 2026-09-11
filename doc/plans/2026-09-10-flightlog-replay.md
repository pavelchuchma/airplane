# Flight log replay into the HUD and QGC (`--replay`)

Status: **implemented** 2026-09-10, verified against
`logs/2026-08-29_123339.cvlog` (the first flight / crash log, protocol v4):
the warm-up reproduces the home lock, the field elevation 461.9 m and the
yaw calibration `+61.1 deg, wind 291/2.2, k 1.041` that
`flightlog yawcal` gets offline from the same file.

Closes the one follow-up left open by
`doc/plans/2026-08-25-telemetry-logging.md`: every session with the real link
writes a `.cvlog`, and the only way to look at one is offline
(`flightlog info|csv|events|yawcal`). The first flight ended in a crash after
23 s; watching those 23 s on the HUD — with the wind, the calibrated
altitude and the yaw calibration as cocpitView *would have* shown them
today — is not possible.

## Principle: the log is the source, everything downstream stays live

A replay is a third telemetry source. It must publish through **the same**
code that the live link uses, so an improved calibrator or HUD immediately
improves the replay of an old flight. Nothing about the log is re-processed:
`DOWN` records carry the raw wire payloads, which is exactly what
`EspNowSource` gets from the bridge.

Two consequences:

- The `MAVLink` construction (`_publish_*` in `espnow_source.py`) must move
  out of the serial source into something both sources can call.
- Recorded `EVENT` records (`home_locked`, `baro_calibrated`,
  `yaw_calibrated`) are **not** injected. They are PC-side conclusions and
  the replay recomputes them from the wire — that is the point. They stay in
  the log as a record of what the *flight* concluded, for comparison.

## Decoding: by the log's header, never by today's `protocol.py`

`flightlog.MessageDecoder.telemetry(hdr)` + `upgrade()` already do this for
the offline commands. The replay uses them too, which buys:

- the first flight log is protocol **v4**, whose yaw is mirrored;
  `upgrade()` negates `yaw`/`yawspeed` for v ≤ 4, so it replays with the
  heading the v5 firmware would have sent;
- a future `PROTOCOL_VERSION` bump never orphans an old log;
- frames of a foreign version (bench sessions with stale air firmware) are
  counted and skipped, not misparsed.

## Time

Three clocks have to be kept apart:

| clock | who reads it | in a replay |
| --- | --- | --- |
| wall (`time.monotonic`) | HUD staleness (`STALE_AFTER_S`), `telemetry.py` snapshot | stays wall time — the HUD must grey out when *the replay* stops feeding it |
| log time (`t − hdr["t_mono0"]`) | pacing, `time_boot_ms`/`time_usec` on the wire to QGC | drives everything |
| calibrator time | `yawcal`'s 60 s window, `home`'s 1 s baro freshness | **must be log time** |

`t_mono0` is the timestamp of the first record, i.e. the same zero
`flightlog info` prints its gap table against — so the number the user reads
out of `info` is the number they can pass to `--replay-start`.

`YawCalibrator` already takes `clock=` (it was written for the offline
`yawcal` command). `HomePoint` hardcodes `time.monotonic()`; it gets the same
injectable clock, because a `--replay-speed 0.5` would otherwise make every
baro sample look older than the 1 s gate and home would never lock.

Pacing is an absolute schedule against `time.monotonic()` — a
`sleep(dt)` per frame would add the publish time to every cycle and drift.

### Gaps

Real logs are mostly gaps: `2026-08-29_123339.cvlog` is 1534 s long with
28034 frames, and holds single gaps of 198 s and 247 s. Played faithfully,
that is four minutes of `NO TELEMETRY`. `--replay-max-gap` (default 3 s)
caps the *wait*; the log clock still advances by the true amount, so
`yawcal` sees the real `dt` between the samples either side and keeps
rejecting what it should reject.

## Skipping the start: warm-up vs. cold

`--replay-start T` (`SS`, `MM:SS` or `HH:MM:SS`). The three processors are
stateful — `HomePoint` locks on 25 early epochs, `BaroCalibrator` rides on
those same samples, `YawCalibrator` carries a Kalman state — so "start at
T" has two honest meanings and both are wanted:

- **default (warm-up)**: everything before `T` is published as fast as it
  decodes, with QGC forwarding suppressed, so the processors reach the state
  they had in the flight at `T`. Then playback goes real-time. This is
  "rewind to T", the thing you want when looking at the crash.
- **`--replay-cold`**: the prefix is dropped. Home, AMSL and the calibrated
  heading stay off — which is exactly the "cocpitView started mid-flight"
  case CLAUDE.md warns about, and the only way to exercise it.

Warm-up needs one new parameter on the hub: `publish(msg, forward=True)`.
Suppressing the UDP forward is not cosmetic — a 200 s prefix is ~10 000
frames, i.e. ~40 000 datagrams QGC would have to swallow in a few hundred
milliseconds.

`--replay-end T` falls out of the same parsing for free.

The HUD draws the replay's position in the log (`HH:MM:SS.s`, bottom centre,
`snap.replay_time_s` straight off the `ReplayClock`) — without it there is no
way to tell *where* in the flight a frame is, and it is the number that goes
back into `--replay-start` when something is worth a second look. Live and
emulated sessions leave the field `None` and the HUD draws nothing.

## Not in scope

An offline `cvlog → tlog` converter for QGC's own "Replay Flight Data".
The replay forwards to QGC live, which covers the need; the converter stays
the ~20 lines it always was if it is ever wanted.

## Steps

1. **`cockpitview/publisher.py`** — move `_publish_telemetry`, `_publish_gps`,
   `_publish_battery`, `_publish_radio_status`, the failsafe `STATUSTEXT`
   edge and the `rxerrors` counter out of `EspNowSource` into
   `TelemetryPublisher(hub, warn=True)`. `EspNowSource` keeps the serial
   link, the frame splitter and the loss window and delegates. No behaviour
   change. `warn=False` for the replay: a v4 log would otherwise print the
   "reflash the air firmware" line.
2. **`telemetry.py`** — `publish(msg, forward=True)`.
3. **`home.py`** — `HomePoint(log=None, clock=time.monotonic)`.
4. **`cockpitview/replay_source.py`** — `ReplayClock` and
   `ReplaySource(hub, path, clock, speed, start_s, end_s, cold, max_gap)`.
5. **`__main__.py`** — the flags, the three-way source selection,
   `FlightLog(None)` (a replay must never write a new `.cvlog`), implied
   `--no-camera` unless a camera was named.
6. **`tests/test_replay.py`** — a synthetic log in a tmpdir, a fake sleep:
   order and count; `--replay-start` warm-up reaches the processors but not
   QGC; cold reaches neither; a v4 header yields negated yaw; foreign
   versions counted, not decoded; a long gap is capped in wall time but not
   in log time.
7. `doc/architecture.md` data flows, CLAUDE.md current state, and the
   follow-up in `doc/plans/2026-08-25-telemetry-logging.md`.

Target:

```
./run.sh --replay logs/2026-08-29_123339.cvlog --replay-start 3:15 --replay-speed 2
```
