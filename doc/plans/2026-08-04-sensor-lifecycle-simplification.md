# Plan: simplify sensor recovery to the bus-disturbance model + close code-review findings

- **Date:** 2026-08-04
- **Status:** planned, not implemented

## Context

The per-sensor recovery machinery in the air firmware grew out of an abandoned hypothesis
(sensor loses power mid-flight and re-attaches). The real pitot problem was the dP sign bug,
already fixed (ca308c5). Decision: the only failure model worth designing for is
**temporary I2C bus disturbance** — failed transactions and/or corrupted-but-ACKed garbage
data for a while, then the bus recovers. Sensors never lose power, so they never lose config
registers. This makes most of the recovery code (re-running `begin()` every 2 s, periodic MPU
wake-up writes, midpoint sanity re-checks) unnecessary, and its removal simultaneously closes
most open findings in `doc/code-review/*.md` (all 10 of the 815a810 review; findings 3–9 of
the pitot review). Net change ≈ −20 to −40 LOC and three divergent lifecycle mechanisms
collapse into one.

Scope decisions: include the ESP-NOW sender-MAC check (pitot finding 2), drop I2C to
100 kHz (findings 9/10), add in-flight bus rescue (815a810 finding 8).

## Design: one shared sensor lifecycle

A small `SensorLifecycle` struct (3 instances: IMU, baro, pitot) replacing
`baroOk/baroRefValid/lastBaro*` (main.cpp:103–110), `pitotOk/pitotZeroValid/lastPitot*`
(112–119) and the `imuFailures/imuGoodStreak` locals (521–522):

```cpp
struct SensorLifecycle {
  const char *name;      // '#' diagnostics
  uint32_t deadAfterUs;
  bool present = false;  // set once at boot; no power loss in flight → absent stays absent
  bool ok = false;
  uint32_t lastGoodUs = 0;
  int goodStreak = 0;
  uint32_t nextProbeMs = 0;
  bool shouldAttempt(uint32_t nowMs) const {
    return present && (ok || (int32_t)(nowMs - nextProbeMs) >= 0);
  }
  bool onResult(bool good, uint32_t nowUs, uint32_t nowMs);
};
```

Per tick: skip if `!shouldAttempt` (dead sensor probed every 2 s so its ~1 s failed
transactions don't starve healthy sensors). Normal data read + plausibility check is the
"good" verdict (pitot: 10–90 % band already inside `ms4525do::readRaw`; baro: 300–1100 hPa
at caller; IMU: |a| ≥ 0.05 g at caller — these stay as the garbage-data defense).

- good: `lastGoodUs = now`; if ok → process; if !ok → streak++, at 10 → revive (streak
  samples discarded; a successful probe does NOT re-arm the throttle, so the streak builds
  at full rate → revival ~200 ms after the bus recovers).
- bad: streak = 0; if ok and `now − lastGoodUs ≥ deadAfterUs` → dead, outputs NAN, flag
  drops, arm `nextProbeMs` (also on the death transition itself); if !ok → re-arm throttle.
- Death is time-based for ALL sensors (IMU was attempt-based: 25 × ~1 s stalls = up to 25 s
  of frozen-but-OK attitude). New constant `kImuDeadAfterUs = 500000`.
- One code path suffices — no separate "probing" state (validated by trace).

dt discipline: callers capture `prevGoodUs` before `onResult` and use
`dt = min(now − prevGoodUs, deadAfterUs)/1e6f` for gyro integration, vario derivative and
both low-pass alphas — fixes the 25× vario spike and the lost-rotation-in-gap findings.
Revival needs no special dt reset: discarded streak samples refresh `lastGoodUs` each tick.

## Changes by file

### esp32/src/air/ms4525do.cpp / .h

- Delete `begin()` (cpp:47–61; h:15–19) — the write-mode probe and the resting-counts
  midpoint check with it (blocked in-flight recovery above ~50 m/s, rejected offset clones).
  Presence is now proven by calibration reads in main.cpp. Delete `kSanityCountTolerance`
  (cpp:16–17); keep `kZeroCounts` (used in the transfer function, cpp:72); `deviceAddress`
  (cpp:24) becomes `constexpr uint8_t kDeviceAddress = 0x28`. Update file headers.

### esp32/src/air/main.cpp

- **Constants (79–97):** delete `kImuMaxConsecutiveFailures`, `kBaroRecoveryIntervalMs`,
  `kPitotRecoveryIntervalMs`; rename `kImuRecoveryStreak` → shared `kRecoveryStreak = 10`;
  add `kImuDeadAfterUs`, `kProbeIntervalMs = 2000`, `kBusRescueAfterUs = 2000000`,
  `kBusRescueIntervalMs = 5000`, `kI2cClockHz`. Rewrite the misleading `kWireTimeoutMs`
  comment (93–95): the timeout does NOT bound the stall (815a810 #10).
- **Calibrations return bool with min good-sample count** (half the window), no more
  1-sample averages: `calibrateGyro` (289–310, currently silent on count==0),
  `calibrateBaroReference` (314–332), `calibratePitotZero` (361–379, drop the
  `pitotOk=false`/`pitotZeroValid` permanent-disable path).
- **Delete** `tryBaroRecovery` (338–354), `tryPitotRecovery` (387–403), the MPU periodic
  wake-write (582–587), `lastAttitudeUs`. Move the recovered/stopped-responding `#` prints
  into the lifecycle (`name` field); fix the stale `kPitotMaxConsecutiveFailures` reference
  in the comment at 451.
- **Rewrite `updateBaro` (409–445) and `updatePitot` (454–484)** on the lifecycle pattern.
  Baro keeps a function-local `lastBaroAttemptUs` solely for the 25 Hz gate (revival there
  takes ~400 ms — fine). Callers NAN their outputs on the death transition.
- **sensorTask boot (489–523):** per-sensor cheap presence probe (≤3 normal-read attempts —
  implements pitot findings 7+8's fix; `bmp280::begin()` chip-id read serves as the baro
  probe; MPU: wake write + one `mpuReadRaw`) → calibrate (retry once) → on success
  `present = ok = true; lastGoodUs = micros()`; on failure `present = false` +
  `# X absent — <field> nan for this flight` (comment: no-power-loss model). Avoids a
  ~55–200 s boot stall of calibration loops against an absent device.
- **sensorTask loop:** IMU block (534–588) on the lifecycle pattern; complementary filter
  runs only on processed samples with the gap-covering dt (one-line comment: the recovery
  sample's rate approximates the gap rotation). Publish `imuOk = imuLc.ok` — IMU is not-OK
  until gyro calibration succeeds (kills the 0.5 s fake-level-horizon at boot, 815a810 #9).
- **Bus rescue:** if ALL present sensors are simultaneously dead > 2 s:
  `Wire.end() → clearI2cBus() → Wire.begin() → setClock(kI2cClockHz) → setTimeOut`,
  throttled to every 5 s; on success reset all `nextProbeMs = nowMs`. (`TwoWire::end()`
  verified present + mutex-guarded in core 2.0.17; sensorTask is the sole Wire user.
  `Wire.begin()` defaults to 100 kHz → setClock must be re-applied.)
- **SensorSnapshot (124–134):** `climbMps = NAN` (+ global `climbRate` init NAN — contract
  "NAN = no baro" now holds for a baro dead from boot, 815a810 #6); add
  `uint32_t snapshotUs = 0`.
- **loop() staleness check (after 692):** `if (now − s.snapshotUs > 200000)` clear all three
  OK flags in the telemetry (wedged-task honesty, 815a810 #7). Comment: a single ~1 s
  stalled transaction honestly blanks all flags for its duration — accepted.
- **ESP-NOW MAC check:** in `onCommandReceived` (main.cpp:163) reject frames whose sender
  `mac` ≠ `peerMac` (memcmp); same in the ground bridge rx callback
  (`esp32/src/ground/main.cpp:40`). Closes pitot finding 2 (failsafe suppression /
  command injection by any radio on channel 1).
- **I2C clock:** `kI2cClockHz = 100000` (was 400 kHz = the MS4525DO's spec maximum on the
  worst-placed device; ~3 ms bus time per 20 ms tick — plenty). Pitot findings 9/10
  (the VIH half of 10 remains a wiring/bench item — 3.3 V supply option).

## Commits (3)

1. `Sensors: one lifecycle for all three sensors, bus-disturbance failure model` — the
   refactor + dt/NAN/staleness/bus-rescue fixes.
2. `I2C: drop the bus clock to 100 kHz` — separate so field behavior changes stay
   attributable (pitot history of "healthy-looking but not working").
3. `ESP-NOW: accept frames only from the configured peer MAC` — air + ground.

## Documentation updates

- Flip status checkboxes + add fix notes in both `doc/code-review/*.md` files
  (815a810: all 10 closed; pitot review: 3,4,5,6,7,8,9 closed; 2 closed by commit 3;
  10 partially — note the 3.3 V wiring option stays open).
- Update `doc/architecture.md` and the CLAUDE.md "Current state" section: recovery model
  paragraph (unified lifecycle, no re-init, bus rescue, 100 kHz, MAC check).

## Verification

- `cd esp32 && pio run -e air && pio run -e ground` — both must build.
- Bench: boot with all sensors → HUD normal; short SDA/SCL disturbance (touch/disconnect a
  line briefly) → flags drop within 0.5–1 s, HUD hides values, revive ~200–400 ms after the
  bus is clean; boot with pitot unplugged → `# MS4525DO absent`, airspeed `---` whole
  session, other sensors unaffected; verify IAS still reads correctly after the 100 kHz
  change (blow test).
- Ground-side MAC check: normal link must still work end-to-end (run cocpitView).