# Code review — commit 815a810 (I2C fault tolerance)

- **Date:** 2026-08-03
- **Scope:** `git diff HEAD~1 -- esp32/` at commit `815a810` ("I2C fault tolerance: sensor task, per-sensor recovery, honest HUD (v3)") — 4 files: `esp32/src/air/main.cpp`, `esp32/src/air/ms4525do.cpp`, `esp32/src/air/ms4525do.h`, `esp32/lib/protocol/messages.h`.
- **Effort:** high (multi-agent finder + adversarial verify). 20 verified findings folding into 13 distinct defects; the 10 most severe are listed.

**Theme:** the honest-HUD guarantees were applied unevenly. The IMU lifecycle kept the attempt-based death counter the pitot was cured of, revives without a valid gyro calibration or a good-sample-ever guard, and pitot recovery reuses a stationary-only boot sanity check that blocks in-flight recovery above ~50 m/s. Secondary issues: vario dt/NAN contract violations and two plausible whole-bus failure modes.

Status legend: `[ ]` open, `[x]` fixed, `[-]` won't fix.

---

## 1. `[x]` IMU death detection is attempt-based, not time-based — CONFIRMED (correctness)

> **Fixed 2026-08-04:** all three sensors share one `SensorLifecycle` with time-based death (`kImuDeadAfterUs` = 0.5 s since the last good read), part of the recovery-model simplification (`doc/plans/2026-08-04-sensor-lifecycle-simplification.md`).

`esp32/src/air/main.cpp:574` (same root cause at `main.cpp:80`)

IMU death detection is attempt-based (25 consecutive failures) even though a failed I2C transaction on a disturbed bus takes ~1 s each — the exact reason this same commit converted the pitot to time-based death — so the frozen attitude can stay flagged `TELEM_FLAG_IMU_OK` for tens of seconds.

**Failure scenario:** The bus gets disturbed in flight (vibration/loose wire) and each `mpuReadRaw` stalls ~1 s (the file header itself says `Wire.setTimeOut` does not bound that path). Reaching `kImuMaxConsecutiveFailures=25` then takes ~25 s of wall time instead of the intended 0.5 s. For that whole period telemetry keeps `TELEM_FLAG_IMU_OK` set while roll/pitch/yaw are frozen — the pilot flies looking at a live-looking but frozen artificial horizon, the lying-HUD failure this commit was written to eliminate (baro and pitot were both made time-based; the IMU was not).

## 2. `[x]` Pitot recovery blocked in fast flight by resting-counts check — CONFIRMED (correctness)

> **Fixed 2026-08-04:** `ms4525do::begin()` (write probe + resting-counts check) deleted; recovery is now plausible data reads only, valid at any airspeed.

`esp32/src/air/main.cpp:396` + `esp32/src/air/ms4525do.cpp:60`

`tryPitotRecovery` gates on `ms4525do::begin()`, whose final resting-counts sanity check (|counts − midpoint| ≤ 1500 ≈ 1580 Pa) is evaluated against in-flight dynamic pressure, so a pitot dropout during fast flight can never recover until the plane slows below ~50 m/s IAS.

**Failure scenario:** The pitot drops off the bus for a moment during a fast pass or dive with IAS above ~50 m/s (dP > 1580 Pa). Every 2 s recovery attempt calls `begin()`, which reads valid pressure counts but rejects them via the boot-time "resting counts near midpoint" check (`ms4525do.cpp:60`), so `begin()` returns false, `pitotOk` stays false, and airspeed stays `---` on the HUD for the rest of the fast segment even though the sensor is healthy again. (At typical 15–30 m/s it works by luck of the tolerance; the check simply isn't valid outside still air.)

## 3. `[x]` Gyro bias can come from a single sample or stay uncalibrated — CONFIRMED (correctness)

> **Fixed 2026-08-04:** `calibrateGyro()` requires ≥ half the 200-sample window and is retried once; on failure the IMU is marked absent for the session instead of running with a bogus bias.

`esp32/src/air/main.cpp:307` (interacts with the revival path at `main.cpp:548`)

`calibrateGyro`'s removed fixed divisor (bias always averaged over all 200 samples) became "average over however many reads succeeded", so on a marginal bus the gyro bias can be derived from a single noisy sample — and when `count == 0` the bias silently stays 0.0 while the newly added revival path can still bring the IMU alive later with a never-calibrated bias.

**Failure scenario:** At boot the IMU responds to only 1–2 of the 200 calibration reads (intermittent contact/vibration): gyro bias is set from one noisy sample instead of a 200-sample average, or not at all if none succeed. The IMU then works (or revives via the 10-good-sample streak) with `TELEM_FLAG_IMU_OK` set, and the complementary filter integrates a wrong bias every cycle — the HUD yaw compass drifts continuously (yaw has no accel correction) and roll/pitch sit offset, presented to the pilot as trustworthy attitude.

## 4. `[x]` Sub-threshold IMU dropouts lose rotation → permanent yaw error — CONFIRMED (correctness)

> **Fixed 2026-08-04:** gyro integration dt is measured from the last good sample (capped at the death threshold), so the recovery sample's rates approximate the gap rotation instead of dropping it.

`esp32/src/air/main.cpp:572`

During a transient IMU dropout below the death threshold (1–24 failed samples) gyro integration for the gap is silently skipped while `TELEM_FLAG_IMU_OK` stays set — any rotation during the gap becomes a permanent, uncorrectable yaw error.

**Failure scenario:** A 0.3 s I2C hiccup occurs while the plane is turning at 100 deg/s: 15 samples are dropped (`imuFailures < 25` so `imuOk` stays true), and when reads resume the next good sample integrates gyro over dt = 20 ms only, so ~30° of rotation is never integrated. Roll/pitch are slowly pulled back by the accelerometer term, but yaw has no correcting reference (no magnetometer), so the HUD compass is permanently offset by ~30° after every such glitch, with the telemetry flag still claiming the IMU is fine.

## 5. `[x]` Vario dt measured from last attempt but delta from last good read — CONFIRMED (correctness)

> **Fixed 2026-08-04:** vario dt (and the pitot filter alpha) now come from the lifecycle's `lastGoodUs`, capped at the death threshold.

`esp32/src/air/main.cpp:440` (same root cause at `main.cpp:418`)

Vario rate uses dt since the last read *attempt* (40 ms) but the altitude delta since the last *good* read (up to ~1 s ago), inflating the climb rate by up to ~25× after a transient baro dropout shorter than the 1 s death threshold.

**Failure scenario:** Baro reads fail (or fall outside the 300–1100 hPa plausibility band) for ~0.9 s while the plane climbs at 5 m/s, then a good read lands: `rate = (alt − altitudeM)/dt` uses dt = 0.04 s but an altitude delta accumulated over ~0.94 s, giving rate ≈ 120 m/s; with alpha = 0.074 the filtered climbRate jumps by ~+9 m/s. The HUD vario and QGC `VFR_HUD.climb` show a wild false spike after every sub-threshold baro glitch. dt should be measured from `lastBaroGoodUs`, not `lastBaroUs`.

## 6. `[x]` Baro absent at boot publishes climb 0.0 instead of NAN — CONFIRMED (correctness)

> **Fixed 2026-08-04:** `climbRate` and `SensorSnapshot.climbMps` initialize to NAN; the "NAN = no baro" contract now holds for a baro dead from boot.

`esp32/src/air/main.cpp:128` (same root cause at `main.cpp:107`)

`SensorSnapshot` defaults `climbMps` to 0.0 (and the `climbRate` global also starts at 0.0), so a baro that is absent from boot publishes climb = 0 instead of NAN, contradicting the `messages.h` contract "climbMps ... NAN = no baro" added in this same commit.

**Failure scenario:** The BMP280 is missing/unresponsive at boot (`bmp280::begin()` fails): `baroAltM` is NAN as documented, but `climbMps` goes out as 0.0 m/s in every telemetry frame with `TELEM_FLAG_BARO_OK` clear. Any consumer honoring the documented field-level NAN convention (rather than the flag) displays a fake steady 0 m/s vario instead of no-data; the NAN promise is only kept for a baro that died after boot, never for one dead from the start.

## 7. `[x]` Wedged sensorTask leaves stale all-OK telemetry at 50 Hz — PLAUSIBLE (correctness)

> **Fixed 2026-08-04:** the snapshot carries `snapshotUs`; loop() clears all three OK flags when it is older than 200 ms (one stalled transaction honestly blanks them for its duration).

`esp32/src/air/main.cpp:689`

The old code's implicit freshness invariant — every telemetry frame carried sensor data read in the same 20 ms loop iteration, and any I2C stall was at least externally visible as a collapsed link rate — is not re-established: `loop()` copies `SensorSnapshot` with no timestamp/age check, so a sensorTask wedged indefinitely inside one transaction leaves telemetry reporting stale data with all OK flags at a healthy 50 Hz.

**Failure scenario:** A slave wedges the bus mid-transaction in a way the Arduino driver never times out of (the exact unbounded-wait path the file header describes); sensorTask blocks forever inside `Wire`, never reaching the failure-counting or time-based death code. `loop()` keeps sending the last published snapshot: HUD shows a live-looking horizon, altitude and airspeed with all OK flags set, permanently frozen at pre-failure values — with no visible symptom at all (link stats stay perfect).

## 8. `[x]` clearI2cBus runs only at boot, never during flight — PLAUSIBLE (correctness)

> **Fixed 2026-08-04:** `tryBusRescue()` — when every present sensor has been silent > 2 s, sensorTask runs `Wire.end()` → `clearI2cBus()` → `Wire.begin()` (throttled to every 5 s) and re-probes immediately.

`esp32/src/air/main.cpp:490`

`clearI2cBus()` runs only once at sensorTask start, so a slave that wedges SDA mid-flight (e.g. a transaction aborted mid-byte by the new 20 ms Wire timeout, or a sensor brownout during a read) can never be cleared, making the new per-sensor recovery machinery permanently ineffective for the very fault class the function exists to fix.

**Failure scenario:** During flight a vibration-induced glitch or timeout-aborted read leaves one sensor holding SDA low; every subsequent I2C transaction to all three sensors fails, all OK flags drop, and the recovery probes fail every 2 s until reboot — the pilot flies the rest of the flight with no attitude, no altitude/vario and no airspeed even though the documented design promises in-flight recovery from transient I2C dropouts.

## 9. `[x]` Dead-at-boot IMU publishes fake level attitude for ~0.5 s — CONFIRMED (correctness)

> **Fixed 2026-08-04:** `imuOk` publishes the lifecycle's `ok`, which is false until the boot probe + gyro calibration succeed — no fabricated attitude window.

`esp32/src/air/main.cpp:603`

With the IMU absent at boot, the first 25 sensor-task cycles publish `imuOk = true` (`imuFailures` counts up from 0) together with the never-initialized attitude roll/pitch/yaw = 0, so `TELEM_FLAG_IMU_OK` is set for ~0.5 s while the attitude data is pure fabrication — the dead-at-boot case has no equivalent of the pitot's "missing at boot stays dead" guard.

**Failure scenario:** Powering up with a disconnected/failed MPU-6500: after the ~2 s calibration (which already printed "MPU-6500 not responding"), the HUD briefly draws a fake perfectly-level horizon, roll indicator and compass for half a second before `TELEM_FLAG_IMU_OK` drops and they hide again — and cocpitView publishes those fake ATTITUDE messages to QGC.

## 10. `[x]` kWireTimeoutMs comment contradicts measured stall behavior — CONFIRMED (cleanup)

> **Fixed 2026-08-04:** comment rewritten — the timeout shortens some failure paths but does not bound the ~1 s stall, which is why sensorTask exists.

`esp32/src/air/main.cpp:96`

The comment above `kWireTimeoutMs` claims the 20 ms timeout makes transactions "fail fast" instead of stalling ~1 s, but the file header (lines 32–37) and the commit message state the measured opposite: the driver stalls ~1006 ms regardless of `Wire.setTimeOut` — that is the entire justification for the sensor task. A future maintainer reading only the constant's comment could conclude the timeout suffices and remove or bypass the sensor-task isolation, reintroducing the 50 Hz link collapse on a disturbed bus.
