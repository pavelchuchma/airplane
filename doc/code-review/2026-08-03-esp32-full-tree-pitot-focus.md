# Code review — full esp32/ tree, pitot (MS4525DO) focus

- **Date:** 2026-08-03
- **Scope:** the entire `esp32/` tree (18 files), primary focus on the MS4525DO pitot subsystem — the pitot did not work in practice (IAS stuck at 0, occasional 5–10 m/s blips, no reaction to blowing) while the MPU-6500 and BMP280 on the same I2C bus worked fine.
- **Effort:** high (multi-agent finder + adversarial verify). 26 verified findings; the 10 most severe are listed.

**Conclusion:** the most likely software cause of the observed symptom is the inverted dP sign convention (finding 1) whose negative result is silently clamped to IAS 0, backed by a cluster of pitot lifecycle bugs (findings 3, 4, 6, 7, 8) and two plausible hardware hypotheses worth a bench test (findings 9, 10). Two safety-relevant ESP-NOW link findings (finding 2) are unrelated to the pitot but should be fixed before servos go live.

Status legend: `[ ]` open, `[x]` fixed, `[-]` won't fix.

---

## 1. `[x]` Pitot dP sign inverted vs ArduPilot/PX4 — negative dP clamped to IAS 0 — CONFIRMED (correctness)

> **Fixed 2026-08-03:** `ms4525do::read()` now negates the transfer function (`kZeroCounts − counts`), matching the ArduPilot/PX4 convention.

`esp32/src/air/ms4525do.cpp:68`

The dP sign convention is inverted relative to the ArduPilot/PX4 MS4525DO drivers for the same Pixhawk-kit hardware: both reference drivers compute `diff_press = -((counts − 0.1·16383)·(Pmax−Pmin)/(0.8·16383) + Pmin)`, i.e. they negate the raw type-A transfer function because on these breakouts airflow into the total-pressure port drives counts BELOW the 8192 midpoint; this driver uses `(counts − 8192)` un-negated, and `main.cpp` then clamps negative filtered dP to zero (`iasMps = sqrtf(2·max(diffPressPa,0)/rho)`), so the wrong sign is permanently invisible.

**Failure scenario:** With the kit plumbed exactly per `doc/parts/pitot-ms4525do.md` (dynamic to Port 1, static to Port 2), airflow makes counts drop below 8192, `dp = pressure − pitotZeroPa` goes negative, the `max(dp,0)` clamp in `updatePitot` (`main.cpp:483`) forces IAS to 0: the HUD airspeed tape reads 0 at any speed and `TELEM_FLAG_PITOT_OK` stays set, so the sensor looks "healthy but not working" — matching the reported symptom while MPU/BMP280 on the same bus work fine.

## 2. `[ ]` ESP-NOW rx callbacks never verify the sender MAC — CONFIRMED (correctness, safety)

`esp32/src/air/main.cpp:163` (same root cause at `esp32/src/ground/main.cpp:40`)

`onCommandReceived` never checks the sender MAC against `peerMac`: ESP-NOW delivers UNencrypted frames from any unregistered sender to the rx callback (`peer.encrypt=true` only protects frames exchanged with the registered peer), so any radio on channel 1 in LR mode can inject `CommandMessage`s without knowing the PMK/LMK.

**Failure scenario:** A stray or malicious ESP32 broadcasting 20-byte frames with `msgId=MSG_COMMAND` and `version=PROTOCOL_VERSION` refreshes `lastCmdMs` on the air side, suppressing the safety-critical 500 ms failsafe while the real ground link is down — and once servos exist, its channel values become live control inputs to the airplane.

## 3. `[x]` begin()'s resting-counts check rejects healthy sensors (in flight + offset clones) — CONFIRMED (correctness)

> **Fixed 2026-08-04:** `ms4525do::begin()` deleted entirely (recovery-model simplification, `doc/plans/2026-08-04-sensor-lifecycle-simplification.md`); presence is proven by the boot zero-calibration reads, recovery by plausible data reads at any airspeed.

`esp32/src/air/ms4525do.cpp:60` (used from `esp32/src/air/main.cpp:396`)

`begin()`'s resting-counts sanity check (|counts − 8192| ≤ 1500, ~1580 Pa) assumes zero airflow, but `begin()` is also the in-flight recovery probe called from `tryPitotRecovery()`; real dynamic pressure above ~1580 Pa (IAS ≥ ~51 m/s, or less with gusts/prop blast on the pitot) makes the probe reject a perfectly healthy sensor, and the same check also rejects clone sensors with a static zero offset > 1500 counts at every boot even though `calibratePitotZero()` would absorb the offset.

**Failure scenario:** (a) The pitot drops off the bus for a moment during fast flight (vibration — the exact scenario `tryPitotRecovery` exists for); every 2 s re-probe fails the midpoint check because the airplane is flying, so airspeed stays `---` until the plane slows below ~50 m/s. (b) A clone unit with a large fixed offset (parts doc gotcha 5: "clone quality varies... sanity-check ~8192 counts at rest") reads e.g. 6500 counts at rest, `begin()` returns false at every boot, and the sensor is permanently reported "# MS4525DO not found" although 4-byte reads work and boot zeroing would have corrected the offset.

## 4. `[x]` Failed zero calibration permanently disables pitot recovery — CONFIRMED (correctness)

> **Fixed 2026-08-04:** calibration is retried once; if it fails twice the sensor is explicitly `present = false` for the session (stated `#` diagnostic) — no more half-dead state where `pitotOk` is false but recovery is silently impossible.

`esp32/src/air/main.cpp:377`

If `calibratePitotZero()` gets zero good reads in its 1.1 s window (transient bus glitch right after a successful `begin()`), it sets `pitotOk = false` while `pitotZeroValid` stays false, and `tryPitotRecovery()` early-returns on `!pitotZeroValid` forever — a sensor that was present and verified at boot is permanently disabled with no recovery path, contradicting the stated design goal that transient I2C dropouts must not kill airspeed for the rest of the flight.

**Failure scenario:** Sensor passes `ms4525do::begin()` at power-on, then a ~1.1 s I2C disturbance (vibration from starting the motor, loose connector) spans the calibration window; all 55 reads fail, `count == 0`, `pitotOk = false`, `pitotZeroValid = false`. The sensor recovers electrically two seconds later but `tryPitotRecovery()` never probes it, so the whole flight has no airspeed, with no way back short of rebooting the airplane.

## 5. `[x]` Vario dt from last attempt but delta from last good read — CONFIRMED (cleanup)

> **Fixed 2026-08-04:** dt measured from the last good read (see finding 5 of the 815a810 review).

`esp32/src/air/main.cpp:417`

`updateBaro()` computes the climb-rate derivative as `(alt − altitudeM)/dt` where dt is time since the last read *attempt* (40 ms) but `altitudeM` may be from the last *good* read up to ~1 s earlier, so the first sample after a transient dropout produces a spiked rate up to ~25× too large. (Duplicate of finding 5 in the 815a810 review.)

**Failure scenario:** Baro reads fail for 0.5–0.9 s (below the 1 s dead threshold) while the plane climbs 2 m; the next good sample computes rate = 2 m / 0.04 s = 50 m/s and feeds it into the vario low-pass — the HUD vario jumps wildly after every transient I2C hiccup. dt should be measured from `lastBaroGoodUs`, not `lastBaroUs`.

## 6. `[x]` Pitot zero calibration accepts a single-sample average — CONFIRMED (correctness)

> **Fixed 2026-08-04:** all three boot calibrations require at least half their sample window and are retried once.

`esp32/src/air/main.cpp:373`

`calibratePitotZero()` accepts the boot zero from as little as one good sample (`count > 0` out of 50): a glitchy bus or wind gust during the 1 s window locks in a zero offset that is pure single-sample noise instead of failing or retrying the calibration.

**Failure scenario:** If 49 of 50 boot reads fail (marginal wiring) the pitot zero is one noisy sample, easily tens of Pa off; per the project's own doc gotcha 1 that offset error reads as "too fast", so the HUD shows several m/s of phantom airspeed all flight (and a future auto-throttle would fly dangerously slow), with no indication calibration was degraded.

## 7. `[x]` begin() decides presence from one read, no retry — CONFIRMED (cleanup)

> **Fixed 2026-08-04:** boot presence is up to 3 normal-read attempts (`probePitot()`), and the zero calibration behind it tolerates a half-failed window.

`esp32/src/air/ms4525do.cpp:57`

`begin()` decides sensor presence from one single `readRaw()` with no retry, and `main.cpp`'s "missing at boot stays dead" policy makes that single read a permanent point of failure for the session.

**Failure scenario:** One glitched byte at boot (status bits ≠ 0, or a short read while the bus settles right after `clearI2cBus`/MPU wake) fails `begin()`; `pitotOk` and `pitotZeroValid` stay false and `tryPitotRecovery()` never runs, so a healthy sensor produces no airspeed for the whole flight. Retrying the probe read 2–3 times (the data is refreshed every 0.5 ms) removes the single-sample fragility.

## 8. `[x]` Write-mode presence probe on a read-only I2C device — PLAUSIBLE (correctness)

> **Fixed 2026-08-04:** the write-mode probe went away with `ms4525do::begin()`; presence is probed with the same 4-byte `read()` the driver uses everywhere.

`esp32/src/air/ms4525do.cpp:51`

`begin()` probes presence with a zero-length write transaction (`beginTransmission`/`endTransmission`, i.e. address byte with the W bit), but the MS4525DO is a read-only I2C device with no defined write frame — units/clones that NACK write addressing are declared missing even though the actual 4-byte reads (all the driver ever needs) would succeed.

**Failure scenario:** A sensor that ACKs reads at 0x28 but NACKs the address+W probe makes `Wire.endTransmission()` return non-zero, `begin()` returns false at boot and at every `tryPitotRecovery()` attempt, and airspeed is permanently `---` with "# MS4525DO not found" despite the device answering an I2C read scan; probing directly with `Wire.requestFrom` (as `readRaw` already does) would detect it.

## 9. `[ ]` 400 kHz bus runs at the MS4525DO's spec maximum — PLAUSIBLE (correctness, hardware)

`esp32/src/air/main.cpp:492`

`Wire.setClock(400000)` runs the shared bus at exactly the MS4525DO's specified maximum SCL frequency (parts doc: "SCL 100–400 kHz max"), leaving zero timing margin for the one device that sits on a wire harness (the pitot breakout is cabled away from the GY-91 board) with only the GY-91's ~10 k pull-ups defining rise times.

**Failure scenario:** Bus rise time degraded by the pitot cable's capacitance makes 400 kHz marginal specifically for the MS4525DO (the MPU-6500 and BMP280 sit directly on the well-pulled-up GY-91 board and tolerate it): the sensor NACKs or returns corrupted transfers, `begin()` fails or reads fail continuously, and airspeed reads `---` in practice while the other two sensors work — dropping to 100 kHz (still 2× the needed 50 Hz × 4-byte throughput) is the standard fix.

## 10. `[ ]` 5 V-powered pitot may not see 3.3 V bus highs as high — PLAUSIBLE (correctness, hardware)

`esp32/src/air/main.cpp:46` (wiring comment)

A 5 V-supplied MS4525DO on a 3.3 V I2C bus violates its VDD-referenced input-high threshold (VIH ~0.7–0.8·VDD = 3.5–4.0 V at 5 V), and the shared bus additionally runs at the sensor's absolute-maximum 400 kHz with only the GY-91's onboard 3.3 V pull-ups plus pitot cable capacitance — the wiring comment claims "open-drain I2C — safe" but only covers the ESP32 side, not the sensor's input thresholds.

**Failure scenario:** The MS4525DO never sees SCL/SDA highs above its input-high threshold (or misses edges at 400 kHz on the marginal bus), so it never ACKs address 0x28: `begin()` fails at boot, `pitotZeroValid` stays false so recovery is permanently disabled, and the HUD airspeed shows `---` for every flight — while the 3.3 V-supplied MPU-6500 and BMP280 on the same bus work fine. Fix directions: run the bus at 100 kHz, and/or power the sensor from 3.3 V (chip operating range is 2.7–5.5 V per `doc/parts/pitot-ms4525do.md`) so its thresholds drop to 3.3 V-compatible levels.
