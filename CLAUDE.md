# CLAUDE.md

RC airplane project: custom ESP32-based control link (replacing the RC
transmitter) and a ground station app. Read `doc/architecture.md` first —
it describes the topology, data flows and the reasoning behind design
decisions. Keep it updated when the design evolves.

## Layout

- `esp32/` — PlatformIO project with two environments: `air` (in the plane:
  sensors, servos, autopilot) and `ground` (transparent ESP-NOW ↔ USB
  bridge). Shared wire protocol in `esp32/lib/protocol/`.
- `cocpitView/` — Python ground station app; joystick input, translates the
  custom protocol to MAVLink for QGroundControl. Note: the directory is
  spelled `cocpitView` (sic), the Python package inside is `cockpitview` —
  do not "fix" either.
- `arduinoNano/` — legacy MPU-6050 firmware (Arduino Nano), the predecessor
  of the esp32 air firmware.
- `doc/` — `architecture.md` plus hardware parts notes in `parts/`.

## Commands

- Firmware build/upload: `cd esp32 && pio run -e air -t upload` (or `-e ground`)
- Flashing the air board is done over USB with the board mounted in the
  airplane, so **disconnect the flight battery first** — USB power and the
  battery/BEC must never be live at the same time. Booting like that prints
  `# battery absent - data nan for this flight` (the balance lead is off with
  the pack); that is the expected USB-powered state, not a regression.
- Both devkits have clone CP2102s with the same USB serial number; boards are
  identified by their factory MAC (`custom_board_mac` in platformio.ini,
  probed via ROM bootloader — see `esp32/tools/board_id.py`). Boards may be
  plugged into any USB port; `custom_usb_location` is only a probe-order hint.
  Note: the probe resets the board it touches.
- Ground app: `cocpitView/run.sh`
- Ground app tests: `cd cocpitView && .venv/bin/python -m unittest discover tests`
- Flight log inspection: `cd cocpitView && .venv/bin/python -m cockpitview.flightlog info|csv|events logs/<file>.cvlog`

## Rules

- Wire-protocol changes must touch both sides atomically:
  `esp32/lib/protocol/messages.h` and its Python mirror in cocpitView,
  and bump `PROTOCOL_VERSION`.
- The ESP-NOW link is safety-critical (it replaces the RC transmitter):
  fixed-rate commands, failsafe on the air side, link stats in telemetry.
- Commits, docs and code comments in English.

## Current state (update as work progresses)

- The ESP-NOW link is live: `esp32/src/air` sends an 84 B `TelemetryMessage`
  at 50 Hz and receives 50 Hz `CommandMessage`s (structs + COBS/CRC-16
  framing + shared radio bring-up in `esp32/lib/protocol/`; channel 1, LR
  mode, PMK/LMK encryption, peer MACs injected as `-DPEER_MAC_STR` by
  `tools/peer_mac.py`). No valid command for 500 ms → failsafe flag in
  telemetry + `# FAILSAFE` diagnostic (no servos yet, the flag is the whole
  action). Air USB serial (115200) carries only `#` diagnostics now — the
  CSV output is gone. `esp32/src/ground` is the protocol-agnostic bridge:
  ESP-NOW payloads ↔ COBS+CRC frames on USB serial at 460800 baud, binary
  only, never prints. Air-side RSSI comes from a promiscuous-mode callback
  (IDF 4.4 has no per-packet RSSI in the ESP-NOW rx callback; core 3.x
  would change the callback signatures).
- The sensor board is a GY-91, but its IMU is an
  MPU-6500 (verified via WHO_AM_I = 0x70 and no AK8963 at 0x0C) — a fake
  "MPU-9250", so there is **no magnetometer**. Yaw drifts; the plan is GPS
  course-over-ground correction, not a compass. In the airframe the board is
  mounted **upside down — rotated 180° about body X** (sensor X still
  forward), so `mpuReadRaw()` negates the Y and Z accel/gyro counts into the
  X-forward/Y-right/Z-up body frame (`kAxisSign`, applied at the driver so
  the gyro bias calibration sees body frame too). `kAxisSign` must stay a
  proper rotation (det +1): a mirror (Z alone was tried) puts gyro and accel
  in opposite frames on one axis and the complementary filter degrades to
  its slow accel path — seconds of rubbery HUD lag, seen on the airframe
  2026-08-25 (pitch jerked the wrong way first, then crawled to the right
  value — the inverted-gyro-Y signature). Level trim measured on the
  airframe 2026-08-25 (`kRollTrimRad`/`kPitchTrimRad`, roll −0.52°, pitch
  +0.09°, subtracted from the accel-derived angles): airplane held in ideal
  level attitude reads roll/pitch 0.0.
  The BMP280 barometer is read
  (`src/air/bmp280.cpp`): altitude relative to boot location +
  low-pass-filtered vario. I2C robustness: **all I2C access lives in a
  dedicated FreeRTOS task** (`sensorTask`, core 0) — the Arduino/IDF I2C
  driver stalls ~1 s per transaction on a disturbed bus regardless of
  `Wire.setTimeOut` (measured), which used to collapse the 50 Hz link loop
  to ~0.5 Hz; `loop()` (core 1) only copies a `SensorSnapshot` under a
  spinlock and never touches the bus, and distrusts a snapshot older than
  200 ms (OK flags read false — a wedged task can't publish stale data as
  healthy). All three sensors share one **`SensorLifecycle`** designed for
  a single failure model: temporary bus disturbance (failed transactions
  or corrupted-but-ACKed data — I2C has no integrity check), sensors never
  lose power, so recovery never re-initializes anything. Time-based death
  (0.5 s IMU/pitot, 1 s baro) → fields NAN + the sensor's TELEM flag drops
  (cocpitView hides the HUD elements; heading falls back to GPS course
  while moving); revival = 10 consecutive plausible reads (single garbage
  samples passing the checks used to blink the frozen HUD back on); a dead
  sensor is probed at most every 2 s so its ~1 s failed transactions don't
  starve the healthy ones. Plausibility checks: IMU |a| ≥ 0.05 g, baro
  300–1100 hPa, pitot 10–90 % count band. dt for gyro integration and the
  vario/pitot filters spans back to the last *good* sample (capped at the
  death threshold) — sub-threshold dropouts don't spike the vario or drop
  rotation. Boot: presence probed by ≤3 normal data reads, calibrations
  (gyro bias / baro reference / pitot zero) need ≥ half their window and
  retry once, else the sensor is `present = false` for the whole session
  (nothing can make it appear without a power loss). Bus-level rescue:
  `clearI2cBus()` clocks a stuck slave free before `Wire.begin()` at boot
  (an MCU reset mid-transaction leaves SDA held low across reboots), and
  reruns in flight (`Wire.end()` → clear → `begin()`, every 5 s) when all
  present sensors have been silent > 2 s. VFR_HUD/ALTITUDE are sent
  unconditionally — a dead baro hides only alt/vario on the HUD, never the
  airspeed.
- The MS4525DO pitot (I2C 0x28, 5 V part on the shared bus) is read at 50 Hz
  (`src/air/ms4525do.cpp` — read-only driver, no `begin()`: the sensor has
  no registers): zero offset averaged at every boot while stationary
  (`calibratePitotZero()`, it drifts with temperature), dP low-pass
  filtered (τ = 0.3 s), IAS = √(2·dP/1.225) with negative dP clamped to 0.
  Telemetry carries `diffPressPa` + `iasMps` (NAN = no/dead sensor);
  cocpitView puts IAS into `VFR_HUD.airspeed` — NaN passes through so the
  HUD tape shows `---`, never a lying 0. A revived sensor resumes with the
  boot zero (in-flight re-zeroing needs still air). Sensor temperature is
  decoded but not sent (future TAS correction). Two things were established
  by measurement and are easy to get wrong again: the **hoses are crossed
  with respect to the port labels** on this kit (total pressure goes to the
  port the labels suggest is the static one; a reversed pair reads a
  rock-solid 0, not a negative value, because of the clamp — fix it in the
  tubing, never by negating in the driver), and the **±1 psi range is
  verified** against a 981 Pa water column, so `kPressureRangePsi` is not to
  be touched. What is *not* solved is **position error**: on a bicycle the
  IAS read ~30 % below GPS, entirely from where the static holes sat. That
  correction belongs to the installation, must be measured on the airframe
  (two reciprocal legs against GPS), and goes in its own constant.
  `doc/parts/pitot-ms4525do.md` has the mounting analysis (the sweet spot is
  c/4 ahead of the wing leading edge, where upwash cancels the C_L
  dependence — further forward is worse) and the calibration procedure.
- GPS (Quescan G10A-F30, u-blox M10) is on UART2 (GPIO16/17, 38400 baud),
  read via TinyGPSPlus (`src/air/gps.cpp`). The chip is flash-less, so the
  firmware re-applies config at every boot via raw UBX-CFG-VALSET frames:
  5 Hz nav rate, airborne ≤4g dynamic model, RMC+GGA+GST sentences only (GST
  carries the receiver's 1-sigma error estimates → hacc/vacc). GPS telemetry
  floats are NAN without a fix (sats/hdop stay live); lat/lon travel as
  int32 deg·1e7 guarded by the `TELEM_FLAG_GPS_FIX` flag.
- cocpitView is the telemetry hub: `cockpitview/espnow_source.py` reads the
  ground bridge's framed serial stream (port resolved by `[env:ground]` MAC;
  `cockpitview/protocol.py` is the Python mirror of `messages.h` + framing),
  sends 50 Hz neutral commands back, and forwards MAVLink to QGC (UDP
  14550): ATTITUDE + VFR_HUD + ALTITUDE + GPS_RAW_INT (always) +
  GLOBAL_POSITION_INT (only with a fix) + RADIO_STATUS + STATUSTEXT on
  failsafe transitions. A built-in emulator remains via `--emulator`. The
  HUD shows `GPS 3D 12` and `LNK -72 U2% D0%` status lines (air-side RSSI,
  uplink/downlink loss), a `G 42` GPS ground-speed line under the airspeed
  box (from GLOBAL_POSITION_INT velocities, so it disappears without a fix)
  and blinks `FAILSAFE` while the air side reports it. `TelemetryHub` runs a
  chain of message processors (`HomePoint`, then `BaroCalibrator`) on every
  published message before it reaches the snapshot and QGC.
  Protocol version mismatches are surfaced as one-shot console warnings
  (air → PC checked on every telemetry frame, PC → air reported back via
  `TELEM_FLAG_VERSION_MISMATCH`).
- The launch point ("home") and the altimeter calibration are one event, so
  one gate in `cockpitview/home.py` owns both: 25 GPS epochs (~5 s at 5 Hz)
  that are 3D, report `hacc ≤ 5 m`, are below 1 m/s and have a baro sample
  younger than 1 s; the median locks `(lat, lon, baro_rel)`. `BaroCalibrator`
  registers as a sink on those very same samples and only adds a terrain
  lookup per sample, so it can stay uncalibrated (outside the grid) while
  home locks — never the reverse, and home needs no terrain grid at all.
  Consequence to keep in mind: starting cocpitView mid-flight leaves both
  off for the whole session, and the emulator cannot exercise either (it
  flies from t = 0 and sends `h_acc = 0`) — testing is unit tests
  (`tests/test_home.py`) plus the airframe. The HUD `DST 1.5km (+358m)` line
  (bottom-left above `GPS`) is the slant range home + the baro altitude
  difference; the vertical leg is a *baro* difference against `baro_rel` at
  the lock (never GPS altitude, and `snap.baro_relative_m` is used instead of
  `altitude_m` because the latter falls back to a GPS `relative_alt` that
  `espnow_source` zero-fills when the baro is dead). GPS lost → `DST ---
  (+358m)`, baro dead → the distance degrades to horizontal and shows
  `(---)`.
- Altitude display ignores GPS altitude entirely (its vertical noise is tens
  of meters). `cockpitview/altimeter.py` locks
  `offset = terrain(lat,lon) − baro_rel` (median of the per-sample
  differences) against the CUZK DMR 5G grid
  (`data/terrain.npz`, fetched by `cockpitview/terrain_fetch.py`). The HUD
  altitude box then shows baro AMSL (`---` until calibrated; falls back to
  baro-relative when no terrain grid is loaded) and `R` is
  `baro AMSL − terrain(GPS position)`. QGC gets the calibrated altitude too
  (ALTITUDE.altitude_amsl + GLOBAL_POSITION_INT.alt); raw GPS altitude stays
  in GPS_RAW_INT.
- Per-cell battery monitoring is implemented (protocol v4, 90 B telemetry):
  `src/air/battery.cpp` reads the 3S balance nodes via ADC1 dividers
  (C1 10k/10k → GPIO35, C2 28k/10k → GPIO34, C3 56k/10k → GPIO39, 100 nF per
  pin, all under the 11 dB linear ceiling) at 10 Hz in `sensorTask`,
  64-sample average per node, cells by subtraction, plausibility
  2.5–4.35 V/cell. Battery has its own `SensorLifecycle` (not in the
  bus-rescue list — it isn't I2C): "present" means the balance lead was
  plugged at boot; telemetry carries `cellMv[3]` guarded by
  `TELEM_FLAG_BATT_OK`. cocpitView sends MAVLink `BATTERY_STATUS` (per-cell
  mV, 2 Hz, skipped while the flag is down → QGC + HUD hide it) and the HUD
  shows `BAT 11.4 3.80/3.79/3.81 Δ20` bottom-right (LNK moved one line up);
  a cell < 3.2 V or Δ > 150 mV blinks the line at 2 Hz. The emulator feeds a
  fake draining pack. The divider harness is built and verified end-to-end;
  `battery.cpp` carries a one-point scale calibration (2026-08-23, full pack
  vs multimeter — channels read 2-2.7 % high on nominal ratios). NOT done:
  the second calibration point on a near-empty pack (offsets; plan:
  `doc/plans/2026-08-10-battery-cell-monitoring.md`).
- Flight logging is live (`cockpitview/flightlog.py`, plan
  `doc/plans/2026-08-25-telemetry-logging.md`): every cocpitView session
  with the real link writes `cocpitView/logs/<timestamp>.cvlog` (gitignored;
  `--no-log`/`--log-dir`; the emulator is never logged — it has no wire
  frames). Records are `<dBH` (monotonic s, type, len) + payload: `DOWN` =
  every raw payload from the bridge *before* any msgId/version filter, `UP`
  = every command sent (50 Hz, unconditionally — also while the plane is
  off, ~6 MB/h), `EVENT` = JSON for `home_locked` / `baro_calibrated`, the 1 Hz
  `link_window` heartbeat (proves the reader was alive during a telemetry
  gap; carries UTC because macOS' monotonic clock stops in sleep) and
  `serial_open`/`serial_lost`. The header JSON stores the struct formats,
  field names and flag map taken from `protocol.py`, plus git hash/dirty and
  CLI args — readers decode by the header, so a bump never orphans old logs;
  frames of a foreign version are counted and skipped, not decoded. Rules
  that follow: **`Telemetry`/`Command` field names are never renamed, only
  added** (they are the pandas column API across all logs) and **`msgId`,
  `version` stay the first two bytes of every message**. Version-dependent
  semantic fixes belong in `flightlog.upgrade()` (identity today). The
  writer thread owns all file I/O: lazy open on the first `DOWN` (no plane →
  no file; pre-open `UP`/`link_window` are dropped, `serial_*` kept), 1 s
  bulk flush, header flushed immediately, `OSError` disables logging without
  touching the app. `_read` now does `ser.read(max(1, ser.in_waiting))` —
  the old `read(256)` quantized timestamps (and HUD latency) to ~50 ms
  batches — verified on the bench 2026-08-27: 5996 frames, dt median
  20.0 ms, p95 22.4 ms. Tests in `cocpitView/tests/` (stdlib unittest). NOT
  done: the replay source.
- Next steps: second battery calibration point; joystick input in
  cocpitView (the command channels are already on the wire, neutral for
  now); servos + a real failsafe action on the air side; course-over-ground
  yaw correction above a groundspeed threshold; mount the pitot on the wing
  and calibrate its position error in flight (check the static holes are not
  blocked first).
