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
- Both devkits have clone CP2102s with the same USB serial number; boards are
  identified by their factory MAC (`custom_board_mac` in platformio.ini,
  probed via ROM bootloader — see `esp32/tools/board_id.py`). Boards may be
  plugged into any USB port; `custom_usb_location` is only a probe-order hint.
  Note: the probe resets the board it touches.
- Ground app: `cocpitView/run.sh`

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
  course-over-ground correction, not a compass. The BMP280 barometer is read
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
  and blinks `FAILSAFE` while the air side reports it.
  Protocol version mismatches are surfaced as one-shot console warnings
  (air → PC checked on every telemetry frame, PC → air reported back via
  `TELEM_FLAG_VERSION_MISMATCH`).
- Altitude display ignores GPS altitude entirely (its vertical noise is tens
  of meters). `cockpitview/altimeter.py` calibrates the baro once on the
  ground: after the first good 3D fix while stationary it locks
  `offset = terrain(lat,lon) − baro_rel` against the CUZK DMR 5G grid
  (`data/terrain.npz`, fetched by `cockpitview/terrain_fetch.py`). The HUD
  altitude box then shows baro AMSL (`---` until calibrated; falls back to
  baro-relative when no terrain grid is loaded) and `R` is
  `baro AMSL − terrain(GPS position)`. QGC gets the calibrated altitude too
  (ALTITUDE.altitude_amsl + GLOBAL_POSITION_INT.alt); raw GPS altitude stays
  in GPS_RAW_INT.
- Next steps: joystick input in cocpitView (the command channels are already
  on the wire, neutral for now); servos + a real failsafe action on the air
  side; course-over-ground yaw correction above a groundspeed threshold;
  mount the pitot on the wing and calibrate its position error in flight
  (check the static holes are not blocked first).
