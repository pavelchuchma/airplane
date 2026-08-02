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
  spinlock and never touches the bus. `clearI2cBus()` clocks a stuck slave
  free before `Wire.begin()` (an MCU reset mid-transaction leaves the slave
  holding SDA low across reboots — every device then reads "not found"),
  and `mpuReadRaw()` checks I2C errors, so a dead bus freezes attitude
  instead of feeding the filter all-0xFF garbage (the HUD used to converge
  to roll −135°/pitch +35°). A dead IMU (0.5 s of failed reads; all-zero
  accel — an MPU that power-cycled back into sleep — counts as failed, and
  a wake-up write is retried periodically) clears `TELEM_FLAG_IMU_OK`:
  cocpitView then stops publishing ATTITUDE, so the HUD hides the horizon,
  roll indicator and yaw compass instead of showing frozen values (heading
  falls back to GPS course while moving). Revival takes 10 consecutive good
  samples — floating-bus garbage occasionally passes the |a| plausibility
  check and a single such sample used to blink the frozen HUD back on.
  The baro has the same lifecycle: pressure outside 300–1100 hPa = bad read
  (a power-cycled BMP280 ACKs but sends reset values), 1 s without a good
  read → alt/climb NAN + `TELEM_FLAG_BARO_OK` drops, re-init via full
  `bmp280::begin()` every 2 s keeping the boot reference. VFR_HUD/ALTITUDE
  are sent unconditionally — a dead baro hides only alt/vario on the HUD,
  never the airspeed.
- The MS4525DO pitot (I2C 0x28, 5 V part on the shared bus) is read at 50 Hz
  (`src/air/ms4525do.cpp`): zero offset averaged at every boot while
  stationary (`calibratePitotZero()`, it drifts with temperature), dP
  low-pass filtered (τ = 0.3 s), IAS = √(2·dP/1.225) with negative dP
  clamped to 0. Telemetry carries `diffPressPa` + `iasMps` (NAN = no/dead
  sensor, 0.5 s without a good read drops `TELEM_FLAG_PITOT_OK` —
  time-based, since failed transactions can take ~1 s each); cocpitView
  puts IAS into `VFR_HUD.airspeed` — NaN passes through so the HUD tape
  shows `---`, never a lying 0. A dead sensor is re-probed every 2 s and
  resumes with the boot zero (`tryPitotRecovery()`; transient I2C dropouts
  from vibration killed airspeed for a whole ride once). A sensor missing
  at boot stays dead — there is no zero to resume with. Pressure counts
  outside the sensor's legal 10–90 % output band are rejected (an all-zero
  bus artifact decodes as status 0 and would poison the filter). Sensor
  temperature is decoded but not sent (future TAS correction).
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
  side; course-over-ground yaw correction above a groundspeed threshold.
