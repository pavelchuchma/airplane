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

- Firmware build/upload: `cd esp32 && pio run -e air -t upload` (or `-e ground`).
  The platform is **pioarduino** (arduino-esp32 3.x on ESP-IDF 5.5, pinned
  in `platformio.ini`), which needs PlatformIO Core ≥ 6.2; both envs build
  with `-DCORE_DEBUG_LEVEL=0` (an `[E]` log line blocks UART0 ~9 ms, and on
  the ground board it would land in the binary frame stream).
- I2C driver failure timing on a bare devkit: `pio run -e i2ctest -t upload`
  (flashes the ground board; reflash `-e ground` afterwards).
- A **jumper on the logic buck's output** (added 2026-11-05) separates the
  pack-side 5.3 V from the devkit's 5V pin. Jumper **open** → USB and the pack
  may be connected together: USB powers the ESP32 and the sensor rail, the
  pack feeds ESC/VTX and the balance lead, so battery monitoring reads real
  cells on a USB boot. Jumper **closed** → the buck powers the logic, which is
  the flight configuration, and USB must then stay out (the buck's 5.3 V
  would back-feed USB VBUS — two 5 V sources in parallel). Plugging USB is
  the user's act, so it is not a question to ask before flashing.
  Consequences: a USB boot prints `# battery absent - data nan for this
  flight` only when the pack is out — with the pack in and the jumper open,
  real cell data is expected; and a battery-only boot with the jumper open
  leaves the ESP32 unpowered (no telemetry at all), so close it before flight.
- Which sensors report `absent` at boot depends on where the board is. Out of
  the airframe it is a bare devkit and *everything* reads absent (I2C
  `i2cWriteReadNonStop` error −1 on every address, GPS silent on UART2) — that
  is the bench state, not damage. Only judge sensor health with the board
  wired into the airplane.
- Both devkits have clone CP2102s with the same USB serial number; boards are
  identified by their factory MAC (`custom_board_mac` in platformio.ini,
  probed via ROM bootloader — see `esp32/tools/board_id.py`). Boards may be
  plugged into any USB port; `custom_usb_location` is only a probe-order hint.
  Note: the probe resets the board it touches.
- `tools/select_port.py` **aborts the upload** when no connected board has
  the env's MAC or its port is busy (PlatformIO would otherwise auto-detect
  the port and flash whatever single board is plugged in). After an upload
  check the `select_port:` line, not just the `SUCCESS` tail, and stop
  cocpitView before flashing — it holds the ground port.
- Ground app: `cocpitView/run.sh`
- Ground app tests: `cd cocpitView && .venv/bin/python -m unittest discover tests`
- Flight log inspection: `cd cocpitView && .venv/bin/python -m cockpitview.flightlog info|csv|events|diag|yawcal logs/<file>.cvlog`
- Air-side diagnostics of a session: `cocpitView/logs/<timestamp>.air.log`
  (plain text, readable directly or with `tail -f` while cocpitView runs).

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
  `tools/peer_mac.py`; the keys and the flying site come from the
  git-ignored `esp32/local.ini` — copy `local.ini.example`, `openssl rand
  -hex 16` twice, flash both boards; `tools/link_keys.py` aborts the build
  without it, `cockpitview/localconf.py` reads the `[site]` default for
  `terrain_fetch`). No valid command for 500 ms → failsafe flag in
  telemetry + `# FAILSAFE` diagnostic (no servos yet, the flag is the whole
  action). Air USB serial (115200) carries only `#` diagnostics, and since
  protocol v7 the same lines also go over the radio (`src/air/diag.cpp`,
  `diag::print()` replaces every `Serial.printf("# ...")`; plan
  `doc/plans/2026-10-20-air-diag-log.md`): variable-length `LogMessage`
  (`MSG_LOG`, seq + `time_boot_ms` + overflow counter), buffered 32 lines
  deep, sent by `loop()` at most every 100 ms and only while commands
  arrive (boot lines wait until cocpitView connects). cocpitView prints
  them as `[air] ...` and writes `logs/<stem>.air.log` next to the `.cvlog`
  (UTC, log position HH:MM:SS.s, air boot time, seq; boot markers and
  lost-line reports); `flightlog diag` regenerates it from the `.cvlog`,
  which stays the source. So a battery-only session no longer loses its
  diagnostics. `esp32/src/ground` is the protocol-agnostic bridge:
  ESP-NOW payloads ↔ COBS+CRC frames on USB serial at 460800 baud, binary
  only, never prints. Air-side RSSI is `rx_ctrl->rssi` of the received
  command (`esp_now_recv_info_t`, IDF 5; the promiscuous-mode sniffer that
  IDF 4.4 needed is gone).
- The sensor board is a GY-91, but its IMU is an
  MPU-6500 (verified via WHO_AM_I = 0x70 and no AK8963 at 0x0C) — a fake
  "MPU-9250", so there is **no magnetometer**. Yaw is gyro integration only:
  arbitrary boot datum plus drift, calibrated PC-side against the GPS (see
  `cockpitview/yawcal.py` below), never against a compass. In the airframe the board is
  mounted **upside down — rotated 180° about body X** (sensor X still
  forward), so `mpuReadRaw()` negates the Y and Z accel/gyro counts into the
  X-forward/Y-right/Z-up body frame (`kAxisSign`, applied at the driver so
  the gyro bias calibration sees body frame too). `kAxisSign` must stay a
  proper rotation (det +1): a mirror (Z alone was tried) puts gyro and accel
  in opposite frames on one axis and the complementary filter degrades to
  its slow accel path — seconds of rubbery HUD lag, seen on the airframe
  2026-08-25 (pitch jerked the wrong way first, then crawled to the right
  value — the inverted-gyro-Y signature). Level trim lives in
  `kRollTrimRad`/`kPitchTrimRad` (`src/air/main.cpp`), subtracted from the
  accel-derived angles; a re-measurement is *added* to the constant, and
  the pitch zero is taken in level flight, not on the bench.
  **IMU registers are configured at boot** (`kMpuConfig`): gyro DLPF
  20 Hz + accel DLPF 10 Hz (anti-aliasing for the 50 Hz polling), ±16 g
  accel (no ADC clipping under motor vibration), ±500 °/s gyro, PLL clock;
  `kAccelScale`/`kGyroScale` follow the ranges. The complementary filter
  skips the accel correction when |a| is outside 1 g ± 0.3 g. The config
  is read back every second and rewritten on mismatch (`# IMU config
  lost`). Before this (power-on defaults, 2026-10-11 ground motor run) the
  attitude wandered 10–30° and shook 12–24°/s with the airframe held
  still: aliased vibration plus a clipped accel mean, not gyro clipping
  (rates peaked at 75 °/s). Verified the same day
  (`2026-10-11_180819.cvlog`, motor run hand-held level and nose-up):
  gyro-rate rms 0.03–0.06 rad/s, attitude swing 0.4–2° per second,
  no wander. That log also has the baro dead for the whole session:
  the boot (18:07:08, spanning `180628` seg3, `180819` and `181117` seg0,
  7 min) had the baro present, calibrated and sane for 7 s, then it
  NACKed instantly and for good while the MPU on the same board stayed
  healthy; only the next battery power cycle revived it. Best fit: the
  floating CSB pin on the GY-91 glitched low and latched the BMP280 into
  SPI mode (irreversible until power-on reset) — tie CSB/NCS to 3V3
  (`doc/todo/001-gy91-csb-ncs-tie-high.md`). Diagnostics added for the
  next occurrence (in the `.air.log` since v7): a chip-id probe at baro death
  (`no ACK` vs `still answers` vs a bus failure) plus the per-sensor
  `I2C <sensor> 1s: ...` bus reports.
  The ~1 s sensor-task stalls (battery boots only, 2026-10-11 … 10-20; each
  froze the whole snapshot, so `loop()` dropped all OK flags — the HUD
  "stutter") are **resolved**: the trigger was a cold ground joint in the
  airframe wiring that made the MS4525DO glitch on the bus whenever the
  airframe was closed on battery (resoldered 2026-10-20, 381 s clean
  since), and the second came from the Arduino/IDF I2C driver, which
  turned each glitched plain read into a ~1 s wait (arduino-esp32 2.x
  only — fixed by the move to 3.x, below), where a glitch now costs one
  sample. A bit-banged bus served in between (`git show e65f783`); its
  per-transaction failure classes are what found the joint. How it was narrowed down
  (VTX, ADC, timing, open vs closed airframe), and why the 1 s is *not*
  the IDF alive interval: `doc/todo/002-sensor-task-1s-stalls-on-battery.md`.
  **Gyro bias calibration waits for stillness** (protocol v6,
  `calibrateGyroWhenStill()`): a 1 s window with every axis within 3 °/s,
  |a| within 1 g ± 0.1 g, no read gap > 100 ms, |bias| ≤ 10 °/s; rejected
  windows restart, no time limit (5 s without any read → IMU absent).
  Meanwhile `TELEM_FLAG_IMU_CALIBRATING` is set, the rate fields are raw
  and the HUD shows `GYRO CAL - hold still (7°/s)` where the horizon will
  be. Reason: the 2026-10-12 logs had biases off by 10 and 30–37 °/s from
  turning the plane over during the old fixed 0.8 s window, which the
  complementary filter turns into a permanent 10–40° attitude offset plus
  yaw drift at the bias rate. Measured on five 5–14 min stationary boots:
  the residual bias moves < 0.05 °/s, so no in-flight recalibration is
  needed (straight flight is indistinguishable from stillness to a
  gyro+accel, so it would be unsafe without a GPS-speed gate anyway).
  The BMP280 barometer is read
  (`src/air/bmp280.cpp`): altitude relative to boot location +
  low-pass-filtered vario. I2C robustness: **all I2C access lives in a
  dedicated FreeRTOS task** (`sensorTask`, core 0) — the Arduino/IDF I2C
  driver stalled ~1 s per transaction on a disturbed bus regardless of
  `Wire.setTimeOut` (measured; where in the driver stack is unknown), which
  used to collapse the 50 Hz link loop to ~0.5 Hz. The bus runs on the
  **ESP-IDF 5 master driver** (`src/air/i2cbus.cpp` over
  `driver/i2c_master.h`, not Wire; plan
  `doc/plans/2026-10-20-arduino3-native-i2c.md`): 100 kHz, internal
  pull-ups, a **5 ms timeout** per transaction (a held SCL costs exactly
  that; a NACK or held SDA < 1 ms, measured with `i2ctest`). Both lines are
  checked around every transaction: held before START → bus clear, still
  held → `bus-busy` without trying; held after → `i2c_master_bus_reset`
  at once (`N bus freed`). Statuses `ok`/`bus-busy`/`failed` (NACK or
  other — IDF does not tell them apart)/`timeout`. Per sensor these are
  summed and reported once a second as an `I2C <sensor> 1s: ...` diag
  line *only* when something happened (an error, a freed bus or a read ≥
  4 ms), so a clean bus leaves the `.air.log` silent; `loop()` (core 1)
  only copies a `SensorSnapshot` under a
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
  sensor is probed at most every 2 s (a leftover from the ~1 s failed
  transactions of the IDF driver, harmless now). Plausibility checks: IMU |a| ≥ 0.05 g, baro
  300–1100 hPa, pitot 10–90 % count band. dt for gyro integration and the
  vario/pitot filters spans back to the last *good* sample (capped at the
  death threshold) — sub-threshold dropouts don't spike the vario or drop
  rotation. Boot: presence probed by ≤3 normal data reads, calibrations
  (gyro bias / baro reference / pitot zero) need ≥ half their window and
  retry once, else the sensor is `present = false` for the whole session
  (nothing can make it appear without a power loss). Bus-level rescue:
  `clearI2cBus()` (→ `i2cbus::clear()`) clocks a stuck slave free at boot
  (an MCU reset mid-transaction leaves SDA held low across reboots — seen
  right after a flash, `I2C bus stuck (SDA low)`), and reruns in flight
  (every 5 s) when all
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
  IAS read ~30 % below GPS, entirely from where the static holes sat — but
  that number does **not** carry over to the airframe. First flight data
  (2026-08-29, 12 s of climb-out, 58 GPS epochs, IAS against wind-corrected
  GPS TAS): IAS/TAS = 0.921 ± 0.039, i.e. ~10 % low after the density
  correction, and yawcal's independent `k` from the same flight says 4 % low.
  So the installed error is single-digit-to-10 %, not 30 %, and the constant
  will be small. Provisional — 12 s and one wind estimate is not a
  calibration. That correction belongs to the installation, must be measured
  on the airframe (two reciprocal legs against GPS), and goes in its own
  constant. Watch the **zero** as well: it read +9.9 Pa (sd 5.2) standing
  still, part wind on the tube but not all — harmless at cruise (2-3 % of
  IAS), 13 % of dP near stall speed, where it matters most.
  `doc/parts/pitot-ms4525do.md` has the mounting analysis (the sweet spot is
  c/4 ahead of the wing leading edge, where upwash cancels the C_L
  dependence — further forward is worse) and the calibration procedure.
- GPS (Quescan G10A-F30, u-blox M10) is on UART2 (GPIO16/17, 38400 baud),
  read via TinyGPSPlus (`src/air/gps.cpp`). The chip is flash-less, so the
  firmware re-applies config at every boot via raw UBX-CFG-VALSET frames:
  5 Hz nav rate, airborne ≤4g dynamic model, RMC+GGA+GST sentences only (GST
  carries the receiver's 1-sigma error estimates → hacc/vacc; GSV once a
  second for the satellites-in-view count). GPS telemetry
  floats are NAN without a fix (sats/hdop stay live); lat/lon travel as
  int32 deg·1e7 guarded by the `TELEM_FLAG_GPS_FIX` flag. Since protocol
  v8 `sats` tells three states apart: satellites *used* with a fix,
  satellites *in view* (GSV sum over the GP/GL/GA/GB talkers) while
  acquiring, and `TELEM_SATS_NO_RECEIVER` (255, MAVLink's own
  `satellites_visible` sentinel) when no NMEA sentence passed its checksum
  for 2 s — `gps::alive()` is time-based, not "ever heard". The HUD shows
  `GPS -- 7` while acquiring and a blinking `GPS ---` for a silent
  receiver (v7 showed `GPS -- 0` for both; `.air.log` gets `GPS receiver
  silent for 2 s` / `talking again` on the transitions). Bench state out
  of the airframe is the silent one.
- cocpitView is the telemetry hub: `cockpitview/espnow_source.py` reads the
  ground bridge's framed serial stream (port resolved by `[env:ground]` MAC;
  `cockpitview/protocol.py` is the Python mirror of `messages.h` + framing),
  sends 50 Hz neutral commands back, and forwards MAVLink to QGC (UDP
  14550): ATTITUDE + VFR_HUD + ALTITUDE + GPS_RAW_INT (always) +
  GLOBAL_POSITION_INT (only with a fix) + RADIO_STATUS + STATUSTEXT on
  failsafe transitions. A built-in emulator remains via `--emulator`. The
  HUD shows `GPS 3D 12` and `LNK -72 U2% D0%` status lines (air-side RSSI,
  uplink/downlink loss), a `G 12` GPS ground-speed line under the airspeed
  box (from GLOBAL_POSITION_INT velocities, so it disappears without a fix)
  and blinks `FAILSAFE` while the air side reports it. `TelemetryHub` runs a
  chain of message processors (`HomePoint`, then `BaroCalibrator`) on every
  published message before it reaches the snapshot and QGC.
  Protocol version mismatches are surfaced as one-shot console warnings
  (air → PC checked on every telemetry frame, PC → air reported back via
  `TELEM_FLAG_VERSION_MISMATCH`). Every integer MAVLink field in
  `publisher.py` goes through `_u16`/`_u32`/`_i16`/`_i32` (NaN/inf → the
  field's "unknown" sentinel, else clamped): pymavlink raises
  `struct.error` on overflow, and on 2026-11-04 one `GPS_RAW_INT` with
  h_acc = 5303 km (the receiver acquiring a fix) killed the serial reader
  thread, so the HUD showed `NO TELEMETRY` until a restart. The reader
  now also drops a frame whose publish fails (`publish_error` event, first
  three tracebacks printed) and reconnects on any other exception
  (`reader_error` event) instead of dying.
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
- The IMU body frame is X-forward, **Y-left**, Z-up (FLU), not "Y-right" as
  `main.cpp` claimed until 2026-08-29: the sensor frame is right-handed and
  `kAxisSign` is a proper rotation, so X-forward + Z-up forces Y-left. Roll
  and pitch carry the usual aerospace signs there; yaw does not — a positive
  rotation about Z is a *left* turn. Up to protocol v4 that mirrored heading
  went straight onto the wire and through the HUD, QGC and yawcal alike. **v5
  flips `yawRad`/`yawRate` once, where telemetry is filled** — not in
  `kAxisSign` (that must stay det +1) and not in the filter (which needs a
  consistent FLU triad). `flightlog.upgrade()` negates yaw/yawspeed for v ≤ 4
  so old logs stay comparable. Caught only by regressing reported yaw against
  GPS course on the first flight log (slope −1.056, rms 5.9°); the emulator
  could never catch it, because it generates yaw as a compass heading and so
  was self-consistent.
- Per-cell battery monitoring is implemented (protocol v5, 90 B telemetry):
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
  vs multimeter — channels read 2-2.7 % high on nominal ratios); a repeat on
  2026-08-29 confirmed it to 0.08-0.16 % per node but could not yield the
  offsets — both points sit at a full pack, ~1 % apart, so the two-point fit
  is singular (node 3 came out slope 6.02, offset +996 mV). After the move
  to IDF 5 `adc_oneshot` + `adc_cali` (2026-10-20) the old constants still
  hold within 0.2 % on a warm board — but a board cold from outdoors read
  N2/N3 0.7 % low against the same DMM, so the ADC drifts ~0.5 % with
  temperature: calibrate on a warm board and repeat the measurement. The
  pack is a
  **3S1P Samsung INR18650-30Q** (unprotected cells, 15 A/cell continuous,
  `doc/parts/battery-samsung-30q.md`); land at 3.5 V/cell resting, never
  below 3.0. Full throttle costs **~0.3-0.36 V/cell** of indicated voltage
  (measured on the airframe from the balance taps, which carry no current),
  so the HUD's 3.2 V alarm fires with ~a quarter of the pack left — state of
  charge must be read at low throttle, and the alarm should be gated on
  throttle once throttle control exists. NOT done: the second calibration
  point, deliberately deferred until after the first real flight, to be taken
  on the discharged pack (needs a ≥20 % voltage span, the DMM in its
  highest-resolution range, and raw `cellMv` averaged ±15 s from the log,
  not the HUD; plan: `doc/plans/2026-08-10-battery-cell-monitoring.md`).
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
  20.0 ms, p95 22.4 ms. Tests in `cocpitView/tests/` (stdlib unittest).
- Flight replay is live (`--replay logs/<file>.cvlog`,
  `cockpitview/replay_source.py`, plan
  `doc/plans/2026-09-10-flightlog-replay.md`): the logged downlink payloads
  are paced by their own timestamps into `publisher.TelemetryPublisher` —
  the `_publish_*` methods were extracted out of `espnow_source.py`, so the
  live link and the replay share one MAVLink path and an old flight is
  re-flown on today's HUD, calibrators and QGC. Decoding goes through the
  log's header + `flightlog.upgrade()` (a v4 log replays with its mirrored
  yaw corrected). `--replay-start MM:SS` (the times `flightlog info` prints)
  jumps into the flight and fast-forwards the run-up through the processors
  with the UDP forward off, so home/baro/yaw hold at the jump-in point;
  `--replay-cold` drops it instead (the only way to exercise a mid-flight
  start), `--replay-speed`, `--replay-end`, `--replay-max-gap` (default 3 s
  — the logs contain 200 s silences; only the *wait* is capped, log time
  jumps the whole gap so yawcal still sees the true dt). The calibrators run
  on the log's clock, not the viewer's (`HomePoint`/`YawCalibrator` take
  `clock=`) — a replay never writes a `.cvlog` of its own. The HUD shows the
  position in the log as `00:02:59.9` bottom centre (only in replay), which
  is the number to feed back to `--replay-start`.
- Heading is calibrated against the GPS, not a compass
  (`cockpitview/yawcal.py`, plan `doc/plans/2026-08-27-yaw-gps-calibration.md`).
  `COG − yaw` is deliberately **not** used — it is off by the wind drift
  angle (18° at 5 m/s wind / 15 m/s flight), so it would calibrate the wind.
  Instead a 60 s window of samples is least-squares fitted to the whole wind
  triangle `v_ground = k·IAS·u(ψ+Δ) + wind`, which is *linear* in
  `(k·cosΔ, k·sinΔ, wind_n, wind_e)`; it yields the yaw offset, the wind (HUD
  `WND 314/4 ±3`) and `k`, the IAS→TAS scale — i.e. the pitot position error
  measured in flight. Observability is the whole game: straight flight is
  singular, so a window is **rejected** (never solved with a wide error bar)
  when the condition number of the equilibrated design matrix exceeds 10
  (360° turn = 1.0, 0–90° sweep = 3.5, ±20° = 9.9, straight = 1e15). A
  two-state (offset, drift rate) Kalman filter carries the estimate between
  windows; each fit applies at its window *centroid* (else a 60 s window lags
  30 s) and only windows disjoint from the last fed one reach the filter
  (consecutive fits share 59/60 of their samples). Sits last in the
  `TelemetryHub` processor chain and rewrites ATTITUDE/VFR_HUD heading, so
  HUD/FPM/QGC all get true heading and **the wire protocol is untouched**;
  the air side keeps sending raw yaw, so logs stay re-analysable. Before a
  lock the HUD boxes the heading as `~037` and shows `WND ---` — the boot
  datum is fine for relative turns, it just must not read as a compass. Two
  sampling traps handled: the air side *holds* its 5 Hz GPS fields across
  50 Hz frames (bit-identical repeats are dropped, else one epoch pairs with
  ten attitudes) and the receiver's velocity solution lags, so attitude is
  interpolated back by `GPS_VELOCITY_LAG_S`. The emulator now flies in wind
  and reports a deliberately wrong yaw (a windless emulator makes
  `COG == heading`, the one case the naive method gets right) and sends a
  realistic `h_acc` — MAVLink's default 0 means "unknown" and kept both
  `home.py` and this switched off in emulated sessions. First airframe data
  (`2026-08-29_123339.cvlog`, 23 s of flight — one solvable window, cond 5.9):
  with the v5 yaw sign it fits wind 291°/2.2 m/s and `k = 1.04` at rms 1.74 —
  the direction cross-checks to 2° against the wind derived independently
  from the drift of the crash spin's ground track (293°/3.2 m/s). The same
  window *before* the sign fix returned wind 155°/17.2 m/s and `k = 0.71` at
  rms 2.70, i.e. **it passed every gate** (cond ≤ 10, rms < 3.0, k in bounds)
  while absorbing the mirrored heading into a fictitious 17 m/s wind. Lesson:
  the gates test geometry and fit quality, neither of which notices a sign
  error — the honest cross-check is `k`, which has a physical band (1.0–1.4)
  the bogus fit fell outside of while still inside `K_BOUNDS`. Still
  provisional: `GPS_VELOCITY_LAG_S` and the drift PSD need a longer flight
  (one window measures neither), and `k` wants reciprocal legs.
- VTX control over IRC Tramp is live (protocol v9, v10 adds the actual
  power; 2026-11-06; `doc/todo/003-vtx-tramp-control.md`, plan
  `doc/plans/2026-10-25-vtx-tramp-control.md`). `src/air/vtx.cpp` speaks
  Tramp on UART1 (RX GPIO26 via 10 kΩ, TX GPIO27 via 1 kΩ, single wire,
  own echo dropped): `r` probe every 1 s until the VTX answers (unpowered
  on a USB-only bench — `absent` is normal there), then `v` every 500 ms,
  `s` every 2 s, one request in flight, ≥ 200 ms apart. It reconciles the
  desired state from `CommandMessage.vtxFreqMhz`/`vtxPowerMw` (0 / 0xFFFF
  = leave, power 0 = pit) against the readback with `F`/`P`/`I`, ≤ 1 set
  per 500 ms, 10 answered-but-disagreeing tries → `VTX_STATUS_REFUSED`;
  race lock and failsafe = no sets. Telemetry carries readback freq/power,
  temperature, `vtxStatus` and (v10) `vtxActualPowerMw` (98 B — the module
  throttles itself at an undocumented temperature, which shows as actual
  < configured; the firmware never lowers power on its own). Diag:
  present, silent/again, every
  readback change, every set/rejection, hex dump of the first `r`/`v`/`s`
  reply and of malformed frames (1 per 5 s), `VTX temp N C at P mW` per
  5 °C change. **Verified on the airframe 2026-11-06**: offsets as in
  the plan's table (`r` says 4990–6199 MHz / 2500 mW — more than the
  manual), pit polarity `I` 0 = enter / `v` byte 7 = 1 in pit, every set
  taken first try with the `v` confirmation ~250 ms later, and a
  Tramp-set configuration **persists across a power cycle** — the module
  shipped at 1600 mW (88 °C on the bench), one session set it to 25 mW
  and it has stayed there. The first `s` after the VTX's own power-up
  reads 0 and is dropped as "no reading yet". The channel is **R2 =
  5695 MHz** from `local.ini` (away from the F4 5800 default every new
  VTX ships with; set 2026-11-06, `VTX set freq 5695` taken first try,
  receiver re-tuned; frequency and power both survive a power cycle).
  Thermal (still air, module loose on the bench — the VTX is mounted
  **outside** the airframe in the slipstream, so this is the worst case):
  25 mW idles at 63 °C, 800 mW settles at ~75 °C (τ ≈ 2.2 min), 1600 mW
  at ~88 °C with no self-throttling seen; the HUD alarm (85 °C) blinks
  on 1600 mW and clears ~30 s after stepping down. Pit mode ≈ 30 cm
  range on the same channel. Ground rule that follows: 25 mW or pit on
  the ground, higher steps only airborne (`doc/parts/fpv-58g-uvc-kit.md`
  §9). cocpitView side (same day):
  `command.CommandState` is the thread-safe desired state the 50 Hz
  sender encodes (channels neutral + VTX freq/power; the joystick lands
  there later); `vtx.py` steps power through the ULTRA's table pit / 25 /
  200 / 400 / 800 / 1600 mW — `V` one step down, `Shift+V` up, `P` pit
  toggle, from the readback on the first press and from the outstanding
  request after, nothing requested until a key; the channel is config
  only (`esp32/local.ini` `[vtx] freq = 5800` or `F4`, `--vtx-freq`
  override, bands A/B/E/F/R validated — the receiver must be retuned
  too). Readback reaches the snapshot outside MAVLink via
  `hub.update_vtx()` (publisher + emulator's fake VTX alike; v≤8 replays
  show no VTX line); HUD `VTX 5800 25mW 42°C` sits between `BAT` and
  `LNK`, `VTX PIT`, `VTX ---` (no Tramp replies), `VTX REFUSED` /
  `VTX LOCKED`, blinking while a request is unmatched > 2 s, while the
  module is hot (`VTX_TEMP_ALARM_C` = 85, the operator's cue to step down
  before the module's own protection does) or throttling (`800/400mW`,
  configured/actual). The emulator's fake VTX warms towards a per-step
  equilibrium (95 °C at 1600 mW) and halves its power from 90 °C, so both
  alarms can be seen without the airframe.
- Next steps: second battery
  calibration point (after the first real flight,
  on the discharged pack); joystick input in
  cocpitView (the command channels are already on the wire, neutral for
  now); servos + a real failsafe action on the air side; fly the yaw
  calibration and measure its two provisional constants; mount the pitot on
  the wing and calibrate its position error in flight (check the static holes
  are not blocked first).
