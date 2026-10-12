# System architecture

## Overview

Two ESP32 boards connected over ESP-NOW form the link between the airplane and
the ground station PC:

- **air** (in the airplane) — reads sensors, drives servos, and will
  eventually run an autopilot.
- **ground** — a transparent ESP-NOW ↔ USB serial bridge; it does not
  understand the messages it forwards.

Both firmwares live in the `esp32/` PlatformIO project as environments
`env:air` and `env:ground`, sharing message definitions in
`esp32/lib/protocol/`.

## Target topology

```
                 airplane                                ground
┌─────────────────────────────────┐      ┌──────────────────────────────────┐
│ sensors ─┐                      │      │                                  │
│ servos ──┤ ESP32 "air"          │      │ ESP32 "ground"                   │
│          │ (autopilot)          │      │ (dumb bridge)                    │
└──────────┴───────┬──────────────┘      └───────┬──────────────────────────┘
                   │        ESP-NOW (LR mode)    │
                   └────────────◄►───────────────┘
                                                 │ USB serial (framed)
                                        ┌────────▼─────────┐
                                        │ cocpitView       │◄── joystick
                                        │ (protocol ↔      │
                                        │  MAVLink bridge) │
                                        └────────┬─────────┘
                                                 │ MAVLink
                                        ┌────────▼─────────┐
                                        │ QGroundControl   │
                                        └──────────────────┘
```

Data flows:

- **Telemetry (down):** air ESP32 → ESP-NOW → ground ESP32 → USB →
  cocpitView → (translated to MAVLink) → QGroundControl.
- **Control (up):** joystick → cocpitView → USB → ground ESP32 → ESP-NOW →
  air ESP32 → servos. This replaces the RC transmitter entirely.
- **Flight log:** cocpitView records both directions as they cross the USB
  link — every raw downlink payload and every command sent — plus the few
  PC-side facts the wire cannot reproduce (baro calibration offset, a 1 Hz
  liveness heartbeat, serial open/loss) into `cocpitView/logs/*.cvlog`
  (`cockpitview/flightlog.py`). The PC is the logging point because it
  already sees everything; there is no on-air storage.
- **Replay:** `--replay FILE.cvlog` plays the recorded downlink payloads
  back through the same publisher the live link uses
  (`cockpitview/replay_source.py` → `publisher.py`), so an old flight is
  re-flown on today's HUD, calibrators and QGC.

## Current vs. target state

cocpitView is already the hub: it reads telemetry itself (USB serial from the
ground bridge by default, a built-in flight-profile emulator via `--emulator`,
or a recorded flight via `--replay`)
and feeds QGC over MAVLink/UDP. The standalone `mpu6050_*.py` scripts are
retired. The air firmware reads the GY-91 board: attitude from the IMU and
altitude/vario from the BMP280 barometer (relative to the boot location,
translated to MAVLink VFR_HUD + ALTITUDE by cocpitView). Note the GY-91 turned
out to carry an MPU-6500, not the advertised MPU-9250 — there is no
magnetometer, so yaw is gyro-integration only: its datum is whatever heading
the aircraft booted on, and it drifts from there. It is calibrated in flight
against the GPS (`cockpitview/yawcal.py`, see below), which avoids compass
calibration and the airframe's magnetic interference entirely.

The MPU-6500 is configured at boot, not left at its power-on defaults: gyro
DLPF 20 Hz and accel DLPF 10 Hz act as the anti-aliasing filters for the
50 Hz polling (the chip otherwise samples at 1–8 kHz with 250/460 Hz
bandwidth, and motor vibration folds into the band as fake rotation), the
accel range is ±16 g so vibration peaks do not clip the ADC (a clipped mean
biases the accel-derived angle and the complementary filter converges to
it), and the gyro range is ±500 °/s. The complementary filter takes the
accelerometer only when |a| is within 0.3 g of 1 g; otherwise the sample is
gyro-only. The config is read back once a second and rewritten if the chip
has lost it — a sensor reset would otherwise silently restore the ±2 g,
unfiltered defaults with an 8× wrong scale. Motivated by the 2026-10-11
ground motor run, where roll and pitch wandered by 10–30° and shook by
12–24° per second while the airframe was held still.

The IMU's body frame is X-forward, **Y-left**, Z-up (FLU) — forced by the
mounting (board upside down) and the requirement that the sensor→body
transform stay a proper rotation. Roll and pitch carry the usual aerospace
signs there, but a positive rotation about Z is a *left* turn, so the air
side flips yaw and yawRate once, where it fills telemetry, and the wire
carries a clockwise heading (protocol v5). Up to v4 it did not, and the
mirrored heading reached the HUD, QGC and the wind-triangle fit alike —
`flightlog.upgrade()` corrects logs of those versions on read. The Quescan
G10A-F30 GPS (u-blox M10) is on
UART2: the firmware configures it at every boot (flash-less chip — 5 Hz nav
rate, airborne dynamic model, RMC+GGA only) and appends position/groundspeed/
course/fix-quality to the CSV; cocpitView forwards them as GPS_RAW_INT +
GLOBAL_POSITION_INT, which gives QGC the map position and the HUD its
flight-path marker, AGL readout and a GPS fix/satellite status line.
Altitude display never uses the (noisy, tens of meters) GPS altitude:
cocpitView calibrates the baro once on the ground against the local terrain
elevation from a pre-fetched CUZK DMR 5G grid at the GPS *position*
(`cockpitview/altimeter.py`), then shows baro AMSL in the altitude box and
`R = baro AMSL − terrain(position)` as the radar altitude.
That calibration and the *launch point* are the same event, so both come from
one gate in `cockpitview/home.py`: the first 25 GPS epochs that are 3D, carry
a reported 1-sigma error ≤ 5 m, are taken below 1 m/s (still on the ground)
and have a fresh baro sample; the median of those fixes locks
`(lat, lon, baro_rel)` and the altimeter, registered as a sink on the very
same samples, adds only the terrain lookup on top. So a launch outside the
fetched grid loses the AMSL/AGL readouts but keeps the launch point, and
starting cocpitView mid-flight leaves both off for the session — nothing that
happens in the air could establish either. The launch point drives the HUD's
`DST` line (slant range home + the baro altitude difference against it); its
vertical leg is a pure baro difference, never GPS altitude, and it needs no
terrain grid.
The MS4525DO pitot sensor sits on the same I2C bus (address 0x28): its zero
offset is re-measured at every boot (the offset is tens of Pa and drifts
with temperature — the airplane must be stationary), the differential
pressure is low-pass filtered and converted to indicated airspeed at fixed
sea-level density; both travel in telemetry and cocpitView feeds the IAS to
`VFR_HUD.airspeed`, which drives the HUD's airspeed tape.
The sensor's scale factor is verified (a 981 Pa water column reads back as the
expected 40.0 m/s), but the IAS carries **no position-error correction yet** —
a first bench test on a bicycle read ~30 % low purely from where the static
holes sat in the rider's pressure field. That correction is a property of the
installation, not of the sensor, so it can only be measured on the airframe
(two reciprocal legs against GPS) and it will live in its own constant, kept
apart from the sensor's pressure range. Until then treat the airspeed tape as
qualitative. See `doc/parts/pitot-ms4525do.md` for the mounting analysis and
the calibration procedure.

All three I2C sensors share one failure model and one recovery mechanism
(`SensorLifecycle` in the air main.cpp). The only failure designed for is a
temporary bus disturbance — transactions fail or ACK corrupted data for a
while, then the bus recovers; the sensors keep their power (and config
registers) throughout, so recovery never re-initializes anything: a
plausible data read is the proof of health. Each sensor dies time-based
(0.5 s without a good read for IMU/pitot, 1 s for the slower baro), its
telemetry fields go NAN (`---` on the HUD instead of a false 0, the frozen
attitude is flagged via `TELEM_FLAG_IMU_OK` clear so cocpitView stops
publishing ATTITUDE and the HUD hides horizon and compass), and revival
takes 10 consecutive plausible reads — data plausibility checks (pitot
10–90 % count band, baro 300–1100 hPa, IMU |a| ≥ 0.05 g) are the defense
against corrupted-but-ACKed reads, since I2C has no data integrity check.
A dead sensor is probed at most every 2 s (its failed transactions are
slow), the pitot resumes with the boot zero offset (in-flight re-zeroing is
impossible, it needs still air). A sensor absent at boot — probe or
calibration failed twice — stays absent for the session: with no power
loss in flight there is nothing that could make it appear later.
Bus-level faults are handled separately: boot clocks a stuck slave free
before `Wire.begin()` (an MCU reset mid-transaction otherwise leaves SDA
held low across reboots, taking down every device on the bus), and when
every present sensor has been silent for 2 s in flight the same rescue
reruns (`Wire.end()` → clear → `Wire.begin()`, every 5 s) — a disturbance
can abort a transaction mid-byte with the same SDA-stuck result. loop()
distrusts a sensor snapshot older than 200 ms and clears the OK flags, so
even a wedged sensor task cannot keep telemetry looking healthy.
Crucially, all I2C access runs in a dedicated FreeRTOS task (core 0), not in
the link loop: the Arduino/IDF I2C driver can stall ~1 s per transaction on
a disturbed bus no matter what `Wire.setTimeOut` says (measured 1006 ms with
a 20 ms timeout), and with sensors read inline that collapsed the 50 Hz loop
— telemetry, failsafe timing and all — to ~0.5 Hz. The loop only copies the
task's latest snapshot under a spinlock, so a bus pathology can only ever
delay sensor data, never the link.

The ESP-NOW link is implemented: the air firmware sends one `TelemetryMessage`
per sample (50 Hz) and receives `CommandMessage`s (lib/protocol/messages.h),
the ground bridge forwards opaque payloads between ESP-NOW and framed USB
serial, and cocpitView (`cockpitview/espnow_source.py` + `protocol.py`) reads
the bridge, sends 50 Hz commands — neutral channels until the joystick
exists — and shows link quality (`LNK -72 U2% D0%`) and the air-side
`FAILSAFE` flag on the HUD, plus RADIO_STATUS/STATUSTEXT for QGC. Remaining
follow-ups: joystick input, and servos + a real failsafe action (today the
flag and a `#` diagnostic are the whole action).

### Heading calibration without a magnetometer

GPS course over ground is not the missing compass: it is where the aircraft
*moves*, which differs from where its nose points by the wind drift angle —
18° already at 5 m/s of wind and 15 m/s of flight. Feeding `COG − yaw` back
would calibrate the wind, not the gyro. What *is* solvable is the whole wind
triangle at once. With the pitot's airspeed, the GPS ground velocity and the
offset, drifting gyro yaw ψ:

    v_ground = k · IAS · u(ψ + Δ) + wind

the substitution `a = k·cosΔ`, `b = k·sinΔ` makes this linear in
`(a, b, wind_n, wind_e)`, so ordinary least squares over a window of samples
yields the yaw offset `Δ = atan2(b, a)`, the wind vector — worth its own HUD
line before landing — and `k = hypot(a, b)`, the scale between indicated and
true airspeed, i.e. an in-flight measurement of the pitot's position error
that would otherwise need reciprocal legs against GPS.

The catch is observability: with every sample on one heading the system is
singular and no amount of averaging separates the offset from the wind.
Flying an arc is what makes it solvable, and the condition number of the
(column-equilibrated) design matrix states exactly how well — 1.0 for a full
360°, 3.5 for a 0–90° sweep, 9.9 for ±20°, 10¹⁵ for straight flight. A window
above `MAX_CONDITION` is rejected outright rather than returning a confident
looking number. Two time scales are then kept apart: wind is quasi-constant
only over a minute, which bounds the window; the offset drifts slowly and
linearly, which a two-state (offset, drift rate) Kalman filter tracks across
windows. Each fit is applied at the *centroid* of its window, so a 60 s
window costs no lag, and only windows disjoint from the last one fed reach
the filter — consecutive windows share 59/60 of their samples, and feeding
all of them would let the filter mistake one measurement for sixty.

The calibrator sits in the `TelemetryHub` processor chain and rewrites the
heading in ATTITUDE/VFR_HUD, so the HUD tape, the flight path marker and QGC
all get a true heading without knowing the module exists. Until an estimate
exists it rewrites nothing and the HUD boxes the heading as `~037` — the boot
datum is still worth reading for relative turns, it just must not be mistaken
for a compass. Gate constants (`GPS_VELOCITY_LAG_S`, the drift PSD) are
provisional until measured on an airframe;
`python -m cockpitview.flightlog yawcal FILE.cvlog` replays a log through the
very same gate, window and filter objects the live path uses, which is how
they get their numbers.

Every session is logged (`--no-log` to opt out): `flightlog.py` writes the
raw wire payloads rather than the derived MAVLink, because the MAVLink stream
is lossy (NaN → 0, integer scaling, messages suppressed while a sensor is
dead, altitude rewritten by the baro calibration) and fully derivable from
the raw records, never the reverse. The `.cvlog` container is a sequence of
`<dBH`-prefixed records (monotonic time, type, length); the first record is a
JSON header carrying the struct formats, field names and flag map of the
protocol version in use, so old logs stay decodable after `PROTOCOL_VERSION`
bumps — readers decode by the header, not by the current `protocol.py`.
Producers only queue bytes; a writer thread owns all file I/O, opens the file
on the first telemetry frame and flushes once per second (a crash loses
≤ 1 s, a truncated tail is dropped on read). Post-flight use:
`python -m cockpitview.flightlog info|csv|events FILE`, or
`flightlog.to_dataframe()` with pandas — this is where the pitot position
error (GPS vs. IAS on reciprocal legs) and the second battery calibration
point will come from.

`--replay FILE.cvlog` is the third telemetry source
(`cockpitview/replay_source.py`, plan
`doc/plans/2026-09-10-flightlog-replay.md`): it paces the logged downlink
payloads by their own timestamps into `publisher.TelemetryPublisher`, the
same code the live link publishes through, so the HUD, the home point, the
altimeter, the yaw calibration and QGC all re-run on the old flight as they
are *today*. Frames are decoded by the log's header and passed through
`flightlog.upgrade()`, so a v4 log replays with its mirrored yaw corrected.
`--replay-start MM:SS` jumps into the flight, fast-forwarding the run-up
through the processors (with the UDP forward suppressed) so home and the
calibrations hold at the jump-in point; `--replay-cold` skips it instead,
which is the only way to exercise a mid-flight start. The calibrators run on
the log's clock, not the viewer's.

## Power architecture (airplane, planned)

Source: **3S1P Samsung INR18650-30Q Li-ion pack** (12.6 V full, ~11.1 V
nominal, 3000 mAh / ~33 Wh; `doc/parts/battery-samsung-30q.md`). All rails
share a common ground, wired star-style from the converter outputs. Per-cell
voltages are monitored from the balance connector via ADC1 dividers (see
`doc/parts/README.md`, Battery monitoring).

The cells are **unprotected** and the ESC's adjustable cutoff is the only
automatic protection in the airplane. Two operating facts belong here rather
than in the parts doc, because they shape what the telemetry can tell you:
the routine landing target is **3.5 V/cell resting** (~20–25 % left; the
9.0 V figure that used to head this section is the absolute floor, not a
target), and full throttle costs **~0.3–0.36 V/cell** of indicated voltage
(measured on the airframe from the balance taps, which carry no current), so
state of charge must be read at low throttle. The HUD's 3.2 V/cell alarm
otherwise fires with roughly a quarter of the pack remaining.

```
3S Li-ion (9.0–12.6 V)
 ├─ ESC (RAY G2 R-20B, 20 A) ─► motor
 │    └─ linear BEC 5 V ──────► servos (PCA9685 V+)
 ├─ buck → 5 V ► logic:                                   (Mini-360)
 │    ├─ ESP32 devkit VIN (on-board AMS1117 feeds the ESP32 only)
 │    ├─ MS4525DO (5 V part; RC filter 10–22 Ω + 100 µF)
 │    └─ LDO 3.3 V (HT7833/ME6211) ► GY-91, GPS
 └─────────────────► FPV camera + VTX  (direct from pack — measured, see below)
```

Reasoning behind the choices:

- **Servos run off the ESC's BEC; the ESP32 never does.** The RAY G2 R-20B
  has a *linear* 5 V/2 A BEC, and on 3S that rating is fiction — dropping
  12.6 V to 5 V burns 7.6 W per amp on a 19 g board. What makes it work is
  that servos idle far below stall: three servos holding trim plus a receiver
  draw ~0.3 A (≈2.3 W, survivable in prop wash), and the ~0.95 A peak when
  all three move at once lasts fractions of a second, which the board's
  thermal mass absorbs. This is measured, not assumed — the same ESC already
  flies the existing 3S model with three servos and a RadioLink receiver.
  The tail risk is a *sustained* overload: a jammed linkage or a surface held
  against its servo pulls ~1.2 A continuously (~9 W), which cooks the BEC and,
  on thermal shutdown, takes the servos with it. That risk is identical in the
  existing model and is accepted. The ESP32 is a different matter — its
  300–500 mA ESP-NOW bursts do not fit in 0.3 A of headroom, and a BEC
  thermal cutout would kill the failsafe brain, so logic gets its own
  converter straight off the pack. Only GND is shared.
- **Never parallel two 5 V sources.** If a dedicated servo BEC is ever added
  (bigger servos, flaps, retracts), the ESC's red wire must come out of its
  connector first — two regulators tied together push current into each other.
  Signal and ground only; the ESC still needs GND as its PWM reference.
- **Camera + VTX run directly off the pack — a buck was tried and measured
  worse.** An earlier revision of this section planned a Mini-360 at ~7.5 V
  to relieve what was assumed to be linear-regulator heat inside the VTX,
  plus an LC filter against buck ripple in the analog video. Bench
  measurement falsified both assumptions. The chain (camera + 200 mW VTX)
  draws 312 mA at 12 V direct (≈3.7 W) but **350 mA from the battery when
  fed through the buck at 7.5 V** — the VTX generates its internal rails
  with *switching* regulators, so it is a constant-power load: lowering its
  input voltage only raises its input current (≈0.5 A at 7.5 V), and the
  buck's ~90 % efficiency is pure added loss (~0.45 W; the module runs warm
  doing nothing useful). The VTX heatsink heat is RF power-amplifier
  dissipation (PA efficiency ~25–35 %), independent of supply voltage — the
  remedy is airflow over the heatsink, not a lower rail. The interference
  fear didn't materialize either: full-throttle motor runs showed no visible
  noise in the video, direct or via the buck (bench test — recheck after
  installation in the airframe; if lines ever appear, the fix is an LC
  filter, not a converter). A 2S tap stays rejected for the old reasons
  (pack imbalance; 2S sags to 6.0–6.6 V near empty, below the VTX's 7 V
  minimum) — but there is no longer any reason to want a lower voltage at
  all. Diagnostic worth remembering: a load that *would* benefit from a
  lower rail shows the opposite signature — battery current roughly
  independent of voltage (constant current = linear regulator behind the
  input).
- **Sensors do not hang on the devkit's 3.3 V pin.** The devkit's AMS1117 is
  marginal (clone quality, ~1.1 V dropout) and the ESP32's 50 Hz ESP-NOW LR
  bursts pull 300–500 mA spikes through it; a separate low-dropout LDO
  (HT7833, ~0.3 V dropout) keeps the sensor rail clean — it rides through 5 V
  rail dips and keeps radio spikes out of the baro/vario data.
- **The "Mini-360" modules are MP2307, not MP1584** — verified 340 kHz
  switching (MP1584 runs 500 kHz–1.5 MHz). Both chips ship under the same
  module name and the specs get quoted interchangeably, so check the marking
  on what actually arrives. With the video buck dropped, only the logic 5 V
  rail uses one. (The MP2307 being *synchronous* — one predictable ripple
  frequency instead of light-load pulse-skipping — mattered for the video
  filter design; it's irrelevant on the logic rail.) Real-world continuous
  limit is ~1–1.5 A despite the "3 A" print. Set the trimmer under no load,
  verify with a meter before connecting anything, and lock it with a drop of
  varnish — vibration can turn it.
- The MS4525DO in the kit is the **DS5 (5 V) variant** — running it at 3.3 V
  is within absolute maximums but outside calibration spec (scale error);
  it stays on the 5 V rail with I2C pulled up to 3.3 V (open-drain, so safe
  for the ESP32 — same arrangement as Pixhawk).

## Design decisions

- **One PlatformIO project, two environments.** The wire protocol must change
  atomically on both sides; shared structs in `lib/protocol/` are compiled
  into both firmwares.
- **The ground firmware is protocol-agnostic.** It only frames packets between
  ESP-NOW and USB serial (SLIP/COBS + CRC on the serial side, since serial
  does not preserve packet boundaries). Protocol changes therefore touch only
  the air firmware and cocpitView — the ground firmware is written once.
- **Custom compact messages over ESP-NOW, MAVLink only in cocpitView.**
  ESP-NOW payloads are limited to 250 B and the radio link benefits from
  small packets; cocpitView translates to MAVLink for QGC.
- **Protocol lives in two languages.** C++ structs in
  `esp32/lib/protocol/messages.h` and a Python mirror in cocpitView
  (`protocol.py`). Every wire-format change bumps `PROTOCOL_VERSION`; the air
  side reports a version mismatch via telemetry so drift is caught
  immediately.
- **The link is safety-critical** (it replaces the RC transmitter):
  - control commands are sent at a fixed rate (50 Hz) regardless of change,
    so silence reliably means link loss;
  - the air firmware has a failsafe from day one: no valid command for
    500 ms (25 consecutive lost frames) → defined action (today just the
    telemetry flag + a `#` diagnostic — no servos yet; later level + hold,
    then RTH via autopilot);
  - telemetry includes RSSI / packet-loss stats so link quality is visible in
    cocpitView before the airplane is lost;
  - ESP32 long-range mode (`WIFI_PROTOCOL_LR`) is enabled — the default
    ESP-NOW range is marginal for flying.
- **Chosen link parameters** (see `lib/protocol/`):
  - one combined 90 B `TelemetryMessage` at 50 Hz (GPS fields simply repeat
    between the 5 Hz GPS updates) — ~4.5 kB/s, comfortable even at the LR
    mode's reduced bitrate, and one parse path instead of two;
  - serial framing `COBS(payload‖CRC-16/CCITT-FALSE) + 0x00` — fixed +1 B
    COBS overhead and trivial resync on the delimiter; the ground UART runs
    at 460800 baud (the clone CP2102s are not reliable at 921600) and
    carries framed binary only, no text;
  - WiFi channel 1, both radios in STA mode with power save off (the default
    modem sleep drops ESP-NOW frames); peers are the factory MACs from
    `platformio.ini`, compiled in as `-DPEER_MAC_STR` by `tools/peer_mac.py`;
  - the radio hop is encrypted with ESP-NOW's built-in PMK/LMK (CCMP in
    hardware; the keys live in the git-ignored `esp32/local.ini`, see
    `local.ini.example`, and reach both firmwares as `-DLINK_*_HEX` via
    `tools/link_keys.py` — air + ground only, the USB side is trusted). A key/channel mismatch fails silently: a dead link right
    after touching either means suspect the keys first;
  - RSSI of the peer's frames is sniffed via a promiscuous-mode callback —
    the Arduino core 2.x / IDF 4.4 ESP-NOW receive callback carries no RSSI
    (that arrived with `esp_now_recv_info_t` in core 3.x; the callback
    signatures change too if the platform is ever bumped).
