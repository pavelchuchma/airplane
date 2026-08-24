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

## Current vs. target state

cocpitView is already the hub: it reads telemetry itself (USB serial from the
air ESP32 by default, or a built-in flight-profile emulator via `--emulator`)
and feeds QGC over MAVLink/UDP. The standalone `mpu6050_*.py` scripts are
retired. The air firmware reads the GY-91 board: attitude from the IMU and
altitude/vario from the BMP280 barometer (relative to the boot location,
translated to MAVLink VFR_HUD + ALTITUDE by cocpitView). Note the GY-91 turned
out to carry an MPU-6500, not the advertised MPU-9250 — there is no
magnetometer, so yaw is gyro-integration only and drifts; the plan is to
correct it from GPS course-over-ground (fine for a fixed-wing, and it avoids
compass calibration entirely). The Quescan G10A-F30 GPS (u-blox M10) is on
UART2: the firmware configures it at every boot (flash-less chip — 5 Hz nav
rate, airborne dynamic model, RMC+GGA only) and appends position/groundspeed/
course/fix-quality to the CSV; cocpitView forwards them as GPS_RAW_INT +
GLOBAL_POSITION_INT, which gives QGC the map position and the HUD its
flight-path marker, AGL readout and a GPS fix/satellite status line.
Altitude display never uses the (noisy, tens of meters) GPS altitude:
cocpitView calibrates the baro once on the ground against the local terrain
elevation from a pre-fetched CUZK DMR 5G grid at the GPS *position*
(`cockpitview/altimeter.py` — assumes the plane sits on the ground when the
first fix arrives), then shows baro AMSL in the altitude box and
`R = baro AMSL − terrain(position)` as the radar altitude.
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
follow-ups: joystick input, servos + a real failsafe action (today the flag
and a `#` diagnostic are the whole action), and the GPS course-over-ground
yaw correction (above a groundspeed threshold).

## Power architecture (airplane, planned)

Source: **3S Li-ion pack** (9.0–12.6 V). All rails share a common ground,
wired star-style from the converter outputs. Per-cell voltages are monitored
from the balance connector via ADC1 dividers (see `doc/parts/README.md`,
Battery monitoring).

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
    hardware, `lib/protocol/link_keys.h`, air + ground only — the USB side
    is trusted). A key/channel mismatch fails silently: a dead link right
    after touching either means suspect the keys first;
  - RSSI of the peer's frames is sniffed via a promiscuous-mode callback —
    the Arduino core 2.x / IDF 4.4 ESP-NOW receive callback carries no RSSI
    (that arrived with `esp_now_recv_info_t` in core 3.x; the callback
    signatures change too if the platform is ever bumped).
