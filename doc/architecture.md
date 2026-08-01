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
  - one combined 76 B `TelemetryMessage` at 50 Hz (GPS fields simply repeat
    between the 5 Hz GPS updates) — ~3.8 kB/s, comfortable even at the LR
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
