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

Today telemetry takes the path
`ESP → mpu6050_hw_bridge.py → QGC → cocpitView`. Once the ground bridge and
the cocpitView serial module exist, `mpu6050_hw_bridge.py` is retired and
cocpitView becomes the hub that feeds QGC.

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
  - control commands are sent at a fixed rate (~50 Hz) regardless of change,
    so silence reliably means link loss;
  - the air firmware has a failsafe from day one: no command for X ms →
    defined action (level + hold, later RTH via autopilot);
  - telemetry includes RSSI / packet-loss stats so link quality is visible in
    cocpitView before the airplane is lost;
  - ESP32 long-range mode (`WIFI_PROTOCOL_LR`) is enabled — the default
    ESP-NOW range is marginal for flying.
