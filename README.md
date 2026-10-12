# airplane

A hobby RC airplane with a custom ESP32 control link replacing the RC
transmitter, and a Python ground station with an FPV HUD that speaks MAVLink
to QGroundControl.

- `esp32/` — PlatformIO project, two environments: `air` (in the plane:
  IMU, barometer, pitot, GPS, battery monitor, telemetry, failsafe) and
  `ground` (transparent ESP-NOW ↔ USB bridge). Shared wire protocol in
  `esp32/lib/protocol/`.
- `cocpitView/` — ground station app: FPV video with an F-16-style HUD,
  joystick-to-command uplink, telemetry → MAVLink forwarding, flight logging
  and replay, GPS-based heading/wind calibration, terrain-referenced altitude.
- `doc/` — [architecture and design decisions](doc/architecture.md), parts
  notes with wiring and calibration procedures, implementation plans.
- `arduinoNano/` — the legacy MPU-6050 firmware the project started from.

## Getting started

```bash
cp esp32/local.ini.example esp32/local.ini   # ESP-NOW keys + flying site, never committed
cd esp32 && pio run -e air -t upload          # or -e ground; both boards need the same keys
cd ../cocpitView && ./run.sh                  # see cocpitView/README.md
```

Licensed under the [MIT License](LICENSE).
