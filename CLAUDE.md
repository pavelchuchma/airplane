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

- `esp32/src/air` is a 1:1 port of the arduinoNano MPU-6050 firmware: the air
  ESP32 connects directly to the PC over USB serial (CSV lines, 115200 baud),
  no ESP-NOW yet. `esp32/src/ground` is still an empty skeleton. The IMU on
  the airframe is now a GY-91 (MPU-9250 + BMP280) — drop-in compatible with
  the MPU-6050 code; magnetometer and barometer are not read yet.
- cocpitView is the telemetry hub: it reads the air ESP32's serial CSV
  directly (or a built-in emulator via `--emulator`) and forwards MAVLink to
  QGC (UDP 14550). The old standalone `emulator/mpu6050_*.py` scripts are
  gone; the flight-profile emulator lives in the `cockpitview` package.
- Next steps: move telemetry onto the ESP-NOW link (air → ground bridge →
  cocpitView) using the protocol structs in `esp32/lib/protocol/`; read the
  GY-91 extras (magnetometer → drift-free yaw, BMP280 → altitude/vario) in
  the air firmware.
