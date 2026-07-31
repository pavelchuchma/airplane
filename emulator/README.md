# MPU-6050 / ESP32 emulator for QGroundControl

Simulates an ESP32 with an MPU-6050 which, after sensor fusion, sends attitude
(roll/pitch/yaw) and GPS/telemetry (position, altitude, speed) to
QGroundControl over **MAVLink via UDP**. Used to test the wiring and HUD
before the real firmware is written.

A physical version that reads a real MPU-6050 (Arduino Nano) instead of
generating synthetic data lives in `../arduinoNano/` + `mpu6050_hw_bridge.py`
below.

## GPS / position

Start: **50.000000N, 15.000000E**, altitude **400 m AMSL**, horizontal flight
speed **10 m/s** (constants `HOME_LAT_DEG`, `HOME_LON_DEG`, `HOME_ALT_M`,
`AIRSPEED_MPS` in `flight_profile.py`).

Position is computed by numerical integration: the velocity (10 m/s along the
fuselage axis) is rotated by the current orientation (quaternion) into NED
coordinates and integrated over time -> the result is a realistic trajectory
including climbing/descending during a loop (altitude ~400->422->400 m) and
correct heading even during inverted flight (e.g. at the top of a loop,
heading = 180°, not 0°).

MAVLink messages sent: `HEARTBEAT`, `ATTITUDE`, `GPS_RAW_INT` (3D fix, so QGC
stops reporting "No GPS Lock"), `GLOBAL_POSITION_INT` (map position),
`VFR_HUD` (speed/altitude/heading for the classic HUD strip), `ALTITUDE`
(dedicated altitude telemetry - monotonic/AMSL/relative; terrain and
bottom_clearance are NaN, no terrain is modeled).

## Flight profile (currently ~43 s, repeats forever)

Phases are defined in `_SEGMENTS` in `flight_profile.py` (name, duration,
kind, parameters) — currently: straight flight, 90° turn, short straight, 90°
turn back, 90° turn, short straight, 90° turn back, short straight, loop.
Phase durations and order can be freely edited in `_SEGMENTS`;
`TOTAL_DURATION` is computed automatically.

Orientation is computed via quaternions (`quaternion.py`), so the loop (pitch
passing through ±90°) correctly projects onto roll/pitch/yaw including the
gimbal-lock flip, exactly like a real flight controller would report it.

## Installation

```bash
cd emulator
python3 -m venv .venv
./.venv/bin/pip install -r requirements.txt
```

(the venv already exists in this directory with pymavlink installed, so you
can just run the script.)

## Running

```bash
./.venv/bin/python mpu6050_emulator.py
```

Optional parameters:

```bash
./.venv/bin/python mpu6050_emulator.py --host 127.0.0.1 --port 14550 --rate 20
```

- `--host` / `--port` — where to send UDP MAVLink packets (default `127.0.0.1:14550`)
- `--rate` — `ATTITUDE` message rate in Hz (default 20)

## QGroundControl setup

By default QGC automatically listens on UDP port **14550** — just run the
script and the vehicle shows up in QGC on its own (after the first
`HEARTBEAT` arrives), no need to manually add a Comm Link.

If the vehicle doesn't show up:
- **Application Settings → General → Autoconnect** — check that UDP is enabled.
- Or manually: **Application Settings → Comm Links → Add**, type `UDP`, port `14550`.

Once connected, the **artificial horizon** (roll/pitch), map position,
altitude and speed should react in the HUD (Fly View) — exactly following the
profile above. Note: a real bare MPU-6050 (without magnetometer/baro/GPS)
couldn't provide any of that - here the emulator synthetically computes
GPS/altitude just to test integration with QGC.

## Physical (hardware) version

`../arduinoNano/` contains a PlatformIO firmware for an Arduino Nano +
MPU-6050 that reads real sensor data and prints roll/pitch/yaw over USB
serial (no GPS/altitude - a bare MPU-6050 can't measure those, see above).

`mpu6050_hw_bridge.py` in this directory reads that serial stream directly
via `pyserial` and forwards it to QGC as MAVLink `HEARTBEAT`/`ATTITUDE` over
UDP - the same MAVLink/UDP path as `mpu6050_emulator.py`, just fed from real
hardware instead of `flight_profile.py`.

```bash
./.venv/bin/python mpu6050_hw_bridge.py
```

- `--serial-port` — serial port of the Arduino (default: `/dev/cu.usbserial-21230`;
  the CH340 chip gets a new number (`usbserial-XXXXX`) nearly every time it's
  unplugged/replugged, so this default goes stale often — override with
  `--serial-port ...` and check the current one with `ls /dev/cu.*` while the
  Arduino is plugged in)
- `--baud` — serial baud rate (default 115200, matches the firmware)
- `--host` / `--port` / `--system-id` — same meaning as in `mpu6050_emulator.py`

Only one process can hold the serial port at a time, so close
`pio device monitor` before starting the bridge script.

## Files

- `mpu6050_emulator.py` — main script, synthetic MAVLink/UDP loop
- `mpu6050_hw_bridge.py` — reads real MPU-6050 data from `../arduinoNano/` over serial and forwards it via MAVLink/UDP
- `flight_profile.py` — flight profile definition (phases, angles over time)
- `quaternion.py` — quaternion math (composing rotations, conversion to Euler, angular rate)
