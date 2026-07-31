# Arduino Nano + MPU-6050 - physical version of the emulator

Physical counterpart of `../emulator/mpu6050_emulator.py`: instead of a
synthetically generated attitude, it reads a real MPU-6050 over I2C, computes
roll/pitch/yaw and sends it over USB serial. Without a magnetometer/baro/GPS
it can't provide heading (drifts), altitude, or position - same limitation
already noted in `../emulator/README.md`.

## Hardware

- Arduino Nano V3 (CH340 USB-serial chip clone, ATmega328P)
- MPU-6050 (I2C accelerometer + gyroscope)

## Wiring

| MPU-6050 | Arduino Nano |
|----------|--------------|
| VCC      | 5V           |
| GND      | GND          |
| SCL      | A5           |
| SDA      | A4           |
| AD0      | GND (I2C address 0x68) |

The module should be mounted rigidly, oriented with its X axis forward and Z
axis up (when lying flat) - otherwise you'll need to adjust the signs in
`src/main.cpp`.

## Build and upload

```bash
pio run              # build
pio run -t upload     # flash the Nano (USB cable)
pio device monitor    # manual debugging only - live-prints the CSV lines
```

CH340 clones often have the old bootloader (board `nanoatmega328`, upload at
57600 baud - see `platformio.ini`). If upload fails, try
`board = nanoatmega328new` + `upload_speed = 115200` (newer Optiboot).

At startup (`setup()`) there's a ~1s gyro calibration against its resting
bias - the Arduino must be lying still at that point.

## Output format (Serial, 115200 baud, 50 Hz)

```
roll,pitch,yaw,rollspeed,pitchspeed,yawspeed
```

All 6 values are floats: angles in radians, angular rates in rad/s. Each
line is terminated with `\n` (`Serial.println`).

## Forwarding to QGroundControl

`pio device monitor` is only an interactive debugging tool (it adds its own
timestamps/colors and holds the port exclusively) - it's not suitable as a
data source for further processing. Instead use
`../emulator/mpu6050_hw_bridge.py`, which opens the port directly via
`pyserial` and forwards the lines as MAVLink `HEARTBEAT`/`ATTITUDE` to QGC
over UDP - see `../emulator/README.md`.
