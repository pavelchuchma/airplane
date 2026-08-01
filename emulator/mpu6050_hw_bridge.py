#!/usr/bin/env python3
"""
Reads CSV lines (roll,pitch,yaw,rollspeed,pitchspeed,yawspeed [rad, rad/s])
from the air ESP32 + MPU-6050 serial link (see ../esp32/src/air/) and
forwards them to QGroundControl as MAVLink HEARTBEAT/ATTITUDE over UDP - the
physical counterpart of mpu6050_emulator.py (no GPS/altitude, since a bare
MPU-6050 can't measure those).

By default the serial port is resolved by the air board's factory MAC
address (the boards' clone CP2102s share one USB serial number, so port
names are ambiguous — see ../esp32/tools/board_id.py). The MAC and the
probe-order hint come from [env:air] in ../esp32/platformio.ini. Note the
probe resets the board, so the stream starts with a fresh gyro calibration.

Usage:
    ./.venv/bin/python mpu6050_hw_bridge.py
    ./.venv/bin/python mpu6050_hw_bridge.py --serial-port /dev/cu.wchusbserial1410 --host 127.0.0.1 --port 14550
"""

import argparse
import configparser
import pathlib
import sys
import time

import serial
from pymavlink import mavutil

ESP32_DIR = pathlib.Path(__file__).resolve().parent.parent / "esp32"
sys.path.insert(0, str(ESP32_DIR / "tools"))
import board_id

HEARTBEAT_PERIOD_S = 1.0


def find_air_port():
    """Resolve the air ESP32's serial port by its factory MAC (resets the board)."""
    config = configparser.ConfigParser(inline_comment_prefixes=(";",))
    config.read(ESP32_DIR / "platformio.ini")
    mac = config.get("env:air", "custom_board_mac", fallback="").strip()
    if not mac:
        sys.exit(f"custom_board_mac not set for [env:air] in {ESP32_DIR / 'platformio.ini'}")
    hint = config.get("env:air", "custom_usb_location", fallback="").strip() or None
    try:
        return board_id.find_board(mac, hint)
    except (LookupError, RuntimeError) as err:
        sys.exit(f"air board not found: {err}")


def parse_line(line):
    parts = line.strip().split(",")
    if len(parts) != 6:
        return None
    try:
        return tuple(float(p) for p in parts)
    except ValueError:
        return None


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--serial-port",
        default=None,
        help="serial port of the air ESP32 (default: resolved by USB location from esp32/platformio.ini)",
    )
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate (default: 115200)")
    parser.add_argument("--host", default="127.0.0.1", help="QGroundControl IP address (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=14550, help="QGC UDP port (default: 14550)")
    parser.add_argument("--system-id", type=int, default=1, help="MAVLink system id of the vehicle")
    args = parser.parse_args()

    serial_port = args.serial_port or find_air_port()
    ser = serial.Serial(serial_port, args.baud, timeout=1.0)
    conn = mavutil.mavlink_connection(
        f"udpout:{args.host}:{args.port}",
        source_system=args.system_id,
        source_component=mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
    )

    print(f"Reading {serial_port} @ {args.baud} baud, sending MAVLink to {args.host}:{args.port}")
    print("Ctrl+C to stop.\n")

    start = time.monotonic()
    last_heartbeat = 0.0

    try:
        while True:
            line = ser.readline().decode("ascii", errors="ignore")
            if not line:
                continue  # timeout with no data, try again

            values = parse_line(line)
            if values is None:
                continue  # e.g. startup/calibration output or a truncated line

            roll, pitch, yaw, rollspeed, pitchspeed, yawspeed = values
            now = time.monotonic()
            t = now - start
            time_boot_ms = int(t * 1000) & 0xFFFFFFFF

            if now - last_heartbeat >= HEARTBEAT_PERIOD_S:
                conn.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_FIXED_WING,
                    mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    0,
                    mavutil.mavlink.MAV_STATE_ACTIVE,
                )
                last_heartbeat = now

            conn.mav.attitude_send(time_boot_ms, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
