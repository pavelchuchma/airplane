#!/usr/bin/env python3
"""
Reads CSV lines (roll,pitch,yaw,rollspeed,pitchspeed,yawspeed [rad, rad/s])
from the Arduino Nano + MPU-6050 serial link (see ../arduinoNano/) and
forwards them to QGroundControl as MAVLink HEARTBEAT/ATTITUDE over UDP - the
physical counterpart of mpu6050_emulator.py (no GPS/altitude, since a bare
MPU-6050 can't measure those).

Usage:
    ./.venv/bin/python mpu6050_hw_bridge.py
    ./.venv/bin/python mpu6050_hw_bridge.py --serial-port /dev/cu.wchusbserial1410 --host 127.0.0.1 --port 14550
"""

import argparse
import time

import serial
from pymavlink import mavutil

HEARTBEAT_PERIOD_S = 1.0


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
        "--serial-port", default="/dev/cu.usbserial-21230", help="Arduino serial port (default: /dev/cu.usbserial-21230)"
    )
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate (default: 115200)")
    parser.add_argument("--host", default="127.0.0.1", help="QGroundControl IP address (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=14550, help="QGC UDP port (default: 14550)")
    parser.add_argument("--system-id", type=int, default=1, help="MAVLink system id of the vehicle")
    args = parser.parse_args()

    ser = serial.Serial(args.serial_port, args.baud, timeout=1.0)
    conn = mavutil.mavlink_connection(
        f"udpout:{args.host}:{args.port}",
        source_system=args.system_id,
        source_component=mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
    )

    print(f"Reading {args.serial_port} @ {args.baud} baud, sending MAVLink to {args.host}:{args.port}")
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
