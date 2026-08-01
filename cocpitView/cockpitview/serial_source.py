"""Serial telemetry source - the air ESP32 over USB serial (default mode).

Reads CSV lines (roll,pitch,yaw,rollspeed,pitchspeed,yawspeed [rad, rad/s],
50 Hz, see ../esp32/src/air/) and publishes them to the TelemetryHub as
MAVLink ATTITUDE plus a 1 Hz HEARTBEAT.

By default the serial port is resolved by the air board's factory MAC
address (the boards' clone CP2102s share one USB serial number, so port
names are ambiguous - see ../esp32/tools/board_id.py). The MAC and the
probe-order hint come from [env:air] in ../esp32/platformio.ini. Note the
probe resets the board, so the stream starts with a fresh gyro calibration.

Unlike the retired emulator/mpu6050_hw_bridge.py this never exits: while the
board is missing (or the link drops) it keeps retrying and the HUD shows
NO TELEMETRY.
"""

import configparser
import pathlib
import sys
import threading
import time

import serial

from .telemetry import heartbeat

ESP32_DIR = pathlib.Path(__file__).resolve().parent.parent.parent / "esp32"
sys.path.insert(0, str(ESP32_DIR / "tools"))
import board_id  # noqa: E402

HEARTBEAT_PERIOD_S = 1.0
RETRY_PERIOD_S = 3.0


def find_air_port():
    """Resolve the air ESP32's serial port by its factory MAC (resets the board)."""
    config = configparser.ConfigParser(inline_comment_prefixes=(";",))
    config.read(ESP32_DIR / "platformio.ini")
    mac = config.get("env:air", "custom_board_mac", fallback="").strip()
    if not mac:
        raise LookupError(f"custom_board_mac not set for [env:air] in {ESP32_DIR / 'platformio.ini'}")
    hint = config.get("env:air", "custom_usb_location", fallback="").strip() or None
    return board_id.find_board(mac, hint)


def parse_line(line):
    parts = line.strip().split(",")
    if len(parts) != 6:
        return None
    try:
        return tuple(float(p) for p in parts)
    except ValueError:
        return None


class SerialSource:
    """Background thread feeding the hub from the air ESP32's serial port."""

    def __init__(self, hub, port=None, baud=115200):
        self._hub = hub
        self._port = port
        self._baud = baud
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def _run(self):
        while True:
            try:
                port = self._port or find_air_port()
            except (LookupError, RuntimeError) as err:
                print(f"air board not found: {err} - retrying in {RETRY_PERIOD_S:.0f} s")
                time.sleep(RETRY_PERIOD_S)
                continue
            try:
                self._pump(port)
            except (serial.SerialException, OSError) as err:
                print(f"serial link lost ({err}) - reconnecting")
                time.sleep(RETRY_PERIOD_S)

    def _pump(self, port):
        with serial.Serial(port, self._baud, timeout=1.0) as ser:
            print(f"Reading air telemetry from {port} @ {self._baud} baud")
            start = time.monotonic()
            last_heartbeat = 0.0
            while True:
                line = ser.readline().decode("ascii", errors="ignore")
                if not line:
                    continue  # timeout with no data, try again

                values = parse_line(line)
                if values is None:
                    continue  # e.g. startup/calibration output or a truncated line

                roll, pitch, yaw, rollspeed, pitchspeed, yawspeed = values
                now = time.monotonic()
                time_boot_ms = int((now - start) * 1000) & 0xFFFFFFFF

                if now - last_heartbeat >= HEARTBEAT_PERIOD_S:
                    self._hub.publish(heartbeat(self._hub.mav))
                    last_heartbeat = now

                self._hub.publish(
                    self._hub.mav.attitude_encode(
                        time_boot_ms, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed
                    )
                )
