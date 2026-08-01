"""Serial telemetry source - the air ESP32 over USB serial (default mode).

Reads CSV lines (roll,pitch,yaw,rollspeed,pitchspeed,yawspeed,alt,climb,
lat,lon,alt_msl,gs,cog,sats,hdop,hacc,vacc - rad, rad/s, m relative to boot
location, m/s, then degrees/m/mps/degrees plus 1-sigma error estimates in m
for the GPS block; 50 Hz, see ../esp32/src/air/) and publishes them to the
TelemetryHub as MAVLink
ATTITUDE + VFR_HUD + ALTITUDE + GPS_RAW_INT + GLOBAL_POSITION_INT plus a
1 Hz HEARTBEAT. alt/climb come from the BMP280 barometer and are "nan"
when it is missing - VFR_HUD/ALTITUDE are then skipped. lat..cog are "nan"
without a GPS fix - GPS_RAW_INT is still sent (fix type + satellite count
for preflight), GLOBAL_POSITION_INT is skipped. Lines starting with '#'
are firmware boot diagnostics.

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
import math
import pathlib
import sys
import threading
import time

import serial
from pymavlink import mavutil

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
    if len(parts) != 17:
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

                (
                    roll, pitch, yaw, rollspeed, pitchspeed, yawspeed, alt_m, climb_mps,
                    lat_deg, lon_deg, alt_msl_m, gs_mps, cog_deg, sats, hdop,
                    hacc_m, vacc_m,
                ) = values
                now = time.monotonic()
                time_boot_ms = int((now - start) * 1000) & 0xFFFFFFFF
                time_usec = int((now - start) * 1e6) & 0xFFFFFFFFFFFFFFFF

                if now - last_heartbeat >= HEARTBEAT_PERIOD_S:
                    self._hub.publish(heartbeat(self._hub.mav))
                    last_heartbeat = now

                self._hub.publish(
                    self._hub.mav.attitude_encode(
                        time_boot_ms, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed
                    )
                )

                if not math.isnan(alt_m):
                    # No airspeed/throttle sensors yet; heading is the
                    # (drifting) integrated yaw. VFR_HUD.alt nominally is
                    # AMSL but we only have the baro-relative altitude - the
                    # HUD prefers ALTITUDE.altitude_relative anyway.
                    self._hub.publish(
                        self._hub.mav.vfr_hud_encode(
                            0.0,
                            0.0 if math.isnan(gs_mps) else gs_mps,
                            int(math.degrees(yaw)) % 360,
                            0,
                            alt_m,
                            climb_mps,
                        )
                    )
                    self._hub.publish(
                        self._hub.mav.altitude_encode(
                            time_usec,
                            float("nan"),  # altitude_monotonic
                            float("nan"),  # altitude_amsl - unknown (no QNH)
                            alt_m,  # altitude_local (relative to boot location)
                            alt_m,  # altitude_relative
                            float("nan"),  # altitude_terrain
                            float("nan"),  # bottom_clearance
                        )
                    )

                self._publish_gps(
                    time_usec, time_boot_ms, lat_deg, lon_deg, alt_msl_m,
                    gs_mps, cog_deg, sats, hdop, hacc_m, vacc_m, alt_m, climb_mps,
                )

    def _publish_gps(
        self, time_usec, time_boot_ms, lat_deg, lon_deg, alt_msl_m,
        gs_mps, cog_deg, sats, hdop, hacc_m, vacc_m, alt_m, climb_mps,
    ):
        """GPS_RAW_INT always (sats visible while acquiring), GPI only with a fix.

        Scaling mirrors emulator_source.py: 1e7 deg, mm, cm/s, centidegrees;
        65535 / UINT16_MAX marks unknown eph/vel/cog per the MAVLink spec,
        0 marks unknown h_acc/v_acc (MAVLink 2 extension fields, in mm).
        """
        fix = not (math.isnan(lat_deg) or math.isnan(lon_deg))
        self._hub.publish(
            self._hub.mav.gps_raw_int_encode(
                time_usec,
                mavutil.mavlink.GPS_FIX_TYPE_3D_FIX if fix else mavutil.mavlink.GPS_FIX_TYPE_NO_FIX,
                int(lat_deg * 1e7) if fix else 0,
                int(lon_deg * 1e7) if fix else 0,
                0 if math.isnan(alt_msl_m) else int(alt_msl_m * 1000),
                65535 if math.isnan(hdop) else min(int(hdop * 100), 65535),
                65535,  # epv - the firmware does not report VDOP
                65535 if math.isnan(gs_mps) else int(gs_mps * 100),
                65535 if math.isnan(cog_deg) else int(cog_deg * 100) % 36000,
                int(sats),
                0,  # alt_ellipsoid - not reported
                0 if math.isnan(hacc_m) else int(hacc_m * 1000),
                0 if math.isnan(vacc_m) else int(vacc_m * 1000),
                0,  # vel_acc - not reported
                0,  # hdg_acc - not reported
            )
        )
        if not fix:
            return

        if math.isnan(cog_deg) or math.isnan(gs_mps):
            vx = vy = 0
        else:
            cog_rad = math.radians(cog_deg)
            vx = int(gs_mps * math.cos(cog_rad) * 100)
            vy = int(gs_mps * math.sin(cog_rad) * 100)
        # COG only counts as heading when actually moving; UINT16_MAX = unknown
        moving = not math.isnan(gs_mps) and gs_mps > 1.0 and not math.isnan(cog_deg)
        self._hub.publish(
            self._hub.mav.global_position_int_encode(
                time_boot_ms,
                int(lat_deg * 1e7),
                int(lon_deg * 1e7),
                0 if math.isnan(alt_msl_m) else int(alt_msl_m * 1000),
                0 if math.isnan(alt_m) else int(alt_m * 1000),  # baro relative alt
                vx,
                vy,
                0 if math.isnan(climb_mps) else int(-climb_mps * 100),
                int(cog_deg * 100) % 36000 if moving else 65535,
            )
        )
