"""Synthetic telemetry source - flight profile playback (--emulator).

Emulates an ESP32 with an MPU-6050 which, after sensor fusion, reports
attitude (roll/pitch/yaw) and GPS/telemetry (position, altitude, speed)
from the scripted flight profile in flight_profile.py. Publishes HEARTBEAT,
ATTITUDE, GPS_RAW_INT, GLOBAL_POSITION_INT, VFR_HUD and ALTITUDE to the
TelemetryHub.
"""

import math
import threading
import time

from pymavlink import mavutil

from . import flight_profile
from . import quaternion as quat
from .telemetry import heartbeat

HEARTBEAT_PERIOD_S = 1.0
RATE_SAMPLE_DT = 0.005  # [s] for numerically differentiating angular rate


def attitude_at(t):
    """Returns (roll, pitch, yaw, rollspeed, pitchspeed, yawspeed) in radians / rad/s."""
    roll_deg, pitch_deg, yaw_deg = flight_profile.nominal_angles_deg(t)
    q1 = quat.from_euler_zyx(math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg))

    roll2_deg, pitch2_deg, yaw2_deg = flight_profile.nominal_angles_deg(t + RATE_SAMPLE_DT)
    q2 = quat.from_euler_zyx(math.radians(roll2_deg), math.radians(pitch2_deg), math.radians(yaw2_deg))

    roll, pitch, yaw = quat.to_euler_zyx(q1)
    rollspeed, pitchspeed, yawspeed = quat.body_angular_rate(q1, q2, RATE_SAMPLE_DT)
    return roll, pitch, yaw, rollspeed, pitchspeed, yawspeed


class EmulatorSource:
    """Background thread replaying the flight profile into the hub."""

    def __init__(self, hub, rate=20.0):
        self._hub = hub
        self._rate = rate
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        print(f"Flight profile repeats every {flight_profile.TOTAL_DURATION:.0f} s ({self._rate:g} Hz).")
        self._thread.start()

    def _publish_gps_and_hud(self, t, time_boot_ms):
        mav = self._hub.mav
        gps = flight_profile.gps_state(t)
        lat_e7 = int(gps["lat_deg"] * 1e7)
        lon_e7 = int(gps["lon_deg"] * 1e7)
        alt_mm = int(gps["alt_m"] * 1000)
        rel_alt_mm = int(gps["relative_alt_m"] * 1000)
        vx_cms = int(gps["vn"] * 100)
        vy_cms = int(gps["ve"] * 100)
        vz_cms = int(gps["vd"] * 100)
        hdg_cdeg = int(gps["heading_deg"] * 100) % 36000
        groundspeed = math.hypot(gps["vn"], gps["ve"])
        time_usec = int(t * 1e6) & 0xFFFFFFFFFFFFFFFF

        self._hub.publish(
            mav.gps_raw_int_encode(
                time_usec,
                mavutil.mavlink.GPS_FIX_TYPE_3D_FIX,
                lat_e7,
                lon_e7,
                alt_mm,
                100,  # eph (HDOP*100)
                100,  # epv (VDOP*100)
                int(groundspeed * 100),  # vel [cm/s]
                hdg_cdeg,  # cog - matches heading when there's no wind
                12,  # satellites_visible
            )
        )

        self._hub.publish(
            mav.global_position_int_encode(
                time_boot_ms,
                lat_e7,
                lon_e7,
                alt_mm,
                rel_alt_mm,
                vx_cms,
                vy_cms,
                vz_cms,
                hdg_cdeg,
            )
        )

        self._hub.publish(
            mav.vfr_hud_encode(
                flight_profile.AIRSPEED_MPS,
                groundspeed,
                int(gps["heading_deg"]) % 360,
                60,  # throttle [%] - fake constant
                gps["alt_m"],
                -gps["vd"],  # climb rate [m/s], positive = climbing
            )
        )

        self._hub.publish(
            mav.altitude_encode(
                time_usec,
                gps["alt_m"],  # altitude_monotonic
                gps["alt_m"],  # altitude_amsl
                gps["relative_alt_m"],  # altitude_local (relative to HOME origin)
                gps["relative_alt_m"],  # altitude_relative
                float("nan"),  # altitude_terrain - unknown (no terrain is modeled)
                float("nan"),  # bottom_clearance - unknown
            )
        )

    def _run(self):
        tick_period = 1.0 / self._rate
        start = time.monotonic()
        last_heartbeat = 0.0
        last_print_phase = None

        while True:
            now = time.monotonic()
            t = now - start

            if now - last_heartbeat >= HEARTBEAT_PERIOD_S:
                self._hub.publish(heartbeat(self._hub.mav))
                last_heartbeat = now

            roll, pitch, yaw, rollspeed, pitchspeed, yawspeed = attitude_at(t)
            time_boot_ms = int(t * 1000) & 0xFFFFFFFF
            self._hub.publish(
                self._hub.mav.attitude_encode(time_boot_ms, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed)
            )
            self._publish_gps_and_hud(t, time_boot_ms)

            phase = flight_profile.phase_at(t)
            if phase["name"] != last_print_phase:
                gps = flight_profile.gps_state(t)
                print(
                    f"[t={t:6.1f}s] phase: {phase['name']:<12} "
                    f"roll={math.degrees(roll):6.1f}  pitch={math.degrees(pitch):6.1f}  yaw={math.degrees(yaw):6.1f}  "
                    f"lat={gps['lat_deg']:.6f}  lon={gps['lon_deg']:.6f}  alt={gps['alt_m']:6.1f}m"
                )
                last_print_phase = phase["name"]

            time.sleep(max(0.0, tick_period - (time.monotonic() - now)))
