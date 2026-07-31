#!/usr/bin/env python3
"""
Emulates an ESP32 with an MPU-6050 which, after sensor fusion, sends attitude
(roll/pitch/yaw) and GPS/telemetry (position, altitude, speed) to
QGroundControl over the MAVLink protocol via UDP.

Usage:
    ./.venv/bin/python mpu6050_emulator.py
    ./.venv/bin/python mpu6050_emulator.py --host 127.0.0.1 --port 14550 --rate 20

By default QGC automatically listens on UDP port 14550, and as soon as a
valid HEARTBEAT arrives the vehicle appears on its own - no need to manually
add a Comm Link.
"""

import argparse
import math
import time

from pymavlink import mavutil

import flight_profile
import quaternion as quat

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


def send_gps_and_hud(conn, t, time_boot_ms):
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

    conn.mav.gps_raw_int_send(
        int(t * 1e6) & 0xFFFFFFFFFFFFFFFF,  # time_usec
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

    conn.mav.global_position_int_send(
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

    conn.mav.vfr_hud_send(
        flight_profile.AIRSPEED_MPS,
        groundspeed,
        int(gps["heading_deg"]) % 360,
        60,  # throttle [%] - fake constant
        gps["alt_m"],
        -gps["vd"],  # climb rate [m/s], positive = climbing
    )

    time_usec = int(t * 1e6) & 0xFFFFFFFFFFFFFFFF
    conn.mav.altitude_send(
        time_usec,
        gps["alt_m"],  # altitude_monotonic
        gps["alt_m"],  # altitude_amsl
        gps["relative_alt_m"],  # altitude_local (relative to HOME origin)
        gps["relative_alt_m"],  # altitude_relative
        float("nan"),  # altitude_terrain - unknown (no terrain is modeled)
        float("nan"),  # bottom_clearance - unknown
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="127.0.0.1", help="QGroundControl IP address (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=14550, help="QGC UDP port (default: 14550)")
    parser.add_argument("--rate", type=float, default=20.0, help="ATTITUDE message rate in Hz (default: 20)")
    parser.add_argument("--system-id", type=int, default=1, help="MAVLink system id of the emulated vehicle")
    args = parser.parse_args()

    conn = mavutil.mavlink_connection(
        f"udpout:{args.host}:{args.port}",
        source_system=args.system_id,
        source_component=mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
    )

    print(f"Sending MAVLink ATTITUDE/HEARTBEAT to {args.host}:{args.port} ({args.rate} Hz)")
    print(f"Flight profile repeats every {flight_profile.TOTAL_DURATION:.0f} s. Ctrl+C to stop.\n")

    tick_period = 1.0 / args.rate
    start = time.monotonic()
    last_heartbeat = 0.0
    last_print_phase = None

    try:
        while True:
            now = time.monotonic()
            t = now - start

            if now - last_heartbeat >= HEARTBEAT_PERIOD_S:
                conn.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_FIXED_WING,
                    mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    0,
                    mavutil.mavlink.MAV_STATE_ACTIVE,
                )
                last_heartbeat = now

            roll, pitch, yaw, rollspeed, pitchspeed, yawspeed = attitude_at(t)
            time_boot_ms = int(t * 1000) & 0xFFFFFFFF
            conn.mav.attitude_send(time_boot_ms, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed)
            send_gps_and_hud(conn, t, time_boot_ms)

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
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
