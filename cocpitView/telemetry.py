"""MAVLink telemetry receiver.

Listens on a UDP port for MAVLink messages forwarded by QGroundControl
(Application Settings -> MAVLink -> MAVLink Forwarding, host `localhost:14445`)
and keeps the latest vehicle state in a thread-safe snapshot.

Handles both telemetry sources of this project:
  - mpu6050_emulator.py: ATTITUDE + VFR_HUD + GLOBAL_POSITION_INT + ALTITUDE
  - mpu6050_hw_bridge.py: ATTITUDE only (speed/altitude/velocity stay invalid)
"""

import math
import threading
import time
from dataclasses import dataclass, field

from pymavlink import mavutil

STALE_AFTER_S = 1.0


@dataclass
class TelemetrySnapshot:
    # ATTITUDE [rad]
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    attitude_ts: float = field(default=-math.inf)

    # VFR_HUD
    airspeed_mps: float = 0.0
    groundspeed_mps: float = 0.0
    vfr_heading_deg: float = 0.0
    vfr_alt_m: float = 0.0
    climb_mps: float = 0.0
    vfr_ts: float = field(default=-math.inf)

    # GLOBAL_POSITION_INT (velocities in m/s, NED)
    vn: float = 0.0
    ve: float = 0.0
    vd: float = 0.0
    gpi_heading_deg: float = 0.0
    gpi_rel_alt_m: float = 0.0
    gpi_ts: float = field(default=-math.inf)

    # ALTITUDE message
    alt_relative_m: float = 0.0
    alt_ts: float = field(default=-math.inf)

    heartbeat_ts: float = field(default=-math.inf)

    now: float = 0.0

    def _fresh(self, ts):
        return self.now - ts <= STALE_AFTER_S

    @property
    def has_attitude(self):
        return self._fresh(self.attitude_ts)

    @property
    def connected(self):
        return self._fresh(self.heartbeat_ts) or self.has_attitude

    @property
    def airspeed_kmh(self):
        """Airspeed in km/h, or None when unknown."""
        if not self._fresh(self.vfr_ts):
            return None
        return self.airspeed_mps * 3.6

    @property
    def altitude_m(self):
        """Relative altitude in meters (ALTITUDE preferred), or None."""
        if self._fresh(self.alt_ts):
            return self.alt_relative_m
        if self._fresh(self.gpi_ts):
            return self.gpi_rel_alt_m
        if self._fresh(self.vfr_ts):
            return self.vfr_alt_m
        return None

    @property
    def heading_deg(self):
        """Magnetic heading in degrees 0-360, falls back to yaw, or None."""
        if self._fresh(self.vfr_ts):
            return self.vfr_heading_deg % 360.0
        if self._fresh(self.gpi_ts):
            return self.gpi_heading_deg % 360.0
        if self._fresh(self.attitude_ts):
            return math.degrees(self.yaw) % 360.0
        return None

    @property
    def velocity_ned(self):
        """(vn, ve, vd) in m/s for the flight path marker, or None."""
        if not self._fresh(self.gpi_ts):
            return None
        return (self.vn, self.ve, self.vd)


class TelemetryReceiver:
    """Background thread reading MAVLink from a UDP port."""

    def __init__(self, port=14445, host="0.0.0.0"):
        self._state = TelemetrySnapshot()
        self._lock = threading.Lock()
        self._conn = mavutil.mavlink_connection(f"udpin:{host}:{port}")
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def snapshot(self):
        with self._lock:
            snap = TelemetrySnapshot(**vars(self._state))
        snap.now = time.monotonic()
        return snap

    def _run(self):
        while True:
            msg = self._conn.recv_match(blocking=True, timeout=1.0)
            if msg is None:
                continue
            now = time.monotonic()
            mtype = msg.get_type()
            with self._lock:
                s = self._state
                if mtype == "ATTITUDE":
                    s.roll = msg.roll
                    s.pitch = msg.pitch
                    s.yaw = msg.yaw
                    s.attitude_ts = now
                elif mtype == "VFR_HUD":
                    s.airspeed_mps = msg.airspeed
                    s.groundspeed_mps = msg.groundspeed
                    s.vfr_heading_deg = float(msg.heading)
                    s.vfr_alt_m = msg.alt
                    s.climb_mps = msg.climb
                    s.vfr_ts = now
                elif mtype == "GLOBAL_POSITION_INT":
                    s.vn = msg.vx / 100.0
                    s.ve = msg.vy / 100.0
                    s.vd = msg.vz / 100.0
                    s.gpi_heading_deg = msg.hdg / 100.0
                    s.gpi_rel_alt_m = msg.relative_alt / 1000.0
                    s.gpi_ts = now
                elif mtype == "ALTITUDE":
                    if not math.isnan(msg.altitude_relative):
                        s.alt_relative_m = msg.altitude_relative
                        s.alt_ts = now
                elif mtype == "HEARTBEAT":
                    s.heartbeat_ts = now
