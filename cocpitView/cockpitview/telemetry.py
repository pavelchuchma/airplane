"""MAVLink telemetry hub.

Telemetry sources (espnow_source.py, emulator_source.py) encode pymavlink
messages and publish() them here. The hub applies every message to a
thread-safe TelemetrySnapshot for the HUD and forwards it unchanged to
QGroundControl over UDP - both consumers see identical data.

Handled message types:
  - emulator_source.py: ATTITUDE + VFR_HUD + GPS_RAW_INT +
    GLOBAL_POSITION_INT + ALTITUDE
  - espnow_source.py: the same set (baro altitude/vario; position/velocity
    from the GPS - GLOBAL_POSITION_INT only while there is a fix), plus
    RADIO_STATUS/STATUSTEXT which pass through to QGC unapplied

ESP-NOW link stats (RSSI, loss percentages, the air-side failsafe flag)
have no natural MAVLink carrier for the HUD, so espnow_source.py sets them
directly on the snapshot via update_link()."""

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
    gpi_alt_amsl_m: float = 0.0
    lat_deg: float = 0.0
    lon_deg: float = 0.0
    gpi_ts: float = field(default=-math.inf)

    # GPS_RAW_INT (fix quality, even without a fix)
    gps_fix_type: int = 0
    gps_sats: int = 0
    gps_hacc_m: float | None = None  # 1-sigma horizontal error [m]
    gps_ts: float = field(default=-math.inf)

    # BATTERY_STATUS (per-cell voltages [mV], 65535 = cell slot unused)
    batt_cells_mv: tuple = ()
    batt_ts: float = field(default=-math.inf)

    # ALTITUDE message
    alt_relative_m: float = 0.0
    alt_ts: float = field(default=-math.inf)
    alt_amsl_m: float = 0.0
    alt_amsl_ts: float = field(default=-math.inf)

    # filled in by the main loop: height above terrain from a Terrain grid,
    # and the altitude for the HUD box (calibrated AMSL, or relative when no
    # terrain grid is loaded and calibration is impossible)
    agl_m: float | None = None
    display_alt_m: float | None = None

    # filled in by the main loop from home.HomePoint: slant range to the
    # launch point and the baro altitude difference against it
    home_dist_m: float | None = None
    home_vert_m: float | None = None

    # filled in by the main loop from yawcal.YawCalibrator: a CalState with
    # the wind and the trust in the heading datum. None without a calibrator
    yaw_cal: object = None

    # filled in by the main loop from replay_source.ReplayClock: position in
    # the log [s]. None for a live or emulated session, which has no such
    # thing - the HUD then draws no clock at all
    replay_time_s: float | None = None

    heartbeat_ts: float = field(default=-math.inf)

    # ESP-NOW link stats, set by espnow_source.py via TelemetryHub.update_link()
    link_rssi_dbm: int | None = None  # air-side RSSI of command frames
    link_cmd_loss_pct: int = 0  # uplink loss measured at the air side
    link_telem_loss_pct: int = 0  # downlink loss measured PC-side
    failsafe_active: bool = False  # air-side failsafe flag
    # boot gyro calibration waiting for stillness: |raw rates| in deg/s, None otherwise
    imu_cal_motion_dps: float | None = None
    link_ts: float = field(default=-math.inf)

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
    def ias_mps(self):
        """Indicated airspeed in m/s, or None when unknown (stale, or the air
        side reports no/dead pitot as NaN)."""
        if not self._fresh(self.vfr_ts) or math.isnan(self.airspeed_mps):
            return None
        return self.airspeed_mps

    @property
    def vario_mps(self):
        """Climb rate in m/s from VFR_HUD (positive = climbing), or None
        when stale or the baro is dead (NaN on the wire)."""
        if not self._fresh(self.vfr_ts) or math.isnan(self.climb_mps):
            return None
        return self.climb_mps

    @property
    def altitude_m(self):
        """Relative altitude in meters (ALTITUDE preferred), or None."""
        if self._fresh(self.alt_ts):
            return self.alt_relative_m
        if self._fresh(self.gpi_ts):
            return self.gpi_rel_alt_m
        if self._fresh(self.vfr_ts) and not math.isnan(self.vfr_alt_m):
            return self.vfr_alt_m
        return None

    @property
    def baro_relative_m(self):
        """Baro altitude relative to the air-side boot location, or None.
        Strictly from ALTITUDE (a dead baro sends NaN there, so the timestamp
        goes stale) - unlike altitude_m this has deliberately no GPS/VFR_HUD
        fallback: those carry a zero-filled relative_alt when the baro is
        dead, which would turn a baro-differential readout into a lie
        instead of "---"."""
        if self._fresh(self.alt_ts):
            return self.alt_relative_m
        return None

    @property
    def heading_deg(self):
        """Heading in degrees 0-360, or None. Yaw-derived sources (VFR_HUD
        heading is the integrated yaw too) only count while ATTITUDE is
        live - a dead IMU freezes yaw, and espnow_source stops publishing
        ATTITUDE to signal that. Fallback is the GPS course from
        GLOBAL_POSITION_INT (NaN unless actually moving)."""
        if self._fresh(self.attitude_ts):
            if self._fresh(self.vfr_ts):
                return self.vfr_heading_deg % 360.0
            return math.degrees(self.yaw) % 360.0
        if self._fresh(self.gpi_ts) and not math.isnan(self.gpi_heading_deg):
            return self.gpi_heading_deg % 360.0
        return None

    @property
    def heading_calibrated(self):
        """True when the heading shown is tied to true north rather than to
        the arbitrary datum the gyro integration started from at boot."""
        return self.yaw_cal is not None and self.yaw_cal.calibrated

    @property
    def position(self):
        """(lat_deg, lon_deg) from GLOBAL_POSITION_INT, or None."""
        if not self._fresh(self.gpi_ts) or (self.lat_deg == 0.0 and self.lon_deg == 0.0):
            return None
        return (self.lat_deg, self.lon_deg)

    @property
    def amsl_m(self):
        """Altitude above mean sea level in meters, or None. Comes only from
        ALTITUDE.altitude_amsl (calibrated baro, see altimeter.py) - GPS
        altitude is deliberately never used, its noise is tens of meters."""
        if self._fresh(self.alt_amsl_ts):
            return self.alt_amsl_m
        return None

    @property
    def gps_status(self):
        """(fix_type, satellites, hacc_m) from GPS_RAW_INT, or None when
        stale; hacc_m is None when the receiver reports no error estimate."""
        if not self._fresh(self.gps_ts):
            return None
        return (self.gps_fix_type, self.gps_sats, self.gps_hacc_m)

    @property
    def velocity_ned(self):
        """(vn, ve, vd) in m/s for the flight path marker, or None."""
        if not self._fresh(self.gpi_ts):
            return None
        return (self.vn, self.ve, self.vd)

    @property
    def gs_mps(self):
        """GPS ground speed in m/s, or None without a (fresh) fix."""
        if not self._fresh(self.gpi_ts):
            return None
        return math.hypot(self.vn, self.ve)

    @property
    def link_status(self):
        """(rssi_dbm, cmd_loss_pct, telem_loss_pct) or None when the source
        never reported link stats / they went stale; rssi_dbm is None until
        the air side has seen a command frame."""
        if not self._fresh(self.link_ts):
            return None
        return (self.link_rssi_dbm, self.link_cmd_loss_pct, self.link_telem_loss_pct)

    @property
    def battery_cells_v(self):
        """Per-cell voltages in volts from BATTERY_STATUS, or None when the
        source never sent one / the air side reports the sense dead (it stops
        publishing, so the timestamp goes stale within a second)."""
        if not self._fresh(self.batt_ts) or not self.batt_cells_mv:
            return None
        return tuple(mv / 1000.0 for mv in self.batt_cells_mv)

    @property
    def failsafe(self):
        """True while the air side reports its command-loss failsafe."""
        return self._fresh(self.link_ts) and self.failsafe_active

    @property
    def imu_calibrating_dps(self):
        """How much the airframe moves [deg/s] while the air side waits for
        stillness to calibrate the gyro, or None when it is not waiting."""
        if not self._fresh(self.link_ts):
            return None
        return self.imu_cal_motion_dps


def apply_message(s, msg, now):
    """Applies a MAVLink message to a TelemetrySnapshot (not thread-safe)."""
    mtype = msg.get_type()
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
    elif mtype == "GPS_RAW_INT":
        s.gps_fix_type = msg.fix_type
        s.gps_sats = msg.satellites_visible
        # MAVLink 2 extension, mm; 0 = unknown (also from MAVLink 1 senders)
        h_acc = getattr(msg, "h_acc", 0)
        s.gps_hacc_m = h_acc / 1000.0 if h_acc else None
        s.gps_ts = now
    elif mtype == "GLOBAL_POSITION_INT":
        s.vn = msg.vx / 100.0
        s.ve = msg.vy / 100.0
        s.vd = msg.vz / 100.0
        # UINT16_MAX = unknown (sent while not moving - COG is meaningless)
        s.gpi_heading_deg = float("nan") if msg.hdg == 65535 else msg.hdg / 100.0
        s.gpi_rel_alt_m = msg.relative_alt / 1000.0
        s.gpi_alt_amsl_m = msg.alt / 1000.0
        s.lat_deg = msg.lat / 1e7
        s.lon_deg = msg.lon / 1e7
        s.gpi_ts = now
    elif mtype == "ALTITUDE":
        if not math.isnan(msg.altitude_relative):
            s.alt_relative_m = msg.altitude_relative
            s.alt_ts = now
        if not math.isnan(msg.altitude_amsl):
            s.alt_amsl_m = msg.altitude_amsl
            s.alt_amsl_ts = now
    elif mtype == "BATTERY_STATUS":
        s.batt_cells_mv = tuple(mv for mv in msg.voltages if mv != 65535)
        s.batt_ts = now
    elif mtype == "HEARTBEAT":
        s.heartbeat_ts = now


def heartbeat(mav):
    """The fixed-wing HEARTBEAT both telemetry sources publish at 1 Hz."""
    return mav.heartbeat_encode(
        mavutil.mavlink.MAV_TYPE_FIXED_WING,
        mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE,
    )


class TelemetryHub:
    """Distributes messages from a telemetry source to the HUD snapshot and QGC."""

    def __init__(self, qgc_host="127.0.0.1", qgc_port=14550, system_id=1, processors=()):
        self._state = TelemetrySnapshot()
        # message processors (home.HomePoint, altimeter.BaroCalibrator) - each
        # observes the stream and may patch the message before it reaches the
        # snapshot and QGC; home first, it is what the calibrator locks on
        self._processors = tuple(processors)
        self._lock = threading.Lock()
        self._conn = mavutil.mavlink_connection(
            f"udpout:{qgc_host}:{qgc_port}",
            source_system=system_id,
            source_component=mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
        )

    @property
    def mav(self):
        """MAVLink encoder for sources: hub.publish(hub.mav.attitude_encode(...))."""
        return self._conn.mav

    def publish(self, msg, forward=True):
        """`forward=False` runs the message through the processors and the
        snapshot but keeps it out of the UDP stream - replay_source.py's
        fast-forward warm-up, where a few thousand frames would arrive at
        QGC in a burst of no use to anyone."""
        for processor in self._processors:
            msg = processor.process(msg)
        now = time.monotonic()
        with self._lock:
            apply_message(self._state, msg, now)
        if forward:
            self._conn.mav.send(msg)

    def update_link(self, rssi_dbm, cmd_loss_pct, telem_loss_pct, failsafe, imu_cal_motion_dps=None):
        """ESP-NOW link stats and air-side status flags for the HUD (no
        MAVLink counterpart - QGC gets RADIO_STATUS/STATUSTEXT separately
        from the source)."""
        now = time.monotonic()
        with self._lock:
            s = self._state
            s.link_rssi_dbm = rssi_dbm
            s.link_cmd_loss_pct = cmd_loss_pct
            s.link_telem_loss_pct = telem_loss_pct
            s.failsafe_active = failsafe
            s.imu_cal_motion_dps = imu_cal_motion_dps
            s.link_ts = now

    def snapshot(self):
        with self._lock:
            snap = TelemetrySnapshot(**vars(self._state))
        snap.now = time.monotonic()
        return snap
