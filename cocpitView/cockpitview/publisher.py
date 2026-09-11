"""Telemetry frame -> MAVLink, shared by the live link and the log replay.

One decoded protocol.Telemetry frame becomes the message set the hub
applies to the HUD snapshot and forwards to QGroundControl: ATTITUDE (only
while the air side reports a live IMU), VFR_HUD + ALTITUDE (always - each
float carries NaN while its sensor is dead), GPS_RAW_INT (always) and
GLOBAL_POSITION_INT (only with a fix), plus a 1 Hz HEARTBEAT, a 2 Hz
RADIO_STATUS and a 2 Hz BATTERY_STATUS, and a STATUSTEXT on every failsafe
transition. Link stats have no natural MAVLink carrier for the HUD and go
straight into the snapshot via hub.update_link().

This lives apart from espnow_source.py so that replay_source.py publishes
through exactly the same code (and therefore through today's calibrators),
rather than through a second, slowly diverging copy.

The caller owns both clocks: `elapsed` is the vehicle's time base for the
wire (time_boot_ms/time_usec) and `now` paces the periodic messages. The
live source passes monotonic time for both; the replay passes log time, so
an old flight goes out with the timestamps it flew with.
"""

import math

from pymavlink import mavutil

from . import protocol
from .telemetry import heartbeat

HEARTBEAT_PERIOD_S = 1.0
RADIO_STATUS_PERIOD_S = 0.5
BATTERY_STATUS_PERIOD_S = 0.5


class TelemetryPublisher:
    """Stateful (failsafe edge, periodic message schedule, cumulative loss);
    one instance per session. `warn=False` silences the protocol-mismatch
    console line - a replay of an old log is not a reason to reflash."""

    def __init__(self, hub, warn=True):
        self._hub = hub
        self._warn = warn
        self._mismatch_warned = False
        self._failsafe = None  # last seen air-side failsafe state (None = unknown)
        self._last_heartbeat = -math.inf
        self._last_radio_status = -math.inf
        self._last_battery_status = -math.inf
        self._forward = True
        self.telem_lost_total = 0  # cumulative, for RADIO_STATUS.rxerrors

    def publish(self, t, elapsed, now, telem_loss_pct=0, forward=True):
        """One telemetry frame. `forward=False` keeps the messages out of the
        UDP stream to QGC (the replay's fast-forward warm-up) while the HUD
        snapshot and the processor chain still see them."""
        self._forward = forward

        if now - self._last_heartbeat >= HEARTBEAT_PERIOD_S:
            self._emit(heartbeat(self._hub.mav))
            self._last_heartbeat = now

        self._publish_telemetry(t, elapsed, telem_loss_pct)

        if now - self._last_radio_status >= RADIO_STATUS_PERIOD_S:
            self._publish_radio_status(t)
            self._last_radio_status = now

        if now - self._last_battery_status >= BATTERY_STATUS_PERIOD_S:
            self._publish_battery(t)
            self._last_battery_status = now

        self._forward = True

    def _emit(self, msg):
        self._hub.publish(msg, forward=self._forward)

    def _publish_telemetry(self, t, elapsed, telem_loss_pct):
        time_boot_ms = int(elapsed * 1000) & 0xFFFFFFFF
        time_usec = int(elapsed * 1e6) & 0xFFFFFFFFFFFFFFFF
        fix = bool(t.flags & protocol.TELEM_FLAG_GPS_FIX)
        failsafe = bool(t.flags & protocol.TELEM_FLAG_FAILSAFE)

        if t.flags & protocol.TELEM_FLAG_VERSION_MISMATCH and self._warn and not self._mismatch_warned:
            self._mismatch_warned = True
            print(
                f"air firmware rejects our commands: it speaks protocol"
                f" v{t.version}, our commands are v{t.last_cmd_version or protocol.PROTOCOL_VERSION}"
                " - reflash / git pull"
            )

        self._hub.update_link(
            rssi_dbm=t.rssi_dbm if t.rssi_dbm != 0 else None,
            cmd_loss_pct=t.cmd_loss_pct,
            telem_loss_pct=telem_loss_pct,
            failsafe=failsafe,
        )
        if failsafe != self._failsafe:
            self._failsafe = failsafe
            self._emit(
                self._hub.mav.statustext_encode(
                    mavutil.mavlink.MAV_SEVERITY_CRITICAL if failsafe else mavutil.mavlink.MAV_SEVERITY_INFO,
                    b"FAILSAFE - command link lost" if failsafe else b"Failsafe cleared",
                )
            )

        # No ATTITUDE while the air side reports a dead IMU (the wire fields
        # are frozen at the last good sample): attitude_ts goes stale within
        # a second and the HUD hides the horizon, roll indicator and the
        # yaw-derived compass instead of showing stuck values.
        if t.flags & protocol.TELEM_FLAG_IMU_OK:
            self._emit(
                self._hub.mav.attitude_encode(
                    time_boot_ms, t.roll, t.pitch, t.yaw, t.rollspeed, t.pitchspeed, t.yawspeed
                )
            )

        # VFR_HUD + ALTITUDE go out unconditionally: each float carries NaN
        # while its sensor is dead (pitot -> airspeed, baro -> alt/climb)
        # and the HUD hides just the affected readout - a dead baro must
        # not take a working airspeed down with it. No throttle sensor yet;
        # heading is the (drifting) integrated yaw. VFR_HUD.alt nominally
        # is AMSL but we only have the baro-relative altitude - the HUD
        # prefers ALTITUDE.altitude_relative anyway.
        self._emit(
            self._hub.mav.vfr_hud_encode(
                t.ias_mps,
                0.0 if math.isnan(t.gs_mps) else t.gs_mps,
                int(math.degrees(t.yaw)) % 360,
                0,
                t.baro_alt_m,
                t.climb_mps,
            )
        )
        self._emit(
            self._hub.mav.altitude_encode(
                time_usec,
                float("nan"),  # altitude_monotonic
                float("nan"),  # altitude_amsl - unknown (no QNH)
                t.baro_alt_m,  # altitude_local (relative to boot location)
                t.baro_alt_m,  # altitude_relative
                float("nan"),  # altitude_terrain
                float("nan"),  # bottom_clearance
            )
        )

        self._publish_gps(t, fix, time_usec, time_boot_ms)

    def _publish_gps(self, t, fix, time_usec, time_boot_ms):
        """GPS_RAW_INT always (sats visible while acquiring), GPI only with a fix.

        Scaling mirrors emulator_source.py: 1e7 deg, mm, cm/s, centidegrees;
        65535 / UINT16_MAX marks unknown eph/vel/cog per the MAVLink spec,
        0 marks unknown h_acc/v_acc (MAVLink 2 extension fields, in mm).
        """
        self._emit(
            self._hub.mav.gps_raw_int_encode(
                time_usec,
                mavutil.mavlink.GPS_FIX_TYPE_3D_FIX if fix else mavutil.mavlink.GPS_FIX_TYPE_NO_FIX,
                t.lat1e7 if fix else 0,
                t.lon1e7 if fix else 0,
                0 if math.isnan(t.alt_msl_m) else int(t.alt_msl_m * 1000),
                65535 if math.isnan(t.hdop) else min(int(t.hdop * 100), 65535),
                65535,  # epv - the firmware does not report VDOP
                65535 if math.isnan(t.gs_mps) else int(t.gs_mps * 100),
                65535 if math.isnan(t.cog_deg) else int(t.cog_deg * 100) % 36000,
                t.sats,
                0,  # alt_ellipsoid - not reported
                0 if math.isnan(t.hacc_m) else int(t.hacc_m * 1000),
                0 if math.isnan(t.vacc_m) else int(t.vacc_m * 1000),
                0,  # vel_acc - not reported
                0,  # hdg_acc - not reported
            )
        )
        if not fix:
            return

        if math.isnan(t.cog_deg) or math.isnan(t.gs_mps):
            vx = vy = 0
        else:
            cog_rad = math.radians(t.cog_deg)
            vx = int(t.gs_mps * math.cos(cog_rad) * 100)
            vy = int(t.gs_mps * math.sin(cog_rad) * 100)
        # COG only counts as heading when actually moving; UINT16_MAX = unknown
        moving = not math.isnan(t.gs_mps) and t.gs_mps > 1.0 and not math.isnan(t.cog_deg)
        self._emit(
            self._hub.mav.global_position_int_encode(
                time_boot_ms,
                t.lat1e7,
                t.lon1e7,
                0 if math.isnan(t.alt_msl_m) else int(t.alt_msl_m * 1000),
                0 if math.isnan(t.baro_alt_m) else int(t.baro_alt_m * 1000),
                vx,
                vy,
                0 if math.isnan(t.climb_mps) else int(-t.climb_mps * 100),
                int(t.cog_deg * 100) % 36000 if moving else 65535,
            )
        )

    def _publish_battery(self, t):
        """BATTERY_STATUS with the three per-cell voltages (mV; QGC renders
        them individually). Not published while the air side reports the
        battery sense dead/absent - the HUD's batt_ts goes stale and the BAT
        line disappears, mirroring how a dead IMU stops ATTITUDE."""
        if not t.flags & protocol.TELEM_FLAG_BATT_OK:
            return
        voltages = [t.cell1_mv, t.cell2_mv, t.cell3_mv] + [65535] * 7  # 65535 = cell unused
        self._emit(
            self._hub.mav.battery_status_encode(
                0,  # id
                mavutil.mavlink.MAV_BATTERY_FUNCTION_ALL,
                mavutil.mavlink.MAV_BATTERY_TYPE_LION,
                32767,  # temperature [cdegC], INT16_MAX = not measured
                voltages,
                -1,  # current_battery - not measured
                -1,  # current_consumed
                -1,  # energy_consumed
                -1,  # battery_remaining [%] - voltage is a poor fuel gauge, don't fake one
            )
        )

    def _publish_radio_status(self, t):
        """RADIO_STATUS so QGC shows link quality natively (SiK convention:
        rssi 0-254 mapped from dBm, 255 = unknown)."""
        if t.rssi_dbm != 0:
            rssi = max(0, min(254, int((t.rssi_dbm + 127) * 1.9)))
        else:
            rssi = 255
        self._emit(
            self._hub.mav.radio_status_encode(
                rssi,
                rssi,  # remrssi - the air side is the only RSSI we have
                100,  # txbuf [%]
                0,  # noise - not measured
                0,  # remnoise
                min(65535, self.telem_lost_total),  # rxerrors
                0,  # fixed
            )
        )
