"""One-shot barometer-to-AMSL calibration against the terrain grid.

The air firmware only knows the baro altitude relative to its boot location
(there is no QNH source), while the terrain grid (terrain.py, CUZK DMR 5G)
knows the absolute ground elevation to ~0.2 m. Assuming the plane sits on
the ground when the first GPS fix arrives, the offset between the two turns
the baro into an AMSL altimeter - the same idea as a pilot setting the
altimeter to the known field elevation.

GPS *altitude* is deliberately never used: its vertical error (tens of
meters) dwarfs both the baro and the grid. GPS only provides the position
for the grid lookup.

BaroCalibrator sits in TelemetryHub.publish(), so the HUD snapshot and the
MAVLink stream forwarded to QGC both see the calibrated altitude.
"""

import math
import statistics
import time

INPUT_FRESH_S = 1.0  # max age of the baro sample when a position arrives
MAX_HACC_M = 5.0  # reject positions with a worse reported 1-sigma error
MAX_GROUNDSPEED_MPS = 1.0  # "still on the ground"
SAMPLES_TO_LOCK = 25  # ~5 s at the 5 Hz GPS rate


class BaroCalibrator:
    """Fills ALTITUDE.altitude_amsl (and GLOBAL_POSITION_INT.alt) from the
    baro-relative altitude once the ground offset has been established."""

    def __init__(self, terrain, log=None):
        self._terrain = terrain
        self._log = log  # flightlog.FlightLog or None; the lock is PC-side state the log can't recompute
        self._offset_m = None  # AMSL = baro_rel + offset, None until locked
        self._samples = []
        self._baro_rel_m = None
        self._baro_ts = -math.inf
        self._fix_type = 0
        self._hacc_m = None

    def process(self, msg):
        """Applies/collects calibration; returns the (possibly patched) msg."""
        mtype = msg.get_type()
        if mtype == "ALTITUDE":
            if not math.isnan(msg.altitude_relative):
                self._baro_rel_m = msg.altitude_relative
                self._baro_ts = time.monotonic()
                if self._offset_m is not None and math.isnan(msg.altitude_amsl):
                    msg.altitude_amsl = msg.altitude_relative + self._offset_m
        elif mtype == "GPS_RAW_INT":
            self._fix_type = msg.fix_type
            h_acc = getattr(msg, "h_acc", 0)  # mm, 0 = unknown
            self._hacc_m = h_acc / 1000.0 if h_acc else None
        elif mtype == "GLOBAL_POSITION_INT":
            if self._offset_m is not None:
                # fused altitude for QGC; raw GPS alt stays in GPS_RAW_INT
                msg.alt = msg.relative_alt + int(round(self._offset_m * 1000))
            else:
                self._collect(msg)
        return msg

    def _collect(self, msg):
        groundspeed = math.hypot(msg.vx, msg.vy) / 100.0
        if (
            self._fix_type < 3
            or self._hacc_m is None  # no error estimate -> don't trust the fix
            or self._hacc_m > MAX_HACC_M
            or groundspeed > MAX_GROUNDSPEED_MPS
            or time.monotonic() - self._baro_ts > INPUT_FRESH_S
        ):
            return
        ground_m = self._terrain.elevation_m(msg.lat / 1e7, msg.lon / 1e7)
        if ground_m is None:
            return
        self._samples.append(ground_m - self._baro_rel_m)
        if len(self._samples) >= SAMPLES_TO_LOCK:
            self._offset_m = statistics.median(self._samples)
            print(f"Baro calibrated: field elevation {ground_m:.1f} m AMSL")
            if self._log is not None:
                self._log.event("baro_calibrated", offset_m=self._offset_m, field_elevation_m=ground_m)
