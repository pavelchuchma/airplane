"""One-shot barometer-to-AMSL calibration against the terrain grid.

The air firmware only knows the baro altitude relative to its boot location
(there is no QNH source), while the terrain grid (terrain.py, CUZK DMR 5G)
knows the absolute ground elevation to ~0.2 m. The offset between the two,
taken at the launch point while the aircraft is still on the ground, turns
the baro into an AMSL altimeter - the same idea as a pilot setting the
altimeter to the known field elevation.

The "trustworthy fix, on the ground" gate and the sample collection live in
home.py (the launch point is the same event); this class is a sink on those
samples and only adds the terrain lookup, which may fail outside the fetched
grid. Fewer than half the samples with a known elevation and the session
stays uncalibrated - nothing that happens later in the air could establish
the offset anyway.

GPS *altitude* is deliberately never used: its vertical error (tens of
meters) dwarfs both the baro and the grid. GPS only provides the position
for the grid lookup.

BaroCalibrator sits in TelemetryHub's processor chain, so the HUD snapshot
and the MAVLink stream forwarded to QGC both see the calibrated altitude.
"""

import math
import statistics

from .home import SAMPLES_TO_LOCK


class BaroCalibrator:
    """Fills ALTITUDE.altitude_amsl (and GLOBAL_POSITION_INT.alt) from the
    baro-relative altitude once the ground offset has been established."""

    def __init__(self, terrain, home, log=None):
        self._terrain = terrain
        self._log = log  # flightlog.FlightLog or None; the lock is PC-side state the log can't recompute
        self._offset_m = None  # AMSL = baro_rel + offset, None until locked
        self._samples = []  # (ground_m, baro_rel_m) of the accepted home samples
        home.add_sink(self)

    def process(self, msg):
        """Patches the altitude fields once calibrated; returns the msg."""
        if self._offset_m is None:
            return msg
        mtype = msg.get_type()
        if mtype == "ALTITUDE":
            if not math.isnan(msg.altitude_relative) and math.isnan(msg.altitude_amsl):
                msg.altitude_amsl = msg.altitude_relative + self._offset_m
        elif mtype == "GLOBAL_POSITION_INT":
            # fused altitude for QGC; raw GPS alt stays in GPS_RAW_INT
            msg.alt = msg.relative_alt + int(round(self._offset_m * 1000))
        return msg

    def on_home_sample(self, lat_deg, lon_deg, baro_rel_m):
        ground_m = self._terrain.elevation_m(lat_deg, lon_deg)
        if ground_m is not None:
            self._samples.append((ground_m, baro_rel_m))

    def on_home_lock(self):
        if len(self._samples) < SAMPLES_TO_LOCK // 2:
            print(f"Baro not calibrated - terrain elevation unknown for the launch point ({len(self._samples)} samples)")
            return
        # median of the per-sample differences, not a difference of medians:
        # one rough grid cell then cannot pull the offset
        self._offset_m = statistics.median(ground_m - baro_rel_m for ground_m, baro_rel_m in self._samples)
        field_elevation_m = statistics.median(ground_m for ground_m, _ in self._samples)
        self._samples = []
        print(f"Baro calibrated: field elevation {field_elevation_m:.1f} m AMSL")
        if self._log is not None:
            self._log.event("baro_calibrated", offset_m=self._offset_m, field_elevation_m=field_elevation_m)
