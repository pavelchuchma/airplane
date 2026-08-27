"""Launch point ("home") locked from the first trustworthy ground fix.

The HUD shows the slant range back to the place the aircraft took off from,
which needs a single reference point per session. It is locked once, from
the first GPS fixes that are trustworthy *and* taken while the aircraft is
still standing on the ground - starting cocpitView mid-flight therefore
leaves the readout off for the whole session, exactly like the altimeter
calibration it shares its conditions with.

This module owns that gate (fix quality / not moving / a fresh baro sample /
enough samples to median away the noise) in one copy, because
altimeter.BaroCalibrator has to lock under identical conditions: it
calibrates the baro against the terrain elevation *of the launch point*.
It registers itself as a sink here and consumes the very same samples, so
the two can never drift apart. The one thing it adds on top is a terrain
lookup per sample, which can fail (outside the fetched grid, nodata pixel) -
so the altimeter may stay uncalibrated while home locks, never the reverse.
Distance to home needs no terrain grid at all.

The vertical leg is a pure baro difference (the relative altitude at the
lock subtracted from the current one), not a difference of AMSL altitudes:
that way it needs neither the terrain grid nor the AMSL calibration, and no
GPS vertical error (tens of meters) enters the readout.
"""

import math
import statistics
import time

INPUT_FRESH_S = 1.0  # max age of the baro sample when a position arrives
MAX_HACC_M = 5.0  # reject positions with a worse reported 1-sigma error
MAX_GROUNDSPEED_MPS = 1.0  # "still on the ground"
SAMPLES_TO_LOCK = 25  # ~5 s at the 5 Hz GPS rate

EARTH_RADIUS_M = 6_371_000.0


class HomePoint:
    """Locks (lat, lon, baro_rel) of the launch point; sits in the
    TelemetryHub processor chain and passes every message through
    untouched - unlike BaroCalibrator it only observes."""

    def __init__(self, log=None):
        self._log = log  # flightlog.FlightLog or None; the lock is PC-side state the log can't recompute
        self._fix = None  # (lat_deg, lon_deg, baro_rel_m), None until locked
        self._samples = []
        self._sinks = []
        self._baro_rel_m = None
        self._baro_ts = -math.inf
        self._fix_type = 0
        self._hacc_m = None

    def add_sink(self, sink):
        """Registers a consumer of the accepted samples: it gets
        on_home_sample(lat_deg, lon_deg, baro_rel_m) per sample and
        on_home_lock() once the median is in."""
        self._sinks.append(sink)

    @property
    def fix(self):
        """(lat_deg, lon_deg, baro_rel_m) of the launch point, or None."""
        return self._fix

    def process(self, msg):
        mtype = msg.get_type()
        if mtype == "ALTITUDE":
            if not math.isnan(msg.altitude_relative):
                self._baro_rel_m = msg.altitude_relative
                self._baro_ts = time.monotonic()
        elif mtype == "GPS_RAW_INT":
            self._fix_type = msg.fix_type
            h_acc = getattr(msg, "h_acc", 0)  # mm, 0 = unknown
            self._hacc_m = h_acc / 1000.0 if h_acc else None
        elif mtype == "GLOBAL_POSITION_INT" and self._fix is None:
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
        lat_deg, lon_deg = msg.lat / 1e7, msg.lon / 1e7
        self._samples.append((lat_deg, lon_deg, self._baro_rel_m))
        for sink in self._sinks:
            sink.on_home_sample(lat_deg, lon_deg, self._baro_rel_m)
        if len(self._samples) < SAMPLES_TO_LOCK:
            return
        # written here (telemetry source thread), read by the main loop: one
        # atomic assignment of the whole triple, so no half-initialized state
        # can be observed and no lock is needed
        self._fix = tuple(statistics.median(v) for v in zip(*self._samples))
        self._samples = []
        lat0, lon0, baro_rel0 = self._fix
        print(f"Home locked: {lat0:.6f}, {lon0:.6f}")
        if self._log is not None:
            self._log.event("home_locked", lat_deg=lat0, lon_deg=lon0, baro_rel_m=baro_rel0)
        for sink in self._sinks:
            sink.on_home_lock()

    def offsets_from(self, snap):
        """(distance_m, vertical_m) of a snapshot against the launch point,
        each None when its input is missing - the two come from independent
        sensors (GPS / baro) and drop out independently. Distance is the
        slant range (the hypotenuse), so it answers "how far away is the
        aircraft"; with a dead baro the vertical leg is unknown and it
        degrades to the horizontal distance."""
        if self._fix is None:
            return (None, None)
        lat0, lon0, baro_rel0 = self._fix
        # the same quantity the lock captured, and None (not a zero-filled
        # GPS relative_alt) when the baro is dead - see baro_relative_m
        baro_rel_m = snap.baro_relative_m
        vertical_m = None if baro_rel_m is None else baro_rel_m - baro_rel0

        position = snap.position
        if position is None:
            return (None, vertical_m)
        lat_deg, lon_deg = position
        # equirectangular projection - below a meter of error at the few
        # kilometers this readout is used for
        north_m = math.radians(lat_deg - lat0) * EARTH_RADIUS_M
        east_m = math.radians(lon_deg - lon0) * math.cos(math.radians((lat_deg + lat0) / 2)) * EARTH_RADIUS_M
        horizontal_m = math.hypot(north_m, east_m)
        if vertical_m is None:
            return (horizontal_m, None)
        return (math.hypot(horizontal_m, vertical_m), vertical_m)
