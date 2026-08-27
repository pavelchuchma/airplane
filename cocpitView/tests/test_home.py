"""Home-point lock gate, distance/vertical math, and the altimeter riding
on the same samples.

Run from cocpitView/:  .venv/bin/python -m unittest discover tests
"""

import math
import unittest

from cockpitview import home
from cockpitview.altimeter import BaroCalibrator
from cockpitview.home import HomePoint
from cockpitview.hud import format_distance
from cockpitview.telemetry import TelemetrySnapshot

HOME_LAT, HOME_LON = 50.000000, 15.000000
BARO_REL_M = 3.0  # the plane booted 3 m below where it now stands


class Msg:
    """Minimal stand-in for a pymavlink message (only get_type() + fields)."""

    def __init__(self, mtype, **fields):
        self._type = mtype
        self.__dict__.update(fields)

    def get_type(self):
        return self._type


def altitude_msg(relative_m=BARO_REL_M, amsl_m=float("nan")):
    return Msg("ALTITUDE", altitude_relative=relative_m, altitude_amsl=amsl_m)


def gps_raw_msg(fix_type=3, hacc_mm=2500):
    return Msg("GPS_RAW_INT", fix_type=fix_type, h_acc=hacc_mm)


def position_msg(lat=HOME_LAT, lon=HOME_LON, vx=0, vy=0, relative_alt_mm=int(BARO_REL_M * 1000)):
    return Msg(
        "GLOBAL_POSITION_INT",
        lat=int(lat * 1e7),
        lon=int(lon * 1e7),
        vx=vx,
        vy=vy,
        relative_alt=relative_alt_mm,
        alt=0,
    )


def feed(point, samples, baro_rel_m=BARO_REL_M, fix_type=3, hacc_mm=2500, vx=0, vy=0, baro=True):
    """Runs `samples` GPS epochs (baro + fix quality + position) through a
    HomePoint the way TelemetryHub.publish would."""
    for _ in range(samples):
        if baro:
            point.process(altitude_msg(baro_rel_m))
        point.process(gps_raw_msg(fix_type, hacc_mm))
        point.process(position_msg(vx=vx, vy=vy))


def snapshot(lat=None, lon=None, alt_relative_m=None):
    """A snapshot with a fresh position / relative altitude, or without."""
    snap = TelemetrySnapshot(now=1000.0)
    if lat is not None:
        snap.lat_deg, snap.lon_deg, snap.gpi_ts = lat, lon, 1000.0
    if alt_relative_m is not None:
        snap.alt_relative_m, snap.alt_ts = alt_relative_m, 1000.0
    return snap


class Lock(unittest.TestCase):
    def test_locks_after_the_full_window(self):
        point = HomePoint()
        feed(point, home.SAMPLES_TO_LOCK - 1)
        self.assertIsNone(point.fix)
        feed(point, 1)
        lat, lon, baro_rel = point.fix
        self.assertAlmostEqual(lat, HOME_LAT, places=7)
        self.assertAlmostEqual(lon, HOME_LON, places=7)
        self.assertAlmostEqual(baro_rel, BARO_REL_M)

    def test_medians_away_position_noise(self):
        point = HomePoint()
        # a third of the samples thrown 100 m north, the median must not care
        for i in range(home.SAMPLES_TO_LOCK):
            point.process(altitude_msg())
            point.process(gps_raw_msg())
            offset = 0.001 if i % 3 == 0 else 0.0
            point.process(position_msg(lat=HOME_LAT + offset))
        lat, _, _ = point.fix
        self.assertAlmostEqual(lat, HOME_LAT, places=7)

    def test_gate_rejects(self):
        cases = {
            "2D fix": dict(fix_type=2),
            "no error estimate": dict(hacc_mm=0),
            "error estimate too large": dict(hacc_mm=int(home.MAX_HACC_M * 1000) + 1),
            "moving": dict(vx=int(home.MAX_GROUNDSPEED_MPS * 100) + 1),
            "no baro sample": dict(baro=False),
        }
        for name, kwargs in cases.items():
            with self.subTest(name):
                point = HomePoint()
                feed(point, 4 * home.SAMPLES_TO_LOCK, **kwargs)
                self.assertIsNone(point.fix)

    def test_locks_once_and_ignores_later_positions(self):
        point = HomePoint()
        feed(point, home.SAMPLES_TO_LOCK)
        fix = point.fix
        for _ in range(home.SAMPLES_TO_LOCK):
            point.process(altitude_msg(100.0))
            point.process(gps_raw_msg())
            point.process(position_msg(lat=HOME_LAT + 0.01))
        self.assertEqual(point.fix, fix)


class Offsets(unittest.TestCase):
    def setUp(self):
        self.point = HomePoint()
        feed(self.point, home.SAMPLES_TO_LOCK)

    def test_none_before_the_lock(self):
        self.assertEqual(HomePoint().offsets_from(snapshot(HOME_LAT, HOME_LON, 10.0)), (None, None))

    def test_zero_at_home(self):
        dist, vert = self.point.offsets_from(snapshot(HOME_LAT, HOME_LON, BARO_REL_M))
        self.assertAlmostEqual(dist, 0.0, places=3)
        self.assertAlmostEqual(vert, 0.0, places=3)

    def test_slant_range(self):
        # 900 m north, 1200 m up -> 1500 m hypotenuse (3-4-5 triangle)
        lat = HOME_LAT + math.degrees(900.0 / home.EARTH_RADIUS_M)
        dist, vert = self.point.offsets_from(snapshot(lat, HOME_LON, BARO_REL_M + 1200.0))
        self.assertAlmostEqual(vert, 1200.0, places=3)
        self.assertAlmostEqual(dist, 1500.0, delta=0.5)

    def test_east_distance_is_scaled_by_latitude(self):
        lat_scale = math.cos(math.radians(HOME_LAT))
        lon = HOME_LON + math.degrees(1000.0 / (home.EARTH_RADIUS_M * lat_scale))
        dist, _ = self.point.offsets_from(snapshot(HOME_LAT, lon, BARO_REL_M))
        self.assertAlmostEqual(dist, 1000.0, delta=0.5)

    def test_below_home_is_negative(self):
        _, vert = self.point.offsets_from(snapshot(HOME_LAT, HOME_LON, BARO_REL_M - 12.0))
        self.assertAlmostEqual(vert, -12.0, places=3)

    def test_without_position_only_the_vertical_survives(self):
        dist, vert = self.point.offsets_from(snapshot(alt_relative_m=BARO_REL_M + 50.0))
        self.assertIsNone(dist)
        self.assertAlmostEqual(vert, 50.0, places=3)

    def test_without_baro_the_distance_is_horizontal_only(self):
        lat = HOME_LAT + math.degrees(900.0 / home.EARTH_RADIUS_M)
        dist, vert = self.point.offsets_from(snapshot(lat, HOME_LON))
        self.assertIsNone(vert)
        self.assertAlmostEqual(dist, 900.0, delta=0.5)


class FlatTerrain:
    def __init__(self, elevation_m=400.0):
        self._elevation_m = elevation_m
        self.queries = 0

    def elevation_m(self, lat_deg, lon_deg):
        self.queries += 1
        return self._elevation_m


class NoTerrain(FlatTerrain):
    def elevation_m(self, lat_deg, lon_deg):
        self.queries += 1
        return None


class SharedLock(unittest.TestCase):
    """The altimeter must lock off the very same samples as the home point -
    same gate, same instant (see home.py)."""

    def test_calibrates_with_the_home_lock(self):
        point = HomePoint()
        terrain = FlatTerrain(400.0)
        calibrator = BaroCalibrator(terrain, point)
        chain = [point, calibrator]

        def publish(msg):
            for processor in chain:
                msg = processor.process(msg)
            return msg

        for _ in range(home.SAMPLES_TO_LOCK - 1):
            publish(altitude_msg())
            publish(gps_raw_msg())
            publish(position_msg())
        # not calibrated yet: altitude_amsl still NaN, GPI alt untouched
        self.assertTrue(math.isnan(publish(altitude_msg()).altitude_amsl))
        publish(gps_raw_msg())
        publish(position_msg())  # the 25th sample locks both

        self.assertIsNotNone(point.fix)
        # offset = terrain - baro_rel = 400 - 3, so a 10 m relative altitude is 407 AMSL
        self.assertAlmostEqual(publish(altitude_msg(10.0)).altitude_amsl, 407.0, places=3)
        self.assertEqual(publish(position_msg(relative_alt_mm=10_000)).alt, 407_000)

    def test_home_locks_even_when_the_terrain_is_unknown(self):
        point = HomePoint()
        terrain = NoTerrain()
        calibrator = BaroCalibrator(terrain, point)
        feed(point, home.SAMPLES_TO_LOCK)
        self.assertIsNotNone(point.fix)
        self.assertGreater(terrain.queries, 0)
        # no elevation -> no offset, ALTITUDE passes through unpatched
        self.assertTrue(math.isnan(calibrator.process(altitude_msg(10.0)).altitude_amsl))

    def test_no_terrain_queries_after_the_lock(self):
        point = HomePoint()
        terrain = FlatTerrain()
        BaroCalibrator(terrain, point)
        feed(point, home.SAMPLES_TO_LOCK)
        queries = terrain.queries
        feed(point, home.SAMPLES_TO_LOCK)
        self.assertEqual(terrain.queries, queries)


class Formatting(unittest.TestCase):
    def test_thresholds(self):
        for meters, text in (
            (0.0, "0m"),
            (7.4, "7m"),
            (840.0, "840m"),
            (999.4, "999m"),
            (999.6, "1.0km"),
            (1000.0, "1.0km"),
            (1542.0, "1.5km"),
            (12345.0, "12.3km"),
        ):
            with self.subTest(meters=meters):
                self.assertEqual(format_distance(meters), text)


if __name__ == "__main__":
    unittest.main()
