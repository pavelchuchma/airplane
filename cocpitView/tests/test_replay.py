"""Log replay: pacing, the warm-up before --replay-start, and decoding by
the log's own header.

Run from cocpitView/:  .venv/bin/python -m unittest discover tests
"""

import json
import math
import pathlib
import tempfile
import unittest

from pymavlink.dialects.v20 import ardupilotmega as mavlink2

from cockpitview import flightlog, protocol, replay_source
from test_flightlog import telemetry_payload

T0 = 100.0  # log timestamps are raw time.monotonic(): never start at zero
IMU_OK = protocol.TELEM_FLAG_IMU_OK


class FakeHub:
    """Records what a source publishes, and whether it would reach QGC."""

    def __init__(self):
        # v20 explicitly: pymavlink's default dialect is MAVLink 1 until a
        # connection switches it over, and the publisher fills extension
        # fields (GPS_RAW_INT h_acc/v_acc) that only exist in MAVLink 2
        self.mav = mavlink2.MAVLink(None, srcSystem=1)
        self.published = []  # (msg, forward)
        self.link_updates = []

    def publish(self, msg, forward=True):
        self.published.append((msg, forward))

    def update_link(self, **kwargs):
        self.link_updates.append(kwargs)

    def attitudes(self, forward=None):
        return [
            msg
            for msg, fwd in self.published
            if msg.get_type() == "ATTITUDE" and (forward is None or fwd == forward)
        ]


def write_log(path, frames, protocol_version=protocol.PROTOCOL_VERSION):
    """frames: [(t_rel, payload)]. The header is the current layout unless a
    version is forced (then only the version field differs - enough for the
    upgrade path, which keys off it)."""
    hdr = flightlog.build_header(T0, {})
    hdr["protocol_version"] = protocol_version
    with open(path, "wb") as f:
        f.write(flightlog.pack_record(T0, flightlog.TYPE_HEADER, json.dumps(hdr).encode()))
        for t_rel, payload in frames:
            f.write(flightlog.pack_record(T0 + t_rel, flightlog.TYPE_DOWN, payload))


def steady_frames(n, dt=0.02, start=0.0, version=protocol.PROTOCOL_VERSION):
    return [
        (start + i * dt, telemetry_payload(version=version, time_boot_ms=1000 + int((start + i * dt) * 1000), flags=IMU_OK))
        for i in range(n)
    ]


class ReplayTestCase(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.path = pathlib.Path(self._tmp.name) / "flight.cvlog"
        self.hub = FakeHub()
        self.waits = []  # every target time the source would have slept until

    def tearDown(self):
        self._tmp.cleanup()

    def run_replay(self, **kwargs):
        source = replay_source.ReplaySource(self.hub, self.path, **kwargs)
        source._sleep_until = self.waits.append  # never actually sleep
        source._run()
        return source


class Playback(ReplayTestCase):
    def test_every_frame_reaches_the_hub_in_order(self):
        write_log(self.path, steady_frames(5))
        self.run_replay()
        booted = [msg.time_boot_ms for msg in self.hub.attitudes()]
        self.assertEqual(len(booted), 5)
        self.assertEqual(booted, sorted(booted))
        self.assertTrue(all(fwd for _, fwd in self.hub.published))

    def test_schedule_follows_log_time_and_speed(self):
        write_log(self.path, steady_frames(4, dt=0.5))
        self.run_replay(speed=2.0)
        # first frame sets the reference, the other three are scheduled
        self.assertEqual(len(self.waits), 3)
        offsets = [round(w - self.waits[0], 6) for w in self.waits]
        self.assertEqual(offsets, [0.0, 0.25, 0.5])  # 0.5 s of log time at 2x

    def test_gap_is_capped_in_wall_time_but_not_in_log_time(self):
        frames = steady_frames(2, dt=0.5) + steady_frames(2, dt=0.5, start=60.0)
        write_log(self.path, frames)
        source = self.run_replay(max_gap_s=1.0)
        offsets = [round(w - self.waits[0], 6) for w in self.waits]
        self.assertEqual(offsets, [0.0, 1.0, 1.5])  # the 59.5 s gap waits 1 s
        self.assertAlmostEqual(source.clock(), 60.5)  # ...but log time jumped it all

    def test_replay_end_stops_the_playback(self):
        write_log(self.path, steady_frames(10, dt=1.0))
        self.run_replay(end_s=3.0)
        self.assertEqual(len(self.hub.attitudes()), 4)  # t = 0, 1, 2, 3

    def test_link_stats_are_published(self):
        write_log(self.path, steady_frames(3))
        self.run_replay()
        self.assertEqual(len(self.hub.link_updates), 3)
        self.assertEqual(self.hub.link_updates[0]["rssi_dbm"], -70)


class StartOffset(ReplayTestCase):
    def setUp(self):
        super().setUp()
        write_log(self.path, steady_frames(20, dt=1.0))  # 0..19 s

    def test_warm_up_reaches_the_processors_but_not_qgc(self):
        source = self.run_replay(start_s=5.0)
        self.assertEqual(len(self.hub.attitudes(forward=False)), 5)  # 0..4 s
        self.assertEqual(len(self.hub.attitudes(forward=True)), 15)  # 5..19 s
        self.assertEqual(len(self.waits), 14)  # nothing is paced during the warm-up
        self.assertAlmostEqual(source.clock(), 19.0)

    def test_cold_start_drops_the_prefix_entirely(self):
        self.run_replay(start_s=5.0, cold=True)
        self.assertEqual(self.hub.attitudes(forward=False), [])
        self.assertEqual(len(self.hub.attitudes(forward=True)), 15)

    def test_clock_runs_in_log_time_during_the_warm_up(self):
        """The processors that time themselves (yawcal's 60 s window,
        home's baro freshness) must see the flight's clock, not the
        fraction of a second the fast-forward takes."""
        seen = []
        source = replay_source.ReplaySource(self.hub, self.path, start_s=5.0)
        source._sleep_until = self.waits.append
        self.hub.publish = lambda msg, forward=True: seen.append(source.clock())
        source._run()
        self.assertEqual(seen[0], 0.0)
        self.assertEqual(max(seen), 19.0)


class HeaderDrivenDecoding(ReplayTestCase):
    def test_v4_mirrored_yaw_is_flipped(self):
        """A v4 log replays with the heading a v5 firmware would have sent
        (flightlog.upgrade), so an old flight is watchable as flown."""
        write_log(self.path, steady_frames(1, version=4), protocol_version=4)
        self.run_replay()
        self.assertAlmostEqual(self.hub.attitudes()[0].yaw, -1.5, places=5)

    def test_v5_yaw_passes_through(self):
        write_log(self.path, steady_frames(1))
        self.run_replay()
        self.assertAlmostEqual(self.hub.attitudes()[0].yaw, 1.5, places=5)

    def test_foreign_version_frames_are_skipped(self):
        frames = steady_frames(2) + [(1.0, telemetry_payload(version=3, flags=IMU_OK)[:84])]
        write_log(self.path, frames)
        self.run_replay()
        self.assertEqual(len(self.hub.attitudes()), 2)

    def test_a_file_that_is_not_a_log_is_rejected_before_the_hud(self):
        self.path.write_bytes(b"not a cvlog at all")
        source = replay_source.ReplaySource(self.hub, self.path)
        with self.assertRaises(ValueError):
            source.start()


class ParseTime(unittest.TestCase):
    def test_forms(self):
        self.assertEqual(replay_source.parse_time("12"), 12.0)
        self.assertEqual(replay_source.parse_time("1.5"), 1.5)
        self.assertEqual(replay_source.parse_time("3:15"), 195.0)
        self.assertEqual(replay_source.parse_time("1:02:03"), 3723.0)

    def test_rejects_nonsense(self):
        for bad in ("1:2:3:4", "abc", ""):
            with self.assertRaises(ValueError):
                replay_source.parse_time(bad)


if __name__ == "__main__":
    unittest.main()


class GyroCalibrationFlag(ReplayTestCase):
    """TELEM_FLAG_IMU_CALIBRATING: no ATTITUDE (the fields are zero), and the
    hub gets the raw-rate magnitude to show next to "hold still"."""

    def test_calibrating_frames_publish_motion_instead_of_attitude(self):
        frames = [
            (0.0, telemetry_payload(time_boot_ms=1000, flags=protocol.TELEM_FLAG_IMU_CALIBRATING)),
            (0.02, telemetry_payload(time_boot_ms=1020, flags=IMU_OK)),
        ]
        write_log(self.path, frames)
        self.run_replay()
        self.assertEqual(len(self.hub.attitudes()), 1)
        self.assertEqual(self.hub.link_updates[0]["imu_cal_motion_dps"], 0.0)
        self.assertIsNone(self.hub.link_updates[1]["imu_cal_motion_dps"])
