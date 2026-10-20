"""Air diagnostic lines: LogMessage codec, the .air.log formatter and writer.

Run from cocpitView/:  .venv/bin/python -m unittest discover tests
"""

import json
import pathlib
import tempfile
import unittest

from cockpitview import flightlog, protocol

from test_flightlog import telemetry_payload

HDR = {
    "t_mono0": 100.0,
    "started_utc": "2026-10-20T15:00:00.000Z",
    "protocol_version": protocol.PROTOCOL_VERSION,
    "log_struct": protocol._LOG_HEADER.format,
    "log_fields": ["msg_id", "version", "seq", "time_boot_ms", "dropped"],
}


def down(seq, ms, text, dropped=0):
    return (flightlog.TYPE_DOWN, protocol.encode_log(seq, ms, text, dropped))


class LogMessageCodec(unittest.TestCase):
    def test_round_trip(self):
        line = protocol.decode_log(protocol.encode_log(412, 98765, "slow I2C: MS4525DO took 1004 ms", dropped=3))
        self.assertEqual((line.seq, line.time_boot_ms, line.dropped), (412, 98765, 3))
        self.assertEqual(line.text, "slow I2C: MS4525DO took 1004 ms")

    def test_malformed_is_none(self):
        self.assertIsNone(protocol.decode_log(b"\x03\x07\x00"))  # shorter than the header
        self.assertIsNone(protocol.decode_log(telemetry_payload()))
        too_long = protocol.encode_log(1, 1, "") + b"x" * (protocol.LOG_TEXT_MAX + 1)
        self.assertIsNone(protocol.decode_log(too_long))

    def test_encode_truncates_like_the_air_side(self):
        line = protocol.decode_log(protocol.encode_log(0, 0, "y" * 200))
        self.assertEqual(len(line.text), protocol.LOG_TEXT_MAX)


class Formatter(unittest.TestCase):
    def feed(self, records, hdr=HDR):
        fmt = flightlog.AirLogFormatter(hdr)
        out = []
        for t, (rtype, payload) in records:
            out += fmt.feed(t, rtype, payload)
        return out, fmt

    def test_line_format(self):
        out, fmt = self.feed([(172.3, down(0, 98765, "gyro bias 0.1 0.2 0.3 deg/s after 1.0 s"))])
        self.assertEqual(
            out,
            [
                "2026-10-20T15:01:12.300Z  00:01:12.3  --- air boot ---",
                "2026-10-20T15:01:12.300Z  00:01:12.3  boot    98.765  #0     gyro bias 0.1 0.2 0.3 deg/s after 1.0 s",
            ],
        )
        self.assertEqual(fmt.lines, 1)

    def test_link_window_reanchors_utc(self):
        heartbeat = json.dumps({"ev": "link_window", "utc": "2026-10-20T16:00:00.000+00:00"}).encode()
        out, _ = self.feed([(200.0, (flightlog.TYPE_EVENT, heartbeat)), (201.5, down(0, 10, "x"))])
        self.assertTrue(out[-1].startswith("2026-10-20T16:00:01.500Z  00:01:41.5  "))

    def test_gap_duplicate_and_reboot(self):
        out, fmt = self.feed(
            [
                (101.0, down(0, 1000, "a")),
                (101.1, down(1, 1100, "b")),
                (101.2, down(1, 1100, "b")),  # duplicate
                (101.3, down(5, 1500, "f", dropped=1)),  # 2..4 missing, 1 of them overflow
                (110.0, down(2, 300, "after reboot", dropped=2)),  # new boot, 0..1 overflowed
            ]
        )
        texts = [line.split("  ", 2)[2] for line in out]
        self.assertEqual(
            texts,
            [
                "--- air boot ---",
                "boot     1.000  #0     a",
                "boot     1.100  #1     b",
                "[lost 3 lines (1 buffer overflow on air, 2 radio)]",
                "boot     1.500  #5     f",
                "--- air boot already running, joined at #2 ---",
                "[lost 2 lines (2 buffer overflow on air, 0 radio)]",
                "boot     0.300  #2     after reboot",
            ],
        )
        self.assertEqual(fmt.lines, 4)

    def test_joining_a_running_boot_reports_no_loss(self):
        out, _ = self.feed([(101.0, down(7, 9000, "late"))])
        self.assertEqual(len(out), 2)
        self.assertIn("joined at #7", out[0])

    def test_ignores_telemetry_foreign_versions_and_old_headers(self):
        stale = bytearray(protocol.encode_log(0, 1, "v6?"))
        stale[1] = 6
        records = [(101.0, (flightlog.TYPE_DOWN, telemetry_payload())), (101.1, (flightlog.TYPE_DOWN, bytes(stale)))]
        self.assertEqual(self.feed(records)[0], [])
        old_hdr = {k: v for k, v in HDR.items() if not k.startswith("log_")}
        self.assertEqual(self.feed([(101.0, down(0, 1, "x"))], hdr=old_hdr)[0], [])


class Writer(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.dir = pathlib.Path(self._tmp.name) / "logs"

    def tearDown(self):
        self._tmp.cleanup()

    def test_air_log_next_to_cvlog_matches_diag(self):
        log = flightlog.FlightLog(self.dir)
        log.downlink(telemetry_payload())
        log.downlink(protocol.encode_log(0, 500, "gyro calibration: waiting for the airframe to be still"))
        log.event("link_window", telem_loss_pct=0, frames=50, utc="2026-10-20T15:00:01.000+00:00")
        log.downlink(protocol.encode_log(1, 1500, "battery absent - data nan for this flight"))
        log.close()

        self.assertEqual(log.air_log_path, flightlog.air_log_path(log.path))
        self.assertEqual(log.air_log_path.name, log.path.stem + ".air.log")
        live = log.air_log_path.read_text().splitlines()
        self.assertEqual(len(live), 3)
        self.assertTrue(live[2].endswith("#1     battery absent - data nan for this flight"))
        self.assertEqual(live, list(flightlog.air_log_lines(log.path)))

    def test_no_air_line_no_air_log(self):
        log = flightlog.FlightLog(self.dir)
        log.downlink(telemetry_payload())
        log.close()
        self.assertIsNone(log.air_log_path)
        self.assertEqual(list(self.dir.glob("*.air.log")), [])


if __name__ == "__main__":
    unittest.main()
