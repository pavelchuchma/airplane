"""Flight log round trip, crash truncation, foreign-version frames, null mode.

Run from cocpitView/:  .venv/bin/python -m unittest discover tests
"""

import json
import math
import pathlib
import struct
import tempfile
import unittest

from cockpitview import flightlog, protocol


def telemetry_payload(version=protocol.PROTOCOL_VERSION, time_boot_ms=1000, ias=12.5, flags=0):
    """A TelemetryMessage with recognisable values (only the v4 layout)."""
    fields = protocol.Telemetry(
        msg_id=protocol.MSG_TELEMETRY, version=version, flags=flags, sats=7, time_boot_ms=time_boot_ms,
        roll=0.1, pitch=-0.2, yaw=1.5, rollspeed=0.0, pitchspeed=0.0, yawspeed=0.0,
        baro_alt_m=42.0, climb_mps=-0.5, diff_press_pa=95.7, ias_mps=ias,
        lat1e7=500000000, lon1e7=145000000, alt_msl_m=math.nan, gs_mps=13.0, cog_deg=270.0,
        hdop=1.1, hacc_m=2.5, vacc_m=4.0, rssi_dbm=-70, cmd_loss_pct=2, tx_fail_pct=0,
        last_cmd_version=version, cell1_mv=3800, cell2_mv=3790, cell3_mv=3810,
    )
    return protocol._TELEMETRY.pack(*fields)


class WriteReadRoundTrip(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.dir = pathlib.Path(self._tmp.name) / "logs"

    def tearDown(self):
        self._tmp.cleanup()

    def write_session(self):
        log = flightlog.FlightLog(self.dir, args={"emulator": False, "serial_port": None})
        log.uplink(protocol.encode_command(0))  # before the first frame: dropped
        log.event("link_window", telem_loss_pct=100, frames=0)  # dropped too
        log.event("serial_open", port="/dev/tty.test")  # kept, written after the header
        for i in range(3):
            log.downlink(telemetry_payload(time_boot_ms=1000 + 20 * i))
        log.downlink(telemetry_payload(version=3)[:84])  # stale air firmware, 84 B v3 frame
        log.downlink(b"\x7f\x04junk")  # unknown message type: logged, ignored by decoders
        log.uplink(protocol.encode_command(7, channels=(100, -200, 0, 5000, 0, 0, 0, 0)))
        log.event("baro_calibrated", offset_m=312.4, field_elevation_m=354.0)
        log.close()
        files = sorted(self.dir.glob("*.cvlog"))
        self.assertEqual(len(files), 1)
        self.assertEqual(log.path, files[0])
        self.assertEqual(log.bytes_written, files[0].stat().st_size)
        return files[0]

    def test_records_and_decoding(self):
        path = self.write_session()
        records = list(flightlog.read(path))
        types = [rtype for _, rtype, _ in records]
        self.assertEqual(
            types,
            [flightlog.TYPE_HEADER, flightlog.TYPE_EVENT] + [flightlog.TYPE_DOWN] * 5 + [flightlog.TYPE_UP, flightlog.TYPE_EVENT],
        )

        hdr = flightlog.header(path)
        self.assertEqual(hdr["format"], flightlog.FORMAT)
        self.assertEqual(hdr["protocol_version"], protocol.PROTOCOL_VERSION)
        self.assertEqual(hdr["telemetry_struct"], protocol._TELEMETRY.format)
        self.assertEqual(hdr["telemetry_fields"], list(protocol.Telemetry._fields))
        self.assertEqual(hdr["telemetry_flags"]["BATT_OK"], protocol.TELEM_FLAG_BATT_OK)
        self.assertEqual(hdr["args"]["serial_port"], None)
        self.assertEqual(records[0][0], hdr["t_mono0"])

        frames = list(flightlog.telemetry_frames(path))
        self.assertEqual([f.time_boot_ms for _, f in frames], [1000, 1020, 1040])
        self.assertIsInstance(frames[0][1], protocol.Telemetry)
        self.assertAlmostEqual(frames[0][1].ias_mps, 12.5, places=5)
        self.assertTrue(math.isnan(frames[0][1].alt_msl_m))
        self.assertLessEqual(frames[0][0], frames[1][0])

        cmds = list(flightlog.commands(path))
        self.assertEqual(len(cmds), 1)  # the pre-open command was dropped
        self.assertEqual(cmds[0][1].seq, 7)
        self.assertEqual(cmds[0][1].ch3, 5000)

        evs = [ev for _, ev in flightlog.events(path)]
        self.assertEqual([ev["ev"] for ev in evs], ["serial_open", "baro_calibrated"])
        self.assertEqual(evs[1]["offset_m"], 312.4)

    def test_foreign_version_counted_not_decoded(self):
        path = self.write_session()
        decoder = flightlog.MessageDecoder.telemetry(flightlog.header(path))
        decoded = [decoder.decode(p) for _, rtype, p in flightlog.read(path) if rtype == flightlog.TYPE_DOWN]
        self.assertEqual(sum(d is not None for d in decoded), 3)
        self.assertEqual(decoder.skipped, {3: 1})  # the junk payload is another msgId, not counted

    def test_truncated_tail_is_dropped(self):
        path = self.write_session()
        complete = list(flightlog.read(path))
        with open(path, "ab") as f:
            f.write(flightlog.RECORD.pack(99.0, flightlog.TYPE_DOWN, 90) + b"\x01\x04half")
        self.assertEqual(list(flightlog.read(path)), complete)
        self.assertEqual(len(list(flightlog.telemetry_frames(path))), 3)

    def test_header_decodes_by_stored_layout(self):
        """A log written with a hypothetical older layout stays readable:
        the reader unpacks by the header's struct, missing fields are NaN."""
        path = self.dir / "old.cvlog"
        self.dir.mkdir()
        fmt, fields = "<BBBBIf", ["msg_id", "version", "flags", "sats", "time_boot_ms", "ias_mps"]
        hdr = dict(flightlog.build_header(0.0, {}), protocol_version=2, telemetry_struct=fmt, telemetry_fields=fields)
        payload = struct.pack(fmt, protocol.MSG_TELEMETRY, 2, 0, 5, 777, 9.5)
        with open(path, "wb") as f:
            f.write(flightlog.pack_record(0.0, flightlog.TYPE_HEADER, json.dumps(hdr).encode()))
            f.write(flightlog.pack_record(0.02, flightlog.TYPE_DOWN, payload))
        (t, frame), = flightlog.telemetry_frames(path)
        self.assertEqual(frame.time_boot_ms, 777)
        self.assertAlmostEqual(frame.ias_mps, 9.5, places=5)
        self.assertTrue(math.isnan(frame.roll))
        self.assertEqual(frame._fields, protocol.Telemetry._fields)

    def test_no_frame_no_file(self):
        log = flightlog.FlightLog(self.dir)
        for i in range(50):
            log.uplink(protocol.encode_command(i))
        log.event("link_window", telem_loss_pct=100, frames=0)
        log.close()
        self.assertFalse(self.dir.exists())
        self.assertIsNone(log.path)

    def test_null_log(self):
        log = flightlog.FlightLog(None)
        self.assertFalse(log.enabled)
        log.downlink(b"\x01")
        log.uplink(b"\x02")
        log.event("serial_open", port="x")
        log.close()
        log.close()  # idempotent
        self.assertIsNone(log.path)

    def test_put_after_close_is_noop(self):
        log = flightlog.FlightLog(self.dir)
        log.downlink(telemetry_payload())
        log.close()
        log.downlink(telemetry_payload())
        log.close()
        self.assertEqual(len(list(flightlog.telemetry_frames(log.path))), 1)


if __name__ == "__main__":
    unittest.main()
