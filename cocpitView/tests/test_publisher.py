"""TelemetryPublisher: every integer MAVLink field must survive whatever the
wire delivers. Regression for the 2026-11-04 session, where one GPS_RAW_INT
with h_acc = 5303 km (a receiver acquiring its fix) overflowed uint32 in
pymavlink's pack and killed the serial reader thread for the rest of the
session."""

import math
import unittest

import serial
from pymavlink import mavutil

from cockpitview import espnow_source, flightlog, protocol, publisher
from cockpitview.telemetry import TelemetryHub

ALL_OK = (
    protocol.TELEM_FLAG_GPS_FIX
    | protocol.TELEM_FLAG_BARO_OK
    | protocol.TELEM_FLAG_PITOT_OK
    | protocol.TELEM_FLAG_IMU_OK
    | protocol.TELEM_FLAG_BATT_OK
)


def frame(**overrides):
    values = dict(
        msg_id=protocol.MSG_TELEMETRY, version=protocol.PROTOCOL_VERSION, flags=ALL_OK, sats=0,
        time_boot_ms=133_400_000,
        roll=0.0, pitch=0.0, yaw=0.5, rollspeed=0.0, pitchspeed=0.0, yawspeed=0.0,
        baro_alt_m=-140.9, climb_mps=-143.3, diff_press_pa=0.0, ias_mps=0.0,
        lat1e7=490_782_152, lon1e7=180_231_487, alt_msl_m=200.0, gs_mps=0.139, cog_deg=0.0,
        hdop=99.99, hacc_m=5_303_308.0, vacc_m=3_750_005.0,
        rssi_dbm=-60, cmd_loss_pct=0, tx_fail_pct=0, last_cmd_version=protocol.PROTOCOL_VERSION,
        cell1_mv=3800, cell2_mv=3800, cell3_mv=3800,
        vtx_freq_mhz=0, vtx_power_mw=0, vtx_temp_c=protocol.VTX_TEMP_UNKNOWN, vtx_status=0,
    )
    values.update(overrides)
    return protocol.Telemetry(**values)


def hub():
    return TelemetryHub(qgc_host="127.0.0.1", qgc_port=14599)


class IntFields(unittest.TestCase):
    def test_clamp_and_unknown(self):
        self.assertEqual(publisher._u32(5.3e9), publisher.UINT32_MAX)
        self.assertEqual(publisher._u32(-5.0), 0)
        self.assertEqual(publisher._u32(math.nan), 0)
        self.assertEqual(publisher._u16(math.nan), publisher.UINT16_MAX)
        self.assertEqual(publisher._u16(math.inf), publisher.UINT16_MAX)
        self.assertEqual(publisher._i16(-1e9), -32768)
        self.assertEqual(publisher._i32(1e12), publisher.INT32_MAX)
        self.assertEqual(publisher._i32(12.7), 12)

    def test_acquiring_receiver_frame_is_sent(self):
        """The exact field values of the frame that crashed the session."""
        publisher.TelemetryPublisher(hub()).publish(frame(), 133400.8, 133400.8, forward=True)

    def test_negative_yaw_wraps_to_compass_heading(self):
        h = hub()
        publisher.TelemetryPublisher(h).publish(frame(yaw=math.radians(-30.5)), 1.0, 1.0, forward=True)
        self.assertEqual(h.snapshot().vfr_heading_deg, 330)

    def test_no_receiver_sentinel_reaches_the_snapshot(self):
        h = hub()
        publisher.TelemetryPublisher(h).publish(
            frame(flags=ALL_OK & ~protocol.TELEM_FLAG_GPS_FIX, sats=protocol.TELEM_SATS_NO_RECEIVER),
            1.0, 1.0, forward=True,
        )
        fix_type, sats, _ = h.snapshot().gps_status
        self.assertEqual(fix_type, mavutil.mavlink.GPS_FIX_TYPE_NO_FIX)
        self.assertIsNone(sats)

    def test_nan_and_inf_everywhere(self):
        nans = {name: math.nan for name, v in frame()._asdict().items() if isinstance(v, float)}
        pub = publisher.TelemetryPublisher(hub())
        pub.publish(frame(**nans), 1.0, 1.0, forward=True)
        infs = {name: math.inf for name in nans}
        pub.publish(frame(**infs), 2.0, 2.0, forward=True)
        pub.publish(frame(**{name: -math.inf for name in nans}), 3.0, 3.0, forward=True)


class FakeSerial:
    """Delivers the given frames in one read, then reports the port lost."""

    def __init__(self, payloads):
        self._data = b"".join(protocol.encode_frame(p) for p in payloads)

    @property
    def in_waiting(self):
        return len(self._data)

    def read(self, n):
        if not self._data:
            raise serial.SerialException("gone")
        out, self._data = self._data[:n], self._data[n:]
        return out


class ReaderSurvivesBadFrame(unittest.TestCase):
    def test_publish_error_is_logged_and_reading_goes_on(self):
        events = []

        class Log(flightlog.FlightLog):
            def event(self, name, **fields):
                events.append((name, fields))

        source = espnow_source.EspNowSource(hub(), port="fake", log=Log(None))
        published = []

        def boom(t, *args, **kwargs):
            published.append(t)
            if len(published) == 1:
                raise ValueError("bad frame")

        source._publisher.publish = boom
        payload = protocol._TELEMETRY.pack(*frame())
        with self.assertRaises(serial.SerialException):
            source._read(FakeSerial([payload, payload]))
        self.assertEqual(len(published), 2)
        names = [name for name, _ in events]
        self.assertEqual(names.count("publish_error"), 1)
        self.assertIn("ValueError: bad frame", events[names.index("publish_error")][1]["error"])


if __name__ == "__main__":
    unittest.main()
