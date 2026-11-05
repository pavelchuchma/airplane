import math
import tempfile
import unittest

from cockpitview import localconf, protocol, publisher, telemetry, vtx
from cockpitview.command import CommandState
from cockpitview.telemetry import TelemetryHub, VtxState

PIT = protocol.CMD_VTX_POWER_PIT
LEAVE = protocol.CMD_VTX_POWER_LEAVE


class FakeClock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def readback(power_mw=25, pit=False, alive=True, **kw):
    return VtxState(alive=alive, freq_mhz=5800, power_mw=power_mw, pit=pit, race_lock=False, refused=False,
                    temp_c=42, **kw)


class PowerStepping(unittest.TestCase):
    def test_table_walk_down_ends_in_pit_and_stays(self):
        p = 1600
        seen = []
        for _ in range(7):
            p = vtx.step_power(p, p == PIT, -1)
            seen.append(p)
        self.assertEqual(seen, [800, 400, 200, 25, PIT, PIT, PIT])

    def test_up_from_pit_is_the_legal_step_and_1600_is_the_ceiling(self):
        self.assertEqual(vtx.step_power(400, True, +1), 25)
        self.assertEqual(vtx.step_power(1600, False, +1), 1600)
        self.assertEqual(vtx.step_power(25, False, +1), 200)

    def test_unlisted_readback_snaps_to_the_side_of_the_move(self):
        self.assertEqual(vtx.step_power(100, False, -1), 25)
        self.assertEqual(vtx.step_power(100, False, +1), 200)

    def test_pit_toggle(self):
        self.assertEqual(vtx.toggle_pit(200, False), PIT)
        self.assertEqual(vtx.toggle_pit(200, True), 200)
        self.assertEqual(vtx.toggle_pit(100, True), 25)  # unlisted configured power


class Frequencies(unittest.TestCase):
    def test_mhz_and_band_channel(self):
        self.assertEqual(vtx.parse_frequency("5800"), 5800)
        self.assertEqual(vtx.parse_frequency("R4"), 5769)
        self.assertEqual(vtx.parse_frequency(" f4 "), 5800)
        self.assertEqual(vtx.channel_name(5800), "F4")
        self.assertEqual(vtx.channel_name(5880), "F8")  # also R7 - first table wins

    def test_rejects_what_the_receiver_cannot_tune(self):
        for bad in ("5000", "R9", "X1", "abc", "5801"):
            with self.assertRaises(ValueError):
                vtx.parse_frequency(bad)

    def test_local_ini_section(self):
        with tempfile.NamedTemporaryFile("w", suffix=".ini", delete=False) as f:
            f.write("[site]\nlat = 50\nlon = 15\n[vtx]\nfreq = R4 ; raceband 4\n")
        self.assertEqual(localconf.vtx_frequency(f.name), "R4")
        with tempfile.NamedTemporaryFile("w", suffix=".ini", delete=False) as f:
            f.write("[site]\nlat = 50\nlon = 15\n")
        self.assertIsNone(localconf.vtx_frequency(f.name))


class Control(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.commands = CommandState(vtx_freq_mhz=5806, clock=self.clock)
        self.control = vtx.VtxControl(self.commands)

    def test_nothing_requested_until_a_key(self):
        self.assertEqual(self.commands.vtx_request(), (5806, LEAVE, math.inf))
        cmd = protocol.decode_command(self.commands.encode(3))
        self.assertEqual((cmd.seq, cmd.vtx_freq_mhz, cmd.vtx_power_mw), (3, 5806, LEAVE))

    def test_first_press_steps_from_the_readback(self):
        self.assertEqual(self.control.step(readback(power_mw=400), -1), 200)
        self.assertEqual(self.commands.vtx_request()[1], 200)

    def test_following_presses_step_from_the_request(self):
        rb = readback(power_mw=400)
        self.control.step(rb, -1)  # 200, VTX still reads 400
        self.assertEqual(self.control.step(rb, -1), 25)
        self.assertEqual(self.control.step(rb, -1), PIT)
        self.assertEqual(self.control.step(rb, +1), 25)  # from the requested pit, not the read 400

    def test_no_readback_no_request(self):
        self.assertIsNone(self.control.step(None, +1))
        self.assertIsNone(self.control.step(readback(alive=False), +1))
        self.assertIsNone(self.control.toggle_pit(readback(power_mw=0)))  # no 'v' yet
        self.assertEqual(self.commands.vtx_request()[1], LEAVE)

    def test_request_age_follows_the_clock(self):
        self.control.step(readback(), +1)
        self.clock.t += 1.5
        self.assertAlmostEqual(self.commands.vtx_request()[2], 1.5)
        self.control.step(readback(), +1)  # 400 - a new value restarts the age
        self.assertAlmostEqual(self.commands.vtx_request()[2], 0.0)


class Pending(unittest.TestCase):
    def test_blinks_only_after_two_seconds_unmatched(self):
        self.assertFalse(readback(power_mw=25, req_power_mw=200, req_age_s=1.0).pending)
        self.assertTrue(readback(power_mw=25, req_power_mw=200, req_age_s=2.5).pending)
        self.assertFalse(readback(power_mw=200, req_power_mw=200, req_age_s=2.5).pending)

    def test_pit_request_matches_pit_readback_regardless_of_power(self):
        self.assertFalse(readback(power_mw=400, pit=True, req_power_mw=PIT, req_age_s=9).pending)
        self.assertTrue(readback(power_mw=400, pit=False, req_power_mw=PIT, req_age_s=9).pending)
        self.assertTrue(readback(power_mw=400, pit=True, req_power_mw=400, req_age_s=9).pending)

    def test_frequency_request(self):
        self.assertTrue(readback(req_freq_mhz=5806, req_age_s=9).pending)
        self.assertFalse(readback(req_freq_mhz=5800, req_age_s=9).pending)
        self.assertFalse(readback(req_age_s=9).pending)  # nothing requested


class SnapshotPath(unittest.TestCase):
    def frame(self, **vtx_fields):
        values = dict(
            msg_id=protocol.MSG_TELEMETRY, version=protocol.PROTOCOL_VERSION, flags=0, sats=0, time_boot_ms=1000,
            roll=0.0, pitch=0.0, yaw=0.0, rollspeed=0.0, pitchspeed=0.0, yawspeed=0.0,
            baro_alt_m=math.nan, climb_mps=math.nan, diff_press_pa=math.nan, ias_mps=math.nan,
            lat1e7=0, lon1e7=0, alt_msl_m=math.nan, gs_mps=math.nan, cog_deg=math.nan,
            hdop=math.nan, hacc_m=math.nan, vacc_m=math.nan,
            rssi_dbm=-60, cmd_loss_pct=0, tx_fail_pct=0, last_cmd_version=protocol.PROTOCOL_VERSION,
            cell1_mv=0, cell2_mv=0, cell3_mv=0,
            vtx_freq_mhz=5800, vtx_power_mw=25, vtx_temp_c=42, vtx_status=protocol.VTX_STATUS_ALIVE,
        )
        values.update(vtx_fields)
        return protocol.Telemetry(**values)

    def test_readback_reaches_the_snapshot(self):
        hub = TelemetryHub(qgc_host="127.0.0.1", qgc_port=14599)
        publisher.TelemetryPublisher(hub).publish(self.frame(), 1.0, 1.0)
        v = hub.snapshot().vtx
        self.assertEqual((v.alive, v.freq_mhz, v.power_mw, v.pit, v.temp_c), (True, 5800, 25, False, 42))

    def test_unknown_temperature_and_dead_vtx(self):
        hub = TelemetryHub(qgc_host="127.0.0.1", qgc_port=14599)
        publisher.TelemetryPublisher(hub).publish(
            self.frame(vtx_status=0, vtx_temp_c=protocol.VTX_TEMP_UNKNOWN, vtx_freq_mhz=0, vtx_power_mw=0), 1.0, 1.0
        )
        v = hub.snapshot().vtx
        self.assertFalse(v.alive)
        self.assertIsNone(v.temp_c)

    def test_old_log_without_vtx_fields_shows_no_vtx(self):
        hub = TelemetryHub(qgc_host="127.0.0.1", qgc_port=14599)
        t = self.frame()._replace(vtx_freq_mhz=math.nan, vtx_power_mw=math.nan, vtx_temp_c=math.nan, vtx_status=math.nan)
        publisher.TelemetryPublisher(hub).publish(t, 1.0, 1.0)
        self.assertIsNone(hub.snapshot().vtx)

    def test_emulator_fake_vtx_follows_a_request(self):
        from cockpitview.emulator_source import EmulatorSource

        hub = TelemetryHub(qgc_host="127.0.0.1", qgc_port=14599)
        clock = FakeClock()
        commands = CommandState(clock=clock)
        source = EmulatorSource(hub, commands=commands)
        source._publish_vtx()
        self.assertEqual((hub.snapshot().vtx.power_mw, hub.snapshot().vtx.pit), (25, False))
        commands.set_vtx_power(PIT)
        source._publish_vtx()
        self.assertFalse(hub.snapshot().vtx.pit)  # not yet - the set takes its round trip
        clock.t += 1.0
        source._publish_vtx()
        self.assertTrue(hub.snapshot().vtx.pit)


if __name__ == "__main__":
    unittest.main()
