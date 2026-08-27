"""Yaw calibration against GPS.

Ground truth is generated rather than measured: a synthetic flight is built
from a known yaw offset, wind vector and IAS scale, and the estimator has to
recover exactly those. That is a stronger check than any recorded flight,
where the truth is only ever approximately known - and it is the reason the
whole thing could be written before the airframe ever flew with it.
"""

import math
import random
import unittest

from cockpitview import yawcal


TRUE_DELTA = math.radians(37.0)  # gyro yaw is this far behind the truth
TRUE_K = 1.25  # the pitot reads 20 % low (position error)
TRUE_WIND = (3.0, -2.0)  # (north, east) m/s
TAS = 12.0


def headings(kind, n=300):
    """Heading profiles spanning the geometries a flight actually produces."""
    if kind == "straight":
        return [0.0] * n
    if kind == "circle":
        return [2 * math.pi * i / n for i in range(n)]
    if kind == "figure_eight":
        half = n // 2
        return [math.pi * i / half for i in range(half)] + [math.pi * (1 - i / half) for i in range(half)]
    if kind == "quarter":  # the emulator's profile: dwell, sweep 0-90, dwell
        sweep = [math.pi / 2 * i / 100 for i in range(100)]
        return [0.0] * 60 + sweep + [math.pi / 2] * 20 + sweep[::-1] + [0.0] * 40
    if kind == "narrow":  # +-10 deg - too little arc to split wind from offset
        return [math.radians(-10.0 + 20.0 * i / n) for i in range(n)]
    raise ValueError(kind)


def flight(kind, noise=0.0, seed=1, dt=0.2, delta=TRUE_DELTA, k=TRUE_K, wind=TRUE_WIND, tas=TAS):
    rnd = random.Random(seed)
    samples = []
    for i, true_heading in enumerate(headings(kind)):
        vn = k * tas * math.cos(true_heading) + wind[0] + rnd.gauss(0.0, noise)
        ve = k * tas * math.sin(true_heading) + wind[1] + rnd.gauss(0.0, noise)
        # the gyro reports the truth minus the offset the estimator must find
        samples.append(yawcal.Sample(i * dt, true_heading - delta, tas, vn, ve))
    return samples


class SolveTest(unittest.TestCase):
    def assert_recovers(self, fit, delta_tol_deg, k_tol=0.01, wind_tol=0.15):
        self.assertIsNone(fit.reject)
        self.assertAlmostEqual(math.degrees(yawcal.wrap_pi(fit.delta - TRUE_DELTA)), 0.0, delta=delta_tol_deg)
        self.assertAlmostEqual(fit.k, TRUE_K, delta=k_tol)
        self.assertAlmostEqual(fit.wind_n, TRUE_WIND[0], delta=wind_tol)
        self.assertAlmostEqual(fit.wind_e, TRUE_WIND[1], delta=wind_tol)

    def test_recovers_truth_without_noise(self):
        for kind in ("circle", "figure_eight", "quarter"):
            with self.subTest(kind=kind):
                self.assert_recovers(yawcal.solve(flight(kind)), delta_tol_deg=1e-6)

    def test_recovers_truth_under_gps_noise(self):
        for kind in ("circle", "figure_eight", "quarter"):
            with self.subTest(kind=kind):
                # 0.5 m/s per velocity component is pessimistic for a 3D fix
                self.assert_recovers(yawcal.solve(flight(kind, noise=0.5)), delta_tol_deg=1.0)

    def test_straight_flight_is_rejected_not_guessed(self):
        """The whole point: with one heading, offset and wind are the same
        unknown. A fit that returns a confident number here is the bug."""
        fit = yawcal.solve(flight("straight"))
        self.assertEqual(fit.reject, "geometry")
        self.assertAlmostEqual(fit.concentration, 1.0, places=6)

    def test_too_narrow_an_arc_is_rejected(self):
        fit = yawcal.solve(flight("narrow", noise=0.3))
        self.assertEqual(fit.reject, "geometry")

    def test_geometry_ranks_by_condition_number(self):
        conds = {k: yawcal.solve(flight(k)).cond for k in ("circle", "figure_eight", "quarter", "narrow")}
        self.assertLess(conds["circle"], conds["figure_eight"])
        self.assertLess(conds["figure_eight"], conds["quarter"])
        self.assertLess(conds["quarter"], conds["narrow"])

    def test_reported_sigma_grows_with_noise(self):
        sigmas = [math.sqrt(yawcal.solve(flight("circle", noise=nz)).delta_var) for nz in (0.1, 0.3, 0.5)]
        self.assertLess(sigmas[0], sigmas[1])
        self.assertLess(sigmas[1], sigmas[2])

    def test_no_bias_from_noise_across_seeds(self):
        errors = [
            math.degrees(yawcal.wrap_pi(yawcal.solve(flight("circle", noise=0.5, seed=s)).delta - TRUE_DELTA))
            for s in range(20)
        ]
        self.assertAlmostEqual(sum(errors) / len(errors), 0.0, delta=0.2)

    def test_windless_air_is_also_solved(self):
        fit = yawcal.solve(flight("circle", wind=(0.0, 0.0)))
        self.assertIsNone(fit.reject)
        self.assertAlmostEqual(math.degrees(yawcal.wrap_pi(fit.delta - TRUE_DELTA)), 0.0, delta=1e-6)
        self.assertAlmostEqual(math.hypot(fit.wind_n, fit.wind_e), 0.0, delta=1e-6)

    def test_dead_pitot_still_yields_offset_and_wind(self):
        """No airspeed: feed IAS = 1 and let k absorb the (assumed constant)
        true airspeed. The offset and the wind stay observable."""
        samples = [s._replace(ias=1.0) for s in flight("circle")]
        fit = yawcal.solve(samples, ias_known=False)
        self.assertIsNone(fit.reject)
        self.assertAlmostEqual(math.degrees(yawcal.wrap_pi(fit.delta - TRUE_DELTA)), 0.0, delta=1e-6)
        self.assertAlmostEqual(fit.k, TRUE_K * TAS, delta=0.01)  # k is now the TAS itself
        self.assertAlmostEqual(fit.wind_n, TRUE_WIND[0], delta=0.01)

    def test_absurd_scale_is_rejected(self):
        """A pitot reading a tenth of the truth is a broken sensor, not a
        position error - the k band must not quietly accept it."""
        fit = yawcal.solve(flight("circle", k=8.0))
        self.assertEqual(fit.reject, "scale")

    def test_too_few_samples_or_too_short_a_span(self):
        self.assertIsNone(yawcal.solve(flight("circle")[:5]))
        dense = [s._replace(t=s.t / 100.0) for s in flight("circle")]  # 300 samples in 0.6 s
        self.assertIsNone(yawcal.solve(dense))

    def test_model_breakdown_shows_up_as_residual(self):
        """Wind that is not constant over the window is exactly what the model
        forbids; it must surface rather than be absorbed into the offset."""
        samples = []
        for i, s in enumerate(flight("circle")):
            gust = 6.0 * math.sin(i / 7.0)  # violent, incoherent with heading
            samples.append(s._replace(vn=s.vn + gust, ve=s.ve - gust))
        self.assertEqual(yawcal.solve(samples).reject, "residual")


class SampleGateTest(unittest.TestCase):
    """The gate is shared by the live link and the .cvlog analyser, so its
    rejection reasons are part of the contract."""

    GOOD = dict(
        t=0.0, psi=0.1, roll=0.0, pitch=0.0, ias=12.0, vn=10.0, ve=1.0,
        fix_type=3, hacc_m=2.0, yaw_rate=0.0,
    )

    def reason(self, **overrides):
        collector = yawcal.SampleCollector()
        collector.add(**{**self.GOOD, **overrides})
        return next(iter(collector.rejected), None)

    def test_good_sample_passes(self):
        self.assertIsNone(self.reason())

    def test_rejections(self):
        cases = {
            "no_fix": dict(fix_type=2),
            "hacc": dict(hacc_m=None),  # 0/unknown h_acc is not a trustworthy fix
            "nan": dict(ias=float("nan")),
            "slow_gs": dict(vn=1.0, ve=0.0),
            "slow_ias": dict(ias=3.0),
            "pitch": dict(pitch=math.radians(45.0)),
            "roll": dict(roll=math.radians(80.0)),
            "yaw_rate": dict(yaw_rate=math.radians(90.0)),
        }
        for expected, overrides in cases.items():
            with self.subTest(expected):
                self.assertEqual(self.reason(**overrides), expected)

    def test_hacc_worse_than_threshold(self):
        self.assertEqual(self.reason(hacc_m=20.0), "hacc")

    def test_held_gps_fields_are_not_resampled(self):
        """The air side ships 50 Hz telemetry carrying a 5 Hz GPS solution, so
        the velocity repeats verbatim between fixes. Taking it every frame
        would pair one GPS epoch with ten different attitudes."""
        collector = yawcal.SampleCollector()
        for i in range(10):
            collector.add(**{**self.GOOD, "t": i * 0.02})
        self.assertEqual(collector.accepted, 1)
        self.assertEqual(collector.rejected["duplicate"], 9)

    def test_window_drops_samples_older_than_window_s(self):
        collector = yawcal.SampleCollector()
        for i in range(1000):
            collector.add(**{**self.GOOD, "t": i * 0.2, "vn": 10.0 + i * 1e-3})
        self.assertLessEqual(len(collector), int(yawcal.WINDOW_S / 0.2) + 1)


class OffsetFilterTest(unittest.TestCase):
    def test_tracks_a_constant_offset(self):
        """Repeated agreeing measurements must beat a single one - but only
        at their own epoch: reading the filter later costs the extrapolation,
        which is the point of carrying a covariance at all."""
        f = yawcal.OffsetFilter()
        for i in range(10):
            f.update(i * 60.0, math.radians(20.0), math.radians(2.0) ** 2)
        offset, sigma = f.at(540.0)  # the epoch of the last measurement
        self.assertAlmostEqual(math.degrees(offset), 20.0, delta=0.5)
        self.assertLess(math.degrees(sigma), 2.0)
        self.assertGreater(math.degrees(f.at(540.0 + 300.0)[1]), 2.0)

    def test_learns_the_drift_rate_and_extrapolates(self):
        """A gyro bias makes the offset ramp; the filter has to follow it and
        keep following during a gap, which is the whole reason it carries a
        rate state instead of just averaging."""
        rate_dps = 1.0 / 60.0  # 1 deg/min
        f = yawcal.OffsetFilter()
        for i in range(40):
            t = i * 30.0
            f.update(t, math.radians(rate_dps * t), math.radians(2.0) ** 2)
        offset, _ = f.at(40 * 30.0 + 120.0)  # 2 min past the last measurement
        self.assertAlmostEqual(math.degrees(offset), rate_dps * (40 * 30.0 + 120.0), delta=1.0)

    def test_uncertainty_grows_while_unfed(self):
        f = yawcal.OffsetFilter()
        f.update(0.0, 0.0, math.radians(2.0) ** 2)
        self.assertLess(f.at(0.0)[1], f.at(600.0)[1])

    def test_coarse_seed_is_overridden_by_a_real_fit(self):
        f = yawcal.OffsetFilter()
        f.seed(0.0, math.radians(50.0))
        self.assertTrue(f.coarse)
        self.assertAlmostEqual(math.degrees(f.at(0.0)[0]), 50.0, places=6)
        f.update(1.0, math.radians(20.0), math.radians(2.0) ** 2)
        self.assertFalse(f.coarse)
        # the seed's 20 deg sigma must lose decisively against a 2 deg fit
        self.assertAlmostEqual(math.degrees(f.at(1.0)[0]), 20.0, delta=0.5)

    def test_seed_does_not_overwrite_an_existing_estimate(self):
        f = yawcal.OffsetFilter()
        f.update(0.0, math.radians(20.0), math.radians(2.0) ** 2)
        f.seed(1.0, math.radians(50.0))
        self.assertAlmostEqual(math.degrees(f.at(1.0)[0]), 20.0, delta=0.5)

    def test_wraps_across_the_discontinuity(self):
        """Offsets near +-180 deg must not average to zero."""
        f = yawcal.OffsetFilter()
        for i in range(10):
            f.update(i * 60.0, math.radians(179.0), math.radians(2.0) ** 2)
        f.update(600.0, math.radians(-179.0), math.radians(2.0) ** 2)
        self.assertGreater(abs(math.degrees(f.at(600.0)[0])), 170.0)

    def test_not_ready_before_any_estimate(self):
        f = yawcal.OffsetFilter()
        self.assertFalse(f.ready)
        self.assertIsNone(f.at(0.0))


class MeasurementVarTest(unittest.TestCase):
    def test_floor_applies_to_an_optimistic_fit(self):
        fit = yawcal.solve(flight("circle", noise=0.1))
        self.assertLess(fit.delta_var, yawcal.MEASUREMENT_SIGMA_FLOOR**2)
        self.assertEqual(yawcal.measurement_var(fit), yawcal.MEASUREMENT_SIGMA_FLOOR**2)


class CalibratorIntegrationTest(unittest.TestCase):
    """The whole chain on the emulator's own message stream: a scripted
    flight with a known wind and a known yaw offset, pushed through the
    processor exactly as TelemetryHub would.

    This is what stands in for a recorded flight until there is one - and it
    is why the emulator flies in wind (flight_profile.WIND_NE_MPS) and lies
    about its yaw (emulator_source.YAW_OFFSET_DEG).
    """

    RATE_HZ = 20.0

    def run_profile(self, seconds, offset_deg=None, wind=None):
        from pymavlink import mavutil

        from cockpitview import emulator_source, flight_profile

        offset_deg = emulator_source.YAW_OFFSET_DEG if offset_deg is None else offset_deg
        clock = [0.0]
        cal = yawcal.YawCalibrator(clock=lambda: clock[0])
        mav = mavutil.mavlink.MAVLink(None)
        patched = []

        original_offset = emulator_source.YAW_OFFSET_DEG
        original_wind = flight_profile.WIND_NE_MPS
        emulator_source.YAW_OFFSET_DEG = offset_deg
        if wind is not None:
            flight_profile.WIND_NE_MPS = wind
        try:
            for step in range(int(seconds * self.RATE_HZ)):
                t = step / self.RATE_HZ
                clock[0] = t
                roll, pitch, yaw, _, _, yawspeed = emulator_source.attitude_at(t)
                gps = flight_profile.gps_state(t)
                cal.process(mav.gps_raw_int_encode(
                    int(t * 1e6), 3, int(gps["lat_deg"] * 1e7), int(gps["lon_deg"] * 1e7),
                    int(gps["alt_m"] * 1000), 100, 100,
                    int(math.hypot(gps["vn"], gps["ve"]) * 100),
                    int(gps["cog_deg"] * 100) % 36000, 12,
                    0, emulator_source.H_ACC_MM, 3000, 200, 0,
                ))
                cal.process(mav.vfr_hud_encode(
                    flight_profile.AIRSPEED_MPS, math.hypot(gps["vn"], gps["ve"]),
                    int(round(math.degrees(yaw))) % 360, 60, gps["alt_m"], -gps["vd"],
                ))
                attitude = cal.process(mav.attitude_encode(
                    int(t * 1000), roll, pitch, yaw, 0.0, 0.0, yawspeed))
                cal.process(mav.global_position_int_encode(
                    int(t * 1000), int(gps["lat_deg"] * 1e7), int(gps["lon_deg"] * 1e7),
                    int(gps["alt_m"] * 1000), int(gps["relative_alt_m"] * 1000),
                    int(gps["vn"] * 100), int(gps["ve"] * 100), int(gps["vd"] * 100),
                    int(gps["cog_deg"] * 100) % 36000,
                ))
                patched.append((t, gps["heading_deg"], math.degrees(attitude.yaw) % 360.0))
        finally:
            emulator_source.YAW_OFFSET_DEG = original_offset
            flight_profile.WIND_NE_MPS = original_wind
        return cal, patched

    def test_recovers_the_emulated_offset_and_wind(self):
        from cockpitview import flight_profile

        cal, _ = self.run_profile(120.0)
        state = cal.state()
        self.assertTrue(state.calibrated)
        self.assertAlmostEqual(state.offset_deg, -37.0, delta=2.0)
        wind_n, wind_e = flight_profile.WIND_NE_MPS
        self.assertAlmostEqual(state.wind_mps, math.hypot(wind_n, wind_e), delta=0.5)
        expected_from = (math.degrees(math.atan2(wind_e, wind_n)) + 180.0) % 360.0
        self.assertAlmostEqual(state.wind_from_deg, expected_from, delta=10.0)
        self.assertAlmostEqual(state.ias_scale, 1.0, delta=0.05)

    def test_patched_attitude_becomes_the_true_heading(self):
        """The payoff: after the lock, ATTITUDE.yaw leaving the processor is
        the real compass heading, so the HUD tape, the flight path marker and
        QGC all get it without knowing this module exists."""
        _, patched = self.run_profile(150.0)
        late = [(true, shown) for t, true, shown in patched if t > 130.0]
        self.assertTrue(late)
        errors = [abs(yawcal.wrap_pi(math.radians(shown - true))) for true, shown in late]
        # the loop segment inverts the aircraft, where "heading" is not a
        # meaningful thing to compare - judge on the median
        errors.sort()
        self.assertLess(math.degrees(errors[len(errors) // 2]), 3.0)

    def test_untouched_before_any_estimate(self):
        """No estimate must mean no rewriting - a heading invented from
        nothing is worse than the honest boot-relative one."""
        cal, patched = self.run_profile(1.0)  # shorter than one straight leg
        self.assertIsNone(cal.state().offset_deg)
        self.assertFalse(cal.state().calibrated)
        for _, _, shown in patched:
            self.assertAlmostEqual(shown, (0.0 + 37.0) % 360.0, delta=0.5)

    def test_coarse_seed_is_wrong_by_the_drift_angle_and_says_so(self):
        """One straight leg cannot separate wind from datum, so the seed is
        off by exactly the drift angle. That is acceptable *because* it is
        reported as uncalibrated - the failure mode to avoid is showing it
        with the confidence of a real fit."""
        cal, _ = self.run_profile(4.0)  # one straight leg, no arc yet
        state = cal.state()
        self.assertTrue(state.coarse)
        self.assertFalse(state.calibrated)
        drift_deg = 18.435  # atan2(2.5, 10 + (-2.5)) for the emulated wind
        self.assertAlmostEqual(state.offset_deg, -37.0 + drift_deg, delta=1.0)

    def test_works_for_a_different_offset(self):
        cal, _ = self.run_profile(120.0, offset_deg=-100.0)
        self.assertAlmostEqual(cal.state().offset_deg, 100.0, delta=2.0)

    def test_works_in_still_air(self):
        cal, _ = self.run_profile(120.0, wind=(0.0, 0.0))
        self.assertAlmostEqual(cal.state().offset_deg, -37.0, delta=2.0)
        self.assertAlmostEqual(cal.state().wind_mps, 0.0, delta=0.5)


class LogReplayTest(unittest.TestCase):
    """The .cvlog analyser must reach the same answer as the live processor -
    it is the tool that will measure GPS_VELOCITY_LAG_S and DRIFT_PSD on the
    first real flight, so it has to be trustworthy before that flight."""

    RATE_HZ = 50.0  # the real telemetry rate; the GPS inside it updates at 5 Hz

    def write_log(self, seconds=150.0):
        """Builds a .cvlog byte-for-byte rather than driving FlightLog: the
        writer stamps records with its own clock, which would compress a
        150 s flight into the few milliseconds the loop takes to run."""
        import json
        import pathlib
        import tempfile

        from cockpitview import emulator_source, flight_profile, flightlog, protocol

        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        path = pathlib.Path(self._tmp.name) / "synthetic.cvlog"
        chunks = [flightlog.pack_record(
            0.0, flightlog.TYPE_HEADER,
            json.dumps(flightlog.build_header(0.0, {"synthetic": True}), default=str).encode(),
        )]
        held = None
        for step in range(int(seconds * self.RATE_HZ)):
            t = step / self.RATE_HZ
            roll, pitch, yaw, _, _, yawspeed = emulator_source.attitude_at(t)
            # the air side refreshes its GPS fields at 5 Hz and *holds* them
            # in between - reproduced here because dropping the repeats is
            # part of what the replay has to get right
            if held is None or step % int(self.RATE_HZ / 5) == 0:
                held = flight_profile.gps_state(t)
            gs = math.hypot(held["vn"], held["ve"])
            chunks.append(flightlog.pack_record(t, flightlog.TYPE_DOWN, protocol._TELEMETRY.pack(*protocol.Telemetry(
                msg_id=protocol.MSG_TELEMETRY, version=protocol.PROTOCOL_VERSION,
                flags=protocol.TELEM_FLAG_GPS_FIX | protocol.TELEM_FLAG_IMU_OK
                | protocol.TELEM_FLAG_PITOT_OK | protocol.TELEM_FLAG_BARO_OK,
                sats=12, time_boot_ms=int(t * 1000),
                roll=roll, pitch=pitch, yaw=yaw,
                rollspeed=0.0, pitchspeed=0.0, yawspeed=yawspeed,
                baro_alt_m=held["relative_alt_m"], climb_mps=-held["vd"],
                diff_press_pa=90.0, ias_mps=flight_profile.AIRSPEED_MPS,
                lat1e7=int(held["lat_deg"] * 1e7), lon1e7=int(held["lon_deg"] * 1e7),
                alt_msl_m=held["alt_m"], gs_mps=gs, cog_deg=held["cog_deg"],
                hdop=1.0, hacc_m=1.5, vacc_m=3.0,
                rssi_dbm=-70, cmd_loss_pct=0, tx_fail_pct=0,
                last_cmd_version=protocol.PROTOCOL_VERSION,
                cell1_mv=3800, cell2_mv=3790, cell3_mv=3810,
            ))))
        path.write_bytes(b"".join(chunks))
        return path

    def test_replay_converges_on_the_flown_offset(self):
        from cockpitview import emulator_source, flightlog

        path = self.write_log()
        hdr = flightlog.header(path)
        results = list(yawcal.replay(
            flightlog.telemetry_frames(path), lambda f: flightlog.flag_columns(hdr, f.flags)
        ))
        self.assertTrue(results)
        accepted = [fit for fit, _, _ in results if fit.reject is None]
        self.assertTrue(accepted, "no window was solvable in a flight full of turns")
        _, offset_deg, sigma_deg = results[-1]
        self.assertAlmostEqual(offset_deg, -emulator_source.YAW_OFFSET_DEG, delta=3.0)
        self.assertLess(sigma_deg, yawcal.CALIBRATED_SIGMA * 180.0 / math.pi)

    def test_replay_drops_the_held_gps_repeats(self):
        """50 Hz frames carrying a 5 Hz GPS solution must produce 5 samples
        per second, not 50 - otherwise the window holds ten copies of every
        epoch and every uncertainty it reports is too small by sqrt(10)."""
        from cockpitview import flightlog

        path = self.write_log(seconds=60.0)
        hdr = flightlog.header(path)
        collector = yawcal.SampleCollector()
        history = yawcal.AttitudeHistory()
        for t, f in flightlog.telemetry_frames(path):
            history.add(t, f.yaw, f.roll, f.pitch, f.yawspeed)
            past = history.at(t - yawcal.GPS_VELOCITY_LAG_S)
            if past is None:
                continue
            cog = math.radians(f.cog_deg)
            collector.add(t, past[0], past[1], past[2], f.ias_mps,
                          f.gs_mps * math.cos(cog), f.gs_mps * math.sin(cog),
                          3, f.hacc_m, past[3])
        self.assertGreater(collector.rejected["duplicate"], 9 * collector.accepted * 0.8)

    def test_cli_subcommand_runs(self):
        import contextlib
        import io

        from cockpitview import flightlog

        path = self.write_log()
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            self.assertEqual(flightlog.main(["yawcal", str(path)]), 0)
        text = out.getvalue()
        self.assertIn("windows accepted", text)
        self.assertIn("cond", text)

    def test_cli_on_a_log_with_nothing_to_fit(self):
        """A bench session (no flight) must say so rather than print an
        empty table or crash."""
        import contextlib
        import io
        import pathlib
        import tempfile

        from cockpitview import flightlog
        from tests.test_flightlog import telemetry_payload

        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        log = flightlog.FlightLog(pathlib.Path(tmp.name), args={})
        for _ in range(500):
            log.downlink(telemetry_payload())
        log.close()
        path = sorted(pathlib.Path(tmp.name).glob("*.cvlog"))[0]
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            self.assertEqual(flightlog.main(["yawcal", str(path)]), 0)
        self.assertIn("no window was ever solvable", out.getvalue())


if __name__ == "__main__":
    unittest.main()
