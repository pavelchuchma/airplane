"""Yaw (heading) calibration against GPS - no magnetometer needed.

The air side integrates gyro-z and nothing else, so its yaw has an
arbitrary datum (the heading at boot becomes "north") that additionally
drifts with the residual gyro bias. GPS course over ground is *not* the
missing reference: it is the direction the aircraft *moves*, which differs
from where its nose points by the wind drift angle - 18 deg already at
5 m/s of wind and 15 m/s of flight. Feeding back COG - yaw would calibrate
the wind, not the gyro.

What is solvable is the whole wind triangle at once. With the airspeed
from the pitot, the ground velocity from the GPS and the (offset,
drifting) gyro yaw psi:

    v_ground = k * IAS * u(psi + delta) + wind

Substituting a = k*cos(delta), b = k*sin(delta) makes this *linear* in
(a, b, wind_n, wind_e):

    vn = IAS*(a*cos(psi) - b*sin(psi)) + wind_n
    ve = IAS*(a*sin(psi) + b*cos(psi)) + wind_e

so plain least squares over a window of samples yields the yaw offset
delta = atan2(b, a), the wind vector, and - a bonus worth having on its
own - k = hypot(a, b), the scale between indicated and true airspeed, i.e.
an in-flight measurement of the pitot's position error (doc/parts/
pitot-ms4525do.md).

The catch is observability: with every sample on the same heading the
system is singular, and no amount of averaging separates delta from the
wind. Flying an arc is what makes it solvable, so `solve` rejects a window
whose geometry does not support the split instead of returning a confident
looking number (`Fit.reject` says which test failed).

Two time scales are then kept apart. Wind is quasi-constant only over a
minute or so, which bounds the window length; the yaw offset drifts
linearly and slowly, which is what `OffsetFilter` tracks - a two-state
(offset, drift rate) Kalman filter fed by the window fits. Each fit is
applied at the *centroid* of its window, not at "now", otherwise a 60 s
window would lag the estimate by 30 s.

Layering, so that the same gate serves the live link and offline log
analysis (the mistake home.py avoids by construction - two copies of one
gate drift apart):

    solve()          pure math on a list of Samples
    AttitudeHistory  attitude read back at the instant the GPS solution is for
    SampleCollector  the gate + the sliding window, clock-free
    OffsetFilter     the (offset, drift rate) tracker
    YawCalibrator    a TelemetryHub processor wiring the four to MAVLink

MEASUREMENT STATUS: every constant below is provisional - derived from
synthetic flights and the emulator, never yet from an airframe. The two
that can only come from a real log are GPS_VELOCITY_LAG_S and DRIFT_PSD;
`python -m cockpitview.flightlog yawcal FILE.cvlog` exists to measure them.
"""

import collections
import math
import time

import numpy as np

# --- sample gate -------------------------------------------------------------

MIN_GROUNDSPEED_MPS = 5.0  # below this COG is noise
MIN_IAS_MPS = 8.0  # and the pitot sits in its own noise floor
MAX_HACC_M = 5.0  # same trust threshold as home.py
MAX_PITCH_DEG = 30.0  # the horizontal projection IAS*cos(pitch) degrades
MAX_ROLL_DEG = 60.0  # beyond a normal turn it is aerobatics, not a wind triangle
# Turns are where the observability comes from, so this is deliberately
# loose: it rejects tumbles, not circuits. What it does NOT have to absorb
# is the GPS solution lag - that is compensated explicitly below.
MAX_YAW_RATE_DPS = 45.0
# The receiver's velocity solution is filtered and therefore old by the time
# it reaches us. Compensated by pairing it with the attitude interpolated
# back this far. PROVISIONAL - measurable from a log by maximising the
# correlation of yawspeed against the derivative of COG through the turns.
GPS_VELOCITY_LAG_S = 0.15

# --- window ------------------------------------------------------------------

WINDOW_S = 60.0  # long enough for heading spread, short enough for constant wind
SOLVE_PERIOD_S = 1.0
MIN_SAMPLES = 30
MIN_SPAN_S = 15.0

# Observability, measured on synthetic windows (tests/test_yawcal.py keeps the
# numbers honest). The condition number of the column-equilibrated design
# matrix is the geometry gate - it is exactly the factor by which an error the
# model does not describe (a wind shift mid-window) is amplified into delta:
#
#   full 360 deg  1.0     figure eight  2.1     0-90 deg sweep  3.5
#   +-45 deg      4.4     +-20 deg      9.9     +-10 deg       19.8
#   straight      1e15 (singular - no wind/offset split exists)
#
# 10 admits everything from a gentle S-turn upwards. `concentration`
# (|mean(exp(i*psi))|, 1.0 = one heading) is kept as the human-readable
# diagnostic next to it, not as a second gate saying the same thing worse.
MAX_CONDITION = 10.0
MAX_RESIDUAL_MPS = 3.0  # turbulence + GPS velocity noise; above it the model does not hold
# Formal least-squares sigma reaches 0.1 deg over a 60 s window, which measures
# the GPS noise and nothing else. The model's own assumptions - wind constant
# over the window, k constant across the flown lift coefficients - are not good
# to anything like that, and a filter fed the optimistic number locks onto the
# first window and ignores the rest of the flight.
MEASUREMENT_SIGMA_FLOOR = math.radians(2.0)
# Sanity band for the IAS scale. The airframe is expected around 1.0-1.4 (the
# pitot reads low from position error), so this only catches a fit that ran away.
K_BOUNDS = (0.5, 2.5)
# Without a pitot the same fit still works with IAS := 1, if the true airspeed
# holds over the window; k then comes out as that airspeed in m/s.
K_BOUNDS_NO_IAS = (3.0, 60.0)

# --- offset filter -----------------------------------------------------------

# Random-walk PSD of the drift rate [rad^2/s^3]. PROVISIONAL: set so the rate
# can wander by roughly 1 deg/min over a minute, which is the order of an
# uncalibrated gyro-z bias. Measurable from a bench log (yaw drift at rest).
DRIFT_PSD = (math.radians(1.0) / 60.0) ** 2 / 60.0
INITIAL_RATE_SIGMA = math.radians(5.0) / 60.0  # 5 deg/min, 1 sigma, before any evidence
# A coarse seed ignores the wind, so its error *is* the drift angle.
COARSE_SIGMA = math.radians(20.0)
# ...and it must come from a leg long enough to be one. Expressed in seconds
# rather than samples because the rate differs by 4x between the real link
# (5 Hz GPS) and the emulator.
COARSE_SEED_S = 3.0
CALIBRATED_SIGMA = math.radians(10.0)  # above this the estimate is not worth showing as truth

Sample = collections.namedtuple("Sample", "t psi ias vn ve")

Fit = collections.namedtuple(
    "Fit",
    "t_start t_mid t_end delta k wind_n wind_e delta_var residual_rms concentration cond n reject",
)


def wrap_pi(rad):
    return (rad + math.pi) % (2 * math.pi) - math.pi


def solve(samples, ias_known=True):
    """Least-squares wind triangle over one window of Samples.

    Returns a Fit whose `reject` is None when every test passed, or the name
    of the first failed test (the numbers are still filled in - offline
    analysis wants to see *how* a window missed). Returns None when there is
    not even enough data to attempt the fit.
    """
    n = len(samples)
    if n < MIN_SAMPLES:
        return None
    t = np.fromiter((s.t for s in samples), float, n)
    span = float(t[-1] - t[0])
    if span < MIN_SPAN_S:
        return None

    psi = np.fromiter((s.psi for s in samples), float, n)
    ias = np.fromiter((s.ias for s in samples), float, n)
    vn = np.fromiter((s.vn for s in samples), float, n)
    ve = np.fromiter((s.ve for s in samples), float, n)
    cos_psi, sin_psi = np.cos(psi), np.sin(psi)

    concentration = float(math.hypot(cos_psi.mean(), sin_psi.mean()))

    m = np.zeros((2 * n, 4))
    m[0::2, 0] = ias * cos_psi
    m[0::2, 1] = -ias * sin_psi
    m[0::2, 2] = 1.0
    m[1::2, 0] = ias * sin_psi
    m[1::2, 1] = ias * cos_psi
    m[1::2, 3] = 1.0
    y = np.empty(2 * n)
    y[0::2] = vn
    y[1::2] = ve

    # numpy 2.2 on Apple's Accelerate BLAS raises spurious FP flags out of
    # matmul - even `np.ones((600, 4)) @ np.ones(4)` warns about division by
    # zero. Silencing them across the whole linear-algebra block costs
    # nothing: every quantity it produces is explicitly checked for
    # finiteness below before it is allowed to mean anything.
    with np.errstate(all="ignore"):
        # equilibrate the columns before asking about conditioning, otherwise
        # the number just reports that airspeed is ~10 and the wind columns 1
        norms = np.linalg.norm(m, axis=0)
        cond = float(np.linalg.cond(m / norms)) if norms.all() else math.inf
        x, *_ = np.linalg.lstsq(m, y, rcond=None)
        residual = y - m @ x
        ssr = float(residual @ residual)
    a, b, wind_n, wind_e = (float(v) for v in x)
    residual_rms = math.sqrt(ssr / (2 * n))
    sigma2 = ssr / (2 * n - 4)

    k = math.hypot(a, b)
    delta = math.atan2(b, a)

    # var(delta) propagated through the covariance of (a, b). Only attempted
    # once the geometry is known to be sound: inverting the normal matrix of a
    # near-singular window does not raise, it quietly returns garbage that can
    # come back as a *small* variance, which is the one failure mode that
    # would slip a meaningless offset past the filter.
    delta_var = math.inf
    if math.isfinite(cond) and cond <= MAX_CONDITION:
        try:
            with np.errstate(all="ignore"):
                cov = sigma2 * np.linalg.inv(m.T @ m)
                jac = np.array([-b, a]) / max(a * a + b * b, 1e-12)
                delta_var = float(jac @ cov[:2, :2] @ jac)
            if not math.isfinite(delta_var) or delta_var < 0.0:
                delta_var = math.inf
        except np.linalg.LinAlgError:
            delta_var = math.inf

    k_lo, k_hi = K_BOUNDS if ias_known else K_BOUNDS_NO_IAS
    reject = None
    if not math.isfinite(cond) or cond > MAX_CONDITION:
        reject = "geometry"
    elif not math.isfinite(delta_var):
        reject = "singular"
    elif residual_rms > MAX_RESIDUAL_MPS:
        reject = "residual"
    elif not k_lo <= k <= k_hi:
        reject = "scale"

    return Fit(
        t_start=float(t[0]),
        t_mid=float(t.mean()),
        t_end=float(t[-1]),
        delta=delta,
        k=k,
        wind_n=wind_n,
        wind_e=wind_e,
        delta_var=delta_var,
        residual_rms=residual_rms,
        concentration=concentration,
        cond=cond,
        n=n,
        reject=reject,
    )


class AttitudeHistory:
    """Short ring of attitude samples, read back at an arbitrary instant.

    Exists because a GPS velocity has to be paired with the attitude that was
    true when the receiver computed it, not when the frame arrived. Both the
    live processor and the .cvlog replay go through this one copy, so an
    offline analysis of a flight cannot disagree with what the HUD did during
    it - which is the whole point of measuring GPS_VELOCITY_LAG_S offline and
    then trusting it in the air.
    """

    def __init__(self, keep_s=2.0):
        self._keep_s = keep_s
        self._entries = collections.deque()
        self._psi = None

    def add(self, t, psi, roll, pitch, yaw_rate):
        """`psi` may arrive wrapped (a quaternion-derived yaw) or unbounded
        (the air side never wraps its integration); it is unwrapped here so
        interpolation never crosses a seam."""
        if self._psi is None:
            self._psi = psi
        else:
            self._psi += wrap_pi(psi - self._psi)
        self._entries.append((t, self._psi, roll, pitch, yaw_rate))
        while self._entries and t - self._entries[0][0] > self._keep_s:
            self._entries.popleft()

    def at(self, t):
        """(psi, roll, pitch, yaw_rate) linearly interpolated at t, or None
        when the history does not bracket it."""
        entries = self._entries
        if len(entries) < 2 or t < entries[0][0]:
            return None
        prev = entries[0]
        for entry in entries:
            if entry[0] >= t:
                t0, psi0, roll0, pitch0, rate0 = prev
                t1, psi1, roll1, pitch1, rate1 = entry
                frac = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
                return tuple(v0 + (v1 - v0) * frac for v0, v1 in
                             ((psi0, psi1), (roll0, roll1), (pitch0, pitch1), (rate0, rate1)))
            prev = entry
        return None  # t is newer than anything recorded


def measurement_var(fit):
    """Variance to hand the filter: the least-squares covariance, floored at
    what the model itself can be trusted to (see MEASUREMENT_SIGMA_FLOOR)."""
    return max(fit.delta_var, MEASUREMENT_SIGMA_FLOOR**2)


class SampleCollector:
    """The gate and the sliding window, driven by explicit timestamps so the
    live link and a replayed .cvlog run through exactly the same code."""

    def __init__(self, ias_known=True):
        self._ias_known = ias_known
        self._samples = collections.deque()
        self._last_solve_t = -math.inf
        self._last_velocity = None
        self.accepted = 0
        self.rejected = collections.Counter()

    def __len__(self):
        return len(self._samples)

    def add(self, t, psi, roll, pitch, ias, vn, ve, fix_type, hacc_m, yaw_rate):
        """Offers one GPS epoch. Returns a Fit when a solve was due (check
        `.reject`), else None. `ias` may be 1.0 with ias_known=False."""
        reason = self._gate(psi, roll, pitch, ias, vn, ve, fix_type, hacc_m, yaw_rate)
        if reason is not None:
            self.rejected[reason] += 1
            return None
        self.accepted += 1
        self._samples.append(Sample(t, psi, ias, vn, ve))
        while self._samples and t - self._samples[0].t > WINDOW_S:
            self._samples.popleft()

        if t - self._last_solve_t < SOLVE_PERIOD_S:
            return None
        fit = solve(list(self._samples), ias_known=self._ias_known)
        if fit is not None:
            self._last_solve_t = t
        return fit

    def _gate(self, psi, roll, pitch, ias, vn, ve, fix_type, hacc_m, yaw_rate):
        if fix_type < 3:
            return "no_fix"
        if hacc_m is None or hacc_m > MAX_HACC_M:
            # no error estimate means no reason to trust the fix - same rule
            # as home.py, and it is what keeps a zero-filled h_acc out
            return "hacc"
        if not all(map(math.isfinite, (psi, roll, pitch, ias, vn, ve, yaw_rate))):
            return "nan"
        if math.hypot(vn, ve) < MIN_GROUNDSPEED_MPS:
            return "slow_gs"
        if self._ias_known and ias < MIN_IAS_MPS:
            return "slow_ias"
        if abs(math.degrees(pitch)) > MAX_PITCH_DEG:
            return "pitch"
        if abs(math.degrees(roll)) > MAX_ROLL_DEG:
            return "roll"
        if abs(math.degrees(yaw_rate)) > MAX_YAW_RATE_DPS:
            return "yaw_rate"
        # The air side samples the GPS at 5 Hz but ships telemetry at 50 Hz,
        # so the position/velocity fields are *held* between fixes. Taking
        # them every frame would pair one GPS epoch with ten different
        # attitudes, nine of them wrong by up to 200 ms of turn. Held fields
        # are bit-identical, so dropping repeats keeps exactly the frame in
        # which each new solution arrived.
        velocity = (vn, ve)
        if velocity == self._last_velocity:
            return "duplicate"
        self._last_velocity = velocity
        return None


class OffsetFilter:
    """Two-state (offset, drift rate) Kalman filter over the window fits.

    The state epoch follows the *centroid* of the last accepted window; the
    public reading extrapolates from there to the requested time, so a long
    window costs accuracy in the extrapolation (visible as a growing sigma)
    rather than a silent lag.
    """

    def __init__(self, drift_psd=DRIFT_PSD):
        self._q = drift_psd
        self._t = None
        self._x = None
        self._p = None
        self.coarse = True  # True until a wind-triangle fit has been applied

    @property
    def ready(self):
        return self._t is not None

    def seed(self, t, delta, sigma=COARSE_SIGMA):
        """Rough datum from a straight-flight COG - yaw, which is wrong by the
        drift angle. Large sigma on purpose: the first real fit overrides it."""
        if self._t is not None:
            return
        self._t = t
        self._x = np.array([wrap_pi(delta), 0.0])
        self._p = np.diag([sigma**2, INITIAL_RATE_SIGMA**2])

    def update(self, t, delta, var):
        if self._t is None:
            self._t = t
            self._x = np.array([wrap_pi(delta), 0.0])
            self._p = np.diag([var, INITIAL_RATE_SIGMA**2])
            self.coarse = False
            return
        x, p = self._predict(t)
        self._t = max(self._t, t)
        h = np.array([1.0, 0.0])
        s = float(h @ p @ h) + var
        gain = (p @ h) / s
        self._x = x + gain * wrap_pi(delta - x[0])
        self._x[0] = wrap_pi(self._x[0])
        self._p = (np.eye(2) - np.outer(gain, h)) @ p
        self.coarse = False

    def _predict(self, t):
        dt = max(0.0, t - self._t)
        f = np.array([[1.0, dt], [0.0, 1.0]])
        q = self._q * np.array([[dt**3 / 3.0, dt**2 / 2.0], [dt**2 / 2.0, dt]])
        return f @ self._x, f @ self._p @ f.T + q

    def at(self, t):
        """(offset_rad, sigma_rad) extrapolated to t, or None before the first
        estimate exists."""
        if self._t is None:
            return None
        x, p = self._predict(t)
        return wrap_pi(float(x[0])), math.sqrt(max(0.0, float(p[0, 0])))


def replay(frames, flags_of, lag_s=GPS_VELOCITY_LAG_S, ias_known=True):
    """Runs logged telemetry through the live gate, window and filter.

    `frames` yields (t, protocol.Telemetry) - flightlog.telemetry_frames() -
    and `flags_of(frame)` returns its decoded flag dict. Yields
    (fit, offset_deg, sigma_deg) per solved window, so a log tells you not
    only what the calibration converged to but which windows it refused and
    why (`fit.reject`).

    Deliberately not a second implementation of anything: the gate, the
    window and the filter are the objects the live processor uses, wired to a
    log instead of to MAVLink.
    """
    history = AttitudeHistory()
    collector = SampleCollector(ias_known=ias_known)
    offsets = OffsetFilter()
    fed_until = -math.inf
    for t, f in frames:
        history.add(t, f.yaw, f.roll, f.pitch, f.yawspeed)
        past = history.at(t - lag_s)
        if past is None:
            continue
        psi, roll, pitch, yaw_rate = past
        flags = flags_of(f)
        # the log carries the polar form the wire uses; the fit wants components
        cog, gs = math.radians(f.cog_deg), f.gs_mps
        vn, ve = gs * math.cos(cog), gs * math.sin(cog)
        ias = f.ias_mps if ias_known else 1.0
        fit = collector.add(
            t, psi, roll, pitch, ias, vn, ve,
            3 if flags.get("gps_fix") else 0, f.hacc_m, yaw_rate,
        )
        if fit is None:
            continue
        if fit.reject is None and fit.t_start >= fed_until:
            fed_until = fit.t_end
            offsets.update(fit.t_mid, fit.delta, measurement_var(fit))
        estimate = offsets.at(t)
        yield fit, (None if estimate is None else math.degrees(estimate[0])), (
            None if estimate is None else math.degrees(estimate[1])
        )


CalState = collections.namedtuple(
    "CalState", "calibrated coarse offset_deg sigma_deg wind_from_deg wind_mps ias_scale"
)


class YawCalibrator:
    """TelemetryHub processor: estimates the yaw offset and rewrites the
    heading in the stream so the HUD, QGC and the flight path marker all see
    a true heading instead of a boot-relative one.

    Until an estimate exists the messages pass through untouched - the tape
    then still shows the boot datum (useful for relative turns) and the HUD's
    WND line reports the calibration as absent, rather than the number
    silently pretending to be true north.
    """

    def __init__(self, log=None, clock=time.monotonic, ias_known=True):
        self._log = log
        self._clock = clock
        self._collector = SampleCollector(ias_known=ias_known)
        self._filter = OffsetFilter()
        self._attitude = AttitudeHistory()
        self._ias = math.nan
        self._fix_type = 0
        self._hacc_m = None
        self._straight = []  # coarse seed: (cog - psi) while straight and fast
        self._last_fit = None
        self._fed_until = -math.inf  # end of the last window fed to the filter
        self._announced = False

    # --- processor chain ---

    def process(self, msg):
        mtype = msg.get_type()
        if mtype == "ATTITUDE":
            self._on_attitude(msg)
            return self._patch_attitude(msg)
        if mtype == "VFR_HUD":
            self._ias = msg.airspeed
            return self._patch_vfr_hud(msg)
        if mtype == "GPS_RAW_INT":
            self._fix_type = msg.fix_type
            h_acc = getattr(msg, "h_acc", 0)  # mm, 0 = unknown
            self._hacc_m = h_acc / 1000.0 if h_acc else None
        elif mtype == "GLOBAL_POSITION_INT":
            self._on_position(msg)
        return msg

    def _on_attitude(self, msg):
        self._attitude.add(self._clock(), msg.yaw, msg.roll, msg.pitch, msg.yawspeed)

    def _on_position(self, msg):
        t = self._clock()
        past = self._attitude.at(t - GPS_VELOCITY_LAG_S)
        if past is None:
            return
        psi, roll, pitch, yaw_rate = past
        vn, ve = msg.vx / 100.0, msg.vy / 100.0
        fit = self._collector.add(
            t, psi, roll, pitch, self._ias, vn, ve, self._fix_type, self._hacc_m, yaw_rate
        )
        self._seed_coarse(t, psi, roll, vn, ve, yaw_rate)
        if fit is None or fit.reject is not None:
            return
        self._last_fit = fit  # wind/scale readout wants the freshest solve
        if fit.t_start < self._fed_until:
            # Windows are solved every second but each is 60 s long, so
            # consecutive fits share 59/60 of their samples. Feeding all of
            # them would let the filter mistake one measurement for sixty and
            # collapse its covariance onto the first window it ever saw. Only
            # windows disjoint from the last one fed carry new information.
            return
        self._fed_until = fit.t_end
        self._filter.update(fit.t_mid, fit.delta, measurement_var(fit))
        self._announce(fit)

    def _seed_coarse(self, t, psi, roll, vn, ve, yaw_rate):
        """One straight, fast leg gives a datum good to the drift angle, which
        beats "north is wherever it booted" while the aircraft has not yet
        flown an arc for the real fit."""
        if self._filter.ready:
            self._straight.clear()
            return
        speed = math.hypot(vn, ve)
        straight = (
            self._fix_type >= 3
            and self._hacc_m is not None
            and self._hacc_m <= MAX_HACC_M
            and speed >= MIN_GROUNDSPEED_MPS
            and abs(math.degrees(roll)) < 10.0
            and abs(math.degrees(yaw_rate)) < 3.0
        )
        if not straight:
            self._straight.clear()
            return
        self._straight.append((t, wrap_pi(math.atan2(ve, vn) - psi)))
        if t - self._straight[0][0] < COARSE_SEED_S:
            return
        # circular mean over the leg - a median is overkill for samples
        # spanning a few degrees, and this handles the +-180 seam for free
        c = sum(math.cos(d) for _, d in self._straight)
        s = sum(math.sin(d) for _, d in self._straight)
        self._filter.seed(t, math.atan2(s, c))
        self._straight.clear()
        print("Yaw datum seeded from straight flight (coarse - wind not yet separated)")

    def _announce(self, fit):
        if self._announced:
            return
        self._announced = True
        wind = math.hypot(fit.wind_n, fit.wind_e)
        print(
            f"Yaw calibrated: offset {math.degrees(fit.delta):+.1f} deg, "
            f"wind {wind:.1f} m/s, IAS scale {fit.k:.3f}"
        )
        if self._log is not None:
            self._log.event(
                "yaw_calibrated",
                offset_deg=math.degrees(fit.delta),
                wind_n=fit.wind_n,
                wind_e=fit.wind_e,
                ias_scale=fit.k,
                residual_rms=fit.residual_rms,
                samples=fit.n,
            )

    # --- message rewriting ---

    def _offset(self):
        return self._filter.at(self._clock())

    def _patch_attitude(self, msg):
        estimate = self._offset()
        if estimate is None:
            return msg
        msg.yaw = wrap_pi(msg.yaw + estimate[0])
        return msg

    def _patch_vfr_hud(self, msg):
        estimate = self._offset()
        if estimate is None:
            return msg
        msg.heading = int(round(math.degrees(estimate[0]) + msg.heading)) % 360
        return msg

    # --- readout ---

    def state(self):
        """CalState for the HUD; `calibrated` is False while the estimate is
        only a coarse seed or its sigma has grown past the point of meaning."""
        estimate = self._offset()
        if estimate is None:
            return CalState(False, True, None, None, None, None, None)
        offset, sigma = estimate
        fit = self._last_fit
        wind_from_deg = wind_mps = ias_scale = None
        if fit is not None:
            wind_mps = math.hypot(fit.wind_n, fit.wind_e)
            # meteorological convention: the direction the wind blows *from*
            wind_from_deg = (math.degrees(math.atan2(fit.wind_e, fit.wind_n)) + 180.0) % 360.0
            ias_scale = fit.k
        return CalState(
            calibrated=not self._filter.coarse and sigma <= CALIBRATED_SIGMA,
            coarse=self._filter.coarse,
            offset_deg=math.degrees(offset),
            sigma_deg=math.degrees(sigma),
            wind_from_deg=wind_from_deg,
            wind_mps=wind_mps,
            ias_scale=ias_scale,
        )
