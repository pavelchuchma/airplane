# Yaw calibration against GPS (no magnetometer)

Status: **implemented in cocpitView, validated synthetically and on the
emulator, never yet flown.** What is left is measurement, not code — see
"On the airframe" below.

## Problem

The GY-91 turned out to carry an MPU-6500, so there is no magnetometer. Yaw
is `yaw += gz*dt` and nothing else (`esp32/src/air/main.cpp`), which means:

- the datum is whatever heading the aircraft happened to boot on, and
- it drifts from there with the residual gyro-z bias.

The HUD heading tape, the flight path marker's drift angle and QGC's heading
arrow all read off it.

## Why COG − yaw is the wrong fix

GPS course over ground is the direction the aircraft *moves*. Its nose points
somewhere else, off by the wind drift angle:

    drift = asin(V_wind_crosswind / V_air)

which is 18° at 5 m/s of wind and 15 m/s of flight — not a small correction,
and it changes with every turn. A feedback loop on `COG − yaw` calibrates the
wind, not the gyro. Averaging only helps if the flight is uniformly
distributed over all headings, which a real circuit is not.

## What is solvable: the whole wind triangle

    v_ground = k · IAS · u(ψ + Δ) + wind

with ψ the gyro yaw, Δ the unknown offset, `k` the unknown IAS→TAS scale and
`wind` unknown. Substituting `a = k·cosΔ`, `b = k·sinΔ`:

    vn = IAS·(a·cos ψ − b·sin ψ) + wind_n
    ve = IAS·(a·sin ψ + b·cos ψ) + wind_e

**linear** in `(a, b, wind_n, wind_e)` — two rows per sample, ordinary least
squares, no iteration and no Jacobian. Then `Δ = atan2(b, a)`,
`k = hypot(a, b)`.

Three outputs, all wanted:

| output | use |
| --- | --- |
| `Δ` | the calibration itself |
| `wind` | HUD readout, useful in its own right before landing |
| `k` | the pitot's **position error**, measured in flight — see `doc/parts/pitot-ms4525do.md`, where it is otherwise a two-reciprocal-legs exercise |

`IAS` enters as `ias_mps · cos(pitch)`; without that, climbs and descents leak
into `k`.

## Observability

With every sample on one heading the system is singular: the offset and the
wind are the same unknown. The condition number of the column-equilibrated
design matrix says exactly how well a window separates them (measured, see
`tests/test_yawcal.py`):

| geometry | cond | concentration |
| --- | --- | --- |
| full 360° | 1.0 | 0.00 |
| figure eight | 2.1 | 0.63 |
| 0–90° sweep (the emulator profile) | 3.5 | 0.85 |
| ±45° | 4.4 | 0.90 |
| ±20° | 9.9 | 0.98 |
| ±10° | 19.8 | 0.995 |
| straight | 10¹⁵ | 1.00 |

`MAX_CONDITION = 10` admits everything from a gentle S-turn upwards. A window
above it is **rejected**, not solved with a wide error bar — the failure mode
worth avoiding is a confident-looking number from a geometry that cannot
produce one. The condition number is also literally the amplification factor
applied to any error the model does not describe (a wind shift mid-window),
which is why it, and not a sample count or window length, is the gate.

## Two time scales

Wind is quasi-constant over a minute at most; the offset drifts slowly and
almost linearly. So:

- window = 60 s, solved every 1 s
- a two-state (offset, drift rate) Kalman filter across windows

Two details that are easy to get wrong and are covered by tests:

1. **Each fit is applied at the centroid of its window**, not at "now" —
   otherwise a 60 s window lags the estimate by 30 s. The filter's epoch
   follows the centroid and the public reading extrapolates forward, so the
   cost of a long window shows up as a growing sigma instead of a silent lag.
2. **Only windows disjoint from the last one fed reach the filter.**
   Consecutive 60 s windows solved every second share 59/60 of their samples;
   feeding all of them lets the filter mistake one measurement for sixty and
   collapse its covariance onto the first window it ever saw.

The formal least-squares sigma reaches 0.1° over a 60 s window, which measures
the GPS noise and nothing else — the model's own assumptions (constant wind,
constant `k` across the flown lift coefficients) are not good to anything like
that. Hence `MEASUREMENT_SIGMA_FLOOR = 2°`.

## Sampling

The air side refreshes its GPS fields at 5 Hz but ships telemetry at 50 Hz,
**holding** them in between. Taking a sample per frame would pair one GPS
epoch with ten different attitudes, nine of them wrong by up to 200 ms of
turn. Held fields are bit-identical, so dropping repeated `(vn, ve)` keeps
exactly the frame each new solution arrived in. On top of that, the GPS
velocity solution is itself filtered and therefore old: the sample is paired
with the attitude interpolated back by `GPS_VELOCITY_LAG_S`.

Gates: 3D fix, `hacc ≤ 5 m` (the same trust rule as `home.py` — a zero/unknown
`h_acc` is not a trustworthy fix), `gs ≥ 5 m/s`, `IAS ≥ 8 m/s`, `|pitch| ≤ 30°`,
`|roll| ≤ 60°`, `|yaw rate| ≤ 45°/s`. The yaw-rate gate is deliberately loose:
turns are where the observability comes from, so it rejects tumbles, not
circuits — the GPS lag is compensated explicitly rather than gated around.

## Degradation

- **No arc flown yet** — after one straight, fast leg a coarse seed takes
  `COG − ψ`, which is wrong by exactly the drift angle (≤ ~20°). Better than
  "north is wherever it booted", and reported as *not* calibrated so it is
  never mistaken for a fix. The first real window overrides it.
- **Dead pitot** — feed `IAS = 1`. If the true airspeed holds over the window,
  Δ and the wind stay observable and `k` comes out as that airspeed in m/s.
- **No estimate at all** — nothing is rewritten and the HUD boxes the heading
  as `~037`. The boot datum is still worth reading for relative turns.
- **Long straight cruise** — no new windows, the filter's sigma grows through
  the drift PSD, and `calibrated` drops once it passes `CALIBRATED_SIGMA`.

## Where it runs

In cocpitView, as a `TelemetryHub` processor, last in the chain. It rewrites
the heading in ATTITUDE/VFR_HUD, so the HUD tape, the flight path marker and
QGC all get a true heading without knowing the module exists. Consequences:

- the wire protocol is untouched — no `PROTOCOL_VERSION` bump;
- the air side keeps integrating raw yaw, so a log always contains the
  uncalibrated truth and the calibration can be re-derived offline with
  different constants;
- it can be developed and tuned against `.cvlog` files without flashing.

If the air side ever needs a true heading for an autopilot, the algorithm
moves there — every input it needs is already on board.

## Validation without a flight

Ground truth is generated, not measured: a synthetic flight is built from a
known offset, wind and `k`, and the estimator has to return exactly those.
That is stronger than a recorded flight, where the truth is only approximately
known. Covered: recovery under noise, no bias across seeds, still air,
rejection of straight flight and of narrow arcs, the dead-pitot fallback, an
absurd `k`, and a mid-window wind shift surfacing as residual rather than
being absorbed into the offset.

The emulator is the end-to-end bed: it now flies in wind
(`flight_profile.WIND_NE_MPS`) and reports a deliberately wrong yaw
(`emulator_source.YAW_OFFSET_DEG`), because a windless emulator makes
`COG == heading`, the one case where the naive approach is trivially right —
it would "pass" without ever exercising the wind triangle. It also now sends a
realistic `h_acc`; the MAVLink default of 0 means "unknown", which kept both
`home.py` and this from ever arming in an emulated session.

## On the airframe — what is still unmeasured

Every constant in `yawcal.py` is provisional. Two can only come from a real
log, and `python -m cockpitview.flightlog yawcal FILE.cvlog` exists to
measure them (it replays a log through the same gate, window and filter
objects the live path uses, so an offline answer cannot disagree with what the
HUD did):

1. **`GPS_VELOCITY_LAG_S`** — re-run the analyser over a range of lag values
   and take the one minimising `rms`. A wrong lag shows up as residual
   concentrated in the turns.
2. **`DRIFT_PSD`** — the slope of `offset` across a long flight is the gyro
   drift rate; a bench log at rest bounds how much that rate itself wanders.

Then cross-check:

- **`k` against the reciprocal-legs method** (`doc/parts/pitot-ms4525do.md`).
  Agreement makes `k` the cheaper standing measurement of position error;
  disagreement means the constant-`k` assumption breaks across lift
  coefficients, which is itself worth knowing.
- **whether a normal circuit produces enough heading spread**, or whether a
  deliberate calibration turn after launch is needed. The `cond` column
  answers this directly.
- **residual RMS** should sit at GPS velocity noise plus turbulence. Much
  above that means the model (constant wind over 60 s) does not hold and the
  window wants shortening.

Not done and deliberately out of scope here: using the estimated wind for
anything beyond the readout, and any air-side port.
