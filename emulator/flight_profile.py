"""
Synthetic flight profile for testing integration with QGroundControl.

Trajectory (~50 s, then repeats):
  1. straight flight          (12 s)
  2. 90-degree turn           ( 6 s)
  3. 90-degree turn back      ( 6 s)
  4. short straight           ( 4 s)
  5. loop                     (10 s)
  6. straight flight          (12 s)

Orientation is computed via quaternions (not directly via Euler angles), so
the "loop" (pitch passing through +-90 degrees) correctly projects onto
roll/pitch/yaw including the gimbal-lock flip, exactly like a real flight
controller / MPU-6050 after sensor fusion would report it.
"""

import math

import quaternion as quat

# Home position and airspeed (given)
HOME_LAT_DEG = 50.000000
HOME_LON_DEG = 15.000000
HOME_ALT_M = 400.0
AIRSPEED_MPS = 10.0  # horizontal flight speed, held constant along the body axis

_METERS_PER_DEG_LAT = 111_320.0

# (name, duration [s], kind, params)
#   "straight" - roll=0, pitch=0, yaw holds its current value
#   "turn"     - banks to "bank" degrees and back (sine), yaw changes by "yaw_delta" degrees
#   "loop"     - pitch sweeps through "pitch_delta" degrees (roll/yaw unchanged)
_SEGMENTS = [
    ("straight1",   3.0, "straight", {}),
    ("turn_right",   6.0, "turn",     {"bank": 25.0, "yaw_delta": 90.0}),
    ("straight2",    1.0, "straight", {}),
    ("turn_left",    6.0, "turn",     {"bank": -25.0, "yaw_delta": -90.0}),
    ("turn_right",   6.0, "turn",     {"bank": 25.0, "yaw_delta": 90.0}),
    ("straight2",    1.0, "straight", {}),
    ("turn_left",    6.0, "turn",     {"bank": -25.0, "yaw_delta": -90.0}),
    ("straight2",    4.0, "straight", {}),
    ("loop",        10.0, "loop",     {"pitch_delta": 360.0}),
]

TOTAL_DURATION = sum(dur for _, dur, _, _ in _SEGMENTS)


def _s_curve(u):
    """Smooth ramp 0 -> 1 with zero velocity at both ends (raised cosine)."""
    return u - math.sin(2 * math.pi * u) / (2 * math.pi)


def _build_phases():
    phases = []
    t_start = 0.0
    yaw_base = 0.0
    for name, dur, kind, params in _SEGMENTS:
        phases.append(
            {
                "name": name,
                "t_start": t_start,
                "t_end": t_start + dur,
                "dur": dur,
                "kind": kind,
                "yaw_base": yaw_base,
                **params,
            }
        )
        if kind == "turn":
            yaw_base += params["yaw_delta"]
        t_start += dur
    return phases


_PHASES = _build_phases()


def phase_at(t):
    """Returns (phase name, phase dict) for time t within one profile cycle."""
    t = t % TOTAL_DURATION
    for phase in _PHASES:
        if phase["t_start"] <= t < phase["t_end"]:
            return phase
    return _PHASES[-1]


def nominal_angles_deg(t):
    """Returns (roll_deg, pitch_deg, yaw_deg) - the "intended" Euler angles at time t."""
    phase = phase_at(t)
    u = (t % TOTAL_DURATION - phase["t_start"]) / phase["dur"]
    kind = phase["kind"]

    if kind == "straight":
        return 0.0, 0.0, phase["yaw_base"]

    if kind == "turn":
        roll = phase["bank"] * math.sin(math.pi * u)
        yaw = phase["yaw_base"] + phase["yaw_delta"] * _s_curve(u)
        return roll, 0.0, yaw

    if kind == "loop":
        pitch = phase["pitch_delta"] * _s_curve(u)
        return 0.0, pitch, phase["yaw_base"]

    raise ValueError(f"unknown phase kind: {kind}")


def _orientation_quat(t):
    roll_deg, pitch_deg, yaw_deg = nominal_angles_deg(t)
    return quat.from_euler_zyx(math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg))


def _velocity_ned(t):
    """Velocity in NED [m/s] - constant airspeed along the body axis, rotated by orientation."""
    q = _orientation_quat(t)
    return quat.rotate_vector(q, (AIRSPEED_MPS, 0.0, 0.0))


def _build_position_table(dt=0.01):
    """Precomputes position (north_m, east_m, down_m) over time for one profile
    cycle by numerically integrating velocity - only needs to run once at module import."""
    table = [(0.0, 0.0, 0.0, 0.0)]
    n = e = d = 0.0
    steps = int(round(TOTAL_DURATION / dt))
    t = 0.0
    for _ in range(steps):
        vn, ve, vd = _velocity_ned(t)
        n += vn * dt
        e += ve * dt
        d += vd * dt
        t += dt
        table.append((t, n, e, d))
    return table


_POSITION_TABLE = _build_position_table()
_POSITION_TABLE_DT = _POSITION_TABLE[1][0] - _POSITION_TABLE[0][0]
_CYCLE_DISPLACEMENT = _POSITION_TABLE[-1][1:]  # (north, east, down) after one profile cycle


def _position_within_cycle(u):
    idx = min(int(u / _POSITION_TABLE_DT), len(_POSITION_TABLE) - 2)
    t0, n0, e0, d0 = _POSITION_TABLE[idx]
    t1, n1, e1, d1 = _POSITION_TABLE[idx + 1]
    frac = 0.0 if t1 == t0 else (u - t0) / (t1 - t0)
    return (
        n0 + (n1 - n0) * frac,
        e0 + (e1 - e0) * frac,
        d0 + (d1 - d0) * frac,
    )


def position_ned(t):
    """Position (north_m, east_m, down_m) relative to HOME at time t (cycles
    accumulate, so the aircraft keeps progressing across repeats instead of
    circling in place)."""
    cycles = int(t // TOTAL_DURATION)
    u = t % TOTAL_DURATION
    n0, e0, d0 = _position_within_cycle(u)
    cn, ce, cd = _CYCLE_DISPLACEMENT
    return n0 + cycles * cn, e0 + cycles * ce, d0 + cycles * cd


def gps_state(t):
    """Returns a dict with GPS/telemetry data at time t:
    lat_deg, lon_deg, alt_m (AMSL), vn/ve/vd (m/s, NED), heading_deg."""
    north_m, east_m, down_m = position_ned(t)
    lat_deg = HOME_LAT_DEG + north_m / _METERS_PER_DEG_LAT
    lon_deg = HOME_LON_DEG + east_m / (_METERS_PER_DEG_LAT * math.cos(math.radians(HOME_LAT_DEG)))
    alt_m = HOME_ALT_M - down_m

    vn, ve, vd = _velocity_ned(t)
    # Actual (compass) heading derived from the velocity direction, not from the
    # "nominal" yaw parameter used to build the orientation - the latter doesn't
    # match the aircraft's actual nose direction during inverted flight (e.g.
    # the top of a loop).
    heading_deg = math.degrees(math.atan2(ve, vn)) % 360.0

    return {
        "lat_deg": lat_deg,
        "lon_deg": lon_deg,
        "alt_m": alt_m,
        "relative_alt_m": -down_m,
        "vn": vn,
        "ve": ve,
        "vd": vd,
        "heading_deg": heading_deg,
    }
