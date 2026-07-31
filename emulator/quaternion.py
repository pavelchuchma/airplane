"""Minimal quaternion math (no numpy) - just enough for this emulator.

A quaternion is represented as a tuple (w, x, y, z), Hamiltonian convention.
"""

import math


def from_axis_angle(axis, angle_rad):
    ax, ay, az = axis
    half = angle_rad / 2.0
    s = math.sin(half)
    return (math.cos(half), ax * s, ay * s, az * s)


def qx(angle_rad):
    return from_axis_angle((1.0, 0.0, 0.0), angle_rad)


def qy(angle_rad):
    return from_axis_angle((0.0, 1.0, 0.0), angle_rad)


def qz(angle_rad):
    return from_axis_angle((0.0, 0.0, 1.0), angle_rad)


def mult(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )


def conj(q):
    w, x, y, z = q
    return (w, -x, -y, -z)


def from_euler_zyx(roll_rad, pitch_rad, yaw_rad):
    """Aerospace convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)."""
    return mult(qz(yaw_rad), mult(qy(pitch_rad), qx(roll_rad)))


def to_euler_zyx(q):
    """Inverse conversion - returns (roll, pitch, yaw) in radians.

    For pitch outside +-90 degrees (e.g. during a loop) the expected
    gimbal-lock flip occurs (roll/yaw jump by 180 degrees) - exactly like a
    real flight controller using Euler angles would report it.
    """
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    sinp = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(sinp)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def rotate_vector(q, v):
    """Rotate vector v = (x, y, z) by quaternion q (active rotation, body -> world)."""
    vx, vy, vz = v
    result = mult(mult(q, (0.0, vx, vy, vz)), conj(q))
    return result[1], result[2], result[3]


def body_angular_rate(q1, q2, dt):
    """Approximate body-frame angular rate [rad/s] between q1 (time t) and
    q2 (time t+dt), i.e. exactly what an MPU-6050 gyroscope would measure."""
    dq = mult(conj(q1), q2)
    w, x, y, z = dq
    if w < 0:
        # take the shorter path (a quaternion and its negation represent the same rotation)
        x, y, z = -x, -y, -z
    return (2 * x / dt, 2 * y / dt, 2 * z / dt)
