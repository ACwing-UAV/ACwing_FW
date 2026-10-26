"""Small shared helpers (MATLAB: quat2rotmLocal.m, so3ToQuat.m, eulZYX)."""
from __future__ import annotations

import numpy as np


class Struct(dict):
    """``dict`` with attribute access, the Python stand-in for a MATLAB struct."""

    def __getattr__(self, name):
        try:
            return self[name]
        except KeyError as exc:
            raise AttributeError(name) from exc

    def __setattr__(self, name, value):
        self[name] = value

    def __repr__(self):
        items = ", ".join(f"{k}={_short(v)}" for k, v in self.items())
        return f"Struct({items})"


def _short(v):
    if isinstance(v, np.ndarray) and v.size > 6:
        return f"<array {v.shape}>"
    return repr(v)


def quat2rotm(q) -> np.ndarray:
    """Scalar-first quaternion ``[w x y z]`` -> rotation matrix (body -> world).

    Same as ``quat2rotmLocal.m``; the quaternion is normalised first.
    """
    q = np.asarray(q, dtype=float).reshape(4)
    q = q / np.linalg.norm(q)
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def rotm2quat(R) -> np.ndarray:
    """Rotation matrix -> ``[w x y z]`` with ``w >= 0`` (as ``so3_to_quat()`` in C)."""
    R = np.asarray(R, dtype=float)
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        q = [0.25 * S, (R[2, 1] - R[1, 2]) / S, (R[0, 2] - R[2, 0]) / S, (R[1, 0] - R[0, 1]) / S]
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        q = [(R[2, 1] - R[1, 2]) / S, 0.25 * S, (R[0, 1] + R[1, 0]) / S, (R[0, 2] + R[2, 0]) / S]
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        q = [(R[0, 2] - R[2, 0]) / S, (R[0, 1] + R[1, 0]) / S, 0.25 * S, (R[1, 2] + R[2, 1]) / S]
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        q = [(R[1, 0] - R[0, 1]) / S, (R[0, 2] + R[2, 0]) / S, (R[1, 2] + R[2, 1]) / S, 0.25 * S]
    q = np.asarray(q)
    q = q / np.linalg.norm(q)
    return -q if q[0] < 0 else q


def eul_zyx(R) -> np.ndarray:
    """Rotation matrix -> ``[roll, pitch, yaw]`` (ZYX sequence) in radians."""
    R = np.asarray(R, dtype=float)
    return np.array([
        np.arctan2(R[2, 1], R[2, 2]),
        np.arcsin(np.clip(-R[2, 0], -1.0, 1.0)),
        np.arctan2(R[1, 0], R[0, 0]),
    ])


def quat_angle_deg(q1, q2) -> float:
    """Angle between two attitudes given as quaternions [deg].

    Uses ``2*atan2(|v|, |w|)`` of the relative quaternion instead of
    ``2*acos(|q1.q2|)``: near 1, acos amplifies the float32 round-off of the
    firmware (~0.05 deg of spurious error).
    """
    q1 = np.asarray(q1, dtype=float).reshape(4)
    q2 = np.asarray(q2, dtype=float).reshape(4)
    w1, v1 = q1[0], q1[1:]
    w2, v2 = q2[0], q2[1:]
    w = w1 * w2 + v1 @ v2
    v = w1 * v2 - w2 * v1 - np.cross(v1, v2)
    return float(2.0 * np.arctan2(np.linalg.norm(v), abs(w)) * 180.0 / np.pi)
