"""Synthetic telemetry stream (INIT, STATUS, STATE) for tests and demos without the board.

The board rotates with a smooth, known angular rate; the frames carry the true
attitude and what the firmware would log with ``MAG_AXIS_MAP`` = identity and
``MAG_HARD_IRON`` = 0 (i.e. the raw sensor field), so that
``calib_mag_axes`` must recover the given axis map and hard-iron.

    python -c "from iekf_tools.simulate import write_stream; write_stream('sim.bin')"
    python -m iekf_tools viewer --replay sim.bin
"""
from __future__ import annotations

import struct

import numpy as np

from .common import rotm2quat
from .filters.original import exp_so3
from .telemetry import TYPE_INIT, TYPE_STATE, TYPE_STATUS, encode_frame

FS = 240.0


def pack_state(seq, t_us, q, bias, acc, gyro, mag, flags) -> bytes:
    """``tlm_state_t`` (71 bytes)."""
    return struct.pack("<HI16fB", int(seq) & 0xFFFF, int(t_us) & 0xFFFFFFFF,
                       *q, *bias, *acc, *gyro, *mag, int(flags))


def pack_init(q0, g_ref, m_ref, b0, fs=FS, flags=0x13) -> bytes:
    """``tlm_init_t`` (137 bytes) with the firmware defaults."""
    f = list(q0) + list(g_ref) + list(m_ref) + list(b0)
    f += [0.01, 0.01, np.deg2rad(0.5), 1e-4,          # sigmaAcc, sigmaMag, sigmaGyro, sigmaBias
          1.0, 45.0, fs, 0.08, 0.08, 1e-2, 1e-4,      # nomAcc, nomMag, fs, tols, p0
          4.0, 0.0, float(np.exp(-1 / (fs * 0.5))), np.deg2rad(5),
          100.0, np.deg2rad(10) ** 2, float(np.exp(-1 / fs)), 0.6, 0.02]
    return struct.pack("<33fHHB", *f, 120, 60, flags)


def pack_status(uptime_ms, n_samples, running=1, calib=100) -> bytes:
    """``tlm_status_t`` (54 bytes)."""
    return struct.pack("<12IHHbB", int(uptime_ms), 4 * n_samples, 0, 0, 0, 0, 30000,
                       4 * n_samples, 4 * n_samples, n_samples // 2, 0, 0, 5, calib, 0, running)


def synthetic_stream(duration=40.0, axis_map=None, hard_iron=(30.0, -12.0, 55.0),
                     field=(20.0, 0.0, -40.0), gyro_bias=(0.01, -0.005, 0.003), seed=1) -> bytes:
    """Returns the bytes of a telemetry stream (see the module docstring).

    ``axis_map``: ``mag_body = M mag_sensor`` (default: 180 deg about x, as on
    the IKS4A1). ``hard_iron``: offset in the body frame [uT].
    """
    rng = np.random.default_rng(seed)
    M = np.diag([1.0, -1.0, -1.0]) if axis_map is None else np.asarray(axis_map, float)
    h = np.asarray(hard_iron, float)
    B = np.asarray(field, float)
    b = np.asarray(gyro_bias, float)
    n = int(duration * FS)
    t = np.arange(n) / FS
    w = np.vstack([0.9 * np.sin(0.5 * t), 0.7 * np.cos(0.31 * t), 1.1 * np.sin(0.23 * t + 1)])
    w[:, t < 2.0] = 0.0                               # still for the static calibration
    out = bytearray()
    out += encode_frame(TYPE_STATUS, pack_status(0, 0, running=0, calib=0))
    out += encode_frame(TYPE_INIT, pack_init([1, 0, 0, 0], [0, 0, 1], B / np.linalg.norm(B), b))
    R = np.eye(3)
    for k in range(n):
        if k:
            R = R @ exp_so3(w[:, k] / FS)
        gyro = w[:, k] + b + 0.002 * rng.standard_normal(3)
        acc = R.T @ np.array([0.0, 0.0, 1.0]) + 0.003 * rng.standard_normal(3)
        mag_sensor = M.T @ (R.T @ B + h) + 0.3 * rng.standard_normal(3)
        flags = 1 | (2 if k % 5 in (0, 2) else 0)    # MAG_NEW at ~96 Hz
        out += encode_frame(TYPE_STATE, pack_state(k, k * 1e6 / FS, rotm2quat(R), b, acc, gyro,
                                                   mag_sensor, flags))
        if k % 24 == 0:
            out += encode_frame(TYPE_STATUS, pack_status(2000 + k * 1000 / FS, k))
    return bytes(out)


def write_stream(path, **kw) -> str:
    """Writes :func:`synthetic_stream` to ``path`` and returns it."""
    with open(path, "wb") as fh:
        fh.write(synthetic_stream(**kw))
    return path
