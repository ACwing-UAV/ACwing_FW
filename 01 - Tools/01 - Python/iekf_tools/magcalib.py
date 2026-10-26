"""Magnetometer calibration from a viewer log (port of ``Matlab/MagCalib``).

``calib_mag_axes`` (calibMagAxes.m) estimates the magnetometer axis map AND the
hard-iron offset; ``fit_hard_iron`` (fitHardIron.m) is a plain sphere fit;
``mag_check`` (magCheck.m) tells whether the calibration in the firmware still
matches the board, from an ordinary log of that firmware.

Principle of ``calib_mag_axes``: a world-fixed vector (the magnetic field),
seen from a rotating body, changes exactly as the gyroscope says::

    m(t_j) = D_ij' m(t_i),   D_ij = rotation integrated from the gyro between i and j

If the magnetometer axes are rotated w.r.t. the gyro axes
(``mag_body = M mag_sensor``) the relation holds only with the right ``M``.
All 24 signed permutations are tried (LIS2MDL axes parallel to the LSM6DSV16X
axes, as on the IKS4A1) and for each the hard-iron ``h`` is solved by least
squares::

    (I - D') M h = M m_j - D' M m_i

The ``M`` with the smallest residual wins, and must win by a clear margin.

Acquisition: live viewer, board still for 2 s, then ~40 s of slow/moderate
rotations about ALL axes (tilts included), away from metal objects. The log
holds the field AFTER the ``MAG_AXIS_MAP`` / ``MAG_HARD_IRON`` of the firmware
that recorded it, so either record with identity / 0, or pass the firmware
values (``map_fw``, ``hard_iron_fw``): the raw field is then reconstructed and
the printed values are absolute in both cases.

Pitfall: on a log of an already calibrated firmware, ``calib_mag_axes(log)``
WITHOUT the firmware values returns ~identity and ~0, i.e. the residual
correction, not the calibration. Pasting it into ``app_config.h`` removes the
real map and offset and locks the yaw onto a fixed body axis.
"""
from __future__ import annotations

import itertools
import warnings

import numpy as np

from .common import Struct
from .filters.original import exp_so3
from .logio import load_log


def signed_permutations():
    """The 24 rotation matrices with a single +-1 per row and column
    (same order as ``signedPermutations`` in calibMagAxes.m)."""
    out = []
    for perm in reversed(list(itertools.permutations(range(3)))):   # MATLAB perms(1:3) order
        for s in range(8):
            sg = 1 - 2 * np.array([s & 1, (s & 2) > 0, (s & 4) > 0], dtype=float)
            M = np.zeros((3, 3))
            for r in range(3):
                M[r, perm[r]] = sg[r]
            if np.linalg.det(M) > 0:
                out.append(M)
    return out


def _log(log_or_path):
    """Accepts a path to a ``.mat`` log or an already loaded log Struct."""
    if isinstance(log_or_path, dict):
        return log_or_path
    return load_log(log_or_path)


def _map_matrix(map_fw) -> np.ndarray:
    """``MAG_AXIS_MAP`` as a 3x3 matrix: accepts 3x3 or the 9 values in the
    row-major order of app_config.h."""
    M = np.asarray(map_fw, float)
    return M.reshape(3, 3) if M.size == 9 else M


def calib_mag_axes(log, gap_sec: float = 0.2, verbose: bool = True,
                   map_fw=None, hard_iron_fw=None) -> Struct:
    """Magnetometer axis map + hard-iron from a viewer log (path or loaded Struct).

    ``map_fw`` / ``hard_iron_fw``: ``MAG_AXIS_MAP`` / ``MAG_HARD_IRON`` of the
    firmware that recorded the log (default identity / 0). With them the raw
    sensor field is reconstructed (``raw = M_fw' (m + h_fw)``), so a log of an
    already calibrated firmware can be used without re-flashing.

    Returns a Struct with ``M`` (3x3, ``mag_body = M mag_sensor``),
    ``hardIronBody`` (3,), ``residuals`` (24, sorted) and ``ratio`` (second
    best / best residual: should be well above 2). Both are absolute.
    """
    L = _log(log)
    N = L.seq.size
    b = np.asarray(L.init.b0, float) if L.init else L.bias[:, 0]
    g = L.gyro - b[:, None]
    m = np.asarray(L.mag, float)
    if map_fw is None and hard_iron_fw is None:
        if verbose:
            print("Assuming the log was recorded with MAG_AXIS_MAP = identity, MAG_HARD_IRON = 0.\n"
                  "Otherwise pass them: calib_mag_axes(log, map_fw=..., hard_iron_fw=...).")
    else:
        Mfw = np.eye(3) if map_fw is None else _map_matrix(map_fw)
        hfw = np.zeros(3) if hard_iron_fw is None else np.asarray(hard_iron_fw, float).ravel()
        m = Mfw.T @ (m + hfw[:, None])                     # back to the raw sensor field

    t = L.t_us * 1e-6
    dt = np.concatenate(([0.0], np.diff(t)))
    fs = float(L.init.fs) if L.init else 240.0
    bad = ~((dt > 0.2 / fs) & (dt < 5 / fs))
    dt[bad] = 1 / fs
    dt[0] = 0.0

    R = np.zeros((N, 3, 3))
    R[0] = np.eye(3)
    for k in range(1, N):
        R[k] = R[k - 1] @ exp_so3(g[:, k] * dt[k])

    idx = np.flatnonzero((L.flags.astype(int) & 2) > 0)      # samples with a NEW mag
    K = int(round(gap_sec * fs))
    pairs = []
    for a in idx:
        jj = idx[idx >= a + K]
        if jj.size:
            pairs.append((a, jj[0]))
    if len(pairs) < 50:
        raise ValueError("Too few samples: the log is too short or has no motion.")
    pairs = np.array(pairs)
    Dt = np.transpose(np.einsum("pji,pjk->pik", R[pairs[:, 0]], R[pairs[:, 1]]), (0, 2, 1))
    ang = np.degrees(np.arccos(np.clip((np.trace(Dt, axis1=1, axis2=2) - 1) / 2, -1, 1)))
    if verbose:
        print(f"Pairs used: {len(pairs)}   mean rotation per pair: {ang.mean():.1f} deg")

    mi, mj = m[:, pairs[:, 0]].T, m[:, pairs[:, 1]].T          # (P, 3)
    cands = signed_permutations()
    res = np.zeros(len(cands))
    hs = np.zeros((len(cands), 3))
    I3 = np.eye(3)
    for c, M in enumerate(cands):
        A = ((I3 - Dt) @ M).reshape(-1, 3)
        Mmi = mi @ M.T
        Mmj = mj @ M.T
        r = (Mmj - np.einsum("pij,pj->pi", Dt, Mmi)).reshape(-1)
        h, *_ = np.linalg.lstsq(A, r, rcond=None)
        hs[c] = h
        res[c] = np.sqrt(np.mean((A @ h - r) ** 2))
    order = np.argsort(res, kind="stable")
    res = res[order]
    Mb = cands[order[0]]
    h_body = Mb @ hs[order[0]]
    ratio = res[1] / res[0]

    if verbose:
        print("\nResidual (uT) of the best 4 hypotheses out of 24:")
        for k in range(4):
            print(f"  {k + 1}) {res[k]:.2f}")
    if ratio < 2:
        warnings.warn(f"The best hypothesis is not clearly separated from the second "
                      f"(ratio {ratio:.1f}): repeat with more rotations about all axes.")
    if verbose:
        print("\nmag_body = M * mag_sensor, with\nM =")
        print(Mb)
        print("offset hard-iron (body frame): {:.2f} {:.2f} {:.2f} uT".format(*h_body))
        print("\nIn app_config.h:")
        rows = [", ".join(f"{v:g}f" for v in Mb[r]) for r in range(3)]
        print("#define MAG_AXIS_MAP   { " + ",   ".join(rows) + " }")
        print("#define MAG_HARD_IRON  {{ {:.2f}f, {:.2f}f, {:.2f}f }}".format(*h_body))
    return Struct(M=Mb, hardIronBody=h_body, residuals=res, ratio=ratio)


def fit_hard_iron(log, do_plot: bool = True, show: bool = True, verbose: bool = True):
    """Hard-iron offset by least-squares sphere fit. Returns ``(offset, radius)``.

    The log holds the field already in the BODY frame (``MAG_AXIS_MAP``
    applied), so the offset can be copied as-is into ``MAG_HARD_IRON``.
    ``calib_mag_axes`` estimates axis map AND hard-iron and is preferred.
    """
    L = _log(log)
    m = L.mag[:, np.all(np.isfinite(L.mag), axis=0)]
    A = np.column_stack((2 * m.T, np.ones(m.shape[1])))
    bb = np.sum(m ** 2, axis=0)
    p, *_ = np.linalg.lstsq(A, bb, rcond=None)
    offset = p[0:3]
    radius = float(np.sqrt(p[3] + offset @ offset))
    res = np.linalg.norm(m - offset[:, None], axis=0) - radius
    span = m.max(axis=1) - m.min(axis=1)
    if verbose:
        print("hard-iron offset [uT]: {:.2f} {:.2f} {:.2f}".format(*offset)
              + f"   |B| = {radius:.1f} uT   residual std {np.std(res, ddof=1):.2f} uT")
    if np.any(span < radius):
        warnings.warn("Poor angular coverage (per-axis span {:.0f}/{:.0f}/{:.0f} uT < 2|B|): "
                      "rotate the board in more directions.".format(*span))
    if verbose:
        print("\nIn app_config.h:\n#define MAG_HARD_IRON  {{ {:.2f}f, {:.2f}f, {:.2f}f }}".format(*offset))
    if do_plot:
        import matplotlib.pyplot as plt
        fig = plt.figure("Fit hard-iron")
        ax = fig.add_subplot(projection="3d")
        ax.plot(m[0], m[1], m[2], ".", ms=2)
        u, v = np.mgrid[0:2 * np.pi:25j, 0:np.pi:13j]
        ax.plot_wireframe(radius * np.cos(u) * np.sin(v) + offset[0],
                          radius * np.sin(u) * np.sin(v) + offset[1],
                          radius * np.cos(v) + offset[2], color="gray", alpha=0.3, lw=0.5)
        ax.set_xlabel("x_b [uT]"); ax.set_ylabel("y_b [uT]"); ax.set_zlabel("z_b [uT]")
        ax.set_box_aspect((1, 1, 1))
        ax.set_title("Magnetometer samples and fitted sphere")
        if show:
            plt.show()
    return offset, radius


def _logm3(R) -> np.ndarray:
    """Rotation vector of R (same formula as the local ``logm3`` in magCheck.m)."""
    c = np.clip((np.trace(R) - 1) / 2, -1, 1)
    a = np.arccos(c)
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return v / 2 if a < 1e-9 else a / (2 * np.sin(a)) * v


def mag_check(log, hard_iron_fw=None, verbose: bool = True) -> Struct:
    """Is the magnetometer calibration in the firmware still valid? (port of ``magCheck.m``)

    Works on an ordinary log of the CURRENT firmware (field after
    ``MAG_AXIS_MAP`` and ``MAG_HARD_IRON``): board still for 2 s, then ~30 s of
    slow rotations about all axes. With a valid calibration the samples lie on a
    sphere centred at 0 (|B| 25-65 uT) and the field direction turns in the body
    frame as the gyro says. A stale hard-iron moves the centre away from 0; when
    the offset is much larger than the Earth field, m stays almost fixed in the
    body frame, the yaw locks onto a fixed body axis and the mag is de-weighted.

    Returns a Struct with ``residualOffset`` (3,), ``B``, ``ratio`` (field
    rotation / gyro rotation), ``normMean``, ``gyroAngle``, ``magAngle`` [rad],
    ``newHardIron`` (current + residual, if ``hard_iron_fw`` is given) and
    ``verdict``: ``'ok'``, ``'hard-iron-mismatch'``, ``'no-data'`` (field
    constant while rotating: no new LIS2MDL data) or ``'no-rotation'``.
    """
    L = _log(log)
    m = np.asarray(L.mag, float)
    ok = np.all(np.isfinite(m), axis=0)
    N = m.shape[1]

    # 1) sphere fit on the corrected field: residual offset and |B|
    mo = m[:, ok]
    A = np.column_stack((2 * mo.T, np.ones(mo.shape[1])))
    p, *_ = np.linalg.lstsq(A, np.sum(mo ** 2, axis=0), rcond=None)
    d = p[0:3]
    rad = float(np.sqrt(p[3] + d @ d))
    nrm = np.linalg.norm(mo, axis=0)
    if verbose:
        print(f"|m| logged      : mean {nrm.mean():.1f} uT, min {nrm.min():.1f}, max {nrm.max():.1f} "
              f"(std {np.std(nrm, ddof=1):.1f})")
        print("sphere fit      : residual offset [{:.1f} {:.1f} {:.1f}] uT".format(*d)
              + f" (|.| = {np.linalg.norm(d):.1f}), |B| = {rad:.1f} uT")

    # 2) does m turn with the body as the gyro says?
    t = np.asarray(L.t_us, float) * 1e-6
    dt = np.concatenate(([0.0], np.diff(t)))
    fs = float(L.init.fs) if L.init else 240.0
    bad = ~((dt > 0.2 / fs) & (dt < 5 / fs))
    dt[bad] = 1 / fs
    dt[0] = 0.0
    g = L.gyro - np.mean(L.gyro[:, :min(N, int(round(fs)))], axis=1, keepdims=True)
    R = np.eye(3)
    gyro_ang = mag_ang = 0.0
    m_prev = m[:, 0]
    k0 = 0
    for k in range(1, N):
        R = R @ exp_so3(g[:, k] * dt[k])
        if t[k] - t[k0] >= 0.25:                               # compare every 0.25 s
            if np.all(np.isfinite(np.concatenate((m_prev, m[:, k])))):
                gyro_ang += np.linalg.norm(_logm3(R))
                c = m_prev @ m[:, k] / (np.linalg.norm(m_prev) * np.linalg.norm(m[:, k]))
                mag_ang += np.arccos(np.clip(c, -1, 1))
            R = np.eye(3)
            m_prev = m[:, k]
            k0 = k
    ratio = mag_ang / max(gyro_ang, np.finfo(float).eps)
    if verbose:
        print(f"rotation        : gyro {np.degrees(gyro_ang):.0f} deg, mag direction "
              f"{np.degrees(mag_ang):.0f} deg (ratio {ratio:.2f}; 0.5-1 expected, "
              "rotations about the field do not move it)")

    # 3) verdict (same thresholds as magCheck.m)
    out = Struct(residualOffset=d, B=rad, ratio=ratio, normMean=float(nrm.mean()),
                 gyroAngle=gyro_ang, magAngle=mag_ang, newHardIron=None, verdict="")
    nd = float(np.linalg.norm(d))
    if np.max(np.std(mo, axis=1, ddof=1)) < 1 and gyro_ang > np.pi:
        out.verdict = "no-data"
        msg = ("VERDICT: the logged field is CONSTANT while the board rotates: the LIS2MDL "
               "delivers no new data\n  (raw = 0 or frozen). Check n_mag / n_mag_dup / "
               "n_sh_nack in STATUS, power-cycle the board.")
    elif gyro_ang < np.pi:
        out.verdict = "no-rotation"
        msg = (f"VERDICT: not enough rotation in the log ({np.degrees(gyro_ang):.0f} deg): "
               "rotate the board about all axes.")
    elif nd > 0.3 * rad or ratio < 0.4 or rad < 20 or rad > 80:
        out.verdict = "hard-iron-mismatch"
        msg = (f"VERDICT: MAG_HARD_IRON no longer matches the board (offset {nd:.0f} uT "
               f"vs field {rad:.0f} uT).\n")
        if hard_iron_fw is not None:
            out.newHardIron = np.asarray(hard_iron_fw, float).ravel() + d
            msg += ("  Corrected value (MAG_AXIS_MAP unchanged):\n"
                    "  #define MAG_HARD_IRON  {{ {:.2f}f, {:.2f}f, {:.2f}f }}\n".format(*out.newHardIron))
        else:
            msg += "  New MAG_HARD_IRON = current MAG_HARD_IRON + [{:.2f} {:.2f} {:.2f}]\n".format(*d)
        msg += ("  If the offset moved because something magnetic was brought near the board, "
                "remove it or\n  recalibrate in the final set-up (calib-mag).")
    else:
        out.verdict = "ok"
        msg = f"VERDICT: calibration consistent (residual offset {nd:.1f} uT, |B| {rad:.1f} uT)."
    if verbose:
        print("\n" + msg)
    return out
