"""Original IEKF (Python port of ``00 - Matlab/FiltersComparison/IEKF_originalVersion``).

``iekf_attitude`` is a line-by-line port of ``iekfAttitude.m``, the filter
first developed and benchmarked in MATLAB and used as the reference for the
validation of the embedded C code. ``exp_so3``, ``log_so3`` and ``skew_sym``
port ``expSO3.m``, ``logSO3.m`` and ``skewSym.m``.

Array layout follows MATLAB: inputs are ``3 x N``, outputs are ``3 x 3 x N``
(attitude), ``3 x N`` (bias) and ``n x n x N`` (covariance).

Theory (summary; see the README and the MATLAB help for the full text)
-----------------------------------------------------------------------
Left-invariant error on SO(3): ``R_true = R_hat * exp(delta)``. Error state
``x = [delta; beta]``, ``beta = b_true - b_hat``::

    delta_dot = -[omega_hat]x delta - beta - n_g,     beta_dot = n_b
    F = [exp(-omega h), -h I; 0, I]                   (depends only on the gyro)
    y = R' v + noise,  y_hat = R_hat' v,  H = [[y_hat]x, 0]

Correction injected with the exponential map (retraction), covariance in
Joseph form.
"""
from __future__ import annotations

from typing import Optional

import numpy as np


def skew_sym(v) -> np.ndarray:
    """Skew-symmetric matrix ``[v]x`` such that ``skew_sym(v) @ w == cross(v, w)``."""
    v = np.asarray(v, dtype=float).reshape(3)
    return np.array([[0.0, -v[2], v[1]],
                     [v[2], 0.0, -v[0]],
                     [-v[1], v[0], 0.0]])


def exp_so3(phi) -> np.ndarray:
    """Exponential map so(3) -> SO(3) (Rodrigues), first order near zero."""
    phi = np.asarray(phi, dtype=float).reshape(3)
    t = np.sqrt(phi @ phi)
    if t < 1e-9:
        return np.eye(3) + skew_sym(phi)
    K = skew_sym(phi / t)
    return np.eye(3) + np.sin(t) * K + (1.0 - np.cos(t)) * (K @ K)


def log_so3(R) -> np.ndarray:
    """Logarithm map SO(3) -> so(3): rotation vector with ``exp_so3(phi) == R``.

    ``norm(phi)`` is the geodesic distance from the identity (rotation angle).
    """
    R = np.asarray(R, dtype=float)
    c = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    t = np.arccos(c)
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if t < 1e-9:
        return np.zeros(3)
    if abs(t - np.pi) < 1e-6:
        # near pi: recover the axis from the diagonal of (R + I)/2
        A = (R + np.eye(3)) / 2.0
        ax = np.sqrt(np.maximum(np.diag(A), 0.0))
        imax = int(np.argmax(ax))
        if ax[imax] > 0:
            s = v[imax] if abs(v[imax]) > 1e-12 else 1.0
            ax = ax * np.sign(s)
        return t * ax / np.linalg.norm(ax)
    return (t / (2.0 * np.sin(t))) * v


_DEFAULTS = dict(
    R0=None, b0=None, estimateBias=True, P0=1.0,
    sigmaGyro=np.deg2rad(0.5), sigmaBias=1e-3, sigmaAcc=0.05, sigmaMag=0.05,
    adaptive=True, accTol=0.05, magTol=0.05, tiltCompensatedMag=False,
)


def iekf_attitude(gyro, acc, mag, dt, gRef, mRef, opts: Optional[dict] = None):
    """Invariant EKF for attitude estimation on SO(3) (port of ``iekfAttitude.m``).

    Parameters
    ----------
    gyro : (3, N) angular rate, body frame [rad/s]
    acc : (3, N) accelerometer, body frame (any unit, normalised internally);
        a column with norm < eps skips the update at that sample
    mag : (3, N) magnetometer, body frame, or ``None`` for acc-only
    dt : scalar or (N,) sample periods [s]; ``gyro[:, k-1]`` is integrated
        over ``dt[k]``
    gRef, mRef : (3,) gravity and magnetic field directions, world frame
    opts : dict, optional, same keys as the MATLAB struct:
        ``R0`` (3x3, default I), ``b0`` (3,), ``estimateBias`` (True),
        ``P0`` (scalar or n x n, 1.0), ``sigmaGyro`` (0.5 deg/s), ``sigmaBias``
        (1e-3), ``sigmaAcc`` (0.05), ``sigmaMag`` (0.05), ``adaptive`` (True),
        ``accTol`` (0.05), ``magTol`` (0.05), ``tiltCompensatedMag`` (False)

    Returns
    -------
    Rest : (3, 3, N) attitude (body -> world)
    bEst : (3, N) gyro bias [rad/s]
    Phist : (n, n, N) error-state covariance, n = 6 (3 without bias)
    """
    o = dict(_DEFAULTS)
    if opts:
        o.update(opts)
    gyro = np.asarray(gyro, dtype=float)
    acc = np.asarray(acc, dtype=float)
    use_mag = mag is not None and np.size(mag) > 0
    if use_mag:
        mag = np.asarray(mag, dtype=float)
    N = gyro.shape[1]
    use_bias = bool(o["estimateBias"])
    n = 6 if use_bias else 3

    dtv = np.full(N, float(dt)) if np.isscalar(dt) or np.size(dt) == 1 else np.asarray(dt, float).reshape(N)
    good = np.isfinite(dtv) & (dtv > 0)
    h_med = float(np.median(dtv[good])) if good.any() else 0.0

    gRef = np.asarray(gRef, float).reshape(3)
    gRef = gRef / np.linalg.norm(gRef)
    if use_mag:
        mRef = np.asarray(mRef, float).reshape(3)
        mRef = mRef / np.linalg.norm(mRef)

    Rest = np.zeros((3, 3, N))
    bEst = np.zeros((3, N))
    Phist = np.zeros((n, n, N))

    R = np.eye(3) if o["R0"] is None else np.asarray(o["R0"], float).copy()
    b = np.zeros(3) if o["b0"] is None else np.asarray(o["b0"], float).reshape(3).copy()
    P0 = o["P0"]
    P = np.eye(n) * float(P0) if np.isscalar(P0) or np.size(P0) == 1 else np.asarray(P0, float).copy()

    Rest[:, :, 0] = R
    bEst[:, 0] = b
    Phist[:, :, 0] = P

    I3 = np.eye(3)
    Qg = o["sigmaGyro"] ** 2 * I3
    Qb = o["sigmaBias"] ** 2 * I3
    Racc0 = o["sigmaAcc"] ** 2 * I3
    Rmag0 = o["sigmaMag"] ** 2 * I3
    eps = np.finfo(float).eps

    nom_acc = np.linalg.norm(acc[:, 0])
    if nom_acc < eps:
        nom_acc = 1.0
    if use_mag:
        nom_mag = np.linalg.norm(mag[:, 0])
        if nom_mag < eps:
            nom_mag = 1.0

    for k in range(1, N):
        h = dtv[k]
        if not np.isfinite(h) or h <= 0:
            h = h_med

        # ---------- propagation ----------
        om = gyro[:, k - 1] - b
        R = R @ exp_so3(om * h)
        if use_bias:
            F = np.eye(6)
            F[0:3, 0:3] = exp_so3(-om * h)
            F[0:3, 3:6] = -I3 * h
            Q = np.zeros((6, 6))
            Q[0:3, 0:3] = Qg * h
            Q[3:6, 3:6] = Qb * h
        else:
            F = exp_so3(-om * h)
            Q = Qg * h
        P = F @ P @ F.T + Q

        # ---------- accelerometer ----------
        na = np.linalg.norm(acc[:, k])
        if na > eps:
            y = acc[:, k] / na
            yhat = R.T @ gRef
            Racc = Racc0
            if o["adaptive"]:
                dev = abs(na - nom_acc) / nom_acc
                Racc = Racc0 * (1.0 + (dev / o["accTol"]) ** 2)
            H = np.zeros((3, n))
            H[:, 0:3] = skew_sym(yhat)
            R, b, P = _apply_update(R, b, P, H, y - yhat, Racc, n, use_bias)

        # ---------- magnetometer ----------
        if use_mag:
            nm = np.linalg.norm(mag[:, k])
            if nm > eps:
                y = mag[:, k] / nm
                if o["tiltCompensatedMag"]:
                    hw = R @ y
                    bw = np.array([np.hypot(hw[0], hw[1]), 0.0, hw[2]])
                    yhat = R.T @ bw
                else:
                    yhat = R.T @ mRef
                Rmag = Rmag0
                if o["adaptive"]:
                    dev = abs(nm - nom_mag) / nom_mag
                    Rmag = Rmag0 * (1.0 + (dev / o["magTol"]) ** 2)
                H = np.zeros((3, n))
                H[:, 0:3] = skew_sym(yhat)
                R, b, P = _apply_update(R, b, P, H, y - yhat, Rmag, n, use_bias)

        Rest[:, :, k] = R
        bEst[:, k] = b
        Phist[:, :, k] = P
    return Rest, bEst, Phist


def _apply_update(R, b, P, H, innov, Rm, n, use_bias):
    """One Kalman correction injected back onto the group (Joseph form)."""
    S = H @ P @ H.T + Rm
    K = np.linalg.solve(S.T, (P @ H.T).T).T          # (P H') / S, as MATLAB right division
    dx = K @ innov
    R = R @ exp_so3(dx[0:3])
    if use_bias:
        b = b + dx[3:6]                               # dx(4:6) estimates b_true - b_hat
    IKH = np.eye(n) - K @ H
    P = IKH @ P @ IKH.T + K @ Rm @ K.T
    P = (P + P.T) / 2.0
    return R, b, P
