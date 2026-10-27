"""Extended IEKF, mirror of the C code (port of ``IEKF_embeddedVersion``).

``iekf_attitude_emb`` ports ``iekfAttitudeEmb.m``: same signature and options
as the original ``iekf_attitude`` plus the firmware extensions. Its maths goes
through the ``core_*`` functions, which port ``iekfCore*.m``, themselves a
line-by-line mirror of ``App/filters/iekf/iekf.c``:

- ``core_default_config``  <- iekfCoreDefaultConfig.m  <- iekf_default_config()
- ``core_init``            <- iekfCoreInit.m           <- iekf_init()
- ``core_propagate``       <- iekfCorePropagate.m      <- iekf_propagate_dtheta()
- ``core_update_acc``      <- iekfCoreUpdateAcc.m      <- iekf_update_acc()
- ``core_update_mag``      <- iekfCoreUpdateMag.m      <- iekf_update_mag_ex()
- ``vector_update``        <- iekfVectorUpdate.m       <- vector_update()
- ``condition_cov``        <- iekfConditionCov.m       <- condition_covariance()
- ``so3_exp_local``, ``so3_orthonormalize``, ``so3_to_quat`` <- so3.c

Differences from the original filter: error reset after the retraction,
re-orthonormalisation, symmetrisation and diagonal floor of P, NIS inflation
with memory and cap, magnetic dip check, decoupling, lockout protection with
world-frame persistence, stillness-trusted acc updates, extra mag variance.
"""
from __future__ import annotations

from typing import Optional

import numpy as np

from ..common import Struct, rotm2quat

so3_to_quat = rotm2quat


def skew_local(v) -> np.ndarray:
    """Skew-symmetric matrix ``[v]x`` (as ``so3_skew()``)."""
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


def so3_exp_local(phi) -> np.ndarray:
    """Rodrigues with the same Taylor series as ``so3_exp()`` in so3.c."""
    phi = np.asarray(phi, dtype=float).reshape(3)
    t2 = phi @ phi
    if t2 < 2.5e-3:
        a = 1.0 - t2 / 6.0 + t2 * t2 / 120.0
        b = 0.5 - t2 / 24.0 + t2 * t2 / 720.0
    else:
        t = np.sqrt(t2)
        a = np.sin(t) / t
        b = (1.0 - np.cos(t)) / t2
    return np.eye(3) + a * skew_local(phi) + b * (np.outer(phi, phi) - t2 * np.eye(3))


def so3_orthonormalize(R) -> np.ndarray:
    """Gram-Schmidt on the columns, as ``so3_orthonormalize()`` in so3.c."""
    c0 = R[:, 0]
    c1 = R[:, 1]
    n0 = np.linalg.norm(c0)
    if n0 < 1e-12:
        return R
    c0 = c0 / n0
    c1 = c1 - (c0 @ c1) * c0
    n1 = np.linalg.norm(c1)
    if n1 < 1e-12:
        return R
    c1 = c1 / n1
    return np.column_stack((c0, c1, np.cross(c0, c1)))


def core_default_config() -> Struct:
    """Defaults identical to ``iekf_default_config()`` in iekf.c."""
    return Struct(
        sigmaGyro=0.5 * np.pi / 180, sigmaBias=1e-4, sigmaAcc=0.05, sigmaMag=0.05,
        accTol=0.08, magTol=0.08, p0Att=1e-2, p0Bias=1e-4,
        estimateBias=True, adaptive=True, tiltCompMag=False,
        accNisGate=0.0, magNisGate=0.0, inflDecay=0.0, dipTol=0.0,
        decouple=False, resetJacobian=True,
        inflMax=1e4, lockoutUpdates=0, lockoutPBump=0.0, graceUpdates=0,
        persistDecay=0.0, persistRatio=0.6, persistMin=0.02,
    )


def core_init(cfg, seed) -> Struct:
    """Initialises the filter state (mirror of ``iekf_init()``).

    ``seed``: ``R0`` (3x3), ``b0`` (3,), ``gRef``, ``mRef`` (3,), ``nomAcc``, ``nomMag``.
    The state is a new Struct; every ``core_*`` function updates it in place
    and also returns it.
    """
    f = Struct()
    f.cfg = cfg
    f.R = so3_orthonormalize(np.asarray(seed["R0"], float))
    f.b = np.asarray(seed["b0"], float).reshape(3).copy() if cfg.estimateBias else np.zeros(3)
    g = np.asarray(seed["gRef"], float).reshape(3)
    m = np.asarray(seed["mRef"], float).reshape(3)
    f.gRef = g / np.linalg.norm(g)
    f.mRef = m / np.linalg.norm(m)
    f.nomAcc = seed["nomAcc"] if seed["nomAcc"] > 1e-6 else 1.0
    f.nomMag = seed["nomMag"] if seed["nomMag"] > 1e-6 else 1.0
    f.accInfl = 1.0
    f.magInfl = 1.0
    f.dipRef = float(np.arccos(np.clip(f.gRef @ f.mRef, -1.0, 1.0)))
    f.lastAccDir = f.R.T @ f.gRef
    f.accTrusted = False
    f.lastAccDev = f.lastMagDev = f.lastDipDev = 0.0
    f.lastAccNis = f.lastMagNis = 0.0
    f.Elast = np.eye(3)
    f.accGatedRun = 0
    f.accGrace = 0
    f.lockouts = 0
    f.innovWMean = np.zeros(3)
    f.innovWPow = 0.0
    f.lastAccInnovB = np.zeros(3)
    eb = 1.0 if cfg.estimateBias else 0.0
    f.P = np.diag([cfg.p0Att] * 3 + [eb * cfg.p0Bias] * 3).astype(float)
    return f


def condition_cov(f) -> Struct:
    """Symmetry and diagonal floor (mirror of ``condition_covariance()``)."""
    P = (f.P + f.P.T) / 2.0
    for i in range(3):
        if P[i, i] < 1e-12:
            P[i, i] = 1e-12
    if f.cfg.estimateBias:
        for i in range(3, 6):
            if P[i, i] < 1e-16:
                P[i, i] = 1e-16
    f.P = P
    return f


def core_propagate(f, dtheta, h) -> Struct:
    """Propagation with an angle increment (mirror of ``iekf_propagate_dtheta()``).

    ``dtheta`` = sum of omega*dt over the step samples (bias NOT removed),
    ``h`` = step duration [s].
    """
    phi = np.asarray(dtheta, float).reshape(3) - f.b * h
    E = so3_exp_local(phi)
    f.R = so3_orthonormalize(f.R @ E)
    f.Elast = E
    F = np.eye(6)
    F[0:3, 0:3] = E.T                                  # exp(-phi): measured gyro only
    if f.cfg.estimateBias:
        F[0:3, 3:6] = -h * np.eye(3)
    P = F @ f.P @ F.T
    qg = f.cfg.sigmaGyro ** 2 * h
    qb = (1.0 if f.cfg.estimateBias else 0.0) * f.cfg.sigmaBias ** 2 * h
    P[0:3, 0:3] += qg * np.eye(3)
    P[3:6, 3:6] += qb * np.eye(3)
    f.P = P
    return condition_cov(f)


def _adaptive_var(r0, n, nom, tol, adaptive):
    dev = abs(n - nom) / nom
    return (r0 * (1.0 + (dev / tol) ** 2) if adaptive else r0), dev


def core_update_acc(f, acc):
    """Accelerometer update (mirror of ``iekf_update_acc()``). Returns ``(f, gated)``."""
    acc = np.asarray(acc, float).reshape(3)
    na = np.linalg.norm(acc)
    if na < 1e-9:
        return f, False
    y = acc / na
    f.lastAccDir = y
    yhat = f.R.T @ f.gRef
    cfg = f.cfg

    r, f.lastAccDev = _adaptive_var(cfg.sigmaAcc ** 2, na, f.nomAcc, cfg.accTol, cfg.adaptive)
    f.lastAccInnovB = y - yhat
    trusted = bool(f.accTrusted) or f.accGrace > 0
    if f.accGrace > 0:
        f.accGrace -= 1
    if trusted:
        f.accInfl = 1.0
    gate = 0.0 if trusted else cfg.accNisGate
    proj = 1 if cfg.decouple else 0                    # 1 = tilt only
    f, f.lastAccNis, f.accInfl = vector_update(f, y, yhat, r, gate, f.accInfl, proj)
    f.accTrusted = False

    # persistence of the innovation in the world frame (updated estimate, as in C)
    ew = f.R @ f.lastAccInnovB
    a = cfg.persistDecay
    f.innovWMean = a * f.innovWMean + (1.0 - a) * ew
    f.innovWPow = a * f.innovWPow + (1.0 - a) * (ew @ ew)

    # lockout protection
    if cfg.lockoutUpdates > 0:
        m2 = f.innovWMean @ f.innovWMean
        persistent = (f.innovWPow > 0) and (m2 > cfg.persistRatio * f.innovWPow) \
            and (m2 > cfg.persistMin ** 2)
        if f.accInfl > 2:
            f.accGatedRun += 1
            if f.accGatedRun >= cfg.lockoutUpdates and persistent:
                f.P[0:3, 0:3] += cfg.lockoutPBump * np.eye(3)
                f.accInfl = 1.0
                f.accGatedRun = 0
                f.accGrace = cfg.graceUpdates
                f.lockouts += 1
        else:
            f.accGatedRun = 0
    gated = (f.lastAccDev > cfg.accTol) or (f.accInfl > 2)
    return f, gated


def core_update_mag(f, mag, extra_var=0.0):
    """Magnetometer update (mirror of ``iekf_update_mag_ex()``). Returns ``(f, gated)``."""
    mag = np.asarray(mag, float).reshape(3)
    nm = np.linalg.norm(mag)
    if nm < 1e-9:
        return f, False
    y = mag / nm
    cfg = f.cfg
    if cfg.tiltCompMag:
        hw = f.R @ y                                   # tilt-compensated compass
        bw = np.array([np.sqrt(hw[0] ** 2 + hw[1] ** 2), 0.0, hw[2]])
        yhat = f.R.T @ bw
    else:
        yhat = f.R.T @ f.mRef

    dev = abs(nm - f.nomMag) / f.nomMag
    f.lastMagDev = dev
    r = cfg.sigmaMag ** 2
    if cfg.adaptive:
        r = r * (1.0 + (dev / cfg.magTol) ** 2)
    f.lastDipDev = 0.0
    if cfg.dipTol > 0:                                 # measured dip vs calibration
        dip = np.arccos(np.clip(y @ f.lastAccDir, -1.0, 1.0))
        f.lastDipDev = abs(dip - f.dipRef)
        r = r * (1.0 + (f.lastDipDev / cfg.dipTol) ** 2)
    r = r + extra_var

    proj = 2 if cfg.decouple else 0                    # 2 = yaw only
    f, f.lastMagNis, f.magInfl = vector_update(f, y, yhat, r, cfg.magNisGate, f.magInfl, proj)
    gated = (dev > cfg.magTol) or (f.magInfl > 2) or (cfg.dipTol > 0 and f.lastDipDev > cfg.dipTol)
    return f, gated


def vector_update(f, y, yhat, r, gate, infl, proj):
    """Common vector correction (mirror of ``vector_update()`` in iekf.c).

    NIS with nominal R -> inflation with memory -> gain -> projection ->
    retraction onto SO(3) -> Joseph form -> error reset -> conditioning.
    ``proj``: 0 none, 1 tilt only (acc), 2 yaw only (mag). Returns ``(f, nis, infl)``.
    """
    cfg = f.cfg
    H = np.zeros((3, 6))
    H[:, 0:3] = skew_local(yhat)
    PHt = f.P @ H.T
    S0 = H @ PHt
    innov = y - yhat

    Si = np.linalg.inv(S0 + r * np.eye(3))
    nis = float(innov @ Si @ innov)

    k = 1.0
    if gate > 0:
        inst = nis / gate
        g = infl * cfg.inflDecay
        if g < 1:
            g = 1.0
        if inst > g:
            g = inst
        if g > cfg.inflMax:
            g = cfg.inflMax
        infl = g
        k = g
    re = r * k
    if k != 1:
        Si = np.linalg.inv(S0 + re * np.eye(3))
    K = PHt @ Si
    K = _project_gain(f, K, proj)

    dx = K @ innov
    f.R = so3_orthonormalize(f.R @ so3_exp_local(dx[0:3]))
    if cfg.estimateBias:
        f.b = f.b + dx[3:6]

    IKH = np.eye(6) - K @ H
    P = IKH @ f.P @ IKH.T + re * (K @ K.T)
    P = (P + P.T) / 2.0
    if cfg.resetJacobian:                              # error reset after the retraction
        J = np.eye(3) - 0.5 * skew_local(dx[0:3])
        P[0:3, 0:3] = J @ P[0:3, 0:3] @ J.T
        if cfg.estimateBias:
            P[0:3, 3:6] = J @ P[0:3, 3:6]
            P[3:6, 0:3] = P[0:3, 3:6].T
    f.P = P
    f = condition_cov(f)
    return f, nis, infl


def _project_gain(f, K, proj):
    if proj == 0:
        return K
    K = K.copy()
    a = f.R.T @ f.gRef
    a = a / np.linalg.norm(a)
    blocks = [slice(0, 3)] + ([slice(3, 6)] if f.cfg.estimateBias else [])
    for rows in blocks:
        for col in range(3):
            d = a @ K[rows, col]
            K[rows, col] = K[rows, col] - d * a if proj == 1 else d * a
    return K


_DEFAULTS = dict(
    R0=None, b0=None, estimateBias=True, P0=1.0,
    sigmaGyro=np.deg2rad(0.5), sigmaBias=1e-3, sigmaAcc=0.05, sigmaMag=0.05,
    adaptive=True, accTol=0.05, magTol=0.05, tiltCompensatedMag=False,
    accNisGate=0.0, magNisGate=0.0, inflTau=0.0, dipTol=0.0,
    decouple=False, resetJacobian=True, nomAcc=None, nomMag=None,
    inflMax=1e4, lockoutTime=0.0, lockoutPBump=np.deg2rad(10) ** 2, graceTime=0.25,
    persistTau=1.0, persistRatio=0.6, persistMin=0.02,
    inflDecay=None, lockoutUpdates=None, graceUpdates=None, persistDecay=None,
    accTrusted=None, magExtraVar=None,
)


def iekf_attitude_emb(gyro, acc, mag, dt, gRef, mRef, opts: Optional[dict] = None):
    """The original filter adapted to what is implemented in C (port of ``iekfAttitudeEmb.m``).

    Same inputs, outputs and options as :func:`iekf_attitude`, plus:

    ``accNisGate``, ``magNisGate`` (NIS gates, 0 = off), ``inflTau`` [s]
    (inflation memory), ``dipTol`` [rad], ``decouple``, ``resetJacobian``
    (default True), ``nomAcc``/``nomMag`` (default: first sample), ``inflMax``,
    ``lockoutTime`` [s] (0 = off), ``lockoutPBump``, ``graceTime``,
    ``persistTau``, ``persistRatio``, ``persistMin``.
    Time constants are converted per ACC UPDATE, as in the firmware; the exact
    per-update values can be given instead: ``inflDecay``,
    ``lockoutUpdates``, ``graceUpdates``, ``persistDecay``.
    Per-sample inputs produced by the firmware pipeline (length N, optional):
    ``accTrusted`` (stillness flag), ``magExtraVar`` (mag-age variance).

    Returns ``Rest (3,3,N), bEst (3,N), Phist (n,n,N), info`` where ``info`` has
    ``lockouts`` and ``accInflMax``.
    """
    o = dict(_DEFAULTS)
    if opts:
        o.update(opts)
    gyro = np.asarray(gyro, float)
    acc = np.asarray(acc, float)
    use_mag = mag is not None and np.size(mag) > 0
    if use_mag:
        mag = np.asarray(mag, float)
    N = gyro.shape[1]

    dtv = np.full(N, float(dt)) if np.isscalar(dt) or np.size(dt) == 1 else np.asarray(dt, float).reshape(N)
    okdt = np.isfinite(dtv) & (dtv > 0)
    h_med = float(np.median(dtv[okdt]))
    # inflation memory and lockout counters run PER ACC UPDATE (as in C)
    t_all = np.concatenate(([0.0], np.cumsum(np.where(okdt[1:], dtv[1:], 0.0))))
    i_acc = np.flatnonzero(np.linalg.norm(acc, axis=0) > np.finfo(float).eps)
    h_acc = h_med
    if i_acc.size > 1:
        h_acc = float(np.median(np.diff(t_all[i_acc])))

    cfg = core_default_config()
    cfg.sigmaGyro, cfg.sigmaBias = o["sigmaGyro"], o["sigmaBias"]
    cfg.sigmaAcc, cfg.sigmaMag = o["sigmaAcc"], o["sigmaMag"]
    cfg.accTol, cfg.magTol = o["accTol"], o["magTol"]
    cfg.estimateBias = bool(o["estimateBias"])
    cfg.adaptive = bool(o["adaptive"])
    cfg.tiltCompMag = bool(o["tiltCompensatedMag"])
    cfg.accNisGate, cfg.magNisGate = o["accNisGate"], o["magNisGate"]
    cfg.inflDecay = np.exp(-h_acc / o["inflTau"]) if o["inflTau"] > 0 else 0.0
    cfg.dipTol, cfg.decouple = o["dipTol"], bool(o["decouple"])
    cfg.inflMax = o["inflMax"]
    if o["lockoutTime"] > 0:
        cfg.lockoutUpdates = int(round(o["lockoutTime"] / h_acc))
        cfg.graceUpdates = int(round(o["graceTime"] / h_acc))
        cfg.lockoutPBump = o["lockoutPBump"]
    cfg.persistDecay = np.exp(-h_acc / o["persistTau"])
    cfg.persistRatio, cfg.persistMin = o["persistRatio"], o["persistMin"]
    for key in ("inflDecay", "lockoutUpdates", "graceUpdates", "persistDecay"):
        if o[key] is not None:
            cfg[key] = o[key]
    if cfg.lockoutUpdates > 0 and cfg.lockoutPBump == 0:
        cfg.lockoutPBump = o["lockoutPBump"]
    cfg.resetJacobian = bool(o["resetJacobian"])
    P0 = o["P0"]
    scalar_p0 = np.isscalar(P0) or np.size(P0) == 1
    cfg.p0Att = cfg.p0Bias = float(P0) if scalar_p0 else 1.0

    seed = Struct(
        R0=np.eye(3) if o["R0"] is None else o["R0"],
        b0=np.zeros(3) if o["b0"] is None else o["b0"],
        gRef=gRef,
        mRef=mRef if use_mag else np.array([1.0, 0.0, 0.0]),
        nomAcc=o["nomAcc"] if o["nomAcc"] is not None else float(np.linalg.norm(acc[:, 0])),
        nomMag=o["nomMag"] if o["nomMag"] is not None else
        (float(np.linalg.norm(mag[:, 0])) if use_mag else 1.0),
    )
    f = core_init(cfg, seed)
    if not scalar_p0:
        P0 = np.asarray(P0, float)
        P = np.zeros((6, 6))
        P[:P0.shape[0], :P0.shape[0]] = P0
        f.P = P

    n = 6 if cfg.estimateBias else 3
    Rest = np.zeros((3, 3, N))
    bEst = np.zeros((3, N))
    Phist = np.zeros((n, n, N))
    Rest[:, :, 0], bEst[:, 0], Phist[:, :, 0] = f.R, f.b, f.P[:n, :n]
    info = Struct(lockouts=0, accInflMax=1.0)

    acc_tr = o["accTrusted"]
    mag_xv = o["magExtraVar"]
    for k in range(1, N):
        h = dtv[k]
        if not np.isfinite(h) or h <= 0:
            h = h_med
        core_propagate(f, gyro[:, k - 1] * h, h)
        if acc_tr is not None:
            f.accTrusted = bool(acc_tr[k])
        core_update_acc(f, acc[:, k])
        if use_mag:
            xv = 0.0 if mag_xv is None else float(mag_xv[k])
            core_update_mag(f, mag[:, k], xv)
        Rest[:, :, k], bEst[:, k], Phist[:, :, k] = f.R, f.b, f.P[:n, :n]
        info.accInflMax = max(info.accInflMax, f.accInfl)
    info.lockouts = f.lockouts
    return Rest, bEst, Phist, info
