"""Embedded filter vs MATLAB/Python reference (port of ``compareEmbeddedVsMatlab.m``).

For every 240 Hz window the firmware (``APP_TLM_REPLAY = 1``) records the
EXACT inputs passed to the filter (960 Hz angle increments, acc mean, mag
measurement) and the resulting state (q, bias, diag P). Here those inputs are
rearranged into the batch format of the original filter:

- every propagation becomes a sample: ``gyro[:, k-1] = dtheta/h``, ``dt[k] = h``;
- acc and mag sit in the samples where the firmware performed the update and
  are 0 elsewhere (the filters skip an update when ``|v| < eps``);
- a mag arriving after the end-of-window acc update goes into an auxiliary
  sample (1 ns propagation), so the end-of-window state is compared at the
  instant the firmware recorded it;
- sample 0 is never used for an update: it only provides the nominal
  magnitudes ``nomAcc``, ``nomMag``.

Two modes, chosen by the firmware and read from the INIT packet:

- ``APP_COMPARE_ORIGINAL = 1``: C extensions disabled. Embedded vs ORIGINAL
  filter, must match up to float32 precision -> MATCH / DIVERGE. A second
  check replays the same calls through the C-mirror core.
- ``APP_COMPARE_ORIGINAL = 0``: full configuration. Embedded vs EXTENDED
  filter (all improvements) -> MATCH / DIVERGE, with a coverage line. The
  original is run too, to show how much the extensions change the estimate.

Criterion: differences in units of the filter's own standard deviation
``sqrt(diag P)``; MATCH if < 0.10 sigma and diag P relative error < 1e-3.
"""
from __future__ import annotations

import sys
from typing import Optional

import numpy as np

from .common import Struct, eul_zyx, quat2rotm, quat_angle_deg, rotm2quat
from .filters.embedded import (core_default_config, core_init, core_propagate,
                               core_update_acc, core_update_mag, iekf_attitude_emb)
from .filters.original import iekf_attitude
from .telemetry import (decode_init, decode_replay, decode_status, frame_counts,
                        parse_telemetry, read_capture)

TH = Struct(z=0.10, zBad=0.5, P=1e-3)


class CompareError(RuntimeError):
    """Capture unusable for the comparison (missing window 0, lost INIT, ...)."""


def _warn(msg):
    print(msg, file=sys.stderr)


def compare_embedded(capture_file, do_plot: bool = True, show: bool = True) -> Struct:
    """Compares the embedded IEKF (C, float32) with the reference filters on the
    inputs recorded in a raw capture (``.bin`` from ``capture`` or the viewer).

    Returns a Struct with ``t``, ``eulEmb``, ``eulMatlab`` (original),
    ``eulRef`` (reference of the mode), ``dAng``, ``dBias``, ``dP`` (vs
    original), ``ok``, ``extensions``, ``adapted`` (mode 1), ``extended``
    (mode 0) and ``init``. Angles are in degrees, arrays are 3 x n_windows.
    """
    data = read_capture(capture_file)
    frames, _ = parse_telemetry(data)
    types = np.array([fr.type for fr in frames])

    # ---------------- capture integrity ----------------
    c = frame_counts(frames)
    print(f"\nCapture: {len(data)} bytes, {len(frames)} valid frames")
    print(f"  STATE {c['STATE']}   INIT {c['INIT']}   STATUS {c['STATUS']}   REPLAY {c['REPLAY']}")
    i_st = np.flatnonzero(types == 3)
    upt = [decode_status(frames[k].payload).uptime_s for k in i_st]
    if i_st.size:
        S = decode_status(frames[i_st[-1]].payload)
        print(f"  firmware (last STATUS): frames dropped UART {S.uart_drop:.0f}, "
              f"telemetry queue {S.tlm_drop:.0f}")

    i_rep = np.flatnonzero(types == 4)
    if i_rep.size == 0:
        raise CompareError("No REPLAY packets: firmware built with APP_TLM_REPLAY = 1?")
    first = -1
    for k in i_rep[::-1]:                       # last window 0 = last filter start
        if decode_replay(frames[k].payload).win == 0:
            first = int(k)
            break
    if first < 0:
        raise CompareError("Window 0 missing: start the capture BEFORE resetting the board.")

    # the INIT must belong to the SAME boot (after the last reset, before window 0)
    reset_idx = -1
    for k in range(1, i_st.size):
        if i_st[k] < first and upt[k] < upt[k - 1]:
            reset_idx = int(i_st[k])
    cand = [k for k in range(len(frames)) if types[k] == 2 and reset_idx < k < first]
    if not cand:
        raise CompareError("The INIT of this boot was lost (UART saturated): without the right "
                           "initial conditions the comparison is meaningless. Repeat the capture.")
    i_init = cand[-1]
    I = decode_init(frames[i_init].payload)
    for k in range(first + 1, len(frames)):
        if types[k] == 2 and frames[k].payload != frames[i_init].payload:
            _warn("  WARNING: different INITs in the capture (reboot halfway?).")
            break

    # ---------------- contiguous windows from 0 ----------------
    reps = i_rep[i_rep >= first]
    decoded = [decode_replay(frames[k].payload) for k in reps]
    W = []
    first_gap = -1
    for r in decoded:
        if r.win != len(W):
            first_gap = len(W)
            break                               # the state cannot be rebuilt beyond a gap
        W.append(r)
    w_all = np.array([r.win for r in decoded])
    n_unique = np.unique(w_all).size
    n_lost = int(w_all.max()) + 1 - n_unique
    nW = len(W)
    print(f"  REPLAY windows from 0: {n_unique} received, {n_lost} lost")
    if first_gap >= 0:
        _warn(f"  Window {first_gap} lost: comparison limited to the first {nW} windows "
              f"({nW / 240:.1f} s).\n  The filter state cannot be rebuilt beyond a gap.")
    if nW < 240:
        raise CompareError("Less than 1 s of contiguous windows: capture too disturbed, repeat it.")

    # ---------------- inputs in the original's format ----------------
    nP = sum(o.op == "P" for w in W for o in w.ops)
    nMg = sum(o.op == "M" for w in W for o in w.ops)
    M = nP + nMg + 8
    gyro = np.zeros((3, M)); acc = np.zeros((3, M)); mag = np.zeros((3, M)); dt = np.zeros(M)
    acc_tr = np.zeros(M, dtype=bool); mag_xv = np.zeros(M)
    acc[:, 0] = [0.0, 0.0, I.nomAcc]            # only for the nominal magnitudes
    mag[:, 0] = [I.nomMag, 0.0, 0.0]
    s = 0
    win_sample = np.zeros(nW, dtype=int); t_win = np.zeros(nW); tacc = 0.0
    n_extra = n_tiny = 0
    for n, w in enumerate(W):
        for o in w.ops:
            if o.op == "P":
                s += 1
                gyro[:, s - 1] = o.v[0:3] / o.v[3]
                dt[s] = o.v[3]
                tacc += o.v[3]
            elif o.op == "A":
                acc[:, s] = o.v
                acc_tr[s] = o.trusted
            else:                               # 'M'
                # own sample if the current one is sample 0, already holds a mag,
                # or holds the acc update that closes a window
                if s == 0 or np.any(mag[:, s] != 0) or np.any(acc[:, s] != 0):
                    s += 1
                    gyro[:, s - 1] = 0.0
                    dt[s] = 1e-9
                    n_tiny += 1
                mag[:, s] = o.v[0:3]
                mag_xv[s] = o.v[3]
                n_extra += int(o.v[3] != 0)
        win_sample[n] = s
        t_win[n] = tacc
    S_ = s + 1
    gyro, acc, mag, dt = gyro[:, :S_], acc[:, :S_], mag[:, :S_], dt[:S_]
    acc_tr, mag_xv = acc_tr[:S_], mag_xv[:S_]

    R0 = quat2rotm(I.q0)
    if I.estimateBias:
        P0 = np.diag([I.p0att] * 3 + [I.p0bias] * 3)
    else:
        P0 = I.p0att * np.eye(3)
    opts = dict(R0=R0, b0=np.asarray(I.b0), estimateBias=I.estimateBias, P0=P0,
                sigmaGyro=I.sigmaGyro, sigmaBias=I.sigmaBias, sigmaAcc=I.sigmaAcc,
                sigmaMag=I.sigmaMag, adaptive=I.adaptive, accTol=I.accTol, magTol=I.magTol,
                tiltCompensatedMag=I.tiltComp)
    ext_active = (I.accNisGate > 0 or I.magNisGate > 0 or I.dipTol > 0 or I.resetJacobian
                  or I.decouple or n_extra > 0 or I.lockoutUpdates > 0)

    # ---------------- ORIGINAL filter ----------------
    Ro, bo, Po = iekf_attitude(gyro, acc, mag, dt, I.gRef, I.mRef, opts)
    orig = _window_metrics(W, win_sample, Ro, bo, Po)
    gated_a = np.array([w.accInfl > 2 for w in W])
    dAng, dBias, dP = orig.dAng, orig.dBias, orig.dP

    print(f"\nEmbedded (C, float32) vs ORIGINAL iekf_attitude: {nW} windows, {t_win[-1]:.1f} s")
    print(f"  inputs  : {S_} samples ({nP} propagations, {n_tiny} auxiliary mag samples)")
    print(f"  attitude: max {dAng.max():.2e} deg   RMS {np.sqrt(np.mean(dAng ** 2)):.2e} deg")
    print(f"  bias    : max {dBias.max():.2e} deg/s")
    print(f"  diag(P) : max relative error {dP.max():.2e}")
    zA, zB = orig.zAng.max(), orig.zBias.max()
    print(f"  in filter sigma: attitude max {zA:.3f}, bias max {zB:.3f} (threshold {TH.z:.2f})")

    ext = None
    adapt = None
    if not ext_active:
        ok = bool(zA < TH.z and zB < TH.z and dP.max() < TH.P)
        if ok:
            print(f"  RESULT: MATCH: differences < {100 * TH.z:.0f}% of the filter uncertainty,")
            print("          due only to float32 vs double. The C code implements the same equations.")
        else:
            _warn("  RESULT: DIVERGE beyond numerical precision.")
        adapt = _replay_adapted(I, W)
        print("  Check: same calls replayed with the core written after the C code:")
        print(f"    attitude max {adapt.dAng.max():.2e} deg, bias max {adapt.dBias.max():.2e} deg/s, "
              f"diag(P) max rel {adapt.dP.max():.2e}")
        if not ok:
            bad = int(np.flatnonzero((orig.zAng > TH.z) | (orig.zBias > TH.z) | (dP > TH.P))[0])
            print(f"  First window above threshold: {bad} (t = {t_win[bad]:.2f} s), operations: "
                  + "".join(o.op for o in W[bad].ops))
            if max(zA, zB) > TH.zBad and bad < 5:
                _warn("  DIAGNOSIS: large divergence from the start -> INIT/initial conditions.")
            elif max(zA, zB) > TH.zBad:
                _warn("  DIAGNOSIS: large divergence halfway -> inspect that window.")
            else:
                _warn("  DIAGNOSIS: small, growing divergence -> numerical drift.")
    else:
        print("  FULL configuration (extensions active): differences w.r.t. the ORIGINAL are expected.")
        print(f"    during acc inflation ({gated_a.sum()} windows): max "
              f"{(dAng[gated_a].max() if gated_a.any() else 0.0):.3f} deg")
        rest = dAng[~gated_a]
        print(f"    rest of the time                  : max {(rest.max() if rest.size else 0.0):.3f} deg, "
              f"RMS {(np.sqrt(np.mean(rest ** 2)) if rest.size else 0.0):.3f} deg")
        optsE = dict(opts, accNisGate=I.accNisGate, magNisGate=I.magNisGate, dipTol=I.dipTol,
                     decouple=I.decouple, resetJacobian=I.resetJacobian, nomAcc=I.nomAcc,
                     nomMag=I.nomMag, inflMax=I.inflMax, inflDecay=I.inflDecay,
                     lockoutUpdates=I.lockoutUpdates, graceUpdates=I.graceUpdates,
                     lockoutPBump=I.lockoutPBump, persistDecay=I.persistDecay,
                     persistRatio=I.persistRatio, persistMin=I.persistMin,
                     accTrusted=acc_tr, magExtraVar=mag_xv)
        Rx, bx, Px, info = iekf_attitude_emb(gyro, acc, mag, dt, I.gRef, I.mRef, optsE)
        ext = _window_metrics(W, win_sample, Rx, bx, Px)
        zAx, zBx = ext.zAng.max(), ext.zBias.max()
        print("\nEmbedded (C, float32) vs EXTENDED iekf_attitude_emb (double):")
        print(f"  attitude: max {ext.dAng.max():.2e} deg   RMS {np.sqrt(np.mean(ext.dAng ** 2)):.2e} deg")
        print(f"  bias    : max {ext.dBias.max():.2e} deg/s")
        print(f"  diag(P) : max relative error {ext.dP.max():.2e}")
        print(f"  in filter sigma: attitude max {zAx:.3f}, bias max {zBx:.3f} (threshold {TH.z:.2f})")
        print(f"  coverage: {int(acc_tr.sum())} acc updates stillness-trusted, {int(gated_a.sum())} "
              f"windows with acc inflated (max x{info.accInflMax:.0f}), {info.lockouts} lockout "
              f"events, {n_extra} mag updates with age variance")
        ext.info = info
        ok = bool(zAx < TH.z and zBx < TH.z and ext.dP.max() < TH.P)
        if ok:
            print("  RESULT: MATCH: the C code implements the extended filter (all improvements)")
            print("          up to float32 vs double.")
        else:
            _warn("  RESULT: DIVERGE beyond numerical precision.")
            bad = int(np.flatnonzero((ext.zAng > TH.z) | (ext.zBias > TH.z) | (ext.dP > TH.P))[0])
            print(f"  First window above threshold: {bad} (t = {t_win[bad]:.2f} s), operations: "
                  + "".join(o.op for o in W[bad].ops))

    # ZYX Euler angles [deg], reference = the filter the embedded one must match
    Rref = orig.R if ext is None else ext.R
    ref_name = "original iekf_attitude" if ext is None else "extended iekf_attitude_emb"
    eE = np.degrees(np.column_stack([eul_zyx(quat2rotm(w.q)) for w in W]))
    eO = np.degrees(np.column_stack([eul_zyx(orig.R[:, :, n]) for n in range(nW)]))
    eR = np.degrees(np.column_stack([eul_zyx(Rref[:, :, n]) for n in range(nW)]))
    dE = np.mod(eE - eR + 180.0, 360.0) - 180.0
    mx = np.abs(dE).max(axis=1)
    print(f"  angles  : max |embedded - {ref_name}| roll {mx[0]:.1e}, pitch {mx[1]:.1e}, yaw {mx[2]:.1e} deg")

    out = Struct(eulEmb=eE, eulMatlab=eO, eulRef=eR, t=t_win, dAng=dAng, dBias=dBias, dP=dP,
                 ok=ok, extensions=ext_active, adapted=adapt, extended=ext, init=I)
    if do_plot:
        _plot(out, W, orig, ext, Rref, ref_name, gated_a, show)
    return out


def _window_metrics(W, win_sample, Rx, bx, Px) -> Struct:
    """Embedded state vs a filter output at the end of every window."""
    n = len(W)
    m = Struct(dAng=np.zeros(n), dBias=np.zeros(n), dP=np.zeros(n), zAng=np.zeros(n),
               zBias=np.zeros(n), R=np.zeros((3, 3, n)), b=np.zeros((3, n)))
    for w in range(n):
        k = win_sample[w]
        m.R[:, :, w] = Rx[:, :, k]
        m.b[:, w] = bx[:, k]
        m.dAng[w] = quat_angle_deg(W[w].q, rotm2quat(Rx[:, :, k]))
        db = np.abs(bx[:, k] - W[w].b)
        m.dBias[w] = db.max() * 180.0 / np.pi
        pd = np.diag(Px[:, :, k])
        npd = pd.size
        m.dP[w] = np.max(np.abs(pd - W[w].Pd[:npd]) / np.maximum(np.abs(pd), 1e-30))
        m.zAng[w] = np.radians(m.dAng[w]) / np.sqrt(pd[0:3].min())
        if npd == 6:
            m.zBias[w] = np.max(db / np.sqrt(pd[3:6]))
    return m


def _replay_adapted(I, W) -> Struct:
    """Same calls replayed through the C-mirror core, window by window."""
    cfg = core_default_config()
    for k_cfg, k_init in (("sigmaGyro", "sigmaGyro"), ("sigmaBias", "sigmaBias"),
                          ("sigmaAcc", "sigmaAcc"), ("sigmaMag", "sigmaMag"),
                          ("accTol", "accTol"), ("magTol", "magTol"), ("p0Att", "p0att"),
                          ("p0Bias", "p0bias"), ("estimateBias", "estimateBias"),
                          ("adaptive", "adaptive"), ("tiltCompMag", "tiltComp"),
                          ("accNisGate", "accNisGate"), ("magNisGate", "magNisGate"),
                          ("inflDecay", "inflDecay"), ("dipTol", "dipTol"),
                          ("decouple", "decouple"), ("resetJacobian", "resetJacobian"),
                          ("inflMax", "inflMax"), ("lockoutUpdates", "lockoutUpdates"),
                          ("lockoutPBump", "lockoutPBump"), ("graceUpdates", "graceUpdates"),
                          ("persistDecay", "persistDecay"), ("persistRatio", "persistRatio"),
                          ("persistMin", "persistMin")):
        cfg[k_cfg] = I[k_init]
    seed = Struct(R0=quat2rotm(I.q0), b0=I.b0, gRef=I.gRef, mRef=I.mRef,
                  nomAcc=I.nomAcc, nomMag=I.nomMag)
    f = core_init(cfg, seed)
    n = len(W)
    A = Struct(dAng=np.zeros(n), dBias=np.zeros(n), dP=np.zeros(n))
    for w in range(n):
        for o in W[w].ops:
            if o.op == "P":
                core_propagate(f, o.v[0:3], o.v[3])
            elif o.op == "A":
                f.accTrusted = o.trusted
                core_update_acc(f, o.v)
            else:
                core_update_mag(f, o.v[0:3], o.v[3])
        A.dAng[w] = quat_angle_deg(W[w].q, rotm2quat(f.R))
        A.dBias[w] = np.abs(f.b - W[w].b).max() * 180.0 / np.pi
        pd = np.diag(f.P)
        A.dP[w] = np.max(np.abs(pd - W[w].Pd) / np.maximum(np.abs(pd), 1e-30))
    return A


def _break_jumps(y):
    """NaN where the angle jumps by more than 180 deg: no spurious vertical lines."""
    y = np.array(y, dtype=float)
    j = np.flatnonzero(np.abs(np.diff(y)) > 180)
    y[j + 1] = np.nan
    return y


def _continuous_quat(q):
    """q and -q are the same rotation: keep each sample close to the previous one."""
    q = q.copy()
    for n in range(1, q.shape[1]):
        if q[:, n] @ q[:, n - 1] < 0:
            q[:, n] = -q[:, n]
    return q


def _plot(out, W, orig, ext, Rref, ref_name, gated_a, show):
    import matplotlib.pyplot as plt

    t, eE, eR, eO = out.t, out.eulEmb, out.eulRef, out.eulMatlab
    dE = np.mod(eE - eR + 180.0, 360.0) - 180.0
    near_lock = np.abs(eE[1]) > 80
    names = ("roll", "pitch", "yaw")

    fig, axs = plt.subplots(4, 1, sharex=True, figsize=(10, 9), num=f"Angles: embedded vs {ref_name}")
    for i in range(3):
        ax = axs[i]
        ax.plot(t, _break_jumps(eE[i]), lw=1.6, label="embedded")
        ax.plot(t, _break_jumps(eR[i]), "--", lw=1.2, label=ref_name)
        if ext is not None:
            ax.plot(t, _break_jumps(eO[i]), ":", lw=1.0, label="original iekf_attitude")
        if i != 1 and near_lock.any():
            ax.plot(t[near_lock], eE[i, near_lock], "k.", ms=2, label="|pitch| > 80 deg (gimbal lock)")
        lim = 180 / (1 + (i == 1))
        ax.set_ylim(-lim, lim)
        ax.set_yticks(np.arange(-180, 181, 90)[(np.arange(-180, 181, 90) >= -lim) & (np.arange(-180, 181, 90) <= lim)])
        ax.set_ylabel(f"{names[i]} [deg]")
        ax.grid(True)
        if i == 0:
            ax.set_title(f"Embedded (C, float32, board) vs {ref_name} (double)")
            ax.legend(loc="best", fontsize=8)
    axs[3].plot(t, dE.T, lw=1.0)
    axs[3].legend(names, loc="best", fontsize=8)
    axs[3].set_ylabel("difference [deg]")
    axs[3].set_xlabel("filter time [s]")
    axs[3].set_title(f"Embedded - {ref_name}: max |diff| {np.abs(dE).max():.1e} deg")
    axs[3].grid(True)
    fig.tight_layout()

    qE = _continuous_quat(np.column_stack([w.q for w in W]))
    qR = np.column_stack([rotm2quat(Rref[:, :, n]) for n in range(len(W))])
    qO = np.column_stack([rotm2quat(orig.R[:, :, n]) for n in range(len(W))])
    qR = qR * np.sign(np.sum(qR * qE, axis=0))
    qO = qO * np.sign(np.sum(qO * qE, axis=0))
    fig, axs = plt.subplots(4, 1, sharex=True, figsize=(10, 8), num=f"Quaternion: embedded vs {ref_name}")
    for i, nm in enumerate("wxyz"):
        ax = axs[i]
        ax.plot(t, qE[i], lw=1.6, label="embedded")
        ax.plot(t, qR[i], "--", lw=1.2, label=ref_name)
        if ext is not None:
            ax.plot(t, qO[i], ":", lw=1.0, label="original iekf_attitude")
        ax.set_ylabel(f"q_{nm}")
        ax.set_ylim(-1, 1)
        ax.grid(True)
        if i == 0:
            ax.set_title("Body->world quaternion (continuous, no singularity)")
            ax.legend(loc="best", fontsize=8)
    axs[3].set_xlabel("filter time [s]")
    fig.tight_layout()

    fig, axs = plt.subplots(2, 1, sharex=True, figsize=(10, 7), num="Embedded vs reference")
    ax = axs[0]
    ax.semilogy(t, np.maximum(out.dAng, 1e-9), lw=1.1, label="vs original")
    if ext is not None:
        ax.semilogy(t, np.maximum(ext.dAng, 1e-9), lw=1.1, label="vs extended")
    if gated_a.any():
        yl = ax.get_ylim()
        ax.plot(t[gated_a], np.full(gated_a.sum(), yl[1]), "r.", ms=3, label="acc de-weighted")
    ax.set_ylabel("attitude difference [deg]")
    ax.grid(True)
    ax.legend(loc="best", fontsize=8)
    ax = axs[1]
    b_emb = np.degrees(np.column_stack([w.b for w in W]))
    b_ref = np.degrees(orig.b if ext is None else ext.b)
    for i, c in enumerate(("C0", "C1", "C2")):
        ax.plot(t, b_emb[i], color=c, lw=1.1, label=f"emb {'xyz'[i]}")
        ax.plot(t, b_ref[i], "--", color=c, lw=1.0, label=f"ref {'xyz'[i]}")
    ax.set_xlabel("filter time [s]")
    ax.set_ylabel("bias [deg/s]")
    ax.grid(True)
    ax.legend(loc="best", fontsize=8, ncol=2)
    fig.tight_layout()
    if show:
        plt.show()
