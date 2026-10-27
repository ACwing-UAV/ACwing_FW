"""Numerical parity of the Python filters with the MATLAB filters (reference made by
tests/reference/make_reference.m with the MATLAB code)."""
import os

import numpy as np
import pytest
import scipy.io as sio

from conftest import DATA
from acwing_tools.common import quat2rotm, rotm2quat
from acwing_tools.filters import exp_so3, iekf_attitude, iekf_attitude_emb, log_so3


@pytest.fixture(scope="module")
def ref():
    return sio.loadmat(os.path.join(DATA, "filter_reference.mat"))


@pytest.fixture(scope="module")
def inp():
    return sio.loadmat(os.path.join(DATA, "filter_inputs.mat"), squeeze_me=True)


def _opts(inp):
    return dict(R0=quat2rotm(inp["q0"]), b0=inp["b0"], P0=np.diag([1e-2] * 3 + [1e-4] * 3),
                sigmaAcc=0.03, adaptive=True, tiltCompensatedMag=False)


G, M = [0, 0, 1], [0.5, 0, -0.85]


def test_so3_maps():
    rng = np.random.default_rng(1)
    for _ in range(50):
        u = rng.standard_normal(3)
        phi = u / np.linalg.norm(u) * rng.uniform(0, 3.1)          # angle below pi
        assert np.allclose(log_so3(exp_so3(phi)), phi, atol=1e-9)
    phi = np.array([0, 0, np.pi - 1e-8])
    assert np.isclose(np.linalg.norm(log_so3(exp_so3(phi))), np.pi, atol=1e-6)
    q = np.array([0.5, 0.5, -0.5, 0.5])
    assert np.allclose(rotm2quat(quat2rotm(q)), q)


def test_original_matches_matlab(inp, ref):
    R, b, P = iekf_attitude(inp["gyro"], inp["acc"], inp["mag"], inp["dt"], G, M, _opts(inp))
    assert np.abs(R - ref["Ro"]).max() < 1e-11
    assert np.abs(b - ref["bo"]).max() < 1e-12
    assert np.abs(P - ref["Po"]).max() < 1e-14


def test_extended_matches_matlab(inp, ref):
    o = dict(_opts(inp), accNisGate=4, magNisGate=9, inflTau=0.5, dipTol=np.deg2rad(5),
             lockoutTime=0.1, nomAcc=1.0, nomMag=0.9, inflMax=100,
             accTrusted=inp["accTr"], magExtraVar=inp["magXV"])
    R, b, P, info = iekf_attitude_emb(inp["gyro"], inp["acc"], inp["mag"], inp["dt"], G, M, o)
    assert np.abs(R - ref["Rx"]).max() < 1e-11
    assert np.abs(b - ref["bx"]).max() < 1e-12
    assert np.abs(P - ref["Px"]).max() < 1e-14
    assert info.lockouts == int(ref["lockouts"].item()) and info.accInflMax == pytest.approx(float(ref["accInflMax"].item()))
    o2 = dict(o, decouple=True, tiltCompensatedMag=True, estimateBias=False, P0=0.01)
    R2, b2, P2, _ = iekf_attitude_emb(inp["gyro"], inp["acc"], inp["mag"], inp["dt"], G, M, o2)
    assert np.abs(R2 - ref["Ry"]).max() < 1e-11 and P2.shape == ref["Py"].shape
    assert np.abs(P2 - ref["Py"]).max() < 1e-14


def test_extended_reduces_to_original_with_extensions_off(inp):
    o = dict(_opts(inp), resetJacobian=False)
    Ro, bo, Po = iekf_attitude(inp["gyro"], inp["acc"], inp["mag"], inp["dt"], G, M, o)
    Re, be, Pe, _ = iekf_attitude_emb(inp["gyro"], inp["acc"], inp["mag"], inp["dt"], G, M, o)
    # same equations up to the C-style numerics (Taylor exp, re-orthonormalisation)
    assert np.abs(Re - Ro).max() < 1e-9 and np.abs(be - bo).max() < 1e-9
