"""Generates tests/data/filter_inputs.mat: synthetic inputs for the filter parity test.

Run from 02 - Tools/01 - Python:  python tests/reference/make_inputs.py
Then, in MATLAB or Octave, run tests/reference/make_reference.m to compute the
reference outputs with the MATLAB filters (tests/data/filter_reference.mat).
"""
import os

import numpy as np
import scipy.io as sio

HERE = os.path.dirname(os.path.abspath(__file__))
rng = np.random.default_rng(3)
N = 600
t = np.arange(N) / 240.0
dt = np.full(N, 1 / 240.0)
dt[100] = 0.0                                                 # invalid step -> median
gyro = np.vstack([0.8 * np.sin(1.3 * t), 0.5 * np.cos(0.7 * t), 1.1 * np.sin(0.4 * t + 1)])
gyro += 0.01 * rng.standard_normal((3, N))
acc = np.vstack([0.05 * np.sin(t), 0.1 * np.cos(2 * t), np.ones(N)]) + 0.02 * rng.standard_normal((3, N))
acc[:, 300:420] += 0.4 * rng.standard_normal((3, 120))        # shaking -> NIS inflation / lockout
acc[:, 50:60] = 0.0                                           # skipped acc updates
mag = np.vstack([0.4 * np.ones(N), 0.1 * np.sin(t), -0.8 * np.ones(N)]) + 0.02 * rng.standard_normal((3, N))
mag[:, ::3] = 0.0                                             # mag at 2/3 of the samples
accTr = (rng.random(N) < 0.1).astype(float)
magXV = 1e-4 * rng.random(N)
sio.savemat(os.path.join(HERE, "..", "data", "filter_inputs.mat"),
            dict(gyro=gyro, acc=acc, mag=mag, dt=dt, accTr=accTr, magXV=magXV,
                 q0=np.array([0.9, 0.1, -0.2, 0.3]), b0=np.array([0.01, -0.02, 0.005])))
print("written tests/data/filter_inputs.mat")
