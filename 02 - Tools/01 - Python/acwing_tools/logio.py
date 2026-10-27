"""Viewer logs: the ``L`` struct saved as ``.mat``, shared with the MATLAB tools.

The live viewers (MATLAB ``acwingLiveViewer.m`` and Python ``viewer.py``) save
the same structure, variable ``L`` in a ``.mat`` file:

==========  =======  ==================================================
field       shape    content
==========  =======  ==================================================
seq         1 x N    filter sample index (wrap-around removed)
t_us        1 x N    filter time [us]
q           4 x N    attitude quaternion [w x y z], body -> world
bias        3 x N    gyro bias [rad/s]
acc         3 x N    accelerometer window mean [g]
gyro        3 x N    gyro window mean [rad/s]
mag         3 x N    magnetometer, body frame, hard-iron removed [uT]
flags       1 x N    STATE flags (telemetry.F_*)
t_pc        1 x N    PC time since the viewer started [s]
init        struct   last INIT packet (empty if none)
status      struct   STATUS packets (10 Hz), struct array
resetAt     1 x K    sample indices (1-based) after a board reboot
==========  =======  ==================================================

So a log recorded with one language can be analysed with the other.
"""
from __future__ import annotations

import numpy as np
import scipy.io as sio

from .common import Struct

ARRAY_FIELDS = ("seq", "t_us", "q", "bias", "acc", "gyro", "mag", "flags", "t_pc")


def _to_struct(obj):
    """scipy ``mat_struct`` / arrays of them -> Struct / list of Struct."""
    if isinstance(obj, sio.matlab.mat_struct):
        return Struct({k: _to_struct(getattr(obj, k)) for k in obj._fieldnames})
    if isinstance(obj, np.ndarray) and obj.dtype == object:
        return [_to_struct(x) for x in obj.ravel()]
    return obj


def load_log(path) -> Struct:
    """Loads a viewer log (``.mat`` written by MATLAB or Python) as a Struct.

    Array fields are returned with the MATLAB layout (``q`` is 4 x N, ``seq``
    is 1-D); ``init`` is None when absent; ``status`` is a list of Struct.
    """
    S = sio.loadmat(path, squeeze_me=True, struct_as_record=False)
    if "L" not in S:
        raise ValueError(f"{path}: no variable 'L' (not a viewer log)")
    L = _to_struct(S["L"])
    for k in ("seq", "t_us", "flags", "t_pc"):
        L[k] = np.atleast_1d(np.asarray(L[k], dtype=float))
    for k, rows in (("q", 4), ("bias", 3), ("acc", 3), ("gyro", 3), ("mag", 3)):
        a = np.asarray(L[k], dtype=float)
        L[k] = a.reshape(rows, -1) if a.size else np.zeros((rows, 0))
    init = L.get("init")
    L.init = init if isinstance(init, Struct) else None
    st = L.get("status")
    if isinstance(st, Struct):
        L.status = [st]
    elif not isinstance(st, list):
        L.status = []
    L.resetAt = np.atleast_1d(np.asarray(L.get("resetAt", []), dtype=float))
    return L


def _struct_array(items):
    """List of dicts -> numpy record array saved by scipy as a MATLAB struct array."""
    if not items:
        return np.zeros((0, 0))
    names = list(items[0].keys())
    arr = np.empty((1, len(items)), dtype=[(n, object) for n in names])
    for i, it in enumerate(items):
        for n in names:
            v = it[n]
            arr[0, i][n] = float(v) if isinstance(v, (bool, np.bool_)) else v
    return arr


def _matlab_value(v):
    if isinstance(v, (bool, np.bool_)):
        return float(v)
    return v


def save_log(path, L) -> None:
    """Saves a log Struct with the layout of ``acwingLiveViewer.m`` (variable ``L``)."""
    out = {k: np.atleast_2d(np.asarray(L[k], dtype=float)) for k in ARRAY_FIELDS}
    out["init"] = ({k: _matlab_value(v) for k, v in L.init.items()}
                   if L.get("init") else np.zeros((0, 0)))
    out["status"] = _struct_array(L.get("status") or [])
    out["resetAt"] = np.atleast_2d(np.asarray(L.get("resetAt", []), dtype=float))
    sio.savemat(path, {"L": out}, do_compression=True)
