"""Embedded-vs-reference comparison on captures produced by the firmware C code
(compiled on a PC with a simulated IMU: same frames as the board)."""
import os
import struct

import numpy as np
import pytest
import scipy.io as sio

from conftest import DATA
from acwing_tools.compare import compare_embedded
from acwing_tools.telemetry import encode_frame, parse_telemetry


@pytest.fixture(scope="module")
def ref():
    return sio.loadmat(os.path.join(DATA, "compare_reference.mat"), squeeze_me=True,
                       struct_as_record=False)


def test_mode1_original_match(ref):
    out = compare_embedded(os.path.join(DATA, "capture_mode1.bin"), do_plot=False)
    r = ref["capture_mode1"]
    assert out.ok and not out.extensions
    np.testing.assert_allclose(out.dAng, r.dAng, rtol=1e-6, atol=1e-9)
    np.testing.assert_allclose(out.dP, r.dP, rtol=1e-6, atol=1e-12)
    np.testing.assert_allclose(out.adapted.dAng, r.adaptdAng, rtol=1e-6, atol=1e-9)


def test_mode0_extended_match_with_lockout(ref):
    out = compare_embedded(os.path.join(DATA, "capture_mode0.bin"), do_plot=True, show=False)
    r = ref["capture_mode0"]
    assert out.ok and out.extensions
    assert out.extended.info.lockouts == 1
    np.testing.assert_allclose(out.extended.dAng, r.extdAng, rtol=1e-6, atol=1e-9)
    np.testing.assert_allclose(out.dAng, r.dAng, rtol=1e-6, atol=1e-9)
    np.testing.assert_allclose(out.eulRef, r.eulRef, atol=1e-6)


def test_negative_control_detects_a_wrong_parameter(tmp_path):
    """A 20 % change of the dip tolerance in INIT must be detected (DIVERGE)."""
    data = open(os.path.join(DATA, "capture_mode0.bin"), "rb").read()
    frames, _ = parse_telemetry(data)
    out = bytearray()
    for fr in frames:
        p = bytearray(fr.payload)
        if fr.type == 2:
            dip = struct.unpack_from("<f", p, 27 * 4)[0]
            struct.pack_into("<f", p, 27 * 4, dip * 1.2)
        out += encode_frame(fr.type, bytes(p))
    path = tmp_path / "tampered.bin"
    path.write_bytes(bytes(out))
    res = compare_embedded(str(path), do_plot=False)
    assert not res.ok
