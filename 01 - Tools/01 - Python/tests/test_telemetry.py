"""Frame format, stream parser and packet decoders."""
import struct

import numpy as np

from acwing_tools.simulate import pack_init, pack_state, pack_status, synthetic_stream
from acwing_tools.telemetry import (crc16ccitt, decode_init, decode_replay, decode_state,
                                  decode_status, encode_frame, frame_counts, parse_telemetry,
                                  status_diagnosis)


def test_crc_check_value():
    assert crc16ccitt(b"123456789") == 0x29B1          # CRC-16/CCITT-FALSE check value


def test_parse_with_garbage_and_split_chunks():
    stream = synthetic_stream(duration=3.0)
    ref, rest = parse_telemetry(stream)
    assert rest == b""
    cut = len(b"".join(encode_frame(f.type, f.payload) for f in ref[:40]))   # a frame boundary
    noisy = b"\x00\xA5\x13" + stream[:cut] + b"\xA5\x5A\x01\x47garbage" + stream[cut:]
    rng = np.random.default_rng(0)
    frames, buf, i = [], b"", 0
    while i < len(noisy):                               # random pieces, as from a serial port
        n = int(rng.integers(1, 300))
        buf += noisy[i:i + n]
        i += n
        fr, buf = parse_telemetry(buf)
        frames += fr
    assert [(f.type, f.payload) for f in frames] == [(f.type, f.payload) for f in ref]


def test_skip_types():
    stream = encode_frame(4, b"\x01" * 100) + encode_frame(3, pack_status(1000, 10))
    fr, _ = parse_telemetry(stream, (4,))
    assert [f.type for f in fr] == [3]
    assert frame_counts(parse_telemetry(stream)[0]) == {"STATE": 0, "INIT": 0, "STATUS": 1, "REPLAY": 1}


def test_corrupted_frame_is_dropped():
    good = encode_frame(3, pack_status(1000, 10))
    bad = bytearray(good)
    bad[10] ^= 0xFF
    fr, _ = parse_telemetry(bytes(bad) + good)
    assert len(fr) == 1 and fr[0].payload == good[4:-2]


def test_decode_state_init_status():
    q = [0.9, 0.1, -0.2, 0.3]
    p = pack_state(1234, 5678, q, [1, 2, 3], [4, 5, 6], [7, 8, 9], [10, 11, 12], 0x25)
    s = decode_state(p)
    assert s.seq == 1234 and s.t_us == 5678 and s.flags == 0x25
    assert np.allclose(s.q, q, atol=1e-7) and np.allclose(s.mag, [10, 11, 12])
    I = decode_init(pack_init([1, 0, 0, 0], [0, 0, 1], [0.6, 0, -0.8], [0.01, 0.02, 0.03]))
    assert I.fs == 240 and I.lockoutUpdates == 120 and I.graceUpdates == 60
    assert I.estimateBias and I.adaptive and not I.tiltComp and I.resetJacobian
    assert np.allclose(I.mRef, [0.6, 0, -0.8], atol=1e-7)
    S = decode_status(pack_status(2500, 100))
    assert S.uptime_s == 2.5 and S.running == 1 and S.calib_count == 100
    assert status_diagnosis(S).startswith("Filter running")
    S.imu_init = -3
    assert "sensor hub" in status_diagnosis(S)


def test_decode_replay():
    ops = (b"P" + struct.pack("<4f", 0.1, 0.2, 0.3, 1 / 960) + b"A" + struct.pack("<3f", 0, 0, 1)
           + b"\x01" + b"M" + struct.pack("<4f", 20, 0, -40, 1e-4))
    tail = struct.pack("<15f", 1, 0, 0, 0, 0.1, 0.2, 0.3, 1, 2, 3, 4, 5, 6, 7, 8)
    R = decode_replay(struct.pack("<IBB", 42, 3, 0) + ops + tail)
    assert R.win == 42 and [o.op for o in R.ops] == ["P", "A", "M"]
    assert R.ops[1].trusted and np.allclose(R.ops[0].v[3], 1 / 960)
    assert np.allclose(R.Pd, [1, 2, 3, 4, 5, 6]) and R.accInfl == 7 and R.magInfl == 8
