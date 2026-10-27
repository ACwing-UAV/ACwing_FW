"""Viewer (headless replay), log I/O, magnetometer calibration, sniffer, CLI."""
import numpy as np

from acwing_tools.__main__ import main
from acwing_tools.capture import tlm_sniffer
from acwing_tools.logio import load_log, save_log
from acwing_tools.magcalib import calib_mag_axes, fit_hard_iron, signed_permutations
from acwing_tools.serial_io import FileSource
from acwing_tools.simulate import write_stream
from acwing_tools.viewer import LiveViewer


def _replay_log(tmp_path, **kw):
    sim = write_stream(str(tmp_path / "sim.bin"), **kw)
    v = LiveViewer(FileSource(sim, rate=None, chunk=777), log_dir=str(tmp_path),
                   stop_at_eof=True, show=False)
    L = v.run()
    return v, L, sim


def test_viewer_replay_log_and_bin(tmp_path):
    v, L, sim = _replay_log(tmp_path, duration=10.0)
    assert L.seq.size == 2400 and v.n_lost == 0 and L.init is not None
    assert len(L.status) == 101
    assert open(v.bin_file, "rb").read() == open(sim, "rb").read()
    L2 = load_log(v.log_file)
    assert np.allclose(L2.q, L.q) and L2.init.fs == 240 and len(L2.status) == len(L.status)
    assert "Filter" not in v.status_text() and "roll" in v.status_text()


def test_log_roundtrip(tmp_path):
    _, L, _ = _replay_log(tmp_path, duration=2.0)
    L.resetAt = np.array([10.0])
    save_log(str(tmp_path / "x.mat"), L)
    L2 = load_log(str(tmp_path / "x.mat"))
    for k in ("seq", "t_us", "q", "bias", "acc", "gyro", "mag", "flags"):
        assert np.allclose(L2[k], L[k])
    assert L2.resetAt.tolist() == [10.0]


def test_calib_mag_axes_recovers_map_and_offset(tmp_path):
    M = np.array([[0.0, 1, 0], [-1, 0, 0], [0, 0, 1]])
    _, L, _ = _replay_log(tmp_path, duration=40.0, axis_map=M, hard_iron=(-20.0, 35.0, 8.0))
    out = calib_mag_axes(L, verbose=False)
    assert len(signed_permutations()) == 24
    assert np.array_equal(out.M, M)
    assert np.allclose(out.hardIronBody, [-20, 35, 8], atol=0.2)
    assert out.ratio > 3


def test_fit_hard_iron_on_body_frame_log(tmp_path):
    _, L, _ = _replay_log(tmp_path, duration=40.0, axis_map=np.eye(3), hard_iron=(10.0, -5.0, 3.0))
    offset, radius = fit_hard_iron(L, do_plot=True, show=False, verbose=False)
    assert np.allclose(offset, [10, -5, 3], atol=0.5)
    assert abs(radius - np.hypot(20, 40)) < 0.5


def test_sniffer_and_cli(tmp_path, capsys):
    sim = write_stream(str(tmp_path / "s.bin"), duration=2.0)
    S = tlm_sniffer("file", data=open(sim, "rb").read())
    assert S is not None and S.running == 1
    assert main(["info", sim]) == 0
    assert "STATE 480" in capsys.readouterr().out
    assert main(["sniff", "--file", sim]) == 0
    assert tlm_sniffer("file", data=b"\xA5" * 300) is None          # stuck DMA diagnosis
    assert "distinct byte values" in capsys.readouterr().out


def test_mag_check_detects_stale_hard_iron(tmp_path):
    """Log of a firmware whose MAG_HARD_IRON no longer matches: the residual offset
    must be found and the corrected value must be the absolute one. Same numbers as
    00 - Matlab/MagCalib/magCheck.m on the same log (checked in Octave)."""
    from acwing_tools.magcalib import mag_check
    hi_true = np.array([30.0, -12.0, 55.0])
    _, L, _ = _replay_log(tmp_path, duration=40.0, axis_map=np.eye(3), hard_iron=tuple(hi_true))
    fw = np.array([5.0, 0.0, 0.0])                     # stale value in the firmware
    L.mag = L.mag - fw[:, None]                       # what that firmware would log
    out = mag_check(L, hard_iron_fw=fw, verbose=False)
    assert np.allclose(out.residualOffset, hi_true - fw, atol=0.5)
    assert np.allclose(out.newHardIron, hi_true, atol=0.5)
    assert abs(out.B - np.hypot(20, 40)) < 0.5
