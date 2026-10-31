"""Flight logs: decoder against a log written by the firmware's own code (host
test ``00 - FC_FW/Tests/host/test_log.c`` → data/flightlog_sample.bin), and
the USB download client against an emulated drone."""
import os
import struct

import numpy as np
import pytest

from acwing_tools import flightlog as fl
from acwing_tools.__main__ import main
from acwing_tools.telemetry import encode_frame, parse_telemetry

DATA = os.path.join(os.path.dirname(__file__), "data", "flightlog_sample.bin")


def test_crc8():
    assert fl.crc8(b"123456789") == 0xF4            # CRC-8/SMBUS check value


def test_decode_sample():
    L = fl.decode_log(open(DATA, "rb").read())
    assert L.bad == 0 and L.unknown == 0 and L.sectors == 6
    assert L.info.fw == "0.5.0" and L.info.hw_rev == "N" and L.info.format == 1
    assert 345 <= L.att.n <= 360 and L.vert.n == L.att.n
    assert np.all(np.diff(L.att.t) > 0) and np.allclose(np.diff(L.att.t), 0.02)
    assert np.allclose(L.att.q[0], 1.0) and np.allclose(L.att.acc[2], 1.0)
    # the test writes gyro x = (t/20 % 1000) / 1000 rad/s
    expect = (np.round(L.att.t * 1000) / 20 % 1000) / 1000
    assert np.allclose(L.att.gyro[0], expect, atol=1e-3)
    assert np.allclose(L.vert.h, 1.0) and np.allclose(L.vert.baro_pa, 101000) and np.all(np.isnan(L.vert.tof))
    assert L.flight.n >= 68 and L.power.n >= 68 and L.drop.n == 0
    assert np.allclose(L.power.vbat, 3.9) and np.all(L.power.present == 1) and np.all(L.power.cells == 1)
    assert L.event.name == ["ARMED", "DISARMED"]
    assert L.flight.armed.max() == 1 and L.flight.armed[-1] == 0


def test_decode_damaged():
    d = bytearray(open(DATA, "rb").read())
    d[fl.SECTOR + 100] ^= 0x55                       # a record in sector 1
    d[3 * fl.SECTOR + 2] ^= 0x01                     # header of sector 3
    L = fl.decode_log(bytes(d))
    L0 = fl.decode_log(open(DATA, "rb").read())
    assert L.bad == 2 and L.sectors == L0.sectors - 1
    assert 0 < L.att.n < L0.att.n and np.all(np.diff(L.att.t) > 0)


# ---------------------------------------------------------------- emulated drone
class FakeFC:
    """Answers LOG_LIST / LOG_READ / LOG_ERASE like log_task.c, through bytes."""

    def __init__(self, logs, armed=False):
        self.logs = dict(logs)
        self.armed = armed
        self.rx = b""
        self.out = bytearray()
        self.reads = 0

    def write(self, b):
        frames, self.rx = parse_telemetry(self.rx + bytes(b))
        for f in frames:
            assert f.type == fl.FT_CMD
            self.cmd(f.payload[0], f.payload[1], f.payload[2:])

    def ack(self, op, seq, res, why=0):
        self.out += encode_frame(fl.FT_ACK, bytes([op, seq, res, why]))

    def cmd(self, op, seq, a):
        if op == fl.OP_LOG_LIST:
            ids = sorted(self.logs)
            for k, i in enumerate(ids):
                n = len(self.logs[i])
                self.out += encode_frame(fl.FT_LOG_ENTRY, struct.pack("<HBBBHII", i, k, len(ids), 0,
                                                                      n // 4096, 1000 * i, n))
            self.ack(op, seq, 0)                          # after the entries, as log_task.c
        elif op == fl.OP_LOG_READ:
            if self.armed:
                return self.ack(op, seq, 1, 8)
            i, off, n = struct.unpack("<HII", a)
            if i not in self.logs:
                return self.ack(op, seq, 3, 14)
            self.reads += 1
            d = self.logs[i][off:off + n]
            for k in range(0, len(d), 240):
                self.out += encode_frame(fl.FT_LOG_DATA, struct.pack("<HI", i, off + k) + d[k:k + 240])
            self.ack(op, seq, 0)
        elif op == fl.OP_LOG_ERASE:
            self.ack(op, seq, 5)
            self.logs.clear()
            self.ack(op, seq, 0)
        else:
            self.ack(op, seq, 4)

    def read(self, n):
        n = min(n, 1000)                              # the USB delivers in pieces
        d, self.out = bytes(self.out[:n]), self.out[n:]
        return d

    def close(self):
        pass


def test_link_list_read_erase(tmp_path):
    sample = open(DATA, "rb").read()
    dev = FakeFC({3: sample, 7: bytes(range(256)) * 32})
    link = fl.FcLink(dev, timeout=0.5)
    logs = link.list_logs()
    assert [e.id for e in logs] == [3, 7] and logs[0].bytes == len(sample) and logs[1].t0_ms == 7000
    got = link.read_log(3, len(sample), chunk=10000)
    assert got == sample and dev.reads == 3          # 24576 B in 10000-byte commands
    files = fl.download_all(link, str(tmp_path))
    assert [os.path.basename(f) for f in files] == ["acwing_flight_00003.bin", "acwing_flight_00007.bin"]
    assert open(files[1], "rb").read() == bytes(range(256)) * 32
    link.erase()
    assert link.list_logs() == []


def test_link_errors():
    link = fl.FcLink(FakeFC({1: b"\xff" * 4096}, armed=True), timeout=0.3)
    with pytest.raises(fl.LinkError, match="ARMED"):
        link.read_log(1, 4096)
    link = fl.FcLink(FakeFC({}), timeout=0.3)
    with pytest.raises(fl.LinkError, match="OUT_OF_RANGE"):
        link.read_log(9, 4096)

    class Silent:
        def write(self, b): pass
        def read(self, n): return b""
    with pytest.raises(fl.LinkError, match="no answer"):
        fl.FcLink(Silent(), timeout=0.2).list_logs()


def test_cli_decode(tmp_path, capsys):
    assert main(["log", "decode", DATA, "--csv", str(tmp_path), "--mat", str(tmp_path)]) == 0
    out = capsys.readouterr().out
    assert "ARMED" in out and "firmware 0.5.0" in out
    names = sorted(os.listdir(tmp_path))
    assert "flightlog_sample_att.csv" in names and "flightlog_sample.mat" in names
    rows = open(tmp_path / "flightlog_sample_event.csv").read().splitlines()
    assert rows[0].startswith("t_s,level,code,arg") and rows[1].endswith("ARMED")
    import scipy.io as sio
    F = sio.loadmat(tmp_path / "flightlog_sample.mat", squeeze_me=True, struct_as_record=False)["F"]
    assert F.att.q.shape[0] == 4 and F.info.fw == "0.5.0"
    assert main(["log", "list"]) == 2                # no port
