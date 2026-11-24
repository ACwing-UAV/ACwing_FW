"""acwing-tools cmd: command encoding, ACK matching, the suites against a fake drone that follows the firmware rules
(ACK cache per link, USB ground-link rules, INVALID for an empty payload, UNSUPPORTED for an unknown op)."""
import struct

import pytest

from acwing_tools import cmdtest as ct
from acwing_tools.flightlog import FT_ACK, FT_CMD
from acwing_tools.telemetry import encode_frame, parse_telemetry


class FakeDrone:
    """Answers like app_task.c / fc_core.c for what the suites look at."""

    def __init__(self, kind="usb", flight_hz=10.0):
        self.kind, self.out, self.rest = kind, bytearray(), b""
        self.last = {}                     # op -> (seq, ack): the ACK cache
        self.effects = []
        self.params = {1: 1.0, 70: 1.0}
        self.flight_hz, self.t_flight = flight_hz, 0.0

    # port interface
    def write(self, b):
        if self.kind == "usb":
            frames, self.rest = parse_telemetry(self.rest + bytes(b))
            for f in frames:
                if f.type == FT_CMD:
                    self.cmd(f.payload)
        else:
            self.cmd(bytes(b))

    def read(self, n):
        import time
        now = time.monotonic()
        if self.flight_hz and now - self.t_flight >= 1.0 / self.flight_hz:
            self.t_flight = now
            self.out += encode_frame(ct.FT["flight"], bytes(10))
        d, self.out = bytes(self.out[:n]), self.out[n:]
        return d

    def close(self):
        pass

    # drone
    def ack(self, op, seq, res, why=0):
        self.out += encode_frame(FT_ACK, bytes([op, seq, res, why]))
        return (res, why)

    def cmd(self, c):
        op, seq, p = c[0], c[1], c[2:]
        if op in ct.NO_ACK:
            return
        if op in self.last and self.last[op][0] == seq:
            self.ack(op, seq, *self.last[op][1])
            return
        a = self.run(op, seq, p)
        self.last[op] = (seq, a)

    def run(self, op, seq, p):
        if self.kind == "usb" and op not in ct.USB_ALLOWED:
            return self.ack(op, seq, 4 if op < 0x10 else 1, 0 if op < 0x10 else 18)
        if op not in ct.OP_NAMES:
            return self.ack(op, seq, 4)
        if not p and op not in ct.EMPTY_OK:
            return self.ack(op, seq, 3)
        self.effects.append(op)
        if op in (ct.OP["param-get"], ct.OP["param-set"]):
            i = struct.unpack_from("<H", p)[0]
            if op == ct.OP["param-set"]:
                self.params[i] = struct.unpack_from("<f", p, 2)[0]
            self.out += encode_frame(ct.FT["param"], struct.pack("<HfBHH", i, self.params.get(i, 0.0), 0, 0, 1))
        if op == ct.OP["arm"]:
            self.out += encode_frame(ct.FT["event"], struct.pack("<BHiB", 0, 1, 0, 0))
        if op == ct.OP["stream-rates"] and p[0]:
            self.flight_hz = p[0]
        return self.ack(op, seq, 0)


def link(kind="usb", **kw):
    d = FakeDrone(kind, **kw)
    return ct.CmdLink(d, kind, raw=(kind == "ble"), wait=0.3), d


def test_encode_named_commands():
    assert ct.encode_args("arm", []) == (0x12, b"\xA5\x5A")
    assert ct.encode_args("takeoff", ["1.5"]) == (0x14, struct.pack("<H", 150))
    assert ct.encode_args("param-set", ["70", "2"]) == (0x21, struct.pack("<Hf", 70, 2.0))
    assert ct.encode_args("param-get", ["logMode"]) == (0x20, struct.pack("<H", 70))   # name from the app's ids.ts
    assert ct.encode_args("cal-start", ["gyro"]) == (0x30, bytes([3, 0]))
    assert ct.encode_args("motor-test", ["1", "10", "500"]) == (0x36, struct.pack("<BBHB", 1, 10, 500, 0x50))
    assert ct.encode_args("request", ["caps"]) == (0x41, b"\x05")
    assert ct.encode_args("raw", ["0x21", "4600", "00002041"]) == (0x21, bytes.fromhex("460000002041"))
    with pytest.raises(ValueError):
        ct.encode_args("param-set", ["70"])
    with pytest.raises(ValueError):
        ct.encode_args("nope", [])


def test_send_matches_the_ack_of_its_seq_and_fixed_seq():
    L, d = link("usb")
    r = L.send(ct.OP["param-get"], struct.pack("<H", 70))
    assert r.acks == [(0, 0)] and any(f.type == ct.FT["param"] for f in r.frames)
    r = L.send(ct.OP["param-get"], struct.pack("<H", 70), seq=7, repeat=2)
    assert r.seq == 7 and r.acks == [(0, 0), (0, 0)]
    assert d.effects.count(ct.OP["param-get"]) == 2                       # the second one came from the cache


def test_usb_frames_and_ble_raw_writes():
    L, d = link("ble")
    r = L.send(ct.OP["arm"], ct.ARM_MAGIC)
    assert r.acks == [(0, 0)] and d.effects == [ct.OP["arm"]]


def test_suite_repeat_usb_and_ble():
    L, d = link("usb")
    steps = ct.suite_repeat(L)
    assert all(s[3] for s in steps), steps
    assert d.effects.count(ct.OP["param-set"]) == 1
    L, d = link("ble")
    steps = ct.suite_repeat(L)
    assert all(s[3] for s in steps), steps
    assert d.effects.count(ct.OP["arm"]) == 1


def test_suite_repeat_catches_a_drone_without_cache():
    L, d = link("ble")
    d.last = None
    d.cmd = lambda c: d.run(c[0], c[1], c[2:])                            # executes every copy
    steps = ct.suite_repeat(L)
    assert not all(s[3] for s in steps)


def test_suite_bad_ble_and_usb():
    for kind in ("ble", "usb"):
        L, _ = link(kind)
        steps = ct.suite_bad(L)
        assert steps and all(s[3] for s in steps), [s for s in steps if not s[3]]
        names = [s[0] for s in steps]
        assert not any(n.startswith(("param-defaults", "cal-save", "log-erase", "disarm")) for n in names)


def test_suite_usb_rules():
    L, _ = link("usb")
    steps = ct.suite_usb(L)
    assert all(s[3] for s in steps), [s for s in steps if not s[3]]
    L, _ = link("ble")
    assert not ct.suite_usb(L)[0][3]


def test_suite_rates_with_reconnection():
    L, d = link("ble")
    fresh = []

    def reopen():
        L2, d2 = link("ble")
        fresh.append(d2)
        return L2
    steps = ct.suite_rates(L, reopen, seconds=1.5)
    assert all(s[3] for s in steps), steps
    assert d.flight_hz == 2 and fresh and fresh[0].flight_hz == 10


def test_suite_bad_accepts_a_refusal_before_the_payload_check():
    L, d = link("ble")
    run = d.run
    d.run = lambda op, seq, p: d.ack(op, seq, 4, 21) if op == ct.OP["motor-test"] else run(op, seq, p)
    steps = ct.suite_bad(L)
    assert all(s[3] for s in steps), [s for s in steps if not s[3]]


def test_cli_needs_a_link(capsys):
    from acwing_tools.__main__ import main
    assert main(["cmd", "send", "arm"]) == 2
    assert "--port" in capsys.readouterr().err


def test_describe():
    f = parse_telemetry(encode_frame(FT_ACK, bytes([0x12, 5, 1, 17])))[0][0]
    assert ct.describe(f) == "ACK    arm seq 5: REJECTED USB_CONNECTED"
    f = parse_telemetry(encode_frame(ct.FT["param"], struct.pack("<HfBHH", 70, 2.0, 0, 0, 1)))[0][0]
    assert ct.describe(f) == "PARAM  70 logMode = 2"
    f = parse_telemetry(encode_frame(ct.FT["param"], struct.pack("<HfBHH", 1, 1.0, 0, 0, 40)))[0][0]
    assert ct.describe(f) == "PARAM  1 takeoffAlt = 1"                          # a GET: no "(1/40)"
    assert ct.describe(f, listing=True) == "PARAM  1 takeoffAlt = 1  (1/40)"


def test_cli_test_suite_name(capsys):
    from acwing_tools.__main__ import build_parser
    a = build_parser().parse_args(["cmd", "test_suite", "usb", "--port", "COM7"])
    assert a.action == "test_suite" and a.what == ["usb"] and a.port == "COM7"


def test_log_get_bad_id_is_a_clean_error(capsys):
    from acwing_tools.__main__ import main
    assert main(["log", "get", "first", "--port", "COM_NONE"]) == 2       # before opening the port, no traceback
    assert "'last'" in capsys.readouterr().err
