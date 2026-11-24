"""Protocol v2 commands from the PC, over USB or BLE: ``acwing-tools cmd``.

A bench tool for the checks that the app cannot do (#37, #74):

* ``send``   one command, by name or raw bytes, with an optional fixed ``seq``
  and repetitions; prints the ACK (result, reason) and the answers.
* ``listen`` prints ACK, EVENT, PARAM, CALIB and CHECKS frames for a while
  (for example while the app works on the other link).
* ``test_suite`` a ready-made sequence with an expected result per step:

  ==============  ==========================================================
  ``repeat``      the same command twice with the same ``seq``: one effect,
                  two identical ACKs (#74/2; ACK cache, #37 part 4)
  ``bad``         every opcode with an empty payload → INVALID, an unknown
                  op → UNSUPPORTED (#74/3). Over USB the flight commands
                  answer REJECTED / NOT_OVER_USB first.
  ``rates``       STREAM_RATES FLIGHT 2 Hz, measured; reconnect, back to
                  10 Hz (#74/14)
  ``usb``         the USB ground-link rules (#37 part 3): parameters,
                  calibrations, requests allowed; ARM, KILL, MOTOR_TEST
                  refused with NOT_OVER_USB
  ==============  ==========================================================

USB: the FC USB port (CDC); commands travel in CMD frames (type 0x20).
BLE: the CMD characteristic (needs ``bleak``); the app must be disconnected,
the board takes one central at a time. Commands, ACK and frame formats:
``03 - Docs/firmware/protocol-v2.md``.
"""
from __future__ import annotations

import os
import re
import struct
import time
from typing import Callable, Dict, List, Optional, Tuple

from .common import Struct
from .flightlog import FT_ACK, FT_CMD, FcLink
from .telemetry import encode_frame, parse_telemetry

# ------------------------------------------------------------------ protocol (proto_v2.h)
OP: Dict[str, int] = {
    "heartbeat": 0x10, "rc": 0x11, "arm": 0x12, "disarm": 0x13, "takeoff": 0x14, "land": 0x15,
    "rth": 0x16, "hold": 0x17, "kill": 0x18, "mode": 0x19, "launch": 0x1A, "flip": 0x1B,
    "param-get": 0x20, "param-set": 0x21, "param-save": 0x22, "param-defaults": 0x23, "param-list": 0x24,
    "cal-start": 0x30, "cal-capture": 0x31, "cal-abort": 0x32, "cal-save": 0x33, "cal-restore": 0x34,
    "cal-set-ref": 0x35, "motor-test": 0x36, "motor-reverse": 0x37, "motor-remap": 0x38,
    "stream-rates": 0x40, "request": 0x41, "set-name": 0x42, "set-time": 0x43,
    "log-list": 0x50, "log-read": 0x51, "log-erase": 0x52,
}
OP_NAMES = {v: k for k, v in OP.items()}
ARM_MAGIC, KILL_MAGIC, PROPS_OFF, WIPE = b"\xA5\x5A", b"KILL", 0x50, b"WIPE"

RESULTS = {0: "OK", 1: "REJECTED", 2: "BUSY", 3: "INVALID", 4: "UNSUPPORTED", 5: "IN_PROGRESS"}
REASONS = {
    0: "", 1: "NOT_LANDED", 2: "CHECKS_FAILING", 3: "THROTTLE_NOT_LOW", 4: "NO_GPS", 5: "BATTERY_LOW",
    6: "NO_HOME", 7: "RADIO_OVERRIDE", 8: "ARMED", 9: "PROPS_CONFIRM_MISSING", 10: "NOT_ARMED",
    11: "CALIBRATING", 12: "NOT_CALIBRATING", 13: "NOT_STILL", 14: "OUT_OF_RANGE", 15: "UNKNOWN_PARAM",
    16: "SENSOR_MISSING", 17: "USB_CONNECTED", 18: "NOT_OVER_USB", 19: "OTHER_LINK", 20: "THRUST_LOW",
    21: "MOTORS_DISABLED", 22: "FIXED_DIRECTION",
}
FT = {"state": 0x01, "init": 0x02, "status": 0x03, "caps": 0x05, "flight": 0x06, "power": 0x07, "nav": 0x08,
      "motors": 0x09, "system": 0x0A, "event": 0x0B, "param": 0x0C, "calib": 0x0D, "ack": 0x0E, "checks": 0x0F}
FT_NAMES = {v: k.upper() for k, v in FT.items()}
CAL = {"accel": 1, "level": 2, "gyro": 3, "compass": 4, "flow": 5, "battery": 6, "motors": 7, "sticks": 8}
#: ops that answer with no ACK
NO_ACK = (OP["heartbeat"], OP["rc"])
#: ops the firmware never takes over USB (app_task.c usb_allowed, ADR 0009); log ops go to the LogTask
USB_ALLOWED = {OP[k] for k in ("param-get", "param-set", "param-save", "param-defaults", "param-list",
                               "cal-start", "cal-capture", "cal-abort", "cal-save", "cal-restore",
                               "cal-set-ref", "stream-rates", "request", "set-name", "set-time",
                               "log-list", "log-read", "log-erase")}
#: ops whose payload may be empty (an empty payload is valid, not INVALID)
EMPTY_OK = {OP[k] for k in ("disarm", "takeoff", "land", "rth", "hold", "param-save", "param-defaults", "param-list",
                            "cal-capture", "cal-abort", "cal-save", "log-list")}
#: refused because of the drone's state before the payload is looked at (fc_core.c / app_task.c): also fine in
#: the "bad" suite. MOTOR_TEST on a build without motor outputs, MOTOR_REVERSE/REMAP on brushed motors.
REFUSED_FIRST = {OP["motor-test"]: {(4, 21)}, OP["motor-reverse"]: {(4, 0), (4, 22), (1, 22)},
                 OP["motor-remap"]: {(4, 0)}}


def param_names() -> Dict[str, int]:
    """Parameter names → ids, read from the app's table (``02 - App/src/protocol/ids.ts``) when the
    repository is there; numbers always work."""
    here = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(here, "..", "..", "02 - App", "src", "protocol", "ids.ts")
    try:
        with open(path, encoding="utf-8") as fh:
            return {k: int(i) for i, k in re.findall(r'\{\s*id:\s*(\d+),\s*key:\s*"(\w+)"', fh.read())}
    except OSError:
        return {}


def _param_id(x: str) -> int:
    if x.isdigit():
        return int(x)
    names = param_names()
    if x not in names:
        raise ValueError(f'unknown parameter "{x}" (give its number; names come from the app\'s ids.ts)')
    return names[x]


def encode_args(name: str, args: List[str]) -> Tuple[int, bytes]:
    """A named command and its text arguments → (op, payload). ``raw OP HEX...`` sends any bytes."""
    a = list(args)
    if name == "raw":
        if not a:
            raise ValueError("raw OP [HEX...]: e.g. raw 0x21 4600 00002041")
        return int(a[0], 0), bytes.fromhex("".join(a[1:]))
    if name not in OP:
        raise ValueError(f'unknown command "{name}" (one of: {", ".join(sorted(OP))}, raw)')
    op = OP[name]
    need = {"takeoff": 1, "mode": 1, "flip": 1, "param-get": 1, "param-set": 2, "cal-start": 1,
            "cal-restore": 1, "motor-test": 3, "motor-reverse": 1, "stream-rates": 5, "request": 1,
            "set-name": 1, "rc": 4, "log-read": 1}.get(name, 0)
    if len(a) < need:
        raise ValueError(f"{name}: {need} argument(s) needed")
    if name in ("arm", "launch"):
        return op, ARM_MAGIC
    if name == "kill":
        return op, KILL_MAGIC
    if name == "log-erase":
        return op, WIPE
    if name == "takeoff":
        return op, struct.pack("<H", int(round(float(a[0]) * 100)))
    if name in ("mode", "flip", "motor-reverse"):
        return op, bytes([int(a[0], 0)])
    if name == "param-get":
        return op, struct.pack("<H", _param_id(a[0]))
    if name == "param-set":
        return op, struct.pack("<Hf", _param_id(a[0]), float(a[1]))
    if name in ("cal-start", "cal-restore"):
        t = CAL[a[0]] if a[0] in CAL else int(a[0], 0)
        return op, bytes([t, int(a[1], 0) if len(a) > 1 and name == "cal-start" else 0])[: 2 if name == "cal-start" else 1]
    if name == "motor-test":
        return op, struct.pack("<BBHB", int(a[0]), int(a[1]), int(a[2]), PROPS_OFF)
    if name == "stream-rates":
        return op, bytes(int(x) for x in a[:5])
    if name == "request":
        return op, bytes([FT[a[0]] if a[0] in FT else int(a[0], 0)])
    if name == "set-name":
        return op, a[0].encode()[:16]
    if name == "set-time":
        t = time.time()
        tz = -(time.altzone if time.localtime(t).tm_isdst > 0 else time.timezone) // 60
        return op, struct.pack("<IHh", int(t), int((t % 1) * 1000), tz)
    if name == "rc":
        return op, struct.pack("<hhhhB", *(int(x) for x in a[:4]), 0)
    if name == "log-read":
        return op, struct.pack("<HII", int(a[0]), int(a[1]) if len(a) > 1 else 0, int(a[2]) if len(a) > 2 else 0)
    return op, b""


def describe(f) -> Optional[str]:
    """One line for the frames worth printing (ACK, EVENT, PARAM, CALIB, CHECKS); None for the rest."""
    p = f.payload
    if f.type == FT["ack"] and len(p) >= 4:
        return f"ACK    {OP_NAMES.get(p[0], hex(p[0]))} seq {p[1]}: {RESULTS.get(p[2], p[2])} {REASONS.get(p[3], p[3])}".rstrip()
    if f.type == FT["event"] and len(p) >= 7:
        lvl, code, arg = struct.unpack_from("<BHi", p)
        return f"EVENT  code {code} arg {arg} level {lvl}"
    if f.type == FT["param"] and len(p) >= 11:
        i, v, fl, idx, cnt = struct.unpack_from("<HfBHH", p)
        name = {v_: k for k, v_ in param_names().items()}.get(i, "")
        return f"PARAM  {i} {name} = {v:g}" + (f"  ({idx + 1}/{cnt})" if cnt > 1 else "")
    if f.type == FT["calib"] and len(p) >= 2:
        return f"CALIB  type {p[0]} state {p[1]}"
    if f.type == FT["checks"] and len(p) >= 2:
        fails = sum(1 for k in range(p[1]) if 2 + 4 * k + 1 < len(p) and p[2 + 4 * k + 1] == 2)
        return f"CHECKS ready {p[0]}, {p[1]} items, {fails} FAIL"
    return None


# ------------------------------------------------------------------ link
class CmdLink:
    """Sends protocol v2 commands and collects what comes back. ``port`` has ``write(bytes)`` and
    ``read(n) -> bytes`` (pyserial, the BLE adapter of :class:`FcLink`, or a test double)."""

    def __init__(self, port, kind: str, raw: bool, wait: float = 1.0):
        self.port, self.kind, self.raw, self.wait = port, kind, raw, wait
        self.seq = 0x40
        self.rest = b""
        self.log: List = []          # every frame received, in order

    @classmethod
    def open(cls, port: Optional[str] = None, ble: Optional[str] = None, wait: float = 1.0) -> "CmdLink":
        if ble is not None:
            fl = FcLink.open_ble(ble)
            return cls(fl.port, "ble", True, wait)
        fl = FcLink.open(port)
        try:                          # hold DTR: the drone sees the port open (usb_link.h)
            fl.port.dtr = True
        except Exception:             # pragma: no cover
            pass
        return cls(fl.port, "usb", False, wait)

    def close(self):
        try:
            self.port.close()
        except Exception:             # pragma: no cover
            pass

    def frames(self, seconds: float) -> List:
        out, t_end = [], time.monotonic() + seconds
        while time.monotonic() < t_end:
            data = self.port.read(4096)
            if not data:
                time.sleep(0.005)
                continue
            fr, self.rest = parse_telemetry(self.rest + data)
            self.log += fr
            out += fr
        return out

    def write(self, op: int, payload: bytes = b"", seq: Optional[int] = None) -> int:
        if seq is None:
            self.seq = (self.seq + 1) & 0xFF
            seq = self.seq
        cmd = bytes([op, seq & 0xFF]) + bytes(payload)
        self.port.write(cmd if self.raw else encode_frame(FT_CMD, cmd))
        return seq & 0xFF

    def send(self, op: int, payload: bytes = b"", seq: Optional[int] = None, repeat: int = 1,
             gap: float = 0.15) -> Struct:
        """Sends ``repeat`` times with the same seq; waits for the ACKs. Result: ``acks`` (list of
        (result, reason)), ``frames`` (everything received meanwhile), ``seq``."""
        acks, frames = [], []
        s = None
        for k in range(max(1, repeat)):
            s = self.write(op, payload, seq if seq is not None else (s if k else None))
            got = self.frames(gap if k < repeat - 1 else 0.0)
            frames += got
        t_end = time.monotonic() + (0 if op in NO_ACK else self.wait)
        while True:
            acks = [(f.payload[2], f.payload[3]) for f in frames
                    if f.type == FT_ACK and len(f.payload) >= 4 and f.payload[0] == op and f.payload[1] == s
                    and f.payload[2] != 5]
            if len(acks) >= max(1, repeat) or time.monotonic() >= t_end:
                break
            frames += self.frames(0.05)
        return Struct(op=op, seq=s, acks=acks, frames=frames)


def fmt_ack(a: Optional[Tuple[int, int]]) -> str:
    if a is None:
        return "no answer"
    return f"{RESULTS.get(a[0], a[0])} {REASONS.get(a[1], a[1])}".strip()


def rate_hz(frames, ftype: int, seconds: float) -> float:
    return sum(1 for f in frames if f.type == ftype) / max(seconds, 1e-6)


# ------------------------------------------------------------------ suites
Step = Tuple[str, str, str, bool]           # (what, expected, got, ok)


def suite_repeat(link: CmdLink) -> List[Step]:
    """#74/2 and the ACK cache: the same command twice with one seq → one effect, two identical ACKs.
    BLE: ARM (then DISARM). USB (ARM not allowed): PARAM_SET of the takeoff altitude, its own value."""
    out: List[Step] = []
    if link.kind == "ble":
        r = link.send(OP["arm"], ARM_MAGIC, repeat=2)
        r.frames += link.frames(0.5)
        armed_events = sum(1 for f in r.frames if f.type == FT["event"] and len(f.payload) >= 3
                           and struct.unpack_from("<H", f.payload, 1)[0] == 1)
        same = len(r.acks) == 2 and r.acks[0] == r.acks[1]
        out.append(("ARM twice, same seq", "two identical ACKs", ", ".join(fmt_ack(a) for a in r.acks) or "none", same))
        out.append(("ARMED events", "1 (if the ACK is OK)", str(armed_events),
                    armed_events == (1 if r.acks and r.acks[0][0] == 0 else 0)))
        link.send(OP["disarm"])
    else:
        pid = param_names().get("takeoffAlt", 1)
        g = link.send(OP["param-get"], struct.pack("<H", pid))
        val = None
        for f in g.frames:
            if f.type == FT["param"] and len(f.payload) >= 6:
                i, v = struct.unpack_from("<Hf", f.payload)
                if i == pid:
                    val = v
        if val is None:
            return [("PARAM_GET takeoffAlt", "PARAM", "no answer", False)]
        r = link.send(OP["param-set"], struct.pack("<Hf", pid, val), repeat=2)
        r.frames += link.frames(0.5)
        echoes = sum(1 for f in r.frames if f.type == FT["param"] and len(f.payload) >= 2 and struct.unpack_from("<H", f.payload)[0] == pid)
        same = len(r.acks) == 2 and r.acks[0] == r.acks[1]
        out.append(("PARAM_SET twice, same seq", "two identical ACKs", ", ".join(fmt_ack(a) for a in r.acks) or "none", same))
        out.append(("PARAM echoes", "1 (executed once)", str(echoes), echoes == 1))
    return out


def suite_bad(link: CmdLink) -> List[Step]:
    """#74/3: every opcode that needs a payload, sent empty → INVALID; an unknown op → UNSUPPORTED."""
    out: List[Step] = []
    for name, op in sorted(OP.items(), key=lambda kv: kv[1]):
        if op in NO_ACK or op in EMPTY_OK or name in ("log-erase",):
            continue
        r = link.send(op, b"")
        got = r.acks[0] if r.acks else None
        if link.kind == "usb" and op not in USB_ALLOWED:
            exp, ok = "REJECTED NOT_OVER_USB", got == (1, 18)
        else:
            early = got in REFUSED_FIRST.get(op, set())
            exp, ok = "INVALID", got is not None and (got[0] == 3 or early)
            if early:
                exp = "INVALID (or refused first)"
        out.append((f"{name} (0x{op:02X}) empty", exp, fmt_ack(got), ok))
    r = link.send(0x7E, b"")
    got = r.acks[0] if r.acks else None
    exp_ok = got is not None and got[0] in (4, 1)       # USB answers REJECTED for ops >= 0x10 it does not take
    out.append(("unknown op 0x7E", "UNSUPPORTED (USB: REJECTED NOT_OVER_USB)", fmt_ack(got), exp_ok))
    return out


def suite_rates(link: CmdLink, reopen: Optional[Callable[[], CmdLink]] = None, seconds: float = 4.0) -> List[Step]:
    """#74/14: FLIGHT at 2 Hz with STREAM_RATES; after a reconnection, back to the default 10 Hz."""
    out: List[Step] = []
    link.frames(0.5)
    before = rate_hz(link.frames(seconds), FT["flight"], seconds)
    r = link.send(OP["stream-rates"], bytes([2, 0, 0, 0, 0]))
    out.append(("STREAM_RATES flight 2 Hz", "OK", fmt_ack(r.acks[0] if r.acks else None), bool(r.acks) and r.acks[0][0] == 0))
    link.frames(0.5)
    after = rate_hz(link.frames(seconds), FT["flight"], seconds)
    out.append(("FLIGHT rate", f"~2 Hz (was {before:.1f})", f"{after:.1f} Hz", 1.0 <= after <= 3.0))
    if reopen is not None:
        link.close()
        time.sleep(1.0)
        link2 = reopen()
        link2.frames(1.0)
        again = rate_hz(link2.frames(seconds), FT["flight"], seconds)
        out.append(("FLIGHT rate after reconnecting", "~10 Hz (default)", f"{again:.1f} Hz", 7.0 <= again <= 13.0))
        link2.close()
    return out


def suite_usb(link: CmdLink) -> List[Step]:
    """#37 part 3: over USB the ground-link commands work, the flight commands are refused."""
    if link.kind != "usb":
        return [("usb suite", "USB link (--port)", "BLE", False)]
    out: List[Step] = []

    def step(what, op, payload, exp_res, exp_why=0):
        r = link.send(op, payload)
        got = r.acks[0] if r.acks else None
        out.append((what, f"{RESULTS[exp_res]} {REASONS[exp_why]}".strip(), fmt_ack(got),
                    got is not None and got[0] == exp_res and (exp_res == 0 or got[1] == exp_why)))
        return r
    r = step("PARAM_GET logMode", OP["param-get"], struct.pack("<H", param_names().get("logMode", 70)), 0)
    out.append(("  PARAM answer", "PARAM frame", "yes" if any(f.type == FT["param"] for f in r.frames) else "no",
                any(f.type == FT["param"] for f in r.frames)))
    step("REQUEST CAPS", OP["request"], bytes([FT["caps"]]), 0)
    step("CAL_START gyro", OP["cal-start"], bytes([CAL["gyro"], 0]), 0)
    step("CAL_ABORT", OP["cal-abort"], b"", 0)
    step("ARM", OP["arm"], ARM_MAGIC, 1, 18)
    step("KILL", OP["kill"], KILL_MAGIC, 1, 18)
    step("MOTOR_TEST", OP["motor-test"], struct.pack("<BBHB", 0, 10, 500, PROPS_OFF), 1, 18)
    step("TAKEOFF", OP["takeoff"], struct.pack("<H", 100), 1, 18)
    return out


SUITES = {"repeat": suite_repeat, "bad": suite_bad, "rates": suite_rates, "usb": suite_usb}


def print_steps(steps: List[Step]) -> int:
    w = max((len(s[0]) for s in steps), default=10)
    fails = 0
    for what, exp, got, ok in steps:
        fails += not ok
        print(f"  {'PASS' if ok else 'FAIL'}  {what:<{w}}  expected: {exp:<28} got: {got}")
    print(f"{len(steps) - fails}/{len(steps)} as expected")
    return fails
