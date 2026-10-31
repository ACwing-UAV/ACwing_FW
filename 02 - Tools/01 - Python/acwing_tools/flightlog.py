"""Flight logs: download over USB and decode.

Python equivalent of ``00 - Matlab/FlightLog`` (acwingLogDecode.m). Format and
protocol are defined by the firmware:

* ``App/log/log_format.h`` – on-flash format 1: a log is a run of 4 KB
  sectors; each sector starts with a 20-byte header, then records
  ``[type][len][payload][crc8]`` until the first 0xFF byte.
* ``App/app/log_task.h`` – USB CDC link. The PC sends protocol v2 frames of
  type 0x20 (``CMD``: ``op, seq, args``); the drone answers with
  LOG_ENTRY (0x10), LOG_DATA (0x11) and ACK (0x0E) frames.
* Over **BLE** the same commands are written raw to the CMD characteristic
  and the answers come with the telemetry notifications. BLE gives access to
  the **last flight only** (LOG_LIST lists one log, LOG_READ accepts only that
  one, LOG_ERASE is USB only); the whole memory goes over USB.

  ============ ===== =================================== ====================
  command      op    arguments                           answer
  ============ ===== =================================== ====================
  LOG_LIST     0x50  –                                   LOG_ENTRY × n, ACK
  LOG_READ     0x51  id u16, offset u32, len u32 (0=all) LOG_DATA …, then ACK
  LOG_ERASE    0x52  ``b"WIPE"``                         ACK IN_PROGRESS, ACK
  ============ ===== =================================== ====================
"""
from __future__ import annotations

import os
import struct
import time
from typing import Dict, List, Optional

import numpy as np

from .common import Struct
from .telemetry import crc16ccitt, encode_frame, parse_telemetry

# ---------------------------------------------------------------- format 1
LOG_MAGIC = 0x474C4341
LOG_FORMAT = 1
SECTOR = 4096
HDR = struct.Struct("<IIHHIBBH")          # magic serial log_id seq t0_ms format rsv crc

# record type -> (name, struct format, field names); scaled to SI below
RECORDS = {
    0x01: ("info",   "<3BcBIH16s", ("fw0", "fw1", "fw2", "hw_rev", "format", "t0_ms", "sensors", "name")),
    0x10: ("att",    "<I4h3h3h",   ("t_ms", "qw", "qx", "qy", "qz", "gx", "gy", "gz", "ax", "ay", "az")),
    0x11: ("vert",   "<IihihBHBIB", ("t_ms", "h_mm", "vz_mms", "h_sp_mm", "vz_sp_mms", "thrust", "tof_mm",
                                     "flow_q", "baro_pa", "flags")),
    0x12: ("flight", "<I4BB4bH",   ("t_ms", "armed", "mode", "autom", "failsafe", "flags",
                                     "thr", "yaw", "pitch", "roll", "silence_ms")),
    0x13: ("event",  "<IBHi",      ("t_ms", "level", "code", "arg")),
    0x14: ("est",    "<I3hHHHHBB", ("t_ms", "bx", "by", "bz", "att_unc_cdeg", "vib_mg", "loop_hz",
                                     "jitter_us", "cpu_pct", "mag_interf_pct")),
    0x20: ("power",  "<IHhHHHBBHB", ("t_ms", "vbat_mv", "current_ca", "used_mah", "cell_mv", "cell_rest_mv",
                                     "pct", "level", "r_cell_mohm", "cells")),
    0x7F: ("drop",   "<IHB",       ("t_ms", "lost", "source")),
}
for _t, (_n, _f, _k) in RECORDS.items():
    assert struct.calcsize(_f) == {0x01: 27, 0x10: 24, 0x11: 25, 0x12: 15, 0x13: 11,
                                   0x14: 20, 0x20: 19, 0x7F: 7}[_t], _n

EVENT_NAMES = {
    1: "ARMED", 2: "DISARMED", 3: "TAKEOFF_STARTED", 4: "LANDED", 5: "MODE_CHANGED", 6: "RTH_STARTED",
    7: "HOME_SET", 8: "TAKEOFF_ABORTED", 20: "FAILSAFE_APP_LINK", 21: "FAILSAFE_RADIO",
    22: "FAILSAFE_BATTERY", 23: "GEOFENCE_BREACH", 24: "GPS_LOST", 25: "FLOW_LOST", 30: "BATTERY_LOW",
    31: "BATTERY_CRITICAL", 32: "BATTERY_EMERGENCY", 33: "BATTERY_MONITOR_LOST", 40: "MAG_INTERFERENCE",
    41: "HIGH_VIBRATION", 42: "MOTOR_HOT", 43: "RADIO_OVERRIDE", 50: "CALIB_DONE", 51: "CALIB_FAILED",
    60: "PARAMS_SAVED", 70: "MOTOR_KILLED",
}


def crc8(data) -> int:
    """CRC-8/SMBUS (poly 0x07, init 0), as ``log_crc8()``."""
    c = 0
    for b in bytes(data):
        c ^= b
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c


def decode_log(data: bytes) -> Struct:
    """Decodes one downloaded log (sectors concatenated).

    Returns a struct with one field per record type (``att``, ``vert``,
    ``flight``, ``event``, ``est``, ``power``, ``drop``), each a struct of
    numpy arrays in SI units, plus ``info`` (first LR_INFO), ``sectors``,
    ``bad`` (records or sectors rejected) and ``unknown`` (records of an
    unknown type, skipped).
    """
    data = bytes(data)
    rows: Dict[int, list] = {t: [] for t in RECORDS}
    bad = unknown = sectors = 0
    for s in range(0, len(data) - SECTOR + 1, SECTOR):
        h = HDR.unpack_from(data, s)
        if h[0] != LOG_MAGIC or h[7] != crc16ccitt(data[s:s + 18]) or h[5] != LOG_FORMAT:
            bad += 1
            continue
        sectors += 1
        i, end = s + HDR.size, s + SECTOR
        while i + 3 <= end and data[i] != 0xFF:
            t, n = data[i], data[i + 1]
            if i + 3 + n > end or crc8(data[i:i + 2 + n]) != data[i + 2 + n]:
                bad += 1                       # cut record (power loss): rest of the sector skipped
                break
            if t in RECORDS and struct.calcsize(RECORDS[t][1]) == n:
                rows[t].append(struct.unpack_from(RECORDS[t][1], data, i + 2))
            else:
                unknown += 1
            i += 3 + n

    out = Struct(sectors=sectors, bad=bad, unknown=unknown, info=None)
    for t, (name, _fmt, keys) in RECORDS.items():
        r = rows[t]
        if name == "info":
            if r:
                v = r[0]
                out.info = Struct(fw=f"{v[0]}.{v[1]}.{v[2]}", hw_rev=v[3].decode(errors="replace"),
                                  format=v[4], t0_ms=v[5], sensors=v[6],
                                  name=v[7].split(b"\0")[0].decode(errors="replace"))
            continue
        cols = {k: np.array([x[j] for x in r], dtype=float) for j, k in enumerate(keys)}
        out[name] = _scale(name, cols, len(r))
    return out


def _scale(name: str, c: Dict[str, np.ndarray], n: int) -> Struct:
    S = Struct(n=n, t=c["t_ms"] * 1e-3)
    if name == "att":
        S.q = np.vstack([c["qw"], c["qx"], c["qy"], c["qz"]]) / 32767.0
        S.gyro = np.vstack([c["gx"], c["gy"], c["gz"]]) * 1e-3          # rad/s
        S.acc = np.vstack([c["ax"], c["ay"], c["az"]]) * 1e-3           # g
    elif name == "vert":
        S.h = c["h_mm"] * 1e-3; S.vz = c["vz_mms"] * 1e-3
        S.h_sp = c["h_sp_mm"] * 1e-3; S.vz_sp = c["vz_sp_mms"] * 1e-3
        S.thrust = c["thrust"] / 255.0
        S.tof = np.where(c["tof_mm"] == 0xFFFF, np.nan, c["tof_mm"] * 1e-3)
        S.flow_q = np.where(c["flow_q"] == 0xFF, np.nan, c["flow_q"])
        S.baro_pa = np.where(c["baro_pa"] == 0, np.nan, c["baro_pa"])
        S.flags = c["flags"]
    elif name == "flight":
        for k in ("armed", "mode", "autom", "failsafe", "flags", "silence_ms"):
            S[k] = c[k]
        S.sticks = np.vstack([c["thr"], c["yaw"], c["pitch"], c["roll"]]) / 100.0
    elif name == "event":
        S.level = c["level"]; S.code = c["code"]; S.arg = c["arg"]
        S.name = [EVENT_NAMES.get(int(x), f"EVENT_{int(x)}") for x in c["code"]]
    elif name == "est":
        S.gyro_bias = np.vstack([c["bx"], c["by"], c["bz"]]) * 1e-4      # rad/s
        S.att_unc_deg = c["att_unc_cdeg"] / 100.0
        for k in ("vib_mg", "loop_hz", "jitter_us", "cpu_pct", "mag_interf_pct"):
            S[k] = c[k]
    elif name == "power":
        S.vbat = c["vbat_mv"] * 1e-3; S.current = c["current_ca"] * 1e-2
        S.used_mah = c["used_mah"]; S.cell = c["cell_mv"] * 1e-3; S.cell_rest = c["cell_rest_mv"] * 1e-3
        S.pct = c["pct"]; S.level = c["level"]; S.r_cell = c["r_cell_mohm"] * 1e-3
        cells = c["cells"].astype(int) if n else np.zeros(0, int)
        S.cells = (cells & 0x7F).astype(float); S.present = ((cells & 0x80) != 0).astype(float)
    elif name == "drop":
        S.lost = c["lost"]; S.source = c["source"]
    return S


def save_csv(L: Struct, out_dir: str, stem: str) -> List[str]:
    """One CSV per record type (time in s, SI units). Returns the files written."""
    os.makedirs(out_dir, exist_ok=True)
    files = []
    for name in ("att", "vert", "flight", "event", "est", "power", "drop"):
        S = L[name]
        if not S.n:
            continue
        cols, heads = [S.t], ["t_s"]
        for k, v in S.items():
            if k in ("n", "t", "name"):
                continue
            v = np.atleast_2d(np.asarray(v, dtype=float))
            if v.shape[0] == 1:
                cols.append(v[0]); heads.append(k)
            else:
                for j in range(v.shape[0]):
                    cols.append(v[j]); heads.append(f"{k}{j}")
        path = os.path.join(out_dir, f"{stem}_{name}.csv")
        with open(path, "w") as fh:
            hdr = heads + (["name"] if name == "event" else [])
            fh.write(",".join(hdr) + "\n")
            for i in range(S.n):
                row = [f"{c[i]:.6g}" for c in cols]
                if name == "event":
                    row.append(S.name[i])
                fh.write(",".join(row) + "\n")
        files.append(path)
    return files


def save_mat(L: Struct, path: str) -> None:
    """Saves the decoded log as ``.mat`` (variable ``F``), readable by acwingLogDecode.m users."""
    import scipy.io as sio

    def conv(x):
        if isinstance(x, Struct):
            return {k: conv(v) for k, v in x.items() if v is not None}
        return x
    sio.savemat(path, {"F": conv(L)}, do_compression=True)


# ---------------------------------------------------------------- USB link
OP_LOG_LIST, OP_LOG_READ, OP_LOG_ERASE = 0x50, 0x51, 0x52
FT_ACK, FT_LOG_ENTRY, FT_LOG_DATA, FT_CMD = 0x0E, 0x10, 0x11, 0x20
ACK_OK, ACK_IN_PROGRESS = 0, 5
ACK_RESULTS = {0: "OK", 1: "REJECTED", 2: "BUSY", 3: "INVALID", 4: "UNSUPPORTED", 5: "IN_PROGRESS"}
REASONS = {8: "ARMED", 14: "OUT_OF_RANGE (unknown log, or not the last flight over BLE)",
           16: "SENSOR_MISSING (no log flash)"}


class LinkError(RuntimeError):
    pass


class FcLink:
    """Command/answer over the FC USB port (any object with ``read(n)`` and ``write(b)``).

    ``raw=True``: commands are written as they are (``op, seq, args``), as on the
    BLE CMD characteristic, instead of being wrapped in a CMD frame.
    """

    def __init__(self, port, timeout: float = 2.0, raw: bool = False):
        self.port = port
        self.timeout = timeout
        self.raw = raw
        self.rest = b""
        self.seq = 0

    @classmethod
    def open(cls, name: str, timeout: float = 2.0) -> "FcLink":
        from .serial_io import open_serial
        return cls(open_serial(name, 115200, timeout=0.05), timeout)

    @classmethod
    def open_ble(cls, target: str = "", timeout: float = 5.0) -> "FcLink":
        """BLE link (needs bleak): the last flight only."""
        from .ble_io import BleSource

        class _Port:
            def __init__(self, src):
                self.src = src

            def write(self, b):
                self.src.send_command(b)

            def read(self, n):
                return self.src.read()

            def close(self):
                self.src.close()
        # STATE at 1 Hz (decimation 240): the log frames get the radio time
        return cls(_Port(BleSource(target, decim=240)), timeout, raw=True)

    def close(self):
        try:
            self.port.close()
        except Exception:                                       # pragma: no cover
            pass

    def _send(self, op: int, args: bytes = b"") -> int:
        self.seq = (self.seq + 1) & 0xFF
        cmd = bytes([op, self.seq]) + bytes(args)
        self.port.write(cmd if self.raw else encode_frame(FT_CMD, cmd))
        return self.seq

    def _frames(self):
        data = self.port.read(4096)
        if not data:
            return []
        frames, self.rest = parse_telemetry(self.rest + data)
        return frames

    def _run(self, op: int, args: bytes = b"", on_frame=None, timeout: Optional[float] = None):
        """Sends a command, feeds every other frame to on_frame until the final ACK."""
        seq = self._send(op, args)
        t_end = time.monotonic() + (timeout or self.timeout)
        while time.monotonic() < t_end:
            for f in self._frames():
                if f.type == FT_ACK and len(f.payload) >= 4 and f.payload[0] == op and f.payload[1] == seq:
                    res, why = f.payload[2], f.payload[3]
                    if res == ACK_IN_PROGRESS:
                        t_end = time.monotonic() + max(timeout or self.timeout, 300.0)
                        continue
                    if res != ACK_OK:
                        raise LinkError(f"0x{op:02X}: {ACK_RESULTS.get(res, res)} "
                                        f"{REASONS.get(why, why) if why else ''}".strip())
                    return
                if on_frame:
                    on_frame(f)
                t_end = max(t_end, time.monotonic() + (timeout or self.timeout))
        raise LinkError(f"0x{op:02X}: no answer (is this the FC USB port?)")

    def list_logs(self) -> List[Struct]:
        out = []

        def take(f):
            if f.type == FT_LOG_ENTRY and len(f.payload) >= 15:
                i, idx, cnt, fl, sect, t0, nb = struct.unpack_from("<HBBBHII", f.payload)
                out.append(Struct(id=i, index=idx, count=cnt, truncated=bool(fl & 1), active=bool(fl & 2),
                                  sectors=sect, t0_ms=t0, bytes=nb))
        self._run(OP_LOG_LIST, b"", take)
        return out

    def read_log(self, log_id: int, size: int, chunk: int = 256 * 1024, progress=None) -> bytes:
        buf = bytearray(b"\xFF" * size)
        off = 0
        while off < size:
            n = min(chunk, size - off)
            got = [0]

            def take(f, base=off):
                if f.type == FT_LOG_DATA and len(f.payload) >= 6:
                    i, o = struct.unpack_from("<HI", f.payload)
                    d = f.payload[6:]
                    if i == log_id and base <= o and o + len(d) <= size:
                        buf[o:o + len(d)] = d
                        got[0] += len(d)
            self._run(OP_LOG_READ, struct.pack("<HII", log_id, off, n), take)
            if got[0] != n:
                raise LinkError(f"log {log_id}: {got[0]} of {n} bytes at offset {off}")
            off += n
            if progress:
                progress(off, size)
        return bytes(buf)

    def erase(self):
        self._run(OP_LOG_ERASE, b"WIPE", None, timeout=300.0)


def download_all(link: FcLink, out_dir: str, ids=None, progress=None) -> List[str]:
    """Downloads the logs (all, or those in ``ids``) as ``acwing_flight_<id>.bin``."""
    os.makedirs(out_dir, exist_ok=True)
    files = []
    for e in link.list_logs():
        if ids and e.id not in ids:
            continue
        data = link.read_log(e.id, e.bytes, progress=(lambda o, n, i=e.id: progress(i, o, n)) if progress else None)
        path = os.path.join(out_dir, f"acwing_flight_{e.id:05d}.bin")
        with open(path, "wb") as fh:
            fh.write(data)
        files.append(path)
    return files


def summary(L: Struct) -> List[str]:
    """Human summary of a decoded log."""
    lines = []
    if L.info:
        lines.append(f"firmware {L.info.fw}, board '{L.info.hw_rev}', format {L.info.format}, "
                     f"started at {L.info.t0_ms / 1000:.1f} s after power-on")
    t = L.att.t
    if L.att.n:
        lines.append(f"{L.att.n} attitude samples over {t[-1] - t[0]:.1f} s ({L.sectors} sectors, "
                     f"{L.bad} bad, {L.unknown} unknown records)")
    if L.power.n:
        p = L.power
        lines.append(f"battery: {p.vbat[0]:.2f} -> {p.vbat[-1]:.2f} V, min cell {np.min(p.cell):.2f} V, "
                     f"used {p.used_mah[-1] - p.used_mah[0]:.0f} mAh, max {np.max(p.current):.1f} A")
    if L.drop.n:
        lines.append(f"WARNING: {int(np.sum(L.drop.lost))} records dropped (queue full)")
    for i in range(L.event.n):
        lines.append(f"  {L.event.t[i]:9.3f} s  {L.event.name[i]}  arg {int(L.event.arg[i])}")
    return lines
