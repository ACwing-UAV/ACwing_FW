"""Telemetry frames: CRC, stream parser, packet decoders, STATUS diagnosis.

Python equivalent of ``00 - Matlab/Telemetry`` (crc16ccitt.m, parseTelemetry.m,
decodeState.m, decodeInit.m, decodeStatus.m, decodeReplay.m,
statusDiagnosis.m). Field names are those of the MATLAB structs.

Frame format (little-endian, see ``App/telemetry/telemetry.h``)::

    0xA5 0x5A | TYPE | LEN | PAYLOAD[LEN] | CRC16 (LSB, MSB)

CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over TYPE..PAYLOAD.
"""
from __future__ import annotations

import binascii
from typing import Iterable, List, Tuple

import numpy as np

from .common import Struct

SYNC0, SYNC1 = 0xA5, 0x5A
TYPE_STATE, TYPE_INIT, TYPE_STATUS, TYPE_REPLAY = 1, 2, 3, 4
TYPE_NAMES = {TYPE_STATE: "STATE", TYPE_INIT: "INIT", TYPE_STATUS: "STATUS", TYPE_REPLAY: "REPLAY"}
OVERHEAD = 6

# STATE flags (TLM_F_* in telemetry.h)
F_RUNNING, F_MAG_NEW, F_ACC_GATED, F_MAG_GATED, F_OVERRUN, F_STILL = 1, 2, 4, 8, 16, 32


def crc16ccitt(data) -> int:
    """CRC-16/CCITT-FALSE, as ``tlm_crc16()`` in C. ``crc16ccitt(b"123456789") == 0x29B1``."""
    return binascii.crc_hqx(bytes(data), 0xFFFF)


def encode_frame(ftype: int, payload: bytes) -> bytes:
    """Builds a complete frame (useful for tests and simulators), as ``tlm_encode()``."""
    payload = bytes(payload)
    if len(payload) > 255:
        raise ValueError("payload longer than 255 bytes")
    body = bytes([ftype, len(payload)]) + payload
    crc = crc16ccitt(body)
    return bytes([SYNC0, SYNC1]) + body + bytes([crc & 0xFF, crc >> 8])


def parse_telemetry(buf, skip_types: Iterable[int] = ()) -> Tuple[List[Struct], bytes]:
    """Extracts the complete, valid frames from a byte buffer.

    Parameters
    ----------
    buf : bytes-like
        Received bytes, possibly the ``rest`` of the previous call followed by
        new data.
    skip_types : iterable of int
        Frame types to skip WITHOUT checking the CRC (e.g. ``(4,)`` for REPLAY
        in the live viewer, which only stores them raw in the ``.bin``). A
        skipped frame is accepted only if a new sync follows immediately.

    Returns
    -------
    frames : list of Struct
        ``frame.type`` (int) and ``frame.payload`` (bytes), in stream order.
    rest : bytes
        Bytes not consumed yet (incomplete frame at the tail): prepend them to
        the next chunk.

    A wrong CRC shifts by one byte and re-synchronises, exactly as
    ``parseTelemetry.m``.
    """
    buf = bytes(buf)
    skip = set(int(t) for t in skip_types)
    n = len(buf)
    frames: List[Struct] = []
    i = 0
    while i + OVERHEAD <= n:
        if buf[i] != SYNC0 or buf[i + 1] != SYNC1:
            j = buf.find(b"\xA5\x5A", i + 1)
            if j < 0:
                i = n - 1                       # keep the last byte: it may be 0xA5
                break
            i = j
            continue
        length = buf[i + 3]
        total = length + OVERHEAD
        if i + total > n:
            break                               # incomplete frame: wait for more bytes
        if skip and buf[i + 2] in skip:
            j = i + total                       # next sync expected here
            if j + 2 > n:
                break
            if buf[j] == SYNC0 and buf[j + 1] == SYNC1:
                i = j
                continue
        body = buf[i + 2:i + 4 + length]
        crc_rx = buf[i + 4 + length] | (buf[i + 5 + length] << 8)
        if crc16ccitt(body) == crc_rx:
            frames.append(Struct(type=buf[i + 2], payload=buf[i + 4:i + 4 + length]))
            i += total
        else:
            i += 1                              # false sync or corrupted data
    return frames, buf[i:]


def _f32(p: bytes, start: int, count: int) -> np.ndarray:
    return np.frombuffer(p, dtype="<f4", count=count, offset=start).astype(float)


def decode_state(p: bytes) -> Struct:
    """Payload TYPE 0x01 (71 bytes) -> Struct. Layout = ``tlm_state_t`` in C."""
    p = bytes(p)
    f = _f32(p, 6, 16)
    return Struct(
        seq=float(int.from_bytes(p[0:2], "little")),
        t_us=float(int.from_bytes(p[2:6], "little")),
        q=f[0:4],          # [w x y z], body -> world
        bias=f[4:7],       # rad/s
        acc=f[7:10],       # g
        gyro=f[10:13],     # rad/s
        mag=f[13:16],      # uT, body frame, hard-iron removed
        flags=float(p[70]),
    )


def decode_init(p: bytes) -> Struct:
    """Payload TYPE 0x02 -> Struct. Layout = ``tlm_init_t`` in C (137 bytes;
    the older 97- and 113-byte formats are accepted too)."""
    p = bytes(p)
    nf = (len(p) - 1) // 4
    if len(p) == 137:
        nf = 33
    f = _f32(p, 0, nf)
    I = Struct(
        q0=f[0:4], gRef=f[4:7], mRef=f[7:10], b0=f[10:13],
        sigmaAcc=f[13], sigmaMag=f[14], sigmaGyro=f[15], sigmaBias=f[16],
        nomAcc=f[17], nomMag=f[18], fs=f[19],
        accTol=f[20], magTol=f[21], p0att=f[22], p0bias=f[23],
        accNisGate=0.0, magNisGate=0.0, inflDecay=0.0, dipTol=0.0,
        inflMax=1e4, lockoutPBump=0.0, persistDecay=0.0, persistRatio=0.6, persistMin=0.02,
        lockoutUpdates=0.0, graceUpdates=0.0,
    )
    if nf >= 28:
        I.accNisGate, I.magNisGate, I.inflDecay, I.dipTol = f[24], f[25], f[26], f[27]
    if len(p) == 137:
        I.inflMax, I.lockoutPBump, I.persistDecay = f[28], f[29], f[30]
        I.persistRatio, I.persistMin = f[31], f[32]
        u = np.frombuffer(p, dtype="<u2", count=2, offset=132)
        I.lockoutUpdates, I.graceUpdates = float(u[0]), float(u[1])
    fl = p[-1]
    I.estimateBias = bool(fl & 1)
    I.adaptive = bool(fl & 2)
    I.tiltComp = bool(fl & 4)
    I.decouple = bool(fl & 8)
    I.resetJacobian = bool(fl & 16)
    for k, v in list(I.items()):
        if isinstance(v, np.floating):
            I[k] = float(v)
    return I


def decode_status(p: bytes) -> Struct:
    """Payload TYPE 0x03 -> Struct. Layout = ``tlm_status_t`` in C.

    62 bytes since phase 2 (BLE fields), 54 bytes in phase-1 captures (BLE
    fields = 0). ``ble_state``: 0 off, 1 advertising, 2 connected, 3 streaming.
    """
    p = bytes(p)
    u = np.frombuffer(p, dtype="<u4", count=12, offset=0).astype(float)
    w = np.frombuffer(p, dtype="<u2", count=2, offset=48).astype(float)
    return Struct(
        uptime_s=u[0] / 1000.0,
        irq_count=u[1], bus_err=u[2], fifo_ovr=u[3],
        tlm_drop=u[4], uart_drop=u[5], proc_us_max=u[6] / 64.0,    # SYSCLK 64 MHz
        n_gyro=u[7], n_acc=u[8], n_mag=u[9], n_mag_dup=u[10], n_sh_nack=u[11],
        words_max=w[0], calib_count=w[1],
        imu_init=float(np.int8(p[52])),
        running=float(p[53]),
        ble_drop=float(np.frombuffer(p, "<u4", 1, 54)[0]) if len(p) >= 62 else 0.0,
        ble_mtu=float(np.frombuffer(p, "<u2", 1, 58)[0]) if len(p) >= 62 else 0.0,
        ble_state=float(p[60]) if len(p) >= 62 else 0.0,
        ble_err=float(p[61]) if len(p) >= 62 else 0.0,
    )


#: names of ``tlm_status_t.ble_state``
BLE_STATES = ("off", "advertising", "connected", "streaming")


def decode_replay(p: bytes) -> Struct:
    """Payload TYPE 0x04 -> Struct (see ``tlm_build_replay()`` and est_pipeline.h).

    Fields: ``win``, ``overflow``, ``ops`` (list of Struct with ``op`` in
    ``'P'|'A'|'M'``, ``v`` and ``trusted``), ``q`` (4), ``b`` (3), ``Pd`` (6),
    ``accInfl``, ``magInfl``.

    Operations: ``'P' dtheta[3] h`` (propagation), ``'A' acc[3] trusted``
    (acc update), ``'M' mag[3] extraVar`` (mag update).
    """
    p = bytes(p)
    tail = _f32(p, len(p) - 60, 15)
    R = Struct(
        win=float(int.from_bytes(p[0:4], "little")),
        overflow=p[5] != 0,
        q=tail[0:4], b=tail[4:7], Pd=tail[7:13],
        accInfl=float(tail[13]), magInfl=float(tail[14]),
        ops=[],
    )
    nops = p[4]
    i = 6
    for _ in range(nops):
        op = chr(p[i])
        if op in ("P", "M"):
            R.ops.append(Struct(op=op, v=_f32(p, i + 1, 4), trusted=False))
            i += 17
        elif op == "A":
            R.ops.append(Struct(op=op, v=_f32(p, i + 1, 3), trusted=p[i + 13] != 0))
            i += 14
        else:
            raise ValueError(f"decode_replay: unknown operation 0x{p[i]:02X}")
    return R


_IMU_CODES = {
    -1: "I2C bus error towards the LSM6DSV16X (pins PB8/PB9, power, jumpers)",
    -2: "wrong LSM6DSV16X WHO_AM_I (address 0x6B? right board?)",
    -3: "LIS2MDL does not answer through the sensor hub (J4/J5 not in Mode 3?)",
    -4: "sensor-hub timeout (J4/J5 in Mode 3? LIS2MDL powered?)",
    -5: "invalid arguments",
}


def status_diagnosis(S) -> str:
    """Translates a STATUS packet into the first broken link of the chain
    IMU -> interrupt -> I2C/DMA -> FIFO -> sensor hub -> calibration -> filter."""
    if S.imu_init == 1:
        return "IMU: initialisation in progress or stuck"
    if S.imu_init < 0:
        return "IMU: " + _IMU_CODES.get(int(S.imu_init), f"error {int(S.imu_init)}")
    if S.irq_count == 0:
        return ("IMU OK but no INT1 interrupts: PA15/EXTI15, label IMU_INT1, "
                "NVIC EXTI15_10, debug = Serial Wire")
    if S.bus_err > 0 and S.n_gyro == 0:
        return ("Interrupts OK but FIFO reads fail: DMA I2C1_RX, I2C1 event/error "
                "interrupts, HAL_I2C_MemRxCpltCallback")
    if S.n_gyro == 0:
        return "Reads OK but no gyro data in the FIFO: FIFO configuration"
    if S.n_mag == 0:
        return "Gyro/acc OK but no mag data from the sensor hub: Mode 3 (J4/J5 on 5-6), n_sh_nack"
    if not S.running:
        return (f"Data OK, calibration {int(S.calib_count)}/100: keep the board STILL "
                "(restarts if it moves)")
    return "Filter running: STATE packets should be arriving"


def read_capture(path) -> bytes:
    """Reads a raw ``.bin`` capture (tlm_capture / viewer) into bytes."""
    with open(path, "rb") as fh:
        return fh.read()


def frame_counts(frames) -> dict:
    """Number of frames per type name, e.g. ``{'STATE': 10, 'INIT': 1, ...}``."""
    counts = {name: 0 for name in TYPE_NAMES.values()}
    for fr in frames:
        name = TYPE_NAMES.get(fr.type)
        if name:
            counts[name] += 1
    return counts


def timing_report(frames) -> List[str]:
    """Firmware rates from the STATUS counters of a capture (first vs last STATUS
    of the same boot) and filter time vs real time from STATE.

    Answers "is the firmware running in real time?": at 960 Hz the board must
    show ~960 gyro samples/s, ~480 IMU interrupts/s, no FIFO overruns, and the
    filter time (STATE t_us) must advance like the uptime.
    """
    st = [(decode_status(f.payload)) for f in frames if f.type == 3]
    sv = [decode_state(f.payload) for f in frames if f.type == 1]
    out: List[str] = []
    if len(st) < 2:
        return ["  timing: fewer than 2 STATUS frames"]
    # last boot only (uptime decreasing = reboot)
    k0 = 0
    for k in range(1, len(st)):
        if st[k].uptime_s < st[k - 1].uptime_s:
            k0 = k
    a, b = st[k0], st[-1]
    du = b.uptime_s - a.uptime_s
    if du <= 0:
        return ["  timing: STATUS uptime does not advance"]
    def rate(f):
        return (b[f] - a[f]) / du
    out.append(f"  firmware over {du:.1f} s of uptime (STATUS {a.uptime_s:.1f} -> {b.uptime_s:.1f} s):")
    out.append(f"    gyro samples {rate('n_gyro'):7.1f}/s   acc {rate('n_acc'):7.1f}/s   "
               f"mag {rate('n_mag'):6.1f}/s   IMU interrupts {rate('irq_count'):6.1f}/s   (960/960/~100/~480 expected)")
    out.append(f"    FIFO overruns +{b.fifo_ovr - a.fifo_ovr:.0f}   I2C errors +{b.bus_err - a.bus_err:.0f}   "
               f"words/IRQ max {b.words_max:.0f}   processing max {b.proc_us_max:.0f} us")
    out.append(f"    telemetry drops: queue +{b.tlm_drop - a.tlm_drop:.0f}   UART +{b.uart_drop - a.uart_drop:.0f}   "
               f"BLE +{b.ble_drop - a.ble_drop:.0f}")
    if len(sv) >= 2:
        span_fw = (sv[-1].t_us - sv[0].t_us) * 1e-6
        dseq = (sv[-1].seq - sv[0].seq) % 65536
        out.append(f"    STATE: {len(sv)} frames, seq +{dseq} ({dseq / 240:.1f} s at 240 Hz), "
                   f"filter time +{span_fw:.1f} s")
        g = rate('n_gyro')
        if g > 0:
            out.append(f"    -> IMU sample rate {g:.1f} Hz ({100 * g / 960:.1f} % of the nominal 960 Hz: "
                       "the sensor's own oscillator)")
        if len(sv) >= 2 and span_fw > 0:
            # real time of the STATE span = gyro samples in it / measured sample rate
            ratio = span_fw / (dseq * 4 / g) if g > 0 else float("nan")
            out.append(f"    -> filter time / real time {100 * ratio:.1f} %"
                       + ("  (OK: every sample integrated with the measured period)" if abs(ratio - 1) < 0.005
                          else "  (!) samples lost or integrated with a wrong period"))
    return out
