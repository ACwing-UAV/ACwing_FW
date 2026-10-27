"""Raw telemetry capture and byte-level sniffer (ports of ``tlmCapture.m`` and ``tlmSniffer.m``)."""
from __future__ import annotations

import os
import time
import warnings
from datetime import datetime

import numpy as np

from .serial_io import open_serial
from .telemetry import decode_status, frame_counts, parse_telemetry, status_diagnosis


def tlm_capture(port: str, duration: float = 60.0, baud: int = 921600, out_dir: str = ".") -> str:
    """Records the raw telemetry to ``acwing_cap_YYYYMMDD_HHMMSS.bin``, no decoding.

    Used for the comparison with the reference filters (``APP_TLM_REPLAY = 1``):
    at ~44 kB/s a live viewer cannot keep up, while writing bytes to disk costs
    almost nothing. Analyse the file afterwards with ``compare``.

    Procedure: start the capture, THEN reset the board, keep it still for
    2-4 s, then move it. Window 0 of the trace arrives right after the
    calibration. Returns the file path.
    """
    path = os.path.join(out_dir, datetime.now().strftime("acwing_cap_%Y%m%d_%H%M%S.bin"))
    ser = open_serial(port, baud)
    print(f"Recording to {path} for {duration:.0f} s: reset the board NOW.")
    n_tot = 0
    n_half = None
    t_half = 0.0
    t0 = time.monotonic()
    t_next = 5.0
    try:
        with open(path, "wb") as fh:
            while (el := time.monotonic() - t0) < duration:
                n = ser.in_waiting
                if n > 0:
                    chunk = ser.read(n)
                    fh.write(chunk)
                    n_tot += len(chunk)
                else:
                    time.sleep(0.01)
                if n_half is None and el > duration / 2:
                    n_half, t_half = n_tot, el
                if el > t_next:
                    print(f"  {el:3.0f} s: {n_tot:8d} bytes ({n_tot / el / 1000:.1f} kB/s)")
                    t_next += 5
    finally:
        ser.close()
    print(f"Done: {n_tot} bytes ({n_tot / duration / 1000:.1f} kB/s on average).")
    print(f"  python -m acwing_tools compare {path}")
    # rate over the second half only: the first half includes the reset and the
    # 2 s calibration, during which only STATUS (~0.6 kB/s) is sent
    if n_half is not None and duration > t_half:
        rate2 = (n_tot - n_half) / (duration - t_half)
        print(f"Rate over the second half: {rate2 / 1000:.1f} kB/s (expected ~44 kB/s with REPLAY)")
        if rate2 < 35e3:
            warnings.warn("Second half below 35 kB/s: REPLAY not enabled, filter not started, or "
                          f"baud rate different from {baud} (check USART1 in CubeMX).")
    return path


def ble_capture(target: str = "", duration: float = 30.0, decim: int | None = 4,
                out_dir: str = ".") -> str:
    """Records the BLE telemetry to ``acwing_ble_YYYYMMDD_HHMMSS.bin``, no graphics.

    Every 2 s it prints what the PC actually receives:
    - notifications/s and kB/s;
    - STATE/s (expected 240/decim);
    - the lag between PC time and firmware time (growing = data delivered late);
    - the BLE fields of the last STATUS (state, MTU, firmware-side drops).

    Use it to tell a slow link (low notifications/s, firmware drops) from a slow
    viewer (rates here are fine but the viewer is late). Returns the file path.
    """
    from .ble_io import BleSource
    from .telemetry import BLE_STATES, decode_state

    path = os.path.join(out_dir, datetime.now().strftime("acwing_ble_%Y%m%d_%H%M%S.bin"))
    src = BleSource(target, decim=decim)
    print(f"Recording to {path} for {duration:.0f} s (Ctrl+C to stop).")
    buf = b""
    n_state = n_state_win = 0
    notif_prev = bytes_prev = 0
    first = None                     # (pc time, firmware time) of the first STATE
    lag = float("nan")
    st = None
    t0 = time.monotonic()
    t_next = 2.0
    try:
        with open(path, "wb") as fh:
            while (el := time.monotonic() - t0) < duration and not src.eof:
                chunk = src.read()
                if chunk:
                    fh.write(chunk)
                    frames, buf = parse_telemetry(buf + chunk)
                    for fr in frames:
                        if fr.type == 1:
                            s = decode_state(fr.payload)
                            n_state += 1
                            n_state_win += 1
                            now = time.monotonic()
                            if first is None or s.t_us * 1e-6 < first[1]:
                                first = (now, s.t_us * 1e-6)
                            lag = (now - first[0]) - (s.t_us * 1e-6 - first[1])
                        elif fr.type == 3:
                            st = decode_status(fr.payload)
                if el > t_next:
                    dn = src.n_notifications - notif_prev
                    db = src.n_bytes - bytes_prev
                    notif_prev, bytes_prev = src.n_notifications, src.n_bytes
                    line = (f"  {el:4.0f} s: {dn / 2:5.0f} notif/s  {db / 2000:5.2f} kB/s  "
                            f"STATE {n_state_win / 2:5.1f}/s  lag {lag:6.2f} s")
                    if st is not None and st.ble_state > 0:
                        line += (f"  | fw: {BLE_STATES[int(st.ble_state)] if st.ble_state < 4 else '?'}"
                                 f" MTU {st.ble_mtu:.0f} drop {st.ble_drop:.0f}")
                    print(line)
                    n_state_win = 0
                    t_next += 2.0
    except KeyboardInterrupt:
        pass
    finally:
        src.close()
    el = time.monotonic() - t0
    print(f"Done: {src.n_bytes} bytes, {src.n_notifications} notifications, {n_state} STATE in {el:.1f} s "
          f"({n_state / max(el, 1e-3):.1f} STATE/s, expected {240 / max(1, decim or 4):.0f}).")
    return path


def tlm_sniffer(port: str, baud: int = 921600, duration: float = 3.0, data: bytes | None = None):
    """Byte-level telemetry debugging.

    Answers, in order: are bytes arriving? are they valid frames? what does the
    firmware say about itself (STATUS packet)? ``data`` analyses given bytes
    instead of reading the port (offline use and tests). Returns the last
    STATUS Struct, or None.
    """
    if data is None:
        ser = open_serial(port, baud)
        time.sleep(duration)
        n = ser.in_waiting
        buf = ser.read(n) if n > 0 else b""
        ser.close()
    else:
        buf = bytes(data)
    nb = len(buf)
    if data is None:
        print(f"\n{port} @ {baud} baud: {nb} bytes in {duration:.1f} s ({nb / duration:.0f} B/s)")
    else:
        print(f"\n{port}: {nb} bytes")
    if nb == 0:
        print("No bytes. Check: firmware running (green LED blinking?),\n"
              "right COM port (\"STMicroelectronics STLink Virtual COM Port\"),\n"
              "USART1 on PB6/PB7, no other program holding the port open.")
        return None

    arr = np.frombuffer(buf, dtype=np.uint8)
    n_sync = int(np.sum((arr[:-1] == 0xA5) & (arr[1:] == 0x5A)))
    frames, _ = parse_telemetry(buf)
    c = frame_counts(frames)
    print(f"A5 5A sync sequences: {n_sync}   valid frames (CRC ok): {len(frames)}")
    print(f"  STATE: {c['STATE']}   INIT: {c['INIT']}   STATUS: {c['STATUS']}   REPLAY: {c['REPLAY']}")
    print("First bytes: " + " ".join(f"{b:02X}" for b in buf[:24]))

    u = np.unique(arr)
    if u.size <= 2:
        print(f"The stream contains only {u.size} distinct byte values "
              f"({' '.join(f'{b:02X}' for b in u)}): the DMA keeps\n"
              "sending the same byte. In CubeMX, USART1 -> DMA Settings -> USART1_TX:\n"
              "tick \"Increment Address\" on Memory (not on Peripheral). Check the\n"
              "same setting on I2C1_RX.")
        return None
    if not frames:
        print("Bytes present but no valid frame: baud rate differs between firmware\n"
              "and PC (check USART1 in CubeMX), or SYSCLK is not 64 MHz.")
        return None
    if c["STATUS"] == 1 and nb < 200:
        print("A single frame: the transmit-complete callback never arrives and the DMA\n"
              "stays \"busy\". Check the USART1 global interrupt and HAL_UART_TxCpltCallback\n"
              "-> app_uart_tx_cplt in main.c.")
    st = [fr for fr in frames if fr.type == 3]
    if not st:
        return None
    S = decode_status(st[-1].payload)
    for k, v in S.items():
        print(f"  {k:12s} {v:g}")
    print(f"DIAGNOSIS: {status_diagnosis(S)}")
    return S
