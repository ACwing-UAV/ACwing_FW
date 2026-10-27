"""Live 3D viewer of the attitude estimated by the firmware (port of ``acwingLiveViewer.m``).

A 3D board + body triad follows the attitude in the levelled NWU world frame
(x = magnetic north, y = west, z = up) defined by the firmware's static
calibration: keep the board STILL ~2 s after reset. The view shows x to the
right, y into the screen and z up.

- Reception runs in a background thread: the plot never slows it down.
- Every received byte is written to a raw ``.bin`` (usable by ``compare``).
- The decoded log is saved as ``.mat`` (same ``L`` struct as the MATLAB
  viewer, see ``logio.py``) every 10 s and on exit (window closed or Ctrl+C).
- ``source`` can be a serial port, the BLE link (``ble_io.BleSource``: same
  frames, STATE decimated) or a recorded ``.bin`` (``replay``), so the viewer
  also runs without the board.

With ``APP_TLM_REPLAY = 1`` (~44 kB/s) use ``capture`` instead.
"""
from __future__ import annotations

import os
import queue
import sys
import threading
import time
from datetime import datetime

import numpy as np

from .common import Struct, eul_zyx, quat2rotm
from .logio import save_log
from .serial_io import FileSource, SerialSource
from .telemetry import (BLE_STATES, F_ACC_GATED, F_MAG_GATED, F_OVERRUN, F_STILL, decode_init,
                        decode_state, decode_status, parse_telemetry, status_diagnosis)

_FIELDS = (("seq", 1), ("t_us", 1), ("q", 4), ("bias", 3), ("acc", 3), ("gyro", 3),
           ("mag", 3), ("flags", 1), ("t_pc", 1))


class _Log:
    """Growable column store with the layout of the MATLAB ``L`` struct."""

    def __init__(self, cap=6000):
        self.n = 0
        self.a = {k: np.zeros((r, cap)) for k, r in _FIELDS}
        self.init = None
        self.status = []
        self.reset_at = []

    def append(self, **vals):
        if self.n == self.a["seq"].shape[1]:
            for k in self.a:
                self.a[k] = np.concatenate((self.a[k], np.zeros_like(self.a[k])), axis=1)
        for k, v in vals.items():
            self.a[k][:, self.n] = v
        self.n += 1

    def struct(self) -> Struct:
        L = Struct({k: (self.a[k][:, :self.n] if r > 1 else self.a[k][0, :self.n]) for k, r in _FIELDS})
        L.init = self.init
        L.status = list(self.status)
        L.resetAt = np.asarray(self.reset_at, dtype=float)
        return L


class LiveViewer:
    """See the module docstring. Use :func:`live_viewer` for the one-call form."""

    def __init__(self, source, log_dir: str = ".", stop_at_eof: bool = False, show: bool = True):
        self.source = source
        self.stop_at_eof = stop_at_eof
        self.show = show
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_file = os.path.join(log_dir, f"acwing_log_{stamp}.mat")
        self.bin_file = self.log_file[:-4] + ".bin"
        self.log = _Log()
        self.q_rx: "queue.Queue[bytes]" = queue.Queue()
        self.stop = threading.Event()
        self.buf = b""
        self.last_seq = None
        self.seq_offset = 0
        self.seq_step = None          # STATE decimation seen in the stream (1, 4, 8, ...)
        self.n_lost = self.n_frames = self.n_status = self.n_replay = self.n_resets = 0
        self.last_uptime = -1.0
        self.latest_q = np.array([1.0, 0.0, 0.0, 0.0])
        self.latest_s = None
        self.latest_status = None
        self.lag_ref = None
        self.t_start = time.monotonic()
        self.finished = False
        self.draw_ms = 0.0                                     # smoothed redraw time

    # ------------------------------------------------------------ reception
    def _reader(self):
        with open(self.bin_file, "wb") as fh:
            while not self.stop.is_set():
                try:
                    chunk = self.source.read()
                except Exception as exc:                        # port unplugged, ...
                    print(f"ERROR reading the source: {exc}", file=sys.stderr)
                    self.stop.set()
                    break
                if chunk:
                    fh.write(chunk)
                    self.q_rx.put(chunk)
                elif getattr(self.source, "eof", False) and self.stop_at_eof:
                    break
        self.q_rx.put(None)                                    # end marker

    def _handle_state(self, s):
        if self.last_seq is not None:
            d = int(s.seq - self.last_seq) % 65536
            if d == 0:
                return                                         # duplicate
            # decimated streams (BLE, REPLAY on) step by N: count only real gaps
            self.seq_step = d if self.seq_step is None else min(self.seq_step, d)
            self.n_lost += max(0, int(round(d / self.seq_step)) - 1)
            if s.seq < self.last_seq:
                self.seq_offset += 65536                       # wrap-around
        self.last_seq = s.seq
        self.n_frames += 1
        t_pc = time.monotonic() - self.t_start
        self.log.append(seq=s.seq + self.seq_offset, t_us=s.t_us, q=s.q, bias=s.bias, acc=s.acc,
                        gyro=s.gyro, mag=s.mag, flags=s.flags, t_pc=t_pc)
        if self.lag_ref is None or s.t_us * 1e-6 < self.lag_ref[1]:
            self.lag_ref = (t_pc, s.t_us * 1e-6)
        self.latest_q = s.q
        self.latest_s = s

    def _handle_status(self, st):
        self.latest_status = st
        self.n_status += 1
        if self.last_uptime >= 0 and st.uptime_s < self.last_uptime:   # the board rebooted
            self.n_resets += 1
            self.log.reset_at.append(self.log.n + 1)
            print(f"WARNING: board reboot detected (sample {self.log.n + 1})", file=sys.stderr)
            self.last_seq = None
        self.last_uptime = st.uptime_s
        self.log.status.append(st)

    def _consume(self, chunk):
        self.buf += chunk
        frames, self.buf = parse_telemetry(self.buf, (4,))    # REPLAY: only in the .bin
        for fr in frames:
            if fr.type == 1:
                self._handle_state(decode_state(fr.payload))
            elif fr.type == 2:
                self.log.init = decode_init(fr.payload)
            elif fr.type == 3:
                self._handle_status(decode_status(fr.payload))
            elif fr.type == 4:
                self.n_replay += 1

    # ------------------------------------------------------------ display
    def status_text(self) -> str:
        R = quat2rotm(self.latest_q)
        roll, pitch, yaw = np.degrees(eul_zyx(R))
        el = time.monotonic() - self.t_start
        rate = self.n_frames / max(el, 1e-3)
        txt = (f"roll {roll:7.1f}   pitch {pitch:7.1f}   yaw {yaw:7.1f}  [deg]\n"
               f"packets {self.n_frames}  ({rate:.0f} Hz)   lost {self.n_lost}   log {self.log.n} samples")
        s = self.latest_s
        if s is not None:
            # display lag: elapsed PC time - elapsed firmware time
            lag = (el - self.lag_ref[0]) - (s.t_us * 1e-6 - self.lag_ref[1])
            txt += f"   lag {lag:.2f} s"
            if lag > 0.5:
                txt += "  <-- VIEWER CANNOT KEEP UP"
            if self.draw_ms > 0:
                txt += f"   draw {self.draw_ms:.0f} ms"
            txt += "\nbias [deg/s]  {:6.3f} {:6.3f} {:6.3f}".format(*np.degrees(s.bias))
            fl = int(s.flags)
            tags = [n for b, n in ((F_ACC_GATED, "ACC de-weighted"), (F_MAG_GATED, "MAG de-weighted"),
                                   (F_OVERRUN, "OVERRUN"), (F_STILL, "STILL")) if fl & b]
            if tags:
                txt += "\n" + "  |  ".join(tags)
        I = self.log.init
        if I:
            txt += (f"\ncalib: sigmaAcc {I.sigmaAcc:.4f}  sigmaMag {I.sigmaMag:.4f}  "
                    f"dip {-np.degrees(np.arcsin(np.clip(I.mRef[2], -1, 1))):.1f} deg")
        st = self.latest_status
        if st is not None and st.ble_state > 0:
            txt += (f"\nBLE: {BLE_STATES[int(st.ble_state)] if st.ble_state < 4 else '?'}"
                    f"  MTU {st.ble_mtu:.0f}  drop {st.ble_drop:.0f}")
            if st.ble_err:
                txt += f"  err 0x{int(st.ble_err):02X}"
            n_notif = getattr(self.source, "n_notifications", None)
            if n_notif is not None:
                txt += f"  ({n_notif} notif., {n_notif / max(el, 1e-3):.0f}/s)"
        if s is None and self.latest_status is not None:
            txt = "STATUS (no attitude data yet)\n" + status_diagnosis(self.latest_status)
        if s is None and self.latest_status is None:
            txt = "waiting for data... (keep the board still after reset)"
        return txt

    def _build_figure(self):
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection

        fig = plt.figure("ACwing FC - live attitude", figsize=(7.5, 8))
        ax = fig.add_axes([0.02, 0.24, 0.96, 0.74], projection="3d")
        lim = 1.6
        ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim); ax.set_zlim(-lim, lim)
        ax.set_box_aspect((1, 1, 1))
        ax.set_xlabel("x  (N)"); ax.set_ylabel("y  (W)"); ax.set_zlabel("z  (Up)")
        # camera behind the scene looking along +y: x to the right, y into the
        # screen, z up (a small elevation keeps the depth readable)
        ax.view_init(elev=20, azim=-90)
        grey = (0.5, 0.5, 0.5)
        for e, lbl in ((np.array([1.4, 0, 0]), "N"), (np.array([0, 1.4, 0]), "W"), (np.array([0, 0, 1.4]), "Up")):
            ax.plot([0, e[0]], [0, e[1]], [0, e[2]], ":", color=grey, lw=1)
            ax.text(*(e * 1.07), lbl, color=grey)
        a, b, c = 0.9, 0.55, 0.05
        self._V = np.array([[-a, -b, -c], [a, -b, -c], [a, b, -c], [-a, b, -c],
                            [-a, -b, c], [a, -b, c], [a, b, c], [-a, b, c]])
        self._F = [[0, 1, 2, 3], [4, 5, 6, 7], [0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]]
        self._board = Poly3DCollection([self._V[f] for f in self._F], facecolor=(0.2, 0.45, 0.3, 0.35),
                                       edgecolor=(0.1, 0.25, 0.15))
        ax.add_collection3d(self._board)
        cols = ((0.85, 0.1, 0.1), (0.1, 0.6, 0.1), (0.1, 0.2, 0.85))
        self._axes = [ax.plot([0, 0], [0, 0], [0, 0], color=cols[i], lw=3)[0] for i in range(3)]
        self._labels = [ax.text(0, 0, 0, lbl, color=cols[i], fontweight="bold")
                        for i, lbl in enumerate(("x_b", "y_b", "z_b"))]
        self._txt = fig.text(0.02, 0.01, "", family="monospace", fontsize=9, va="bottom")
        self._fig = fig
        self._closed = False
        fig.canvas.mpl_connect("close_event", lambda evt: setattr(self, "_closed", True))
        self._redraw()
        if self.show:
            plt.show(block=False)
        return fig

    def _redraw(self):
        R = quat2rotm(self.latest_q)
        Vw = self._V @ R.T
        self._board.set_verts([Vw[f] for f in self._F])
        for i in range(3):
            e = R[:, i] * 1.25
            self._axes[i].set_data_3d([0, e[0]], [0, e[1]], [0, e[2]])
            self._labels[i].set_position_3d(tuple(1.1 * e))
        self._txt.set_text(self.status_text())
        self._fig.canvas.draw_idle()

    # ------------------------------------------------------------ main loop
    def run(self, refresh_hz: float = 15.0) -> Struct:
        import matplotlib.pyplot as plt

        print(f"Log: {self.log_file}\nRaw capture: {self.bin_file}")
        self._build_figure()
        reader = threading.Thread(target=self._reader, daemon=True)
        reader.start()
        t_save = time.monotonic()
        t_draw = 0.0
        ended = False
        try:
            while not self._closed and not ended:
                got = False
                while True:
                    try:
                        chunk = self.q_rx.get_nowait()
                    except queue.Empty:
                        break
                    if chunk is None:
                        if self.stop_at_eof and self.show:     # replay done: keep the window
                            print("End of the replayed capture: close the window to save the log.")
                            self._redraw()
                        else:
                            ended = True
                        break
                    self._consume(chunk)
                    got = True
                now = time.monotonic()
                if got and now - t_draw > 1.0 / refresh_hz:
                    t0 = time.monotonic()
                    self._redraw()
                    if self.show:
                        self._fig.canvas.flush_events()       # draw now, to time it
                    self.draw_ms = 0.8 * self.draw_ms + 0.2 * 1e3 * (time.monotonic() - t0)
                    t_draw = now
                if now - t_save > 10:
                    self.save()
                    t_save = now
                if self.show:
                    # GUI events without plt.pause (which re-shows the window at
                    # every call and makes it sluggish on Windows)
                    self._fig.canvas.start_event_loop(0.01)
                else:
                    time.sleep(0.005)
        except KeyboardInterrupt:
            pass
        finally:
            self.finish()
        return self.log.struct()

    def save(self):
        if self.log.n == 0 and self.n_status == 0:
            return
        try:
            save_log(self.log_file, self.log.struct())
        except OSError as exc:
            print(f"ERROR while saving {self.log_file}:\n  {exc}\n"
                  "Use a writable folder: --log-dir <path>", file=sys.stderr)

    def finish(self):
        if self.finished:
            return
        self.finished = True
        self.stop.set()
        self.source.close()
        self.save()
        print(f"Log: {self.log_file}\n  {self.log.n} STATE samples, {self.n_status} STATUS, "
              f"{self.n_lost} lost, {self.n_resets} reboots.")
        if self.n_replay > 0:
            print(f"  {self.n_replay} REPLAY packets: python -m acwing_tools compare {self.bin_file}")


def live_viewer(port: str | None = None, baud: int = 921600, log_dir: str = ".",
                replay: str | None = None, replay_rate: float | None = 19000.0,
                show: bool = True, ble: str | None = None, ble_decim: int | None = 4) -> Struct:
    """One-call viewer. Exactly one source:

    - ``port``: serial port (e.g. ``"COM6"``, ``"/dev/ttyACM0"``);
    - ``ble``: BLE board name or address, ``""`` = automatic discovery; STATE
      every ``ble_decim`` acc updates (4 -> 60 Hz);
    - ``replay``: a recorded ``.bin``, played at ``replay_rate`` bytes/s.

    Returns the log Struct."""
    if ble is not None:
        from .ble_io import BleSource
        return LiveViewer(BleSource(ble, decim=ble_decim), log_dir=log_dir, show=show).run()
    if replay:
        src = FileSource(replay, rate=replay_rate)
        stop_at_eof = True
    else:
        if not port:
            raise ValueError("give a serial port or a .bin to replay")
        src = SerialSource(port, baud)
        stop_at_eof = False
    return LiveViewer(src, log_dir=log_dir, stop_at_eof=stop_at_eof, show=show).run()
