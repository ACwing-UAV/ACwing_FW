"""BLE byte source for the viewer (Python side of ``ble_tlm_svc.c``), based on ``bleak``.

The firmware sends over BLE exactly the same frames as over USB (STATE, INIT,
STATUS; REPLAY never) as a **byte stream**: each notification of the TLM
characteristic carries up to ATT_MTU-3 bytes and may cut a frame anywhere. The
receiver just concatenates the notifications and feeds the usual parser, so
:class:`BleSource` has the same interface as ``SerialSource``/``FileSource``
(``read()``, ``eof``, ``close()``) and the viewer, the ``.bin`` and the ``.mat``
log are identical to the USB case.

GATT service (see ``00 - FC_FW/App/bsp/ble_tlm_svc.h``):

=========  ======================================  ==========================
Service    ``7E4A0001-2C6B-4F8E-9A5D-3B1C0E1D2F30``
TLM        ``7E4A0002-...``                         notify: frame byte stream
CMD        ``7E4A0003-...``                         write w/o response:
                                                    ``[0x01]`` resend INIT,
                                                    ``[0x02, N]`` STATE every N
=========  ======================================  ==========================

Reception runs in a **separate process** by default: ``bleak`` and the OS BLE
stack deliver notifications through Python callbacks, and in the viewer's
process the 3D redraw holds the interpreter long enough to delay them. On
Windows that showed as 23 packets/s instead of 60 and a growing lag, with no
loss. The worker process only receives, batches every ~20 ms and hands the
bytes to the viewer through a pipe. ``process=False`` keeps the previous
in-process thread (useful for debugging).
Requires ``pip install bleak`` (Windows 10+, macOS, Linux/BlueZ).
"""
from __future__ import annotations

import asyncio
import collections
import multiprocessing as mp
import queue as _queue
import threading
import time
from typing import Iterable, List, Optional, Tuple

SVC_UUID = "7e4a0001-2c6b-4f8e-9a5d-3b1c0e1d2f30"
TLM_UUID = "7e4a0002-2c6b-4f8e-9a5d-3b1c0e1d2f30"
CMD_UUID = "7e4a0003-2c6b-4f8e-9a5d-3b1c0e1d2f30"

CMD_SEND_INIT = 0x01
CMD_STATE_DECIM = 0x02

#: advertised names recognised as an ACwing board: "ACwFC" (current firmware), "IEKF..."
#: (firmware before the rename), "STM32WB" (CubeMX default name)
BOARD_NAME_PREFIXES = ("ACw", "IEKF")
BOARD_NAMES = ("STM32WB",)


def _bleak():
    try:
        import bleak
    except ImportError as exc:                                  # pragma: no cover
        raise ImportError("BLE needs the 'bleak' package: pip install bleak "
                          "(or pip install -e \".[ble]\")") from exc
    return bleak


def is_board_name(name: Optional[str]) -> bool:
    """True for the names the firmware may advertise (ACw..., IEKF..., or the CubeMX default)."""
    if not name:
        return False
    return name.startswith(BOARD_NAME_PREFIXES) or name in BOARD_NAMES


def pick_board(devices: Iterable[Tuple[str, str, int]], target: str = "") -> Optional[Tuple[str, str, int]]:
    """Chooses a device from ``(name, address, rssi)`` tuples.

    ``target`` empty: the strongest board with a recognised name. Otherwise the
    device whose name or address equals ``target`` (address case-insensitive).
    """
    devs = list(devices)
    if target:
        t = target.lower()
        for d in devs:
            if (d[0] or "") == target or d[1].lower() == t:
                return d
        return None
    boards = [d for d in devs if is_board_name(d[0])]
    return max(boards, key=lambda d: d[2]) if boards else None


def scan(timeout: float = 5.0) -> List[Tuple[str, str, int]]:
    """Advertising devices as ``(name, address, rssi)``, strongest first."""
    bleak = _bleak()

    async def _run():
        found = await bleak.BleakScanner.discover(timeout=timeout, return_adv=True)
        out = []
        for dev, adv in found.values():
            name = adv.local_name or dev.name or ""
            out.append((name, dev.address, int(adv.rssi)))
        return out

    try:
        devs = asyncio.run(_run())
    except Exception as exc:
        raise RuntimeError(_adapter_hint(exc)) from exc
    return sorted(devs, key=lambda d: -d[2])


def _adapter_hint(exc: Exception) -> str:
    return (f"Bluetooth not available ({type(exc).__name__}: {exc}). Check that the PC has a "
            "Bluetooth 4.0+ adapter and that Bluetooth is on (Linux: bluetoothd/BlueZ running).")


# ---------------------------------------------------------------- worker process
def _ble_worker(target, decim, scan_timeout, conn, stop_evt, cmd_q, fake_stream=None,
                fake_rate=5400.0, fake_mtu=247):
    """Runs in its own process: connects, receives notifications and sends
    ``("data", bytes, n_notifications)`` batches through ``conn`` every ~20 ms.
    Other messages: ``("info", name, address, rssi, mtu)``, ``("error", text)``,
    ``("eof",)``. ``fake_stream`` replaces the radio (tests, demos): the bytes
    are cut into notifications of up to ``fake_mtu``-3 bytes at ``fake_rate`` B/s.
    """
    buf = bytearray()
    count = [0]

    def on_notify(_sender, data):
        buf.extend(data)
        count[0] += 1

    def flush():
        if buf:
            conn.send(("data", bytes(buf), count[0]))
            buf.clear()
            count[0] = 0

    async def run_fake():
        conn.send(("info", "FAKE", "00:00:00:00:00:00", 0, fake_mtu))
        i, t0, step = 0, time.monotonic(), max(1, fake_mtu - 3)
        while i < len(fake_stream) and not stop_evt.is_set():
            due = int((time.monotonic() - t0) * fake_rate) if fake_rate else len(fake_stream)
            while i < min(due, len(fake_stream)):
                on_notify(None, fake_stream[i:i + step])
                i += step
            flush()
            await asyncio.sleep(0.02)
        flush()
        conn.send(("eof",))

    async def run_radio():
        bleak = _bleak()
        try:
            found = await bleak.BleakScanner.discover(timeout=scan_timeout, return_adv=True)
        except Exception as exc:
            conn.send(("error", _adapter_hint(exc)))
            return
        devs = [(adv.local_name or d.name or "", d.address, int(adv.rssi)) for d, adv in found.values()]
        dev = pick_board(devs, target)
        if dev is None:
            seen = ", ".join(f'"{n}" {a}' for n, a, _ in devs if n) or "none with a name"
            what = f'"{target}"' if target else 'a board named "ACwFC", "IEKF..." or "STM32WB"'
            conn.send(("error", f"BLE: {what} not found (devices seen: {seen}). Is it advertising "
                                "(USB viewer: \"BLE: advertising\")? Is a phone connected to it?"))
            return
        client = bleak.BleakClient(dev[1])
        try:
            await client.connect()
            if client.services.get_characteristic(TLM_UUID) is None:
                conn.send(("error", f'"{dev[0]}" has no ACwing telemetry service ({SVC_UUID}): '
                                    "is APP_TLM_BLE = 1 in the firmware?"))
                return
            await client.start_notify(TLM_UUID, on_notify)         # board: INIT sent right away
            if decim is not None:
                n = max(1, min(240, int(decim)))
                await client.write_gatt_char(CMD_UUID, bytes([CMD_STATE_DECIM, n]), response=False)
            try:
                mtu = int(client.mtu_size)
            except Exception:
                mtu = 0
            conn.send(("info", dev[0], dev[1], dev[2], mtu))
            while not stop_evt.is_set() and client.is_connected:
                try:
                    while True:
                        payload = cmd_q.get_nowait()
                        await client.write_gatt_char(CMD_UUID, bytes(payload), response=False)
                except _queue.Empty:
                    pass
                flush()
                await asyncio.sleep(0.02)
            flush()
        except Exception as exc:
            conn.send(("error", f"BLE: {type(exc).__name__}: {exc}"))
        finally:
            try:
                if client.is_connected:
                    try:
                        await client.stop_notify(TLM_UUID)
                    except Exception:
                        pass
                    await client.disconnect()
            except Exception:
                pass
            conn.send(("eof",))

    try:
        asyncio.run(run_fake() if fake_stream is not None else run_radio())
    except Exception as exc:                                         # pragma: no cover
        try:
            conn.send(("error", f"BLE worker: {type(exc).__name__}: {exc}"))
        except Exception:
            pass
    finally:
        conn.close()


class BleSource:
    """Byte source fed by the TLM notifications (same interface as ``SerialSource``).

    Parameters
    ----------
    target : name or address of the board; empty = automatic discovery
             (:func:`pick_board`).
    decim  : STATE decimation requested with the CMD characteristic
             (STATE every ``decim`` accelerometer updates: 4 -> 60 Hz);
             ``None`` keeps the firmware default (``APP_BLE_STATE_DECIM``).
    scan_timeout : seconds of scanning for the automatic discovery.
    connect : ``False`` builds an unconnected source (tests: feed it with
              :meth:`on_notify`).
    process : receive in a separate process (default, see the module docstring);
              ``False`` = thread in this process.
    fake_stream : bytes to emit as notifications instead of the radio (tests,
              demos; process mode), at ``fake_rate`` B/s.
    """

    def __init__(self, target: str = "", decim: Optional[int] = 4, scan_timeout: float = 5.0,
                 connect: bool = True, verbose: bool = True, process: bool = True,
                 fake_stream: Optional[bytes] = None, fake_rate: float = 5400.0):
        self._chunks: "collections.deque[bytes]" = collections.deque()
        self._lock = threading.Lock()
        self.eof = False
        self.n_notifications = 0
        self.n_bytes = 0
        self.mtu: Optional[int] = None
        self.name = ""
        self.address = ""
        self.verbose = verbose
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._thread: Optional[threading.Thread] = None
        self._client = None
        self._proc = None
        self._conn = None
        self._stop_evt = None
        self._cmd_q = None
        self.rssi = None
        if connect or fake_stream is not None:
            try:
                if process or fake_stream is not None:
                    self._start_process(target, decim, scan_timeout, fake_stream, fake_rate)
                else:
                    self._connect(target, decim, scan_timeout)
            except Exception:
                self.close()
                raise

    # ------------------------------------------------------------ notifications
    def on_notify(self, _sender, data) -> None:
        """Notification callback (bleak thread): only appends, never parses."""
        b = bytes(data)
        with self._lock:
            self._chunks.append(b)
            self.n_notifications += 1
            self.n_bytes += len(b)
        if self._client is not None and not self._client.is_connected:   # pragma: no cover
            self.eof = True

    def read(self) -> bytes:
        """Everything received since the last call (``b""`` after a short wait if nothing)."""
        if self._conn is not None:
            return self._read_process()
        with self._lock:
            if self._chunks:
                out = b"".join(self._chunks)
                self._chunks.clear()
                return out
        if self._client is not None and not self._client.is_connected:   # pragma: no cover
            self.eof = True
        time.sleep(0.005)
        return b""

    # ------------------------------------------------------------ process mode
    def _start_process(self, target, decim, scan_timeout, fake_stream, fake_rate):
        ctx = mp.get_context("spawn")                    # same behaviour on Windows, macOS, Linux
        parent, child = ctx.Pipe(duplex=False)
        self._stop_evt = ctx.Event()
        self._cmd_q = ctx.Queue()
        self._proc = ctx.Process(target=_ble_worker, daemon=True,
                                 args=(target, decim, scan_timeout, child, self._stop_evt,
                                       self._cmd_q, fake_stream, fake_rate))
        if self.verbose and fake_stream is None:
            print(f"Scanning for the board ({scan_timeout:.0f} s)...")
        self._proc.start()
        child.close()
        self._conn = parent
        deadline = time.monotonic() + scan_timeout + 40.0
        while time.monotonic() < deadline:                # wait for "info" or "error"
            if not parent.poll(0.1):
                if not self._proc.is_alive():
                    raise RuntimeError("BLE worker process ended before connecting")
                continue
            msg = parent.recv()
            if msg[0] == "info":
                _, self.name, self.address, self.rssi, mtu = msg
                self.mtu = mtu or None
                if self.verbose:
                    rate = f"{240 / max(1, int(decim)):.0f} Hz" if decim else "firmware default"
                    print(f'Connected to "{self.name}" ({self.address}, RSSI {self.rssi} dBm): '
                          f"STATE {rate}" + (f", ATT MTU {self.mtu}" if self.mtu else ""))
                return
            if msg[0] == "error":
                raise RuntimeError(msg[1])
            if msg[0] == "eof":
                raise RuntimeError("BLE worker process ended before connecting")
        raise RuntimeError("BLE: no answer from the worker process (connection timeout)")

    def _read_process(self) -> bytes:
        out = []
        try:
            while self._conn.poll():
                msg = self._conn.recv()
                if msg[0] == "data":
                    out.append(msg[1])
                    self.n_notifications += msg[2]
                    self.n_bytes += len(msg[1])
                elif msg[0] == "error":
                    print(f"ERROR: {msg[1]}")
                    self.eof = True
                elif msg[0] == "eof":
                    self.eof = True
        except (EOFError, OSError):
            self.eof = True
        if out:
            return b"".join(out)
        time.sleep(0.005)
        return b""

    # ------------------------------------------------------------ thread mode
    def _run(self, coro, timeout: float = 20.0):
        return asyncio.run_coroutine_threadsafe(coro, self._loop).result(timeout)

    def _connect(self, target: str, decim: Optional[int], scan_timeout: float) -> None:  # pragma: no cover
        bleak = _bleak()
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._loop.run_forever, daemon=True)
        self._thread.start()

        dev = self._discover(bleak, target, scan_timeout)
        self.name, self.address = dev[0], dev[1]
        if self.verbose:
            print(f'Connecting to "{self.name}" ({self.address}, RSSI {dev[2]} dBm)...')

        async def _open():
            client = bleak.BleakClient(self.address)
            await client.connect()
            if client.services.get_characteristic(TLM_UUID) is None:
                await client.disconnect()
                raise RuntimeError(f'"{self.name}" has no ACwing telemetry service ({SVC_UUID}): '
                                   "is APP_TLM_BLE = 1 in the firmware?")
            await client.start_notify(TLM_UUID, self.on_notify)   # board: INIT sent right away
            if decim is not None:
                n = max(1, min(240, int(decim)))
                await client.write_gatt_char(CMD_UUID, bytes([CMD_STATE_DECIM, n]), response=False)
            return client

        self._client = self._run(_open(), timeout=30.0)
        try:
            self.mtu = int(self._client.mtu_size)
        except Exception:
            self.mtu = None
        if self.verbose:
            rate = f"{240 / max(1, int(decim)):.0f} Hz" if decim else "firmware default"
            print(f"Connected: STATE {rate}" + (f", ATT MTU {self.mtu}" if self.mtu else ""))

    def _discover(self, bleak, target: str, scan_timeout: float):  # pragma: no cover
        async def _scan():
            found = await bleak.BleakScanner.discover(timeout=scan_timeout, return_adv=True)
            return [(adv.local_name or d.name or "", d.address, int(adv.rssi)) for d, adv in found.values()]

        if self.verbose:
            print(f"Scanning for the board ({scan_timeout:.0f} s)...")
        try:
            devs = self._run(_scan(), timeout=scan_timeout + 10)
        except Exception as exc:
            raise RuntimeError(_adapter_hint(exc)) from exc
        dev = pick_board(devs, target)
        if dev is None:
            seen = ", ".join(f'"{n}" {a}' for n, a, _ in devs if n) or "none with a name"
            what = f'"{target}"' if target else 'a board named "ACwFC", "IEKF..." or "STM32WB"'
            raise RuntimeError(f"BLE: {what} not found (devices seen: {seen}). Is it advertising "
                               "(USB viewer: \"BLE: advertising\")? Is a phone connected to it?")
        return dev

    def send_command(self, payload: bytes) -> None:  # pragma: no cover
        """Writes the CMD characteristic (e.g. ``bytes([0x01])`` = resend INIT)."""
        if self._cmd_q is not None:
            self._cmd_q.put(bytes(payload))
            return
        if self._client is None:
            raise RuntimeError("not connected")
        self._run(self._client.write_gatt_char(CMD_UUID, bytes(payload), response=False))

    def close(self) -> None:
        if self._proc is not None:
            # keep draining the pipe while the worker stops: a worker blocked in
            # send() on a full pipe could not see stop_evt (BufferError on exit)
            self._stop_evt.set()
            t_end = time.monotonic() + 8.0
            while self._proc.is_alive() and time.monotonic() < t_end:
                try:
                    while self._conn.poll():
                        self._conn.recv()
                except (EOFError, OSError):
                    pass
                self._proc.join(0.05)
            if self._proc.is_alive():                              # pragma: no cover
                self._proc.terminate()
            self._proc = None
            try:
                self._conn.close()
            except Exception:
                pass
        if self._client is not None and self._loop is not None:   # pragma: no cover
            async def _close():
                try:
                    await self._client.stop_notify(TLM_UUID)
                except Exception:
                    pass
                await self._client.disconnect()
            try:
                self._run(_close(), timeout=10.0)
            except Exception:
                pass
        if self._loop is not None:                                 # pragma: no cover
            self._loop.call_soon_threadsafe(self._loop.stop)
        self._client = None
