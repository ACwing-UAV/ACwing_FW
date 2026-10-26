"""BLE byte source for the viewer (Python side of ``ble_tlm_svc.c``), based on ``bleak``.

The firmware sends over BLE exactly the same frames as over USB (STATE, INIT,
STATUS; REPLAY never) as a **byte stream**: each notification of the TLM
characteristic carries up to ATT_MTU-3 bytes and may cut a frame anywhere. The
receiver just concatenates the notifications and feeds the usual parser, so
:class:`BleSource` has the same interface as ``SerialSource``/``FileSource``
(``read()``, ``eof``, ``close()``) and the viewer, the ``.bin`` and the ``.mat``
log are identical to the USB case.

GATT service (see ``00 - STM32_FW/App/platform/ble_tlm_svc.h``):

=========  ======================================  ==========================
Service    ``7E4A0001-2C6B-4F8E-9A5D-3B1C0E1D2F30``
TLM        ``7E4A0002-...``                         notify: frame byte stream
CMD        ``7E4A0003-...``                         write w/o response:
                                                    ``[0x01]`` resend INIT,
                                                    ``[0x02, N]`` STATE every N
=========  ======================================  ==========================

``bleak`` runs on asyncio: here it lives in a private event loop on a
background thread, so the rest of the tools stay synchronous.
Requires ``pip install bleak`` (Windows 10+, macOS, Linux/BlueZ).
"""
from __future__ import annotations

import asyncio
import collections
import threading
import time
from typing import Iterable, List, Optional, Tuple

SVC_UUID = "7e4a0001-2c6b-4f8e-9a5d-3b1c0e1d2f30"
TLM_UUID = "7e4a0002-2c6b-4f8e-9a5d-3b1c0e1d2f30"
CMD_UUID = "7e4a0003-2c6b-4f8e-9a5d-3b1c0e1d2f30"

CMD_SEND_INIT = 0x01
CMD_STATE_DECIM = 0x02

#: advertised names recognised as an IEKF board ("STM32WB" = CubeMX default name)
BOARD_NAME_PREFIXES = ("IEKF",)
BOARD_NAMES = ("STM32WB",)


def _bleak():
    try:
        import bleak
    except ImportError as exc:                                  # pragma: no cover
        raise ImportError("BLE needs the 'bleak' package: pip install bleak "
                          "(or pip install -e \".[ble]\")") from exc
    return bleak


def is_board_name(name: Optional[str]) -> bool:
    """True for the names the firmware may advertise (IEKF..., or the CubeMX default)."""
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
    """

    def __init__(self, target: str = "", decim: Optional[int] = 4, scan_timeout: float = 5.0,
                 connect: bool = True, verbose: bool = True):
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
        if connect:
            try:
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
        with self._lock:
            if self._chunks:
                out = b"".join(self._chunks)
                self._chunks.clear()
                return out
        if self._client is not None and not self._client.is_connected:   # pragma: no cover
            self.eof = True
        time.sleep(0.005)
        return b""

    # ------------------------------------------------------------ connection
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
                raise RuntimeError(f'"{self.name}" has no IEKF telemetry service ({SVC_UUID}): '
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
            what = f'"{target}"' if target else 'a board named "IEKF..." or "STM32WB"'
            raise RuntimeError(f"BLE: {what} not found (devices seen: {seen}). Is it advertising "
                               "(USB viewer: \"BLE: advertising\")? Is a phone connected to it?")
        return dev

    def send_command(self, payload: bytes) -> None:  # pragma: no cover
        """Writes the CMD characteristic (e.g. ``bytes([0x01])`` = resend INIT)."""
        if self._client is None:
            raise RuntimeError("not connected")
        self._run(self._client.write_gatt_char(CMD_UUID, bytes(payload), response=False))

    def close(self) -> None:
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
