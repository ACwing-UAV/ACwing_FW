"""Byte sources for the viewer and the capture: serial port or a recorded ``.bin``."""
from __future__ import annotations

import time


def open_serial(port: str, baud: int = 921600, timeout: float = 0.05):
    """Opens the ST-LINK virtual COM port (pyserial), input buffer flushed."""
    try:
        import serial
    except ImportError as exc:                                  # pragma: no cover
        raise ImportError("pyserial is required: pip install pyserial") from exc
    try:
        ser = serial.Serial(port, baudrate=baud, timeout=timeout)
    except serial.SerialException as exc:
        if isinstance(getattr(exc, "args", [None])[0], str) and "could not open port" in exc.args[0] \
                and ("FileNotFoundError" in exc.args[0] or "No such file" in exc.args[0]):
            raise OSError(f"port {port} not found: is the cable plugged in? The ports present: "
                          "python -m serial.tools.list_ports -v") from None
        raise
    except FileNotFoundError:
        raise OSError(f"port {port} not found: is the cable plugged in? The ports present: "
                      "python -m serial.tools.list_ports -v") from None
    ser.reset_input_buffer()
    return ser


class SerialSource:
    """Reads whatever is available on the serial port (non-blocking beyond ``timeout``)."""

    def __init__(self, port: str, baud: int = 921600):
        self.ser = open_serial(port, baud)
        self.eof = False

    def read(self) -> bytes:
        n = self.ser.in_waiting
        return self.ser.read(n if n > 0 else 1)

    def close(self):
        try:
            self.ser.close()
        except Exception:                                       # pragma: no cover
            pass


class FileSource:
    """Replays a raw capture at ``rate`` bytes/s (``None`` = as fast as possible).

    Lets the viewer and the other tools run without the board.
    """

    def __init__(self, path, rate: float | None = 19000.0, chunk: int = 512):
        with open(path, "rb") as fh:
            self.data = fh.read()
        self.pos = 0
        self.rate = rate
        self.chunk = chunk
        self.t0 = time.monotonic()
        self.eof = False

    def read(self) -> bytes:
        if self.pos >= len(self.data):
            self.eof = True
            time.sleep(0.01)
            return b""
        if self.rate:
            due = int((time.monotonic() - self.t0) * self.rate)
            n = max(0, min(due - self.pos, len(self.data) - self.pos))
            if n == 0:
                time.sleep(0.005)
                return b""
        else:
            n = min(self.chunk, len(self.data) - self.pos)
        out = self.data[self.pos:self.pos + n]
        self.pos += n
        return out

    def close(self):
        pass
