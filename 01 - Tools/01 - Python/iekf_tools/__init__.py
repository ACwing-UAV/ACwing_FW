"""Python tools for the STM32WB55 IEKF attitude firmware.

Python equivalents of the MATLAB tools in ``01 - Tools/00 - Matlab/``: telemetry parsing and
decoding, live 3D viewer, raw capture, byte-level sniffer, magnetometer
calibration, and the comparison of the embedded filter with the original and
the extended MATLAB filters (both ported here).

Conventions (same as the MATLAB tools and the firmware):
- quaternions are scalar-first ``[w, x, y, z]``, body -> world;
- world frame NWU (x = magnetic north, y = west, z = up);
- decoded packets are :class:`Struct` objects (a ``dict`` with attribute
  access) whose field names are those of the MATLAB structs, so logs saved as
  ``.mat`` can be exchanged between the MATLAB and the Python tools.
"""
from .common import Struct, quat2rotm, rotm2quat, eul_zyx

__all__ = ["Struct", "quat2rotm", "rotm2quat", "eul_zyx"]
__version__ = "1.0.0"
