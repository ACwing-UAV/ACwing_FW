# 0008. Flight log as a sector ring, downloaded over USB CDC

- **Status:** Accepted
- **Date:** 2026-10-31

## Context

The ACwing FC rev A has a 16 MB W25Q128JV NOR flash for logs (SPI1, shared
with the IMU) and a USB FS port. Logs must survive power loss and a full
flash, must not delay the flight loop, and must reach the PC tools (MATLAB
and Python) without special drivers.

## Decision

- **Format**: a ring of 4 KB sectors; each sector has a 20-byte header (magic,
  global serial, log id, sequence, start time, CRC-16) and holds whole
  records `[type][len][payload][crc8]`. A log is a run of sectors; when the
  flash is full the oldest sectors are reused.
- **No file system.** Mounting reads one header per sector and rebuilds the
  list.
- **Producers** (ImuTask, BattTask) put records in their own SPSC byte ring;
  the **LogTask** is the only owner of the flash.
- **USB CDC** (virtual COM port) carrying the same `A5 5A` frames as the rest
  of the protocol: CMD frames from the PC, LOG_LIST / LOG_READ / LOG_ERASE
  (opcodes 0x50/0x51 reserved in the handover, 0x52 added), ACK after the
  data.

## Consequences

- Wear is spread over the whole chip; a power loss costs at most one page and
  leaves every log readable.
- The format is defined once (`log_format.h`) and decoded by Python and
  MATLAB; a host test writes a log with the firmware's code and the PC tests
  decode it.
- CDC needs no driver on Windows 10+, macOS or Linux, and the same framing
  could later carry the app protocol or the PC telemetry over USB.
- Downloading 16 MB takes about a minute at the CDC rate; recording pauses
  while the PC talks to the drone.

## Alternatives considered

- **USB mass storage (MSC) with FAT.** Logs would appear as files, but FAT on
  a NOR flash needs a translation layer for wear and power loss, the host can
  write to the volume while the firmware logs, and MSC blocks the flash for
  the whole time the drive is mounted.
- **littlefs.** Robust and wear-levelled, but a general file system is more
  code and RAM than an append-only log needs, and the PC still needs a
  protocol to fetch files.
- **The whole memory over BLE.** At BLE rates 16 MB take about half an
  hour. Instead (2026-10-31) BLE serves **the last flight only**, with the
  same commands, on every platform; USB serves everything, in the web app,
  on Android and on the PC.
