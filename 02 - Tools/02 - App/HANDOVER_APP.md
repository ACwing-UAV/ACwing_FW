# ACwing Pilot: app handover for firmware 0.5.0

**For:** the agent/developer who owns the ACwing Pilot app (`02 - Tools/02 - App/`).
**From:** the flight-controller firmware (`00 - FC_FW/`), version **0.5.0**.
**Protocol version:** still 2. Nothing in the BLE protocol changed incompatibly.

[`HANDOVER_FW.md`](HANDOVER_FW.md) is the contract the app gave the firmware.
This document goes the other way. It lists what the firmware now does beyond
that contract, and what the app should build to use it:

- battery monitor and failsafes;
- the flight log in the drone's flash;
- log download over USB.

The small items are already done in the app (§1). This file covers the larger
ones.

> **Executable reference.** Where this text and the code disagree, the code
> wins and this file must be fixed:
>
> | File | What it is |
> |---|---|
> | `00 - FC_FW/App/log/log_format.h` | on-flash log format 1: sector header, record types and layouts |
> | `00 - FC_FW/App/app/log_task.c` | USB log commands: answers, refusals, ACK after the data |
> | `00 - FC_FW/App/telemetry/proto_v2.h` | `PV2_FT_LOG_ENTRY`, `PV2_FT_LOG_DATA`, `PV2_FT_CMD`, `PV2_OP_LOG_*`, `pv2_log_entry_t` |
> | `02 - Tools/01 - Python/acwing_tools/flightlog.py` | working client (`FcLink`) and decoder (`decode_log`): port these |
> | `02 - Tools/01 - Python/tests/test_flightlog.py` | `FakeFC`: an emulated drone that answers the USB log commands |
> | `02 - Tools/01 - Python/tests/data/flightlog_sample.bin` | a log written by the firmware's own code: the test vector |
> | `03 - Docs/firmware/battery.md`, `03 - Docs/firmware/logging.md` | behaviour, explained |

---

## Contents

1. [Already done in the app](#1-already-done-in-the-app)
2. [Battery: what the firmware does now](#2-battery-what-the-firmware-does-now)
3. [Battery: work left in the app](#3-battery-work-left-in-the-app)
4. [USB transport](#4-usb-transport)
5. [USB log protocol](#5-usb-log-protocol)
6. [Log format 1 and the decoder](#6-log-format-1-and-the-decoder)
7. [Flight logs screen](#7-flight-logs-screen)
8. [Simulator](#8-simulator)
9. [Acceptance tests](#9-acceptance-tests)
10. [Open points and firmware work they need](#10-open-points-and-firmware-work-they-need)

---

## 1. Already done in the app

These changes come with this handover (`npm test`: 22 tests, typecheck and
build clean):

| Area | Change |
|---|---|
| `protocol/ids.ts` | params **23 battCapacity** (mAh, 0–20000, needs CURRENT), **24 battCells** (0 = auto, 1–6), **70 logMode** (0 off, 1 each flight, 2 always; needs FLASH_LOG); EVENT **8 TAKEOFF_ABORTED**, **33 BATTERY_MONITOR_LOST**; `ACTION` names for the two enum params |
| Settings › Safety › Battery | capacity (`ParamNumber`, integers) and cells (`ParamSelect`: Auto, 1S…6S) |
| Settings › Drone & firmware › Logs | logMode selector and "flash log used %" (SYSTEM.log), shown only with CAPS FLASH_LOG |
| `ParamSelect` / `ParamNumber` | options without a name show value + unit ("3S"); `ParamNumber` hides parameters the drone lacks and shows integers for integer steps |
| Pre-flight checks | battery item with value 0: "No battery (USB power)" (WARN) or "Battery monitor stopped" (FAIL) |
| BATTERY wizard | the scaleI preview is `scaleI × charged / counted`, as the firmware computes it (see §2) |
| i18n | every new string in en, it, fr, de, es |
| Simulator | battery as the firmware: one event per level crossed (30/31/32), level latched while armed, CRITICAL → RTH/LAND once with EVENT 22, EMERGENCY → LAND locked (SET_MODE, HOLD, RTH, TAKEOFF refused with BATTERY_LOW), battery landing not replaced by the app-link failsafe, ARM refused at CRITICAL, scaleI as the firmware |
| Tests | `drone.test.ts`: the new params, the EMERGENCY lock, the battery check values |

## 2. Battery: what the firmware does now

On boards that measure the battery (the ACwing FC; not the Nucleo bench by
default), with CAPS bit 8 CURRENT when a current sensor is wired:

- **POWER** at 5 Hz (STREAM_RATES 1–10) and on `REQUEST(POWER)`. Elsewhere
  POWER is never sent and `REQUEST(POWER)` is `UNSUPPORTED`: the app must keep
  working without POWER.
  - `cellAvg` is the **sagged** cell voltage the failsafes use.
  - `warn` is **latched**: it goes up after 2 s below a threshold and comes
    down only when disarmed. Show `warn`, not a level recomputed from
    `cellAvg`.
  - `pct` is a voltage estimate corrected by coulomb counting once
    battCapacity is set.
  - `remain` is `0xFFFF` (n/a) unless armed, with a current sensor and a
    capacity.
- **CAPS.cells** comes from the detection (or battCells); it is 0 until known.
- **Events:**
  - 30 WARN, 31 ERROR, 32 CRITICAL, one per level crossed, argument = cell mV;
  - 22 ERROR after 31, argument = the action (`2` LAND, `3` RTH);
  - **33 ERROR** (new) when the firmware's battery task stops: the battery is
    unknown and ARM is refused.
- **Failsafes:**
  - CRITICAL in the air → LAND (RTH only with GNSS and home), FLIGHT.failsafe
    bit 2. The pilot can take over with SET_MODE; it does not trigger again.
  - **EMERGENCY → LAND locked until disarm.** SET_MODE, HOLD, RTH and TAKEOFF
    are `REJECTED / BATTERY_LOW`; only KILL works.
  - While a battery failsafe lands the drone, an app-link loss only sets
    FLIGHT.failsafe bit 0; FLIGHT.auto stays LAND.
- **ARM / TAKEOFF** are refused with `BATTERY_LOW` at CRITICAL or worse, or when
  the battery task is silent.
- **CHECKS item 1:** N/A without a monitor; WARN with value 0 when no battery
  is connected (USB power, motors unpowered); FAIL with value 0 when the
  monitor stopped; otherwise cell mV with FAIL < battCrit (or latched
  CRITICAL), WARN < battWarn (or latched LOW).
- **BATTERY calibration** as §10.6, with one difference. The firmware stores the
  mAh of the last flight together with the scaleI they were counted with, and
  computes `scaleI = scaleI_flight × mAh_charged / mAh_counted`. The handover
  formula `mAh / usedMah` is the same only when scaleI was 1. CALIB data
  `usedMah` is that last-flight count (0 if no flight above 20 mAh since the
  calibration data was saved). The firmware refuses:
  - a voltage that differs from the reading by more than ±30 %
    (`INVALID / OUT_OF_RANGE`);
  - a current ratio outside 0.5–2 (`INVALID / OUT_OF_RANGE`);
  - `CAL_SET_REF(6,1,…)` without a recorded flight (`REJECTED / OUT_OF_RANGE`).

## 3. Battery: work left in the app

| # | What | Notes |
|---|---|---|
| B1 | **Battery-failsafe banner** in the cockpit while FLIGHT.failsafe bit 2 is set: "Landing: battery critical", with "take control" (SET_MODE) when `warn` = CRITICAL | the toast of EVENT 22 is not enough: it disappears |
| B2 | **EMERGENCY lock state**: when `warn` = EMERGENCY and armed, grey out mode buttons, HOLD, RTH and TAKEOFF and say why; keep KILL | today these commands fail with the generic "battery too low" toast |
| B3 | **Remaining time n/a**: when `remain` is n/a while armed and the drone has CURRENT, hint "Set the battery capacity" with a link to Settings › Safety | most users will not find parameter 23 by themselves |
| B4 | **EVENT 33**: treat as a pre-flight blocker (red in the Connect screen) and a critical alert in flight | the alert is already spoken (level ERROR) |
| B5 | **Cells**: CAPS.cells may be 0 at connection and become known later (POWER.cells); the Safety card already prefers POWER.cells. Check the Cockpit and the Telemetry screen do the same | |
| B6 | **BATTERY wizard copy**: step 2 should say "after charging the pack you flew last" and show 0 mAh as "no flight recorded yet" | the firmware refuses SET_REF(6,1) in that case |

## 4. USB transport

The FC's USB-C port is a **USB CDC virtual COM port**.

- **USB IDs:** VID/PID 0x0483/0x5740 (ST's development pair; a product ID will
  come later). The descriptors read manufacturer "ACwing", product "ACwing FC",
  and a serial number derived from the MCU's unique ID.
- **Baud rate:** ignored. Open the port at any rate (115200 is fine).
- **Bench:** on the Nucleo bench this is the **USB_USER** connector.

**What it carries today: only the log commands (§5).** Every other opcode is
answered `UNSUPPORTED`. USB is therefore not a flight link yet: no telemetry,
no RC, no app-link supervision (see §10).

| Platform | How | Status |
|---|---|---|
| Desktop PWA (Chrome, Edge) | **Web Serial** (`navigator.serial.requestPort({ filters: [{ usbVendorId: 0x0483, usbProductId: 0x5740 }] })`) | to build |
| Android (Capacitor) | a USB-serial plugin (CDC-ACM, e.g. one built on usb-serial-for-android); USB host permission dialog | to build; Web Serial on Android Chrome may also work, but check it on the devices you target |
| iOS / iPadOS | **not possible**: apps cannot open a USB CDC device without MFi | show "Use a computer, or download over Bluetooth" (see §10) |
| Simulator | an emulated USB drone (§8) | to build |

Implementation notes:

- Add `"usb"` to `Transport.kind` and implement `open/write/close` over the
  serial port: a reader loop feeding the existing `StreamParser`, writes of
  whole frames. `rssi()` and `mtu()` return null.
- Frames are the same `A5 5A | TYPE | LEN | PAYLOAD | CRC16` as over BLE, in a
  byte stream: reuse `frame.ts` (`encodeFrame`, `StreamParser`) as they are.
- **Commands over USB are wrapped**: PC → drone frames have TYPE `0x20` (CMD)
  and payload `op, seq, args`, the same bytes the BLE CMD characteristic gets.
  Add `encodeCmdFrame(cmd: Uint8Array) = encodeFrame(0x20, cmd)` and use it
  only on the USB transport.
- The USB link is **separate** from the BLE session: a user may be connected
  over BLE (flying, or just looking) and plug USB in to download logs. Do not
  route USB through the `drone` singleton's flight state; give the log screen
  its own small client (§5).

## 5. USB log protocol

| Command | Op | Arguments (LE) | Answer, in order |
|---|---|---|---|
| LOG_LIST | `0x50` | – | one **LOG_ENTRY** (`0x10`) per log, then **ACK** |
| LOG_READ | `0x51` | `id u16, offset u32, length u32` (length 0 = to the end) | **LOG_DATA** (`0x11`) frames, then **ACK** |
| LOG_ERASE | `0x52` | `"WIPE"` (`57 49 50 45`) | ACK `IN_PROGRESS` (5), then ACK `OK` when the erase ends |

**LOG_ENTRY (15 B):**

| off | type | field |
|---|---|---|
| 0 | u16 | id |
| 2 | u8 | index in the list |
| 3 | u8 | count |
| 4 | u8 | flags: bit 0 truncated (its beginning was overwritten), bit 1 being written |
| 5 | u16 | sectors |
| 7 | u32 | start time, ms after the drone's power-on |
| 11 | u32 | bytes (= sectors × 4096) |

**LOG_DATA (6 + n B):** `id u16, offset u32, data[n]`, with n ≤ 240.

Rules:

- **The ACK comes after the data**, so the ACK means "complete". Wait for
  the ACK with the command's op and seq; collect everything before it.
- Refusals, ACK `result / reason`:

  | Result / reason | Meaning |
  |---|---|
  | `REJECTED / ARMED` (1/8) | READ or ERASE while armed: "Disarm to download" |
  | `REJECTED / SENSOR_MISSING` (1/16) | no log flash on this board |
  | `INVALID / OUT_OF_RANGE` (3/14) | unknown id or offset past the end |
  | `BUSY` (2) | a READ is already running |
  | `INVALID` (3) | ERASE without the magic |
  | `UNSUPPORTED` (4) | any other opcode |

- **Read in chunks**, for example 256 KB per LOG_READ as the Python client
  does. Check that the offsets are contiguous and the byte count matches. On a
  gap or a timeout, re-read that chunk only. USB FS gives roughly 0.3–1 MB/s:
  a full 16 MB in less than a minute.
- **Timeouts:**
  - 2 s without any frame while a command is pending → retry with a new seq;
  - ERASE can take minutes once IN_PROGRESS has arrived, so extend the wait
    (the Python client allows 300 s).
- **The drone pauses its recording** while the PC/app talks to it and for 5 s
  after the last command. The log being written is closed at the first
  command, so it can be downloaded at once.
- **No LOG_* over BLE yet.** The firmware answers them only on USB (§10).
- **Reference client:** `FcLink` in `acwing_tools/flightlog.py`: `_run()`,
  `list_logs()`, `read_log()`, `erase()`. About 100 lines to port.

## 6. Log format 1 and the decoder

Defined in `log_format.h`; the Python `decode_log()` and MATLAB
`acwingLogDecode.m` implement it and agree on the test vector.

- **Layout:**
  - a downloaded log is `sectors × 4096` bytes;
  - each sector starts with a 20-byte header: magic `0x474C4341` "ACLG",
    serial u32, log id u16, sequence u16, start time u32, format u8 (= 1),
    reserved u8, CRC-16/CCITT-FALSE of bytes 0–17;
  - then records `[type][len][payload][crc8]` (CRC-8/SMBUS, poly 0x07, init
    0, over type, len and payload) until the end of the sector or the first
    `0xFF`.
- **Robustness:**
  - a bad header → skip the sector;
  - a bad record CRC → skip the rest of the sector (a power loss leaves one
    cut record);
  - an unknown type with a valid CRC → skip it and count it: future records
    must not break old apps;
  - a known type whose length differs from the struct → treat as unknown.
- **Records** (little-endian, packed):

  | Type | Name | Bytes | Fields → units |
  |---|---|---|---|
  | `0x01` | INFO | 27 | fw u8×3, hwRev char, format u8, t0 ms u32, sensors u16, name char[16] |
  | `0x10` | ATT | 24 | t ms u32, q i16×4 (/32767, [w x y z] body → NWU), gyro i16×3 (mrad/s), acc i16×3 (mg) |
  | `0x11` | VERT | 25 | t, h mm i32, vz mm/s i16, h_sp mm i32, vz_sp mm/s i16, thrust u8 (/255), tof mm u16 (0xFFFF n/a), flowQ u8 (0xFF n/a), baro Pa u32 (0 n/a), flags u8 (bit 0 alt valid, 1 ToF valid, 2 ToF ground, 3 flow usable, 4 alt loop) |
  | `0x12` | FLIGHT | 15 | t, armed, mode, auto, failsafe u8, flags u8 (0 in air, 1 landed, 2 RC stale, 3 geofence), sticks i8×4 (/100: thr, yaw, pitch, roll), silence ms u16 |
  | `0x13` | EVENT | 11 | t, level u8, code u16, arg i32 (same codes as the EVENT frame) |
  | `0x14` | EST | 20 | t, gyro bias i16×3 (0.1 mrad/s), att unc cdeg u16, vib mg u16, loop Hz u16, jitter µs u16, cpu % u8, mag interf % u8 |
  | `0x20` | POWER | 19 | t, vbat mV u16, current cA i16, used mAh u16, cell mV u16, cell at rest mV u16, pct u8, level u8, R cell mΩ u16, cells u8 (bit 7 = present) |
  | `0x7F` | DROP | 7 | t, lost u16, source u8 (0 ImuTask, 1 BattTask) |

  Rates: ATT and VERT 50 Hz, FLIGHT, EST and POWER 10 Hz.
- **Time** is ms after the drone's power-on. The drone has no clock, so the
  app should stamp each download with the phone's date and show times relative
  to the log start (`INFO.t0`).
- **Test vector** `flightlog_sample.bin` (24 576 B) must decode to:
  - 6 sectors, 0 bad, 0 unknown;
  - INFO fw `0.5.0`, hwRev `N`;
  - 350 ATT and 350 VERT at 20 ms steps;
  - 70 FLIGHT and 70 POWER (vbat 3.9 V, 1S, present);
  - 2 EVENT: ARMED then DISARMED.

  Copy it into the app's tests.

## 7. Flight logs screen

Settings › Drone & firmware › Logs already has the "Flight logs on the drone"
and "Download" strings (`about.logsOnDrone`, `about.download`), unused so far.
Suggested screen:

1. **Connect over USB.** A button, or detect an already granted port. Show
   the platform limits of §4 (iOS).
2. **List.** One row per log: start time (relative to power-on, plus the
   download date once stored), duration estimate (`bytes / 3.4 kB/s`, exact
   after decoding), size. Badges: "truncated", "recording". Actions: download,
   download all.
3. **Download.** Progress per log and in total, cancel, retry of the failed
   chunk. Store the raw `.bin`: IndexedDB on the PWA, the Filesystem plugin on
   Capacitor. The raw file is what the PC tools read.
4. **Viewer** for a stored log:
   - summary: duration, min cell, mAh, max current, events;
   - charts on the existing chart component: height with set point and ToF;
     battery cell (sagged and at rest) and current; attitude (roll/pitch/yaw
     from q); vibration and loop rate;
   - an events timeline with the translated texts (`events.N`);
   - records lost (DROP) shown as a warning.
5. **Export / share.**
   - `.bin` (for `acwing-tools log decode` and MATLAB);
   - a CSV per record type, with the same columns as `save_csv()` in Python
     so files are interchangeable.
6. **Erase all**, with confirmation, only when disarmed, with "erasing…"
   until the final ACK.

Show the drone's **logMode** on this screen too (the selector already exists
in Settings). With logMode = 0 nothing is recorded.

## 8. Simulator

- **`SimUsbTransport`**: answers LOG_LIST, LOG_READ and LOG_ERASE from in-memory
  logs, the way `FakeFC` does in `tests/test_flightlog.py`:
  - entries before the ACK;
  - data in ≤ 240-byte frames;
  - `IN_PROGRESS` then `OK` for ERASE;
  - `ARMED` refusal when armed;
  - `UNSUPPORTED` for anything else.
- **A TypeScript log encoder** for the simulator, the inverse of the decoder:
  it lets the simulator produce a log of the simulated flight. Test it by
  round trip and against `flightlog_sample.bin`.
- Optional: the 2 s voltage filter of the firmware's battery levels (the
  simulator reacts at once today).

## 9. Acceptance tests

With the simulator (unit tests) and then with a board (Nucleo: USB_USER port,
RAM log; ACwing FC: W25Q log):

1. **Decoder:** the test vector decodes as in §6. A flipped byte in a record
   skips only the rest of that sector; a flipped byte in a header skips only
   that sector.
2. **List and read:**
   - LOG_LIST of 2 logs → 2 entries, then the ACK;
   - a 256 KB chunked read of a log gives exactly the bytes of LOG_READ with
     length 0;
   - a cut USB cable during a read → error, and the next read resumes from the
     failed chunk.
3. **Refusals:** READ while armed → "Disarm to download"; unknown id →
   "OUT_OF_RANGE"; a board without log flash → the screen says so.
4. **Recording:**
   - logMode 1: arm, wait 10 s, disarm → a new log about 11 s long that
     starts with INFO and has ARMED and DISARMED events;
   - logMode 0 → no new log.
5. **Erase:** IN_PROGRESS then OK; the list is empty and SYSTEM.log is 0 %.
6. **Battery (simulator now, board later):**
   - crossing battWarn, battCrit and battEmerg gives one toast/voice each;
   - the failsafe banner appears at CRITICAL;
   - at EMERGENCY the mode/HOLD/RTH/TAKEOFF controls are disabled with the
     reason, and KILL works;
   - with capacity 0 and a current sensor, the "set capacity" hint appears.
7. **Params:**
   - 23 and 24 appear under Safety › Battery, and 70 under Drone & firmware;
   - 23 and 70 are hidden on a drone without CURRENT or FLASH_LOG;
   - battCells 7 → "value out of range".

## 10. Open points and firmware work they need

| Point | Needs in the firmware | Comment |
|---|---|---|
| **Log download over BLE** (the only way on iOS) | route LOG_LIST/READ/ERASE received on the BLE CMD characteristic to the LogTask, and send LOG_ENTRY/LOG_DATA on the BLE HI ring with pacing | about 10 kB/s: minutes per flight, about 30 min for a full flash; worth it for "last flight only". Decide before building §7 for iOS |
| **USB as a flight link** (telemetry, RC, app link) | CMD frames from USB into `app_link`, frames to USB as well as BLE | needed on the ACwing FC for the PC tools anyway (no ST-LINK virtual COM port there) |
| Wall-clock time in the log | an RTC set by the app at connection (a new command) | today times are relative to power-on |
| `HANDOVER_FW.md` | add params 23, 24, 70, events 8 and 33, the USB log commands and the scaleI note to the contract | so the two documents agree |
| Firmware update over USB (DFU) | the STM32 ROM bootloader already does DFU on this port (BOOT0) | the "Update firmware" button could use WebUSB DFU later |
