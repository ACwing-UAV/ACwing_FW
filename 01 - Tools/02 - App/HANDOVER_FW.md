# ACwing Pilot ↔ flight controller: firmware handover

**For:** the agent/developer who owns the flight-controller firmware (`00 - STM32_FW/`).
**From:** the ACwing Pilot app (`01 - Tools/02 - App/`).
**Protocol version:** 2 (`PROTO_VERSION = 2`, sent in CAPS).

This document is the complete contract between the drone and the app: what the
firmware must send, what it receives, the rules behind every command, and which
data feeds which screen of the app. Everything here is already implemented on
the app side and exercised against a simulator.

> **Executable reference.** Where this text and the code disagree, the code
> wins and this file must be fixed:
>
> | File | What it is |
> |---|---|
> | `src/protocol/ids.ts` | every number: frame types, opcodes, enums, bit masks, parameter table |
> | `src/protocol/messages.ts` | byte layout of every frame (`encodeX` = what the firmware must write) |
> | `src/protocol/commands.ts` | byte layout of every command (`Cmd.x` = what the firmware receives) |
> | `src/protocol/frame.ts` | framing, CRC16, stream parser |
> | `src/sim/simDrone.ts` | **behavioural reference**: a drone that answers every command the way the firmware should (arming rules, failsafes, calibration state machines, CHECKS) |
> | `src/link/drone.ts` | how the app sends, retries and times out |
>
> `npm test` runs the protocol and simulator tests (`src/**/*.test.ts`).

---

## Contents

1. [Scope and principles](#1-scope-and-principles)
2. [Transport (BLE)](#2-transport-ble)
3. [Framing and frame catalogue](#3-framing-and-frame-catalogue)
4. [Frame layouts, drone → app](#4-frame-layouts-drone--app)
5. [Commands, app → drone](#5-commands-app--drone)
6. [Link supervision, input arbitration, failsafes](#6-link-supervision-input-arbitration-failsafes)
7. [Arming and automatic flight](#7-arming-and-automatic-flight)
8. [Parameters](#8-parameters)
9. [Pre-flight checks (CHECKS)](#9-pre-flight-checks-checks)
10. [Calibrations](#10-calibrations)
11. [Events](#11-events)
12. [Data per app section](#12-data-per-app-section)
13. [Indoor and outdoor models](#13-indoor-and-outdoor-models)
14. [Bandwidth budget](#14-bandwidth-budget)
15. [Suggested firmware structure and phases](#15-suggested-firmware-structure-and-phases)
16. [Acceptance tests](#16-acceptance-tests)
17. [Out of scope and open points](#17-out-of-scope-and-open-points)

---

## 1. Scope and principles

- **One link, no server.** The app talks to the drone directly over Bluetooth
  LE, on PC, Mac, tablets, Android and iOS. There is no backend.
- **Backward compatible.** The phase-2 GATT service, the frame format and the
  frames `0x01 STATE`, `0x02 INIT`, `0x03 STATUS`, `0x04 REPLAY` and the
  legacy commands `0x01`/`0x02` stay exactly as they are. Protocol v2 only
  **adds** frame types `0x05…0x0F` and command opcodes `≥ 0x10`. The PC tools
  (MATLAB/Python) keep working and simply ignore the new frame types.
- **The drone is the authority.** The app never assumes a command worked: it
  waits for the ACK and then for the telemetry that shows the new state. Every
  safety rule (arming, failsafe, geofence, limits) is enforced by the firmware;
  the app only mirrors them in the UI.
- **The drone describes itself.** The app adapts to what CAPS says
  (indoor/outdoor, sensors fitted, motors, cells). Nothing in the app is
  hard-wired to a model.
- **Settings live on the drone.** Parameters and calibrations are stored in the
  drone's flash; the app only stores its own preferences (language, units,
  stick feel, voice).
- **Coordinates and units.** World NWU, body FLU, quaternion body→world
  `[w x y z]` (unchanged). On the wire: little-endian, packed, integers in
  fixed-point units given per field (cm, cdeg, mV…). `0xFF` / `0xFFFF` /
  `-128` mark "not available" where stated.

## 2. Transport (BLE)

Unchanged from phase 2 (`App/platform/ble_tlm_svc.c`, `App/telemetry/tlm_ble.c`):

| Item | Value |
|---|---|
| Service | `7E4A0001-2C6B-4F8E-9A5D-3B1C0E1D2F30` |
| TLM characteristic | `7E4A0002-…`, **notify**; continuous byte stream of frames, split at any MTU |
| CMD characteristic | `7E4A0003-…`, **write and write-without-response**, **≤ 20 bytes** |
| Advertised name | must start with **`ACw`** (the app's device picker filters on `ACw`, `IEKF`, `STM32WB`). Keep `ACwFC` or use `ACw-<short id>` |
| Connection params | keep the phase-2 request (15–30 ms, 4 s supervision) |
| MTU | the app works with any MTU (≥ 23). Frames may span notifications |

Required changes on this layer:

1. **CMD handling must leave the BLE callback.** Today
   `tlm_ble_on_command()` handles the two legacy commands inline. Keep that
   for `0x01`/`0x02`, and for every other opcode copy the bytes (≤ 20) into a
   small queue (≥ 8 entries) consumed by the control/application task.
   RC at 25 Hz plus bursts of commands must never block the BLE stack.
2. **On notification enable (CCCD)** send, in this order: `CAPS`, `INIT`
   (as today), then `CHECKS`, then start the periodic streams.
3. **TX priority** when the TX pool is short: `ACK` > `EVENT` > `CALIB` >
   `PARAM` > `FLIGHT` > `CHECKS` > `POWER` > `NAV` > `MOTORS` > `SYSTEM` >
   `STATUS` > `STATE`. Dropping STATE is fine; dropping an ACK costs a 600 ms
   retry; dropping an EVENT may hide a warning.
4. **USB/UART** may carry the same frames (the PC tools ignore unknown
   types), but the app does not use USB yet.

## 3. Framing and frame catalogue

Unchanged frame: `A5 5A | TYPE u8 | LEN u8 | PAYLOAD[LEN] | CRC16 (LSB, MSB)`,
CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, no xor-out) over
`TYPE, LEN, PAYLOAD`. Reference: `frame.ts → crc16()`, `encodeFrame()`.

| Type | Name | Payload | Rate / trigger | Status |
|---|---|---|---|---|
| `0x01` | STATE | 71 B | 240 Hz / decimation (`[0x02,N]`) | existing |
| `0x02` | INIT | 137 B | on subscribe, on `[0x01]` | existing |
| `0x03` | STATUS | 62 B | 10 Hz | existing |
| `0x04` | REPLAY | – | USB only | existing, ignored by the app |
| `0x05` | **CAPS** | 39 B | on subscribe, on REQUEST, every 5 s | new |
| `0x06` | **FLIGHT** | 25 B | 10 Hz (STREAM_RATES 1–20) | new |
| `0x07` | **POWER** | 13 B | 5 Hz (1–10) | new |
| `0x08` | **NAV** | 31 B | 5 Hz (1–10) | new |
| `0x09` | **MOTORS** | 1 + 4·n B | 5 Hz (1–10) | new |
| `0x0A` | **SYSTEM** | 23 B | 2 Hz (1–5) | new |
| `0x0B` | **EVENT** | 8 + text (≤ 40) B | on event; level ≥ ERROR repeated ×3, 100 ms apart | new |
| `0x0C` | **PARAM** | 11 B | answer to PARAM_GET/SET/LIST/SAVE/DEFAULTS | new |
| `0x0D` | **CALIB** | 6 + data B | 10 Hz while a calibration is active + on every state change | new |
| `0x0E` | **ACK** | 4 B | answer to every command with a sequence number | new |
| `0x0F` | **CHECKS** | 2 + 4·n B | 2 Hz disarmed, 0.5 Hz armed, on REQUEST | new |

The app reads STATE for the attitude (HUD, telemetry) and STATUS for the BLE
diagnostics (`bleDrop`, `bleMtu`); it ignores INIT content.

## 4. Frame layouts, drone → app

All little-endian, packed. `off` = byte offset in the payload.

### 0x05 CAPS (39 B)

| off | type | field | notes |
|---|---|---|---|
| 0 | u8 | proto | `2` |
| 1 | u8×3 | fw major, minor, patch | shown in Settings › Drone & firmware |
| 4 | u8 | hwRev | one ASCII letter, e.g. `'H'` |
| 5 | u8 | model | `0` INDOOR, `1` OUTDOOR |
| 6 | u16 | sensors | bit mask, see below |
| 8 | u8 | nMotors | 4, 6 or 8 |
| 9 | u8 | frame | `0` QUAD_X, `1` HEXA_X, `2` OCTO_X |
| 10 | u8 | cells | detected series cells, `0` = not known yet |
| 11 | char[16] | name | UTF-8, zero padded; user name (SET_NAME), default `ACwing #1` |
| 27 | char[12] | serial | ASCII, zero padded |

`sensors` bits: `0` IMU, `1` BARO, `2` MAG_FC, `3` MAG_GPS, `4` GNSS, `5` FLOW,
`6` TOF, `7` ESC_TLM (RPM/temperature), `8` CURRENT, `9` ELRS, `10` LEDS,
`11` FLASH_LOG. **Set a bit only if the part is detected at boot**, not if it
is merely supported: the app hides every feature whose sensor bit is 0.

### 0x06 FLIGHT (25 B)

| off | type | field | unit / notes |
|---|---|---|---|
| 0 | u8 | arm | `0` disarmed, `1` armed |
| 1 | u8 | mode | `0` ACRO, `1` ANGLE, `2` ALT_HOLD, `3` POS_HOLD |
| 2 | u8 | auto | `0` none, `1` TAKEOFF, `2` LAND, `3` RTH, `4` HOLD (pause), `5` FAILSAFE |
| 3 | u8 | failsafe | bits: `0` APP_LINK, `1` RADIO, `2` BATTERY, `3` GEOFENCE, `4` GPS, `5` FLOW |
| 4 | u16 | flags | bits: `0` IN_AIR, `1` LANDED, `2` HOME_SET, `3` GEOFENCE_ON, `4` BEGINNER, `5` ARM_BLOCKED (checks not ready and `armRequireChecks`=1) |
| 6 | u8 | input | who commands the sticks: `0` none, `1` APP, `2` ELRS |
| 7 | u16 | flightTime | s since takeoff (kept after landing until the next arm) |
| 9 | i32 | altRel | cm above takeoff point, + up |
| 13 | i16 | vz | cm/s, + up |
| 15 | u16 | groundSpeed | cm/s |
| 17 | u16 | heading | cdeg 0…35999, 0 = north, clockwise (compass heading) |
| 19 | u16 | homeDist | m, `0xFFFF` = n/a (no GNSS or no home) |
| 21 | u16 | homeBearing | cdeg, direction **from the drone to home**, clockwise from north |
| 23 | u16 | rthEta | s, `0xFFFF` = n/a |

### 0x07 POWER (13 B)

| off | type | field | unit / notes |
|---|---|---|---|
| 0 | u16 | vbat | mV, whole pack |
| 2 | i16 | current | cA (0.01 A); 0 if no current sensor |
| 4 | u16 | used | mAh since power-on |
| 6 | u8 | cells | detected cells |
| 7 | u16 | cellAvg | mV |
| 9 | u8 | pct | 0…100, firmware estimate (voltage under load + used mAh) |
| 10 | u16 | remain | s of flight left, `0xFFFF` = n/a |
| 12 | u8 | warn | `0` OK, `1` LOW (< battWarn), `2` CRITICAL (< battCrit), `3` EMERGENCY (< battEmerg) |

### 0x08 NAV (31 B)

Fields of absent sensors carry their n/a value; **send NAV also on indoor
models** (ToF, flow, baro).

| off | type | field | unit / notes |
|---|---|---|---|
| 0 | u8 | fix | `0` none, `2` 2D, `3` 3D, **`0xFF` = no GNSS hardware** |
| 1 | u8 | sats | |
| 2 | u16 | hdop | ×100 |
| 4 | i32 | lat | 1e-7 deg |
| 8 | i32 | lon | 1e-7 deg |
| 12 | i32 | altMsl | cm |
| 16 | i16×3 | vel N, E, D | cm/s |
| 22 | u16 | tof | mm, `0xFFFF` = n/a or out of range |
| 24 | u8 | flowQ | 0…100, `0xFF` = n/a |
| 25 | u32 | baro | Pa |
| 29 | u16 | wind | cm/s estimate, `0xFFFF` = n/a |

### 0x09 MOTORS (1 + 4·n B)

`n u8`, then for each motor (M1…Mn, Betaflight numbering, see §10.7):
`rpm u16` (`0xFFFF` = n/a, e.g. brushed motors), `temp i8` °C (`-128` = n/a),
`out u8` commanded output 0…100 %.

### 0x0A SYSTEM (23 B)

| off | type | field | unit / notes |
|---|---|---|---|
| 0 | u8 | lq | ELRS link quality %, `0xFF` = no receiver or not linked |
| 1 | i8 | rssi | dBm (ELRS) |
| 2 | i8 | snr | dB (ELRS) |
| 3 | u16 | pktRate | Hz (ELRS) |
| 5 | u16 | txPower | mW (ELRS telemetry) |
| 7 | u8 | cpu | % |
| 8 | u16 | loopHz | measured control loop rate |
| 10 | u16 | jitter | µs |
| 12 | u8 | log | flash log used % |
| 13 | i16 | imuTemp | c°C |
| 15 | u16 | vibration | mg RMS |
| 17 | u16 | accClip | count since arm |
| 19 | u16 | attUnc | cdeg, 1σ attitude uncertainty from the estimator |
| 21 | u8 | estFlags | bit0 EKF ok, bit1 acc update gated, bit2 mag update gated, bit3 magnetic interference |
| 22 | u8 | magInterf | % |

### 0x0B EVENT (8 + len B)

`level u8` (`0` INFO, `1` WARN, `2` ERROR, `3` CRITICAL), `code u16` (§11),
`arg i32` (code-specific), `len u8`, `text[len]` (ASCII, ≤ 40, English
fallback shown only for codes the app does not know). Events of level ≥ ERROR
are sent 3 times 100 ms apart; the app de-duplicates (same code + arg within
600 ms).

### 0x0C PARAM (11 B)

`id u16`, `value f32`, `flags u8` (bit0 = differs from flash, bit1 =
read-only), `index u16` (position in the table), `count u16` (table size).

### 0x0D CALIB (6 + data B)

Header: `type u8` (§10), `state u8` (`0` IDLE, `1` RUNNING, `2` WAITING_USER,
`3` DONE, `4` FAILED, `5` SAVED), `step u8` (1-based, see each type),
`progress u8` 0…100, `quality u8` 0…100 (`0xFF` n/a), `reason u8` (Reason, when
FAILED). Data per type:

| type | data (off from 6) | size |
|---|---|---|
| 1 ACCEL | `doneMask u8` (bit = position), `current u8` (0…5), `stillPct u8` (0…100 of the 2 s still window), `offset i16×3` mg, `scale u16×3` ×1e4, `fitErr u16` %×100 | 17 |
| 2 LEVEL | `roll i16` cdeg, `pitch i16` cdeg (measured), `remaining u16` ms, `trimRoll i16` cdeg, `trimPitch i16` cdeg | 10 |
| 3 GYRO | `remaining u16` ms, `bias i16×3` m°/s, `noise u16` m°/s RMS, `temp i16` c°C | 12 |
| 4 COMPASS | `coverage u8×5` (36 bits, bit i = byte i>>3, bit i&7), `interfFc u8` %, `interfGps u8` % (`0xFF` n/a), `qualityFc u8`, `qualityGps u8` (`0xFF` until DONE / n/a) | 9 |
| 5 FLOW_TOF | `tof u16` mm (raw), `height u16` mm (corrected), `flowQ u8`, `match u8` % (gyro vs flow), `axesDone u8` (bit0 roll, bit1 pitch), `scaleX u16` ×1e4, `scaleY u16` ×1e4, `tofOffset i16` mm | 13 |
| 6 BATTERY | `vbat u16` mV (as measured with the current scale), `scaleV u16` ×10, `scaleI u16` ×1e4, `usedMah u16` (last flight) | 8 |
| 7 MOTORS | `motor u8` (spinning motor, 0 none), `spinning u8` 0/1, `revMask u8` (bit m-1 = reversed), `okMask u8` (bit m-1 = tested) | 4 |
| 8 STICKS | `centre u16×4`, `min u16×4`, `max u16×4` (µs, order throttle, yaw, pitch, roll), `auxPos u8×4` (AUX1…4: `0` unused, `2` two-position, `3` three-position) | 28 |

### 0x0E ACK (4 B)

`op u8`, `seq u8`, `result u8`, `reason u8`.

`result`: `0` OK, `1` REJECTED (valid but not allowed now, see `reason`),
`2` BUSY, `3` INVALID (malformed, bad magic, out of range), `4` UNSUPPORTED
(unknown opcode or feature not built), `5` IN_PROGRESS (accepted, still
running; optional).

`reason`: `0` NONE, `1` NOT_LANDED, `2` CHECKS_FAILING, `3` THROTTLE_NOT_LOW,
`4` NO_GPS, `5` BATTERY_LOW, `6` NO_HOME, `7` RADIO_OVERRIDE, `8` ARMED,
`9` PROPS_CONFIRM_MISSING, `10` NOT_ARMED, `11` CALIBRATING, `12`
NOT_CALIBRATING, `13` NOT_STILL, `14` OUT_OF_RANGE, `15` UNKNOWN_PARAM,
`16` SENSOR_MISSING. The app translates each reason into the user's language
("Not done: throttle is not at minimum").

### 0x0F CHECKS (2 + 4·n B)

`ready u8` (1 = arming allowed as far as checks go), `n u8`, then n ×
`id u8`, `state u8` (`0` OK, `1` WARN, `2` FAIL, `3` N/A), `value i16`.
Item meaning in §9.

## 5. Commands, app → drone

Format: `[op u8][seq u8][payload…]`, **≤ 20 bytes**. Legacy `0x01` (send
INIT) and `0x02 N` (STATE decimation) keep their phase-2 format without seq.

**Sequence and ACK rules**

- Every command with a seq is answered by exactly one ACK `{op, seq, result,
  reason}`, **except RC (0x11) and HEARTBEAT (0x10)** (seq = 0, no ACK).
- The app re-sends a command without ACK after **600 ms**, **twice at most**,
  with the **same seq**. The firmware must keep a cache of the last ≥ 16
  `(op, seq) → ACK` pairs and, on a repeat, **re-send the cached ACK without
  executing again** (a repeated ARM, TAKEOFF or MOTOR_TEST must not happen
  twice). Reference: `SimDrone.command()`.
- Send the ACK **after** validating, before or while starting the action. The
  result of a long action is visible in FLIGHT/CALIB, not in the ACK.
- Unknown opcode → `UNSUPPORTED`. Payload too short / bad magic → `INVALID`.

| op | name | payload after seq | ACK / rules |
|---|---|---|---|
| `0x10` | HEARTBEAT | – | no ACK; refreshes app-link timer (5 Hz when the cockpit is closed) |
| `0x11` | RC | `thr i16, yaw i16, pitch i16, roll i16` (−1000…1000), `buttons u8` | no ACK; 25 Hz while the cockpit is open; refreshes app-link timer. thr −1000 = stick bottom; yaw + = right; pitch + = forward; roll + = right |
| `0x12` | ARM | `A5 5A` | §7.1 |
| `0x13` | DISARM | – | only on the ground (`NOT_LANDED`) |
| `0x14` | TAKEOFF | `alt u16` cm (0 = param `takeoffAlt`) | §7.2 |
| `0x15` | LAND | – | `NOT_ARMED` if not armed or not in air |
| `0x16` | RTH | – | `NO_GPS` without GNSS, `NO_HOME`, `NOT_ARMED` |
| `0x17` | HOLD | – | in air: auto = HOLD (pause); on ground: no-op OK |
| `0x18` | KILL | `4B 49 4C 4C` ("KILL") | **always executed**, any state, in flight too; motors off at once, disarm, EVENT 70 CRITICAL |
| `0x19` | SET_MODE | `mode u8` | `INVALID` > 3; POS_HOLD without GNSS and FLOW → `SENSOR_MISSING`; ACRO with `beginner`=1 → `REJECTED/NONE` |
| `0x20` | PARAM_GET | `id u16` | answer PARAM frame; `INVALID/UNKNOWN_PARAM` |
| `0x21` | PARAM_SET | `id u16, value f32` | `INVALID/OUT_OF_RANGE` outside min…max; RAM only (flag dirty), answer PARAM |
| `0x22` | PARAM_SAVE | – | write all params to flash, EVENT 60, then PARAM for every id (dirty cleared). Allowed armed? **No** → `REJECTED/ARMED` recommended (flash write stalls) — the app only saves from Settings |
| `0x23` | PARAM_DEFAULTS | – | `REJECTED/ARMED` if armed; RAM ← defaults (dirty until SAVE), PARAM for every id |
| `0x24` | PARAM_LIST | – | PARAM for every id (index/count filled), paced to the TX pool |
| `0x30` | CAL_START | `type u8, option u8` | §10 |
| `0x31` | CAL_CAPTURE | – | §10 (step-specific "next / capture now / skip") |
| `0x32` | CAL_ABORT | – | discard results, state IDLE, previous values kept. `NOT_CALIBRATING` if none |
| `0x33` | CAL_SAVE | – | only in DONE (MOTORS: in any running state); write to flash, CALIB state SAVED once, EVENT 50 (arg = type), then IDLE |
| `0x34` | CAL_RESTORE | `type u8` | restore the previous saved set of that type (keep one backup per type); `REJECTED/ARMED` if armed |
| `0x35` | CAL_SET_REF | `type u8, index u8, value f32` | reference values typed by the user, §10.5 and §10.6 |
| `0x36` | MOTOR_TEST | `motor u8 (1…n), pct u8 (≤ 20), ms u16 (≤ 3000), 0x50` | `PROPS_CONFIRM_MISSING` without `0x50`; `ARMED`; `INVALID/OUT_OF_RANGE`. App sends pct 8, 2000 ms |
| `0x37` | MOTOR_REVERSE | `motor u8` | toggle direction (DShot spin-direction command / ESC setting), store with CAL_SAVE |
| `0x38` | MOTOR_REMAP | `map u8×n` (n ≤ 8; map[i] = output for motor i+1) | `ARMED`. Reserved: the app does not send it yet |
| `0x40` | STREAM_RATES | `flight, power, nav, motors, system` u8 Hz | clamp to the ranges in §3; 0 = keep current. Reserved: the app uses the defaults today |
| `0x41` | REQUEST | `frameType u8` | send that frame once now (CAPS, CHECKS, INIT, FLIGHT, POWER, NAV, MOTORS, SYSTEM); others `UNSUPPORTED` |
| `0x42` | SET_NAME | `name char[16]` (zero padded UTF-8) | store in flash, re-send CAPS |

What the app sends right after connecting: `REQUEST CAPS`, `REQUEST CHECKS`,
`PARAM_LIST`, `[0x02 N]` (STATE decimation from the user's telemetry rate:
N = 240 / {5, 10, 20, 50} Hz), then HEARTBEAT at 5 Hz (or RC at 25 Hz).

## 6. Link supervision, input arbitration, failsafes

**App link.** The app link is alive while RC or HEARTBEAT (or any command)
arrives. Silence measured on the drone:

| silence | in air | on ground |
|---|---|---|
| > 1 s | `failsafe |= APP_LINK`; action **fsAppShort** (`0` hover/HOLD, `1` land) | nothing (armed on ground: disarm after 3 s) |
| > 3 s | action **fsAppLong** (`0` hover, `1` land, `2` RTH; RTH without GNSS or home → land); EVENT 20 ERROR | – |
| link back (< 0.5 s) | clear APP_LINK; if the short action was HOLD and the long one never triggered, give control back. If the long action triggered, **keep it** until the pilot changes mode/cancels | – |

The app shows its own "link lost" banner after 1.5 s without any frame; the
drone acts on its own timers above.

> **Requirement: the app-link watchdog must not depend on the BLE stack.**
> The failsafe has to trigger even when the cause of the silence is on the
> drone (BLE stack, CPU2 or radio stalled, TX/RX queue blocked, task
> starved), not only when the phone goes away.
>
> - The BLE path only **records** the arrival: the CMD write callback (or the
>   task that drains the CMD queue) stores `t_last_app = now` from a
>   monotonic hardware tick (e.g. the 1 kHz timebase or a free-running
>   timer), together with the RC values. Nothing else happens in the BLE
>   context.
> - The **check** runs in the control/flight-manager loop (≥ 50 Hz, the loop
>   that also runs the attitude controller), which computes
>   `silence = now − t_last_app` and applies the 1 s / 3 s actions. It never
>   waits on BLE: no blocking calls, no mutex shared with the BLE thread
>   (a single 32-bit timestamp written atomically is enough; RC values via a
>   double buffer or sequence counter).
> - **No other source refreshes the timer**: BLE connection events, CCCD
>   writes, notifications sent, link-layer keep-alives do not count. Only
>   frames received on CMD (RC, HEARTBEAT or any command) do.
> - **Disconnection** (GAP disconnect event) is not required to start the
>   count: the timer runs regardless. A disconnect may only make the
>   failsafe start *earlier*, never later.
> - If the BLE stack reports a fatal error or CPU2 stops answering, treat it
>   as silence (do not reset the timer), and keep the hardware watchdog
>   (IWDG) fed only by the control loop, so that a hung control task resets
>   the MCU rather than flying on stale commands.
> - Stale RC: when `silence > 0.25 s`, stop using the last RC values for
>   attitude/thrust setpoints (hold the last mode's neutral: level attitude,
>   hold altitude where available) before the 1 s failsafe proper.
>
> Test: with the drone in flight state on the bench (props off), stall the
> BLE thread on purpose (debug flag that blocks its loop) and check that the
> 1 s and 3 s actions, the APP_LINK bit and EVENT 20 still happen on time.

**Input arbitration.** `input` in FLIGHT says who flies:

1. **ELRS radio** has absolute priority whenever the receiver is linked and its
   arm switch or sticks are active. Then RC from the app is ignored, app ARM →
   `REJECTED/RADIO_OVERRIDE`, and EVENT 43 WARN is sent once. The app switches
   to a telemetry-only cockpit.
2. **App** (RC frames) otherwise.
3. **None**: no input → in air, failsafe as above.

**RC mapping** (same as the ELRS mapping, after the app's own
deadband/expo/sensitivity):

| mode | throttle | yaw | pitch / roll |
|---|---|---|---|
| ACRO | collective | rate | rate |
| ANGLE | collective | rate (maxYawRate) | angle (maxTilt) |
| ALT_HOLD | climb rate, centre = hold (maxClimb) | rate | angle |
| POS_HOLD | climb rate, centre = hold | rate | horizontal velocity (maxHSpeed), centre = hold position |

Beginner mode (`beginner`=1) halves maxTilt, maxYawRate, maxClimb, maxHSpeed
and forbids ACRO. Indoor models cap horizontal speed at 2 m/s.

**Other failsafes** (bits in FLIGHT.failsafe, one EVENT each):

| trigger | action | event |
|---|---|---|
| ELRS lost (receiver fitted and was linked) | **fsRadio** (`0` hover, `1` land, `2` RTH) | 21 ERROR |
| battery < battCrit per cell (sagged, filtered ≥ 2 s) | RTH if GNSS+home and enough margin, else land | 31 ERROR, then 22 |
| battery < battEmerg | land now, no override except KILL | 32 CRITICAL |
| geofence: alt > gfMaxAlt, ToF height > gfCeiling (indoor), dist > gfMaxDist (GNSS) | **gfAction** (`0` stop and move back inside, `1` hover, `2` RTH) | 23 WARN |
| GNSS lost in POS_HOLD/RTH | **fsGpsLost** (`0` use optical flow if fitted, `1` ALT_HOLD, `2` land) | 24 WARN |
| flow lost (quality < threshold) in POS_HOLD indoor | **fsFlowLost** (`0` ALT_HOLD + warning, `1` land) | 25 WARN |

## 7. Arming and automatic flight

### 7.1 ARM

Accept only if **all** of:

- payload `A5 5A` (else `INVALID`);
- not armed (`ARMED`), no calibration running (`CALIBRATING`), on the ground (`NOT_LANDED`);
- ELRS not in control (`RADIO_OVERRIDE`);
- throttle low on the active input: **ACRO/ANGLE ≤ −900**, **ALT_HOLD/POS_HOLD ≤ +100** (centre stick = hold altitude) → else `THROTTLE_NOT_LOW`. If the app sent no RC in the last 0.5 s (slide-to-arm from the pre-flight screen), the throttle rule is satisfied;
- if `armRequireChecks`=1: CHECKS `ready` (`CHECKS_FAILING`); battery FAIL → `BATTERY_LOW`.

On success: motors at idle, EVENT 1, HOME set now if GNSS 3D fix (EVENT 7),
flight time reset. Auto-disarm if not taken off within 10 s.

### 7.2 TAKEOFF

`alt u16` cm, 0 → param `takeoffAlt` (default **1.0 m**), clamp 0.5…3 m.
Requires armed and on ground (`NOT_ARMED` / `NOT_LANDED`). Climb at ≤ 0.6 m/s
to the target, hold, then `auto` = NONE and mode = POS_HOLD if GNSS or FLOW,
else ALT_HOLD. EVENT 3 at start. The RC throttle stick is ignored during
takeoff; pitch/roll/yaw are honoured.

### 7.3 LAND

Descend at `landSpeed` (default 0.7 m/s; slow to 0.3 m/s below 1 m when ToF is
available), detect touchdown, EVENT 4, `flags.LANDED`, then **auto-disarm
after `autoDisarm` s** (default 2) → EVENT 2. Any stick input does not cancel
LAND (only KILL, a mode change or a new TAKEOFF).

### 7.4 RTH (GNSS models only)

Climb to `rthAlt` (or stay higher), fly home, land as 7.3. Cancelled by
pitch/roll stick > 30 % if `rthStickCancel`=1. `homeDist`, `homeBearing`,
`rthEta` in FLIGHT. On models without GNSS the command is rejected (`NO_GPS`)
and every failsafe action "RTH" becomes "land".

### 7.5 HOLD (pause)

In air: stop, hold position (POS_HOLD capable) or altitude. Moving any stick
by more than 30 % gives control back (`auto` = NONE).

### 7.6 KILL

Immediate motor stop (also in flight: the app asks for a 2 s hold and a
confirmation). Disarm, EVENT 70 CRITICAL.

## 8. Parameters

Values are **f32 on the wire**, ids are stable forever (add new ids, never
reuse). Stored in flash with a version/CRC; on a bad CRC load defaults.

| id | key | default | min | max | unit | meaning |
|---|---|---|---|---|---|---|
| 1 | takeoffAlt | **1.0** | 0.5 | 3 | m | auto takeoff target |
| 2 | landSpeed | 0.7 | 0.3 | 1.5 | m/s | |
| 3 | autoDisarm | 2 | 1 | 10 | s | after touchdown |
| 4 | maxTilt | 30 | 10 | 45 | ° | |
| 5 | maxYawRate | 120 | 30 | 360 | °/s | |
| 6 | maxClimb | 2.0 | 0.5 | 5 | m/s | |
| 7 | maxHSpeed | 8 | 1 | 15 | m/s | indoor capped at 2 |
| 8 | beginner | 0 | 0 | 1 | bool | |
| 10 | fsAppShort | 0 | 0 | 1 | enum | 0 hover, 1 land |
| 11 | fsAppLong | 2 | 0 | 2 | enum | 0 hover, 1 land, 2 RTH |
| 12 | fsRadio | 2 | 0 | 2 | enum | 0 hover, 1 land, 2 RTH |
| 13 | fsGpsLost | 0 | 0 | 2 | enum | 0 use flow, 1 ALT_HOLD, 2 land (GNSS models) |
| 14 | fsFlowLost | 0 | 0 | 1 | enum | 0 ALT_HOLD + warn, 1 land (FLOW models) |
| 20 | battWarn | 3.6 | 3.3 | 3.9 | V/cell | |
| 21 | battCrit | 3.5 | 3.2 | 3.8 | V/cell | |
| 22 | battEmerg | 3.3 | 3.0 | 3.6 | V/cell | |
| 30 | gfMaxAlt | 30 | 5 | 120 | m | all models (baro) |
| 31 | gfCeiling | 2.5 | 1 | 4 | m | ToF models |
| 32 | gfMaxDist | 150 | 20 | 500 | m | GNSS models |
| 33 | gfAction | 0 | 0 | 2 | enum | 0 stop and move back, 1 hover, 2 RTH |
| 40 | rthAlt | 30 | 5 | 60 | m | GNSS models |
| 41 | rthStickCancel | 1 | 0 | 1 | bool | GNSS models |
| 50 | armRequireChecks | 1 | 0 | 1 | bool | |
| 60 | compassPrimary | 1 | 0 | 1 | enum | 0 FC compass, 1 GPS compass (MAG_GPS models) |

Rules: battWarn > battCrit > battEmerg (if a SET breaks the order →
`INVALID/OUT_OF_RANGE`). Enum values the model cannot do (RTH without GNSS)
are accepted but executed as "land". `flags.bit0` dirty = RAM ≠ flash; the app
shows "Changes not saved on the drone yet" until PARAM_SAVE. Parameters that
need a sensor the drone lacks may be omitted from PARAM_LIST.

## 9. Pre-flight checks (CHECKS)

Send only the items that apply, others as N/A (the app greys them). `ready` =
no item in FAIL.

| id | item | value (i16) | OK / WARN / FAIL |
|---|---|---|---|
| 1 | Battery | average cell mV | FAIL < battCrit, WARN < battWarn |
| 2 | IMU & level | 0 | FAIL if IMU missing / not calibrated (accel never calibrated) |
| 3 | Compass | interference % | WARN > 15 % or compass recommended; FAIL if uncalibrated on GNSS models |
| 4 | GNSS | satellites | N/A without GNSS; WARN < 3D or < 8 sats |
| 5 | Home | horizontal accuracy dm (value/10 = m) | N/A without GNSS; WARN not set |
| 6 | ELRS radio | LQ % | N/A without receiver/not bound; WARN LQ < 70 |
| 7 | EKF | 0 | FAIL if not converged; WARN if attUnc > 3° |
| 8 | Geofence · failsafe | geofence radius m (gfMaxDist), or gfMaxAlt without GNSS | WARN if a failsafe action cannot be executed |
| 9 | Optical flow | quality % | N/A without FLOW; WARN < 50 |
| 10 | Throttle low | 0 | FAIL if the active input throttle is not low (§7.1) |
| 11 | Calibration | **low byte = required** calibrations, **high byte = recommended** (bit = CalType − 1) | FAIL if any required, WARN if any recommended |

Item 11 drives the badges in Settings › Calibration and the "Calibrate"
shortcut on the pre-flight screen. Suggested logic: ACCEL required if never
done; COMPASS recommended if interference high or never done outdoors;
FLOW_TOF recommended if never done on FLOW models; MOTORS required after a
remap; others recommended when their stored data is missing.

## 10. Calibrations

Common rules:

- Only disarmed (`CAL_START` armed → `REJECTED/ARMED`); one at a time
  (`BUSY/CALIBRATING` if another is not DONE); missing sensor →
  `REJECTED/SENSOR_MISSING`; unknown type → `INVALID`.
- During a calibration send CALIB at 10 Hz and once on every state/step
  change. Results are applied to RAM only; **CAL_SAVE** writes them to flash
  (keeping the previous set as the backup for CAL_RESTORE). **CAL_ABORT** or
  a lost app link (> 3 s) discards them.
- Failure: state FAILED with `reason` (e.g. `NOT_STILL`), EVENT 51 (arg =
  type). The app offers "Restart" (= CAL_ABORT + CAL_START).
- The app leaving the wizard sends CAL_ABORT unless the state is SAVED.

Per type (the app's screens follow exactly these steps):

### 10.1 ACCEL (type 1) – six positions

`CAL_START(1)` → WAITING_USER, step 2 (positions), `current` = first missing
position. Position order and expected specific force (body FLU):
`0` level (+Z up, f = 0,0,+1 g), `1` upside down (0,0,−1), `2` nose down
(−1,0,0), `3` nose up (+1,0,0), `4` left side down (0,−1,0), `5` right side
down (0,+1,0). For the current position: when the gyro norm stays under the
still threshold, fill `stillPct` 0→100 over **2 s** and capture automatically;
movement resets it. `CAL_CAPTURE` forces the capture now (if roughly still,
else `REJECTED/NOT_STILL`). The firmware may also accept positions in any
order by detecting the dominant axis. After 6: ellipsoid/6-point fit →
`offset`, `scale`, `fitErr`, `quality`; DONE, step 3, progress 100.

### 10.2 LEVEL (type 2) – trim

`CAL_START(2)` → RUNNING, step 2; average roll/pitch over **5 s** (`remaining`
counts down, progress 0→100); movement → FAILED/NOT_STILL. DONE, step 3:
`trimRoll/trimPitch` = −measured. In DONE the user can fine-tune:
`CAL_SET_REF(2, 0, roll_deg)`, `CAL_SET_REF(2, 1, pitch_deg)` (|v| ≤ 5°,
else `INVALID/OUT_OF_RANGE`), answered with an updated CALIB. Then CAL_SAVE.

### 10.3 GYRO (type 3) – bias

The firmware does this at every power-on already. `CAL_START(3)` → RUNNING,
5 s still (`remaining`), movement restarts the window (or FAILED/NOT_STILL
after 3 restarts). DONE with `bias`, `noise`, `temp`. CAL_SAVE stores the
bias as the boot default.

### 10.4 COMPASS (type 4)

`CAL_START(4, option)`: option `0` both compasses, `1` FC only, `2` GPS only
(→ `SENSOR_MISSING` if absent). RUNNING, step 2. Fill `coverage`: 36 sectors
in 3 rings of 12 (30° each): ring 0 (bits 0–11) rotation about the vertical
("spin flat"), ring 1 (bits 12–23) nose up/down, ring 2 (bits 24–35) roll
left/right. Any binning of the body-frame field direction that reaches 100 %
exactly when the data is good enough for the fit is fine. `progress` =
coverage %. At 100 %: ellipsoid fit per compass → DONE, step 3,
`qualityFc/qualityGps` and `interfFc/interfGps` (% field deviation with motors
off; optional motor-current compensation later). The user picks the primary
compass with PARAM_SET `compassPrimary` before CAL_SAVE.

### 10.5 FLOW_TOF (type 5)

`CAL_START(5)` → WAITING_USER, step 1: drone flat on the floor, `tof` raw
live. `CAL_CAPTURE` → `tofOffset` = current reading (ground = 0 m), step 2,
RUNNING: the user holds the drone ~1 m high and tilts it left-right then
forward-back; compare flow rate with gyro rate per axis (`match`), estimate
`scaleX/scaleY`, set `axesDone` bits when an axis has enough excitation.
Both axes → DONE, step 3. Rejected without FLOW (`SENSOR_MISSING`).

### 10.6 BATTERY (type 6)

`CAL_START(6)` → WAITING_USER, step 1, `vbat` live with the current scale.
`CAL_SET_REF(6, 0, volts_measured)` (2.5…30 V) → new `scaleV` =
scaleV·(measured / read), step 2. Then either
`CAL_SET_REF(6, 1, mAh_put_back_by_charger)` (50…20000) → `scaleI` =
mAh / usedMah, or `CAL_CAPTURE` (skip) → DONE, step 3. CAL_SAVE.

### 10.7 MOTORS (type 7) – order and direction, **props off**

Motor numbering for quad X (Betaflight): **M1 rear right CW, M2 front right
CCW, M3 rear left CCW, M4 front left CW** (the app draws this layout;
`revMask` flips the drawn direction). `CAL_START(7)` → WAITING_USER. The app
sends `MOTOR_TEST(m, 8 %, 2000 ms, 'P')` when the user taps a motor: spin
that output, CALIB with `motor`, `spinning`=1, set the `okMask` bit; at the
end `spinning`=0. `MOTOR_REVERSE(m)` toggles `revMask`. `CAL_SAVE` stores
direction (and remap) in flash. Any MOTOR_TEST must stop after `ms`, on
CAL_ABORT, on link loss and on any new command other than MOTOR_TEST.

### 10.8 STICKS (type 8) – ELRS

Rejected without a linked receiver (`SENSOR_MISSING`). `CAL_START(8)` →
WAITING_USER, step 1, `centre` live (µs). `CAL_CAPTURE` → store centres,
step 2, RUNNING: track `min`/`max` per channel while the user moves the
sticks; detect AUX switch positions (`auxPos`). `CAL_CAPTURE` → DONE, step 3.
CAL_SAVE.

## 11. Events

| code | level | name | arg | when |
|---|---|---|---|---|
| 1 | INFO | ARMED | – | arm accepted |
| 2 | INFO | DISARMED | – | any disarm |
| 3 | INFO | TAKEOFF_STARTED | target cm | |
| 4 | INFO | LANDED | – | touchdown |
| 5 | INFO | MODE_CHANGED | mode | |
| 6 | INFO | RTH_STARTED | – | command or failsafe |
| 7 | INFO | HOME_SET | – | |
| 20 | ERROR | FAILSAFE_APP_LINK | action | long app-link failsafe |
| 21 | ERROR | FAILSAFE_RADIO | action | |
| 22 | ERROR | FAILSAFE_BATTERY | action | |
| 23 | WARN | GEOFENCE_BREACH | 0 alt, 1 ceiling, 2 distance | |
| 24 | WARN | GPS_LOST | – | |
| 25 | WARN | FLOW_LOST | – | |
| 30 | WARN | BATTERY_LOW | cell mV | crossing battWarn |
| 31 | ERROR | BATTERY_CRITICAL | cell mV | crossing battCrit |
| 32 | CRITICAL | BATTERY_EMERGENCY | cell mV | crossing battEmerg |
| 40 | WARN | MAG_INTERFERENCE | % | > 30 %, rate-limited 1/30 s |
| 41 | WARN | HIGH_VIBRATION | mg | rate-limited |
| 42 | WARN | MOTOR_HOT | motor | > 80 °C |
| 43 | WARN | RADIO_OVERRIDE | – | ELRS takes control |
| 50 | INFO | CALIB_DONE | type | after CAL_SAVE |
| 51 | ERROR | CALIB_FAILED | type | |
| 60 | INFO | PARAMS_SAVED | – | |
| 70 | CRITICAL | MOTOR_KILLED | – | KILL |

The app turns events into a toast, a spoken message in the user's language
and a vibration; WARN codes 23, 24, 25, 30, 40, 41, 42, 43 can be muted by the
user, ERROR/CRITICAL cannot. Unknown codes show the `text`.

## 12. Data per app section

Every section exists on PC, tablet and phone (web, iOS, Android); the data
is the same on every device.

| App section | Reads (frames · fields) | Sends |
|---|---|---|
| **Connect** (device list, pre-flight checks, slide to arm) | advertising name; CAPS (name, model, fw); CHECKS (all items, `ready`); POWER.cells; SYSTEM.lq | REQUEST CAPS/CHECKS, PARAM_LIST, `[0x02 N]`, HEARTBEAT, ARM |
| **Cockpit** (HUD, sticks, modes, actions) | STATE (attitude); FLIGHT (all); POWER (vbat, cell, pct, used, remain, warn); NAV (fix, sats, hdop, tof, flowQ, wind); MOTORS (rpm or out); SYSTEM (lq, rssi, magInterf, estFlags); EVENT; CAPS.sensors | RC 25 Hz, ARM, DISARM, TAKEOFF, LAND, RTH (GNSS only), HOLD, KILL, SET_MODE |
| **Telemetry** (charts, all values, messages, CSV export) | STATE (q, gyro, acc, mag, bias); FLIGHT; POWER; NAV; MOTORS; SYSTEM (all); STATUS (bleDrop, bleMtu); EVENT log | – |
| **Settings › General** | CAPS.name | SET_NAME |
| **Settings › Controls & sticks** | PARAM 1–8; SYSTEM.lq (ELRS linked) | PARAM_SET, PARAM_SAVE |
| **Settings › Safety & failsafe** | PARAM 10–14, 20–22, 30–33, 40–41, 50; CAPS.sensors (hides GNSS/TOF/FLOW items); POWER.cells | PARAM_SET, PARAM_SAVE |
| **Settings › Calibration** | CHECKS item 11; SYSTEM.magInterf; CAPS.sensors | CAL_RESTORE; opens the wizards |
| **Calibration wizards** (8) | CALIB (all types); POWER; CAPS.nMotors/frame; PARAM 60 | CAL_START, CAL_CAPTURE, CAL_SET_REF, CAL_ABORT, CAL_SAVE, MOTOR_TEST, MOTOR_REVERSE, PARAM_SET 60 |
| **Settings › Connection** | SYSTEM (lq, rssi, snr, pktRate, txPower); STATUS (bleMtu, bleDrop) | `[0x02 N]` (telemetry rate) |
| **Settings › Alerts & voice** | EVENT, FLIGHT, POWER (callouts are generated in the app) | – |
| **Settings › Drone & firmware** | CAPS (fw, hwRev, serial, model, sensors, nMotors, frame, cells); SYSTEM.log | PARAM_DEFAULTS + PARAM_SAVE (reset) |

## 13. Indoor and outdoor models

The app does not know model names; it reads CAPS:

| CAPS says | App behaviour |
|---|---|
| no `GNSS` | no GNSS/Home checks (N/A), no RTH button (RTH → Land), no map/distance, no `fsGpsLost`, `gfMaxDist`, `rthAlt`, `rthStickCancel` |
| no `MAG_GPS` | compass wizard without the GPS/FC choice, no `compassPrimary` |
| `FLOW`+`TOF` | ToF altitude and flow quality in the cockpit, ceiling geofence, flow calibration, POS_HOLD available indoor |
| no `ESC_TLM` | motors shown as output % instead of RPM |
| no `ELRS` | no radio items, no sticks calibration |
| `cells` | battery thresholds shown per cell; 1S indoor models fine |

Reference configurations in the simulator: outdoor = 4S, IMU, BARO, MAG_FC,
MAG_GPS, GNSS, FLOW, TOF, ESC_TLM, CURRENT, ELRS, LEDS, FLASH_LOG; indoor = 1S,
same without GNSS, MAG_GPS, ESC_TLM and with ELRS fitted but not bound.

## 14. Bandwidth budget

With 6 bytes of framing per frame, at the default rates:

| frame | bytes × Hz | B/s |
|---|---|---|
| STATE (20 Hz, app default) | 77 × 20 | 1540 |
| STATUS | 68 × 10 | 680 |
| FLIGHT | 31 × 10 | 310 |
| NAV | 37 × 5 | 185 |
| MOTORS (quad) | 23 × 5 | 115 |
| CHECKS (11 items, disarmed) | 52 × 2 | 104 |
| POWER | 19 × 5 | 95 |
| SYSTEM | 29 × 2 | 58 |
| CAPS | 45 × 0.2 | 9 |
| **total** | | **≈ 3.1 kB/s** |

Phase 2 measured 57 STATE/s ≈ 4.4 kB/s without loss, so there is margin; at
50 Hz STATE the total is ≈ 5.4 kB/s and STATE is the first to be thinned.
Uplink: RC 11 B × 25 Hz = 275 B/s.

## 15. Suggested firmware structure and phases

Modules (names indicative): `app_link` (CMD queue, seq/ACK cache, app-link
timer, RC store), `tlm_v2` (encoders for 0x05–0x0F, scheduler with the rates
and the TX priority), `params` (table, flash store, defaults), `checks`,
`events` (queue, repeat ×3), `calib_mgr` (state machine + per-type modules),
`flight_mgr` (arm/disarm, auto actions, failsafes, input arbitration).
Telemetry encoders should be generated from, or tested against,
`src/protocol/messages.ts` (round-trip tests in `protocol.test.ts`).

Suggested order (each step is usable with the app):

1. **Link and identity** – CMD queue, ACK/seq cache, HEARTBEAT, REQUEST, CAPS,
   SYSTEM (CPU, loop, estimator fields), EVENT. *App: connect screen, About,
   Telemetry system cards.*
2. **Parameters** – table, PARAM_GET/SET/SAVE/DEFAULTS/LIST, flash. *App: all
   settings pages.*
3. **Calibrations available today** – ACCEL, LEVEL, GYRO, COMPASS (the
   current hardware: LSM6DSV16X + LIS2MDL). CALIB, CHECKS item 11. *App: 4
   wizards.*
4. **Power and checks** – POWER (once the ADC/current sensor exist), CHECKS,
   BATTERY calibration.
5. **Flight** – FLIGHT, RC, ARM/DISARM/KILL, SET_MODE, app-link failsafe,
   MOTORS, MOTOR_TEST/REVERSE (motors are disabled today: keep MOTOR_TEST
   behind the props-off magic and a compile switch).
6. **Automatic flight** – TAKEOFF/LAND/HOLD (needs altitude: baro + ToF).
7. **Outdoor** – NAV with GNSS, home, RTH, geofence, GPS compass.
8. **ELRS** – input arbitration, radio failsafe, STICKS calibration.

Until a feature exists, report it honestly: sensor bit 0 in CAPS, N/A in
CHECKS, `0xFF`/`0xFFFF` fields, `UNSUPPORTED` ACKs. The app is built for that.

## 16. Acceptance tests

Run the app (`npm run dev`, Chrome/Edge on a PC with Bluetooth, or the
Android/iOS build) against the board, and compare each behaviour with the
simulator (Connect › Simulator).

1. Connect: the device appears as `ACw…`; CAPS name and fw show in the device
   card within 1 s; the pre-flight list matches CHECKS.
2. Repeated command: send ARM twice with the same seq (e.g. by dropping the
   first ACK) → one arm, two identical ACKs.
3. Every opcode of §5 with a bad payload → `INVALID`; unknown op → `UNSUPPORTED`.
4. Settings: change takeoff altitude → PARAM echo, "not saved" bar; Save →
   EVENT 60, bar disappears; power-cycle → value kept.
5. App link: arm and take off (or simulate in-air flag), close the app → after
   1 s fsAppShort, after 3 s fsAppLong + EVENT 20.
6. ARM rules: throttle stick up in ANGLE → `THROTTLE_NOT_LOW`; a FAIL check
   with armRequireChecks=1 → `CHECKS_FAILING`.
7. Takeoff to 1.0 m, hold, land, auto-disarm after 2 s; events 3, 4, 2.
8. Each calibration wizard runs start to Save on PC and on a phone, and Abort
   leaves the previous values in place.
9. Motor test: rejected when armed or without the `P` magic; stops after 2 s,
   on abort and on link loss.
10. Indoor CAPS (no GNSS): RTH button absent, RTH command → `NO_GPS`,
    failsafe "RTH" executes "land".
11. 10 minutes connected at 20 Hz STATE: STATUS.bleDrop stays 0, no CRC errors
    in the app (Settings › Connection).
12. Watchdog independence (§6): stall the BLE thread on the drone with a debug
    flag while in flight state (props off) → fsAppShort after 1 s, fsAppLong
    and EVENT 20 after 3 s, exactly as when the phone is switched off.

## 17. Out of scope and open points

- **Firmware update over BLE** – not in v2 (Settings shows "with the USB
  tools"). A later DFU service can be added without touching this protocol.
- **USB and Wi-Fi links** for the app – tabs exist in the UI, marked "not
  available yet". The frames are transport-independent.
- **Flight logs download** from the drone flash – `log %` is shown; a
  LOG_LIST/LOG_READ pair (`0x50`/`0x51`) is reserved for a later version.
- **MOTOR_REMAP** and **STREAM_RATES** are defined and accepted by the
  simulator but the app UI does not send them yet.
- **Open point for the firmware owner:** if a field cannot be produced with
  the planned hardware, keep its offset and send the n/a value; do not shorten
  frames. Propose changes as a new frame type or by appending fields at the end
  (the app decoders read only the bytes they know, so appended bytes are
  backward compatible).
