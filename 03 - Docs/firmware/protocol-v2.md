# ACwing Pilot protocol v2

The **ACwing Pilot** app (`02 - Tools/02 - App/`) talks to the drone with
protocol v2. The contract is
[`HANDOVER_FW.md`](https://github.com/andrea-cirillo/ACwing_FlightController_FW/blob/main/02%20-%20Tools/02%20-%20App/HANDOVER_FW.md),
and the app simulator `src/sim/simDrone.ts` is the reference for behaviour.

Protocol v2 only **adds** frames `0x05…0x0F` and commands `≥ 0x10` on the GATT
service of [BLE telemetry](ble.md). The phase-2 frames, the legacy commands
and the PC tools are unchanged; the PC tools ignore the new frame types.

The code is in `App/app/app_task.c` (dispatcher, frames, flash), in
`App/telemetry/` (`proto_v2`, `app_link`, `ack_cache`, `tlm_ble`) and in the
[flight core](flight-core.md).

## Frames sent by the drone

| Type | Frame | When | Source |
|---|---|---|---|
| `0x05` | CAPS | on subscribe, on REQUEST, every 5 s, when a sensor appears | AppTask |
| `0x06` | FLIGHT | 10 Hz (STREAM_RATES 1–20) | flight-core snapshot |
| `0x07` | POWER | not sent: no battery measurement yet | – |
| `0x08` | NAV | 5 Hz, also on indoor models (baro, ToF, flow quality; `fix = 0xFF`) | estimator snapshot |
| `0x09` | MOTORS | 5 Hz | flight-core snapshot |
| `0x0A` | SYSTEM | 2 Hz | estimator snapshot (loop rate, jitter, CPU, vibration, attitude uncertainty, magnetic interference) |
| `0x0B` | EVENT | on event; ERROR and CRITICAL sent 3 times, 100 ms apart | flight core, AppTask |
| `0x0C` | PARAM | answer to PARAM_GET/SET/LIST/SAVE/DEFAULTS, lists paced to the TX ring | AppTask |
| `0x0D` | CALIB | 10 Hz while a calibration is active, and on every state change | calib_mgr |
| `0x0E` | ACK | one per command with a sequence number | AppTask |
| `0x0F` | CHECKS | 2 Hz disarmed, 0.5 Hz armed, on REQUEST | checks |

The payload layouts are packed C structs in `telemetry/proto_v2.h`, checked by
`_Static_assert`. The host test checks them byte for byte against the app's own
encoders and decoders (see [Testing](testing.md)).

## Connection: subscribe order

```mermaid
sequenceDiagram
  participant App as ACwing Pilot
  participant M0 as BLE stack (CPU2)
  participant BLE as tlm_ble (rings)
  participant AT as AppTask
  participant TT as TelemetryTask
  App->>M0: connect, MTU exchange, subscribe (CCCD)
  M0->>BLE: state STREAMING → subscribe flag, LO ring closed
  AT->>BLE: take_subscribe(): reset ACK cache
  AT->>BLE: HI: CAPS
  AT->>BLE: HI: INIT (copy of the estimator INIT frame)
  AT->>BLE: HI: CHECKS
  AT->>BLE: open_streams(): LO ring accepts frames
  TT->>BLE: LO: STATE (decimated), STATUS
  AT->>BLE: HI: FLIGHT 10 Hz, NAV, MOTORS, SYSTEM, CHECKS, CAPS…
  BLE-->>App: notifications, HI first, frame boundaries kept
```

The app always learns what the drone can do (CAPS) and the filter
configuration (INIT) before the first periodic frame.

## Commands and ACKs

Commands are `[op][seq][payload]`, at most 20 bytes, written without
response on the CMD characteristic. RC (`0x11`) and HEARTBEAT (`0x10`) have no
ACK; every other command gets **exactly one** ACK.

```mermaid
sequenceDiagram
  participant App
  participant L as app_link (BLE thread)
  participant AT as AppTask
  participant C as ACK cache
  participant FC as flight core (ImuTask)
  App->>L: ARM [0x12][seq 7][A5 5A]
  L->>L: time stamp (app-link timer)
  L->>AT: command queue (SPSC)
  AT->>C: begin(0x12, 7) → NEW
  AT->>FC: core command queue (SPSC)
  FC->>FC: exec: arming rules (§7.1 of the handover)
  FC-->>AT: result OK, EVENT ARMED
  AT->>C: finish(0x12, 7, OK)
  AT-->>App: ACK {0x12, 7, OK}, EVENT 1
  Note over App: no ACK within 600 ms?<br/>re-send, same seq (twice at most)
  App->>L: ARM [0x12][seq 7] (repeat)
  L->>AT: command queue
  AT->>C: begin(0x12, 7) → DONE
  AT-->>App: cached ACK, ARM not executed again
```

- **ACK cache** (`ack_cache.c`, 32 entries, the handover asks ≥ 16). A repeat
  of a command still running (PENDING) is ignored, since its ACK is on the
  way. A repeat of a finished one gets the cached ACK again. A repeated ARM,
  TAKEOFF or MOTOR_TEST can never run twice.
- **Core commands** (ARM, DISARM, TAKEOFF, LAND, RTH, HOLD, KILL, SET_MODE,
  MOTOR_TEST) go to the flight core and are executed in the ImuTask. While
  one is pending, the AppTask takes no new command (at most one control cycle,
  4 ms).
- **Local commands** (parameters, calibrations, requests, name) run in the
  AppTask. Any of them stops a running motor test first, as the handover asks.
- Unknown opcode → `UNSUPPORTED`; a payload that is too short or has a bad
  magic → `INVALID`.

## BLE rings and priorities

Two rings of 2 KB feed the TX thread (`telemetry/tlm_ble.c`). The thread
drains **HI** first and switches ring only at a frame boundary, so a frame is
never interleaved with another. Each frame type may be queued only up to a fill
level, so that the important frames always find room:

| Frame types | Ring | Admitted up to |
|---|---|---|
| ACK, EVENT, CAPS | HI | 100 % |
| CALIB, PARAM, INIT, STATUS | HI / LO | 85 % |
| FLIGHT, CHECKS | HI | 70 % |
| POWER, NAV | HI | 55 % |
| STATE | LO | 50 % |
| MOTORS, SYSTEM | HI | 45 % |

Parameter lists are paced: the AppTask sends the next PARAM only while more
than 256 bytes are free in the HI ring, so a list never pushes out an ACK.

## What is implemented, and what is reported as missing

| Area | Implemented | Reported honestly as missing (as §15 of the handover asks) |
|---|---|---|
| Frames | CAPS (sensor bits detected at run time), FLIGHT 10 Hz, NAV 5 Hz (also indoor: baro, ToF, flow quality), MOTORS 5 Hz, SYSTEM 2 Hz, CHECKS 2 Hz (0.5 Hz armed), CALIB, EVENT (ERROR and CRITICAL sent 3 times, 100 ms apart), PARAM, ACK | POWER: not sent, and `REQUEST(POWER)` returns `UNSUPPORTED` (no ADC yet) |
| Commands | ACK cache of 32 entries: a repeat gets the cached ACK and is not executed again. ARM, DISARM, KILL, SET_MODE, TAKEOFF, LAND, HOLD, PARAM_*, CAL_*, MOTOR_TEST (behind `MOTORS_ENABLE`), STREAM_RATES, REQUEST, SET_NAME | RTH → `NO_GPS`. MOTOR_REVERSE and MOTOR_REMAP → `UNSUPPORTED` |
| Parameters | the full ids.ts table; A/B flash pages with seq + CRC32; save allowed only when disarmed | – |
| Calibrations | ACCEL (6 positions), LEVEL, GYRO, COMPASS (ellipsoid fit, 36-sector coverage), each with one backup per type (CAL_RESTORE) | FLOW_TOF → `UNSUPPORTED` while the board is present. BATTERY, STICKS, MOTORS → `SENSOR_MISSING` |
| Failsafes | app link (RC stale 0.25 s, short 1 s, long 3 s with EVENT 20, link back 0.5 s, ground disarm 3 s), geofence ceiling (gfMaxAlt, gfCeiling with ToF), flow lost in POS_HOLD | radio, battery, GNSS: no hardware yet |

## Parameters and flash

- The table is the same as `ids.ts` (`App/app/params.c`): ids, defaults,
  ranges and the sensor each parameter needs. Parameters whose sensor is
  missing are not listed. PARAM_SET checks the range and the battery
  thresholds' order (warn > crit > emergency), and changes RAM only (flag
  *dirty*).
- PARAM_SAVE, CAL_SAVE, CAL_RESTORE and SET_NAME write one **flash image**
  (`App/app/cfg_store.c`): parameters, name, calibrations, and one backup per
  calibration type. There are two pages used alternately (A/B), each image
  with a sequence number and a CRC32, so a reset during a write leaves the
  previous image valid. All of these are refused while armed.
- On the WB55 the write is coordinated with CPU2 (`bsp/cfg_flash_wb.c`): the
  flash semaphore is taken, and the erase starts only when CPU2 allows it, as
  ST requires while the radio runs.

## Calibrations

`App/control/calib_mgr.c` runs in the AppTask. The ImuTask sends raw samples
(calibration removed) only while a calibration needs them, and takes the
result when it is DONE.

| Type | Procedure | Result |
|---|---|---|
| ACCEL | six positions in any order, 2 s still window each | per-axis offset and scale (FAILED outside 0.9–1.1 or offset > 0.25 g) |
| LEVEL | 5 s average of roll and pitch, the board still | trim = − measured (CAL_SET_REF ≤ 5°) |
| GYRO | 5 s still window; movement restarts it, more than 3 restarts → FAILED | bias |
| COMPASS | 36-sector coverage in 3 rings, 6-parameter axis-aligned ellipsoid fit | hard iron + per-axis scale, quality and interference |

ACCEL and COMPASS restart the attitude estimator, because the gravity and
field references change; GYRO moves the IEKF bias by the same amount. FLOW_TOF
is `UNSUPPORTED` for now; BATTERY, STICKS and MOTORS are `SENSOR_MISSING`.
