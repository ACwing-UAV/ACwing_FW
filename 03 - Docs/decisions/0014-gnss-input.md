# 0014. GNSS input: u-blox M10 on the Qwiic bus, UBX only, detected but not a position source yet

- **Status:** Accepted (2026-11-14); implemented in firmware 0.15.0, host validation below
- **Date:** 2026-11-14

## Context

The ACwing GPS board carries a u-blox M10 receiver and an IIS2MDC compass. It plugs into the Qwiic connector, the
same I²C bus as the ACwing Flow board. The outdoor features need it:

- POS_HOLD outdoors;
- home and RTH;
- the distance geofence;
- the GPS compass.

They need more than a driver: a horizontal position estimator, home, and a choice between the two compasses. The
work is split into four steps, each with its own PR:

1. **GNSS input (this ADR):** drivers, detection, the solution in the telemetry and the checks.
2. Horizontal estimator (GNSS + IMU, the flow indoors), home, compass selection, declination.
3. POS_HOLD outdoors, RTH, distance geofence, `fsGpsLost`.
4. App map and mission protocol.

Decisions already taken with the owner:

- the GPS compass is the primary one;
- the FC compass is a redundant cross-check and the fallback, never averaged with it;
- the map uses MapLibre + OpenFreeMap + PMTiles behind a provider abstraction.

## Decision

### 1. Transport: I²C (DDC) in the ExtTask, not a UART

- The M10 sits at 0x42 on the Qwiic bus. It is read by the **ExtTask**, which already probes, polls and loses the
  Flow devices. The board can be plugged in at any time, and the same error rules apply: 50 errors and the device
  is lost, then probed again every 5 s.
- DDC registers:
  - 0xFD/0xFE hold the byte count;
  - 0xFF is the stream.
- A write is the UBX frame as plain bytes. On `sensor_bus_t`, the frame's first byte goes as the "register" and the
  rest as data, which gives the same transfer on the wire. No new bus type is needed.
- At most 128 bytes are read per 10 ms poll, in 64-byte reads. A 64-byte read takes about 1.6 ms at 400 kHz, and the
  HAL timeout is 3 ms. That is about 12.8 kB/s, ten times what NAV-PVT needs at 5 Hz.

### 2. Configuration: one CFG-VALSET in the RAM layer at every probe

| Key | Value | Why |
|---|---|---|
| `CFG-I2COUTPROT-UBX` | 1 | |
| `CFG-I2COUTPROT-NMEA` | 0 | NMEA is never parsed; it would only take bus time |
| `CFG-MSGOUT-UBX_NAV_PVT_I2C` | 1 | one message has position, velocity, accuracy, fix and time |
| `CFG-RATE-MEAS` | 200 ms (5 Hz) | the IMU fills the gaps in step 2; it is the common rate of M10 setups with 4 constellations |
| `CFG-NAVSPG-DYNMODEL` | 8, airborne < 4 g | hand launch and flips exceed the 1 g model |

- **RAM layer only.** Nothing is written to the receiver's flash, so there is no wear and no hidden state. The
  configuration is sent again after a power cycle of the GPS board, because the probe runs again.
- **Configured** means an ACK-ACK has arrived, or a NAV-PVT has (the ACK can be lost behind the NMEA backlog from the
  boot).
- Without either within 1 s the frame is sent again. After 3 tries the device is dropped and probed again 5 s later.
- No NAV-PVT for 2 s also drops the device: a receiver that stops answering must not leave an old solution in use.

### 3. The solution is a latest-value slot, the compass goes into the queue

- NAV-PVT goes into a seqlock slot (`ext_gnss_read`), with its reception time and a counter. Only the newest fix
  matters, and the AppTask and the future estimator read it on their own schedule.
- The IIS2MDC samples (50 Hz, raw, sensor axes) go into the existing SPSC queue as `EXT_MAG`. Each sample keeps its
  time for the fusion in step 2. Until then the flight context drops them.

### 4. Detected is not usable

The CAPS bits `GNSS` and `MAG_GPS` mean **the hardware answered**. They are not taken as "the drone can navigate on
GNSS":

- the flight core's `gnss()` stays 0 until step 2 fuses the solution. A new `pos_source()` (GNSS navigating, or the
  flow) gates POS_HOLD, the mode after a takeoff and the end of a hand launch;
- without this rule, plugging in the GPS board would have enabled POS_HOLD with no position, the same fault the
  `pos_hold` flag fixed for the flow;
- CHECKS items 4 (GNSS: 3D and ≥ 8 satellites, value = satellites) and 5 (home: WARN, none is set yet) are now
  evaluated with the board, as **WARN, never FAIL**;
- item 3 (compass) gives FAIL when uncalibrated "on GNSS models" only once the GNSS navigates (`checks_in_t.gnss_nav`).
  Until then, a plugged board does not block arming for a compass no navigation uses.

### 5. NAV frame

The layout does not change.

- `fix`, `sats`, `lat`, `lon`, `altMsl` and `vel NED` come from the receiver while the solution is usable (3D,
  `gnssFixOK`, position valid) and fresh. Otherwise `altMsl` keeps the baro and the velocity keeps the flow.
- **`hdop` carries the pDOP.** NAV-PVT has no hDOP, and pDOP ≥ hDOP, so the value never looks better than it is.
  The app label now reads PDOP.

## Consequences

- With the GPS board the app sees an outdoor model:
  - it shows the GNSS data, and the RTH button and GNSS parameters appear;
  - RTH is still refused (`NO_GPS`), POS_HOLD is refused without the flow, and the takeoff ends in ALT_HOLD;
  - this is the honest state until step 3; the documents say so.
- The GPS-board compass is read but not used; its mounting matrix and calibration come in step 2.
- ExtTask stack: 768 → 1024 B (the DDC chunk, the frame being built, the decoded solution). The heap and stack
  margins in SYSTEM (FW 0.14) will show the real use.
- The configuration keys are from the M10 interface description. They have been checked only against synthetic
  frames, not against a real receiver yet: the on-board check issue covers it.

## Alternatives considered

- **UART.** USART1 belongs to the RC receiver; a UART GPS would need the LPUART and a second connector. The owner
  chose the Qwiic bus: one plug for both expansions. The cost is the shared bus time (§1), to be measured on the board.
- **NMEA parsing.** The output is text, larger, has no velocity accuracy and needs several sentences per solution.
  Rejected.
- **Saving the configuration to the receiver's flash.** It saves the VALSET at boot, but it leaves state outside the
  firmware: a receiver configured by u-center would behave differently. Rejected.
- **10 Hz.** M10 can do it with fewer constellations. It doubles the bus load for little gain, since the IMU fills
  the gaps. It can be revisited with flight logs.
- **The FC compass as the primary one.** It is close to the motor currents and the battery. The owner chose the
  GPS compass as primary.

## Validation (host, firmware 0.15)

`make gnss_test` (in `make`, so in the CI), 58 checks:

| Case | Result |
|---|---|
| parser | frame found after NMEA and a false sync; bad checksum counted and dropped; a 300-byte payload skipped without overrun |
| NAV-PVT | every field used decoded; 3D / 2D / no gnssFixOK / invalid LLH / GNSS+DR |
| CFG-VALSET | bytes of the 5 keys, RAM layer, checksum |
| M10 on a simulated DDC | configuration on probe; ACK found behind 490 B of NMEA; a frame split over two reads; a backlog drained in order; no answer → 3 VALSET → dropped at 3 s; NAV-PVT stands for the ACK |
| IIS2MDC | absent, wrong WHO_AM_I, configuration, sample |
| ExtTask | board plugged in later → GNSS + MAG_GPS at the 5 s probe; latest solution; 50 compass samples per second; silence → lost after 2 s → probed again; compass lost after 50 errors |
| NAV and CHECKS | fields from the solution; stale → no fix; 0xFF without the board; items 3, 4, 5 as in §4 |

**On the board:** a manual-check issue covers the real receiver (ACK, 5 Hz, NAV-PVT on I²C, time to first fix, the
I²C load with the Flow board also plugged in) and the IIS2MDC WHO_AM_I and data.
