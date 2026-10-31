<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="03%20-%20Docs/00%20-%20Brand/ACwing_logo_white.png">
    <img alt="ACwing" src="03%20-%20Docs/00%20-%20Brand/ACwing_logo_black.png" width="260">
  </picture>
</p>

<h1 align="center">ACwing – firmware</h1>

<p align="center">
  Flight controller on STM32WB55 · attitude IEKF and altitude Kalman filter · FreeRTOS · BLE<br>
  ACwing Flow bridge on STM32C011 · PC tools in MATLAB and Python · ACwing Pilot app<br>
  by Andrea Cirillo
</p>

---

**ACwing** is an open project for small multirotors. The hardware (flight
controller, power board, GNSS and optical-flow expansions) is in
[ACwing_FlightController_HW](https://github.com/andrea-cirillo/ACwing_FlightController_HW).
This repository holds the software:

| Folder | What |
|---|---|
| [`00 - FC_FW`](00%20-%20FC_FW) | flight-controller firmware (STM32CubeIDE project `ACwing_FC_FW`): attitude IEKF on SO(3), altitude Kalman filter, flight core with arming, failsafes and TAKEOFF/LAND, battery monitor and failsafes, flight log with USB download, ACwing Pilot protocol v2, telemetry over USB and BLE |
| [`01 - FLOW_FW`](01%20-%20FLOW_FW) | ACwing Flow bridge firmware (STM32C011): PMW3901 optical flow → I²C target |
| [`02 - Tools`](02%20-%20Tools) | PC tools in MATLAB and Python (live viewer, calibration, validation) and the ACwing Pilot app |
| [`03 - Docs`](03%20-%20Docs) | the documentation: theory, firmware, tools, guides, decisions |

## Documentation

The documentation is published at **[andrea-cirillo.github.io/ACwing_FlightController_FW](https://andrea-cirillo.github.io/ACwing_FlightController_FW/)**,
built from [`03 - Docs`](03%20-%20Docs/index.md); the pages also read directly on
GitHub. The [API reference](https://andrea-cirillo.github.io/ACwing_FlightController_FW/api/fc/) (Doxygen) exists only on the published site.

| Start here | |
|---|---|
| [Home](03%20-%20Docs/index.md) | what ACwing is, status, where to start |
| [Repository structure](03%20-%20Docs/overview/repository.md) | every folder and file |
| Theory | [vehicle model](03%20-%20Docs/theory/vehicle-model.md) · [attitude IEKF](03%20-%20Docs/theory/iekf.md) · [IEKF on the microcontroller](03%20-%20Docs/theory/iekf-embedded.md) · [altitude Kalman filter](03%20-%20Docs/theory/altitude-kf.md) |
| Firmware | [architecture](03%20-%20Docs/firmware/architecture.md) · [flight core](03%20-%20Docs/firmware/flight-core.md) · [sensors](03%20-%20Docs/firmware/sensors.md) · [battery](03%20-%20Docs/firmware/battery.md) · [flight log and USB](03%20-%20Docs/firmware/logging.md) · [protocol v2](03%20-%20Docs/firmware/protocol-v2.md) · [BLE](03%20-%20Docs/firmware/ble.md) · [Flow bridge](03%20-%20Docs/firmware/flow-bridge.md) · [build](03%20-%20Docs/firmware/build.md) · [testing](03%20-%20Docs/firmware/testing.md) |
| Tools | [PC tools and live viewer](03%20-%20Docs/tools/pc-tools.md) · [magnetometer calibration](03%20-%20Docs/tools/mag-calibration.md) · [validation](03%20-%20Docs/tools/validation.md) |
| Why it is built this way | [decisions (ADR)](03%20-%20Docs/decisions/index.md) |
| API | Doxygen: [FC firmware](https://andrea-cirillo.github.io/ACwing_FlightController_FW/api/fc/) · [Flow bridge](https://andrea-cirillo.github.io/ACwing_FlightController_FW/api/flow/) · [how it is built](03%20-%20Docs/api/index.md) |

## Quick start

| I want to… | |
|---|---|
| build and flash the FC firmware | import `00 - FC_FW` in STM32CubeIDE: [Building and flashing](03%20-%20Docs/firmware/build.md) |
| see the attitude live | MATLAB `acwingLiveViewer` or `acwing-tools viewer`: [PC tools](03%20-%20Docs/tools/pc-tools.md) |
| run the host tests | `make` in `00 - FC_FW/Tests/host`, `make test` in `01 - FLOW_FW`: [Testing](03%20-%20Docs/firmware/testing.md) |
| build the documentation | `pip install -r "03 - Docs/requirements.txt"`, `mkdocs serve`: [Writing the documentation](03%20-%20Docs/project/documentation.md) |

## Status

- **Phase 1, attitude estimation: concluded.** Validated on the board against
  the MATLAB reference (≤ 0.003 σ).
- **Phase 2:**
  - BLE telemetry is validated on the board.
  - App protocol v2, failsafes, altitude with TAKEOFF/LAND, the Flow
    bridge, the battery monitor and the flight log with USB download are
    implemented and host-tested, not yet run on the board.
- **Phase 3, control loops and motors: planned.** Motors are disabled
  (`MOTORS_ENABLE = 0`).

Details and next steps: [Roadmap](03%20-%20Docs/project/roadmap.md).
