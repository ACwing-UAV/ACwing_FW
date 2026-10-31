# Repository structure

The repository has four numbered top-level folders: the two firmwares
(flight controller, ACwing Flow bridge), the PC tools and the documentation
assets. Names follow the ACwing convention of the
hardware repository: `ACwing_<part>_<what>` (CubeIDE project `ACwing_FC_FW`,
Python package `acwing_tools`, MATLAB viewer `acwingLiveViewer`, logs
`acwing_log_*`). Names of the filter itself (`iekf.c`, `iekfAttitude.m`, …)
keep the name of the algorithm.

```
ACwing_FlightController_FW/
├── 00 - FC_FW/                     STM32CubeIDE project of the flight-controller firmware (import this folder)
│   ├── App/                        application code, hardware-independent where possible
│   │   ├── app/                    app_main (FreeRTOS tasks, IRQ glue), app_config.h (ALL parameters),
│   │   │                           est_pipeline (multi-rate attitude estimation), tlm_build (telemetry frames),
│   │   │                           app_task (protocol v2: commands, params, flash, v2 frames), params,
│   │   │                           cfg_store (A/B flash image), fc_shared (ImuTask <-> AppTask data),
│   │   │                           ext_sens (ExtTask: probe/poll the Qwiic ToF and flow, host-tested),
│   │   │                           batt_task (BattTask: battery monitor, signals the flight core),
│   │   │                           log_task (LogTask: flight log, USB log commands)
│   │   ├── filters/                state estimators (no HAL, no RTOS, host-testable)
│   │   │   ├── iekf/               attitude IEKF on SO(3): so3 (Exp/Log, orthonormalisation), iekf (filter),
│   │   │   │                       accwin (rotation-compensated acc mean), calib (static calibration window)
│   │   │   └── altitude_kf/        altitude Kalman filter (alt_est): height, climb rate, baro and acc bias
│   │   ├── control/                fc_core (arming, modes, TAKEOFF/LAND/HOLD, failsafes), alt_ctrl (height
│   │   │                           loop), calib_mgr (calibrations), checks (pre-flight), battery (estimator:
│   │   │                           cells, level, state of charge, time left), control hooks (weak,
│   │   │                           empty), motor PWM (disabled)
│   │   ├── log/                    flight log: log_format.h (on-flash format), log_store (sector ring),
│   │   │                           log_queue (producer rings), log_feed (record builders), log_flash_ram
│   │   ├── drivers/                chip drivers, portable (bus given as function pointers, host-testable):
│   │   │                           sensor_bus.h, imu_lsm6dsv16x (IMU, sensor hub with compass and barometer,
│   │   │                           FIFO parser), vl53l1x (ToF), flow_bridge (ACwing Flow), w25q (log flash)
│   │   ├── bsp/                    board support, tied to the STM32WB55 + HAL/RTOS: imu_i2c (HAL I²C + DMA
│   │   │                           binding, bus lock), ble_tlm_svc (GATT service + TX thread on STM32_WPAN),
│   │   │                           cfg_flash_wb (flash pages shared with CPU2), ext_i2c (HAL binding of the
│   │   │                           Qwiic devices for the vl53l1x / flow_bridge drivers), batt_adc (ADC1
│   │   │                           injected group), usb_cdc + usb/ (USB CDC device: usbd_conf, descriptors)
│   │   ├── telemetry/              frame format and CRC (telemetry.h), UART + DMA transport;
│   │   │                           tlm_ble: BLE queues HI/LO + link state; proto_v2 (app protocol ids and
│   │   │                           layouts), app_link (CMD receive side), ack_cache, usb_link (USB byte
│   │   │                           stream), spsc (all host-testable)
│   │   └── third_party/            ST lsm6dsv16x-pid driver (BSD-3), CMSIS-DSP headers and library,
│   │                               ST USB device library 2.11.6 (st_usb, SLA0044) and the HAL PCD/USB
│   │                               and LL ADC files (st_hal_usb_adc, BSD-3), kept out of CubeMX's reach
│   ├── Core/                       CubeMX-generated: main.c, interrupts, HAL MSP, FreeRTOS config, startup,
│   │                               radio-core infrastructure (app_entry, hw_timerserver, app_conf.h)
│   ├── Tests/host/                 host tests (make): flight core, failsafes, TAKEOFF/LAND, altitude filter,
│   │                               drivers, AppTask protocol, BLE rings; ExtTask end to end with the Flow
│   │                               bridge logic of 01 - FLOW_FW; battery (power_test); flight log (log_test)
│   ├── STM32_WPAN/App/             CubeMX-generated BLE application (app_ble.c with our 3 hooks, see BLE telemetry)
│   ├── Drivers/, Middlewares/      CubeMX-generated: STM32WB HAL, CMSIS, FreeRTOS, ST STM32_WPAN (BLE)
│   ├── Utilities/                  CubeMX-generated: low-power manager used by STM32_WPAN
│   ├── ACwing_FC_FW.ioc            CubeMX configuration (pins, clocks, DMA, FreeRTOS, BLE)
│   ├── ACwing_FC_FW.launch         STM32CubeIDE debug configuration (ST-LINK, SWD)
│   ├── .project, .cproject, .mxproject, .settings/   STM32CubeIDE project files
│   └── STM32WB55RGVX_FLASH.ld, ..._RAM.ld   linker scripts
├── 01 - FLOW_FW/                   ACwing Flow bridge firmware (STM32C011F4U6): PMW3901 over SPI -> I²C target
│                                   0x3B; Makefile, CMSIS only, host tests; see its README
├── 02 - Tools/                     PC tools (MATLAB and Python, equivalent) and the ACwing Pilot app
│   ├── 00 - Matlab/                MATLAB tools (R2019b+)
│   │   ├── initPath.m              adds every folder below to the path
│   │   ├── Viewer/                 acwingLiveViewer.m (live 3D triad + logging, USB or BLE), quat2rotmLocal.m
│   │   ├── Telemetry/              parseTelemetry, decode*, crc16ccitt, statusDiagnosis
│   │   ├── MagCalib/               calibMagAxes.m (axis map + hard-iron), fitHardIron.m, magCheck.m
│   │   ├── FlightLog/              acwingLogDownload.m (USB), acwingLogDecode.m (flight log format 1)
│   │   └── FiltersComparison/      tlmCapture, tlmSniffer, compareEmbeddedVsMatlab
│   │       ├── IEKF_originalVersion/   ORIGINAL filter developed in MATLAB: iekfAttitude.m, expSO3, logSO3, skewSym
│   │       └── IEKF_embeddedVersion/   EXTENDED filter: iekfAttitudeEmb.m + iekfCore*.m (MATLAB mirror of the C code)
│   ├── 01 - Python/                Python equivalents of the MATLAB tools (package acwing_tools, CLI acwing-tools)
│   │   ├── acwing_tools/           viewer, capture + sniffer, compare, magcalib, telemetry,
│   │   │                           ble_io (BLE link, bleak), filters/ (original + embedded mirror),
│   │   │                           logio (.mat logs), simulate, flightlog (flight log download + decode)
│   │   ├── tests/                  pytest suite: parity with the MATLAB outputs, recorded captures
│   │   ├── pyproject.toml, requirements.txt
│   │   └── README.md               installation, MATLAB ↔ Python map, commands
│   └── 02 - App/                   ACwing Pilot: flight/telemetry/settings/calibration app (web/PWA, iOS, Android)
│       ├── src/                    React + TypeScript; protocol/ (v2 frames and commands), sim/ (reference drone)
│       ├── HANDOVER_FW.md          drone ↔ app protocol v2: what the firmware must implement
│       └── README.md               run, simulator, native builds
├── 03 - Docs/                      documentation site sources (MkDocs Material): theory, firmware,
│   │                               tools, guides, decisions (ADR); renders on GitHub as well
│   ├── 00 - Brand/                 ACwing logos (SVG/PNG), copied from the hardware repository
│   ├── api/                        API reference landing page; Doxygen output goes to api/fc, api/flow
│   └── requirements.txt            pinned MkDocs versions
├── .github/workflows/docs.yml      CI: Doxygen + mkdocs build --strict, publish to GitHub Pages
├── mkdocs.yml                      site configuration (navigation, math, Mermaid, link checking)
├── Doxyfile.fc, Doxyfile.flow      API reference of the two firmwares (Doxygen + Graphviz)
├── .gitignore                      build outputs (Debug/, Release/, site/, api/fc, api/flow) are not versioned
└── README.md                       short entry point: what ACwing is, how to start, links to this site
```

**Paths in this document.** Short paths are relative to the folder they
belong to:

| Paths like | Are relative to |
|---|---|
| `App/…`, `Core/…`, `ACwing_FC_FW.ioc` | `00 - FC_FW/` |
| `Viewer/…`, `MagCalib/…`, `FiltersComparison/…`, `initPath.m` | `02 - Tools/00 - Matlab/` |
| `acwing_tools/…`, `tests/…` | `02 - Tools/01 - Python/` |

**Three implementations of the same filter**, each with its role:

| Implementation | Role |
|---|---|
| `App/filters/iekf/` (C, float32) | what flies |
| `FiltersComparison/IEKF_originalVersion/` (MATLAB) | the filter as first designed, kept **unchanged** as the reference of the equations |
| `FiltersComparison/IEKF_embeddedVersion/` (MATLAB) | MATLAB mirror of the C code, written line by line after `iekf.c`, with every improvement of [IEKF on the microcontroller](../theory/iekf-embedded.md) |

`acwing_tools/filters/` (Python) ports both MATLAB filters, matching them to
~1e-14.

**Where to start:** see the table on the [home page](../index.md#where-to-start).
