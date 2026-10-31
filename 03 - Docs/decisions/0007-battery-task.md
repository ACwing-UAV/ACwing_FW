# 0007. Battery monitor in its own task, decisions in the flight core

- **Status:** Accepted
- **Date:** 2026-10-31

## Context

The battery failsafes (LAND below battCrit, locked landing below battEmerg)
change the flight state, which only the flight core may change
([ADR 0003](0003-flight-core-in-imu-task.md)). The measurement itself (ADC,
filters, cell count, internal resistance, state of charge, calibration
scales, flight charge) is a separate concern that grows over time. The first
idea was to read the ADC in the ImuTask loop next to the flight core.

## Decision

- A **BattTask** (`osPriorityNormal`, 50 Hz) owns the ADC and the estimator
  (`control/battery.c`, plain C, host-tested). It publishes its state through
  a seqlock for the AppTask (POWER, CHECKS, CALIB) and the log.
- It **signals** the flight core through an SPSC queue, only when the warning
  level or the presence changes, and refreshes a heartbeat time stamp every
  step.
- The flight core keeps the **decisions**: events 30/31/32 and 22, LAND or
  RTH, the EMERGENCY lock, ARM and TAKEOFF refusal.
- The flight core treats a heartbeat older than 1 s as "battery unknown"
  (EVENT 33, ARM refused), because the IWDG is fed only by the ImuTask and
  would not notice a stalled BattTask.

## Consequences

- The ImuTask loop does not grow; the battery code can change without
  touching the flight core.
- The level reaches the flight core within one BattTask period (20 ms) plus
  one flight-core cycle (4 ms): negligible against the 2 s filter the
  handover asks for.
- One more task (768 B stack) and one more failure mode (a silent BattTask),
  which is detected and reported.

## Alternatives considered

- **ADC and estimator in the ImuTask.** Fewer moving parts and no heartbeat,
  but the realtime loop would carry filtering and calibration code that does
  not need 250 Hz, and every change to it would touch the flight loop.
- **Failsafe decisions in the BattTask.** Simpler signalling, but two tasks
  would write the flight state.
