# 0002. Multi-rate estimation pipeline driven by the IMU FIFO

- **Status:** Accepted
- **Date:** phase 1 (recorded on 2026-10-27)

## Context

The gyroscope has to drive the propagation and the rate loop at 960 Hz. The
accelerometer is noisy and vibrates, so it does not need to be used at
960 Hz. The magnetometer is slow, and it is read through the sensor hub.
Timestamps from separate reads would add jitter.

## Decision

- The LSM6DSV16X FIFO holds gyro, acc, magnetometer and (later) barometer
  samples in the order they were taken. One DMA burst per interrupt reads
  them all.
- `est_pipeline.c` handles the samples:
  - propagation every gyro step (960 Hz);
  - accelerometer update every 4 steps (240 Hz), on a rotation-compensated
    window mean;
  - magnetometer update when a new sample arrives, with its age compensated.
- The sample period is measured against the MCU crystal, not assumed.

## Consequences

- One time base for every sensor, with no per-sensor timestamps.
- The rate loop hook runs right after the propagation, before the
  corrections, which gives the lowest latency.
- The pipeline is pure C, so the same code runs in the PC simulation, in the
  host tests and on the board.
- Cost: the INT1 level on an edge-triggered EXTI needs the "read again at
  once" recovery (see [Troubleshooting](../guides/troubleshooting.md)).

## Alternatives considered

- **One update per sample at 960 Hz.** It costs CPU, and it feeds vibration
  into the attitude.
- **Separate tasks per sensor.** They need locks and timestamps, and they
  bring jitter.
