# Roadmap

```mermaid
flowchart LR
  P1["Phase 1<br/>attitude IEKF<br/><b>done, on the board</b>"] --> P2["Phase 2<br/>BLE, app protocol v2,<br/>altitude, Flow bridge,<br/>battery, flight log, USB<br/><b>BLE on the board;<br/>the rest host-tested</b>"]
  P2 --> P3["Phase 3<br/>control loops,<br/>motors, thrust model"]
  P2 --> FC["ACwing FC rev A<br/>port"]
  FC --> P3
```

## On the board, next

- Run the acceptance tests of `HANDOVER_FW.md` §16 with the ACwing Pilot app
  (see [ACwing Pilot protocol v2](../firmware/protocol-v2.md)).
- Hand test of TAKEOFF / LAND on the bench
  ([Flight core](../firmware/flight-core.md#testing-on-the-bench)).
- ACwing Flow: program the bridge, then check the LED, CAPS and the flow axis
  signs `APP_FLOW_SIGN_*`
  ([ACwing Flow bridge](../firmware/flow-bridge.md#on-the-first-board)).
- Tune the altitude filter ($\sigma_a$, $\sigma_b$) and the height loop on the
  first flights ([Altitude Kalman filter](../theory/altitude-kf.md#limits-and-next-steps)).
- USB CDC on the Nucleo USB_USER port: enumeration, `acwing-tools log list/get`
  with the RAM log ([Flight log and USB](../firmware/logging.md#status-and-limits)).
- Battery chain with a bench supply on A0 (`APP_BATT_ADC 1`): levels,
  events, LAND, calibration ([Battery](../firmware/battery.md#testing)).

## Firmware features

- FLOW_TOF calibration: ToF offset, flow scale, axis-sign check. Then POS_HOLD
  on the flow velocity.
- ACwing Pilot app: flight logs (BLE last flight, USB all logs, viewer,
  export) and the battery items of
  [`HANDOVER_APP.md`](https://github.com/andrea-cirillo/ACwing_FlightController_FW/blob/main/02%20-%20Tools/02%20-%20App/HANDOVER_APP.md)
  are written and tested on the simulator: run them against the board. Still
  open: USB in the native Android app.
- Log commands over BLE (iOS cannot use USB CDC) and USB as a flight link,
  if the app needs them (HANDOVER_APP §10).
- The PC telemetry (STATE, INIT, STATUS) over the USB CDC link too: on the
  ACwing FC there is no ST-LINK virtual COM port.
- GNSS (ACwing GPS board): NAV, home, RTH, distance geofence, GPS compass.
- ELRS receiver: input arbitration, radio failsafe, STICKS calibration.

## Estimation

- Gyro scale and misalignment calibration (turntable or known rotations), then
  a lower `IEKF_SIGMA_GYRO`.
- Verify the magnetometer latency $\bar\tau$ on the bench, with a constant-rate
  rotation about the vertical. The internal filter delay of the LIS2MDL is not
  modelled.
- Read the LSM6DSV16X `INTERNAL_FREQ_FINE` register at start-up, so the true
  ODR is used from the first sample instead of the nominal one during the
  first 2 s.
- I²C Fast-mode Plus and 1920 Hz sampling (check the bus edges on a scope
  first).

## Port to the ACwing FC rev A

When the first boards are built:

- pin map and CubeMX configuration from the FC pin map in the hardware
  repository;
- IMU on SPI, with a new `bsp/` binding;
- IIS2MDC compass and LPS22HB barometer on the sensor hub;
- IMU axes remapped to the board axes, as noted in the FC README;
- `APP_HW_REV` = 'A' (enables the battery ADC on PA5/PA4 and the W25Q log);
- SPI1 binding of the W25Q128JV log flash (CS PE4), sharing the bus lock with
  the IMU; `erase_chip` for a fast LOG_ERASE;
- move the configuration pages below the CPU2 stack on the 512 KB part
  (STM32WB55CEU6), if that part is chosen.

## Phase 3: control

- Rate PID at 960 Hz with a notch-filtered gyro, attitude loop at 240 Hz, both
  in the existing hooks (`ctrl_rate_step`, `ctrl_attitude_step`), fed by
  `fc_setpoint()`.
- Motor power stage and mixer; DShot on the ACwing FC.
- A **thrust model** in the estimators: rejecting sustained accelerations in
  the IEKF acc update, and as an input of the altitude filter.
