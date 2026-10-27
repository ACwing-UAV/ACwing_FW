# Troubleshooting


| Symptom | Cause / action |
|---|---|
| LED1 blinking every 200 ms | `imu_init` failing: jumpers (Mode 3), I²C pins, power |
| No bytes at all | wrong COM port, port held by another program (`clear all`), firmware halted |
| Only `A5 A5 A5 …` | USART1 TX DMA without *Memory increment* |
| Bytes but no valid frame | baud mismatch or SYSCLK ≠ 64 MHz |
| One frame, then nothing | UART TX-complete callback not wired / USART1 IRQ disabled |
| Only STATUS frames | filter never started: `statusDiagnosis` (IMU init, no INT1, no mag from hub, board moving during calibration) |
| HardFault ~2 s after reset | unaligned float write into a packed telemetry struct ([Telemetry protocol](../firmware/telemetry.md)) |
| Attitude "converges slowly" in the viewer | viewer lagging: check the *lag* field; disable REPLAY or use `tlmCapture` |
| Yaw moves when tilting | wrong `MAG_AXIS_MAP` / hard-iron → [Magnetometer calibration](../tools/mag-calibration.md) |
| `iekfAttitude` not found | run `initPath.m` (in `02 - Tools/00 - Matlab`) |
| Build errors with paths that no longer exist, after moving or re-cloning the project | stale makefiles in the local `Debug/` folder: delete it (or *Project → Clean*), re-import `00 - FC_FW` and build again |
| Comparison: "INIT of this boot was lost" | UART saturated during the capture; repeat |
| Euler plots jump by 180° | gimbal lock (|pitch| ≈ 90°); use the quaternion figure |
| `acwing-tools info`: IMU interrupts 0/s, gyro far below 960/s, tens of I²C errors/s; viewer below its rate with a growing `lag` and `lost 0` | INT1 stuck high: it is a *level* (FIFO ≥ watermark) and the EXTI is on the *rising edge*. One late read leaves the FIFO above the watermark, so no edge arrives any more and the task lives on its 5 ms timeout, with big reads, DMA timeouts and lost samples. Fixed in `ImuTask`: immediate re-read after a large read or a timeout wake-up, and a 12 ms DMA timeout. `s_drainReads` in *Live Expressions* counts the recoveries |
| `lag` grows slowly (about 1 %/s) with no loss; `acwing-tools info`: filter time behind the uptime | the gyro was integrated with the nominal 1/960 s for part of the samples, while the LSM6DSV16X runs on its own RC oscillator (916 Hz measured). Fixed: the sample period is measured over 2 s windows against the MCU clock and used for every sample. `s_odrHz` in *Live Expressions* shows it |
| `undefined reference to arm_mat_*` | CMSIS-DSP GCC library not linked ([Building and flashing](../firmware/build.md)) |
