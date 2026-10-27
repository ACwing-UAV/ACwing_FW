# Building and flashing


1. The folder `00 - FC_FW` is an STM32CubeIDE project: *File → Import →
   General → Existing Projects into Workspace*, select `00 - FC_FW`. Build
   outputs (`Debug/`) are not versioned: the first build regenerates them. To change
   pins or clocks, open `ACwing_FC_FW.ioc` in CubeMX and regenerate (user code is
   kept). Relevant settings already in the `.ioc`: SYSCLK 64 MHz; I2C1 on PB8/PB9 with
   DMA RX and event/error IRQs; USART1 921600 baud with DMA TX (**Memory increment
   enabled**) and global IRQ; PA15 EXTI rising edge (`IMU_INT1`); FreeRTOS
   CMSIS_V2, 1 kHz tick, 52 KB heap; HAL timebase on a TIM; Serial Wire debug;
   STM32_WPAN (BLE) with HSEM, IPCC, RTC, RNG, RF ([CubeMX configuration](ble.md#cubemx-configuration)).
   Interrupts that call RTOS functions need priority ≥ 5.
2. Include paths (all configurations): `App/app`, `App/filters/iekf`, `App/telemetry`,
   `App/bsp`, `App/control`, `App/drivers/common`,
   `App/drivers/imu_lsm6dsv16x`, `App/third_party/st_lsm6dsv16x`,
   `App/third_party/cmsis_dsp/Include`.
3. Linker: library `arm_cortexM4lf_math` from `App/third_party/cmsis_dsp/Lib`
   (the **GCC** `.a`, not the Keil `.lib`). FPU: FPv4-SP-D16, hard ABI.
4. `main.c` already calls `app_start()` and forwards the HAL callbacks
   (`HAL_GPIO_EXTI_Callback`, `HAL_I2C_MemRxCpltCallback`,
   `HAL_I2C_ErrorCallback`, `HAL_UART_TxCpltCallback`).
5. Debug/flash: `ACwing_FC_FW.launch` (ST-LINK GDB server, SWD, connect under reset).
6. BLE: the wireless stack must be flashed once on the Cortex-M0+ ([Wireless stack on the Cortex-M0+](ble.md#wireless-stack-on-the-cortex-m0)). Without
   it, the radio core never starts. Set `APP_TLM_BLE 0` to run on USB only.

Bring-up check in *Live Expressions*: `s_irqCount` grows by ~480/s at 960 Hz,
`s_busErr` stays 0, `s_est.running` becomes 1 about 2 s after reset with the board
still. `s_procCycMax / 64` = worst-case processing time in µs.

All parameters live in `App/app/app_config.h`. The switches that matter for the
procedures below:

| Macro | Normal use | Comparison with MATLAB |
|---|---|---|
| `APP_TLM_REPLAY` | 0 | 1 |
| `APP_COMPARE_ORIGINAL` | 0 | 1 (base-equation test vs original) or 0 (extended-filter test) |
| `MAG_AXIS_MAP`, `MAG_HARD_IRON` | from [Magnetometer calibration](../tools/mag-calibration.md) | unchanged |
| `APP_TLM_BLE` | 1 (BLE + USB) or 0 (USB only) | unchanged (REPLAY is USB-only) |
| `APP_BLE_STATE_DECIM` | 4 (STATE at 60 Hz over BLE) | — |
