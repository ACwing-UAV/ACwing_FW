# Building and flashing


1. The folder `00 - FC_FW` is an STM32CubeIDE project: *File → Import →
   General → Existing Projects into Workspace*, select `00 - FC_FW`. Build
   outputs (`Debug/`) are not versioned: the first build regenerates them. To change
   pins or clocks, open `ACwing_FC_FW.ioc` in CubeMX and regenerate (user code is
   kept). Relevant settings already in the `.ioc`: SYSCLK 64 MHz; I2C1 on PB8/PB9 with
   DMA RX and event/error IRQs; USART1 921600 baud with DMA TX (**Memory increment
   enabled**) and global IRQ; PA15 EXTI rising edge (`IMU_INT1`); FreeRTOS
   CMSIS_V2, 1 kHz tick, 52 KB heap; HAL timebase on a TIM; Serial Wire debug;
   STM32_WPAN (BLE) with HSEM, IPCC, RTC, RNG, RF ([CubeMX configuration](ble.md#cubemx-configuration));
   TIM2 CH1–CH4 PWM on PA0–PA3 for the motors (prescaler 0, period 1999: 32 kHz).
   Interrupts that call RTOS functions need priority ≥ 5.

   **Rules for a regeneration** (checked by `Tests/target/check_cubemx.py` in the CI):
   - **leave the USB peripheral disabled** (Connectivity → USB). USB belongs to `App/bsp/usb` (ST device library,
     HSI48 trimmed by the CRS, the CPU2 clock semaphore). PA11/PA12 stay as plain `USB_DM`/`USB_DP` signals.
     Enabling it generates a second `hpcd_USB_FS` and `HAL_PCD_MspInit`: the link fails. It also moves PLLSAI1 to
     48 MHz, and the ADC clock with it;
   - **HAL time base TIM17 at priority 15** (NVIC → `TIM1_TRG_COM_TIM17`): FreeRTOS and the IMU interrupt must
     preempt it;
   - TIM1 cannot replace TIM2 for the motors: its CH4 exists only on PA11 (USB_DM);
   - commit with LF line endings: `.gitattributes` converts them, and `git add --renormalize .` fixes a commit
     made before it.
2. Include paths: they are in `.cproject` (Debug and Release), among them `App/app`, `App/control`,
   `App/filters/iekf`, `App/filters/altitude_kf`, `App/filters/hover_est`, `App/telemetry`, `App/bsp`, `App/log`,
   the drivers and the third-party folders. A new source folder with headers must be added to **both**
   configurations: the CI job `firmware-target` builds the project from `.cproject` and fails otherwise.
   Locally, with `arm-none-eabi-gcc` installed:

   ```bash
   python3 "00 - FC_FW/Tests/target/build_fw.py" --config Debug
   ```

   It compiles every source folder of the configuration with the same paths and defines (at `-O2`), links with
   `STM32WB55RGVX_FLASH.ld` and prints the memory usage. It fails above 90 % of the static RAM or of the flash
   (`--max-ram-pct`, `--max-flash-pct`): the RAM left is the boot stack, which the interrupts use after the kernel
   starts, and the margin for growth. `--define "APP_HW_REV='A'"` builds the rev A flight board, whose flight log is
   on the W25Q flash instead of a 48 KB RAM buffer; the CI holds it to 80 % of the RAM.

   | Firmware 0.14 | Flash (504 KB) | RAM (192 KB) |
   |---|---|---|
   | rev A (flight) | 168 KB, 33.5 % | 114 KB, 59.5 % |
   | NUCLEO bench (log in RAM) | 159 KB, 31.6 % | 162 KB, 84.1 % |

   The static RAM includes the FreeRTOS heap (52 KB, task stacks and queues). What is really used of it, and the
   stack margin of each task, is in the SYSTEM frame (HANDOVER_FW §4, `heapMinFree`, `stackMinFree`). It does not replace the IDE build (optimisation and
   warning flags may differ), it catches missing paths, missing files and link errors.
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
