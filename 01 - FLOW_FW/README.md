# ACwing Flow bridge firmware (STM32C011F4U6)

Firmware for the **flow bridge** of the ACwing Flow rev A board. It reads the
PixArt PMW3901 optical-flow sensor over SPI and serves the accumulated motion
to the ACwing FC as an I²C target at **0x3B** on the Qwiic bus.

The VL53L1X ToF on the same board does **not** go through the bridge: the FC
reads it directly at 0x29. The bridge never drives the ToF XSHUT line, so the
ToF works whether the bridge is programmed or not.

```
PMW3901 ──SPI1 mode 3, 1.5 MHz──► STM32C011 ──I²C1 target 0x3B──► Qwiic ──► ACwing FC (flow_bridge.c)
VL53L1X ──────────────────────────────────────I²C 0x29──────────► Qwiic ──► ACwing FC (vl53l1x.c)
```

## Pins

Checked against the board schematic and against ST's open pin data for the
STM32C011F4U6 (UFQFPN-20).

| Pin | Port | Net | Use |
|---|---|---|---|
| 5 | PA0 | TOF_XSHUT | left in its reset state (high impedance); the 10 k pull-up keeps the ToF on |
| 6 | PA1 | TOF_INT | not used (the FC polls the ToF) |
| 7 | PA2 | PMW_NRESET | output, high = running |
| 8 | PA3 | PMW_MOTION | not used (the motion burst is polled) |
| 9 | PA4 | PMW_NCS | output, high = deselected |
| 10/11/12 | PA5/PA6/PA7 | SPI_SCK/MISO/MOSI | SPI1, AF0 |
| 13 | PA8 | LED_K | output, low = LED on |
| 16/17 | PA13/PA14 | SWDIO/SWCLK | SWD, test pads TP1/TP2 |
| 18/19 | PB6/PB7 | I2C_SCL/SDA | I2C1 target, AF6, open drain (pull-ups on the FC) |

## What it does

- **Start-up.** HSI48 runs the core at 48 MHz with 1 wait state. The
  PMW3901 then gets a hardware reset, an SPI reset and a power-up reset. The
  firmware checks the product id (0x49 / 0xB6) and writes the vendor
  "performance optimisation" register sequence. If the sensor does not
  answer, the bridge retries every second and reports STATUS.bit0 = 0.
- **Every 10 ms: motion burst** (register 0x16, 12 bytes). A sample with
  SQUAL < 0x19 and the shutter at its maximum has no surface features. The
  firmware rejects it and sets STATUS.bit3. Every other sample adds its
  ΔX/ΔY, and the time since the previous burst, to the accumulators.
- **I²C register map** (`Core/Inc/flow_regs.h`). The FC driver
  `00 - FC_FW/App/drivers/flow_bridge/flow_bridge.h` uses the same map:

  | addr | name | content |
  |---|---|---|
  | 0x00 | WHO_AM_I | 0xF1 |
  | 0x01 | VERSION | 0x10 (1.0) |
  | 0x02 | STATUS | bit0 sensor ok, bit1 new data, bit2 overflow, bit3 last sample rejected |
  | 0x03 | COUNT | samples accumulated (u8, wraps) |
  | 0x04 / 0x06 | DX / DY | i16 counts since the last frame |
  | 0x08 | SQUAL | last surface quality |
  | 0x09 | SHUTTER | u16, last shutter |
  | 0x0B | DT | u16, time covered by DX/DY, 100 µs units |
  | 0x10 | CTRL (W) | bit0 reset the bridge, bit1 re-initialise the PMW3901 |

  **FRAME** is a read of 11 bytes starting at 0x02, in one transaction. It
  latches the values and clears DX, DY and DT. The fraction of DT below
  100 µs carries over, so no time is lost. DX/DY saturate at ±32767 and set
  bit2.
- **Concurrency.** The I²C interrupt serves the bytes and latches the frame.
  The main loop adds samples with interrupts masked for a few
  microseconds.
- **Watchdog.** The IWDG resets the bridge after about 0.5 s. Only the main
  loop feeds it, including during the long initialisation delays. It is
  frozen while the debugger halts the core.
- **LED** (green, D1):

  | Pattern | Meaning |
  |---|---|
  | on | sensor OK and the FC reading frames |
  | 1 Hz blink | sensor OK, no reads from the FC |
  | short flash every 2 s | no surface features (too dark, or too close: the PMW3901 needs ≥ 80 mm) |
  | 5 Hz blink | PMW3901 not found |

Flash use is about 3.5 kB of 16 kB, RAM about 1.1 kB of 6 kB.

## Build and flash

Requirement: `arm-none-eabi-gcc`. The one shipped with STM32CubeIDE works:
add its `bin` folder to `PATH`, or pass `PREFIX=<path>/arm-none-eabi-`.

```
make                 # build/ACwing_Flow_FW.elf / .bin / .hex
make flash           # STM32_Programmer_CLI over SWD (connect under reset)
make test            # host tests
```

**SWD connection** on the board's test pads: TP1 SWDIO, TP2 SWCLK, TP3 NRST,
TP4 3.3 V, TP5 GND, from an ST-LINK (for example the one on the NUCLEO-WB55,
with its CN4 jumpers removed). The board can be powered from the Qwiic cable.
PA14 is also BOOT0. The SWCLK pull-down keeps it low, so the chip boots from
flash after the first programming.

**In STM32CubeIDE:**
1. *File → New → Makefile Project with Existing Code*, choose this folder and
   the *MCU ARM GCC* toolchain.
2. For debugging, create an *STM32 C/C++ Application* configuration on
   `build/ACwing_Flow_FW.elf` with ST-LINK (SWD) and the device STM32C011F4Ux.

## Tests

`Tests/host` (run by `make test`) has 22 checks, built with AddressSanitizer
and UBSan:
- the bridge map agrees with the FC driver;
- the FC driver's `flow_init` and `flow_read` work over a simulated I²C bus
  that calls the same functions as the interrupt handler;
- latch and clear, carried-over time, saturation, rejected samples, CTRL,
  read-only registers;
- the PMW3901 id check, the initialisation sequence (74 writes) and the burst
  decoding, against a simulated sensor.

## On the first board

1. Program the bridge with `make flash` and check the LED: a 1 Hz blink means
   the sensor was found and the FC is not reading yet.
2. Connect the board to the FC. CAPS must show `FLOW` and `TOF`, and the
   LED turns steady on.
3. Hold the board at least 10 cm above a textured, well-lit floor and move it
   by hand. NAV `flowQ` should go above 50, and `vel` should follow the
   motion.
4. Check the axis signs (`APP_FLOW_SIGN_X/Y` in the FC `app_config.h`) by
   moving the drone forward and to the left. The FLOW_TOF calibration will
   check them later as well.

## Licences

- `Core/Src/pmw3901.c`: register sequences from the Bitcraze PMW3901 driver
  (MIT; notice in the file).
- `Core/Startup`, `Core/Src/system_stm32c0xx.c` and
  `Drivers/CMSIS/Device/ST`: STMicroelectronics CMSIS device files
  (Apache-2.0).
- `Drivers/CMSIS/Include`: Arm CMSIS-Core (Apache-2.0).
