# ACwing Flow bridge firmware (STM32C011F4U6)

Firmware for the **flow bridge** of the ACwing Flow rev A board. It reads the
PixArt PMW3901 optical-flow sensor over SPI and serves the accumulated motion
to the ACwing FC as an I²C target at **0x3B** on the Qwiic bus. The VL53L1X ToF
on the same board is read by the FC directly, at 0x29.

The full description is on the documentation site: pins, register map, frame
reading, LED codes, tests and the first-board checklist are on the
[ACwing Flow bridge](../03%20-%20Docs/firmware/flow-bridge.md) page.

## Quick start

```
make                 # build/ACwing_Flow_FW.elf / .bin / .hex (arm-none-eabi-gcc)
make flash           # STM32_Programmer_CLI over SWD: pads TP1 SWDIO, TP2 SWCLK, TP3 NRST, TP4 3.3 V, TP5 GND
make test            # host tests (register map vs the FC driver, I²C protocol, PMW3901)
```

| Folder | Content |
|---|---|
| `Core/Inc/flow_regs.h` | I²C register map, shared with the FC driver `flow_bridge.h` |
| `Core/Src/main.c` | clocks, pins, SPI1, I2C1 target interrupt, main loop, IWDG, LED |
| `Core/Src/bridge.c` | register file and I²C target protocol (portable, host-tested) |
| `Core/Src/pmw3901.c` | PMW3901 driver (portable, host-tested) |
| `Core/Startup`, `Drivers/CMSIS` | ST and Arm CMSIS files (Apache-2.0) |
| `Tests/host` | host tests |
