# 0006. A bridge MCU for the PMW3901, firmware on registers

- **Status:** Accepted
- **Date:** 2026-10-17 (hardware), 2026-10-27 (firmware)

## Context

The PMW3901 optical-flow sensor speaks SPI with strict timing (tSRAD,
tSWW…). The ACwing Flow board sits under the battery and reaches the FC
through a 4-wire Qwiic cable (I²C), shared with the GPS board. The FC has no
spare SPI lines toward the cable.

## Decision

- An **STM32C011F4U6** on the Flow board reads the PMW3901 over SPI. Every
  10 ms it reads a motion burst, rejects the samples without surface
  features, and accumulates the motion together with the time it covers.
- It serves the result as an **I²C target at 0x3B**, with a register map
  shared with the FC driver. The FRAME read latches the values and clears
  them.
- The VL53L1X ToF stays **directly on the bus** (0x29). The bridge never
  drives its XSHUT line.
- The bridge firmware is bare-metal on registers with CMSIS only, built with
  a Makefile (`01 - FLOW_FW`). The register file and the PMW3901 driver are
  portable and host-tested together with the FC driver.

Details: [ACwing Flow bridge](../firmware/flow-bridge.md),
[Sensors and expansion boards](../firmware/sensors.md).

## Consequences

- Two wires to the FC, chainable with the GPS board, and the SPI timing stays
  local to the board.
- The accumulation and the time stamp in the bridge mean the FC can read at
  any rate without losing motion.
- 16 KB of flash is plenty: the firmware uses about 3.5 kB.
- Cost: a second firmware to program over SWD, and a register map to keep in
  sync. A host test fails if the FC and the bridge disagree.

## Alternatives considered

- **PMW3901 on an SPI line of the FC.** It needs extra wires and a chip select
  on the cable, and it puts SPI timing across a connector.
- **A flow module with a UART protocol.** The ACwing FC has no free UART pair
  (see the FC pin map).
- **CubeMX + HAL for the bridge.** It works, but it is heavier for 16 KB, and
  it adds nothing for three peripherals.
