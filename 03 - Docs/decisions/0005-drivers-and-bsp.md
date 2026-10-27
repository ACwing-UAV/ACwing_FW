# 0005. Portable `drivers/` separate from board-specific `bsp/`

- **Status:** Accepted
- **Date:** 2026-10-27 (renamed from `platform/` to `bsp/` the same day)

## Context

The firmware runs today on the NUCLEO-WB55 + IKS4A1 bench, with the IMU on
I²C. Next it runs on the ACwing FC rev A, with the IMU on SPI and I²C1 used
only for the expansion bus. A single "drivers" folder was proposed.

## Decision

Two folders, with a clear boundary:

| Folder | Content | Depends on |
|---|---|---|
| `App/drivers/` | chip drivers: LSM6DSV16X (with the sensor hub), VL53L1X, flow bridge | only the `sensor_bus_t` / `vl53_bus_t` function pointers |
| `App/bsp/` | board support: HAL I²C + DMA binding and bus lock, Qwiic binding, BLE GATT service, flash pages shared with CPU2 | STM32 HAL, FreeRTOS, STM32_WPAN |

Task logic that used to sit in the binding moved to `app/`. For example, the
ExtTask probing and polling is now `app/ext_sens.c`, and its HAL binding is
`bsp/ext_i2c.c`.

## Consequences

- Everything in `drivers/` builds in the host tests. Nothing in `bsp/` does,
  and that is visible from the folder alone.
- The port to the ACwing FC rev A changes `bsp/` (and the CubeMX project),
  not the drivers.
- Cost: one more folder, and a binding function per bus.

## Alternatives considered

- **One flat `drivers/` folder.** Simpler to browse, but it hides which code
  can be tested without the board.
- **`drivers/chips` + `drivers/stm32`.** The same boundary under one folder.
  It was judged equivalent, and `bsp` is the usual embedded name for the
  second part.
