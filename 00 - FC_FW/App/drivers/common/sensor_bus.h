/**
 * @file  sensor_bus.h
 * @brief Platform-independent bus interface.
 *
 * Sensor drivers include NEITHER HAL nor RTOS: they only receive these
 * function pointers. The same library therefore runs on STM32 (HAL binding,
 * see bsp/imu_i2c.c), on another MCU, or on a PC with a simulated bus.
 */
#ifndef SENSOR_BUS_H
#define SENSOR_BUS_H

#include <stdint.h>

/** All functions return 0 on success, != 0 on error. */
typedef int32_t (*bus_write_fn)(void *handle, uint8_t reg, const uint8_t *data, uint16_t len);
typedef int32_t (*bus_read_fn) (void *handle, uint8_t reg, uint8_t *data, uint16_t len);
typedef void    (*bus_delay_fn)(uint32_t ms);

typedef struct {
    bus_write_fn write;
    bus_read_fn  read;
    bus_delay_fn delay_ms;   /**< optional (used only in init/reset)  */
    void        *handle;     /**< opaque context passed to read/write  */
} sensor_bus_t;

#endif /* SENSOR_BUS_H */
