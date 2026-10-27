/**
 * @file  ext_i2c.h
 * @brief HAL binding of the expansion (Qwiic) devices on I2C1.
 *
 * Builds the bus descriptors of the portable drivers (vl53l1x, flow_bridge)
 * on top of the STM32 HAL. Every transaction takes the I2C lock of imu_i2c.c
 * (priority inheritance): on the bench the IMU shares the bus; on the ACwing
 * FC rev A the IMU is on SPI and the lock is uncontended.
 */
#ifndef EXT_I2C_H
#define EXT_I2C_H

#include "vl53l1x.h"
#include "sensor_bus.h"

void ext_i2c_tof_bus(vl53_bus_t *bus, uint8_t addr7);      /* 16-bit registers */
void ext_i2c_flow_bus(sensor_bus_t *bus, uint8_t addr7);   /* 8-bit registers  */

#endif /* EXT_I2C_H */
