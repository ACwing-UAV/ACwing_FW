/**
 * @file  imu_i2c.h
 * @brief STM32 HAL (STM32WB55) binding of the sensor_bus_t interface + DMA burst
 *        read of the FIFO with task notification.
 *
 * In IKS4A1 Mode 3 the MCU I2C bus only hosts the LSM6DSV16X (the LIS2MDL sits
 * behind the sensor hub) and a single task uses it: no mutex needed.
 */
#ifndef IMU_I2C_H
#define IMU_I2C_H

#include "main.h"
#include "sensor_bus.h"
#include "cmsis_os2.h"

void    imu_i2c_make_bus(sensor_bus_t *bus, I2C_HandleTypeDef *hi2c, uint8_t addr7);
/* Starts the DMA read and blocks the calling task until completion.
 * 0 = OK, -1 error/timeout. */
int32_t imu_i2c_read_dma(uint8_t reg, uint8_t *buf, uint16_t len, uint32_t timeout_ms);
/* To be called from HAL_I2C_MemRxCpltCallback / HAL_I2C_ErrorCallback */
void    imu_i2c_rx_cplt_isr(I2C_HandleTypeDef *hi2c);
void    imu_i2c_error_isr(I2C_HandleTypeDef *hi2c);

#endif /* IMU_I2C_H */
