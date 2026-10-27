/**
 * @file  imu_i2c.h
 * @brief STM32 HAL (STM32WB55) binding of the sensor_bus_t interface + DMA burst
 *        read of the FIFO with task notification.
 *
 * In IKS4A1 Mode 3 the LIS2MDL and the LPS22DF sit behind the LSM6DSV16X sensor
 * hub. The expansion devices (ACwing Flow: VL53L1X 0x29, flow bridge 0x3B) share
 * the bus on the bench, so every transaction takes a mutex with priority
 * inheritance (imu_i2c_lock). On the ACwing FC rev A the IMU is on SPI and I2C1
 * is the expansion bus only.
 */
#ifndef IMU_I2C_H
#define IMU_I2C_H

#include "main.h"
#include "sensor_bus.h"
#include "cmsis_os2.h"

/** Creates the bus lock: call once before starting the tasks that use the bus */
void    imu_i2c_bus_init(I2C_HandleTypeDef *hi2c);
int     imu_i2c_lock(void);       /**< 0 = taken (or no kernel yet), -1 = timeout */
void    imu_i2c_unlock(void);
I2C_HandleTypeDef *imu_i2c_handle(void);
void    imu_i2c_make_bus(sensor_bus_t *bus, I2C_HandleTypeDef *hi2c, uint8_t addr7);
/** Starts the DMA read and blocks the calling task until completion.
 * 0 = OK, -1 error/timeout. */
int32_t imu_i2c_read_dma(uint8_t reg, uint8_t *buf, uint16_t len, uint32_t timeout_ms);
/** To be called from HAL_I2C_MemRxCpltCallback / HAL_I2C_ErrorCallback */
void    imu_i2c_rx_cplt_isr(I2C_HandleTypeDef *hi2c);
void    imu_i2c_error_isr(I2C_HandleTypeDef *hi2c);

#endif /* IMU_I2C_H */
