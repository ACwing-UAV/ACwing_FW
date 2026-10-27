/**
 * @file  app_main.h
 * @brief Integration into main.c (CubeMX, CMSIS-RTOS2):
 *
 *   osKernelInitialize();
 *   app_start();                 // <-- add this
 *   osKernelStart();
 *
 *   void HAL_GPIO_EXTI_Callback(uint16_t pin)          { app_exti(pin); }
 *   void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *h){ app_i2c_rx_cplt(h); }
 *   void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *h)    { app_i2c_error(h); }
 *   void HAL_UART_TxCpltCallback(UART_HandleTypeDef *h) { app_uart_tx_cplt(h); }
 */
#ifndef APP_MAIN_H
#define APP_MAIN_H

#include "main.h"

void app_start(void);
void app_exti(uint16_t pin);
void app_i2c_rx_cplt(I2C_HandleTypeDef *hi2c);
void app_i2c_error(I2C_HandleTypeDef *hi2c);
void app_uart_tx_cplt(UART_HandleTypeDef *huart);

#endif /* APP_MAIN_H */
