/**
 * @file  tlm_uart.h
 * @brief USB transport: UART -> ST-LINK Virtual COM Port (USART1 on the NUCLEO-WB55).
 */
#ifndef TLM_UART_H
#define TLM_UART_H

#include <stdint.h>
#include "main.h"      /* UART_HandleTypeDef (CubeMX) */

int  tlm_uart_init(UART_HandleTypeDef *huart);
/** Non-blocking beyond timeout_ms: if the previous DMA has not finished, the frame is dropped */
int  tlm_uart_send(const uint8_t *buf, uint16_t len, uint32_t timeout_ms);
/** To be called from HAL_UART_TxCpltCallback() (see 03 - Docs/firmware/build.md) */
void tlm_uart_tx_cplt_isr(UART_HandleTypeDef *huart);
uint32_t tlm_uart_dropped(void);

#endif /* TLM_UART_H */
