/**
 * @file  rc_uart.h
 * @brief Receiver UART (USART1 on the ACwing FC rev A, connector J5): reception into a lock-free ring.
 *
 * The ISR (HAL receive-to-idle) is the only producer, the RcTask the only consumer. Built only with APP_RC_UART.
 * Not run on hardware yet.
 */
#ifndef RC_UART_H
#define RC_UART_H

#include <stdint.h>
#include "rc_input.h"

/** (re)opens the port with the serial settings of a protocol; baud 0 closes it. 0 = ok */
int      rc_uart_open(const rc_uart_cfg_t *cfg);
/** received bytes since the last call (at most cap) */
uint32_t rc_uart_read(uint8_t *dst, uint32_t cap);
/** framing / noise / overrun errors since power-on (diagnostics) */
uint32_t rc_uart_errors(void);

#endif /* RC_UART_H */
