/**
 * @file  tlm_uart.c
 * @brief Frame transmission via UART + DMA, synchronised with a CMSIS-RTOS2 semaphore.
 */
#include "tlm_uart.h"
#include "cmsis_os2.h"
#include <string.h>

#define TLM_UART_BUF 256u

static UART_HandleTypeDef *s_huart;
static osSemaphoreId_t     s_txFree;
static uint8_t             s_dmaBuf[TLM_UART_BUF];   /* must stay valid for the whole DMA    */
static volatile uint32_t   s_dropped;

int tlm_uart_init(UART_HandleTypeDef *huart)
{
    s_huart  = huart;
    s_txFree = osSemaphoreNew(1, 1, NULL);
    return s_txFree ? 0 : -1;
}

int tlm_uart_send(const uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
    if (!s_huart || len > TLM_UART_BUF) return -1;
    if (osSemaphoreAcquire(s_txFree, timeout_ms) != osOK) { s_dropped++; return -1; }
    memcpy(s_dmaBuf, buf, len);
    if (HAL_UART_Transmit_DMA(s_huart, s_dmaBuf, len) != HAL_OK) {
        osSemaphoreRelease(s_txFree);
        s_dropped++;
        return -1;
    }
    return 0;
}

void tlm_uart_tx_cplt_isr(UART_HandleTypeDef *huart)
{
    if (huart == s_huart) (void)osSemaphoreRelease(s_txFree);   /* ISR-safe in CMSIS-RTOS2 */
}

uint32_t tlm_uart_dropped(void) { return s_dropped; }
