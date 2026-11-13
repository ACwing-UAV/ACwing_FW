/**
 * @file  rc_uart.c
 * @brief Receiver UART (see rc_uart.h). HAL receive-to-idle in interrupt mode: a CRSF frame (26 bytes at 420 kbaud,
 *        every 2-4 ms) ends with an idle line, so each callback usually carries one whole frame.
 */
#include "app_config.h"
#if APP_RC_UART && !defined(HOST_TEST)
#include "rc_uart.h"
#include "main.h"
#include "spsc.h"
#include <string.h>

extern UART_HandleTypeDef huart1;

#define RING   512u
#define CHUNK  64u

static uint8_t           s_ring[RING];
static volatile uint32_t s_head, s_tail, s_err;
static uint8_t           s_chunk[CHUNK];
static volatile uint8_t  s_open;

static void restart_rx(void)
{
    if (s_open) (void)HAL_UARTEx_ReceiveToIdle_IT(&huart1, s_chunk, CHUNK);
}

int rc_uart_open(const rc_uart_cfg_t *cfg)
{
    s_open = 0u;
    (void)HAL_UART_AbortReceive(&huart1);
    (void)HAL_UART_DeInit(&huart1);
    if (!cfg->baud) return 0;
    huart1.Instance = USART1;
    huart1.Init.BaudRate = cfg->baud;
    huart1.Init.WordLength = cfg->parity_even ? UART_WORDLENGTH_9B : UART_WORDLENGTH_8B;   /* 8 data + parity */
    huart1.Init.Parity = cfg->parity_even ? UART_PARITY_EVEN : UART_PARITY_NONE;
    huart1.Init.StopBits = cfg->stop2 ? UART_STOPBITS_2 : UART_STOPBITS_1;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart1.AdvancedInit.AdvFeatureInit = cfg->inverted ? UART_ADVFEATURE_RXINVERT_INIT : UART_ADVFEATURE_NO_INIT;
    huart1.AdvancedInit.RxPinLevelInvert = cfg->inverted ? UART_ADVFEATURE_RXINV_ENABLE : UART_ADVFEATURE_RXINV_DISABLE;
    if (HAL_UART_Init(&huart1) != HAL_OK) return -1;
    s_head = s_tail = 0u;
    s_open = 1u;
    restart_rx();
    return 0;
}

uint32_t rc_uart_read(uint8_t *dst, uint32_t cap)
{
    uint32_t n = 0;
    const uint32_t h = s_head;
    SPSC_BARRIER();
    while (s_tail != h && n < cap) { dst[n++] = s_ring[s_tail % RING]; s_tail++; }
    return n;
}

uint32_t rc_uart_errors(void) { return s_err; }

/* ---- HAL callbacks (USE_HAL_UART_REGISTER_CALLBACKS = 0): USART1 belongs to the receiver in this build ---- */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *h, uint16_t size)
{
    if (h != &huart1) return;
    uint32_t hd = s_head;
    for (uint16_t i = 0; i < size; ++i) {
        if (hd - s_tail >= RING) { s_err++; break; }          /* RcTask late: drop, the parser resyncs */
        s_ring[hd % RING] = s_chunk[i];
        hd++;
    }
    SPSC_BARRIER();
    s_head = hd;
    restart_rx();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *h)
{
    if (h != &huart1) return;
    s_err++;
    restart_rx();
}
#endif
