/**
 * @file  tlm_ble.h
 * @brief BLE telemetry, transport-independent part (phase 2).
 *
 * The BLE link carries EXACTLY the same frames as the UART (A5 5A | TYPE | LEN |
 * PAYLOAD | CRC16), as a byte stream: the frames are queued here and sent as
 * notifications of up to (ATT_MTU - 3) bytes, regardless of the frame
 * boundaries. The receiver concatenates the notifications and re-synchronises
 * on A5 5A + CRC, with the same parser used for the serial port
 * (parseTelemetry.m). Any MTU therefore works (23 by default, 185 on iOS, up
 * to 247 on Windows/macOS), with no fragmentation protocol.
 *
 * What goes over BLE (TelemetryTask decides, see app_main.c):
 *   STATE  decimated to APP_BLE_STATE_DECIM (default 240/4 = 60 Hz)
 *   STATUS 10 Hz, INIT ~1 Hz and immediately when a client subscribes
 *   REPLAY never (44 kB/s: USB only)
 *
 * Threads: one producer (TelemetryTask: tlm_ble_push) and one consumer (BLE TX
 * thread in ble_tlm_svc.c: peek/consume). Lock-free single-producer
 * single-consumer ring: no mutex, ImuTask is never involved.
 *
 * Host-testable: no HAL, no STM32_WPAN, no RTOS here.
 */
#ifndef TLM_BLE_H
#define TLM_BLE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TLM_BLE_RING_SIZE   2048u    /* power of 2; ~380 ms at ~5.4 kB/s     */
#define TLM_BLE_MTU_DEFAULT 23u      /* ATT_MTU before any exchange           */

typedef enum {
    TLM_BLE_OFF       = 0,   /* BLE not compiled in or stack not started       */
    TLM_BLE_READY     = 1,   /* service registered, advertising                */
    TLM_BLE_CONNECTED = 2,   /* central connected, notifications not enabled   */
    TLM_BLE_STREAMING = 3    /* notifications enabled: data is flowing         */
} tlm_ble_state_t;

/* Commands written by the client on the CMD characteristic (iPhone app, MATLAB) */
#define TLM_BLE_CMD_SEND_INIT   0x01u   /* [0x01]          resend INIT now           */
#define TLM_BLE_CMD_STATE_DECIM 0x02u   /* [0x02, N]       STATE every N acc updates */

void     tlm_ble_reset(void);

/* ---- producer side (TelemetryTask) ---- */
/* Queues a whole frame or nothing. Returns 0 if queued, 1 if discarded because
 * no client is streaming (not an error), -1 if the ring is full (counted). */
int      tlm_ble_push(const uint8_t *frame, uint16_t len);
/* 1 if a STATE frame with this sequence number must go over BLE (decimation) */
int      tlm_ble_want_state(uint16_t seq);
/* 1 once after a client subscribed or asked for it: send INIT now */
int      tlm_ble_take_init_request(void);

/* ---- consumer side (BLE TX thread) ---- */
/* Copies up to max queued bytes into dst without removing them */
uint16_t tlm_ble_peek(uint8_t *dst, uint16_t max);
void     tlm_ble_consume(uint16_t n);
/* Drops everything queued (consumer-side operation, safe with the producer) */
void     tlm_ble_discard(void);
uint16_t tlm_ble_queued(void);

/* ---- link events (BLE stack glue) ---- */
void     tlm_ble_set_state(tlm_ble_state_t s);
void     tlm_ble_set_mtu(uint16_t att_mtu);
void     tlm_ble_set_error(uint8_t ble_status);   /* last non-recoverable stack error */
void     tlm_ble_on_command(const uint8_t *data, uint16_t len);

/* ---- diagnostics (STATUS frame) ---- */
tlm_ble_state_t tlm_ble_state(void);
uint16_t tlm_ble_mtu(void);
uint16_t tlm_ble_chunk_max(void);   /* ATT_MTU - 3 */
uint32_t tlm_ble_dropped(void);
uint8_t  tlm_ble_error(void);
uint8_t  tlm_ble_state_decim(void);

#ifdef __cplusplus
}
#endif
#endif /* TLM_BLE_H */
