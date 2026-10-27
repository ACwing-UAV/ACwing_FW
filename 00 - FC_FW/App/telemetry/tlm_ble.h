/**
 * @file  tlm_ble.h
 * @brief BLE telemetry, transport-independent part (phase 2 + protocol v2).
 *
 * The BLE link carries the same frames as the UART (A5 5A | TYPE | LEN |
 * PAYLOAD | CRC16), as a byte stream: frames are queued here and sent as
 * notifications of up to (ATT_MTU - 3) bytes, regardless of the frame
 * boundaries. The receiver concatenates the notifications and re-synchronises
 * on A5 5A + CRC. Any MTU works (23 .. 247), no fragmentation protocol.
 *
 * Two queues, each with ONE producer (lock-free single-producer
 * single-consumer rings, whole frames only):
 *   HI  AppTask        protocol v2 frames: ACK, EVENT, CALIB, PARAM, FLIGHT,
 *                      CHECKS, NAV, MOTORS, SYSTEM, CAPS, and INIT on subscribe
 *   LO  TelemetryTask  STATE (decimated), STATUS, INIT (~1 Hz)
 * The BLE TX thread is the single consumer of both. It switches queue only at
 * a frame boundary (a frame split over notifications is completed first), and
 * always prefers HI.
 *
 * TX priority when the link is short (HANDOVER_FW.md §2.3): every frame type
 * may only fill its queue up to a level, so the low-priority frames are the
 * first to be refused:
 *   ACK = EVENT = CAPS (100 %) > CALIB = PARAM = INIT (85 %) > FLIGHT = CHECKS (70 %)
 *   > POWER = NAV (55 %) > MOTORS = SYSTEM (45 %);   LO: STATUS 85 % > STATE 50 %.
 * Refused frames are counted in STATUS.ble_drop.
 *
 * On notification enable (CCCD) the AppTask sends CAPS, INIT, CHECKS in this
 * order, then calls tlm_ble_open_streams(): until then the LO queue accepts
 * nothing, so STATE/STATUS never precede CAPS.
 *
 * Host-testable: no HAL, no STM32_WPAN, no RTOS here.
 */
#ifndef TLM_BLE_H
#define TLM_BLE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TLM_BLE_RING_SIZE   2048u    /* per queue, power of 2                */
#define TLM_BLE_MTU_DEFAULT 23u      /* ATT_MTU before any exchange           */

typedef enum {
    TLM_BLE_OFF       = 0,   /* BLE not compiled in or stack not started       */
    TLM_BLE_READY     = 1,   /* service registered, advertising                */
    TLM_BLE_CONNECTED = 2,   /* central connected, notifications not enabled   */
    TLM_BLE_STREAMING = 3    /* notifications enabled: data is flowing         */
} tlm_ble_state_t;

/* Legacy commands written on the CMD characteristic (phase 2, unchanged) */
#define TLM_BLE_CMD_SEND_INIT   0x01u   /* [0x01]          resend INIT now           */
#define TLM_BLE_CMD_STATE_DECIM 0x02u   /* [0x02, N]       STATE every N acc updates */

void     tlm_ble_reset(void);

/* ---- producers ---- */
/* Queue a whole frame or nothing. 0 = queued, 1 = no client streaming (or
 * streams not open yet, LO only), -1 = refused (priority level or full, counted). */
int      tlm_ble_push(const uint8_t *frame, uint16_t len);      /* LO: TelemetryTask */
int      tlm_ble_push_hi(const uint8_t *frame, uint16_t len);   /* HI: AppTask        */
uint16_t tlm_ble_hi_free(void);                                  /* bytes free in HI   */
/* 1 if a STATE frame with this sequence number must go over BLE (decimation) */
int      tlm_ble_want_state(uint16_t seq);
/* 1 once after the legacy [0x01] command: send INIT now (TelemetryTask) */
int      tlm_ble_take_init_request(void);
/* 1 once after a client enabled notifications (AppTask: CAPS, INIT, CHECKS) */
int      tlm_ble_take_subscribe(void);
void     tlm_ble_open_streams(void);

/* ---- consumer side (BLE TX thread) ---- */
/* Copies up to max bytes of the next notification into dst without removing
 * them; tlm_ble_consume(n) with the same n removes exactly those bytes. */
uint16_t tlm_ble_peek(uint8_t *dst, uint16_t max);
void     tlm_ble_consume(uint16_t n);
void     tlm_ble_discard(void);      /* drop everything queued (consumer side) */
uint16_t tlm_ble_queued(void);

/* ---- link events (BLE stack glue) ---- */
void     tlm_ble_set_state(tlm_ble_state_t s);
void     tlm_ble_set_mtu(uint16_t att_mtu);
void     tlm_ble_set_error(uint8_t ble_status);   /* last non-recoverable stack error */
void     tlm_ble_on_command(const uint8_t *data, uint16_t len);  /* legacy 0x01/0x02 */

/* ---- diagnostics (STATUS frame) ---- */
tlm_ble_state_t tlm_ble_state(void);
uint16_t tlm_ble_mtu(void);
uint16_t tlm_ble_chunk_max(void);   /* ATT_MTU - 3 */
uint32_t tlm_ble_dropped(void);
uint8_t  tlm_ble_error(void);
uint8_t  tlm_ble_state_decim(void);

/* Bench test of HANDOVER_FW.md §6 / §16.12 (debugger Live Expression):
 * 1 = the BLE path stops on purpose (CMD writes ignored, TX thread idle).
 * The app-link failsafe must still fire at 1 s and 3 s. */
extern volatile uint8_t g_dbg_ble_stall;

#ifdef __cplusplus
}
#endif
#endif /* TLM_BLE_H */
