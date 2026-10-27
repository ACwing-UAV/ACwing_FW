/**
 * @file  app_link.h
 * @brief Receive side of the ACwing Pilot link (HANDOVER_FW.md §2 and §6).
 *
 * The BLE CMD write callback (HCI user-event thread of the stack) calls
 * appl_on_cmd_write() and NOTHING else happens in that context:
 *   - the arrival time is stored from a monotonic hardware tick (one aligned
 *     32-bit store: atomic on Cortex-M, no lock);
 *   - RC (0x11) values are stored behind a sequence counter (seqlock):
 *     the reader in the flight core never waits, it retries or keeps the
 *     previous values;
 *   - HEARTBEAT (0x10) only refreshes the time;
 *   - every other v2 opcode (>= 0x10) is copied (<= 20 bytes) into a
 *     single-producer single-consumer queue drained by the AppTask.
 * Legacy 0x01/0x02 are still handled inline by tlm_ble_on_command().
 *
 * Only CMD frames refresh the app-link time: connection events, CCCD writes,
 * notifications sent or link-layer keep-alives do not (they never call this).
 * A GAP disconnection only marks the link down, which can make the failsafe
 * start earlier, never later (appl_silence_ms()).
 *
 * Host-testable: no HAL, no RTOS.
 */
#ifndef APP_LINK_H
#define APP_LINK_H

#include <stdint.h>

#define APPL_CMD_MAX      20u
#define APPL_QUEUE_LEN    16u      /**< power of 2, >= 8 required by the handover */
typedef struct {
    uint8_t len;
    uint8_t b[APPL_CMD_MAX];
} appl_cmd_t;

typedef struct {
    int16_t thr, yaw, pitch, roll;   /**< -1000..1000: thr -1000 = stick bottom */
    uint8_t buttons;
} appl_rc_t;

void     appl_reset(void);

/* ---- BLE context (one producer) ---- */
void     appl_on_cmd_write(const uint8_t *data, uint16_t len, uint32_t now_ms);
void     appl_on_connect(void);
void     appl_on_disconnect(void);

/* ---- AppTask (one consumer) ---- */
int      appl_pop(appl_cmd_t *out);            /**< 1 if a command was taken */
uint32_t appl_queue_dropped(void);

/* ---- flight core (any reader, never blocks) ---- */
uint8_t  appl_ever(void);                      /**< 1 once any CMD frame arrived */
uint32_t appl_last_ms(void);                   /**< time of the last CMD frame   */
uint8_t  appl_link_down(void);                 /**< GAP disconnected since then  */
/** Silence seen by the failsafe: now - last CMD frame, at least 'down_floor_ms'
 * while the link is reported down; UINT32_MAX if nothing ever arrived. */
uint32_t appl_silence_ms(uint32_t now_ms, uint32_t down_floor_ms);
/** Latest RC values and their arrival time. Returns 1 with a consistent copy,
 * 0 if no RC ever arrived (out untouched). */
int      appl_rc_read(appl_rc_t *out, uint32_t *t_ms);

#endif /* APP_LINK_H */
