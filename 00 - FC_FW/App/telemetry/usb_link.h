/**
 * @file  usb_link.h
 * @brief Byte-stream link over USB CDC: protocol v2 frames in both
 *        directions (`A5 5A | TYPE | LEN | PAYLOAD | CRC16`).
 *
 * The USB link is a **ground link**: telemetry, parameters, calibrations and
 * the flight log. It never flies the drone (ADR 0009): the AppTask refuses the
 * flight commands that arrive this way, and the flight core refuses ARM,
 * TAKEOFF and MOTOR_TEST while a USB host is connected.
 *
 * - RX: the USB interrupt hands the received bytes to usbl_rx_isr() (SPSC
 *   byte ring); the LogTask calls usbl_poll(), which resynchronises on
 *   `A5 5A`, checks the CRC and returns the payload of every CMD frame
 *   (type 0x20: `op, seq, args`, the same bytes as a BLE CMD write).
 * - TX: one SPSC ring per producing task (LogTask, AppTask, TelemetryTask),
 *   each holding whole frames. The board binding (called by the LogTask)
 *   drains them with usbl_tx_peek()/usbl_tx_consume(), one ring at a time and
 *   only at frame boundaries, so frames of different tasks never interleave.
 * - Host state: the binding reports whether a host has configured the device
 *   and whether it holds DTR (port open); usbl_open() adds "a command arrived
 *   in the last USBL_OPEN_MS", for hosts that do not set DTR.
 *
 * Transport-agnostic and host-testable: the same code would carry a UART.
 */
#ifndef USB_LINK_H
#define USB_LINK_H

#include <stdint.h>
#include <stddef.h>

#define USBL_RX_SIZE   1024u
#define USBL_TX_SIZE   4096u    /**< per producer */
#define USBL_CMD_MAX   32u
#define USBL_OPEN_MS   3000u    /**< without DTR, the link stays open this long after a command */

/** producers of the TX rings */
enum { USBL_SRC_LOG = 0, USBL_SRC_APP = 1, USBL_SRC_TLM = 2, USBL_SRC_COUNT };

void     usbl_reset(void);

/* ---- USB interrupt / board binding ---- */
void     usbl_rx_isr(const uint8_t *p, uint32_t n);   /**< bytes that do not fit are dropped */
/** host state: configured by a host, DTR set (any context, single writer each) */
void     usbl_set_configured(uint8_t on);
void     usbl_set_dtr(uint8_t on);

/* ---- any task ---- */
/** a USB host has configured the device (cable to a PC): ARM is refused */
uint8_t  usbl_host(void);
/** a program has the port open: host and (DTR or a command in the last USBL_OPEN_MS) */
uint8_t  usbl_open(uint32_t now_ms);

/* ---- LogTask (RX consumer) ---- */
/** next CMD frame payload; returns its length (>= 2), 0 if none. Records the arrival time. */
uint32_t usbl_poll(uint8_t *cmd, uint32_t cap, uint32_t now_ms);
uint32_t usbl_rx_errors(void);                       /**< frames with a bad CRC */

/* ---- producers (one task per source) ---- */
uint32_t usbl_tx_free_src(uint8_t src);
/** a whole v2 frame built from type and payload; 0 = queued whole */
int      usbl_send_src(uint8_t src, uint8_t type, const void *payload, uint8_t len);
/** a frame already encoded; 0 = queued whole */
int      usbl_put_frame(uint8_t src, const uint8_t *frame, uint32_t n);
/** the LogTask's ring (log answers) */
static inline uint32_t usbl_tx_free(void) { return usbl_tx_free_src(USBL_SRC_LOG); }
static inline int usbl_send(uint8_t type, const void *payload, uint8_t len) { return usbl_send_src(USBL_SRC_LOG, type, payload, len); }
uint32_t usbl_tx_dropped(uint8_t src);               /**< frames that did not fit */

/* ---- board binding (one consumer, the LogTask) ---- */
uint32_t usbl_tx_peek(uint8_t *buf, uint32_t cap);
void     usbl_tx_consume(uint32_t n);
/** nobody listening: empties every ring */
void     usbl_tx_flush(void);

#endif /* USB_LINK_H */
