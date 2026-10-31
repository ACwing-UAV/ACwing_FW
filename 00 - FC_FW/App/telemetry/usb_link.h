/**
 * @file  usb_link.h
 * @brief Byte-stream link over USB CDC: protocol v2 frames in both
 *        directions (`A5 5A | TYPE | LEN | PAYLOAD | CRC16`).
 *
 * - RX: the USB interrupt hands the received bytes to usbl_rx_isr() (SPSC
 *   byte ring); the consumer task calls usbl_poll(), which resynchronises on
 *   `A5 5A`, checks the CRC and returns the payload of every CMD frame
 *   (type 0x20: `op, seq, args`, the same bytes as a BLE CMD write).
 * - TX: the consumer task queues whole frames with usbl_send(); the board
 *   binding drains the ring with usbl_tx_peek()/usbl_tx_consume() when the
 *   USB endpoint is free.
 *
 * Transport-agnostic and host-testable: the same code would carry a UART.
 */
#ifndef USB_LINK_H
#define USB_LINK_H

#include <stdint.h>
#include <stddef.h>

#define USBL_RX_SIZE   1024u
#define USBL_TX_SIZE   4096u
#define USBL_CMD_MAX   32u

void     usbl_reset(void);

/* ---- USB interrupt ---- */
void     usbl_rx_isr(const uint8_t *p, uint32_t n);   /**< bytes that do not fit are dropped */

/* ---- consumer task ---- */
/** next CMD frame payload; returns its length (≥ 2), 0 if none */
uint32_t usbl_poll(uint8_t *cmd, uint32_t cap);
uint32_t usbl_tx_free(void);
int      usbl_send(uint8_t type, const void *payload, uint8_t len);   /**< 0 = queued whole */
uint32_t usbl_rx_errors(void);                       /**< frames with a bad CRC */

/* ---- board binding (same task as the producer) ---- */
uint32_t usbl_tx_peek(uint8_t *buf, uint32_t cap);
void     usbl_tx_consume(uint32_t n);

#endif /* USB_LINK_H */
