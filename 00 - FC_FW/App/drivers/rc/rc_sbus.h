/**
 * @file  rc_sbus.h
 * @brief SBUS (FrSky, Futaba, RadioMaster / Jumper receivers, ...): 100 kbaud 8E2, **inverted** signal (the UART
 *        inverts RX in hardware). 25-byte frame: 0x0F, 16 x 11-bit channels, flags (bit 2 frame lost,
 *        bit 3 failsafe), end byte 0x00 (SBUS2: 0x04/0x14/0x24/0x34).
 */
#ifndef RC_SBUS_H
#define RC_SBUS_H

#include "rc_frame.h"

#define SBUS_BAUD   100000u
#define SBUS_FRAME  25u

typedef struct { uint8_t buf[SBUS_FRAME]; uint8_t n; } rc_sbus_t;

void    rc_sbus_init(rc_sbus_t *p);
uint8_t rc_sbus_feed(rc_sbus_t *p, uint8_t b, rc_frame_t *frame);

#endif /* RC_SBUS_H */
