/**
 * @file  rc_ibus.h
 * @brief FlySky i-BUS (servo output of FS-iA6B, FS-iA10B, ...): 115200 8N1. 32-byte frame: 0x20 0x40,
 *        14 channels u16 little-endian in us, checksum u16 = 0xFFFF - sum of the first 30 bytes.
 *        No failsafe flag: the receiver sends its failsafe values (configure them on the transmitter).
 */
#ifndef RC_IBUS_H
#define RC_IBUS_H

#include "rc_frame.h"

#define IBUS_BAUD   115200u
#define IBUS_FRAME  32u

typedef struct { uint8_t buf[IBUS_FRAME]; uint8_t n; } rc_ibus_t;

void    rc_ibus_init(rc_ibus_t *p);
uint8_t rc_ibus_feed(rc_ibus_t *p, uint8_t b, rc_frame_t *frame);

#endif /* RC_IBUS_H */
