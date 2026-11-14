/**
 * @file  rc_crsf.h
 * @brief CRSF (TBS Crossfire / Tracer, ExpressLRS): 420 kbaud 8N1, full duplex.
 *
 * Frame: [addr][len][type][payload...][crc8], len = bytes after it (type + payload + crc), crc8 DVB-S2 (poly 0xD5)
 * over type and payload. Used here: 0x16 RC_CHANNELS_PACKED (16 x 11 bit, 172..1811 = 988..2012 us) and
 * 0x14 LINK_STATISTICS (uplink RSSI, LQ, SNR, transmitter power).
 */
#ifndef RC_CRSF_H
#define RC_CRSF_H

#include "rc_frame.h"

#define CRSF_BAUD          420000u
#define CRSF_ADDR_FC       0xC8u
#define CRSF_T_LINK_STATS  0x14u
#define CRSF_T_RC_CHANNELS 0x16u
#define CRSF_MAX_FRAME     64u

typedef struct {
    uint8_t buf[CRSF_MAX_FRAME];
    uint8_t n;
} rc_crsf_t;

void    rc_crsf_init(rc_crsf_t *p);
/** feeds one byte; returns RC_GOT_CHANNELS (frame filled) or RC_GOT_LINK (link filled) or RC_NONE */
uint8_t rc_crsf_feed(rc_crsf_t *p, uint8_t b, rc_frame_t *frame, rc_link_t *link);
uint8_t rc_crsf_crc8(const uint8_t *d, uint8_t n);
/** 11-bit packed channels (CRSF and SBUS) -> us */
void    rc_unpack11(const uint8_t *src, uint8_t n_ch, uint16_t *us);

#endif /* RC_CRSF_H */
