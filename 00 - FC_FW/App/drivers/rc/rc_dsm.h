/**
 * @file  rc_dsm.h
 * @brief Spektrum DSM2 / DSMX serial (satellite / remote receivers): 115200 8N1, 16-byte frames every 11 or 22 ms,
 *        sync by the gap between frames. Byte 0 fades, byte 1 system (0x01 DSM2 1024/22 ms; 0x12 DSM2 2048/11 ms;
 *        0xA2, 0xB2 DSMX 2048), then 7 words big-endian: 2048 mode id = bits 14..11, value = bits 10..0;
 *        1024 mode id = bits 13..10, value = bits 9..0. A frame carries up to 7 channels; channels accumulate.
 */
#ifndef RC_DSM_H
#define RC_DSM_H

#include "rc_frame.h"

#define DSM_BAUD     115200u
#define DSM_FRAME    16u
#define DSM_GAP_MS   3u          /**< a pause this long starts a new frame */
#define DSM_CH       12u

typedef struct {
    uint8_t  buf[DSM_FRAME];
    uint8_t  n;
    uint32_t last_ms;
    uint8_t  have_last;
    uint16_t us[DSM_CH];        /**< channels accumulate over the frames */
} rc_dsm_t;

void    rc_dsm_init(rc_dsm_t *p);
uint8_t rc_dsm_feed(rc_dsm_t *p, uint8_t b, uint32_t now_ms, rc_frame_t *frame);

#endif /* RC_DSM_H */
