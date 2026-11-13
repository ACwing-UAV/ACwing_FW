/**
 * @file  rc_frame.h
 * @brief One decoded frame of an RC receiver, whatever the protocol (drivers/rc).
 *
 * Channels in microseconds (1000..2000 nominal, 1500 centre), in the order the receiver sends them; the channel map
 * (AETR / TAER) and the calibration are applied by control/rc_input. Link statistics are filled only by the
 * protocols that carry them (CRSF); 0xFF / -128 / 0 mean "not known".
 */
#ifndef RC_FRAME_H
#define RC_FRAME_H

#include <stdint.h>

#define RC_MAX_CH 16u

typedef struct {
    uint16_t us[RC_MAX_CH];
    uint8_t  n;              /**< channels in this frame                            */
    uint8_t  failsafe;       /**< the receiver reports a lost link (SBUS flag)       */
    uint8_t  frame_lost;     /**< the receiver missed a frame from the transmitter   */
} rc_frame_t;

typedef struct {
    uint8_t  lq;             /**< uplink link quality %, 0xFF = n/a                  */
    int8_t   rssi;           /**< dBm, -128 = n/a                                    */
    int8_t   snr;            /**< dB, -128 = n/a                                     */
    uint16_t tx_power_mw;    /**< transmitter power, 0 = n/a                          */
    uint16_t pkt_rate_hz;    /**< 0 = n/a                                             */
} rc_link_t;

/** what a parser reports after a byte */
enum { RC_NONE = 0, RC_GOT_CHANNELS = 1, RC_GOT_LINK = 2 };

#endif /* RC_FRAME_H */
