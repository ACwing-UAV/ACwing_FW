/**
 * @file  rc_input.h
 * @brief RC receiver input, protocol-independent: parser selection, channel map, normalisation, link state.
 *
 * Bytes from the receiver UART (RcTask) -> the parser of the selected protocol (drivers/rc) -> sticks in the same
 * scale as the app's RC command (-1000..1000; throttle -1000 = stick bottom), AUX channels in us, link statistics.
 * The state is published with a sequence counter: any task reads a consistent copy with rc_input_read().
 *
 * Protocols (parameter rcProtocol) and the receivers that use them:
 *   1 CRSF  ExpressLRS (any brand), TBS Crossfire / Tracer                       420 kbaud 8N1
 *   2 SBUS  FrSky (ACCST/ACCESS), Futaba (FASST/FHSS), RadioMaster/Jumper/FlySky SBUS receivers   100 kbaud 8E2 inverted
 *   3 IBUS  FlySky i-BUS (FS-iA6B, FS-iA10B, FS-X6B)                              115200 8N1
 *   4 DSM   Spektrum DSM2 / DSMX serial satellites                                115200 8N1
 *
 * Host-testable: no HAL, no RTOS.
 */
#ifndef RC_INPUT_H
#define RC_INPUT_H

#include <stdint.h>
#include "rc_frame.h"

enum { RC_PROTO_NONE = 0, RC_PROTO_CRSF = 1, RC_PROTO_SBUS = 2, RC_PROTO_IBUS = 3, RC_PROTO_DSM = 4, RC_PROTO_COUNT };
enum { RC_MAP_AETR = 0, RC_MAP_TAER = 1 };

#define RC_STALE_MS   250u      /**< no valid frame this long: the link is lost (same as the app RC, HANDOVER §6) */
#define RC_N_AUX      4u

/** serial settings of a protocol (bsp/rc_uart.c) */
typedef struct { uint32_t baud; uint8_t parity_even, stop2, inverted; } rc_uart_cfg_t;

/** STICKS calibration (us), order throttle, yaw, pitch, roll; invalid (zeros) = 1000 / 1500 / 2000 */
typedef struct { uint16_t centre[4], min[4], max[4]; } rc_cal_t;

typedef struct {
    uint8_t   ever;             /**< a receiver has sent valid frames since power-on            */
    uint8_t   valid;            /**< frames fresh and the receiver not in failsafe               */
    int16_t   thr, yaw, pitch, roll;   /**< -1000..1000, app scale (pitch + forward, yaw + right) */
    uint16_t  aux_us[RC_N_AUX]; /**< AUX1..AUX4                                                  */
    uint16_t  stick_us[4];      /**< raw sticks after the channel map: thr, yaw, pitch, roll      */
    rc_link_t link;             /**< lq / rssi / snr / tx power (CRSF), lq from SBUS lost frames  */
    uint32_t  t_ms;             /**< last valid frame                                            */
    uint32_t  frames;           /**< valid frames since power-on                                 */
} rc_state_t;

void    rc_input_init(void);
/** protocol and channel map (parameters); a change restarts the parsers. Returns 1 when the protocol changed. */
uint8_t rc_input_configure(uint8_t protocol, uint8_t map);
uint8_t rc_input_protocol(void);
void    rc_input_uart_cfg(uint8_t protocol, rc_uart_cfg_t *out);
/** RcTask: received bytes; now_ms for the DSM frame gaps and the staleness */
void    rc_input_feed(const uint8_t *b, uint32_t n, uint32_t now_ms);
/** RcTask, periodically: staleness and packet rate */
void    rc_input_service(uint32_t now_ms);
/** any task: a consistent copy; returns 0 before the first frame */
uint8_t rc_input_read(rc_state_t *out);

/** STICKS calibration from another task (AppTask): taken over by the RcTask at its next frame.
 *  An invalid one (centre not between min and max with 150 us on each side) restores the defaults. */
void    rc_input_set_cal(const rc_cal_t *cal);
uint8_t rc_cal_valid(const rc_cal_t *cal);

/** normalisation helpers (us -> -1000..1000), default 1000 / 1500 / 2000 */
int16_t rc_stick_from_us(uint16_t us);
int16_t rc_throttle_from_us(uint16_t us);
/** with a calibration: stick around its centre, throttle over its range */
int16_t rc_stick_cal(uint16_t us, uint16_t mn, uint16_t centre, uint16_t mx);
int16_t rc_throttle_cal(uint16_t us, uint16_t mn, uint16_t mx);

#endif /* RC_INPUT_H */
