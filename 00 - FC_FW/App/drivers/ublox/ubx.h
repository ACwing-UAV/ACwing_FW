/**
 * @file  ubx.h
 * @brief u-blox UBX protocol: byte-stream parser, frame builder, NAV-PVT decoder, CFG-VALSET builder.
 *
 * Frame: 0xB5 0x62 | class | id | length u16 LE | payload | CK_A CK_B
 * (8-bit Fletcher over class..payload). Pure C, no HAL, no RTOS: host-tested.
 *
 * Only the messages the firmware uses are decoded: NAV-PVT (0x01 0x07, 92 B), ACK-ACK / ACK-NAK (0x05 0x01 / 0x00).
 * A longer payload than UBX_MAX_PAYLOAD is skipped (checksum still consumed, counted in `skipped`).
 */
#ifndef UBX_H
#define UBX_H

#include <stdint.h>

#define UBX_SYNC1            0xB5u
#define UBX_SYNC2            0x62u
#define UBX_MAX_PAYLOAD      100u

#define UBX_CLS_NAV          0x01u
#define UBX_ID_NAV_PVT       0x07u
#define UBX_LEN_NAV_PVT      92u
#define UBX_CLS_ACK          0x05u
#define UBX_ID_ACK_ACK       0x01u
#define UBX_ID_ACK_NAK       0x00u
#define UBX_CLS_CFG          0x06u
#define UBX_ID_CFG_VALSET    0x8Au

/** CFG-VALSET layers */
#define UBX_LAYER_RAM        0x01u

/** configuration keys used (u-blox M10 interface description); the size is in bits 28..30 of the key */
#define UBX_KEY_I2COUTPROT_UBX     0x10720001u   /**< L : UBX output on I2C (DDC)          */
#define UBX_KEY_I2COUTPROT_NMEA    0x10720002u   /**< L : NMEA output on I2C               */
#define UBX_KEY_MSGOUT_NAV_PVT_I2C 0x20910006u   /**< U1: NAV-PVT every n solutions on I2C */
#define UBX_KEY_RATE_MEAS          0x30210001u   /**< U2: measurement period, ms           */
#define UBX_KEY_NAVSPG_DYNMODEL    0x20110021u   /**< E1: dynamic platform model           */
#define UBX_DYN_AIRBORNE_4G        8u

/** NAV-PVT fixType */
enum { UBX_FIX_NONE = 0, UBX_FIX_DR = 1, UBX_FIX_2D = 2, UBX_FIX_3D = 3, UBX_FIX_GNSS_DR = 4, UBX_FIX_TIME = 5 };

typedef struct {
    uint8_t  state;
    uint8_t  cls, id;
    uint16_t len, n;
    uint8_t  ck_a, ck_b;
    uint8_t  buf[UBX_MAX_PAYLOAD];
    uint32_t bad_ck, skipped;     /**< counters: checksum errors, payloads too long */
} ubx_parser_t;

void ubx_parser_reset(ubx_parser_t *p);
/** feeds one byte; 1 when a complete message with a valid checksum is in p->cls / id / len / buf */
int  ubx_parse_byte(ubx_parser_t *p, uint8_t b);

void     ubx_checksum(const uint8_t *d, uint16_t n, uint8_t *ck_a, uint8_t *ck_b);
/** whole frame into out; returns its length, 0 if it does not fit */
uint16_t ubx_frame(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len, uint8_t *out, uint16_t cap);

typedef struct { uint32_t key; uint32_t value; } ubx_kv_t;
/** CFG-VALSET frame (version 0, the given layers) for n keys; returns its length, 0 if it does not fit */
uint16_t ubx_valset(uint8_t layers, const ubx_kv_t *kv, uint8_t n, uint8_t *out, uint16_t cap);

/** NAV-PVT, the fields used (units of the message) */
typedef struct {
    uint32_t itow_ms;
    uint16_t year;
    uint8_t  month, day, hour, min, sec;
    uint8_t  valid;               /**< bit0 date, bit1 time, bit2 fully resolved                 */
    uint8_t  fix_type;            /**< UBX_FIX_*                                                 */
    uint8_t  flags;               /**< bit0 gnssFixOK                                            */
    uint8_t  num_sv;
    int32_t  lon_e7, lat_e7;
    int32_t  h_ellips_mm, h_msl_mm;
    uint32_t h_acc_mm, v_acc_mm;
    int32_t  vel_ned_mms[3];
    int32_t  gspeed_mms;
    int32_t  head_mot_e5;         /**< heading of motion, 1e-5 deg                               */
    uint32_t s_acc_mms;
    uint16_t pdop_x100;
    uint8_t  invalid_llh;         /**< flags3 bit0: lat / lon / height not valid                 */
} ubx_pvt_t;

/** 0 on success, -1 on a wrong length */
int ubx_decode_pvt(const uint8_t *payload, uint16_t len, ubx_pvt_t *pvt);

/** the solution can be used for navigation: 3D (or GNSS + DR) fix, gnssFixOK, position valid */
uint8_t ubx_pvt_usable(const ubx_pvt_t *pvt);

#endif /* UBX_H */
