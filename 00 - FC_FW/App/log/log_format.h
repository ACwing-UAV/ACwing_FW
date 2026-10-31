/**
 * @file  log_format.h
 * @brief On-flash format of the ACwing flight log (format 1).
 *
 * The log flash (W25Q128JV, 16 MB, 4 KB sectors) is a **ring of sectors**.
 * A log (one per arming, or one per power-on with logMode = always) is a run
 * of consecutive sectors; when the flash is full the oldest sectors are
 * erased and reused, so the oldest log may lose its beginning ("truncated").
 *
 * Every sector starts with a 20-byte header, then records until the end of
 * the sector or the first 0xFF byte (erased flash). A record never crosses a
 * sector boundary:
 *
 *     [type u8][len u8][payload: len bytes][crc8 over type, len, payload]
 *
 * CRC-8: poly 0x07, init 0x00 (CRC-8/SMBUS). A reader that finds a bad CRC
 * skips to the next sector. All values little-endian, packed.
 *
 * The PC decoders (02 - Tools) follow this file: change both together and
 * bump LOG_FORMAT.
 */
#ifndef LOG_FORMAT_H
#define LOG_FORMAT_H

#include <stdint.h>

#define LOG_FORMAT       1u
#define LOG_MAGIC        0x474C4341u      /**< "ACLG" */
#define LOG_SECTOR       4096u
#define LOG_PAGE         256u
#define LOG_HDR_SIZE     20u
#define LOG_REC_OVERHEAD 3u               /**< type, len, crc8 */
#define LOG_REC_MAX      64u              /**< largest payload */

#pragma pack(push, 1)
/** sector header */
typedef struct {
    uint32_t magic;
    uint32_t serial;      /**< global sector write counter: the newest sector has the highest */
    uint16_t log_id;      /**< increments for every new log                    */
    uint16_t seq;         /**< sector index inside the log, 0 = first          */
    uint32_t t0_ms;       /**< uptime when the log started                     */
    uint8_t  format;      /**< LOG_FORMAT                                      */
    uint8_t  rsv;
    uint16_t crc;         /**< CRC-16/CCITT-FALSE of the 18 bytes above         */
} log_sector_hdr_t;

/* ---- record types ---- */
enum {
    LR_INFO   = 0x01,     /**< first record of a log                           */
    LR_ATT    = 0x10,     /**< attitude and IMU, 50 Hz                          */
    LR_VERT   = 0x11,     /**< vertical estimate and loop, 50 Hz                */
    LR_FLIGHT = 0x12,     /**< flight core state, 10 Hz                         */
    LR_EVENT  = 0x13,     /**< every EVENT of the flight core                   */
    LR_EST    = 0x14,     /**< estimator health, 10 Hz                          */
    LR_POWER  = 0x20,     /**< battery, 10 Hz (BattTask)                        */
    LR_DROP   = 0x7F      /**< records lost because a queue was full            */
};

typedef struct {          /**< LR_INFO */
    uint8_t  fw[3];
    char     hw_rev;
    uint8_t  format;
    uint32_t t0_ms;
    uint16_t sensors;     /**< CAPS bits                                       */
    char     name[16];
} log_info_t;             /**< 27 */

typedef struct {          /**< LR_ATT */
    uint32_t t_ms;
    int16_t  q[4];        /**< ×32767, body -> world NWU [w x y z]             */
    int16_t  gyro[3];     /**< mrad/s, body (calibrated)                        */
    int16_t  acc[3];      /**< mg, body (calibrated)                            */
} log_att_t;              /**< 24 */

typedef struct {          /**< LR_VERT */
    uint32_t t_ms;
    int32_t  h_mm;        /**< altitude estimate                                */
    int16_t  vz_mms;
    int32_t  h_sp_mm;     /**< height set point (when the altitude loop runs)   */
    int16_t  vz_sp_mms;
    uint8_t  thrust;      /**< collective ×255                                  */
    uint16_t tof_mm;      /**< 0xFFFF n/a                                       */
    uint8_t  flow_q;      /**< 0xFF n/a                                         */
    uint32_t baro_pa;     /**< 0 n/a                                            */
    uint8_t  flags;       /**< bit0 alt valid, 1 ToF valid, 2 ToF ground, 3 flow usable, 4 alt loop */
} log_vert_t;             /**< 25 */

typedef struct {          /**< LR_FLIGHT */
    uint32_t t_ms;
    uint8_t  armed, mode, autom, failsafe;
    uint8_t  flags;       /**< bit0 in air, 1 landed, 2 RC stale, 3 geofence    */
    int8_t   stick[4];    /**< ×100: thr, yaw, pitch, roll                      */
    uint16_t silence_ms;  /**< app link                                         */
} log_flight_t;           /**< 15 */

typedef struct {          /**< LR_EVENT */
    uint32_t t_ms;
    uint8_t  level;
    uint16_t code;
    int32_t  arg;
} log_event_t;            /**< 11 */

typedef struct {          /**< LR_EST */
    uint32_t t_ms;
    int16_t  gyro_bias[3];  /**< 0.1 mrad/s                                     */
    uint16_t att_unc_cdeg;
    uint16_t vib_mg;
    uint16_t loop_hz;
    uint16_t jitter_us;
    uint8_t  cpu_pct;
    uint8_t  mag_interf_pct;
} log_est_t;              /**< 20 */

typedef struct {          /**< LR_POWER */
    uint32_t t_ms;
    uint16_t vbat_mv;
    int16_t  current_ca;
    uint16_t used_mah;
    uint16_t cell_mv;     /**< sagged, filtered                                 */
    uint16_t cell_rest_mv;
    uint8_t  pct;
    uint8_t  level;       /**< PV2_BATT_*                                       */
    uint16_t r_cell_mohm;
    uint8_t  cells;       /**< bit7 = battery present                           */
} log_power_t;            /**< 19 */

typedef struct {          /**< LR_DROP */
    uint32_t t_ms;
    uint16_t lost;        /**< records dropped since the last LR_DROP          */
    uint8_t  source;      /**< 0 ImuTask, 1 BattTask                            */
} log_drop_t;             /**< 7 */
#pragma pack(pop)

_Static_assert(sizeof(log_sector_hdr_t) == LOG_HDR_SIZE, "sector header");
_Static_assert(sizeof(log_info_t)   == 27, "LR_INFO");
_Static_assert(sizeof(log_att_t)    == 24, "LR_ATT");
_Static_assert(sizeof(log_vert_t)   == 25, "LR_VERT");
_Static_assert(sizeof(log_flight_t) == 15, "LR_FLIGHT");
_Static_assert(sizeof(log_event_t)  == 11, "LR_EVENT");
_Static_assert(sizeof(log_est_t)    == 20, "LR_EST");
_Static_assert(sizeof(log_power_t)  == 19, "LR_POWER");
_Static_assert(sizeof(log_drop_t)   == 7,  "LR_DROP");

uint8_t  log_crc8(const uint8_t *p, uint32_t n);
uint16_t log_crc16(const uint8_t *p, uint32_t n);

#endif /* LOG_FORMAT_H */
