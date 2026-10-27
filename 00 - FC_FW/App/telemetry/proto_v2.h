/**
 * @file  proto_v2.h
 * @brief ACwing Pilot app protocol v2: identifiers and frame layouts.
 *
 * Contract: 02 - Tools/02 - App/HANDOVER_FW.md. Executable reference (wins over
 * the text): 02 - Tools/02 - App/src/protocol/{ids,messages,commands}.ts.
 * Every number below is the same as in ids.ts; every packed struct has the
 * byte layout of the matching encodeX() in messages.ts (checked by
 * _Static_assert here and by the host test against the app decoders).
 *
 * v2 only ADDS frame types 0x05..0x0F and command opcodes >= 0x10: the phase-2
 * frames 0x01..0x04 and the legacy commands 0x01/0x02 are unchanged.
 *
 * Host-testable: no HAL, no RTOS.
 */
#ifndef PROTO_V2_H
#define PROTO_V2_H

#include <stdint.h>
#include <stddef.h>

#define PV2_PROTO_VERSION   2u
#define PV2_CMD_MAX_LEN     20u

/* ---------------- frame types, drone -> app ---------------- */
#define PV2_FT_STATE    0x01u
#define PV2_FT_INIT     0x02u
#define PV2_FT_STATUS   0x03u
#define PV2_FT_REPLAY   0x04u
#define PV2_FT_CAPS     0x05u
#define PV2_FT_FLIGHT   0x06u
#define PV2_FT_POWER    0x07u
#define PV2_FT_NAV      0x08u
#define PV2_FT_MOTORS   0x09u
#define PV2_FT_SYSTEM   0x0Au
#define PV2_FT_EVENT    0x0Bu
#define PV2_FT_PARAM    0x0Cu
#define PV2_FT_CALIB    0x0Du
#define PV2_FT_ACK      0x0Eu
#define PV2_FT_CHECKS   0x0Fu

/* ---------------- command opcodes, app -> drone ---------------- */
#define PV2_OP_SEND_INIT      0x01u   /* legacy [0x01]           */
#define PV2_OP_STATE_DECIM    0x02u   /* legacy [0x02, N]        */
#define PV2_OP_HEARTBEAT      0x10u
#define PV2_OP_RC             0x11u
#define PV2_OP_ARM            0x12u
#define PV2_OP_DISARM         0x13u
#define PV2_OP_TAKEOFF        0x14u
#define PV2_OP_LAND           0x15u
#define PV2_OP_RTH            0x16u
#define PV2_OP_HOLD           0x17u
#define PV2_OP_KILL           0x18u
#define PV2_OP_SET_MODE       0x19u
#define PV2_OP_PARAM_GET      0x20u
#define PV2_OP_PARAM_SET      0x21u
#define PV2_OP_PARAM_SAVE     0x22u
#define PV2_OP_PARAM_DEFAULTS 0x23u
#define PV2_OP_PARAM_LIST     0x24u
#define PV2_OP_CAL_START      0x30u
#define PV2_OP_CAL_CAPTURE    0x31u
#define PV2_OP_CAL_ABORT      0x32u
#define PV2_OP_CAL_SAVE       0x33u
#define PV2_OP_CAL_RESTORE    0x34u
#define PV2_OP_CAL_SET_REF    0x35u
#define PV2_OP_MOTOR_TEST     0x36u
#define PV2_OP_MOTOR_REVERSE  0x37u
#define PV2_OP_MOTOR_REMAP    0x38u
#define PV2_OP_STREAM_RATES   0x40u
#define PV2_OP_REQUEST        0x41u
#define PV2_OP_SET_NAME       0x42u

#define PV2_ARM_MAGIC0        0xA5u
#define PV2_ARM_MAGIC1        0x5Au
#define PV2_PROPS_OFF_MAGIC   0x50u   /* 'P' */
/* KILL magic: "KILL" = 4B 49 4C 4C */

/* ---------------- ACK ---------------- */
enum { PV2_ACK_OK = 0, PV2_ACK_REJECTED = 1, PV2_ACK_BUSY = 2, PV2_ACK_INVALID = 3,
       PV2_ACK_UNSUPPORTED = 4, PV2_ACK_IN_PROGRESS = 5 };

enum {
    PV2_R_NONE = 0, PV2_R_NOT_LANDED = 1, PV2_R_CHECKS_FAILING = 2, PV2_R_THROTTLE_NOT_LOW = 3,
    PV2_R_NO_GPS = 4, PV2_R_BATTERY_LOW = 5, PV2_R_NO_HOME = 6, PV2_R_RADIO_OVERRIDE = 7,
    PV2_R_ARMED = 8, PV2_R_PROPS_CONFIRM_MISSING = 9, PV2_R_NOT_ARMED = 10, PV2_R_CALIBRATING = 11,
    PV2_R_NOT_CALIBRATING = 12, PV2_R_NOT_STILL = 13, PV2_R_OUT_OF_RANGE = 14,
    PV2_R_UNKNOWN_PARAM = 15, PV2_R_SENSOR_MISSING = 16
};

/* ---------------- enums and bit masks ---------------- */
enum { PV2_MODE_ACRO = 0, PV2_MODE_ANGLE = 1, PV2_MODE_ALT_HOLD = 2, PV2_MODE_POS_HOLD = 3 };
enum { PV2_AUTO_NONE = 0, PV2_AUTO_TAKEOFF = 1, PV2_AUTO_LAND = 2, PV2_AUTO_RTH = 3,
       PV2_AUTO_HOLD = 4, PV2_AUTO_FAILSAFE = 5 };
enum { PV2_INPUT_NONE = 0, PV2_INPUT_APP = 1, PV2_INPUT_ELRS = 2 };

#define PV2_FS_APP_LINK   (1u << 0)
#define PV2_FS_RADIO      (1u << 1)
#define PV2_FS_BATTERY    (1u << 2)
#define PV2_FS_GEOFENCE   (1u << 3)
#define PV2_FS_GPS        (1u << 4)
#define PV2_FS_FLOW       (1u << 5)

#define PV2_FF_IN_AIR       (1u << 0)
#define PV2_FF_LANDED       (1u << 1)
#define PV2_FF_HOME_SET     (1u << 2)
#define PV2_FF_GEOFENCE_ON  (1u << 3)
#define PV2_FF_BEGINNER     (1u << 4)
#define PV2_FF_ARM_BLOCKED  (1u << 5)

#define PV2_S_IMU        (1u << 0)
#define PV2_S_BARO       (1u << 1)
#define PV2_S_MAG_FC     (1u << 2)
#define PV2_S_MAG_GPS    (1u << 3)
#define PV2_S_GNSS       (1u << 4)
#define PV2_S_FLOW       (1u << 5)
#define PV2_S_TOF        (1u << 6)
#define PV2_S_ESC_TLM    (1u << 7)
#define PV2_S_CURRENT    (1u << 8)
#define PV2_S_ELRS       (1u << 9)
#define PV2_S_LEDS       (1u << 10)
#define PV2_S_FLASH_LOG  (1u << 11)

enum { PV2_MODEL_INDOOR = 0, PV2_MODEL_OUTDOOR = 1 };
enum { PV2_FRAME_QUAD_X = 0, PV2_FRAME_HEXA_X = 1, PV2_FRAME_OCTO_X = 2 };
enum { PV2_BATT_OK = 0, PV2_BATT_LOW = 1, PV2_BATT_CRITICAL = 2, PV2_BATT_EMERGENCY = 3 };
enum { PV2_EV_INFO = 0, PV2_EV_WARN = 1, PV2_EV_ERROR = 2, PV2_EV_CRITICAL = 3 };

enum {
    PV2_E_ARMED = 1, PV2_E_DISARMED = 2, PV2_E_TAKEOFF_STARTED = 3, PV2_E_LANDED = 4,
    PV2_E_MODE_CHANGED = 5, PV2_E_RTH_STARTED = 6, PV2_E_HOME_SET = 7,
    PV2_E_TAKEOFF_ABORTED = 8,      /* firmware extension (WARN): no lift-off; the app shows the text */
    PV2_E_FAILSAFE_APP_LINK = 20, PV2_E_FAILSAFE_RADIO = 21, PV2_E_FAILSAFE_BATTERY = 22,
    PV2_E_GEOFENCE_BREACH = 23, PV2_E_GPS_LOST = 24, PV2_E_FLOW_LOST = 25,
    PV2_E_BATTERY_LOW = 30, PV2_E_BATTERY_CRITICAL = 31, PV2_E_BATTERY_EMERGENCY = 32,
    PV2_E_MAG_INTERFERENCE = 40, PV2_E_HIGH_VIBRATION = 41, PV2_E_MOTOR_HOT = 42,
    PV2_E_RADIO_OVERRIDE = 43, PV2_E_CALIB_DONE = 50, PV2_E_CALIB_FAILED = 51,
    PV2_E_PARAMS_SAVED = 60, PV2_E_MOTOR_KILLED = 70
};

enum { PV2_CAL_ACCEL = 1, PV2_CAL_LEVEL = 2, PV2_CAL_GYRO = 3, PV2_CAL_COMPASS = 4,
       PV2_CAL_FLOW_TOF = 5, PV2_CAL_BATTERY = 6, PV2_CAL_MOTORS = 7, PV2_CAL_STICKS = 8 };
enum { PV2_CS_IDLE = 0, PV2_CS_RUNNING = 1, PV2_CS_WAITING_USER = 2, PV2_CS_DONE = 3,
       PV2_CS_FAILED = 4, PV2_CS_SAVED = 5 };

enum { PV2_CHK_BATTERY = 1, PV2_CHK_IMU = 2, PV2_CHK_COMPASS = 3, PV2_CHK_GNSS = 4,
       PV2_CHK_HOME = 5, PV2_CHK_RADIO = 6, PV2_CHK_EKF = 7, PV2_CHK_FAILSAFE = 8,
       PV2_CHK_FLOW = 9, PV2_CHK_THROTTLE = 10, PV2_CHK_CALIBRATION = 11 };
enum { PV2_CK_OK = 0, PV2_CK_WARN = 1, PV2_CK_FAIL = 2, PV2_CK_NA = 3 };

/* ---------------- payload layouts (little-endian, packed) ----------------
 * As in telemetry.h: fill float fields through memcpy from aligned locals. */
#pragma pack(push, 1)
typedef struct {                  /* 0x05 CAPS */
    uint8_t  proto;
    uint8_t  fw[3];
    uint8_t  hw_rev;              /* ASCII letter */
    uint8_t  model;
    uint16_t sensors;
    uint8_t  n_motors;
    uint8_t  frame;
    uint8_t  cells;
    char     name[16];            /* zero padded */
    char     serial[12];          /* zero padded */
} pv2_caps_t;                     /* 39 */

typedef struct {                  /* 0x06 FLIGHT */
    uint8_t  arm, mode, autom, failsafe;
    uint16_t flags;
    uint8_t  input;
    uint16_t flight_time_s;
    int32_t  alt_rel_cm;
    int16_t  vz_cms;
    uint16_t ground_speed_cms;
    uint16_t heading_cdeg;
    uint16_t home_dist_m;         /* 0xFFFF n/a */
    uint16_t home_bearing_cdeg;
    uint16_t rth_eta_s;           /* 0xFFFF n/a */
} pv2_flight_t;                   /* 25 */

typedef struct {                  /* 0x07 POWER */
    uint16_t vbat_mv;
    int16_t  current_ca;
    uint16_t used_mah;
    uint8_t  cells;
    uint16_t cell_mv;
    uint8_t  pct;
    uint16_t remain_s;            /* 0xFFFF n/a */
    uint8_t  warn;
} pv2_power_t;                    /* 13 */

typedef struct {                  /* 0x08 NAV */
    uint8_t  fix;                 /* 0xFF = no GNSS hardware */
    uint8_t  sats;
    uint16_t hdop_x100;
    int32_t  lat_e7, lon_e7;
    int32_t  alt_msl_cm;
    int16_t  vel_ned_cms[3];
    uint16_t tof_mm;              /* 0xFFFF n/a */
    uint8_t  flow_q;              /* 0xFF n/a */
    uint32_t baro_pa;
    uint16_t wind_cms;            /* 0xFFFF n/a */
} pv2_nav_t;                      /* 31 */

typedef struct {                  /* 0x09 MOTORS, one entry (after n u8) */
    uint16_t rpm;                 /* 0xFFFF n/a */
    int8_t   temp_c;              /* -128 n/a */
    uint8_t  out_pct;
} pv2_motor_t;                    /* 4 */

typedef struct {                  /* 0x0A SYSTEM */
    uint8_t  lq;                  /* 0xFF no receiver / not linked */
    int8_t   rssi, snr;
    uint16_t pkt_rate_hz, tx_power_mw;
    uint8_t  cpu_pct;
    uint16_t loop_hz, jitter_us;
    uint8_t  log_pct;
    int16_t  imu_temp_cc;
    uint16_t vibration_mg;
    uint16_t acc_clip;
    uint16_t att_unc_cdeg;
    uint8_t  est_flags;           /* bit0 EKF ok, bit1 acc gated, bit2 mag gated, bit3 mag interference */
    uint8_t  mag_interf_pct;
} pv2_system_t;                   /* 23 */

typedef struct {                  /* 0x0B EVENT header, then text[len] */
    uint8_t  level;
    uint16_t code;
    int32_t  arg;
    uint8_t  len;
} pv2_event_hdr_t;                /* 8 */

typedef struct {                  /* 0x0C PARAM */
    uint16_t id;
    float    value;
    uint8_t  flags;               /* bit0 dirty (RAM != flash), bit1 read-only */
    uint16_t index, count;
} pv2_param_t;                    /* 11 */

typedef struct {                  /* 0x0D CALIB header */
    uint8_t type, state, step, progress, quality, reason;
} pv2_calib_hdr_t;                /* 6 */

typedef struct { uint8_t done_mask, current, still_pct; int16_t offset_mg[3]; uint16_t scale_e4[3]; uint16_t fit_err_x100; } pv2_cal_accel_t;   /* 17 */
typedef struct { int16_t roll_cdeg, pitch_cdeg; uint16_t remaining_ms; int16_t trim_roll_cdeg, trim_pitch_cdeg; } pv2_cal_level_t;              /* 10 */
typedef struct { uint16_t remaining_ms; int16_t bias_mdps[3]; uint16_t noise_mdps; int16_t temp_cc; } pv2_cal_gyro_t;                            /* 12 */
typedef struct { uint8_t coverage[5]; uint8_t interf_fc, interf_gps, quality_fc, quality_gps; } pv2_cal_compass_t;                               /* 9 */
typedef struct { uint16_t tof_mm, height_mm; uint8_t flow_q, match_pct, axes_done; uint16_t scale_x_e4, scale_y_e4; int16_t tof_offset_mm; } pv2_cal_flow_t; /* 13 */
typedef struct { uint16_t vbat_mv, scale_v_x10, scale_i_e4, used_mah; } pv2_cal_battery_t;                                                         /* 8 */
typedef struct { uint8_t motor, spinning, rev_mask, ok_mask; } pv2_cal_motors_t;                                                                   /* 4 */
typedef struct { uint16_t centre[4], min[4], max[4]; uint8_t aux_pos[4]; } pv2_cal_sticks_t;                                                      /* 28 */

typedef struct {                  /* 0x0E ACK */
    uint8_t op, seq, result, reason;
} pv2_ack_t;                      /* 4 */

typedef struct {                  /* 0x0F CHECKS item (after ready u8, n u8) */
    uint8_t id, state;
    int16_t value;
} pv2_check_t;                    /* 4 */
#pragma pack(pop)

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(pv2_caps_t)        == 39, "CAPS layout");
_Static_assert(sizeof(pv2_flight_t)      == 25, "FLIGHT layout");
_Static_assert(sizeof(pv2_power_t)       == 13, "POWER layout");
_Static_assert(sizeof(pv2_nav_t)         == 31, "NAV layout");
_Static_assert(sizeof(pv2_motor_t)       == 4,  "MOTORS entry layout");
_Static_assert(sizeof(pv2_system_t)      == 23, "SYSTEM layout");
_Static_assert(sizeof(pv2_event_hdr_t)   == 8,  "EVENT header layout");
_Static_assert(sizeof(pv2_param_t)       == 11, "PARAM layout");
_Static_assert(sizeof(pv2_calib_hdr_t)   == 6,  "CALIB header layout");
_Static_assert(sizeof(pv2_cal_accel_t)   == 17, "CALIB accel layout");
_Static_assert(sizeof(pv2_cal_level_t)   == 10, "CALIB level layout");
_Static_assert(sizeof(pv2_cal_gyro_t)    == 12, "CALIB gyro layout");
_Static_assert(sizeof(pv2_cal_compass_t) == 9,  "CALIB compass layout");
_Static_assert(sizeof(pv2_cal_flow_t)    == 13, "CALIB flow layout");
_Static_assert(sizeof(pv2_cal_battery_t) == 8,  "CALIB battery layout");
_Static_assert(sizeof(pv2_cal_motors_t)  == 4,  "CALIB motors layout");
_Static_assert(sizeof(pv2_cal_sticks_t)  == 28, "CALIB sticks layout");
_Static_assert(sizeof(pv2_ack_t)         == 4,  "ACK layout");
_Static_assert(sizeof(pv2_check_t)       == 4,  "CHECKS item layout");
#endif

#define PV2_EVENT_TEXT_MAX   40u
#define PV2_CHECKS_MAX       11u
#define PV2_MOTORS_MAX       8u
#define PV2_PAYLOAD_MAX      (2u + 4u * PV2_CHECKS_MAX)   /* largest v2 payload: 46 */

/* ---------------- encoders: payload -> complete frame (A5 5A ... CRC) ----------------
 * Each returns the frame length, 0 if cap is too small. */
size_t pv2_frame(uint8_t type, const void *payload, uint8_t len, uint8_t *out, size_t cap);
size_t pv2_event(uint8_t level, uint16_t code, int32_t arg, const char *text, uint8_t *out, size_t cap);
size_t pv2_ack(uint8_t op, uint8_t seq, uint8_t result, uint8_t reason, uint8_t *out, size_t cap);
size_t pv2_param(uint16_t id, float value, uint8_t flags, uint16_t index, uint16_t count, uint8_t *out, size_t cap);
size_t pv2_checks(uint8_t ready, const pv2_check_t *items, uint8_t n, uint8_t *out, size_t cap);
size_t pv2_motors(const pv2_motor_t *m, uint8_t n, uint8_t *out, size_t cap);
size_t pv2_calib(const pv2_calib_hdr_t *h, const void *data, uint8_t data_len, uint8_t *out, size_t cap);

/* little-endian readers for command payloads (unaligned-safe) */
static inline uint16_t pv2_rd_u16(const uint8_t *p) { return (uint16_t)(p[0] | ((uint16_t)p[1] << 8)); }
static inline int16_t  pv2_rd_i16(const uint8_t *p) { return (int16_t)pv2_rd_u16(p); }
float pv2_rd_f32(const uint8_t *p);

#endif /* PROTO_V2_H */
