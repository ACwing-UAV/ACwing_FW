/**
 * @file  params.h
 * @brief Drone parameters of protocol v2 (HANDOVER_FW.md §8, ids.ts PARAMS).
 *
 * Ids are stable forever: add new ids, never reuse one. Values are float32 on
 * the wire and here. RAM values are written only by the AppTask (one 32-bit
 * store each, atomic on Cortex-M); the flight core reads them at any time
 * with param_get() without locks.
 *
 * Host-testable: no HAL, no RTOS.
 */
#ifndef PARAMS_H
#define PARAMS_H

#include <stdint.h>

/* indices in the table (fast access from the flight core) */
enum {
    P_TAKEOFF_ALT = 0, P_LAND_SPEED, P_AUTO_DISARM, P_MAX_TILT, P_MAX_YAW_RATE, P_MAX_CLIMB,
    P_MAX_HSPEED, P_BEGINNER,
    P_FS_APP_SHORT, P_FS_APP_LONG, P_FS_RADIO, P_FS_GPS_LOST, P_FS_FLOW_LOST,
    P_BATT_WARN, P_BATT_CRIT, P_BATT_EMERG,
    P_GF_MAX_ALT, P_GF_CEILING, P_GF_MAX_DIST, P_GF_ACTION,
    P_RTH_ALT, P_RTH_STICK_CANCEL,
    P_ARM_REQUIRE_CHECKS,
    P_COMPASS_PRIMARY,
    PARAM_COUNT
};

typedef struct {
    uint16_t id;
    float    def, min, max;
    uint16_t needs;          /* CAPS sensor bits required to list it (0 = always) */
} param_def_t;

extern const param_def_t g_param_def[PARAM_COUNT];

void     params_defaults(void);                  /* RAM <- defaults (dirty until saved) */
int      params_index(uint16_t id);              /* -1 if unknown */
float    param_get(int idx);                     /* lock-free read, any thread */
#define  PARAM(i) param_get(i)

/* PARAM_SET rules: returns an ACK result (PV2_ACK_*) and sets *reason */
uint8_t  params_set(uint16_t id, float v, uint8_t *reason);

/* flash image */
void     params_load(const float *values, uint16_t n);   /* out-of-range values -> default */
void     params_get_all(float *values);                   /* PARAM_COUNT values */
void     params_mark_saved(void);                         /* saved <- RAM */
uint8_t  params_dirty(int idx);                           /* RAM != flash */

/* listing: parameters whose sensor is missing are omitted */
uint8_t  params_visible(int idx, uint16_t sensors);
uint16_t params_visible_count(uint16_t sensors);
int      params_visible_pos(int idx, uint16_t sensors);   /* index in PARAM_LIST, -1 if hidden */

#endif /* PARAMS_H */
