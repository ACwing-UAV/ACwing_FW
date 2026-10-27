/**
 * @file  params.c
 * @brief Parameter table and rules (see params.h).
 */
#include "params.h"
#include "proto_v2.h"
#include <math.h>
#include <string.h>

/* Same table as 01 - Tools/02 - App/src/protocol/ids.ts (PARAMS), same order. */
const param_def_t g_param_def[PARAM_COUNT] = {
    [P_TAKEOFF_ALT]        = {  1, 1.0f,   0.5f,   3.0f,   0 },
    [P_LAND_SPEED]         = {  2, 0.7f,   0.3f,   1.5f,   0 },
    [P_AUTO_DISARM]        = {  3, 2.0f,   1.0f,  10.0f,   0 },
    [P_MAX_TILT]           = {  4, 30.0f, 10.0f,  45.0f,   0 },
    [P_MAX_YAW_RATE]       = {  5, 120.0f, 30.0f, 360.0f,  0 },
    [P_MAX_CLIMB]          = {  6, 2.0f,   0.5f,   5.0f,   0 },
    [P_MAX_HSPEED]         = {  7, 8.0f,   1.0f,  15.0f,   0 },
    [P_BEGINNER]           = {  8, 0.0f,   0.0f,   1.0f,   0 },
    [P_FS_APP_SHORT]       = { 10, 0.0f,   0.0f,   1.0f,   0 },
    [P_FS_APP_LONG]        = { 11, 2.0f,   0.0f,   2.0f,   0 },
    [P_FS_RADIO]           = { 12, 2.0f,   0.0f,   2.0f,   0 },
    [P_FS_GPS_LOST]        = { 13, 0.0f,   0.0f,   2.0f,   PV2_S_GNSS },
    [P_FS_FLOW_LOST]       = { 14, 0.0f,   0.0f,   1.0f,   PV2_S_FLOW },
    [P_BATT_WARN]          = { 20, 3.6f,   3.3f,   3.9f,   0 },
    [P_BATT_CRIT]          = { 21, 3.5f,   3.2f,   3.8f,   0 },
    [P_BATT_EMERG]         = { 22, 3.3f,   3.0f,   3.6f,   0 },
    [P_GF_MAX_ALT]         = { 30, 30.0f,  5.0f, 120.0f,   0 },
    [P_GF_CEILING]         = { 31, 2.5f,   1.0f,   4.0f,   PV2_S_TOF },
    [P_GF_MAX_DIST]        = { 32, 150.0f, 20.0f, 500.0f,  PV2_S_GNSS },
    [P_GF_ACTION]          = { 33, 0.0f,   0.0f,   2.0f,   0 },
    [P_RTH_ALT]            = { 40, 30.0f,  5.0f,  60.0f,   PV2_S_GNSS },
    [P_RTH_STICK_CANCEL]   = { 41, 1.0f,   0.0f,   1.0f,   PV2_S_GNSS },
    [P_ARM_REQUIRE_CHECKS] = { 50, 1.0f,   0.0f,   1.0f,   0 },
    [P_COMPASS_PRIMARY]    = { 60, 1.0f,   0.0f,   1.0f,   PV2_S_MAG_GPS },
};

static volatile float s_val[PARAM_COUNT];
static float          s_saved[PARAM_COUNT];

void params_defaults(void)
{
    for (int i = 0; i < PARAM_COUNT; ++i) s_val[i] = g_param_def[i].def;
}

int params_index(uint16_t id)
{
    for (int i = 0; i < PARAM_COUNT; ++i) if (g_param_def[i].id == id) return i;
    return -1;
}

float param_get(int idx)
{
    if (idx < 0 || idx >= PARAM_COUNT) return 0.0f;
    return s_val[idx];
}

static int in_range(int i, float v)
{
    const float eps = 1e-6f;
    return isfinite(v) && v >= g_param_def[i].min - eps && v <= g_param_def[i].max + eps;
}

uint8_t params_set(uint16_t id, float v, uint8_t *reason)
{
    const int i = params_index(id);
    if (i < 0) { *reason = PV2_R_UNKNOWN_PARAM; return PV2_ACK_INVALID; }
    if (!in_range(i, v)) { *reason = PV2_R_OUT_OF_RANGE; return PV2_ACK_INVALID; }
    /* battWarn > battCrit > battEmerg must hold after the change */
    float w = s_val[P_BATT_WARN], c = s_val[P_BATT_CRIT], e = s_val[P_BATT_EMERG];
    if (i == P_BATT_WARN) w = v;
    if (i == P_BATT_CRIT) c = v;
    if (i == P_BATT_EMERG) e = v;
    if (!(w > c && c > e)) { *reason = PV2_R_OUT_OF_RANGE; return PV2_ACK_INVALID; }
    s_val[i] = v;
    *reason = PV2_R_NONE;
    return PV2_ACK_OK;
}

void params_load(const float *values, uint16_t n)
{
    params_defaults();
    for (int i = 0; i < PARAM_COUNT && i < (int)n; ++i)
        if (in_range(i, values[i])) s_val[i] = values[i];
    if (!(s_val[P_BATT_WARN] > s_val[P_BATT_CRIT] && s_val[P_BATT_CRIT] > s_val[P_BATT_EMERG])) {
        s_val[P_BATT_WARN]  = g_param_def[P_BATT_WARN].def;
        s_val[P_BATT_CRIT]  = g_param_def[P_BATT_CRIT].def;
        s_val[P_BATT_EMERG] = g_param_def[P_BATT_EMERG].def;
    }
}

void params_get_all(float *values)
{
    for (int i = 0; i < PARAM_COUNT; ++i) values[i] = s_val[i];
}

void params_mark_saved(void)
{
    for (int i = 0; i < PARAM_COUNT; ++i) s_saved[i] = s_val[i];
}

uint8_t params_dirty(int idx)
{
    if (idx < 0 || idx >= PARAM_COUNT) return 0;
    return fabsf(s_val[idx] - s_saved[idx]) > 1e-6f ? 1u : 0u;
}

uint8_t params_visible(int idx, uint16_t sensors)
{
    if (idx < 0 || idx >= PARAM_COUNT) return 0;
    const uint16_t need = g_param_def[idx].needs;
    return (uint8_t)((need & sensors) == need);
}

uint16_t params_visible_count(uint16_t sensors)
{
    uint16_t n = 0;
    for (int i = 0; i < PARAM_COUNT; ++i) n += params_visible(i, sensors);
    return n;
}

int params_visible_pos(int idx, uint16_t sensors)
{
    if (!params_visible(idx, sensors)) return -1;
    int pos = 0;
    for (int i = 0; i < idx; ++i) pos += params_visible(i, sensors);
    return pos;
}
