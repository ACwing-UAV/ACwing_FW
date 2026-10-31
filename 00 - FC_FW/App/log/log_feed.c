/**
 * @file  log_feed.c
 * @brief Record builders (see log_feed.h).
 */
#include "log_feed.h"
#include "log_queue.h"
#include "log_format.h"
#include <math.h>
#include <string.h>

static uint32_t (*s_clock)(void);

void logf_set_clock(uint32_t (*now_ms)(void)) { s_clock = now_ms; }

static int16_t  i16(float v) { return (int16_t)(v > 32767.0f ? 32767 : (v < -32768.0f ? -32768 : lrintf(v))); }
static uint16_t u16(float v) { return (uint16_t)(v > 65535.0f ? 65535 : (v < 0.0f ? 0 : lrintf(v))); }
static int32_t  i32(float v) { return (int32_t)(v > 2.0e9f ? 2000000000 : (v < -2.0e9f ? -2000000000 : lrintf(v))); }
static int8_t   i8(float v)  { return (int8_t)(v > 127.0f ? 127 : (v < -128.0f ? -128 : lrintf(v))); }

void logf_att(uint32_t t, const float q[4], const float w[3], const float a[3])
{
    if (!log_q_active()) return;
    log_att_t r;
    r.t_ms = t;
    for (int i = 0; i < 4; ++i) r.q[i] = i16(q[i] * 32767.0f);
    for (int i = 0; i < 3; ++i) { r.gyro[i] = i16(w[i] * 1000.0f); r.acc[i] = i16(a[i] * 1000.0f); }
    (void)log_q_put(LOG_SRC_IMU, LR_ATT, &r, sizeof r);
}

void logf_vert(uint32_t t, const fc_vert_t *v, const fc_setpoint_t *sp, uint16_t tof_mm, uint32_t baro_pa)
{
    if (!log_q_active()) return;
    log_vert_t r;
    memset(&r, 0, sizeof r);
    r.t_ms = t;
    r.h_mm = i32(v->h * 1000.0f);
    r.vz_mms = i16(v->vz * 1000.0f);
    r.h_sp_mm = i32(sp->h_sp * 1000.0f);
    r.vz_sp_mms = i16(sp->vz_sp * 1000.0f);
    const float th = sp->thrust * 255.0f;
    r.thrust = (uint8_t)(th > 255.0f ? 255 : (th < 0.0f ? 0 : lrintf(th)));
    r.tof_mm = tof_mm;
    r.flow_q = v->flow_q;
    r.baro_pa = baro_pa;
    r.flags = (uint8_t)((v->valid ? 1u : 0u) | (v->tof_valid ? 2u : 0u) | (v->tof_ground ? 4u : 0u) |
                        (v->flow_usable ? 8u : 0u) | (sp->alt_ctrl ? 16u : 0u));
    (void)log_q_put(LOG_SRC_IMU, LR_VERT, &r, sizeof r);
}

void logf_flight(uint32_t t, const fc_state_t *s)
{
    if (!log_q_active()) return;
    log_flight_t r;
    r.t_ms = t;
    r.armed = s->armed; r.mode = s->mode; r.autom = s->autom; r.failsafe = s->failsafe;
    r.flags = (uint8_t)((s->in_air ? 1u : 0u) | (s->landed ? 2u : 0u) | (s->rc_stale ? 4u : 0u) |
                        (s->geofence ? 8u : 0u));
    for (int i = 0; i < 4; ++i) r.stick[i] = i8(s->stick[i] * 100.0f);
    r.silence_ms = (uint16_t)(s->silence_ms > 65535u ? 65535u : s->silence_ms);
    (void)log_q_put(LOG_SRC_IMU, LR_FLIGHT, &r, sizeof r);
}

void logf_est(uint32_t t, const float b[3], float unc, float vib, float hz, float jit, float cpu, float mi)
{
    if (!log_q_active()) return;
    log_est_t r;
    r.t_ms = t;
    for (int i = 0; i < 3; ++i) r.gyro_bias[i] = i16(b[i] * 10000.0f);
    r.att_unc_cdeg = u16(unc * 100.0f);
    r.vib_mg = u16(vib);
    r.loop_hz = u16(hz);
    r.jitter_us = u16(jit);
    r.cpu_pct = (uint8_t)u16(cpu > 255.0f ? 255.0f : cpu);
    r.mag_interf_pct = (uint8_t)u16(mi > 255.0f ? 255.0f : mi);
    (void)log_q_put(LOG_SRC_IMU, LR_EST, &r, sizeof r);
}

void logf_event(const fc_event_t *e)
{
    if (!log_q_active()) return;
    log_event_t r;
    r.t_ms = s_clock ? s_clock() : 0u;
    r.level = e->level; r.code = e->code; r.arg = e->arg;
    (void)log_q_put(LOG_SRC_IMU, LR_EVENT, &r, sizeof r);
}

void logf_power(uint32_t t, const batt_state_t *b)
{
    if (!log_q_active()) return;
    log_power_t r;
    r.t_ms = t;
    r.vbat_mv = u16(b->vbat * 1000.0f);
    r.current_ca = i16(b->current * 100.0f);
    r.used_mah = u16(b->used_mah);
    r.cell_mv = u16(b->cell_v * 1000.0f);
    r.cell_rest_mv = u16(b->cell_rest_v * 1000.0f);
    r.pct = b->pct;
    r.level = b->level;
    r.r_cell_mohm = u16(b->r_cell * 1000.0f);
    r.cells = (uint8_t)((b->cells & 0x7Fu) | (b->present ? 0x80u : 0u));
    (void)log_q_put(LOG_SRC_BATT, LR_POWER, &r, sizeof r);
}
