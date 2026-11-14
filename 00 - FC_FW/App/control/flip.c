/**
 * @file  flip.c
 * @brief Autonomous flip state machine (see flip.h, ADR 0012).
 */
#include "flip.h"
#include "proto_v2.h"
#include <math.h>
#include <string.h>

#define COS5  0.9962f
#define COS10 0.9848f
#define TWO_PI 6.2831853f

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

uint8_t fl_gate(const fl_gate_in_t *g)
{
    if (!g->armed || !g->in_air)                     return PV2_R_NOT_ARMED;
    if (!g->auto_none || g->failsafe || g->beginner || g->acro || !g->link_ok) return FL_REFUSED;
    if (!g->alt_valid)                               return PV2_R_SENSOR_MISSING;
    if (!(g->tw >= g->min_tw))                       return PV2_R_THRUST_LOW;
    if (g->height < g->min_h || g->ceiling_room < FL_CEILING_ROOM_M) return PV2_R_OUT_OF_RANGE;
    if (g->cos_tilt < COS10 || g->w >= 1.0f || g->v_h >= 1.0f || fabsf(g->vz) >= 0.5f || g->gyro_sat)
        return PV2_R_NOT_STILL;
    if (!g->batt_ok)                                 return PV2_R_BATTERY_LOW;
    return PV2_R_NONE;
}

static void enter(fl_t *f, uint8_t phase) { f->phase = phase; f->t_phase = 0u; }

void fl_start(fl_t *f, uint8_t axis, float rate)
{
    memset(f, 0, sizeof *f);
    f->axis = (axis <= FL_PITCH_BACK) ? axis : FL_ROLL_RIGHT;
    f->rate = rate;
    enter(f, FL_CLIMB);
}

void fl_cancel(fl_t *f) { memset(f, 0, sizeof *f); }

/* body axis and sign of the rotation: + roll = right wing down, + pitch = nose down (FLU) */
static int axis_index(const fl_t *f) { return (f->axis <= FL_ROLL_LEFT) ? 0 : 1; }
static float axis_sign(const fl_t *f) { return (f->axis == FL_ROLL_RIGHT || f->axis == FL_PITCH_FWD) ? 1.0f : -1.0f; }

static void rotate_step(fl_t *f, const fl_in_t *in, uint32_t dt_ms, fl_out_t *out)
{
    const int ax = axis_index(f);
    const float sg = axis_sign(f);
    const float w_ax = sg * in->w[ax];                     /* rate in the direction of the flip */
    f->angle += w_ax * (float)dt_ms * 1e-3f;
    const float lead = (w_ax > 0.0f) ? w_ax * w_ax / (2.0f * FL_ALPHA) : 0.0f;
    uint8_t why = FL_AB_NONE;
    if (f->t_phase > FL_ROT_MAX_MS)      why = FL_AB_TIMEOUT;
    else if (in->stick > FL_STICK_ABORT) why = FL_AB_STICK;
    else if (!in->link_ok)               why = FL_AB_LINK;
    else if (in->gyro_sat)               why = FL_AB_GYRO_SAT;
    if (why) { f->cut = why; enter(f, FL_RECOVER); return; }
    if (f->angle >= TWO_PI - lead - FL_ROT_MARGIN) { enter(f, FL_RECOVER); return; }
    out->rate_mode = 1u;
    out->rate[ax] = sg * f->rate;
    out->collective = FL_ROT_COLLECTIVE;
}

static void recover_step(fl_t *f, const fl_in_t *in, fl_out_t *out)
{
    const float wn = sqrtf(in->w[0] * in->w[0] + in->w[1] * in->w[1] + in->w[2] * in->w[2]);
    if (f->t_phase >= FL_REC_MIN_MS && in->cos_tilt > COS5 && wn < 1.0f) { enter(f, FL_DONE); return; }
    if (f->t_phase > FL_REC_MAX_MS) { enter(f, FL_ABORT); out->abort_reason = FL_AB_NOT_LEVEL; return; }
    const float h = clampf(in->hover, 0.2f, 0.9f);
    if (in->cos_tilt < 0.5f)  out->collective = h;            /* far from level: the attitude first */
    else if (in->vz < -0.05f) out->collective = 1.0f;         /* falling: all the thrust there is */
    else                      out->collective = clampf(h / in->cos_tilt, 0.0f, 1.0f);
}

void fl_step(fl_t *f, const fl_in_t *in, uint32_t dt_ms, fl_out_t *out)
{
    memset(out, 0, sizeof *out);
    f->t_phase += dt_ms;
    if (f->phase != FL_IDLE && f->phase != FL_DONE && f->phase != FL_ABORT && !in->est_ok) {
        enter(f, FL_ABORT); out->abort_reason = FL_AB_ESTIMATE;
    }
    switch (f->phase) {
    case FL_CLIMB:
        if (f->t_phase >= FL_CLIMB_MS) { enter(f, FL_ROTATE); rotate_step(f, in, dt_ms, out); break; }
        out->collective = 1.0f;
        break;
    case FL_ROTATE:  rotate_step(f, in, dt_ms, out); break;
    case FL_RECOVER: recover_step(f, in, out); break;
    default: break;
    }
    /* entered RECOVER on this step: its first command, not a gap */
    if (f->phase == FL_RECOVER && out->collective == 0.0f) recover_step(f, in, out);
    out->phase = f->phase;
    if (!out->abort_reason) out->abort_reason = f->cut;
    out->vector_hold = (uint8_t)(f->phase == FL_CLIMB || f->phase == FL_ROTATE || f->phase == FL_RECOVER);
}
