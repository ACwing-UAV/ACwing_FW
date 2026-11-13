/**
 * @file  launch.c
 * @brief Hand launch state machine (see launch.h, ADR 0011).
 */
#include "launch.h"
#include <math.h>
#include <string.h>

#define COS30 0.866f
#define COS15 0.966f

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

float ln_tw_available(float hover, float v_cell)
{
    if (!(hover > 0.05f) || !(v_cell > 2.5f)) return 0.0f;
    const float k = v_cell / 4.2f;
    return k * k / hover;
}

static void enter(ln_t *l, uint8_t phase) { l->phase = phase; l->t_phase = 0u; }

void ln_start(ln_t *l, uint8_t drop_allowed)
{
    memset(l, 0, sizeof *l);
    l->drop = drop_allowed;
    enter(l, LN_READY);
}

void ln_cancel(ln_t *l) { memset(l, 0, sizeof *l); }

static void integrate(float x[3], const float d[3], float dt) { for (int i = 0; i < 3; ++i) x[i] += d[i] * dt; }

/* held still and level: the presentation, and the zero of the velocity */
static uint8_t still(const ln_in_t *in)
{
    return (uint8_t)(in->w < 1.0f && fabsf(in->f_g - 1.0f) < 0.1f && in->cos_tilt > COS30);
}

/* READY: presentation, push, free fall -> RELEASED */
static void ready_step(ln_t *l, const ln_in_t *in, uint32_t dt_ms, ln_out_t *out)
{
    const float dt = (float)dt_ms * 1e-3f;
    if (l->t_phase > LN_READY_MAX_MS) { enter(l, LN_ABORT); out->abort_reason = LN_AB_TIMEOUT; return; }
    l->link_bad_ms = in->link_ok ? 0u : l->link_bad_ms + dt_ms;
    if (l->link_bad_ms > LN_LINK_LOST_MS) { enter(l, LN_ABORT); out->abort_reason = LN_AB_LINK; return; }

    if (still(in)) {
        l->still_ms += dt_ms;
        if (l->still_ms >= LN_PRESENT_MS) l->presented = 1u;
        l->v[0] = l->v[1] = l->v[2] = 0.0f;       /* still: the velocity is zero */
        l->pushed = 0u;                            /* a new gesture starts from here */
        l->ff_ms = 0u;
        return;
    }
    l->still_ms = 0u;
    if (!l->presented) return;                     /* carried, swung, dropped before the presentation: no launch */

    /* quiet (held, carried) for LN_QUIET_MS: the hand is about still, the velocity back to zero, the push
     * forgotten. The dead reckoning never runs for longer than one gesture, so its drift stays small. */
    if (in->f_g > 0.7f && in->f_g < LN_PUSH_G) {
        l->quiet_ms += dt_ms;
        if (l->quiet_ms >= LN_QUIET_MS) { l->v[0] = l->v[1] = l->v[2] = 0.0f; l->pushed = 0u; }
    } else l->quiet_ms = 0u;
    if (in->f_g > LN_PUSH_G) l->pushed = 1u;
    if (in->f_g < LN_FREEFALL_G) {
        if (l->ff_ms == 0u) { l->p[0] = l->p[1] = l->p[2] = 0.0f; }   /* the release point */
        l->ff_ms += dt_ms;
        integrate(l->p, l->v, dt);
    } else l->ff_ms = 0u;
    integrate(l->v, in->a_w, dt);

    if (l->ff_ms >= LN_FREEFALL_MS && (l->pushed || l->drop) && !in->gyro_sat) {
        l->sep_ms = l->ff_ms;
        enter(l, LN_RELEASED);
    }
}

/* RELEASED: motors off until the drone is away from the hand */
static void released_step(ln_t *l, const ln_in_t *in, uint32_t dt_ms, ln_out_t *out)
{
    const float dt = (float)dt_ms * 1e-3f;
    if (!in->est_ok) { enter(l, LN_ABORT); out->abort_reason = LN_AB_ESTIMATE; return; }
    integrate(l->p, l->v, dt);
    integrate(l->v, in->a_w, dt);
    l->sep_ms += dt_ms;
    const float d = sqrtf(l->p[0] * l->p[0] + l->p[1] * l->p[1] + l->p[2] * l->p[2]);
    const uint8_t apex = (uint8_t)(l->pushed && l->v[2] <= 0.0f);        /* a toss at the top of its arc */
    /* a toss too weak to rise LN_APEX_MIN_M falls back onto the hand: without launchDrop, the motors stay off */
    if (apex && l->p[2] < LN_APEX_MIN_M && !l->drop) { enter(l, LN_ABORT); out->abort_reason = LN_AB_WEAK_TOSS; return; }
    const uint8_t sep = (uint8_t)(d >= LN_SEPARATION_M || (apex && l->p[2] >= LN_APEX_MIN_M));
    /* the time fallback only for a fall (drop): never while the drone may still be over the hand */
    const uint8_t late = (uint8_t)(l->drop && l->sep_ms >= LN_FREEFALL_MS + LN_SEP_MAX_MS);
    if (sep || late) {
        out->released = 1u;
        out->start_ms = (uint16_t)(l->sep_ms > 0xFFFFu ? 0xFFFFu : l->sep_ms);
        enter(l, LN_RECOVER);
    }
}

/* RECOVER: thrust and level attitude until stable */
static void recover_step(ln_t *l, const ln_in_t *in, ln_out_t *out)
{
    if (!in->est_ok) { enter(l, LN_ABORT); out->abort_reason = LN_AB_ESTIMATE; return; }
    if (l->t_phase > LN_RECOVER_MAX_MS) { enter(l, LN_ABORT); out->abort_reason = LN_AB_NOT_RECOVERED; return; }
    if (l->t_phase >= LN_RECOVER_MIN_MS && in->cos_tilt > COS15 && in->w < 1.0f && in->vz >= 0.0f) {
        enter(l, LN_DONE);
        return;
    }
    const float h = clampf(in->hover, 0.2f, 0.9f);
    if (in->cos_tilt < 0.5f) out->thrust = h;                         /* far from level: the attitude first */
    else if (in->vz < -0.05f) out->thrust = 1.0f;                      /* falling: all the thrust there is */
    else out->thrust = clampf(h / in->cos_tilt, 0.0f, 1.0f);
}

void ln_step(ln_t *l, const ln_in_t *in, uint32_t dt_ms, ln_out_t *out)
{
    memset(out, 0, sizeof *out);
    l->t_phase += dt_ms;
    switch (l->phase) {
    case LN_READY:    ready_step(l, in, dt_ms, out); break;
    case LN_RELEASED: released_step(l, in, dt_ms, out); break;
    case LN_RECOVER:  recover_step(l, in, out); break;
    default: break;
    }
    out->phase = l->phase;
    out->motors_off = (uint8_t)(l->phase == LN_READY || l->phase == LN_RELEASED);
    if (l->phase == LN_RECOVER && out->thrust == 0.0f && !out->released) out->thrust = clampf(in->hover, 0.2f, 0.9f);
    if (out->released) out->thrust = 1.0f;
}
