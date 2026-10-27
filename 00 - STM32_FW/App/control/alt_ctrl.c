/**
 * @file  alt_ctrl.c
 */
#include "alt_ctrl.h"

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

void alt_ctrl_init(alt_ctrl_t *c, float hover)
{
    c->hover = hover;
    c->kp_h = 1.2f;  c->kp_v = 0.25f;  c->ki_v = 0.15f;
    c->thr_min = 0.05f;  c->thr_max = 0.90f;
    c->i_lim = 0.35f;
    c->integ = 0.0f;
}

void alt_ctrl_reset(alt_ctrl_t *c, float integ) { c->integ = clampf(integ, -c->i_lim, c->i_lim); }

float alt_ctrl_vz_sp(const alt_ctrl_t *c, float h_sp, float h, float up, float dn)
{
    return clampf(c->kp_h * (h_sp - h), -dn, up);
}

float alt_ctrl_step(alt_ctrl_t *c, float vz_sp, float vz, float ct, float dt)
{
    const float e = vz_sp - vz;
    const float raw = c->hover + c->kp_v * e + c->integ;
    const float lim = clampf(raw / (ct > 0.5f ? ct : 0.5f), c->thr_min, c->thr_max);
    /* anti-windup: integrate only when not saturated in the error direction */
    if (!((lim >= c->thr_max && e > 0.0f) || (lim <= c->thr_min && e < 0.0f)))
        c->integ = clampf(c->integ + c->ki_v * e * dt, -c->i_lim, c->i_lim);
    return lim;
}
