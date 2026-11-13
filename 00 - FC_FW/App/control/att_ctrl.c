/**
 * @file  att_ctrl.c
 * @brief Attitude and rate controllers (see att_ctrl.h).
 */
#include "att_ctrl.h"
#include <math.h>
#include <string.h>

#define PI_F 3.14159265358979f

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

/* ------------------------------------------------------------------ biquads (RBJ cookbook) */
static void passthrough(biquad_t *f) { memset(f, 0, sizeof *f); f->b0 = 1.0f; }

void biquad_lowpass(biquad_t *f, float fc, float fs)
{
    if (!(fc > 0.0f) || !(fs > 0.0f)) { passthrough(f); return; }
    fc = clampf(fc, 1.0f, 0.45f * fs);
    const float w = 2.0f * PI_F * fc / fs, cs = cosf(w), al = sinf(w) / (2.0f * 0.70710678f);
    const float a0 = 1.0f + al;
    f->b0 = (1.0f - cs) * 0.5f / a0; f->b1 = (1.0f - cs) / a0; f->b2 = f->b0;
    f->a1 = -2.0f * cs / a0;         f->a2 = (1.0f - al) / a0;
    f->z1 = f->z2 = 0.0f;
}

void biquad_notch(biquad_t *f, float fc, float q, float fs)
{
    if (!(fc > 0.0f) || !(fs > 0.0f)) { passthrough(f); return; }
    fc = clampf(fc, 1.0f, 0.45f * fs);
    if (!(q > 0.1f)) q = 0.1f;
    const float w = 2.0f * PI_F * fc / fs, cs = cosf(w), al = sinf(w) / (2.0f * q);
    const float a0 = 1.0f + al;
    f->b0 = 1.0f / a0; f->b1 = -2.0f * cs / a0; f->b2 = f->b0;
    f->a1 = f->b1;     f->a2 = (1.0f - al) / a0;
    f->z1 = f->z2 = 0.0f;
}

float biquad_step(biquad_t *f, float x)
{
    const float y = f->b0 * x + f->z1;
    f->z1 = f->b1 * x - f->a1 * y + f->z2;
    f->z2 = f->b2 * x - f->a2 * y;
    return y;
}

void biquad_reset(biquad_t *f, float x)
{
    const float den = 1.0f + f->a1 + f->a2;
    const float g = (fabsf(den) > 1e-9f) ? (f->b0 + f->b1 + f->b2) / den : 1.0f;
    const float y = g * x;
    f->z1 = y - f->b0 * x;
    f->z2 = f->b2 * x - f->a2 * y;
}

/* ------------------------------------------------------------------ gains */
void att_gains_default(att_gains_t *g)
{
    /* 1S micro quad (8520 motors, 55 mm props, ~75 g, arm 40 mm, J ~ 3.5e-5 kg m^2, vehicle-model.md):
     * one unit of roll torque ~ 1100 rad/s^2, yaw ~ 270 rad/s^2. Crossover of the rate loop ~ 20 rad/s, below
     * the motor pole (~ 28 rad/s, tau_m ~ 35 ms); tilt loop 6 rad/s. A starting point: tune on the first flights. */
    memset(g, 0, sizeof *g);
    g->rate_p[0] = g->rate_p[1] = 0.020f;  g->rate_p[2] = 0.040f;
    g->rate_i[0] = g->rate_i[1] = 0.20f;   g->rate_i[2] = 0.20f;
    g->rate_d[0] = g->rate_d[1] = 0.0004f; g->rate_d[2] = 0.0f;
    g->att_p = 6.0f;
    g->rate_max_tilt = 6.0f;               /* rad/s (~ 340 deg/s) */
    g->i_lim = 0.15f;
    g->out_lim = 0.5f;
    g->gyro_lpf_hz = 90.0f;
    g->dterm_lpf_hz = 70.0f;
    g->notch_hz = 0.0f; g->notch_q = 3.0f;
    g->rate_hz = 960.0f;
}

static void build_filters(att_ctrl_t *c)
{
    for (int i = 0; i < 3; ++i) {
        biquad_lowpass(&c->gyro_lpf[i], c->g.gyro_lpf_hz, c->g.rate_hz);
        biquad_notch(&c->gyro_notch[i], c->g.notch_hz, c->g.notch_q, c->g.rate_hz);
        biquad_lowpass(&c->dterm_lpf[i], c->g.dterm_lpf_hz, c->g.rate_hz);
    }
}

void att_ctrl_init(att_ctrl_t *c, const att_gains_t *g)
{
    memset(c, 0, sizeof *c);
    c->g = *g;
    if (!(c->g.rate_hz > 0.0f)) c->g.rate_hz = 960.0f;
    build_filters(c);
}

void att_ctrl_set_gains(att_ctrl_t *c, const att_gains_t *g)
{
    const uint8_t refilter = (uint8_t)(g->gyro_lpf_hz != c->g.gyro_lpf_hz || g->dterm_lpf_hz != c->g.dterm_lpf_hz ||
                                       g->notch_hz != c->g.notch_hz || g->notch_q != c->g.notch_q ||
                                       g->rate_hz != c->g.rate_hz);
    c->g = *g;
    if (!(c->g.rate_hz > 0.0f)) c->g.rate_hz = 960.0f;
    for (int i = 0; i < 3; ++i) c->i[i] = clampf(c->i[i], -c->g.i_lim, c->g.i_lim);
    if (refilter) {
        build_filters(c);
        for (int i = 0; i < 3; ++i) {
            biquad_reset(&c->gyro_notch[i], c->gyro_f[i]);
            biquad_reset(&c->gyro_lpf[i], c->gyro_f[i]);
            biquad_reset(&c->dterm_lpf[i], 0.0f);
        }
    }
}

void att_ctrl_reset(att_ctrl_t *c)
{
    for (int i = 0; i < 3; ++i) {
        c->i[i] = 0.0f;
        c->torque[i] = 0.0f;
        biquad_reset(&c->dterm_lpf[i], 0.0f);
    }
    c->have_prev = 0u;
}

/* ------------------------------------------------------------------ attitude (240 Hz) */
void att_ctrl_attitude(att_ctrl_t *c, const float q[4], const att_sp_t *sp)
{
    if (sp->acro) {
        c->rate_sp[0] = sp->roll_rate; c->rate_sp[1] = sp->pitch_rate; c->rate_sp[2] = sp->yaw_rate;
        c->tilt_err = 0.0f;
        return;
    }
    const float w = q[0], x = q[1], y = q[2], z = q[3];
    /* body z axis in the world (third column of R) and the current heading */
    const float zb[3] = { 2.0f * (x * z + w * y), 2.0f * (y * z - w * x), 1.0f - 2.0f * (x * x + y * y) };
    const float psi = atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
    /* desired body z: R_d = Rz(psi) Ry(pitch) Rx(roll), third column */
    const float cp = cosf(psi), sp_ = sinf(psi), ct = cosf(sp->pitch), st = sinf(sp->pitch);
    const float cr = cosf(sp->roll), sr = sinf(sp->roll);
    const float zd[3] = { cp * st * cr + sp_ * sr, sp_ * st * cr - cp * sr, ct * cr };
    /* rotation zb -> zd: axis zb x zd (world), angle atan2(|x|, .) */
    const float cw[3] = { zb[1] * zd[2] - zb[2] * zd[1], zb[2] * zd[0] - zb[0] * zd[2], zb[0] * zd[1] - zb[1] * zd[0] };
    const float s = sqrtf(cw[0] * cw[0] + cw[1] * cw[1] + cw[2] * cw[2]);
    const float d = zb[0] * zd[0] + zb[1] * zd[1] + zb[2] * zd[2];
    const float ang = atan2f(s, d);
    float e[3] = { 0.0f, 0.0f, 0.0f };
    if (s > 1e-7f) {
        /* to the body frame: e = R^T cw / s * ang */
        const float R[9] = {
            1.0f - 2.0f * (y * y + z * z), 2.0f * (x * y - w * z),        2.0f * (x * z + w * y),
            2.0f * (x * y + w * z),        1.0f - 2.0f * (x * x + z * z), 2.0f * (y * z - w * x),
            2.0f * (x * z - w * y),        2.0f * (y * z + w * x),        1.0f - 2.0f * (x * x + y * y) };
        for (int i = 0; i < 3; ++i) e[i] = (R[i] * cw[0] + R[3 + i] * cw[1] + R[6 + i] * cw[2]) / s * ang;
    }
    c->tilt_err = ang;
    c->rate_sp[0] = clampf(c->g.att_p * e[0], -c->g.rate_max_tilt, c->g.rate_max_tilt);
    c->rate_sp[1] = clampf(c->g.att_p * e[1], -c->g.rate_max_tilt, c->g.rate_max_tilt);
    c->rate_sp[2] = sp->yaw_rate;
}

/* ------------------------------------------------------------------ rate (960 Hz) */
void att_ctrl_rate(att_ctrl_t *c, const float gyro[3], uint8_t integrate, float torque[3])
{
    const float dt = 1.0f / c->g.rate_hz;
    for (int i = 0; i < 3; ++i) {
        const float g = biquad_step(&c->gyro_lpf[i], biquad_step(&c->gyro_notch[i], gyro[i]));
        c->gyro_f[i] = g;
        const float err = c->rate_sp[i] - g;
        /* D on the measurement: no kick on setpoint steps */
        const float dg = c->have_prev ? (g - c->prev_rate[i]) / dt : 0.0f;
        c->prev_rate[i] = g;
        const float dterm = -c->g.rate_d[i] * biquad_step(&c->dterm_lpf[i], dg);
        const float p = c->g.rate_p[i] * err;
        float out = p + c->i[i] + dterm;
        /* integrator: only when allowed, and not further into a saturated output */
        if (integrate && !((out >= c->g.out_lim && err > 0.0f) || (out <= -c->g.out_lim && err < 0.0f))) {
            c->i[i] = clampf(c->i[i] + c->g.rate_i[i] * err * dt, -c->g.i_lim, c->g.i_lim);
            out = p + c->i[i] + dterm;
        }
        c->torque[i] = torque[i] = clampf(out, -c->g.out_lim, c->g.out_lim);
    }
    c->have_prev = 1u;
}
