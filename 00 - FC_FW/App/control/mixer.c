/**
 * @file  mixer.c
 * @brief Quad-X mixer (see mixer.h).
 */
#include "mixer.h"
#include <math.h>
#include <string.h>

static const float k_r[MIXER_MOTORS] = { -1.0f, -1.0f, +1.0f, +1.0f };
static const float k_p[MIXER_MOTORS] = { +1.0f, -1.0f, +1.0f, -1.0f };
static const float k_y[MIXER_MOTORS] = { +1.0f, -1.0f, -1.0f, +1.0f };

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

static float batt_k(float v_cell) { return (v_cell > 2.5f) ? MIXER_VREF / clampf(v_cell, 3.0f, 4.35f) : 1.0f; }

void mixer_direct(const float thrust[MIXER_MOTORS], float v_cell, const mixer_cfg_t *cfg, mixer_out_t *o)
{
    memset(o, 0, sizeof *o);
    o->batt_k = batt_k(v_cell);
    float sum = 0.0f;
    for (unsigned i = 0; i < MIXER_MOTORS; ++i) {
        const float t = clampf(thrust[i], 0.0f, 1.0f);
        o->t[i] = t; sum += t;
        o->duty[i] = clampf(mixer_curve_inv(t, cfg->theta) * o->batt_k, 0.0f, 1.0f);
    }
    o->collective = sum / (float)MIXER_MOTORS;
}

float mixer_curve_inv(float t, float theta)
{
    t = clampf(t, 0.0f, 1.0f);
    theta = clampf(theta, 0.0f, 1.0f);
    if (theta < 1e-4f) return t;
    const float b = 1.0f - theta;
    return (-b + sqrtf(b * b + 4.0f * theta * t)) / (2.0f * theta);
}

void mixer_quadx(float collective, const float torque[3], float v_cell, const mixer_cfg_t *cfg, mixer_out_t *o)
{
    memset(o, 0, sizeof *o);
    const float idle = clampf(cfg->idle, 0.0f, 0.3f);
    const float room = 1.0f - idle;                 /* the span every motor can use */
    float rp[MIXER_MOTORS], yw[MIXER_MOTORS];
    float rmin = 0.0f, rmax = 0.0f;
    for (unsigned i = 0; i < MIXER_MOTORS; ++i) {
        rp[i] = torque[0] * k_r[i] + torque[1] * k_p[i];
        yw[i] = torque[2] * k_y[i];
        if (rp[i] < rmin) rmin = rp[i];
        if (rp[i] > rmax) rmax = rp[i];
    }
    /* 1. roll / pitch alone must fit in the span */
    float span = rmax - rmin;
    if (span > room) {
        const float k = room / span;
        for (unsigned i = 0; i < MIXER_MOTORS; ++i) rp[i] *= k;
        rmin *= k; rmax *= k;
        span = room;
        o->sat_rp = 1u;
    }
    /* 2. yaw gets the room left: largest k <= 1 with the span of rp + k*yw within room */
    float mmin = 1e9f, mmax = -1e9f, k = 1.0f;
    for (int it = 0; it < 2; ++it) {
        mmin = 1e9f; mmax = -1e9f;
        for (unsigned i = 0; i < MIXER_MOTORS; ++i) {
            const float m = rp[i] + k * yw[i];
            if (m < mmin) mmin = m;
            if (m > mmax) mmax = m;
        }
        if (mmax - mmin <= room + 1e-6f) break;
        /* the yaw pattern is +-|yaw| on the diagonals: the span grows linearly with k beyond the roll/pitch span */
        const float ys = fabsf(torque[2]) * 2.0f;
        k = (ys > 1e-9f) ? clampf((room - span) / ys, 0.0f, 1.0f) : 0.0f;
        o->sat_yaw = 1u;
    }
    /* 3. collective: every motor in [idle, 1] */
    float T = collective;
    const float lo = idle - mmin, hi = 1.0f - mmax;
    if (T < lo) { T = lo; o->sat_col = 1u; }
    if (T > hi) { T = hi; o->sat_col = 1u; }
    o->collective = T;
    /* 4. thrust curve and battery */
    const float bk = batt_k(v_cell);
    o->batt_k = bk;
    for (unsigned i = 0; i < MIXER_MOTORS; ++i) {
        const float t = clampf(T + rp[i] + k * yw[i], 0.0f, 1.0f);
        o->t[i] = t;
        o->duty[i] = clampf(mixer_curve_inv(t, cfg->theta) * bk, 0.0f, 1.0f);
    }
}
