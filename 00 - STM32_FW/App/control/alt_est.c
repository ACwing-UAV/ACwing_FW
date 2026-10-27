/**
 * @file  alt_est.c
 * @brief Vertical Kalman filter (see alt_est.h).
 */
#include "alt_est.h"
#include <math.h>
#include <string.h>

#define G0          9.80665f
#define BARO_INIT_N 25u          /* 0.5 s of samples to set the reference pressure */
#define TOF_GATE_S  3.0f         /* innovation gate, sigmas                         */
#define TOF_REANCHOR 15u         /* consecutive rejects (0.5 s at 30 Hz)            */

void alt_est_default_cfg(alt_est_cfg_t *c)
{
    c->sigma_acc        = 0.5f;
    c->sigma_acc_bias   = 0.01f;
    c->sigma_baro_bias  = 0.05f;
    c->sigma_baro       = 0.30f;
    c->sigma_tof        = 0.02f;
    c->tof_min_m        = 0.04f;
    c->tof_max_m        = 4.0f;
    c->tof_max_tilt_cos = 0.866f;      /* 30 deg */
    c->tof_offset_m     = 0.0113f;     /* frame C2: lens 11.3 mm above the floor */
}

void alt_est_init(alt_est_t *e, const alt_est_cfg_t *c)
{
    memset(e, 0, sizeof *e);
    e->c = *c;
    e->P[0][0] = 0.01f; e->P[1][1] = 0.01f; e->P[2][2] = 1.0f; e->P[3][3] = 0.04f;
    e->tof_age_s = 1e3f;
}

float alt_est_accel_up(const float q[4], const float f[3], float nom_g)
{
    /* third row of R(q), body -> world (NWU, z up), q = [w x y z] */
    const float w = q[0], x = q[1], y = q[2], z = q[3];
    const float r20 = 2.0f * (x*z - w*y);
    const float r21 = 2.0f * (y*z + w*x);
    const float r22 = 1.0f - 2.0f * (x*x + y*y);
    const float fz  = r20*f[0] + r21*f[1] + r22*f[2];
    const float n   = (nom_g > 0.5f) ? nom_g : 1.0f;
    return (fz / n - 1.0f) * G0;
}

void alt_est_predict(alt_est_t *e, float a, float dt)
{
    if (dt <= 0.0f || dt > 0.1f) return;
    float *x = e->x;
    const float u = a - x[3];
    x[0] += x[1]*dt + 0.5f*u*dt*dt;
    x[1] += u*dt;

    /* P = F P F' + Q, F = [1 dt 0 -dt2/2; 0 1 0 -dt; 0 0 1 0; 0 0 0 1] */
    const float h2 = 0.5f*dt*dt;
    float F[4][4] = { { 1, dt, 0, -h2 }, { 0, 1, 0, -dt }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };
    float T[4][4], N[4][4];
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            float s = 0.0f;
            for (int k = 0; k < 4; ++k) s += F[i][k] * e->P[k][j];
            T[i][j] = s;
        }
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            float s = 0.0f;
            for (int k = 0; k < 4; ++k) s += T[i][k] * F[j][k];
            N[i][j] = s;
        }
    const float qa = e->c.sigma_acc * e->c.sigma_acc;
    const float G[2] = { h2, dt };
    for (int i = 0; i < 2; ++i) for (int j = 0; j < 2; ++j) N[i][j] += G[i]*G[j]*qa;
    /* baro bias random walk only while it is observable (ToF in the last 1 s) */
    if (e->tof_age_s < 1.0f) N[2][2] += e->c.sigma_baro_bias * e->c.sigma_baro_bias * dt;
    N[3][3] += e->c.sigma_acc_bias * e->c.sigma_acc_bias * dt;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) e->P[i][j] = 0.5f * (N[i][j] + N[j][i]);
    e->tof_age_s += dt;
}

/* scalar update with H = [h0 0 h2 0]; returns 0 if applied, -1 if gated */
static int update(alt_est_t *e, float h0, float h2, float z, float r, float gate)
{
    const float pred = h0*e->x[0] + h2*e->x[2];
    float PH[4];
    for (int i = 0; i < 4; ++i) PH[i] = e->P[i][0]*h0 + e->P[i][2]*h2;
    const float S = h0*PH[0] + h2*PH[2] + r;
    if (S <= 0.0f) return -1;
    const float y = z - pred;
    if (gate > 0.0f && y*y > gate*gate*S) return -1;
    float K[4];
    for (int i = 0; i < 4; ++i) K[i] = PH[i] / S;
    for (int i = 0; i < 4; ++i) e->x[i] += K[i] * y;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) e->P[i][j] -= K[i] * PH[j];
    for (int i = 0; i < 4; ++i)
        for (int j = i + 1; j < 4; ++j) { const float m = 0.5f*(e->P[i][j] + e->P[j][i]); e->P[i][j] = e->P[j][i] = m; }
    return 0;
}

void alt_est_baro(alt_est_t *e, float pa)
{
    if (!(pa > 30000.0f && pa < 120000.0f)) return;      /* also rejects NaN */
    e->pressure = pa;
    if (!e->baro_ok) {
        e->p_sum += pa;
        if (++e->p_n < BARO_INIT_N) return;
        e->p_ref = e->p_sum / (float)e->p_n;
        e->baro_ok = 1;
        /* bias such that the baro agrees with the current h */
        e->x[2] = -e->x[0];
        e->valid = 1;
        return;
    }
    e->baro_alt = 44330.0f * (1.0f - powf(pa / e->p_ref, 0.190295f));
    (void)update(e, 1.0f, 1.0f, e->baro_alt, e->c.sigma_baro * e->c.sigma_baro, 0.0f);
}

void alt_est_tof(alt_est_t *e, float range, uint8_t ok, float ct)
{
    const alt_est_cfg_t *c = &e->c;
    e->tof_valid = 0;
    /* below the minimum range, or no return right after a short reading: on the floor */
    const uint8_t short_rd = (range < c->tof_min_m + 0.02f) || (!ok && e->have_tof && e->tof_h < 0.15f);
    e->tof_ground = (uint8_t)(short_rd && ct > c->tof_max_tilt_cos);
    float hm;
    float r;
    if (e->tof_ground) {
        hm = 0.0f;                                   /* the floor: h = ground */
        r  = 0.03f * 0.03f;
    } else {
        if (!ok || range > c->tof_max_m || ct < c->tof_max_tilt_cos) return;
        hm = range * ct - c->tof_offset_m;
        const float s = c->sigma_tof + 0.01f * range;
        r  = s * s;
    }
    if (!e->have_tof) {
        /* first ToF: the floor sits under the current h */
        e->ground = e->x[0] - hm;
        e->have_tof = 1;
    }
    if (update(e, 1.0f, 0.0f, hm + e->ground, r, TOF_GATE_S) != 0) {
        if (++e->tof_rejects >= TOF_REANCHOR) {      /* the floor moved: re-anchor */
            e->ground = e->x[0] - hm;
            e->tof_rejects = 0;
        } else {
            return;
        }
    } else {
        e->tof_rejects = 0;
    }
    e->tof_h = hm;
    e->tof_valid = 1;
    e->tof_age_s = 0.0f;
    e->valid = 1;
}

void alt_est_zero(alt_est_t *e)
{
    const float h0 = e->x[0];
    e->x[0] = 0.0f;
    e->x[2] += h0;           /* baro: z = h + b keeps its prediction */
    e->ground -= h0;
}
