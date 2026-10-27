/**
 * @file  calib.c
 * @brief Static calibration (see calib.h).
 */
#include "calib.h"
#include "so3.h"
#include <string.h>
#include <math.h>

void calib_reset(calib_t *c) { memset(c, 0, sizeof *c); }

uint32_t calib_count(const calib_t *c) { return c->n; }

int calib_add(calib_t *c, const float acc[3], const float gyro[3], const float mag[3],
              float gyro_still_thr)
{
    const float gn = vec3_norm(gyro);
    if (gn > gyro_still_thr) return -1;          /* it is moving    */
    if (c->n >= CALIB_MAX_SAMPLES) return 0;
    memcpy(c->acc[c->n], acc, 3*sizeof(float));
    memcpy(c->mag[c->n], mag, 3*sizeof(float));
    for (int i = 0; i < 3; ++i) c->gyro_sum[i] += gyro[i];
    if (gn > c->gyro_norm_max) c->gyro_norm_max = gn;
    c->n++;
    return 0;
}

/* 1:1 port of directionNoiseStd.m */
float direction_noise_std(const float (*v)[3], uint32_t n)
{
    if (n < 2) return 0.0f;
    float um[3] = { 0, 0, 0 };
    for (uint32_t k = 0; k < n; ++k) {
        const float nk = vec3_norm(v[k]);
        if (nk < 1e-9f) continue;
        for (int i = 0; i < 3; ++i) um[i] += v[k][i] / nk;
    }
    if (vec3_normalize(um)) return 0.0f;

    float e = 0.0f;
    for (uint32_t k = 0; k < n; ++k) {
        const float nk = vec3_norm(v[k]);
        if (nk < 1e-9f) continue;
        float u[3] = { v[k][0]/nk, v[k][1]/nk, v[k][2]/nk };
        const float d = vec3_dot(um, u);
        float r[3] = { u[0] - d*um[0], u[1] - d*um[1], u[2] - d*um[2] };
        e += vec3_dot(r, r);
    }
    return sqrtf((e / (float)n) / 2.0f);          /* 2 tangent degrees of freedom */
}

int calib_finish(const calib_t *c, float sigma_min, calib_result_t *out)
{
    if (c->n < 10) return -1;
    const float inv_n = 1.0f / (float)c->n;

    float am[3] = { 0, 0, 0 }, mm[3] = { 0, 0, 0 };
    float an = 0.0f, mn = 0.0f;
    for (uint32_t k = 0; k < c->n; ++k) {
        for (int i = 0; i < 3; ++i) { am[i] += c->acc[k][i]; mm[i] += c->mag[k][i]; }
        an += vec3_norm(c->acc[k]);
        mn += vec3_norm(c->mag[k]);
    }
    for (int i = 0; i < 3; ++i) { am[i] *= inv_n; mm[i] *= inv_n; }
    if (vec3_normalize(am) || vec3_normalize(mm)) return -2;

    /* TRIAD: world axes expressed in the body frame */
    float z[3] = { am[0], am[1], am[2] };                     /* up                */
    const float d = vec3_dot(mm, z);
    float x[3] = { mm[0] - d*z[0], mm[1] - d*z[1], mm[2] - d*z[2] }; /* horiz. north */
    if (vec3_normalize(x)) return -3;                         /* field ~ vertical  */
    float y[3];
    vec3_cross(z, x, y);                                      /* west              */

    iekf_seed_t *s = &out->seed;
    /* R0 (body->world): rows = world axes in the body frame */
    s->R0[0] = x[0]; s->R0[1] = x[1]; s->R0[2] = x[2];
    s->R0[3] = y[0]; s->R0[4] = y[1]; s->R0[5] = y[2];
    s->R0[6] = z[0]; s->R0[7] = z[1]; s->R0[8] = z[2];

    mat3_vec(s->R0, am, s->g_ref);                            /* = [0 0 1]              */
    mat3_vec(s->R0, mm, s->m_ref);                            /* = [cos I, 0, -sin I]   */
    for (int i = 0; i < 3; ++i) s->b0[i] = c->gyro_sum[i] * inv_n;
    s->nom_acc = an * inv_n;
    s->nom_mag = mn * inv_n;

    out->sigma_acc_raw = direction_noise_std((const float (*)[3])c->acc, c->n);
    out->sigma_mag_raw = direction_noise_std((const float (*)[3])c->mag, c->n);
    out->sigma_acc = fmaxf(out->sigma_acc_raw, sigma_min);
    out->sigma_mag = fmaxf(out->sigma_mag_raw, sigma_min);
    return 0;
}
