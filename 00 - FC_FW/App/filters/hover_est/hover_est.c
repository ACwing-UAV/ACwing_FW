/**
 * @file  hover_est.c
 * @brief Hover-thrust estimate (see hover_est.h).
 */
#include "hover_est.h"
#include <math.h>
#include <string.h>

#define G0 9.80665f

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

void hover_est_init(hover_est_t *e, float seed, float sigma0)
{
    memset(e, 0, sizeof *e);
    e->th = clampf(seed, HOVER_EST_MIN, HOVER_EST_MAX);
    e->P = sigma0 * sigma0;
    e->q = 0.012f * 0.012f;        /* sqrt(q) = 0.012 s^-1/2: a +15 g step tracked in ~2 s (test_hover.c) */
    e->r = 2.0f * 2.0f;            /* sigma_a = 2 m/s^2: vibration with the props on (tune from the logs) */
    e->gate = 9.0f;
}

void hover_est_predict(hover_est_t *e, float dt)
{
    if (dt > 0.0f && dt < 1.0f) e->P += e->q * dt;
}

uint8_t hover_est_update(hover_est_t *e, float a_up, float t_c, float dt)
{
    if (e->infl_s > 0.0f) e->infl_s -= dt;
    const float th = e->th;
    const float pred = G0 * (t_c / th - 1.0f);
    const float H = -G0 * t_c / (th * th);
    const float R = e->r * (e->infl_s > 0.0f ? 4.0f : 1.0f);
    const float S = H * H * e->P + R;
    const float y = a_up - pred;
    e->nis = y * y / S;
    if (e->nis > e->gate) {
        e->infl_s = 0.5f;
        e->n_rej++;
        return 0u;
    }
    const float K = e->P * H / S;
    e->th = clampf(th + K * y, HOVER_EST_MIN, HOVER_EST_MAX);
    e->P = (1.0f - K * H) * e->P;
    if (e->P < 1e-8f) e->P = 1e-8f;
    e->n_upd++;
    return 1u;
}
