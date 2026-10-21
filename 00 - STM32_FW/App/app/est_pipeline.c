/**
 * @file  est_pipeline.c
 */
#include "est_pipeline.h"
#include "control_hooks.h"
#include "so3.h"
#include <string.h>
#include <math.h>

static void trace_put(est_t *e, uint8_t op, const float *v, int nf, int trusted)
{
    if (!e->trace_enable || !e->running) return;
    est_trace_t *t = &e->trace;
    const uint16_t need = (uint16_t)(1u + 4u*(unsigned)nf + (op == EST_OP_ACC ? 1u : 0u));
    if (t->len + need > EST_TRACE_MAX) { t->overflow = 1; return; }
    t->buf[t->len++] = op;
    memcpy(&t->buf[t->len], v, 4u*(unsigned)nf);
    t->len = (uint16_t)(t->len + 4u*(unsigned)nf);
    if (op == EST_OP_ACC) t->buf[t->len++] = (uint8_t)(trusted ? 1 : 0);
    t->nops++;
}

void est_init(est_t *e, const est_config_t *cfg, est_cb_t on_state, est_cb_t on_init, void *user)
{
    memset(e, 0, sizeof *e);
    e->cfg      = *cfg;
    e->on_state = on_state;
    e->on_init  = on_init;
    e->user     = user;
    calib_reset(&e->cal);
    accwin_reset(&e->win);
}

static void on_step(est_t *e, float h)
{
    float gm[3];
    const float inv = 1.0f / (float)e->k;
    for (int i = 0; i < 3; ++i) {
        gm[i] = e->gsum[i] * inv;
        e->gwin_sum[i] += gm[i];
    }
    e->gwin_n++;

    if (e->running) {
        const float pv[4] = { e->dth[0], e->dth[1], e->dth[2], h };
        trace_put(e, EST_OP_PROP, pv, 4, 0);
        iekf_propagate_dtheta(&e->f, e->dth, h);
        accwin_rotate(&e->win, e->f.E_last);
        e->t += h;

        ctrl_rate_in_t in;
        for (int i = 0; i < 3; ++i) in.gyro[i] = e->omega[i] = gm[i] - e->f.b[i];
        in.dt = h; in.step = e->step;
        ctrl_rate_step(&in);
    }
    e->step++;
    memset(e->dth, 0, sizeof e->dth);
    memset(e->gsum, 0, sizeof e->gsum);
    e->k = 0;
}

static void on_acc_window(est_t *e, float h_win)
{
    if (accwin_mean(&e->win, e->acc_mean) == 0) e->have_acc_mean = 1;
    accwin_reset(&e->win);
    if (e->gwin_n) {
        for (int i = 0; i < 3; ++i) e->gyro_mean[i] = e->gwin_sum[i] / (float)e->gwin_n;
    }
    memset(e->gwin_sum, 0, sizeof e->gwin_sum);
    e->gwin_n = 0;

    if (!e->running || !e->have_acc_mean) return;

    if (e->cfg.still_time_s > 0.0f) {
        /* RAW gyro, not corrected with the estimated bias: after a large error the
         * bias estimate is itself perturbed and would blind the detector.
         * The real LSM6DSV16X bias is well below the threshold. */
        const float *w = e->gyro_mean;
        const float dev = fabsf(vec3_norm(e->acc_mean) - e->f.nom_acc) / e->f.nom_acc;
        if (dev < e->cfg.still_acc_tol && vec3_norm(w) < e->cfg.still_gyro_thr)
            e->still_t += h_win;
        else
            e->still_t = 0.0f;
        if (e->still_t >= e->cfg.still_time_s) {
            e->f.acc_trusted = 1;            /* applies to the next update */
            e->flags |= EST_F_STILL;
        }
    }

    trace_put(e, EST_OP_ACC, e->acc_mean, 3, e->f.acc_trusted);
    if (iekf_update_acc(&e->f, e->acc_mean)) e->flags |= EST_F_ACC_GATED;

    ctrl_att_in_t in;
    iekf_get_quat(&e->f, in.q);
    in.dt = h_win; in.step = e->step;
    ctrl_attitude_step(&in);

    e->flags |= EST_F_RUNNING;
    if (e->on_state) e->on_state(e, e->user);
    e->flags = 0;
    memset(&e->trace, 0, sizeof e->trace);
    e->win_count++;
}

static void on_mag(est_t *e, const float m_sensor[3])
{
    const float *M = e->cfg.mag_map;
    for (int i = 0; i < 3; ++i)
        e->mag[i] = M[3*i]*m_sensor[0] + M[3*i + 1]*m_sensor[1] + M[3*i + 2]*m_sensor[2]
                    - e->cfg.mag_hard_iron[i];

    if (e->running) {
        /* A world-fixed vector seen from the body evolves as v' = -omega x v:
         * a measurement tau seconds old is brought to "now" with y - tau*(omega x y). */
        float y[3], wxy[3];
        vec3_cross(e->omega, e->mag, wxy);
        for (int i = 0; i < 3; ++i) y[i] = e->mag[i] - e->cfg.mag_latency_s * wxy[i];
        const float nm  = vec3_norm(e->mag);
        const float wxu = (nm > 1e-6f) ? vec3_norm(wxy) / nm : 0.0f;  /* |omega x u| */
        const float sd  = wxu * e->cfg.mag_latency_std_s;
        e->flags |= EST_F_MAG_NEW;
        const float mv[4] = { y[0], y[1], y[2], sd*sd };
        trace_put(e, EST_OP_MAG, mv, 4, 0);
        if (iekf_update_mag_ex(&e->f, y, sd*sd)) e->flags |= EST_F_MAG_GATED;
        return;
    }

    /* ---- static calibration: one sample per new mag reading ---- */
    if (!e->have_acc_mean) return;
    if (calib_add(&e->cal, e->acc_mean, e->gyro_mean, e->mag, e->cfg.calib_gyro_still) != 0) {
        calib_reset(&e->cal);                       /* it moved: start over       */
        return;
    }
    if (calib_count(&e->cal) < e->cfg.calib_samples) return;

    if (calib_finish(&e->cal, e->cfg.sigma_vec_min, &e->calres) != 0) {
        calib_reset(&e->cal);
        return;
    }
    iekf_config_t c = e->cfg.ekf;
    c.sigma_acc = e->calres.sigma_acc;
    c.sigma_mag = e->calres.sigma_mag;
    e->cfg.ekf  = c;
    iekf_init(&e->f, &c, &e->calres.seed);
    accwin_reset(&e->win);
    e->running = 1;
    e->t = 0.0f;
    e->win_count = 0;
    memset(&e->trace, 0, sizeof e->trace);
    if (e->on_init) e->on_init(e, e->user);
}

void est_process(est_t *e, const imu_event_t *ev, size_t n, float dt)
{
    for (size_t j = 0; j < n; ++j) {
        switch (ev[j].type) {
        case IMU_EV_GYRO:
            for (int i = 0; i < 3; ++i) {
                e->dth[i]  += ev[j].v[i] * dt;
                e->gsum[i] += ev[j].v[i];
            }
            if (++e->k >= e->cfg.gyro_per_step) {
                const float h = (float)e->k * dt;
                on_step(e, h);
                if ((e->step % e->cfg.steps_per_acc) == 0)
                    on_acc_window(e, h * (float)e->cfg.steps_per_acc);
            }
            break;
        case IMU_EV_ACC:
            accwin_add(&e->win, ev[j].v);
            break;
        case IMU_EV_MAG:
            on_mag(e, ev[j].v);
            break;
        default:
            break;
        }
    }
}
