/**
 * @file  tlm_build.c
 * Note: fields of the packed structs are ALWAYS filled via memcpy from aligned
 * local variables (see telemetry.h: unaligned VSTR -> HardFault).
 */
#include "tlm_build.h"
#include "so3.h"
#include <string.h>

size_t tlm_build_state(const est_t *e, uint8_t *frame, size_t cap)
{
    tlm_state_t p;
    float q[4];
    p.seq  = (uint16_t)(e->step / e->cfg.steps_per_acc);
    p.t_us = (uint32_t)(e->t * 1e6f);
    iekf_get_quat(&e->f, q);
    memcpy(p.q,    q,            sizeof q);
    memcpy(p.bias, e->f.b,       sizeof p.bias);
    memcpy(p.acc,  e->acc_mean,  sizeof p.acc);
    memcpy(p.gyro, e->gyro_mean, sizeof p.gyro);
    memcpy(p.mag,  e->mag,       sizeof p.mag);
    p.flags = e->flags;
    return tlm_encode(TLM_TYPE_STATE, &p, sizeof p, frame, cap);
}

size_t tlm_build_init(const est_t *e, float fs_state_hz, uint8_t *frame, size_t cap)
{
    const calib_result_t *cr = &e->calres;
    const iekf_config_t  *c  = &e->cfg.ekf;
    tlm_init_t p;
    float q0[4], v;
    so3_to_quat(cr->seed.R0, q0);
    memcpy(p.q0,    q0,            sizeof q0);
    memcpy(p.g_ref, cr->seed.g_ref, sizeof p.g_ref);
    memcpy(p.m_ref, cr->seed.m_ref, sizeof p.m_ref);
    memcpy(p.b0,    cr->seed.b0,    sizeof p.b0);
#define PUT(field, val) do { v = (val); memcpy(&p.field, &v, sizeof v); } while (0)
    PUT(sigma_acc, c->sigma_acc);     PUT(sigma_mag, c->sigma_mag);
    PUT(sigma_gyro, c->sigma_gyro);   PUT(sigma_bias, c->sigma_bias);
    PUT(nom_acc, cr->seed.nom_acc);   PUT(nom_mag, cr->seed.nom_mag);
    PUT(fs_hz, fs_state_hz);
    PUT(acc_tol, c->acc_tol);         PUT(mag_tol, c->mag_tol);
    PUT(p0_att, c->p0_att);           PUT(p0_bias, c->p0_bias);
    PUT(acc_nis_gate, c->acc_nis_gate); PUT(mag_nis_gate, c->mag_nis_gate);
    PUT(infl_decay, c->infl_decay);   PUT(dip_tol, c->dip_tol);
    PUT(infl_max, c->infl_max);       PUT(lockout_p_bump, c->lockout_p_bump);
    PUT(persist_decay, c->persist_decay); PUT(persist_ratio, c->persist_ratio);
    PUT(persist_min, c->persist_min);
#undef PUT
    {
        uint16_t u = c->lockout_updates; memcpy(&p.lockout_updates, &u, 2);
        u = c->grace_updates;            memcpy(&p.grace_updates, &u, 2);
    }
    p.flags = (uint8_t)((c->estimate_bias  ? TLM_I_EST_BIAS  : 0u) |
                        (c->adaptive       ? TLM_I_ADAPTIVE  : 0u) |
                        (c->tilt_comp_mag  ? TLM_I_TILT_COMP : 0u) |
                        (c->decouple       ? TLM_I_DECOUPLE  : 0u) |
                        (c->reset_jacobian ? TLM_I_RESET_JAC : 0u));
    return tlm_encode(TLM_TYPE_INIT, &p, sizeof p, frame, cap);
}

size_t tlm_build_replay(const est_t *e, uint8_t *frame, size_t cap)
{
    uint8_t pl[TLM_REPLAY_MAX];
    size_t  n = 0;
    const uint32_t win = e->win_count;
    memcpy(&pl[n], &win, 4); n += 4;
    pl[n++] = e->trace.nops;
    pl[n++] = e->trace.overflow ? 1u : 0u;
    memcpy(&pl[n], e->trace.buf, e->trace.len); n += e->trace.len;

    float tail[15];
    iekf_get_quat(&e->f, &tail[0]);
    for (int i = 0; i < 3; ++i) tail[4 + i] = e->f.b[i];
    for (int i = 0; i < 6; ++i) tail[7 + i] = e->f.P[i*IEKF_NX + i];
    tail[13] = e->f.acc_infl;
    tail[14] = e->f.mag_infl;
    memcpy(&pl[n], tail, sizeof tail); n += sizeof tail;
    return tlm_encode(TLM_TYPE_REPLAY, pl, (uint8_t)n, frame, cap);
}
