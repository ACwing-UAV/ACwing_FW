/**
 * @file  ctl_law_cascade.c
 * @brief The default control law (ADR 0013, index 0): the cascade of firmware 0.7-0.11, behind ctl_api.h only.
 *
 *   outer     250 Hz  vertical: alt_ctrl.c (height P -> climb-rate PI around the hover thrust / cos tilt);
 *                     horizontal: pos_ctrl.c (flow velocity PI, position P, tilt cone)
 *   attitude  240 Hz  att_ctrl.c tilt loop (reduced attitude) -> body-rate setpoint
 *   rate      960 Hz  att_ctrl.c rate PID -> torques; collective and torques to the actuators (mixer)
 *
 * The theory and the numbers are in theory/control-loops.md. This file only maps the reference of the flight
 * manager onto those loops; another law can replace it entirely (ctl_register(), parameter ctlLaw).
 */
#include "ctl_api.h"
#include "alt_ctrl.h"
#include "pos_ctrl.h"
#include "att_ctrl.h"
#include <math.h>
#include <string.h>

/* parameter ids used by this law (HANDOVER_FW §8) */
#define ID_RATE_P   90u
#define ID_RATE_I   91u
#define ID_RATE_D   92u
#define ID_YAW_P    93u
#define ID_YAW_I    94u
#define ID_ATT_P    95u
#define ID_GYRO_LPF 96u
#define ID_DTERM_LPF 97u
#define ID_NOTCH    98u
#define ID_IDLE     99u

static alt_ctrl_t s_ac;
static pos_ctrl_t s_pc;
static att_ctrl_t s_att;
static uint8_t    s_closed;          /* the vertical loop ran at the last outer step   */
static uint8_t    s_was_armed_outer, s_was_armed_rate;
static float      s_coll;            /* collective of the last outer step               */
static float      s_roll, s_pitch;   /* attitude targets of the last outer step         */
/* snapshot of the attitude step, used by the rate steps until the next one (as the firmware always did) */
static ctl_ref_t  s_r_att;
static float      s_coll_att;
static float      s_idle = 0.05f;
static uint32_t   s_n_att;

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static float par(uint16_t id, float def) { const float v = ctl_get_param(id); return (v == v) ? v : def; }

static void gains_from_params(att_gains_t *g)
{
    att_gains_default(g);
    g->rate_p[0] = g->rate_p[1] = par(ID_RATE_P, g->rate_p[0]);
    g->rate_i[0] = g->rate_i[1] = par(ID_RATE_I, g->rate_i[0]);
    g->rate_d[0] = g->rate_d[1] = par(ID_RATE_D, g->rate_d[0]);
    g->rate_p[2] = par(ID_YAW_P, g->rate_p[2]);
    g->rate_i[2] = par(ID_YAW_I, g->rate_i[2]);
    g->att_p = par(ID_ATT_P, g->att_p);
    g->gyro_lpf_hz = par(ID_GYRO_LPF, g->gyro_lpf_hz);
    g->dterm_lpf_hz = par(ID_DTERM_LPF, g->dterm_lpf_hz);
    g->notch_hz = par(ID_NOTCH, g->notch_hz);
}

static void cascade_init(void)
{
    alt_ctrl_init(&s_ac, ctl_get_hover_thrust());
    pc_gains_t pg; pc_gains_default(&pg); pc_init(&s_pc, &pg);
    att_gains_t g; gains_from_params(&g); att_ctrl_init(&s_att, &g);
    s_idle = par(ID_IDLE, 0.05f);
    s_n_att = 0u;
}

static void cascade_reset(void)
{
    alt_ctrl_reset(&s_ac, 0.0f);
    pc_reset(&s_pc);
    att_ctrl_reset(&s_att);
    s_closed = 0u; s_was_armed_outer = s_was_armed_rate = 0u;
    s_coll = s_coll_att = 0.0f; s_roll = s_pitch = 0.0f;
    memset(&s_r_att, 0, sizeof s_r_att);
}

/* ------------------------------------------------------------------ outer: vertical */
static float vertical(const ctl_ref_t *r, float dt, float *vz_out)
{
    const float vz = ctl_get_vz(), ct = ctl_get_cos_tilt();
    *vz_out = 0.0f;
    if (r->vert == CTL_V_COLLECTIVE) {
        s_closed = 0u;
        float thr = r->collective;
        if (r->vz_ceiling < CTL_NO_LIMIT) {
            /* geofence in the open-loop modes: the collective may not push the climb rate above vz_ceiling; the
             * vertical loop limits it only while it asks for less (ready for a bumpless change) */
            const float lim = alt_ctrl_step(&s_ac, r->vz_ceiling, vz, ct, dt);
            if (lim < thr) thr = lim;
            else alt_ctrl_reset(&s_ac, 0.0f);
        }
        return clampf(thr, 0.0f, 1.0f);
    }
    /* entering the loop (from open loop, at arm, after a hand launch): from the hover thrust, no old integrator */
    if (!s_closed) alt_ctrl_reset(&s_ac, 0.0f);
    s_closed = 1u;
    float vz_sp = (r->vert == CTL_V_HEIGHT) ? alt_ctrl_vz_sp(&s_ac, r->h_sp, ctl_get_height(), r->vz_up, r->vz_dn)
                                            : r->vz_sp;
    if (vz_sp > 0.0f && vz_sp > r->vz_ceiling) vz_sp = r->vz_ceiling;           /* at the ceiling */
    /* the take-off spool-up ramp is the upper limit: the integrator does not wind up on the ground */
    const float thr_max = s_ac.thr_max;
    if (r->thrust_max < s_ac.thr_max) s_ac.thr_max = r->thrust_max;
    const float thr = alt_ctrl_step(&s_ac, vz_sp, vz, ct, dt);
    s_ac.thr_max = thr_max;
    *vz_out = vz_sp;
    return thr;
}

/* ------------------------------------------------------------------ outer: horizontal */
static void horizontal(const ctl_ref_t *r, float dt)
{
    if (r->horiz != CTL_H_VELOCITY) {
        pc_reset(&s_pc);
        s_roll = r->roll; s_pitch = r->pitch;
        return;
    }
    pc_in_t in;
    ctl_get_vel_body(in.vel_b);
    in.yaw = ctl_get_yaw();
    in.v_max = r->v_max;
    in.stick_fwd = (r->v_max > 0.0f) ? r->vel_sp[0] / r->v_max : 0.0f;    /* pos_ctrl works in stick fractions */
    in.stick_left = (r->v_max > 0.0f) ? r->vel_sp[1] / r->v_max : 0.0f;
    in.tilt_max = r->tilt_max;
    in.dt = dt;
    pc_step(&s_pc, &in, &s_roll, &s_pitch);
}

static void cascade_outer(float dt)
{
    ctl_ref_t r;
    ctl_get_ref(&r);
    ctl_status_t st;
    memset(&st, 0, sizeof st);
    if (!r.armed) {
        s_closed = 0u; s_was_armed_outer = 0u; s_coll = 0.0f;
        pc_reset(&s_pc);
        s_roll = r.roll; s_pitch = r.pitch;
        st.roll = s_roll; st.pitch = s_pitch;
        ctl_set_status(&st);
        return;
    }
    if (!s_was_armed_outer) { alt_ctrl_reset(&s_ac, 0.0f); s_was_armed_outer = 1u; }
    s_ac.hover = ctl_get_hover_thrust();          /* the vertical loop works around the live hover estimate */
    s_coll = vertical(&r, dt, &st.vz_sp);
    horizontal(&r, dt);
    st.collective = s_coll;
    st.roll = s_roll; st.pitch = s_pitch;
    st.saturated = (uint8_t)(s_coll <= s_ac.thr_min + 0.01f || s_coll >= s_ac.thr_max - 0.01f);
    ctl_set_status(&st);
}

/* ------------------------------------------------------------------ attitude (240 Hz) */
static void cascade_attitude(float dt)
{
    (void)dt;
    if ((s_n_att++ % 48u) == 0u) {                /* parameters at 5 Hz */
        att_gains_t g;
        gains_from_params(&g);
        att_ctrl_set_gains(&s_att, &g);
        s_idle = par(ID_IDLE, 0.05f);
    }
    ctl_get_ref(&s_r_att);
    s_coll_att = s_coll;
    float q[4], trim[2];
    ctl_get_attitude(q);
    ctl_get_level_trim(trim);
    att_sp_t sp;
    memset(&sp, 0, sizeof sp);
    sp.acro = (uint8_t)(s_r_att.att == CTL_ATT_RATE);
    /* LEVEL calibration: trim = -(board attitude when the frame is level), so the board must hold sp - trim */
    sp.roll = s_roll - trim[0];
    sp.pitch = s_pitch - trim[1];
    sp.roll_rate = s_r_att.rate[0]; sp.pitch_rate = s_r_att.rate[1]; sp.yaw_rate = s_r_att.rate[2];
    att_ctrl_attitude(&s_att, q, &sp);
}

/* ------------------------------------------------------------------ rate (960 Hz) */
static void cascade_rate(float dt)
{
    (void)dt;
    float w[3], torque[3];
    ctl_get_rates(w);
    if (!s_r_att.armed) {
        if (s_was_armed_rate) att_ctrl_reset(&s_att);
        s_was_armed_rate = 0u;
        att_ctrl_rate(&s_att, w, 0u, torque);      /* the gyro filters keep running, settled at the next arm */
        return;                                    /* no command: the actuators write nothing while disarmed */
    }
    if (!s_was_armed_rate) att_ctrl_reset(&s_att);
    s_was_armed_rate = 1u;
    att_ctrl_rate(&s_att, w, s_r_att.in_air, torque);
    if (s_r_att.motors_off) {                      /* in the hand (hand launch): loops ready from zero */
        static const float zero[3] = { 0.0f, 0.0f, 0.0f };
        att_ctrl_reset(&s_att);
        ctl_set_thrust_torque(0.0f, zero);
        return;
    }
    if (!s_r_att.in_air && s_coll_att <= s_idle + 0.02f) {
        att_ctrl_reset(&s_att);                    /* idle on the ground: no torque, no memory */
        torque[0] = torque[1] = torque[2] = 0.0f;
    }
    ctl_set_thrust_torque(s_coll_att, torque);
}

const ctl_law_t g_ctl_law_cascade = {
    "cascade_pid", cascade_init, cascade_reset, cascade_outer, cascade_attitude, cascade_rate,
};
