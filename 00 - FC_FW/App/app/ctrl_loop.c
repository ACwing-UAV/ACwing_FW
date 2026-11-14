/**
 * @file  ctrl_loop.c
 * @brief Phase 3: the attitude and rate loops and the mixer in the ImuTask (strong definitions of the
 *        control_hooks.h hooks).
 *
 *   est_pipeline, 240 Hz  ctrl_attitude_step(): setpoints of the flight core (fc_setpoint, same task) and the
 *                         IEKF attitude -> body-rate setpoint; gains from the parameters (5 Hz); battery (V/cell)
 *   est_pipeline, 960 Hz  ctrl_rate_step(): gyro -> torques -> mixer -> motors_set()
 *
 * Everything runs in the ImuTask, the task that also runs fc_service() and feeds the watchdog: no lock. The
 * motors are written only while armed. Disarmed, motors_arm(0) (fc_core) has already stopped them and a motor
 * test (MOTOR_TEST, disarmed only) owns the outputs: this file must not overwrite them.
 *
 * On the ground the loops do not integrate. At idle (collective at most idle + 2 %) the torques are zero, so a
 * drone armed on a slope does not spin up one side.
 */
#include "control_hooks.h"
#include "att_ctrl.h"
#include "mixer.h"
#include "fc_core.h"
#include "params.h"
#include "motors.h"
#include "battery.h"
#include "proto_v2.h"
#include <string.h>

#define DEG2RAD 0.017453292519943295f

_Static_assert(MOTORS_N == MIXER_MOTORS, "the quad-X mixer drives 4 motors");

static att_ctrl_t     s_att;
static mixer_cfg_t    s_mix;
static fc_setpoint_t  s_sp;
static float          s_vcell;                     /* V per cell under load, 0 = unknown */
static float          s_trim[2];                   /* rad: LEVEL calibration roll, pitch  */
static uint8_t        s_was_armed;
static uint32_t       s_n_att;
static volatile uint8_t s_pct[MIXER_MOTORS];
static uint32_t       s_rate_steps, s_seen_steps, s_seen_ms;
static uint8_t        s_have_seen, s_stalled;

static void gains_from_params(att_gains_t *g)
{
    att_gains_default(g);
    g->rate_p[0] = g->rate_p[1] = PARAM(P_RATE_P);
    g->rate_i[0] = g->rate_i[1] = PARAM(P_RATE_I);
    g->rate_d[0] = g->rate_d[1] = PARAM(P_RATE_D);
    g->rate_p[2] = PARAM(P_YAW_P);
    g->rate_i[2] = PARAM(P_YAW_I);
    g->att_p = PARAM(P_ATT_P);
    g->gyro_lpf_hz = PARAM(P_GYRO_LPF);
    g->dterm_lpf_hz = PARAM(P_DTERM_LPF);
    g->notch_hz = PARAM(P_GYRO_NOTCH);
}

void ctrl_init(void)
{
    att_gains_t g;
    gains_from_params(&g);
    att_ctrl_init(&s_att, &g);
    s_mix.idle = PARAM(P_MOTOR_IDLE);
    s_mix.theta = PARAM(P_THR_CURVE);
    memset(&s_sp, 0, sizeof s_sp);
    s_vcell = 0.0f; s_was_armed = 0u; s_n_att = 0u;
    s_rate_steps = s_seen_steps = s_seen_ms = 0u; s_have_seen = 0u; s_stalled = 0u;
    for (unsigned i = 0; i < MIXER_MOTORS; ++i) s_pct[i] = 0u;
}

void ctrl_set_level_trim(float roll_deg, float pitch_deg)
{
    s_trim[0] = roll_deg * DEG2RAD;
    s_trim[1] = pitch_deg * DEG2RAD;
}

uint8_t ctrl_motor_pct(uint8_t m) { return (m < MIXER_MOTORS) ? s_pct[m] : 0u; }

void ctrl_attitude_step(const ctrl_att_in_t *in)
{
    /* parameters and battery at 5 Hz: the AppTask writes the parameters with atomic stores */
    if ((s_n_att++ % 48u) == 0u) {
        att_gains_t g;
        gains_from_params(&g);
        att_ctrl_set_gains(&s_att, &g);
        s_mix.idle = PARAM(P_MOTOR_IDLE);
        s_mix.theta = PARAM(P_THR_CURVE);
    }
    {
        batt_state_t b;
        memset(&b, 0, sizeof b);
        batt_read(&b);
        s_vcell = (b.present && b.cells) ? b.cell_v : 0.0f;          /* sagged, filtered 0.5 s (battery.c) */
    }
    fc_setpoint(&s_sp);
    att_sp_t sp;
    memset(&sp, 0, sizeof sp);
    sp.acro = (uint8_t)(s_sp.mode == PV2_MODE_ACRO && s_sp.autom == PV2_AUTO_NONE);
    /* LEVEL calibration: trim = -(board attitude when the frame is level), so the board must hold sp - trim */
    sp.roll = s_sp.roll - s_trim[0];
    sp.pitch = s_sp.pitch - s_trim[1];
    sp.roll_rate = s_sp.roll_rate; sp.pitch_rate = s_sp.pitch_rate; sp.yaw_rate = s_sp.yaw_rate;
    att_ctrl_attitude(&s_att, in->q, &sp);
}

uint8_t ctrl_service(uint32_t now)
{
    if (!s_have_seen || s_rate_steps != s_seen_steps) {
        s_seen_steps = s_rate_steps; s_seen_ms = now; s_have_seen = 1u;
        s_stalled = 0u;
        return 0u;
    }
    if (s_sp.armed && (now - s_seen_ms) > CTRL_STALL_MS) {
        if (!s_stalled) {
            static const float off[MIXER_MOTORS] = { 0 };
            motors_set(off);
            for (unsigned i = 0; i < MIXER_MOTORS; ++i) s_pct[i] = 0u;
        }
        s_stalled = 1u;
    }
    return s_stalled;
}

void ctrl_rate_step(const ctrl_rate_in_t *in)
{
    s_rate_steps++;
    if (!s_sp.armed) {
        if (s_was_armed) {
            att_ctrl_reset(&s_att);
            for (unsigned i = 0; i < MIXER_MOTORS; ++i) s_pct[i] = 0u;
        }
        s_was_armed = 0u;
        /* keep the gyro filters running, so they are settled at the next arm */
        float dummy[3];
        att_ctrl_rate(&s_att, in->gyro, 0u, dummy);
        return;
    }
    if (!s_was_armed) att_ctrl_reset(&s_att);
    s_was_armed = 1u;

    float torque[3];
    att_ctrl_rate(&s_att, in->gyro, s_sp.in_air, torque);
    if (s_sp.motors_off) {
        /* armed in the hand (hand launch, ADR 0011): duty 0, not the idle; loops ready from zero */
        static const float zero[MIXER_MOTORS] = { 0.0f, 0.0f, 0.0f, 0.0f };
        att_ctrl_reset(&s_att);
        motors_set(zero);
        for (unsigned i = 0; i < MIXER_MOTORS; ++i) s_pct[i] = 0u;
        return;
    }
    if (!s_sp.in_air && s_sp.thrust <= s_mix.idle + 0.02f) {
        att_ctrl_reset(&s_att);                     /* idle on the ground: no torque, no memory */
        torque[0] = torque[1] = torque[2] = 0.0f;
    }
    mixer_out_t mo;
    mixer_quadx(s_sp.thrust, torque, s_vcell, &s_mix, &mo);
    motors_set(mo.duty);
    for (unsigned i = 0; i < MIXER_MOTORS; ++i) s_pct[i] = (uint8_t)(mo.duty[i] * 100.0f + 0.5f);
}
