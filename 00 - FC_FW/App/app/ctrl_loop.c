/**
 * @file  ctrl_loop.c
 * @brief The actuator layer of the control (ADR 0013) and the strong definitions of the control_hooks.h hooks.
 *
 *   est_pipeline, 240 Hz  ctrl_attitude_step(): attitude and cell voltage to the law (ctl_io), its attitude step;
 *                         mixer parameters (5 Hz)
 *   est_pipeline, 960 Hz  ctrl_rate_step(): gyro to the law, its rate step, then its command -> allocation (quad-X
 *                         mixer, or the law's own per-motor thrust) -> thrust curve, battery -> motors_set()
 *
 * The control law itself (which loops, which gains) is behind ctl_api.h and can be replaced (ctl_register,
 * parameter ctlLaw). What stays here applies to every law and cannot be bypassed by one:
 *   - the motors are written only while armed. Disarmed, motors_arm(0) (fc_core) has already stopped them and a
 *     motor test (MOTOR_TEST, disarmed only) owns the outputs: this file must not overwrite them;
 *   - motors_off (armed in the hand, hand launch) and a rate step without a command: duty 0;
 *   - the stall watchdog (ctrl_service): no rate step for CTRL_STALL_MS while armed -> motors off.
 * Everything runs in the ImuTask, the task that also runs fc_service() and feeds the watchdog: no lock.
 */
#include "control_hooks.h"
#include "ctl_io.h"
#include "mixer.h"
#include "params.h"
#include "motors.h"
#include "battery.h"
#include <string.h>

#define DEG2RAD 0.017453292519943295f

_Static_assert(MOTORS_N == MIXER_MOTORS, "the quad-X mixer drives 4 motors");

static mixer_cfg_t    s_mix;
static ctl_ref_t      s_r;                         /* reference seen at the last attitude step */
static float          s_vcell;                     /* V per cell under load, 0 = unknown */
static uint8_t        s_was_armed;
static uint32_t       s_n_att;
static volatile uint8_t s_pct[MIXER_MOTORS];
static uint32_t       s_rate_steps, s_seen_steps, s_seen_ms;
static uint8_t        s_have_seen, s_stalled;

static void mixer_from_params(void)
{
    s_mix.idle = PARAM(P_MOTOR_IDLE);
    s_mix.theta = PARAM(P_THR_CURVE);
}

void ctrl_init(void)
{
    mixer_from_params();
    memset(&s_r, 0, sizeof s_r);
    s_vcell = 0.0f; s_was_armed = 0u; s_n_att = 0u;
    s_rate_steps = s_seen_steps = s_seen_ms = 0u; s_have_seen = 0u; s_stalled = 0u;
    for (unsigned i = 0; i < MIXER_MOTORS; ++i) s_pct[i] = 0u;
}

void ctrl_set_level_trim(float roll_deg, float pitch_deg)
{
    ctl_io_set_level_trim(roll_deg * DEG2RAD, pitch_deg * DEG2RAD);
}

uint8_t ctrl_motor_pct(uint8_t m) { return (m < MIXER_MOTORS) ? s_pct[m] : 0u; }

void ctrl_attitude_step(const ctrl_att_in_t *in)
{
    /* parameters and battery at 5 Hz: the AppTask writes the parameters with atomic stores */
    if ((s_n_att++ % 48u) == 0u) mixer_from_params();
    {
        batt_state_t b;
        memset(&b, 0, sizeof b);
        batt_read(&b);
        s_vcell = (b.present && b.cells) ? b.cell_v : 0.0f;          /* sagged, filtered 0.5 s (battery.c) */
    }
    ctl_io_feed_cell_voltage(s_vcell);
    ctl_io_feed_attitude(in->q);
    ctl_io_attitude(in->dt);
    ctl_get_ref(&s_r);
}

static void outputs_zero(void)
{
    static const float zero[MIXER_MOTORS] = { 0.0f, 0.0f, 0.0f, 0.0f };
    motors_set(zero);
    for (unsigned i = 0; i < MIXER_MOTORS; ++i) s_pct[i] = 0u;
}

void ctrl_rate_step(const ctrl_rate_in_t *in)
{
    s_rate_steps++;
    ctl_io_feed_rates(in->gyro);
    ctl_io_rate(in->dt);                           /* the law runs disarmed too (its filters stay settled) */
    ctl_cmd_t c;
    ctl_io_take_cmd(&c);
    if (!s_r.armed) {
        if (s_was_armed) for (unsigned i = 0; i < MIXER_MOTORS; ++i) s_pct[i] = 0u;
        s_was_armed = 0u;
        return;                                    /* never written disarmed */
    }
    s_was_armed = 1u;
    if (s_r.motors_off || c.kind == CTL_CMD_NONE) { outputs_zero(); return; }
    mixer_out_t mo;
    if (c.kind == CTL_CMD_MOTORS) mixer_direct(c.motor, s_vcell, &s_mix, &mo);
    else                          mixer_quadx(c.collective, c.torque, s_vcell, &s_mix, &mo);
    motors_set(mo.duty);
    for (unsigned i = 0; i < MIXER_MOTORS; ++i) s_pct[i] = (uint8_t)(mo.duty[i] * 100.0f + 0.5f);
}

uint8_t ctrl_service(uint32_t now)
{
    if (!s_have_seen || s_rate_steps != s_seen_steps) {
        s_seen_steps = s_rate_steps; s_seen_ms = now; s_have_seen = 1u;
        s_stalled = 0u;
        return 0u;
    }
    if (s_r.armed && (now - s_seen_ms) > CTRL_STALL_MS) {
        if (!s_stalled) outputs_zero();
        s_stalled = 1u;
    }
    return s_stalled;
}
