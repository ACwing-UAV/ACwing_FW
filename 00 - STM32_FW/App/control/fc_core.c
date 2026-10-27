/**
 * @file  fc_core.c
 * @brief Flight core (see fc_core.h). Behavioural reference: the app
 *        simulator src/sim/simDrone.ts (exec(), stepFlight()).
 */
#include "fc_core.h"
#include "app_link.h"
#include "params.h"
#include "proto_v2.h"
#include "spsc.h"
#include "alt_ctrl.h"
#include <string.h>
#include <math.h>

#define DEG2RAD 0.017453292519943295f

volatile uint8_t g_fc_dbg_in_air;

SPSC_DEFINE(s_cmdq, fc_cmd_t,   8u);
SPSC_DEFINE(s_resq, fc_res_t,   8u);
SPSC_DEFINE(s_evq,  fc_event_t, 16u);

static fc_config_t s_cfg;
static volatile uint16_t s_sensors;

/* ---- ground information from the AppTask ---- */
static volatile uint8_t s_checks_ready, s_batt_fail, s_calibrating;

/* ---- flight state (owned by the flight-control context) ---- */
static uint8_t  s_armed, s_mode = PV2_MODE_ANGLE, s_auto, s_failsafe, s_in_air;
static uint8_t  s_fs_long, s_fs_short_hold;
static uint8_t  s_took_off;
static uint32_t s_arm_ms, s_landed_ms, s_flight_ms, s_last_ms;
static uint8_t  s_have_last;
static appl_rc_t s_rc;
static uint32_t s_rc_ms;
static uint8_t  s_have_rc;
static float    s_last_thr = -1.0f;
static float    s_stick[4];
static uint8_t  s_rc_stale = 1;
static uint32_t s_silence;

/* vertical state and automatic flight */
static fc_vert_t  s_v;
static uint8_t    s_have_v;
static float      s_h_ref;          /* take-off point (FLIGHT.altRel = 0)     */
static float      s_h_ground;       /* ground level for lift-off detection    */
static float      s_h_sp;           /* height held                             */
static float      s_to_target;      /* TAKEOFF target height (h frame)         */
static uint32_t   s_to_t0;
static uint8_t    s_to_lifted, s_dbg_forced, s_landed, s_geofence, s_geofence_ev;
static uint32_t   s_touch_ms, s_flow_bad_ms;
static alt_ctrl_t s_ac;
static fc_setpoint_t s_sp;

/* motor test */
static uint8_t  s_test_motor, s_test_pct;
static uint32_t s_test_until;

/* ---- published snapshot ---- */
static volatile uint32_t s_snap_seq;
static fc_state_t        s_snap;

/* ================================================================== */
void fc_init(const fc_config_t *cfg)
{
    s_cfg = *cfg;
    s_sensors = cfg->sensors;
    if (s_cfg.n_motors == 0u || s_cfg.n_motors > 8u) s_cfg.n_motors = 4u;
    spsc_reset(&s_cmdq); spsc_reset(&s_resq); spsc_reset(&s_evq);
    s_armed = s_auto = s_failsafe = s_in_air = 0u;
    s_mode = PV2_MODE_ANGLE;
    s_fs_long = s_fs_short_hold = s_took_off = 0u;
    s_flight_ms = 0u; s_have_last = 0u; s_have_rc = 0u;
    s_test_motor = 0u;
    s_checks_ready = s_batt_fail = s_calibrating = 0u;
    g_fc_dbg_in_air = 0u;
    s_have_v = 0u; s_dbg_forced = 0u; s_landed = 0u; s_geofence = s_geofence_ev = 0u;
    s_touch_ms = s_flow_bad_ms = 0u;
    alt_ctrl_init(&s_ac, 0.45f);
    memset(&s_sp, 0, sizeof s_sp);
}

void fc_set_sensors(uint16_t sensors) { s_sensors = sensors; }
void fc_setpoint(fc_setpoint_t *out)  { *out = s_sp; }

uint8_t fc_is_core_op(uint8_t op)
{
    switch (op) {
    case PV2_OP_ARM: case PV2_OP_DISARM: case PV2_OP_TAKEOFF: case PV2_OP_LAND:
    case PV2_OP_RTH: case PV2_OP_HOLD: case PV2_OP_KILL: case PV2_OP_SET_MODE:
    case PV2_OP_MOTOR_TEST: case FC_OP_MOTOR_STOP:
        return 1u;
    default:
        return 0u;
    }
}

int fc_post(const fc_cmd_t *c)            { return spsc_push(&s_cmdq, c); }
int fc_take_result(fc_res_t *r)           { return spsc_pop(&s_resq, r); }
int fc_take_event(fc_event_t *e)          { return spsc_pop(&s_evq, e); }

void fc_set_ground_info(uint8_t ready, uint8_t batt_fail, uint8_t calibrating)
{
    s_checks_ready = ready; s_batt_fail = batt_fail; s_calibrating = calibrating;
}

void fc_snapshot(fc_state_t *out)
{
    for (int tries = 0; tries < 8; ++tries) {
        const uint32_t s0 = s_snap_seq;
        if (s0 & 1u) continue;
        SPSC_BARRIER();
        fc_state_t v = s_snap;
        SPSC_BARRIER();
        if (s_snap_seq == s0) { *out = v; return; }
    }
    /* writer kept busy (cannot happen at 250 Hz): leave *out unchanged */
}

/* ================================================================== */
static void event(uint8_t level, uint16_t code, int32_t arg)
{
    const fc_event_t e = { level, code, arg };
    (void)spsc_push(&s_evq, &e);             /* full: the AppTask is stalled, nothing to do */
}

static uint8_t has(uint16_t bits) { return (uint8_t)((s_sensors & bits) != 0u); }
static uint8_t gnss(void)         { return has(PV2_S_GNSS); }
static uint8_t home_set(void)     { return 0u; }   /* no GNSS on this hardware yet */
static uint8_t alt_sensor(void)   { return has(PV2_S_BARO | PV2_S_TOF); }
static uint8_t alt_ok(void)       { return (uint8_t)(alt_sensor() && s_have_v && s_v.valid); }
static uint8_t indoor(void)       { return (uint8_t)(has(PV2_S_TOF) && !gnss()); }
static float   clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

/* top of the allowed space, in the h frame; *kind = EVENT 23 arg (0 alt, 1 ceiling) */
static float ceiling_h_kind(int32_t *kind)
{
    float c = s_h_ref + PARAM(P_GF_MAX_ALT);
    int32_t k = 0;
    if (indoor() && s_v.tof_valid) {
        const float floor_h = s_v.h - s_v.tof_h;
        const float ci = floor_h + PARAM(P_GF_CEILING);
        if (ci < c) { c = ci; k = 1; }
    }
    if (kind) *kind = k;
    return c;
}
static float ceiling_h(void) { return ceiling_h_kind(0); }

static void lift_off(void)
{
    s_in_air = 1u; s_took_off = 1u; s_landed = 0u; s_touch_ms = 0u;
}

static void touch_down(uint32_t now)
{
    s_in_air = 0u;
    s_landed = 1u;
    if (s_auto == PV2_AUTO_LAND || s_auto == PV2_AUTO_RTH) event(PV2_EV_INFO, PV2_E_LANDED, 0);
    s_auto = PV2_AUTO_NONE;
    s_landed_ms = now;
    s_touch_ms = 0u;
}
static uint8_t hold_mode(void)    { return s_mode == PV2_MODE_ALT_HOLD || s_mode == PV2_MODE_POS_HOLD; }

static void test_stop(void)
{
    if (s_test_motor && s_cfg.motor_test) s_cfg.motor_test(0u, 0.0f);
    s_test_motor = 0u;
}

static void disarm(void)
{
    test_stop();
    if (s_cfg.motors_arm) s_cfg.motors_arm(0u);
    const uint8_t was = s_armed;
    s_armed = 0u;
    s_auto = PV2_AUTO_NONE;
    s_failsafe = 0u;
    s_fs_long = s_fs_short_hold = 0u;
    s_in_air = 0u;                 /* motors off: not flying any more */
    s_dbg_forced = 0u;
    s_geofence = 0u;
    if (was) event(PV2_EV_INFO, PV2_E_DISARMED, 0);
}

/* failsafe action value -> automatic action; RTH without GNSS + home -> land */
static uint8_t fs_action(int p_idx)
{
    const int a = (int)(PARAM(p_idx) + 0.5f);
    if (a == 2) return (gnss() && home_set()) ? PV2_AUTO_RTH : PV2_AUTO_LAND;
    if (a == 1) return PV2_AUTO_LAND;
    return PV2_AUTO_HOLD;
}

/* ================================================================== */
static void result(const fc_cmd_t *c, uint8_t res, uint8_t why)
{
    if (c->op == FC_OP_MOTOR_STOP) return;   /* internal: no ACK */
    const fc_res_t r = { c->op, c->seq, res, why };
    (void)spsc_push(&s_resq, &r);
}

static void exec(const fc_cmd_t *c, uint32_t now, uint8_t est_running)
{
#define OK()        result(c, PV2_ACK_OK, PV2_R_NONE)
#define REJ(why)    result(c, PV2_ACK_REJECTED, (why))
#define INVALID(w)  result(c, PV2_ACK_INVALID, (w))
    const uint8_t *p = c->p;

    /* a running motor test stops on any other command */
    if (c->op != PV2_OP_MOTOR_TEST) test_stop();

    switch (c->op) {
    case PV2_OP_ARM: {
        if (c->len < 2u || p[0] != PV2_ARM_MAGIC0 || p[1] != PV2_ARM_MAGIC1) { INVALID(PV2_R_NONE); return; }
        if (s_armed)       { REJ(PV2_R_ARMED); return; }
        if (s_calibrating) { REJ(PV2_R_CALIBRATING); return; }
        if (s_in_air)      { REJ(PV2_R_NOT_LANDED); return; }
        /* ELRS in control -> RADIO_OVERRIDE: no receiver on this hardware */
        if (s_have_rc && (now - s_rc_ms) < FC_ARM_RC_FRESH_MS &&
            s_rc.thr > (hold_mode() ? 100 : -900)) { REJ(PV2_R_THROTTLE_NOT_LOW); return; }
        if (s_batt_fail) { REJ(PV2_R_BATTERY_LOW); return; }
        /* never arm without a running attitude estimate, whatever armRequireChecks says */
        if (!est_running || (PARAM(P_ARM_REQUIRE_CHECKS) > 0.5f && !s_checks_ready)) {
            REJ(PV2_R_CHECKS_FAILING); return;
        }
        s_armed = 1u;
        s_auto = PV2_AUTO_NONE; s_failsafe = 0u; s_fs_long = 0u;
        s_took_off = 0u; s_arm_ms = now; s_flight_ms = 0u;
        s_landed = 0u; s_geofence_ev = 0u;
        if (s_have_v && s_v.valid) { s_h_ref = s_h_ground = s_h_sp = s_v.h; }
        alt_ctrl_reset(&s_ac, 0.0f);
        if (s_cfg.motors_arm) s_cfg.motors_arm(1u);
        OK();
        event(PV2_EV_INFO, PV2_E_ARMED, 0);
        return;
    }
    case PV2_OP_DISARM:
        if (!s_armed) { OK(); return; }
        if (s_in_air) { REJ(PV2_R_NOT_LANDED); return; }
        OK();
        disarm();
        return;
    case PV2_OP_TAKEOFF: {
        if (!alt_sensor()) { result(c, PV2_ACK_UNSUPPORTED, PV2_R_SENSOR_MISSING); return; }
        if (!s_armed) { REJ(PV2_R_NOT_ARMED); return; }
        if (s_in_air) { REJ(PV2_R_NOT_LANDED); return; }
        if (!alt_ok()) { REJ(PV2_R_SENSOR_MISSING); return; }     /* baro/ToF not ready yet */
        float alt = (c->len >= 2u) ? (float)pv2_rd_u16(p) / 100.0f : 0.0f;
        if (alt <= 0.0f) alt = PARAM(P_TAKEOFF_ALT);
        if (alt < 0.5f) alt = 0.5f;
        if (alt > 3.0f) alt = 3.0f;
        /* climb from where the drone stands, never above the ceiling */
        s_h_ground = s_v.h;
        s_to_target = s_h_ground + alt;
        {
            const float ct = ceiling_h() - 0.2f;
            if (s_to_target > ct) s_to_target = ct;
        }
        s_h_sp = s_h_ground;
        s_to_t0 = now; s_to_lifted = 0u;
        alt_ctrl_reset(&s_ac, 0.0f);
        lift_off();                  /* motors spool up: not landed any more (as the app simulator) */
        s_auto = PV2_AUTO_TAKEOFF;
        OK();
        event(PV2_EV_INFO, PV2_E_TAKEOFF_STARTED, (int32_t)(alt * 100.0f + 0.5f));
        return;
    }
    case PV2_OP_LAND:
        if (!s_armed || !s_in_air) { REJ(PV2_R_NOT_ARMED); return; }
        s_auto = PV2_AUTO_LAND;
        s_touch_ms = 0u;
        OK();
        return;
    case PV2_OP_RTH:
        if (!gnss())                { REJ(PV2_R_NO_GPS); return; }
        if (!home_set())            { REJ(PV2_R_NO_HOME); return; }
        if (!s_armed || !s_in_air)  { REJ(PV2_R_NOT_ARMED); return; }
        s_auto = PV2_AUTO_RTH;
        OK();
        event(PV2_EV_INFO, PV2_E_RTH_STARTED, 0);
        return;
    case PV2_OP_HOLD:
        if (s_in_air) { s_auto = PV2_AUTO_HOLD; s_h_sp = s_v.h; }
        OK();
        return;
    case PV2_OP_KILL: {
        /* always executed, any state, in flight too */
        if (c->len < 4u || p[0] != 0x4Bu || p[1] != 0x49u || p[2] != 0x4Cu || p[3] != 0x4Cu) {
            INVALID(PV2_R_NONE); return;
        }
        if (s_cfg.motors_arm) s_cfg.motors_arm(0u);     /* motors off before anything else */
        test_stop();
        OK();
        disarm();
        event(PV2_EV_CRITICAL, PV2_E_MOTOR_KILLED, 0);
        return;
    }
    case PV2_OP_SET_MODE: {
        if (c->len < 1u || p[0] > PV2_MODE_POS_HOLD) { INVALID(PV2_R_NONE); return; }
        const uint8_t m = p[0];
        if (m == PV2_MODE_POS_HOLD && !has(PV2_S_GNSS | PV2_S_FLOW)) { REJ(PV2_R_SENSOR_MISSING); return; }
        if (m == PV2_MODE_ALT_HOLD && !alt_sensor())                { REJ(PV2_R_SENSOR_MISSING); return; }
        if (m == PV2_MODE_ACRO && PARAM(P_BEGINNER) > 0.5f)         { REJ(PV2_R_NONE); return; }
        if (m != s_mode && (m == PV2_MODE_ALT_HOLD || m == PV2_MODE_POS_HOLD)) s_h_sp = s_v.h;
        s_mode = m;
        /* a mode change cancels an automatic action (also a kept failsafe action) */
        if (s_auto != PV2_AUTO_NONE && s_auto != PV2_AUTO_TAKEOFF) { s_auto = PV2_AUTO_NONE; s_fs_long = 0u; }
        OK();
        event(PV2_EV_INFO, PV2_E_MODE_CHANGED, m);
        return;
    }
    case PV2_OP_MOTOR_TEST: {
        if (!s_cfg.motors_built) { result(c, PV2_ACK_UNSUPPORTED, PV2_R_NONE); return; }
        if (c->len < 5u) { INVALID(PV2_R_NONE); return; }
        const uint8_t  m   = p[0], pct = p[1];
        const uint16_t ms  = pv2_rd_u16(&p[2]);
        if (p[4] != PV2_PROPS_OFF_MAGIC) { REJ(PV2_R_PROPS_CONFIRM_MISSING); return; }
        if (s_armed) { REJ(PV2_R_ARMED); return; }
        if (m < 1u || m > s_cfg.n_motors || pct > 20u || ms > 3000u) { INVALID(PV2_R_OUT_OF_RANGE); return; }
        test_stop();
        s_test_motor = m; s_test_pct = pct; s_test_until = now + ms;
        if (s_cfg.motor_test) s_cfg.motor_test(m, (float)pct / 100.0f);
        OK();
        return;
    }
    case FC_OP_MOTOR_STOP:
        return;                                    /* test_stop() done above */
    default:
        result(c, PV2_ACK_UNSUPPORTED, PV2_R_NONE);
        return;
    }
#undef OK
#undef REJ
#undef INVALID
}

/* ================================================================== */
static void update_sticks(uint32_t now)
{
    appl_rc_t rc;
    uint32_t  t;
    if (appl_rc_read(&rc, &t)) { s_rc = rc; s_rc_ms = t; s_have_rc = 1u; }
    s_rc_stale = (uint8_t)(!s_have_rc || (now - s_rc_ms) > FC_RC_STALE_MS);
    if (!s_rc_stale) {
        s_stick[0] = (float)s_rc.thr   / 1000.0f;
        s_stick[1] = (float)s_rc.yaw   / 1000.0f;
        s_stick[2] = (float)s_rc.pitch / 1000.0f;
        s_stick[3] = (float)s_rc.roll  / 1000.0f;
        s_last_thr = s_stick[0];
    } else {
        /* stale RC: the last mode's neutral -- level attitude, no yaw rate;
         * altitude hold modes: centre stick = hold altitude; otherwise keep
         * the last collective (no altitude sensor to do better) */
        s_stick[0] = hold_mode() ? 0.0f : (s_armed ? s_last_thr : -1.0f);
        s_stick[1] = s_stick[2] = s_stick[3] = 0.0f;
    }
}

static int stick_moved(void)
{
    if (s_rc_stale) return 0;
    const int16_t v[4] = { s_rc.thr, s_rc.yaw, s_rc.pitch, s_rc.roll };
    for (int i = 0; i < 4; ++i) if (v[i] > FC_STICK_CANCEL || v[i] < -FC_STICK_CANCEL) return 1;
    return 0;
}

static void app_link_failsafe(uint32_t now)
{
    /* a GAP disconnection counts as at least the short silence: earlier, never later */
    const uint32_t sil = appl_silence_ms(now, FC_FS_SHORT_MS + 1u);
    s_silence = sil;
    if (!s_armed) return;

    if (!s_in_air) {
        /* armed on the ground: disarm after 3 s of silence */
        if (sil > FC_GROUND_DISARM_MS) disarm();
        return;
    }
    /* ELRS in control: the app link does not matter (no receiver on this hardware) */
    if (sil > FC_FS_LONG_MS && !s_fs_long) {
        s_auto = fs_action(P_FS_APP_LONG);
        s_failsafe |= PV2_FS_APP_LINK;
        s_fs_long = 1u;
        s_fs_short_hold = 0u;
        event(PV2_EV_ERROR, PV2_E_FAILSAFE_APP_LINK, s_auto);
    } else if (sil > FC_FS_SHORT_MS && !(s_failsafe & PV2_FS_APP_LINK)) {
        s_auto = (PARAM(P_FS_APP_SHORT) > 0.5f) ? PV2_AUTO_LAND : PV2_AUTO_HOLD;
        s_failsafe |= PV2_FS_APP_LINK;
        s_fs_short_hold = (uint8_t)(s_auto == PV2_AUTO_HOLD);
    } else if (sil < FC_LINK_BACK_MS && (s_failsafe & PV2_FS_APP_LINK)) {
        s_failsafe &= (uint8_t)~PV2_FS_APP_LINK;
        /* short HOLD only: control back to the pilot. A long action is kept
         * until the pilot changes mode or cancels it. */
        if (!s_fs_long && s_fs_short_hold && s_auto == PV2_AUTO_HOLD) s_auto = PV2_AUTO_NONE;
        s_fs_short_hold = 0u;
    }
}

/* ---- in air / on ground ---- */
static void air_ground(uint32_t now, uint32_t dt)
{
    /* bench override (no altitude sensor, or forced): debugger flag */
    if (s_armed && g_fc_dbg_in_air && !s_in_air) {
        lift_off(); s_dbg_forced = 1u;
        if (s_auto == PV2_AUTO_TAKEOFF) s_auto = PV2_AUTO_NONE;
        return;
    }
    if (s_dbg_forced) {
        if (!g_fc_dbg_in_air && s_in_air) { s_dbg_forced = 0u; touch_down(now); }
        return;
    }
    if (!s_armed || !alt_ok()) return;

    const float dh = s_v.h - s_h_ground;
    if (!s_in_air) {
        /* manual flight (or lifted by hand on the bench) */
        if (dh > FC_LIFTOFF_DH + 0.05f && !s_v.tof_ground) lift_off();
        return;
    }

    if (s_auto == PV2_AUTO_TAKEOFF) {
        if (!s_to_lifted) {
            if (dh > FC_LIFTOFF_DH) s_to_lifted = 1u;
            else if ((now - s_to_t0) > FC_LIFTOFF_TIMEOUT_MS) {
                /* no lift-off: stop climbing, back on the ground, motors at idle */
                s_auto = PV2_AUTO_NONE;
                s_in_air = 0u;
                s_took_off = 0u;
                s_arm_ms = now;               /* the no-take-off auto-disarm counts again */
                event(PV2_EV_WARN, PV2_E_TAKEOFF_ABORTED, 0);
                return;
            }
        }
        if (s_v.h >= s_to_target - 0.05f) {
            /* target reached: hold it in the mode the sensors allow (§7.2) */
            s_auto = PV2_AUTO_NONE;
            s_mode = has(PV2_S_GNSS | PV2_S_FLOW) ? PV2_MODE_POS_HOLD : PV2_MODE_ALT_HOLD;
            s_h_sp = s_to_target;
            event(PV2_EV_INFO, PV2_E_MODE_CHANGED, s_mode);
        }
        return;                              /* no touchdown while taking off */
    }

    /* touchdown: descent commanded, no vertical motion, close to the floor */
    uint8_t descending;
    if (s_auto == PV2_AUTO_LAND || s_auto == PV2_AUTO_RTH) descending = 1u;
    else if (s_auto != PV2_AUTO_NONE) descending = 0u;
    else if (hold_mode()) descending = (uint8_t)(s_stick[0] < -0.3f);
    else descending = (uint8_t)(s_stick[0] < -0.9f);
    const uint8_t tof = (uint8_t)(has(PV2_S_TOF) && (s_v.tof_valid || s_v.tof_ground));
    const uint8_t near = tof ? (uint8_t)(s_v.tof_ground || s_v.tof_h < 0.08f) : 1u;
    if (descending && near && fabsf(s_v.vz) < FC_TOUCH_VZ) s_touch_ms += dt;
    else s_touch_ms = 0u;
    if (s_touch_ms > (tof ? FC_TOUCH_MS : FC_TOUCH_MS_NO_TOF)) touch_down(now);
}

/* ---- flow lost in POS_HOLD without GNSS (§6, fsFlowLost) ---- */
static void flow_failsafe(uint32_t dt)
{
    if (!has(PV2_S_FLOW) || gnss()) { s_failsafe &= (uint8_t)~PV2_FS_FLOW; return; }
    const uint8_t q = s_have_v ? s_v.flow_q : 0xFFu;
    const uint8_t bad = (uint8_t)(q == 0xFFu || q < FC_FLOW_LOST_Q);
    if (s_armed && s_in_air && s_mode == PV2_MODE_POS_HOLD && bad) {
        s_flow_bad_ms += dt;
        if (s_flow_bad_ms > FC_FLOW_LOST_MS && !(s_failsafe & PV2_FS_FLOW)) {
            s_failsafe |= PV2_FS_FLOW;
            event(PV2_EV_WARN, PV2_E_FLOW_LOST, 0);
            if (PARAM(P_FS_FLOW_LOST) > 0.5f) s_auto = PV2_AUTO_LAND;
            else { s_mode = PV2_MODE_ALT_HOLD; s_h_sp = s_v.h; }
        }
    } else {
        s_flow_bad_ms = 0u;
        if ((s_failsafe & PV2_FS_FLOW) && q != 0xFFu && q >= FC_FLOW_BACK_Q) s_failsafe &= (uint8_t)~PV2_FS_FLOW;
    }
}

/* ---- setpoints for the control loops ---- */
static void setpoints(uint32_t now, uint32_t dt_ms)
{
    (void)now;
    static uint8_t prev_auto;
    const float dt = (float)dt_ms * 1e-3f;
    const float beg = (PARAM(P_BEGINNER) > 0.5f) ? 0.5f : 1.0f;
    fc_setpoint_t sp;
    memset(&sp, 0, sizeof sp);
    sp.armed = s_armed; sp.mode = s_mode; sp.autom = s_auto;

    /* entering HOLD / failsafe HOLD: hold the current height */
    if (s_auto != prev_auto && (s_auto == PV2_AUTO_HOLD || s_auto == PV2_AUTO_FAILSAFE)) s_h_sp = s_v.h;
    prev_auto = s_auto;

    /* attitude from the sticks (neutral when the RC is stale) */
    const float tilt = PARAM(P_MAX_TILT) * beg * DEG2RAD;
    const float yawr = PARAM(P_MAX_YAW_RATE) * beg * DEG2RAD;
    sp.roll  = s_stick[3] * tilt;
    sp.pitch = s_stick[2] * tilt;
    sp.roll_rate  = s_stick[3] * 4.0f * yawr;           /* ACRO: 4x the yaw rate scale */
    sp.pitch_rate = s_stick[2] * 4.0f * yawr;
    sp.yaw_rate   = s_stick[1] * yawr;

    if (!s_armed) { s_sp = sp; return; }

    const uint8_t vert = (uint8_t)(s_in_air && alt_ok() &&
                                   (s_auto != PV2_AUTO_NONE || hold_mode()));
    if (!vert) {
        /* manual collective (ACRO / ANGLE, or no altitude estimate) */
        sp.thrust = clampf(0.5f * (s_stick[0] + 1.0f), 0.0f, 1.0f);
        if (hold_mode() && !s_in_air) sp.thrust = 0.0f;      /* hold modes: take off with TAKEOFF */
        s_sp = sp;
        return;
    }

    const float maxc = PARAM(P_MAX_CLIMB) * beg;
    const float land = PARAM(P_LAND_SPEED);
    float vz;
    switch (s_auto) {
    case PV2_AUTO_TAKEOFF:
        vz = alt_ctrl_vz_sp(&s_ac, s_to_target, s_v.h, FC_TAKEOFF_VZ, land);
        s_h_sp = s_v.h;
        break;
    case PV2_AUTO_LAND:
    case PV2_AUTO_RTH:                       /* no GNSS here: RTH lands in place */
        vz = -((has(PV2_S_TOF) && s_v.tof_valid && s_v.tof_h < 1.0f) ? FC_LAND_SLOW_VZ : land);
        if (s_v.tof_ground) vz = -FC_LAND_SLOW_VZ;
        s_h_sp = s_v.h;
        break;
    case PV2_AUTO_HOLD:
    case PV2_AUTO_FAILSAFE:
        vz = alt_ctrl_vz_sp(&s_ac, s_h_sp, s_v.h, maxc, land);
        break;
    default:                                 /* ALT_HOLD / POS_HOLD, pilot: climb rate */
        if (fabsf(s_stick[0]) < 0.05f) vz = alt_ctrl_vz_sp(&s_ac, s_h_sp, s_v.h, maxc, land);
        else { vz = s_stick[0] * maxc; s_h_sp = s_v.h; }
        break;
    }

    /* geofence: no climb above the ceiling, come back below it (§6, gfMaxAlt / gfCeiling) */
    int32_t gf_kind;
    const float ceil = ceiling_h_kind(&gf_kind);
    if (s_v.h >= ceil && vz > 0.0f) {
        vz = (s_v.h > ceil + 0.3f) ? -0.3f : 0.0f;
        if (s_h_sp > ceil) s_h_sp = ceil;
        if (s_auto == PV2_AUTO_TAKEOFF) s_to_target = s_v.h;
        if (!s_geofence) {
            s_geofence = 1u;
            s_failsafe |= PV2_FS_GEOFENCE;
            if (!s_geofence_ev) { event(PV2_EV_WARN, PV2_E_GEOFENCE_BREACH, gf_kind); s_geofence_ev = 1u; }
        }
    } else if (s_v.h < ceil - 0.2f && s_geofence) {
        s_geofence = 0u;
        s_failsafe &= (uint8_t)~PV2_FS_GEOFENCE;
        s_geofence_ev = 0u;
    }

    /* take-off spool-up: thrust ramps from idle until the drone leaves the ground;
     * the ramp is the controller's upper limit, so the integrator does not wind up */
    const float thr_max = s_ac.thr_max;
    if (s_auto == PV2_AUTO_TAKEOFF && !s_to_lifted) {
        const float ramp = 0.10f + 0.40f * (float)(now - s_to_t0) * 1e-3f;
        if (ramp < s_ac.thr_max) s_ac.thr_max = ramp;
    }
    const float thr = alt_ctrl_step(&s_ac, vz, s_v.vz, s_v.cos_tilt, dt);
    s_ac.thr_max = thr_max;
    sp.alt_ctrl = 1u;
    sp.thrust = thr;
    sp.vz_sp = vz;
    sp.h_sp = s_h_sp;
    s_sp = sp;
}

static void publish(void)
{
    s_snap_seq++;
    SPSC_BARRIER();
    s_snap.armed = s_armed; s_snap.mode = s_mode; s_snap.autom = s_auto;
    s_snap.failsafe = s_failsafe; s_snap.in_air = s_in_air;
    s_snap.input = (s_silence < FC_FS_SHORT_MS) ? PV2_INPUT_APP : PV2_INPUT_NONE;
    s_snap.rc_stale = s_rc_stale;
    s_snap.fs_long = s_fs_long;
    s_snap.flight_time_s = (uint16_t)((s_flight_ms / 1000u > 0xFFFFu) ? 0xFFFFu : s_flight_ms / 1000u);
    memset(s_snap.motor_out_pct, 0, sizeof s_snap.motor_out_pct);
    if (s_test_motor) s_snap.motor_out_pct[s_test_motor - 1u] = s_test_pct;
    s_snap.test_motor = s_test_motor;
    s_snap.landed = s_landed;
    s_snap.geofence = s_geofence;
    if (s_have_v && s_v.valid) {
        const float a = (s_v.h - (s_armed || s_took_off ? s_h_ref : s_v.h)) * 100.0f;
        s_snap.alt_rel_cm = (int32_t)(a + (a >= 0.0f ? 0.5f : -0.5f));
        const float w = clampf(s_v.vz * 100.0f, -32767.0f, 32767.0f);
        s_snap.vz_cms = (int16_t)w;
    } else {
        s_snap.alt_rel_cm = 0; s_snap.vz_cms = 0;
    }
    s_snap.silence_ms = s_silence;
    memcpy(s_snap.stick, s_stick, sizeof s_stick);
    SPSC_BARRIER();
    s_snap_seq++;
}

void fc_service(uint32_t now, uint8_t est_running, const fc_vert_t *v)
{
    const uint32_t dt = s_have_last ? (now - s_last_ms) : 0u;
    s_last_ms = now; s_have_last = 1u;
    if (v) { s_v = *v; s_have_v = 1u; } else { s_have_v = 0u; s_v.valid = 0u; s_v.flow_q = 0xFFu; }

    update_sticks(now);

    fc_cmd_t c;
    while (spsc_pop(&s_cmdq, &c)) exec(&c, now, est_running);

    air_ground(now, dt);

    app_link_failsafe(now);
    flow_failsafe(dt);

    /* ---- pilot cancels HOLD (and RTH, if allowed) with the sticks ---- */
    if (s_armed && s_in_air && s_silence < FC_LINK_BACK_MS && !(s_failsafe & PV2_FS_APP_LINK)) {
        if (s_auto == PV2_AUTO_HOLD && stick_moved()) s_auto = PV2_AUTO_NONE;
        if (s_auto == PV2_AUTO_RTH && PARAM(P_RTH_STICK_CANCEL) > 0.5f &&
            !s_rc_stale && (s_rc.pitch > FC_STICK_CANCEL || s_rc.pitch < -FC_STICK_CANCEL ||
                            s_rc.roll > FC_STICK_CANCEL || s_rc.roll < -FC_STICK_CANCEL))
            s_auto = PV2_AUTO_NONE;
    }

    /* ---- automatic disarm ---- */
    if (s_armed && !s_in_air) {
        if (!s_took_off && (now - s_arm_ms) > FC_NO_TAKEOFF_MS) disarm();
        else if (s_took_off && (now - s_landed_ms) > (uint32_t)(PARAM(P_AUTO_DISARM) * 1000.0f)) disarm();
    }
    if (s_armed && s_in_air) s_flight_ms += dt;

    /* ---- motor test: stops after ms, on link loss ---- */
    if (s_test_motor && ((int32_t)(now - s_test_until) >= 0 || s_silence > FC_FS_SHORT_MS)) test_stop();

    setpoints(now, dt);
    publish();
}
