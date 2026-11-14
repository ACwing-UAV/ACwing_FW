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
#include "ctl_io.h"
#include "hover_track.h"
#include "input_arb.h"
#include "radio_fs.h"
#include "rc_switches.h"
#include "launch.h"
#include "flip.h"
#include <string.h>
#include <math.h>

#define DEG2RAD 0.017453292519943295f

volatile uint8_t g_fc_dbg_in_air;

SPSC_DEFINE(s_cmdq, fc_cmd_t,   8u);
SPSC_DEFINE(s_resq, fc_res_t,   8u);
SPSC_DEFINE(s_evq,  fc_event_t, 16u);
SPSC_DEFINE(s_batq, fc_batt_sig_t, 8u);

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
static ia_sticks_t s_app;          /* last app RC                                   */
static ia_sticks_t s_rc;           /* sticks of the source in control (input_arb.c) */
static uint8_t  s_radio_ctrl;      /* the RC radio has control                      */
static uint32_t s_radio_sil;       /* since the last valid radio frame              */
static ln_t     s_ln;              /* hand launch (launch.c)                          */
static ln_out_t s_lno;
static fl_t     s_fl;              /* autonomous flip (flip.c)                        */
static fl_out_t s_flo;
static uint8_t  s_fl_mode;         /* mode before the flip, restored after it          */
static float    s_fl_h0, s_fl_hmin;
static rc_state_t s_radio;         /* last receiver state (switches)                 */
static uint8_t  s_radio_took;      /* the radio took control on this step            */
static uint8_t  s_exec_radio;      /* the command being executed comes from the radio switches */
static uint8_t  s_rfs_hold;        /* the radio failsafe chose HOLD                 */
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
static ctl_ref_t s_ref;             /* reference for the control law (ctl_api.h) */
static fc_setpoint_t s_ref_sp;     /* its manager part, before the law            */
static uint8_t  s_ctl_sat;         /* the law's collective at a limit              */
static fc_setpoint_t s_sp;
static uint8_t    s_prev_auto;      /* setpoints(): automatic action of the previous step */
static uint8_t    s_hold_prev;      /* HOLD active at the previous step (stick-cancel reference) */
static float      s_hold_thr;       /* throttle stick when HOLD began                  */

/* manual collective, landing idle, in air without an altitude estimate (§6) */
static float      s_hover = HT_SEED_DEFAULT;  /* hover thrust in use (hover_track.c) */
static uint32_t   s_lift_ms, s_now;
static uint8_t    s_land_idle;      /* touched down: idle until disarm or stick at the bottom */
static uint8_t    s_air_thr;        /* in air detected from the collective                    */
static uint32_t   s_air_thr_ms;     /* collective above the lift-off level for this long      */
static uint32_t   s_land_ol_t0;     /* open-loop LAND started (no altitude estimate)          */
static uint8_t    s_land_ol;
static float      s_land_ol_from;   /* collective when the open-loop LAND started            */

/* battery */
static uint8_t           s_batt_level, s_batt_present, s_batt_lock, s_batt_stale, s_batt_have_hb;
static volatile uint32_t s_batt_hb_ms;
static volatile uint8_t  s_batt_hb_set;

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
    s_flight_ms = 0u; s_have_last = 0u;
    memset(&s_app, 0, sizeof s_app); memset(&s_rc, 0, sizeof s_rc);
    s_radio_ctrl = 0u; s_radio_sil = UINT32_MAX; s_rfs_hold = 0u;
    ia_init(); rfs_init(); rsw_init();
    ln_cancel(&s_ln); memset(&s_lno, 0, sizeof s_lno);
    memset(&s_radio, 0, sizeof s_radio); s_radio_took = 0u; s_exec_radio = 0u;
    s_test_motor = 0u;
    s_checks_ready = s_batt_fail = s_calibrating = 0u;
    g_fc_dbg_in_air = 0u;
    s_have_v = 0u; s_dbg_forced = 0u; s_landed = 0u; s_geofence = s_geofence_ev = 0u;
    s_touch_ms = s_flow_bad_ms = 0u;
    s_lift_ms = s_now = 0u;
    hover_track_init();
    s_hover = hover_track_value();
    ctl_io_init();                      /* the control law (ADR 0013): default cascade, or the one of ctlLaw */
    ctl_io_feed_hover(s_hover);
    memset(&s_sp, 0, sizeof s_sp);
    memset(&s_ref, 0, sizeof s_ref); memset(&s_ref_sp, 0, sizeof s_ref_sp); s_ctl_sat = 0u;
    s_prev_auto = PV2_AUTO_NONE; s_hold_prev = 0u; s_hold_thr = 0.0f;
    s_land_idle = 0u; s_air_thr = 0u; s_air_thr_ms = 0u;
    s_land_ol = 0u; s_land_ol_t0 = 0u; s_land_ol_from = 0.0f;
    s_rc_stale = 1u; s_last_thr = -1.0f;
    memset(s_stick, 0, sizeof s_stick);
    spsc_reset(&s_batq);
    s_batt_level = PV2_BATT_OK; s_batt_present = 0u; s_batt_lock = 0u;
    s_batt_stale = cfg->batt_monitor;      /* unknown until the first heartbeat */
    s_batt_have_hb = 0u; s_batt_hb_set = 0u;
}

int  fc_batt_signal(const fc_batt_sig_t *b) { return spsc_push(&s_batq, b); }
void fc_batt_heartbeat(uint32_t now)       { s_batt_hb_ms = now; SPSC_BARRIER(); s_batt_hb_set = 1u; }

void fc_set_sensors(uint16_t sensors) { s_sensors = sensors; }
void fc_setpoint(fc_setpoint_t *out)  { *out = s_sp; }
uint8_t fc_vector_hold(void)          { return (uint8_t)(s_auto == PV2_AUTO_FLIP && s_flo.vector_hold); }

uint8_t fc_is_core_op(uint8_t op)
{
    switch (op) {
    case PV2_OP_ARM: case PV2_OP_DISARM: case PV2_OP_TAKEOFF: case PV2_OP_LAND:
    case PV2_OP_RTH: case PV2_OP_HOLD: case PV2_OP_KILL: case PV2_OP_SET_MODE:
    case PV2_OP_MOTOR_TEST: case FC_OP_MOTOR_STOP: case PV2_OP_LAUNCH: case PV2_OP_FLIP:
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
    if (s_cfg.on_event) s_cfg.on_event(&e);
}

static uint8_t has(uint16_t bits) { return (uint8_t)((s_sensors & bits) != 0u); }
/* GNSS usable for navigation. The GPS board found (PV2_S_GNSS, CAPS) is not enough: without a position estimate
 * that fuses it, POS_HOLD would fly as ALT_HOLD and RTH would have no position. 0 until the horizontal estimator
 * exists (GNSS step 2); then: board found, 3D fix, accuracy within the limits. */
static uint8_t gnss(void)         { return 0u; }
/* a horizontal position source for POS_HOLD: the GNSS solution outdoors, the optical flow indoors */
static uint8_t pos_source(void)   { return (uint8_t)(gnss() || has(PV2_S_FLOW)); }
static uint8_t home_set(void)     { return 0u; }   /* no GNSS on this hardware yet */
static uint8_t alt_sensor(void)   { return has(PV2_S_BARO | PV2_S_TOF); }
static uint8_t alt_ok(void)       { return (uint8_t)(alt_sensor() && s_have_v && s_v.valid); }
static uint8_t indoor(void)       { return (uint8_t)(has(PV2_S_TOF) && !gnss()); }
static float   clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
/* age of a timestamp from another task: it may be a few ms newer than `now` */
static uint32_t age_ms(uint32_t now, uint32_t t) { const int32_t d = (int32_t)(now - t); return d > 0 ? (uint32_t)d : 0u; }

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
    s_lift_ms = s_now;
    s_in_air = 1u; s_took_off = 1u; s_landed = 0u; s_touch_ms = 0u; s_land_idle = 0u;
}

static void touch_down(uint32_t now)
{
    s_in_air = 0u;
    s_landed = 1u;
    s_air_thr = 0u; s_air_thr_ms = 0u; s_land_ol = 0u;
    /* motors at idle until disarm, or until the pilot puts the stick at the bottom
     * (then a manual take-off is possible again): after an automatic landing the
     * stick may stand at the centre, which would be about hover thrust (§7.3) */
    s_land_idle = 1u;
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
    s_batt_lock = 0u;
    s_land_idle = 0u; s_air_thr = 0u; s_air_thr_ms = 0u; s_land_ol = 0u;
    ln_cancel(&s_ln); memset(&s_lno, 0, sizeof s_lno);
    fl_cancel(&s_fl); memset(&s_flo, 0, sizeof s_flo);
    if (was) hover_track_disarm(s_flight_ms);
    if (was) event(PV2_EV_INFO, PV2_E_DISARMED, 0);
}

/* the battery monitor says no flight: level CRITICAL or worse, or it stopped */
static uint8_t batt_unusable(void)
{
    return (uint8_t)(s_cfg.batt_monitor && (s_batt_stale || s_batt_level >= PV2_BATT_CRITICAL));
}
/* a battery failsafe is flying the drone down: other failsafes do not take over */
static uint8_t batt_owns_auto(void)
{
    return (uint8_t)((s_failsafe & PV2_FS_BATTERY) && (s_auto == PV2_AUTO_LAND || s_auto == PV2_AUTO_RTH));
}

/* failsafe action value -> automatic action; RTH without GNSS + home -> land */
static uint8_t fs_action(int p_idx)
{
    const int a = (int)(PARAM(p_idx) + 0.5f);
    if (a == 2) return (gnss() && home_set()) ? PV2_AUTO_RTH : PV2_AUTO_LAND;
    if (a == 1) return PV2_AUTO_LAND;
    return PV2_AUTO_HOLD;
}

/* ---- arming: the checks and the state, shared by ARM and LAUNCH ---- */
static uint8_t arm_refusal(uint32_t now, uint8_t est_running)
{
    if (s_armed)       return PV2_R_ARMED;
    if (s_calibrating) return PV2_R_CALIBRATING;
    if (s_in_air)      return PV2_R_NOT_LANDED;
    if (s_radio_ctrl && !s_exec_radio) return PV2_R_RADIO_OVERRIDE;     /* the RC radio flies (input_arb.c) */
    if (s_rc.have && age_ms(now, s_rc.t_ms) < FC_ARM_RC_FRESH_MS &&
        s_rc.thr > (hold_mode() ? 100 : -900)) return PV2_R_THROTTLE_NOT_LOW;
    if (s_batt_fail || batt_unusable()) return PV2_R_BATTERY_LOW;
    /* never arm without a running attitude estimate, whatever armRequireChecks says */
    if (!est_running || (PARAM(P_ARM_REQUIRE_CHECKS) > 0.5f && !s_checks_ready)) return PV2_R_CHECKS_FAILING;
    return PV2_R_NONE;
}

static void arm_now(uint32_t now)
{
    s_armed = 1u;
    s_auto = PV2_AUTO_NONE; s_failsafe = 0u; s_fs_long = 0u;
    s_took_off = 0u; s_arm_ms = now; s_flight_ms = 0u;
    s_landed = 0u; s_geofence_ev = 0u;
    s_land_idle = 0u; s_air_thr = 0u; s_air_thr_ms = 0u; s_land_ol = 0u;
    if (s_have_v && s_v.valid) { s_h_ref = s_h_ground = s_h_sp = s_v.h; }
    hover_track_arm(PARAM(P_HOVER_THR));
    s_hover = hover_track_value();               /* the law resets its loops when it sees the drone armed */
    if (s_cfg.motors_arm) s_cfg.motors_arm(1u);
    event(PV2_EV_INFO, PV2_E_ARMED, 0);
}

/* LAUNCH: the altitude source it needs besides arming (ADR 0011 §1); PV2_R_NONE = none missing */
static uint8_t launch_refusal(void)
{
    return (uint8_t)((!alt_sensor() || !alt_ok()) ? PV2_R_SENSOR_MISSING : PV2_R_NONE);
}

static uint8_t pilot_link_ok(void);

/* FLIP: the entry gate of ADR 0012 §2 (flip.c), from the flight core's state; PV2_R_NONE = may start */
static uint8_t flip_refusal(void)
{
    fl_gate_in_t g;
    memset(&g, 0, sizeof g);
    g.armed = s_armed; g.in_air = s_in_air;
    g.auto_none = (uint8_t)(s_auto == PV2_AUTO_NONE);
    g.failsafe = s_failsafe;
    g.beginner = (uint8_t)(PARAM(P_BEGINNER) > 0.5f);
    g.acro = (uint8_t)(s_mode == PV2_MODE_ACRO);
    g.alt_valid = alt_ok();
    g.height = (s_have_v && s_v.tof_valid) ? s_v.tof_h : s_v.h - s_h_ground;   /* above the floor, else the take-off */
    g.ceiling_room = ceiling_h() - s_v.h;
    g.tw = ln_tw_available(s_hover, s_v.v_cell);
    g.cos_tilt = s_v.cos_tilt; g.w = s_v.w; g.vz = s_v.vz;
    g.v_h = s_v.flow_usable ? sqrtf(s_v.vel_b[0] * s_v.vel_b[0] + s_v.vel_b[1] * s_v.vel_b[1]) : 0.0f;
    g.batt_ok = (uint8_t)(s_batt_level == PV2_BATT_OK && !s_batt_stale);
    g.link_ok = pilot_link_ok();
    g.gyro_sat = s_v.gyro_sat;
    g.min_h = PARAM(P_FLIP_MIN_H); g.min_tw = PARAM(P_FLIP_MIN_TW);
    return fl_gate(&g);
}

/* ================================================================== */
static void result(const fc_cmd_t *c, uint8_t res, uint8_t why)
{
    if (c->op == FC_OP_MOTOR_STOP) return;   /* internal: no ACK */
    if (s_exec_radio) {                      /* radio switch: no ACK, a refusal is an EVENT (the app shows why) */
        if (res != PV2_ACK_OK) event(PV2_EV_WARN, PV2_E_RADIO_REFUSED, (int32_t)(((uint32_t)c->op << 8) | why));
        return;
    }
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

    /* battery emergency: landing, no override except KILL (§6) */
    if (s_batt_lock && (c->op == PV2_OP_SET_MODE || c->op == PV2_OP_HOLD || c->op == PV2_OP_RTH ||
                        c->op == PV2_OP_TAKEOFF)) { REJ(PV2_R_BATTERY_LOW); return; }

    /* a cable to a PC: no arming, no take-off, no spinning motors (ADR 0009) */
    if ((c->op == PV2_OP_ARM || c->op == PV2_OP_TAKEOFF || c->op == PV2_OP_MOTOR_TEST) &&
        s_cfg.usb_host && s_cfg.usb_host()) {
        if (c->op == PV2_OP_ARM && s_armed) { REJ(PV2_R_ARMED); return; }
        REJ(PV2_R_USB_CONNECTED); return;
    }

    switch (c->op) {
    case PV2_OP_ARM: {
        if (c->len < 2u || p[0] != PV2_ARM_MAGIC0 || p[1] != PV2_ARM_MAGIC1) { INVALID(PV2_R_NONE); return; }
        const uint8_t why = arm_refusal(now, est_running);
        if (why) { REJ(why); return; }
        OK();
        arm_now(now);
        return;
    }
    case PV2_OP_FLIP: {
        if (c->len < 1u || p[0] > FL_PITCH_BACK) { INVALID(PV2_R_NONE); return; }
        const uint8_t why = flip_refusal();
        if (why) { REJ(why == FL_REFUSED ? PV2_R_NONE : why); return; }
        OK();
        s_fl_mode = s_mode; s_fl_h0 = s_fl_hmin = s_v.h;
        fl_start(&s_fl, p[0], PARAM(P_FLIP_RATE) * DEG2RAD);
        memset(&s_flo, 0, sizeof s_flo);
        s_auto = PV2_AUTO_FLIP;
        return;
    }
    case PV2_OP_LAUNCH: {
        if (c->len < 2u || p[0] != PV2_ARM_MAGIC0 || p[1] != PV2_ARM_MAGIC1) { INVALID(PV2_R_NONE); return; }
        if (s_exec_radio) { REJ(PV2_R_NONE); return; }           /* never from the arm switch (ADR 0011 §3) */
        if (PARAM(P_BEGINNER) > 0.5f) { REJ(PV2_R_NONE); return; }   /* as ACRO */
        uint8_t why = arm_refusal(now, est_running);
        if (!why) why = launch_refusal();
        if (why) { REJ(why); return; }
        const uint8_t drop = (uint8_t)(PARAM(P_LAUNCH_DROP) > 0.5f);
        if (ln_tw_available(hover_track_value(), s_v.v_cell) < (drop ? LN_TW_MIN_DROP : LN_TW_MIN)) {
            REJ(PV2_R_THRUST_LOW); return;
        }
        OK();
        arm_now(now);
        ln_start(&s_ln, drop);
        s_auto = PV2_AUTO_LAUNCH;
        event(PV2_EV_INFO, PV2_E_LAUNCH_READY, drop);
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
        if (batt_unusable()) { REJ(PV2_R_BATTERY_LOW); return; }
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
        lift_off();                  /* the law enters its vertical loop from the hover thrust */                  /* motors spool up: not landed any more (as the app simulator) */
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
        if (m == PV2_MODE_POS_HOLD && !pos_source()) { REJ(PV2_R_SENSOR_MISSING); return; }
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
/* the app RC, as the input arbitration wants it */
static void read_app_rc(void)
{
    appl_rc_t rc;
    uint32_t  t;
    if (!appl_rc_read(&rc, &t)) return;
    s_app.have = 1u; s_app.thr = rc.thr; s_app.yaw = rc.yaw; s_app.pitch = rc.pitch; s_app.roll = rc.roll; s_app.t_ms = t;
}

/* who flies (input_arb.c) and with which sticks */
static void arbitrate(uint32_t now)
{
    rc_state_t radio;
    memset(&radio, 0, sizeof radio);
    const uint8_t have_radio = rc_input_read(&radio);
    ia_out_t o;
    ia_step(now, s_armed, &s_app, &radio, &o);
    if (o.took_control) event(PV2_EV_WARN, PV2_E_RADIO_OVERRIDE, 0);
    s_radio_ctrl = o.radio;
    s_radio = radio;
    s_radio_took = o.took_control;
    s_rc = o.sticks;
    s_radio_sil = have_radio ? age_ms(now, radio.t_ms) : UINT32_MAX;
}

/* the radio switches (rc_switches.c) as flight-core commands, executed as the app ones */
static void radio_switches(uint32_t now, uint8_t est_running)
{
    const rsw_in_t in = { s_radio.valid, s_radio_ctrl, s_radio_took, s_radio.aux_us[0], s_radio.aux_us[1],
                          s_armed, s_in_air, (uint8_t)(PARAM(P_RC_ARM_SW_KILL) > 0.5f),
                          (uint8_t)(PARAM(P_RC_MODE_MAP) + 0.5f) };
    rsw_out_t o;
    rsw_step(&in, &o);
    fc_cmd_t c;
    memset(&c, 0, sizeof c);
    s_exec_radio = 1u;
    switch (o.action) {
    case RSW_ARM:    c.op = PV2_OP_ARM;  c.len = 2u; c.p[0] = PV2_ARM_MAGIC0; c.p[1] = PV2_ARM_MAGIC1; exec(&c, now, est_running); break;
    case RSW_DISARM: c.op = PV2_OP_DISARM; exec(&c, now, est_running); break;
    case RSW_KILL:   c.op = PV2_OP_KILL; c.len = 4u; memcpy(c.p, "KILL", 4); exec(&c, now, est_running); break;
    default: break;
    }
    if (o.mode >= 0) {
        memset(&c, 0, sizeof c);
        c.op = PV2_OP_SET_MODE; c.len = 1u; c.p[0] = (uint8_t)o.mode;
        if (o.mode != (int8_t)s_mode) exec(&c, now, est_running);
    }
    s_exec_radio = 0u;
}

static void update_sticks(uint32_t now)
{
    read_app_rc();
    arbitrate(now);
    s_rc_stale = (uint8_t)(!s_rc.have || age_ms(now, s_rc.t_ms) > FC_RC_STALE_MS);
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
        /* stale RC: on the ground the collective goes to idle, so that a stick left
         * up (cockpit closed, then slide-to-arm) never lifts the drone off (§6) */
        s_stick[0] = hold_mode() ? 0.0f : ((s_armed && s_in_air) ? s_last_thr : -1.0f);
        s_stick[1] = s_stick[2] = s_stick[3] = 0.0f;
    }
}

/* HOLD is cancelled by a stick moved by more than 30 % (§7.5). Yaw, pitch and
 * roll spring to the centre; the throttle counts as a change from where it was
 * when HOLD began: in ACRO / ANGLE it is the collective and does not stand at
 * the centre, so an absolute test would cancel HOLD at once. */
static int stick_moved(void)
{
    if (s_rc_stale) return 0;
    const int16_t v[3] = { s_rc.yaw, s_rc.pitch, s_rc.roll };
    for (int i = 0; i < 3; ++i) if (v[i] > FC_STICK_CANCEL || v[i] < -FC_STICK_CANCEL) return 1;
    return fabsf((float)s_rc.thr / 1000.0f - s_hold_thr) > (float)FC_STICK_CANCEL / 1000.0f;
}

static void app_link_failsafe(uint32_t now)
{
    /* a GAP disconnection counts as at least the short silence: earlier, never later */
    const uint32_t sil = appl_silence_ms(now, FC_FS_SHORT_MS + 1u);
    s_silence = sil;
    if (!s_armed) return;
    if (s_radio_ctrl) { s_failsafe &= (uint8_t)~PV2_FS_APP_LINK; return; }   /* the radio flies: the app link does not matter */

    if (!s_in_air) {
        /* armed on the ground: disarm after 3 s of silence */
        if (sil > FC_GROUND_DISARM_MS) disarm();
        return;
    }
    /* ELRS in control: the app link does not matter (no receiver on this hardware) */
    if (batt_owns_auto()) {
        /* landing for the battery: the link loss is only flagged */
        if (sil > FC_FS_SHORT_MS) s_failsafe |= PV2_FS_APP_LINK;
        else if (sil < FC_LINK_BACK_MS) s_failsafe &= (uint8_t)~PV2_FS_APP_LINK;
        return;
    }
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

/* ---- radio failsafe (§6, fsRadio): radio_fs.c decides, this applies ---- */
static void radio_failsafe(void)
{
    switch (rfs_step(s_radio_ctrl, s_radio_sil, s_armed, s_in_air)) {
    case RFS_TRIGGER:
        s_failsafe |= PV2_FS_RADIO;
        if (!batt_owns_auto()) s_auto = fs_action(P_FS_RADIO);    /* RTH without GNSS + home -> land */
        s_rfs_hold = (uint8_t)(s_auto == PV2_AUTO_HOLD);
        event(PV2_EV_ERROR, PV2_E_FAILSAFE_RADIO, s_auto);
        break;
    case RFS_BACK:
        s_failsafe &= (uint8_t)~PV2_FS_RADIO;
        if (s_rfs_hold && s_auto == PV2_AUTO_HOLD) s_auto = PV2_AUTO_NONE;   /* hover only: control back */
        s_rfs_hold = 0u;
        break;
    case RFS_DISARM:
        disarm();
        break;
    default:
        if (!rfs_active()) s_failsafe &= (uint8_t)~PV2_FS_RADIO;
        break;
    }
}

/* the pilot in control is connected (stick-cancel of HOLD / RTH) */
static uint8_t pilot_link_ok(void)
{
    if (s_radio_ctrl) return (uint8_t)(!rfs_active() && !s_rc_stale);
    return (uint8_t)(s_silence < FC_LINK_BACK_MS && !(s_failsafe & PV2_FS_APP_LINK));
}

/* ---- battery (§6): signals from the BattTask, failsafes ---- */
static void battery(uint32_t now)
{
    if (!s_cfg.batt_monitor) return;
    fc_batt_sig_t b;
    while (spsc_pop(&s_batq, &b)) {
        if (b.level > s_batt_level) {
            /* crossing a threshold downwards: one event (§11 codes 30/31/32) */
            static const uint8_t lv[4]    = { 0, PV2_EV_WARN, PV2_EV_ERROR, PV2_EV_CRITICAL };
            static const uint16_t code[4] = { 0, PV2_E_BATTERY_LOW, PV2_E_BATTERY_CRITICAL, PV2_E_BATTERY_EMERGENCY };
            if (b.level <= PV2_BATT_EMERGENCY) event(lv[b.level], code[b.level], b.cell_mv);
        }
        s_batt_level = (b.level <= PV2_BATT_EMERGENCY) ? b.level : PV2_BATT_EMERGENCY;
        s_batt_present = b.present;
    }

    /* the BattTask must be alive: its heartbeat is the only proof */
    uint8_t stale = 1u;
    if (s_batt_hb_set) {
        SPSC_BARRIER();
        stale = (uint8_t)((now - s_batt_hb_ms) > FC_BATT_STALE_MS);
        s_batt_have_hb = 1u;
    }
    if (stale && !s_batt_stale && s_batt_have_hb)
        event(PV2_EV_ERROR, PV2_E_BATTERY_MONITOR_LOST, 0);
    s_batt_stale = stale;

    if (!s_armed || !s_in_air) return;
    if (s_batt_level >= PV2_BATT_CRITICAL && !(s_failsafe & PV2_FS_BATTERY)) {
        /* RTH needs GNSS, home and enough charge to get there: none on this hardware */
        const uint8_t act = (gnss() && home_set()) ? PV2_AUTO_RTH : PV2_AUTO_LAND;
        s_failsafe |= PV2_FS_BATTERY;
        if (s_auto != PV2_AUTO_LAND && s_auto != PV2_AUTO_RTH) { s_auto = act; s_touch_ms = 0u; }
        s_fs_short_hold = 0u;
        event(PV2_EV_ERROR, PV2_E_FAILSAFE_BATTERY, s_auto);
    }
    if (s_batt_level >= PV2_BATT_EMERGENCY && !s_batt_lock) {
        s_batt_lock = 1u;
        s_failsafe |= PV2_FS_BATTERY;
        if (s_auto != PV2_AUTO_LAND) { s_auto = PV2_AUTO_LAND; s_touch_ms = 0u; }
    }
    if (s_batt_lock) s_auto = PV2_AUTO_LAND;          /* nothing but KILL overrides it */
}

/* ---- in air / on ground without an altitude estimate (no baro / ToF, or not valid) ----
 * Lift-off: the collective commanded at the previous step stays above
 * FC_AIR_THR_K x hover for FC_AIR_MS. Touchdown: stick at the bottom for
 * FC_TOUCH_MS_NO_TOF, or the open-loop LAND has run for FC_LAND_OL_MS. Without
 * this the drone flown by hand stays "on the ground": no in-air failsafe, and the
 * 10 s no-take-off auto-disarm would stop the motors in flight. */
static void air_ground_no_alt(uint32_t now, uint32_t dt)
{
    if (s_auto == PV2_AUTO_TAKEOFF) s_auto = s_in_air ? PV2_AUTO_HOLD : PV2_AUTO_NONE;   /* lost the estimate */
    if (!s_in_air) {
        if (!s_land_idle && s_sp.thrust > FC_AIR_THR_K * s_hover) {
            s_air_thr_ms += dt;
            if (s_air_thr_ms >= FC_AIR_MS) { lift_off(); s_air_thr = 1u; }
        } else s_air_thr_ms = 0u;
        return;
    }
    if (s_land_ol && (now - s_land_ol_t0) > FC_LAND_OL_MS) { touch_down(now); return; }
    const uint8_t low = (uint8_t)(!s_rc_stale && s_stick[0] < FC_THR_LOW && s_auto == PV2_AUTO_NONE);
    if (low) s_touch_ms += dt; else s_touch_ms = 0u;
    if (s_touch_ms > FC_TOUCH_MS_NO_TOF) touch_down(now);
}

/* ---- in air / on ground ---- */
static void air_ground(uint32_t now, uint32_t dt)
{
    /* hand launch: held in the hand, lifted, thrown: launch.c decides when the drone flies. Flip: inverted, the
     * ToF and the touchdown test mean nothing until it is level again */
    if (s_auto == PV2_AUTO_LAUNCH || s_auto == PV2_AUTO_FLIP) return;
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
    if (!s_armed) return;
    if (!alt_ok()) { air_ground_no_alt(now, dt); return; }
    s_air_thr = 0u; s_air_thr_ms = 0u;

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
            s_mode = pos_source() ? PV2_MODE_POS_HOLD : PV2_MODE_ALT_HOLD;
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
    const uint8_t bad = (uint8_t)(q == 0xFFu || q < FC_FLOW_LOST_Q || !s_v.flow_usable);
    /* below the PMW3901 minimum height (on the floor, lift-off, last centimetres of
     * a landing) the flow is invalid by design: not a failure */
    const uint8_t low = (uint8_t)(s_v.tof_ground || (s_v.tof_valid && s_v.tof_h < FC_FLOW_CHECK_H));
    const uint8_t judged = (uint8_t)(s_auto != PV2_AUTO_TAKEOFF && s_auto != PV2_AUTO_LAND && !low);
    if (s_armed && s_in_air && s_mode == PV2_MODE_POS_HOLD && judged && bad) {
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

/* collective from the throttle stick (ACRO / ANGLE, or no altitude estimate):
 * bottom = 0, centre = the learned hover, top = 1. Centre = hover makes the
 * change between the altitude-hold modes and the manual ones free of jumps:
 * the app puts the throttle stick at the centre when the mode changes in the
 * air (§6). */
static float collective(float stick)
{
    const float st = clampf(stick, -1.0f, 1.0f);
    return (st <= 0.0f) ? (st + 1.0f) * s_hover : s_hover + st * (1.0f - s_hover);
}

/* geofence on the height (§6, gfMaxAlt / gfCeiling): *vz limited; flags and EVENT 23 */
static void geofence_vz(float *vz)
{
    int32_t gf_kind;
    const float ceil = ceiling_h_kind(&gf_kind);
    if (s_v.h >= ceil && *vz > 0.0f) {
        *vz = (s_v.h > ceil + 0.3f) ? -0.3f : 0.0f;
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
}

/* geofence on the height in a closed-loop vertical mode (§6): flags, EVENT 23, and the cut of a climb that the law
 * applies (ref.vz_ceiling). climbing: the reference asks to go up (a climb rate > 0, or a height above). */
static float geofence_ceiling(uint8_t climbing)
{
    float vz = climbing ? 1.0f : 0.0f;
    geofence_vz(&vz);
    return (climbing && vz <= 0.0f) ? vz : CTL_NO_LIMIT;
}

/* the reference for the control law (ctl_api.h): what the drone must do, not how */
static void ref_attitude(ctl_ref_t *r, const fc_setpoint_t *sp)
{
    r->roll = sp->roll; r->pitch = sp->pitch;
    r->rate[0] = sp->roll_rate; r->rate[1] = sp->pitch_rate; r->rate[2] = sp->yaw_rate;
}

static void setpoints(uint32_t now, uint32_t dt_ms)
{
    (void)dt_ms;
    const float beg = (PARAM(P_BEGINNER) > 0.5f) ? 0.5f : 1.0f;
    fc_setpoint_t sp;
    memset(&sp, 0, sizeof sp);
    sp.armed = s_armed; sp.mode = s_mode; sp.autom = s_auto; sp.in_air = s_in_air;
    ctl_ref_t r;
    memset(&r, 0, sizeof r);
    r.armed = s_armed; r.in_air = s_in_air;
    r.vz_ceiling = CTL_NO_LIMIT; r.thrust_max = 1.0f;

    /* entering HOLD / failsafe HOLD: hold the current height */
    if (s_auto != s_prev_auto && (s_auto == PV2_AUTO_HOLD || s_auto == PV2_AUTO_FAILSAFE)) s_h_sp = s_v.h;
    s_prev_auto = s_auto;

    /* attitude from the sticks (neutral when the RC is stale) */
    const float tilt = PARAM(P_MAX_TILT) * beg * DEG2RAD;
    const float yawr = PARAM(P_MAX_YAW_RATE) * beg * DEG2RAD;
    sp.roll  = s_stick[3] * tilt;
    sp.pitch = s_stick[2] * tilt;
    sp.roll_rate  = s_stick[3] * 4.0f * yawr;           /* ACRO: 4x the yaw rate scale */
    sp.pitch_rate = s_stick[2] * 4.0f * yawr;
    sp.yaw_rate   = -s_stick[1] * yawr;                  /* stick right = clockwise = -z (FLU) */
    r.att = (uint8_t)((s_mode == PV2_MODE_ACRO && s_auto == PV2_AUTO_NONE) ? CTL_ATT_RATE : CTL_ATT_ANGLE);
    r.tilt_max = tilt;

    if (!s_armed) { s_ref_sp = sp; s_ref = r; ref_attitude(&s_ref, &s_ref_sp); return; }
    if (s_auto == PV2_AUTO_LAUNCH) {
        /* hand launch (launch.c): motors stopped in the hand, then thrust and level attitude */
        sp.motors_off = r.motors_off = s_lno.motors_off;
        sp.roll = sp.pitch = sp.roll_rate = sp.pitch_rate = sp.yaw_rate = 0.0f;
        r.att = CTL_ATT_ANGLE;
        r.vert = CTL_V_COLLECTIVE;
        r.collective = s_lno.motors_off ? 0.0f : s_lno.thrust;
        s_ref_sp = sp; s_ref = r; ref_attitude(&s_ref, &s_ref_sp);
        return;
    }

    if (s_auto == PV2_AUTO_FLIP) {
        /* flip (flip.c): body rates while rotating, level otherwise; the collective of the phase */
        r.att = s_flo.rate_mode ? CTL_ATT_RATE : CTL_ATT_ANGLE;
        sp.roll = sp.pitch = 0.0f;
        sp.roll_rate = s_flo.rate[0]; sp.pitch_rate = s_flo.rate[1]; sp.yaw_rate = 0.0f;
        r.vert = CTL_V_COLLECTIVE;
        r.collective = clampf(s_flo.collective, 0.0f, 1.0f);
        s_ref_sp = sp; s_ref = r; ref_attitude(&s_ref, &s_ref_sp);
        return;
    }

    /* the pilot put the stick at the bottom after a touchdown: manual take-off allowed again */
    if (s_land_idle && !s_rc_stale && s_stick[0] < FC_THR_LOW) s_land_idle = 0u;

    const uint8_t vert = (uint8_t)(s_in_air && alt_ok() &&
                                   (s_auto != PV2_AUTO_NONE || hold_mode()));
    if (!vert) {
        float thr;
        r.vert = CTL_V_COLLECTIVE;
        if (!s_in_air && (hold_mode() || s_land_idle)) {
            thr = 0.0f;                                      /* hold modes take off with TAKEOFF; idle after a landing */
            s_land_ol = 0u;
        } else if (s_in_air && (s_auto == PV2_AUTO_LAND || s_auto == PV2_AUTO_RTH)) {
            /* no altitude estimate: open-loop descent, from the collective at the start
             * to FC_LAND_OL_K x hover in FC_LAND_OL_RAMP_MS (touchdown in air_ground_no_alt) */
            if (!s_land_ol) { s_land_ol = 1u; s_land_ol_t0 = now; s_land_ol_from = s_sp.thrust; }
            const float k = clampf((float)(now - s_land_ol_t0) / (float)FC_LAND_OL_RAMP_MS, 0.0f, 1.0f);
            thr = s_land_ol_from + (FC_LAND_OL_K * s_hover - s_land_ol_from) * k;
        } else if (s_in_air && (s_auto == PV2_AUTO_HOLD || s_auto == PV2_AUTO_FAILSAFE)) {
            thr = s_hover;                                   /* open-loop hover: drifts in height */
            s_land_ol = 0u;
        } else {
            thr = collective(s_stick[0]);                    /* ACRO / ANGLE (any mode without an estimate) */
            s_land_ol = 0u;
            /* geofence also in the manual modes: the collective may not push above the ceiling. The climb rate
             * allowed falls with the distance to the ceiling (braking at FC_GF_BRAKE_A), so a full-throttle climb
             * stops in time; the law limits the collective only while it asks for less. */
            if (s_in_air && alt_ok()) {
                const float d = ceiling_h() - s_v.h;
                r.vz_ceiling = (d > 0.0f) ? sqrtf(2.0f * FC_GF_BRAKE_A * d) : (d < -0.3f ? -0.3f : 0.0f);
                float vz_now = s_v.vz;
                geofence_vz(&vz_now);                         /* flags and EVENT 23 at the ceiling */
            }
        }
        r.collective = clampf(thr, 0.0f, 1.0f);
        s_ref_sp = sp; s_ref = r; ref_attitude(&s_ref, &s_ref_sp);
        return;
    }
    s_land_ol = 0u;

    const float maxc = PARAM(P_MAX_CLIMB) * beg;
    const float land = PARAM(P_LAND_SPEED);
    uint8_t climbing;
    switch (s_auto) {
    case PV2_AUTO_TAKEOFF:
        r.vert = CTL_V_HEIGHT; r.h_sp = s_to_target; r.vz_up = FC_TAKEOFF_VZ; r.vz_dn = land;
        climbing = (uint8_t)(s_to_target > s_v.h);
        s_h_sp = s_v.h;
        break;
    case PV2_AUTO_LAND:
    case PV2_AUTO_RTH:                       /* no GNSS here: RTH lands in place */
        r.vert = CTL_V_CLIMB;
        r.vz_sp = -((has(PV2_S_TOF) && s_v.tof_valid && s_v.tof_h < 1.0f) ? FC_LAND_SLOW_VZ : land);
        if (s_v.tof_ground) r.vz_sp = -FC_LAND_SLOW_VZ;
        climbing = 0u;
        s_h_sp = s_v.h;
        break;
    case PV2_AUTO_HOLD:
    case PV2_AUTO_FAILSAFE:
        r.vert = CTL_V_HEIGHT; r.h_sp = s_h_sp; r.vz_up = maxc; r.vz_dn = land;
        climbing = (uint8_t)(s_h_sp > s_v.h);
        break;
    default:                                 /* ALT_HOLD / POS_HOLD, pilot: climb rate */
        if (fabsf(s_stick[0]) < 0.05f) {
            r.vert = CTL_V_HEIGHT; r.h_sp = s_h_sp; r.vz_up = maxc; r.vz_dn = land;
            climbing = (uint8_t)(s_h_sp > s_v.h);
        } else {
            r.vert = CTL_V_CLIMB; r.vz_sp = s_stick[0] * maxc; s_h_sp = s_v.h;
            climbing = (uint8_t)(r.vz_sp > 0.0f);
        }
        break;
    }
    r.vz_ceiling = geofence_ceiling(climbing);          /* may also lower s_h_sp and the take-off target */

    /* take-off spool-up: thrust ramps from idle until the drone leaves the ground; the ramp is the upper limit of
     * the collective, so the integrator does not wind up */
    if (s_auto == PV2_AUTO_TAKEOFF && !s_to_lifted) r.thrust_max = 0.10f + 0.40f * (float)(now - s_to_t0) * 1e-3f;
    sp.alt_ctrl = 1u;
    sp.h_sp = s_h_sp;
    s_ref_sp = sp; s_ref = r; ref_attitude(&s_ref, &s_ref_sp);
}

/* ---- hand launch (launch.c, ADR 0011): the module decides, the core applies ---- */
static void launch_done(void)
{
    s_auto = PV2_AUTO_NONE;
    s_mode = pos_source() ? PV2_MODE_POS_HOLD : PV2_MODE_ALT_HOLD;   /* as after TAKEOFF */
    s_h_sp = s_v.h;                                    /* the height reached (the law enters its loop fresh) */
    event(PV2_EV_INFO, PV2_E_MODE_CHANGED, s_mode);
}

static void launch_abort(uint8_t reason)
{
    event(PV2_EV_WARN, PV2_E_LAUNCH_ABORTED, reason);
    if (reason == LN_AB_NOT_RECOVERED && s_in_air) { s_auto = PV2_AUTO_LAND; s_touch_ms = 0u; return; }
    disarm();                                          /* in the hand, or no attitude estimate: motors off */
}

static void launch_service(uint8_t est_running, uint32_t dt_ms)
{
    if (s_auto != PV2_AUTO_LAUNCH) {                   /* ended by a command, a failsafe or a disarm */
        if (s_ln.phase) { ln_cancel(&s_ln); memset(&s_lno, 0, sizeof s_lno); }
        return;
    }
    const ln_in_t in = { s_v.f_g, { s_v.a_w[0], s_v.a_w[1], s_v.a_w[2] }, s_v.w, s_v.cos_tilt, s_v.gyro_sat,
                         s_v.vz, (uint8_t)(est_running && s_have_v), pilot_link_ok(), s_hover };
    ln_step(&s_ln, &in, dt_ms, &s_lno);
    if (s_lno.released) {
        lift_off();
        event(PV2_EV_INFO, PV2_E_LAUNCH_RELEASED, s_lno.start_ms);
    }
    if (s_lno.phase == LN_DONE) { ln_cancel(&s_ln); launch_done(); }
    else if (s_lno.phase == LN_ABORT) { const uint8_t r = s_lno.abort_reason; ln_cancel(&s_ln); launch_abort(r); }
}

/* ---- autonomous flip (flip.c, ADR 0012): the module decides, the core applies ---- */
static void flip_service(uint8_t est_running, uint32_t dt_ms)
{
    if (s_auto != PV2_AUTO_FLIP) {                     /* ended by a command, a failsafe or a disarm */
        if (s_fl.phase) { fl_cancel(&s_fl); memset(&s_flo, 0, sizeof s_flo); }
        return;
    }
    fl_in_t in;
    memset(&in, 0, sizeof in);
    memcpy(in.w, s_v.w_b, sizeof in.w);
    in.cos_tilt = s_v.cos_tilt; in.vz = s_v.vz; in.hover = s_hover;
    if (!s_rc_stale) in.stick = fmaxf(fabsf(s_stick[2]), fabsf(s_stick[3]));
    in.link_ok = pilot_link_ok(); in.gyro_sat = s_v.gyro_sat;
    in.est_ok = (uint8_t)(est_running && s_have_v);
    const uint8_t phase = s_fl.phase;
    fl_step(&s_fl, &in, dt_ms, &s_flo);
    if (s_v.h < s_fl_hmin) s_fl_hmin = s_v.h;
    if (s_flo.phase == FL_DONE) {
        s_auto = PV2_AUTO_NONE;
        s_mode = s_fl_mode;                            /* the mode of before, at the starting height */
        s_h_sp = s_fl_h0;
        if (s_flo.abort_reason) event(PV2_EV_WARN, PV2_E_FLIP_ABORTED, (int32_t)((FL_ROTATE << 8) | s_flo.abort_reason));
        else event(PV2_EV_INFO, PV2_E_FLIP_DONE, (int32_t)((s_fl_h0 - s_fl_hmin) * 100.0f + 0.5f));
        fl_cancel(&s_fl);
    } else if (s_flo.phase == FL_ABORT) {
        event(PV2_EV_WARN, PV2_E_FLIP_ABORTED, (int32_t)((phase << 8) | s_flo.abort_reason));
        fl_cancel(&s_fl);
        s_auto = PV2_AUTO_LAND; s_touch_ms = 0u;       /* not level in time: land */
    }
}

/* ---- horizontal position (pos_ctrl.c): after setpoints(), it may replace roll / pitch ---- */
static uint8_t auto_holds_position(void)
{
    return (uint8_t)(s_auto == PV2_AUTO_TAKEOFF || s_auto == PV2_AUTO_LAND || s_auto == PV2_AUTO_RTH ||
                     s_auto == PV2_AUTO_HOLD || s_auto == PV2_AUTO_FAILSAFE);
}

/* the horizontal group of the reference: a velocity (zero = hold the point) while a position can be held */
static void horizontal_ref(void)
{
    ctl_ref_t *r = &s_ref;
    const uint8_t wanted = (uint8_t)(s_mode == PV2_MODE_POS_HOLD || auto_holds_position());
    /* a position needs the flow above its minimum height, and the height held by the altitude loop */
    s_ref_sp.pos_hold = (uint8_t)(s_armed && s_in_air && s_ref_sp.alt_ctrl && wanted && s_have_v && s_v.flow_usable &&
                                  !(s_failsafe & PV2_FS_FLOW));
    if (!s_ref_sp.pos_hold) { r->horiz = CTL_H_NONE; return; }
    const float beg = (PARAM(P_BEGINNER) > 0.5f) ? 0.5f : 1.0f;
    float vmax = PARAM(P_MAX_HSPEED) * beg;
    if (indoor() && vmax > FC_INDOOR_HSPEED) vmax = FC_INDOOR_HSPEED;
    /* the pilot moves the drone only in POS_HOLD; an automatic action holds the point */
    const uint8_t pilot = (uint8_t)(s_auto == PV2_AUTO_NONE && !s_rc_stale);
    r->horiz = CTL_H_VELOCITY;
    r->v_max = vmax;
    r->vel_sp[0] = pilot ? s_stick[2] * vmax : 0.0f;          /* pitch stick + forward  */
    r->vel_sp[1] = pilot ? -s_stick[3] * vmax : 0.0f;         /* roll stick + right = -y */
}

/* the control law (ctl_api.h): the state and the reference in, the status out; fc_setpoint() is the manager
 * part of the reference with what the law did (collective, attitude targets, climb rate) */
static void control_step(uint32_t dt_ms)
{
    ctl_vehicle_t veh;
    veh.h_valid = (uint8_t)(s_have_v && s_v.valid);
    veh.h = s_v.h; veh.vz = s_v.vz; veh.a_up = s_v.a_up;
    veh.cos_tilt = s_have_v ? s_v.cos_tilt : 1.0f; veh.yaw = s_v.yaw;
    veh.vel_valid = (uint8_t)(s_have_v && s_v.flow_usable);
    veh.vel_b[0] = s_v.vel_b[0]; veh.vel_b[1] = s_v.vel_b[1];
    ctl_io_feed_vehicle(&veh);
    ctl_io_feed_hover(s_hover);
    ctl_io_set_ref(&s_ref);
    ctl_io_outer((float)dt_ms * 1e-3f);
    ctl_status_t st;
    ctl_get_status(&st);
    fc_setpoint_t sp = s_ref_sp;
    sp.thrust = s_ref.armed ? st.collective : 0.0f;
    if (s_ref.horiz == CTL_H_VELOCITY) { sp.roll = st.roll; sp.pitch = st.pitch; }
    if (sp.alt_ctrl) sp.vz_sp = st.vz_sp;
    s_sp = sp;
    s_ctl_sat = st.saturated;
}

/* ---- hover thrust (hover_track.c): the flight core only reports its state ---- */
static uint8_t hover_airborne(uint32_t now)
{
    return (uint8_t)(s_in_air && alt_ok() && !s_v.tof_ground && s_auto != PV2_AUTO_LAUNCH && s_auto != PV2_AUTO_FLIP &&
        (now - s_lift_ms) > FC_HOVER_SPOOL_MS &&
        !(s_auto == PV2_AUTO_TAKEOFF && !s_to_lifted) &&
        !(s_v.tof_valid && s_v.tof_h < FC_HOVER_MIN_TOF_H) &&
        !s_geofence && !s_land_idle);
}

static void hover_update(uint32_t now)
{
    hover_gate_t g;
    g.airborne = hover_airborne(now);
    g.saturated = s_ctl_sat;                         /* the collective at a limit of the law */
    g.a_up = s_v.a_up;
    g.thrust = s_sp.thrust;
    g.cos_tilt = s_v.cos_tilt;
    g.vz = s_v.vz;
    if (hover_track_step(now, s_armed, &g))
        event(PV2_EV_WARN, PV2_E_HOVER_THRUST, (int32_t)(hover_track_value() * 1000.0f + 0.5f));
    s_hover = hover_track_value();
}

static void publish(void)
{
    s_snap_seq++;
    SPSC_BARRIER();
    s_snap.armed = s_armed; s_snap.mode = s_mode; s_snap.autom = s_auto;
    s_snap.failsafe = s_failsafe; s_snap.in_air = s_in_air;
    s_snap.input = s_radio_ctrl ? PV2_INPUT_ELRS : ((s_silence < FC_FS_SHORT_MS) ? PV2_INPUT_APP : PV2_INPUT_NONE);
    s_snap.rc_stale = s_rc_stale;
    s_snap.fs_long = s_fs_long;
    s_snap.flight_time_s = (uint16_t)((s_flight_ms / 1000u > 0xFFFFu) ? 0xFFFFu : s_flight_ms / 1000u);
    memset(s_snap.motor_out_pct, 0, sizeof s_snap.motor_out_pct);
    if (s_test_motor) s_snap.motor_out_pct[s_test_motor - 1u] = s_test_pct;
    s_snap.test_motor = s_test_motor;
    s_snap.landed = s_landed;
    s_snap.geofence = s_geofence;
    s_snap.batt_level = s_batt_level;
    s_snap.batt_stale = (uint8_t)(s_cfg.batt_monitor && s_batt_stale);
    if (s_have_v && s_v.valid) {
        const float a = (s_v.h - (s_armed || s_took_off ? s_h_ref : s_v.h)) * 100.0f;
        s_snap.alt_rel_cm = (int32_t)(a + (a >= 0.0f ? 0.5f : -0.5f));
        const float w = clampf(s_v.vz * 100.0f, -32767.0f, 32767.0f);
        s_snap.vz_cms = (int16_t)w;
    } else {
        s_snap.alt_rel_cm = 0; s_snap.vz_cms = 0;
    }
    s_snap.silence_ms = s_silence;
    s_snap.land_idle = s_land_idle;
    s_snap.air_by_thrust = s_air_thr;
    s_snap.hover = s_hover;
    s_snap.hover_sd = hover_track_sd();
    s_snap.hover_nis = hover_track_nis();
    memcpy(s_snap.stick, s_stick, sizeof s_stick);
    SPSC_BARRIER();
    s_snap_seq++;
}

void fc_service(uint32_t now, uint8_t est_running, const fc_vert_t *v)
{
    s_now = now;
    const uint32_t dt = s_have_last ? (now - s_last_ms) : 0u;
    s_last_ms = now; s_have_last = 1u;
    if (v) { s_v = *v; s_have_v = 1u; } else { s_have_v = 0u; s_v.valid = 0u; s_v.flow_q = 0xFFu; }

    update_sticks(now);

    fc_cmd_t c;
    while (spsc_pop(&s_cmdq, &c)) exec(&c, now, est_running);
    radio_switches(now, est_running);
    launch_service(est_running, dt);
    flip_service(est_running, dt);

    air_ground(now, dt);

    app_link_failsafe(now);
    radio_failsafe();
    flow_failsafe(dt);
    battery(now);

    /* ---- HOLD began: the throttle stick at that moment is the cancel reference ---- */
    if (s_auto == PV2_AUTO_HOLD && !s_hold_prev) s_hold_thr = s_stick[0];
    s_hold_prev = (uint8_t)(s_auto == PV2_AUTO_HOLD);

    /* ---- pilot cancels HOLD (and RTH, if allowed) with the sticks ---- */
    if (s_armed && s_in_air && pilot_link_ok()) {
        if (s_auto == PV2_AUTO_HOLD && stick_moved()) s_auto = PV2_AUTO_NONE;
        if (s_auto == PV2_AUTO_RTH && PARAM(P_RTH_STICK_CANCEL) > 0.5f &&
            !s_rc_stale && (s_rc.pitch > FC_STICK_CANCEL || s_rc.pitch < -FC_STICK_CANCEL ||
                            s_rc.roll > FC_STICK_CANCEL || s_rc.roll < -FC_STICK_CANCEL))
            s_auto = PV2_AUTO_NONE;
    }

    /* ---- automatic disarm ---- */
    if (s_armed && !s_in_air && s_auto != PV2_AUTO_LAUNCH) {       /* the launch has its own timeout */
        if (!s_took_off && (now - s_arm_ms) > FC_NO_TAKEOFF_MS) disarm();
        else if (s_took_off && (now - s_landed_ms) > (uint32_t)(PARAM(P_AUTO_DISARM) * 1000.0f)) disarm();
    }
    if (s_armed && s_in_air) s_flight_ms += dt;

    /* ---- motor test: stops after ms, on link loss ---- */
    if (s_test_motor && ((int32_t)(now - s_test_until) >= 0 || s_silence > FC_FS_SHORT_MS)) test_stop();

    setpoints(now, dt);
    horizontal_ref();
    control_step(dt);
    hover_update(now);
    publish();
}
