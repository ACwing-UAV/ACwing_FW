/**
 * @file  app_task.c
 * @brief AppTask (see app_task.h). Behavioural reference: the app simulator
 *        src/sim/simDrone.ts (command(), tick()) and HANDOVER_FW.md §3-§11.
 *
 * The protocol logic is app_step(now): pure C on top of host-testable modules
 * (tlm_ble, app_link, fc_core, params, calib_mgr, cfg_store, checks). The RTOS
 * part is only the task loop at the end of the file (not built with HOST_TEST).
 */
#include "app_task.h"
#include "proto_v2.h"
#include "ack_cache.h"
#include "app_link.h"
#include "tlm_ble.h"
#include "fc_core.h"
#include "params.h"
#include "cfg_store.h"
#include "checks.h"
#include "fc_shared.h"
#include "ext_sens.h"
#include "batt_task.h"
#include <string.h>
#include <math.h>

#define FRAME_CAP        64u
#define CORE_PENDING_MS  1000u    /* a core command not answered after 1 s: stop waiting */
#define REPEAT_N         3u       /* ERROR / CRITICAL events: 3 copies ...               */
#define REPEAT_MS        100u     /* ... 100 ms apart                                     */
#define CAL_LINK_ABORT_MS 3000u   /* calibration aborted when the app is silent           */
#define LIST_HEADROOM    256u     /* bytes left in the HI ring for ACKs while listing     */
#define EV_RATE_MS       30000u   /* MAG_INTERFERENCE / HIGH_VIBRATION                   */
#define VIB_WARN_MG      1500.0f

/* ------------------------------------------------------------------ state */
static app_task_cfg_t s_cfg;
static cfg_image_t    s_img;              /* what is (or will be) in flash */
static cal_set_t      s_cal;              /* calibrations in effect (RAM)  */
static uint16_t       s_sensors;
static uint8_t        s_frame[FRAME_CAP];

/* core command waiting for its result */
static uint8_t  s_pend, s_pend_op, s_pend_seq;
static uint32_t s_pend_t;

/* stream rates (Hz) and next send times */
static uint8_t  s_hz_flight = 10, s_hz_power = 5, s_hz_nav = 5, s_hz_motors = 5, s_hz_system = 2;
static uint32_t s_t_power;
static uint16_t s_fl_count;
static uint32_t s_t_flight, s_t_nav, s_t_motors, s_t_system, s_t_checks, s_t_caps, s_t_calib, s_t_ground;

/* paced PARAM list */
static int      s_list_next = -1;         /* table index, -1 = no list running */

/* ERROR+ event copies */
typedef struct { uint8_t f[FRAME_CAP]; uint8_t len, left; uint32_t next; } rep_t;
static rep_t    s_rep[6];

/* rate-limited events */
static uint32_t s_t_ev_mag, s_t_ev_vib;
static uint8_t  s_have_ev_mag, s_have_ev_vib;

/* checks */
static uint8_t  s_ready, s_batt_fail;

/* ------------------------------------------------------------------ helpers */
static uint8_t armed_now(void);
static void tx(const uint8_t *f, size_t len) { if (len) (void)tlm_ble_push_hi(f, (uint16_t)len); }

static uint32_t bit(uint8_t type) { return 1u << (type - 1u); }

static void ack(uint8_t op, uint8_t seq, uint8_t res, uint8_t why)
{
    ackc_finish(op, seq, res, why);
    tx(s_frame, pv2_ack(op, seq, res, why, s_frame, sizeof s_frame));
}

static const char *ev_text(uint16_t code)
{
    switch (code) {
    case PV2_E_ARMED:              return "Armed";
    case PV2_E_DISARMED:           return "Disarmed";
    case PV2_E_TAKEOFF_STARTED:    return "Takeoff";
    case PV2_E_LANDED:             return "Landed";
    case PV2_E_MODE_CHANGED:       return "Mode changed";
    case PV2_E_RTH_STARTED:        return "Return to home";
    case PV2_E_HOME_SET:           return "Home set";
    case PV2_E_TAKEOFF_ABORTED:    return "Takeoff aborted: no lift-off";
    case PV2_E_FAILSAFE_APP_LINK:  return "App link lost: failsafe";
    case PV2_E_FAILSAFE_RADIO:     return "Radio lost: failsafe";
    case PV2_E_FAILSAFE_BATTERY:   return "Battery failsafe";
    case PV2_E_GEOFENCE_BREACH:    return "Geofence reached";
    case PV2_E_GPS_LOST:           return "GPS lost";
    case PV2_E_FLOW_LOST:          return "Optical flow lost";
    case PV2_E_BATTERY_LOW:        return "Battery low";
    case PV2_E_BATTERY_CRITICAL:   return "Battery critical";
    case PV2_E_BATTERY_EMERGENCY:  return "Battery emergency: landing";
    case PV2_E_BATTERY_MONITOR_LOST: return "Battery monitor stopped";
    case PV2_E_MAG_INTERFERENCE:   return "Magnetic interference";
    case PV2_E_HIGH_VIBRATION:     return "High vibration";
    case PV2_E_MOTOR_HOT:          return "Motor hot";
    case PV2_E_RADIO_OVERRIDE:     return "Radio took control";
    case PV2_E_CALIB_DONE:         return "Calibration saved";
    case PV2_E_CALIB_FAILED:       return "Calibration failed";
    case PV2_E_PARAMS_SAVED:       return "Settings saved";
    case PV2_E_MOTOR_KILLED:       return "Motors killed";
    default:                       return "";
    }
}

static void send_event(uint8_t level, uint16_t code, int32_t arg, uint32_t now)
{
    const size_t n = pv2_event(level, code, arg, ev_text(code), s_frame, sizeof s_frame);
    tx(s_frame, n);
    if (level < PV2_EV_ERROR || !n) return;
    for (unsigned i = 0; i < sizeof s_rep / sizeof s_rep[0]; ++i) {
        if (s_rep[i].left) continue;
        memcpy(s_rep[i].f, s_frame, n);
        s_rep[i].len = (uint8_t)n;
        s_rep[i].left = REPEAT_N - 1u;
        s_rep[i].next = now + REPEAT_MS;
        return;
    }
}

static void repeats(uint32_t now)
{
    for (unsigned i = 0; i < sizeof s_rep / sizeof s_rep[0]; ++i) {
        rep_t *r = &s_rep[i];
        if (!r->left || (int32_t)(now - r->next) < 0) continue;
        tx(r->f, r->len);
        r->left--;
        r->next = now + REPEAT_MS;
    }
}

static uint16_t sensors_now(void)
{
    fsh_est_t e;
    memset(&e, 0, sizeof e);
    fsh_est_read(&e);
    return (uint16_t)(e.sensors | ext_sensors() | s_cfg.board_sensors);
}

static batt_state_t batt_now(void)
{
    batt_state_t b;
    memset(&b, 0, sizeof b);
    if (s_cfg.batt_monitor) batt_read(&b);
    return b;
}

/* ------------------------------------------------------------------ frames */
static void send_caps(void)
{
    pv2_caps_t c;
    memset(&c, 0, sizeof c);
    c.proto = PV2_PROTO_VERSION;
    c.fw[0] = APP_FW_MAJOR; c.fw[1] = APP_FW_MINOR; c.fw[2] = APP_FW_PATCH;
    c.hw_rev = (uint8_t)s_cfg.hw_rev;
    c.model = (s_sensors & PV2_S_GNSS) ? 1u : 0u;
    c.sensors = s_sensors;
    c.n_motors = s_cfg.n_motors;
    c.frame = (s_cfg.n_motors == 8u) ? 2u : (s_cfg.n_motors == 6u ? 1u : 0u);
    c.cells = batt_now().cells;                     /* 0 = not known yet */
    memcpy(c.name, s_img.name, sizeof c.name);
    {
        uint32_t h = 2166136261u;                   /* FNV-1a of the 96-bit UID */
        for (int i = 0; i < 3; ++i)
            for (int b = 0; b < 4; ++b) { h ^= (s_cfg.uid[i] >> (8 * b)) & 0xFFu; h *= 16777619u; }
        static const char hex[] = "0123456789ABCDEF";
        memcpy(c.serial, "ACW-", 4);
        for (int i = 0; i < 8; ++i) c.serial[4 + i] = hex[(h >> (28 - 4 * i)) & 0xFu];
    }
    tx(s_frame, pv2_frame(PV2_FT_CAPS, &c, sizeof c, s_frame, sizeof s_frame));
}

static float yaw_heading_deg(const float q[4])
{
    const float w = q[0], x = q[1], y = q[2], z = q[3];
    const float yaw = atan2f(2.0f * (w*z + x*y), 1.0f - 2.0f * (y*y + z*z));   /* NWU, CCW + */
    float h = -yaw * 57.29578f;                                                  /* compass, CW + */
    while (h < 0.0f) h += 360.0f;
    while (h >= 360.0f) h -= 360.0f;
    return h;
}

static void send_flight(void)
{
    fc_state_t f;
    memset(&f, 0, sizeof f);
    fc_snapshot(&f);
    fsh_est_t e;
    memset(&e, 0, sizeof e);
    fsh_est_read(&e);
    pv2_flight_t m;
    memset(&m, 0, sizeof m);
    m.arm = f.armed; m.mode = f.mode; m.autom = f.autom; m.failsafe = f.failsafe;
    uint16_t fl = 0;
    if (f.in_air) fl |= PV2_FF_IN_AIR;
    if (f.landed) fl |= PV2_FF_LANDED;
    if (s_sensors & (PV2_S_BARO | PV2_S_TOF | PV2_S_GNSS)) fl |= PV2_FF_GEOFENCE_ON;
    if (PARAM(P_BEGINNER) > 0.5f) fl |= PV2_FF_BEGINNER;
    if (!f.armed && !s_ready) fl |= PV2_FF_ARM_BLOCKED;
    m.flags = fl;
    m.input = f.input;
    m.flight_time_s = f.flight_time_s;
    m.alt_rel_cm = f.alt_rel_cm;
    m.vz_cms = f.vz_cms;
    if (e.flow_usable) {
        const float gs = sqrtf(e.vel_body[0]*e.vel_body[0] + e.vel_body[1]*e.vel_body[1]) * 100.0f;
        m.ground_speed_cms = (uint16_t)(gs > 65535.0f ? 65535.0f : gs);
    }
    m.heading_cdeg = (uint16_t)(yaw_heading_deg(e.q) * 100.0f + 0.5f) % 36000u;
    m.home_dist_m = 0xFFFFu;
    m.home_bearing_cdeg = 0u;
    m.rth_eta_s = 0xFFFFu;
    tx(s_frame, pv2_frame(PV2_FT_FLIGHT, &m, sizeof m, s_frame, sizeof s_frame));
}

static int16_t sat16(float v) { return (int16_t)(v > 32767.0f ? 32767 : (v < -32768.0f ? -32768 : (int)v)); }

static void send_nav(void)
{
    fsh_est_t e;
    memset(&e, 0, sizeof e);
    fsh_est_read(&e);
    pv2_nav_t n;
    memset(&n, 0, sizeof n);
    n.fix = 0xFFu;                                   /* no GNSS hardware */
    n.hdop_x100 = 0xFFFFu;
    if (e.baro_pa) {
        n.alt_msl_cm = (int32_t)(44330.0f * (1.0f - powf((float)e.baro_pa / 101325.0f, 0.190295f)) * 100.0f);
        n.baro_pa = e.baro_pa;
    }
    {
        const float psi = -yaw_heading_deg(e.q) * 0.01745329f;      /* NWU yaw */
        float vn = 0.0f, ve = 0.0f;
        if (e.flow_usable) {
            const float vx = e.vel_body[0], vy = e.vel_body[1];       /* body x fwd, y left */
            vn = cosf(psi) * vx - sinf(psi) * vy;
            ve = -(sinf(psi) * vx + cosf(psi) * vy);                  /* E = -W */
        }
        n.vel_ned_cms[0] = sat16(vn * 100.0f);
        n.vel_ned_cms[1] = sat16(ve * 100.0f);
        n.vel_ned_cms[2] = e.alt_valid ? sat16(-e.vz * 100.0f) : 0;
    }
    n.tof_mm = e.tof_mm;
    n.flow_q = e.flow_q;
    n.wind_cms = 0xFFFFu;
    tx(s_frame, pv2_frame(PV2_FT_NAV, &n, sizeof n, s_frame, sizeof s_frame));
}

static void send_motors(void)
{
    fc_state_t f;
    memset(&f, 0, sizeof f);
    fc_snapshot(&f);
    pv2_motor_t m[PV2_MOTORS_MAX];
    const uint8_t n = (s_cfg.n_motors > PV2_MOTORS_MAX) ? PV2_MOTORS_MAX : s_cfg.n_motors;
    for (uint8_t i = 0; i < n; ++i) { m[i].rpm = 0xFFFFu; m[i].temp_c = -128; m[i].out_pct = f.motor_out_pct[i]; }
    tx(s_frame, pv2_motors(m, n, s_frame, sizeof s_frame));
}

static void send_system(void)
{
    fsh_est_t e;
    memset(&e, 0, sizeof e);
    fsh_est_read(&e);
    pv2_system_t s;
    memset(&s, 0, sizeof s);
    s.lq = 0xFFu;
    s.cpu_pct = (uint8_t)(e.cpu_pct > 100.0f ? 100.0f : e.cpu_pct);
    s.loop_hz = (uint16_t)(e.loop_hz + 0.5f);
    s.jitter_us = (uint16_t)(e.jitter_us > 65535.0f ? 65535.0f : e.jitter_us);
    s.imu_temp_cc = sat16(e.imu_temp_c * 100.0f);
    s.vibration_mg = (uint16_t)(e.vib_mg > 65535.0f ? 65535.0f : e.vib_mg);
    s.acc_clip = e.acc_clip;
    s.att_unc_cdeg = (uint16_t)(e.att_unc_deg * 100.0f > 65535.0f ? 65535.0f : e.att_unc_deg * 100.0f);
    s.est_flags = (uint8_t)((e.running ? 1u : 0u) | (e.acc_gated ? 2u : 0u) | (e.mag_gated ? 4u : 0u) |
                            (e.mag_interf_pct > 30.0f ? 8u : 0u));
    s.mag_interf_pct = (uint8_t)(e.mag_interf_pct > 100.0f ? 100.0f : e.mag_interf_pct);
    s.log_pct = s_cfg.log_pct ? s_cfg.log_pct() : 0u;
    if (s.log_pct == 0xFFu) s.log_pct = 0u;
    tx(s_frame, pv2_frame(PV2_FT_SYSTEM, &s, sizeof s, s_frame, sizeof s_frame));
}

static uint16_t u16c(float v) { return (uint16_t)(v > 65535.0f ? 65535.0f : (v < 0.0f ? 0.0f : v + 0.5f)); }

static void send_power(void)
{
    const batt_state_t b = batt_now();
    pv2_power_t p;
    memset(&p, 0, sizeof p);
    p.vbat_mv = u16c(b.vbat * 1000.0f);
    p.current_ca = (s_sensors & PV2_S_CURRENT) ? sat16(b.current * 100.0f) : 0;
    p.used_mah = u16c(b.used_mah);
    p.cells = b.cells;
    p.cell_mv = u16c(b.cell_v * 1000.0f);
    p.pct = b.pct;
    p.remain_s = (b.remain_s < 0.0f) ? 0xFFFFu : u16c(b.remain_s > 65534.0f ? 65534.0f : b.remain_s);
    p.warn = b.level;
    tx(s_frame, pv2_frame(PV2_FT_POWER, &p, sizeof p, s_frame, sizeof s_frame));
}

static void build_checks(pv2_check_t *it, uint8_t *n)
{
    fsh_est_t e;
    memset(&e, 0, sizeof e);
    fsh_est_read(&e);
    fc_state_t f;
    memset(&f, 0, sizeof f);
    fc_snapshot(&f);
    checks_in_t in;
    memset(&in, 0, sizeof in);
    in.sensors = s_sensors;
    in.imu_ok = e.imu_ok;
    in.est_running = e.running;
    in.att_unc_deg = e.att_unc_deg;
    in.mag_interf_pct = e.mag_interf_pct;
    in.cal_done_mask = s_img.cal.done_mask;
    {
        appl_rc_t rc; uint32_t t;
        if (appl_rc_read(&rc, &t) && (f.silence_ms < FC_ARM_RC_FRESH_MS) && !f.rc_stale) {
            in.rc_fresh = 1u; in.rc_thr = rc.thr;
        }
    }
    in.hold_mode = (uint8_t)(f.mode == PV2_MODE_ALT_HOLD || f.mode == PV2_MODE_POS_HOLD);
    in.gf_max_alt = PARAM(P_GF_MAX_ALT);
    in.gf_max_dist = PARAM(P_GF_MAX_DIST);
    in.alt_valid = e.alt_valid;
    in.flow_q = e.flow_q;
    in.flow_ok = e.flow_ok;
    in.in_air = f.in_air;
    if (s_cfg.batt_monitor) {
        const batt_state_t b = batt_now();
        in.batt_monitor = 1u;
        in.batt_stale = f.batt_stale;
        in.batt_present = b.present;
        in.batt_level = f.batt_level;
        in.batt_cell_v = b.cell_v;
        in.batt_warn = PARAM(P_BATT_WARN);
        in.batt_crit = PARAM(P_BATT_CRIT);
    }
    *n = checks_build(&in, it, &s_ready, &s_batt_fail);
}

static void send_checks(void)
{
    pv2_check_t it[PV2_CHECKS_MAX];
    uint8_t n = 0;
    build_checks(it, &n);
    tx(s_frame, pv2_checks(s_ready, it, n, s_frame, sizeof s_frame));
}

static void send_init(void)
{
    if (!s_cfg.get_init) return;
    static uint8_t f[160];
    const size_t n = s_cfg.get_init(f, sizeof f);
    if (n) (void)tlm_ble_push_hi(f, (uint16_t)n);
}

static void send_param(int idx)
{
    const param_def_t *d = &g_param_def[idx];
    const int pos = params_visible_pos(idx, s_sensors);
    const uint16_t count = params_visible_count(s_sensors);
    tx(s_frame, pv2_param(d->id, param_get(idx), params_dirty(idx) ? 1u : 0u,
                          (uint16_t)(pos < 0 ? 0 : pos), count, s_frame, sizeof s_frame));
}

static void list_start(void) { s_list_next = 0; }

static void list_pace(void)
{
    while (s_list_next >= 0 && s_list_next < PARAM_COUNT && tlm_ble_hi_free() > LIST_HEADROOM) {
        if (params_visible(s_list_next, s_sensors)) send_param(s_list_next);
        s_list_next++;
    }
    if (s_list_next >= PARAM_COUNT) s_list_next = -1;
}

static void send_calib(void)
{
    const size_t n = cal_frame(s_frame, sizeof s_frame);
    tx(s_frame, n);
}

/* ------------------------------------------------------------------ flash */
static int save_image(void)
{
    float v[PARAM_COUNT];
    params_get_all(v);
    cfg_params_put(&s_img, v);
    return cfg_save(&s_img);
}

/* the flash image as it is, with the parameters last saved (no unsaved edits) */
static int save_image_keep_params(void) { return cfg_save(&s_img); }

/* calibration apply hook (calib_mgr -> ImuTask, BATTERY -> BattTask) */
static void cal_apply(uint8_t type, const cal_set_t *set)
{
    if (type == PV2_CAL_BATTERY) (void)batt_task_post_scales(set->battery.scale_v, set->battery.scale_i);
    else (void)fsh_apply_post(type, set);
}

/* BATTERY calibration readings */
static void cal_batt(float *vbat, float *used, float *scale_i)
{
    *vbat = batt_now().vbat;
    *used = s_img.batt_used_mah;
    *scale_i = s_img.batt_used_scale_i;
}

/* after a flight: keep its charge for the BATTERY calibration (§10.6) */
static void battery_flight(void)
{
    if (!s_cfg.batt_monitor) return;
    batt_flight_t fl;
    memset(&fl, 0, sizeof fl);
    batt_task_last_flight(&fl);
    if (fl.count == s_fl_count) return;
    s_fl_count = fl.count;
    if (fl.used_mah < 20.0f || armed_now()) return;       /* not a flight worth keeping */
    s_img.batt_used_mah = fl.used_mah;
    s_img.batt_used_scale_i = fl.scale_i;
    (void)save_image_keep_params();                       /* disarmed: flash write allowed */
}

/* ------------------------------------------------------------------ commands */
static uint8_t armed_now(void)
{
    fc_state_t f;
    memset(&f, 0, sizeof f);
    fc_snapshot(&f);
    return f.armed;
}

static void local_cmd(uint8_t op, uint8_t seq, const uint8_t *p, uint8_t plen, uint32_t now)
{
    uint8_t why = PV2_R_NONE, res;
    switch (op) {
    case PV2_OP_PARAM_GET: {
        if (plen < 2u) { ack(op, seq, PV2_ACK_INVALID, PV2_R_NONE); return; }
        const int idx = params_index(pv2_rd_u16(p));
        if (idx < 0 || !params_visible(idx, s_sensors)) { ack(op, seq, PV2_ACK_INVALID, PV2_R_UNKNOWN_PARAM); return; }
        ack(op, seq, PV2_ACK_OK, PV2_R_NONE);
        send_param(idx);
        return;
    }
    case PV2_OP_PARAM_SET: {
        if (plen < 6u) { ack(op, seq, PV2_ACK_INVALID, PV2_R_NONE); return; }
        const uint16_t id = pv2_rd_u16(p);
        const int idx = params_index(id);
        if (idx >= 0 && !params_visible(idx, s_sensors)) { ack(op, seq, PV2_ACK_INVALID, PV2_R_UNKNOWN_PARAM); return; }
        res = params_set(id, pv2_rd_f32(&p[2]), &why);
        ack(op, seq, res, why);
        if (res == PV2_ACK_OK) send_param(idx);
        return;
    }
    case PV2_OP_PARAM_SAVE:
        if (armed_now()) { ack(op, seq, PV2_ACK_REJECTED, PV2_R_ARMED); return; }
        if (save_image() != 0) { ack(op, seq, PV2_ACK_REJECTED, PV2_R_NONE); return; }   /* flash error */
        params_mark_saved();
        ack(op, seq, PV2_ACK_OK, PV2_R_NONE);
        send_event(PV2_EV_INFO, PV2_E_PARAMS_SAVED, 0, now);
        list_start();
        return;
    case PV2_OP_PARAM_DEFAULTS:
        if (armed_now()) { ack(op, seq, PV2_ACK_REJECTED, PV2_R_ARMED); return; }
        params_defaults();
        ack(op, seq, PV2_ACK_OK, PV2_R_NONE);
        list_start();
        return;
    case PV2_OP_PARAM_LIST:
        ack(op, seq, PV2_ACK_OK, PV2_R_NONE);
        list_start();
        return;

    case PV2_OP_CAL_START:
        if (plen < 1u) { ack(op, seq, PV2_ACK_INVALID, PV2_R_NONE); return; }
        res = cal_start(p[0], plen >= 2u ? p[1] : 0u, armed_now(), now, &why);
        ack(op, seq, res, why);
        if (res == PV2_ACK_OK) send_calib();
        return;
    case PV2_OP_CAL_CAPTURE:
        ack(op, seq, cal_capture(now, &why), why);
        return;
    case PV2_OP_CAL_ABORT:
        ack(op, seq, cal_abort(&why), why);
        send_calib();
        return;
    case PV2_OP_CAL_SAVE: {
        res = cal_save_check(&why);
        if (res != PV2_ACK_OK) { ack(op, seq, res, why); return; }
        const uint8_t t = cal_type();
        /* the set saved until now becomes the backup of this type */
        cal_copy_type(&s_img.bak, &s_img.cal, t);
        s_img.bak_mask |= bit(t);
        cal_copy_type(&s_img.cal, cal_result(), t);
        s_img.cal.done_mask |= bit(t);
        s_cal.done_mask |= bit(t);
        if (save_image() != 0) { ack(op, seq, PV2_ACK_REJECTED, PV2_R_NONE); return; }
        cal_saved();
        ack(op, seq, PV2_ACK_OK, PV2_R_NONE);
        send_calib();
        send_event(PV2_EV_INFO, PV2_E_CALIB_DONE, t, now);
        return;
    }
    case PV2_OP_CAL_RESTORE: {
        if (plen < 1u) { ack(op, seq, PV2_ACK_INVALID, PV2_R_NONE); return; }
        const uint8_t t = p[0];
        if (t < PV2_CAL_ACCEL || t > PV2_CAL_STICKS) { ack(op, seq, PV2_ACK_INVALID, PV2_R_NONE); return; }
        if (armed_now()) { ack(op, seq, PV2_ACK_REJECTED, PV2_R_ARMED); return; }
        if (cal_running()) { ack(op, seq, PV2_ACK_REJECTED, PV2_R_CALIBRATING); return; }
        if (!(s_img.bak_mask & bit(t))) { ack(op, seq, PV2_ACK_REJECTED, PV2_R_NONE); return; }
        cal_set_t cur = s_img.cal;
        cal_copy_type(&s_img.cal, &s_img.bak, t);     /* swap: a second RESTORE undoes it */
        cal_copy_type(&s_img.bak, &cur, t);
        if (save_image() != 0) { ack(op, seq, PV2_ACK_REJECTED, PV2_R_NONE); return; }
        cal_copy_type(&s_cal, &s_img.cal, t);
        cal_apply(t, &s_cal);
        ack(op, seq, PV2_ACK_OK, PV2_R_NONE);
        return;
    }
    case PV2_OP_CAL_SET_REF:
        if (plen < 6u) { ack(op, seq, PV2_ACK_INVALID, PV2_R_NONE); return; }
        ack(op, seq, cal_set_ref(p[0], p[1], pv2_rd_f32(&p[2]), &why), why);
        return;

    case PV2_OP_MOTOR_REVERSE:
    case PV2_OP_MOTOR_REMAP:
        ack(op, seq, PV2_ACK_UNSUPPORTED, PV2_R_NONE);     /* no DShot / ESC link yet */
        return;

    case PV2_OP_STREAM_RATES: {
        if (plen < 5u) { ack(op, seq, PV2_ACK_INVALID, PV2_R_NONE); return; }
#define RATE(dst, v, lo, hi) do { if ((v) != 0u) dst = (uint8_t)((v) < (lo) ? (lo) : ((v) > (hi) ? (hi) : (v))); } while (0)
        RATE(s_hz_flight, p[0], 1u, 20u);
        RATE(s_hz_power,  p[1], 1u, 10u);
        RATE(s_hz_nav,    p[2], 1u, 10u);
        RATE(s_hz_motors, p[3], 1u, 10u);
        RATE(s_hz_system, p[4], 1u, 5u);
#undef RATE
        ack(op, seq, PV2_ACK_OK, PV2_R_NONE);
        return;
    }
    case PV2_OP_REQUEST:
        if (plen < 1u) { ack(op, seq, PV2_ACK_INVALID, PV2_R_NONE); return; }
        switch (p[0]) {
        case PV2_FT_CAPS:   ack(op, seq, PV2_ACK_OK, PV2_R_NONE); send_caps();   return;
        case PV2_FT_CHECKS: ack(op, seq, PV2_ACK_OK, PV2_R_NONE); send_checks(); return;
        case PV2_FT_INIT:   ack(op, seq, PV2_ACK_OK, PV2_R_NONE); send_init();   return;
        case PV2_FT_FLIGHT: ack(op, seq, PV2_ACK_OK, PV2_R_NONE); send_flight(); return;
        case PV2_FT_NAV:    ack(op, seq, PV2_ACK_OK, PV2_R_NONE); send_nav();    return;
        case PV2_FT_MOTORS: ack(op, seq, PV2_ACK_OK, PV2_R_NONE); send_motors(); return;
        case PV2_FT_SYSTEM: ack(op, seq, PV2_ACK_OK, PV2_R_NONE); send_system(); return;
        case PV2_FT_POWER:
            if (!s_cfg.batt_monitor) { ack(op, seq, PV2_ACK_UNSUPPORTED, PV2_R_NONE); return; }
            ack(op, seq, PV2_ACK_OK, PV2_R_NONE); send_power(); return;
        default:            ack(op, seq, PV2_ACK_UNSUPPORTED, PV2_R_NONE);      return;
        }
    case PV2_OP_SET_NAME: {
        if (armed_now()) { ack(op, seq, PV2_ACK_REJECTED, PV2_R_ARMED); return; }
        if (plen < 1u) { ack(op, seq, PV2_ACK_INVALID, PV2_R_NONE); return; }
        char old[16];
        memcpy(old, s_img.name, sizeof old);
        memset(s_img.name, 0, sizeof s_img.name);
        memcpy(s_img.name, p, plen > 16u ? 16u : plen);
        if (save_image() != 0) { memcpy(s_img.name, old, sizeof old); ack(op, seq, PV2_ACK_REJECTED, PV2_R_NONE); return; }
        ack(op, seq, PV2_ACK_OK, PV2_R_NONE);
        send_caps();
        return;
    }
    default:
        ack(op, seq, PV2_ACK_UNSUPPORTED, PV2_R_NONE);
        return;
    }
}

static void commands(uint32_t now)
{
    /* results of the flight core first: they free the pipeline */
    fc_res_t r;
    while (fc_take_result(&r)) {
        ack(r.op, r.seq, r.result, r.reason);
        if (s_pend && r.op == s_pend_op && r.seq == s_pend_seq) s_pend = 0u;
    }
    if (s_pend && (now - s_pend_t) > CORE_PENDING_MS) s_pend = 0u;   /* cannot happen: the IWDG would reset first */

    appl_cmd_t c;
    while (!s_pend && appl_pop(&c)) {
        if (c.len < 2u) continue;
        const uint8_t op = c.b[0], seq = c.b[1];
        const uint8_t plen = (uint8_t)(c.len - 2u);
        uint8_t res, why;
        const uint8_t st = ackc_begin(op, seq, &res, &why);
        if (st == ACKC_DONE) { tx(s_frame, pv2_ack(op, seq, res, why, s_frame, sizeof s_frame)); continue; }
        if (st == ACKC_PENDING) continue;                 /* its ACK is on the way */

        if (fc_is_core_op(op)) {
            fc_cmd_t fc;
            memset(&fc, 0, sizeof fc);
            fc.op = op; fc.seq = seq;
            fc.len = (uint8_t)(plen > sizeof fc.p ? sizeof fc.p : plen);
            memcpy(fc.p, &c.b[2], fc.len);
            if (fc_post(&fc) != 0) { ack(op, seq, PV2_ACK_BUSY, PV2_R_NONE); continue; }
            s_pend = 1u; s_pend_op = op; s_pend_seq = seq; s_pend_t = now;
        } else {
            /* any other command stops a running motor test (§10.7) */
            const fc_cmd_t stop = { FC_OP_MOTOR_STOP, 0u, 0u, { 0 } };
            (void)fc_post(&stop);
            local_cmd(op, seq, &c.b[2], plen, now);
        }
    }
}

/* ------------------------------------------------------------------ periodic */
static int due(uint32_t now, uint32_t *t, uint32_t period_ms)
{
    if ((int32_t)(now - *t) < 0) return 0;
    *t = now + period_ms;
    return 1;
}

static void events(uint32_t now)
{
    fc_event_t e;
    while (fc_take_event(&e)) send_event(e.level, e.code, e.arg, now);

    const uint8_t failed = cal_take_failed();
    if (failed) send_event(PV2_EV_ERROR, PV2_E_CALIB_FAILED, failed, now);

    fsh_est_t s;
    memset(&s, 0, sizeof s);
    fsh_est_read(&s);
    if (s.running && (s_sensors & PV2_S_MAG_FC) && s.mag_interf_pct > 30.0f &&
        (!s_have_ev_mag || (now - s_t_ev_mag) > EV_RATE_MS)) {
        send_event(PV2_EV_WARN, PV2_E_MAG_INTERFERENCE, (int32_t)(s.mag_interf_pct + 0.5f), now);
        s_t_ev_mag = now; s_have_ev_mag = 1u;
    }
    if (armed_now() && s.vib_mg > VIB_WARN_MG && (!s_have_ev_vib || (now - s_t_ev_vib) > EV_RATE_MS)) {
        send_event(PV2_EV_WARN, PV2_E_HIGH_VIBRATION, (int32_t)s.vib_mg, now);
        s_t_ev_vib = now; s_have_ev_vib = 1u;
    }
}

static void calibration(uint32_t now)
{
    cal_sample_t smp;
    while (fsh_cal_pop(&smp)) cal_feed(&smp, now);
    cal_tick(now);
    /* lost app link: abort, the previous values are kept (§10) */
    if (cal_active() && appl_silence_ms(now, FC_FS_SHORT_MS + 1u) > CAL_LINK_ABORT_MS) {
        uint8_t why;
        (void)cal_abort(&why);
    }
    g_fsh_cal_want = cal_wants_samples();
    if (cal_take_changed()) { send_calib(); s_t_calib = now + 100u; }
    else if (cal_active() && due(now, &s_t_calib, 100u)) send_calib();
}

static void streams(uint32_t now, uint8_t armed)
{
    if (tlm_ble_state() != TLM_BLE_STREAMING) return;
    if (due(now, &s_t_flight, 1000u / s_hz_flight)) send_flight();
    if (due(now, &s_t_nav,    1000u / s_hz_nav))    send_nav();
    if (due(now, &s_t_motors, 1000u / s_hz_motors)) send_motors();
    if (due(now, &s_t_system, 1000u / s_hz_system)) send_system();
    if (due(now, &s_t_checks, armed ? 2000u : 500u)) send_checks();
    if (due(now, &s_t_caps, 5000u)) send_caps();
    if (s_cfg.batt_monitor && due(now, &s_t_power, 1000u / s_hz_power)) send_power();
}

/* ------------------------------------------------------------------ entry points */
const cal_set_t *app_task_boot(const float mag_hi_default[3])
{
    memset(&s_img, 0, sizeof s_img);
    cal_set_defaults(&s_img.cal, mag_hi_default);
    cal_set_defaults(&s_img.bak, mag_hi_default);
    params_defaults();
    memcpy(s_img.name, "ACwing #1", 9);
    cfg_image_t img;
    s_img.batt_used_scale_i = 1.0f;
    if (cfg_load(&img) == 0) {
        s_img = img;
        float v[PARAM_COUNT];
        cfg_params_get(&img, v);
        params_load(v, PARAM_COUNT);
        s_img.name[15] = '\0';
    }
    params_mark_saved();
    s_cal = s_img.cal;
    fsh_reset();
    /* the ImuTask takes these before starting the estimator */
    (void)fsh_apply_post(PV2_CAL_GYRO, &s_cal);
    (void)fsh_apply_post(PV2_CAL_ACCEL, &s_cal);
    (void)fsh_apply_post(PV2_CAL_COMPASS, &s_cal);
    (void)fsh_apply_post(PV2_CAL_LEVEL, &s_cal);
    return &s_cal;
}

void app_init(const app_task_cfg_t *cfg)
{
    s_cfg = *cfg;
    if (s_cfg.n_motors == 0u) s_cfg.n_motors = 4u;
    s_sensors = sensors_now();
    fc_set_sensors(s_sensors);
    ackc_reset();
    const cal_config_t cc = { s_sensors, s_cfg.motors_built, cal_apply,
                              s_cfg.batt_monitor ? cal_batt : NULL };
    cal_init(&cc, &s_cal);
    s_pend = 0u; s_list_next = -1;
    memset(s_rep, 0, sizeof s_rep);
    s_fl_count = 0u;
}

void app_step(uint32_t now)
{
    /* sensors found later (expansion board plugged in while disarmed) */
    const uint16_t sn = sensors_now();
    if (sn != s_sensors && !armed_now()) {
        s_sensors = sn;
        fc_set_sensors(sn);
        cal_set_sensors(sn);
        if (tlm_ble_state() == TLM_BLE_STREAMING) send_caps();
    }

    /* new subscriber: CAPS, INIT, CHECKS first, then the periodic streams (§2) */
    if (tlm_ble_take_subscribe()) {
        ackc_reset();
        s_list_next = -1;
        send_caps();
        send_init();
        send_checks();
        tlm_ble_open_streams();
        s_t_flight = s_t_nav = s_t_motors = s_t_system = s_t_power = now;
        s_t_checks = now + 500u; s_t_caps = now + 5000u;
    }

    commands(now);
    events(now);
    repeats(now);
    calibration(now);

    /* ground information for ARM (10 Hz) */
    if (due(now, &s_t_ground, 100u)) {
        pv2_check_t it[PV2_CHECKS_MAX];
        uint8_t n;
        build_checks(it, &n);
        fc_set_ground_info(s_ready, s_batt_fail, cal_running());
    }

    battery_flight();
    streams(now, armed_now());
    list_pace();
}

/* ------------------------------------------------------------------ RTOS shell */
#ifndef HOST_TEST
#include "cmsis_os2.h"

static void AppTask(void *arg)
{
    (void)arg;
    for (;;) {
        app_step(osKernelGetTickCount());
        osDelay(5);
    }
}

void app_task_start(const app_task_cfg_t *cfg)
{
    app_init(cfg);
    static const osThreadAttr_t a = { .name = "app", .priority = osPriorityBelowNormal,
                                      .stack_size = 3072 };
    (void)osThreadNew(AppTask, NULL, &a);
}
#endif
