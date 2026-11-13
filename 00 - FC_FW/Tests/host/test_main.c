/**
 * Host tests of the protocol-v2 firmware: flight core (arming, TAKEOFF / LAND,
 * app-link failsafe), vertical estimator, ToF/flow drivers, AppTask protocol
 * (subscribe order, ACK cache, parameters, flash), BLE ring priorities.
 * Every frame the drone sends is also written to frames.bin for the
 * cross-check with the app decoders (decode_frames.test.ts).
 */
#include "proto_v2.h"
#include "telemetry.h"
#include "tlm_ble.h"
#include "app_link.h"
#include "ack_cache.h"
#include "fc_core.h"
#include "params.h"
#include "cfg_store.h"
#include "fc_shared.h"
#include "app_task.h"
#include "alt_est.h"
#include "alt_ctrl.h"
#include "vl53l1x.h"
#include "flow_bridge.h"
#include "checks.h"
#include "control_hooks.h"
#include "mixer.h"
#include "rc_input.h"
#include "rc_crsf.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

extern int      g_flash_fail, g_flash_writes;
extern uint16_t g_ext_sensors;
void host_flash_erase(void);

static int s_fail, s_pass;
#define CHECK(c, ...) do { if (c) s_pass++; else { s_fail++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static uint32_t now;
static FILE    *s_bin;

/* ------------------------------------------------------------ frames out */
typedef struct { uint8_t type, len; uint8_t p[255]; } frm_t;
static frm_t   s_f[4096];
static int     s_nf;

static void drain(void)
{
    static uint8_t acc[8192];
    static size_t  na;
    uint8_t buf[244];
    for (;;) {
        const uint16_t n = tlm_ble_peek(buf, sizeof buf);
        if (!n) break;
        tlm_ble_consume(n);
        memcpy(acc + na, buf, n); na += n;
        if (s_bin) fwrite(buf, 1, n, s_bin);
    }
    size_t i = 0;
    while (na - i >= 6) {
        if (acc[i] != 0xA5 || acc[i + 1] != 0x5A) { i++; continue; }
        const uint8_t len = acc[i + 3];
        if (na - i < (size_t)len + 6u) break;
        if (s_nf < (int)(sizeof s_f / sizeof s_f[0])) {
            s_f[s_nf].type = acc[i + 2]; s_f[s_nf].len = len;
            memcpy(s_f[s_nf].p, &acc[i + 4], len); s_nf++;
        }
        i += len + 6u;
    }
    memmove(acc, acc + i, na - i); na -= i;
}

static int count_type(int from, uint8_t type)
{
    int n = 0;
    for (int i = from; i < s_nf; ++i) n += (s_f[i].type == type);
    return n;
}

static const pv2_ack_t *find_ack(int from, uint8_t op, uint8_t seq, int *count)
{
    const pv2_ack_t *a = NULL;
    int n = 0;
    for (int i = from; i < s_nf; ++i)
        if (s_f[i].type == PV2_FT_ACK) {
            const pv2_ack_t *x = (const pv2_ack_t *)s_f[i].p;
            if (x->op == op && x->seq == seq) { if (!a) a = x; n++; }
        }
    if (count) *count = n;
    return a;
}

static int find_event(int from, uint16_t code)
{
    int n = 0;
    for (int i = from; i < s_nf; ++i)
        if (s_f[i].type == PV2_FT_EVENT) {
            pv2_event_hdr_t h; memcpy(&h, s_f[i].p, sizeof h);
            if (h.code == code) n++;
        }
    return n;
}

/* ------------------------------------------------------------ commands in */
static void cmd(uint8_t op, uint8_t seq, const void *p, uint8_t n)
{
    uint8_t b[20];
    b[0] = op; b[1] = seq;
    if (n) memcpy(b + 2, p, n);
    appl_on_cmd_write(b, (uint16_t)(2 + n), now);
}
static void heartbeat(void) { cmd(PV2_OP_HEARTBEAT, 0, NULL, 0); }

/* ------------------------------------------------------------ vertical plant */
typedef struct { float h, vz, hover, a; } plant_t;
static void plant_step(plant_t *pl, float thrust, uint8_t armed, float dt)
{
    float a = armed ? 9.81f * (thrust / pl->hover - 1.0f) - 0.6f * pl->vz : -9.81f;
    pl->vz += a * dt;
    pl->h  += pl->vz * dt;
    if (pl->h <= 0.0f) { pl->h = 0.0f; if (pl->vz < 0.0f) pl->vz = 0.0f; a = 0.0f; }   /* resting: no acceleration */
    pl->a = a;                          /* what the accelerometer gives (vertical, gravity removed) */
}

/* ============================================================ tests */
static void test_ack_cache(void)
{
    ackc_reset();
    uint8_t r, w;
    CHECK(ackc_begin(0x12, 7, &r, &w) == ACKC_NEW, "new");
    CHECK(ackc_begin(0x12, 7, &r, &w) == ACKC_PENDING, "pending");
    ackc_finish(0x12, 7, 1, 8);
    CHECK(ackc_begin(0x12, 7, &r, &w) == ACKC_DONE && r == 1 && w == 8, "done cached");
    for (int i = 0; i < 31; ++i) { (void)ackc_begin(0x20, (uint8_t)i, &r, &w); ackc_finish(0x20, (uint8_t)i, 0, 0); }
    CHECK(ackc_begin(0x12, 7, &r, &w) == ACKC_DONE, "kept with 32 entries (>= 16 required)");
}

static void fc_setup(uint16_t sensors)
{
    appl_reset();
    const fc_config_t c = { sensors, 4, 0, NULL, NULL, NULL, 0, NULL };
    fc_init(&c);
    params_defaults();
    { uint8_t why; (void)params_set(80, 0.42f, &why); }   /* hoverThr of the test vehicle (plant hover 0.40-0.42), as the app sets it */
    fc_set_ground_info(1, 0, 0);
}

static fc_res_t post(uint8_t op, uint8_t seq, const void *p, uint8_t n, const fc_vert_t *v)
{
    fc_cmd_t c; memset(&c, 0, sizeof c);
    c.op = op; c.seq = seq; c.len = n; if (n) memcpy(c.p, p, n);
    (void)fc_post(&c);
    fc_service(now, 1, v);
    fc_res_t r; memset(&r, 0xEE, sizeof r);
    (void)fc_take_result(&r);
    return r;
}

static int drain_events(uint16_t code)
{
    fc_event_t e; int n = 0;
    while (fc_take_event(&e)) n += (e.code == code);
    return n;
}

static void test_takeoff_land(void)
{
    fc_setup(PV2_S_IMU | PV2_S_BARO);
    now = 1000;
    heartbeat();
    plant_t pl = { 0, 0, 0.42f, 0 };
    fc_vert_t v = { 1, 0, 0, 0, 0, 0, 1.0f, 0xFF, 0, 0 };
    fc_service(now, 1, &v);
    const uint8_t arm[2] = { 0xA5, 0x5A };
    fc_res_t r = post(PV2_OP_ARM, 1, arm, 2, &v);
    CHECK(r.result == PV2_ACK_OK, "arm %d/%d", r.result, r.reason);
    const uint16_t alt = 100;
    r = post(PV2_OP_TAKEOFF, 2, &alt, 2, &v);
    CHECK(r.result == PV2_ACK_OK, "takeoff %d/%d", r.result, r.reason);
    CHECK(drain_events(PV2_E_TAKEOFF_STARTED) == 1, "event 3");

    fc_state_t s;
    float maxh = 0; uint32_t t_done = 0; float vmax = 0;
    for (int k = 0; k < 15000 / 4; ++k) {
        now += 4;
        if (k % 25 == 0) heartbeat();
        fc_setpoint_t sp; fc_setpoint(&sp);
        plant_step(&pl, sp.thrust, sp.armed, 0.004f);
        v.h = pl.h; v.vz = pl.vz; v.a_up = pl.a;
        fc_service(now, 1, &v);
        fc_snapshot(&s);
        if (pl.h > maxh) maxh = pl.h;
        if (pl.vz > vmax) vmax = pl.vz;
        if (!t_done && s.autom == PV2_AUTO_NONE) t_done = now;
    }
    CHECK(t_done != 0, "takeoff completed");
    CHECK(s.mode == PV2_MODE_ALT_HOLD, "ALT_HOLD after takeoff without GNSS/FLOW (mode %d)", s.mode);
    CHECK(fabsf(pl.h - 1.0f) < 0.1f, "holds 1 m (h %.2f)", pl.h);
    CHECK(maxh < 1.25f, "overshoot (max %.2f)", maxh);
    CHECK(vmax < 0.75f, "climb rate <= 0.6 m/s (+margin), max %.2f", vmax);
    CHECK(s.in_air && s.alt_rel_cm > 85 && s.alt_rel_cm < 115, "FLIGHT altRel %d", (int)s.alt_rel_cm);

    r = post(PV2_OP_LAND, 3, NULL, 0, &v);
    CHECK(r.result == PV2_ACK_OK, "land");
    uint32_t t_land = 0, t_disarm = 0;
    int landed = 0, disarmed = 0;
    for (int k = 0; k < 15000 / 4; ++k) {
        now += 4;
        if (k % 25 == 0) heartbeat();
        fc_setpoint_t sp; fc_setpoint(&sp);
        plant_step(&pl, sp.thrust, sp.armed, 0.004f);
        v.h = pl.h; v.vz = pl.vz; v.a_up = pl.a;
        fc_service(now, 1, &v);
        fc_event_t e;
        while (fc_take_event(&e)) {
            if (e.code == PV2_E_LANDED)   { landed++;   t_land = now; }
            if (e.code == PV2_E_DISARMED) { disarmed++; t_disarm = now; }
        }
    }
    fc_snapshot(&s);
    CHECK(landed == 1, "LANDED once");
    CHECK(disarmed == 1 && !s.armed, "auto-disarm");
    CHECK(t_disarm - t_land >= 2000 && t_disarm - t_land < 2100, "autoDisarm 2 s (%u ms)", (unsigned)(t_disarm - t_land));
    CHECK(s.landed, "FLIGHT LANDED flag");
}

/* indoor model: baro + ToF + flow -> POS_HOLD after takeoff, slow landing below 1 m */
static void test_takeoff_tof(void)
{
    fc_setup(PV2_S_IMU | PV2_S_BARO | PV2_S_TOF | PV2_S_FLOW);
    now = 1000; heartbeat();
    plant_t pl = { 0, 0, 0.40f, 0 };
    fc_vert_t v = { 1, 0, 0, 0, 1, 0, 1.0f, 80, 0, 0 };
    const uint8_t arm[2] = { 0xA5, 0x5A };
    fc_res_t r = post(PV2_OP_ARM, 1, arm, 2, &v);
    const uint16_t alt = 150;
    r = post(PV2_OP_TAKEOFF, 2, &alt, 2, &v);
    CHECK(r.result == PV2_ACK_OK, "takeoff indoor");
    fc_state_t s;
    for (int k = 0; k < 12000 / 4; ++k) {
        now += 4; if (k % 25 == 0) heartbeat();
        fc_setpoint_t sp; fc_setpoint(&sp);
        plant_step(&pl, sp.thrust, sp.armed, 0.004f);
        v.h = pl.h; v.vz = pl.vz; v.a_up = pl.a;
        v.tof_h = pl.h; v.tof_ground = pl.h < 0.03f; v.tof_valid = !v.tof_ground; v.flow_usable = pl.h >= 0.08f;
        fc_service(now, 1, &v);
    }
    fc_snapshot(&s);
    {
        fc_setpoint_t sp; fc_setpoint(&sp);
        CHECK(sp.pos_hold, "position hold on flow at 1.5 m");
        CHECK(drain_events(PV2_E_FLOW_LOST) == 0, "no FLOW_LOST during the takeoff from the floor");
    }
    CHECK(s.mode == PV2_MODE_POS_HOLD && s.autom == PV2_AUTO_NONE, "POS_HOLD after takeoff with FLOW (mode %d)", s.mode);
    CHECK(fabsf(pl.h - 1.5f) < 0.1f, "holds 1.5 m (%.2f)", pl.h);
    r = post(PV2_OP_LAND, 3, NULL, 0, &v);
    float vmin_low = 0, vmin_high = 0;
    int landed = 0, flow_lost_land = 0, pos_hold_low = 0;
    for (int k = 0; k < 15000 / 4; ++k) {
        now += 4; if (k % 25 == 0) heartbeat();
        fc_setpoint_t sp; fc_setpoint(&sp);
        plant_step(&pl, sp.thrust, sp.armed, 0.004f);
        v.h = pl.h; v.vz = pl.vz; v.a_up = pl.a;
        v.tof_h = pl.h; v.tof_ground = pl.h < 0.03f; v.tof_valid = !v.tof_ground; v.flow_usable = pl.h >= 0.08f;
        fc_service(now, 1, &v);
        if (pl.h < 0.8f && pl.h > 0.05f && pl.vz < vmin_low) vmin_low = pl.vz;
        if (pl.h > 1.1f && pl.vz < vmin_high) vmin_high = pl.vz;
        fc_event_t e; while (fc_take_event(&e)) { landed += (e.code == PV2_E_LANDED); flow_lost_land += (e.code == PV2_E_FLOW_LOST); }
        if (pl.h < 0.07f && pl.h > 0.0f && sp.pos_hold) pos_hold_low++;
    }
    CHECK(flow_lost_land == 0, "no FLOW_LOST while landing below 80 mm");
    CHECK(pos_hold_low == 0, "no position hold on flow below 80 mm");
    CHECK(vmin_high < -0.5f, "landSpeed 0.7 m/s above 1 m (%.2f)", vmin_high);
    CHECK(vmin_low > -0.4f, "0.3 m/s below 1 m with ToF (%.2f)", vmin_low);
    CHECK(landed == 1, "landed indoor");

    /* flow lost in POS_HOLD: ALT_HOLD + EVENT 25 (fsFlowLost = 0) */
    fc_setup(PV2_S_IMU | PV2_S_BARO | PV2_S_TOF | PV2_S_FLOW);
    now = 1000; heartbeat();
    pl.h = 0; pl.vz = 0;
    v.flow_q = 80; v.h = 0; v.vz = 0; v.tof_ground = 1; v.tof_valid = 0;
    (void)post(PV2_OP_ARM, 4, arm, 2, &v);
    (void)post(PV2_OP_TAKEOFF, 5, NULL, 0, &v);
    int lost = 0;
    for (int k = 0; k < 12000 / 4; ++k) {
        now += 4; if (k % 25 == 0) heartbeat();
        fc_setpoint_t sp; fc_setpoint(&sp);
        plant_step(&pl, sp.thrust, sp.armed, 0.004f);
        v.h = pl.h; v.vz = pl.vz; v.a_up = pl.a; v.tof_h = pl.h; v.tof_ground = pl.h < 0.03f; v.tof_valid = !v.tof_ground; v.flow_usable = pl.h >= 0.08f; v.flow_usable = pl.h >= 0.08f;
        if (k > 8000 / 4) v.flow_q = 10;
        fc_service(now, 1, &v);
        fc_event_t e; while (fc_take_event(&e)) lost += (e.code == PV2_E_FLOW_LOST);
    }
    fc_snapshot(&s);
    CHECK(lost == 1 && s.mode == PV2_MODE_ALT_HOLD && (s.failsafe & PV2_FS_FLOW), "flow lost -> ALT_HOLD");

    /* ceiling (gfCeiling 1 m indoor): takeoff to 1.5 m stops below 1 m, EVENT 23 arg 1 */
    fc_setup(PV2_S_IMU | PV2_S_BARO | PV2_S_TOF);
    { uint8_t why; (void)params_set(31, 1.0f, &why); }
    now = 1000; heartbeat();
    pl.h = 0; pl.vz = 0;
    v.flow_q = 0xFF; v.h = 0; v.vz = 0; v.tof_ground = 0; v.tof_valid = 1; v.tof_h = 0;
    (void)post(PV2_OP_ARM, 6, arm, 2, &v);
    (void)post(PV2_OP_TAKEOFF, 7, &alt, 2, &v);
    float hmax = 0;
    for (int k = 0; k < 12000 / 4; ++k) {
        now += 4; if (k % 25 == 0) heartbeat();
        fc_setpoint_t sp; fc_setpoint(&sp);
        plant_step(&pl, sp.thrust, sp.armed, 0.004f);
        v.h = pl.h; v.vz = pl.vz; v.a_up = pl.a; v.tof_h = pl.h;
        fc_service(now, 1, &v);
        if (pl.h > hmax) hmax = pl.h;
        fc_event_t e; while (fc_take_event(&e)) {}
    }
    CHECK(hmax < 1.05f, "below the ceiling (max %.2f)", hmax);
    params_defaults();
}

static void test_takeoff_rules(void)
{
    /* no altitude sensor: UNSUPPORTED / SENSOR_MISSING (bench without baro) */
    fc_setup(PV2_S_IMU);
    now = 1000; heartbeat();
    fc_vert_t v = { 0 };
    const uint8_t arm[2] = { 0xA5, 0x5A };
    fc_res_t r = post(PV2_OP_ARM, 1, arm, 2, NULL);
    r = post(PV2_OP_TAKEOFF, 2, NULL, 0, NULL);
    CHECK(r.result == PV2_ACK_UNSUPPORTED && r.reason == PV2_R_SENSOR_MISSING, "takeoff without baro %d/%d", r.result, r.reason);

    /* baro present, not armed: NOT_ARMED */
    fc_setup(PV2_S_IMU | PV2_S_BARO);
    now = 1000; heartbeat();
    v.valid = 1; v.cos_tilt = 1; v.flow_q = 0xFF;
    r = post(PV2_OP_TAKEOFF, 3, NULL, 0, &v);
    CHECK(r.result == PV2_ACK_REJECTED && r.reason == PV2_R_NOT_ARMED, "takeoff not armed");

    /* no lift-off (motors not built / props off on the bench): aborted after 6 s */
    r = post(PV2_OP_ARM, 4, arm, 2, &v);
    r = post(PV2_OP_TAKEOFF, 5, NULL, 0, &v);
    CHECK(r.result == PV2_ACK_OK, "takeoff accepted");
    fc_state_t s;
    fc_snapshot(&s);
    CHECK(s.in_air && s.autom == PV2_AUTO_TAKEOFF, "in air during takeoff (as the simulator)");
    int aborted = 0;
    for (int k = 0; k < 7000 / 4; ++k) {
        now += 4; if (k % 25 == 0) heartbeat();
        fc_service(now, 1, &v);
        fc_event_t e; while (fc_take_event(&e)) aborted += (e.code == PV2_E_TAKEOFF_ABORTED);
    }
    fc_snapshot(&s);
    CHECK(aborted == 1 && !s.in_air && s.autom == PV2_AUTO_NONE && s.armed, "takeoff aborted, still armed on the ground");
    fc_setpoint_t sp; fc_setpoint(&sp);
    CHECK(sp.thrust < 0.01f, "idle thrust after abort in ANGLE (%.2f)", sp.thrust);
    /* the no-takeoff auto-disarm restarts at the abort */
    for (int k = 0; k < 11000 / 4; ++k) { now += 4; if (k % 25 == 0) heartbeat(); fc_service(now, 1, &v); }
    fc_snapshot(&s);
    CHECK(!s.armed, "auto-disarm 10 s after the abort");

    /* a repeated TAKEOFF while in the air: NOT_LANDED */
    fc_setup(PV2_S_IMU | PV2_S_BARO);
    now = 1000; heartbeat();
    r = post(PV2_OP_ARM, 6, arm, 2, &v);
    r = post(PV2_OP_TAKEOFF, 7, NULL, 0, &v);
    r = post(PV2_OP_TAKEOFF, 8, NULL, 0, &v);
    CHECK(r.result == PV2_ACK_REJECTED && r.reason == PV2_R_NOT_LANDED, "second takeoff");
    /* ceiling: target clamped below gfMaxAlt */
    {
        uint8_t why;
        (void)params_set(31, 1.0f, &why);           /* gfCeiling, ToF models only: no effect here */
    }
}

static void test_app_link_failsafe(void)
{
    /* in the air (takeoff to 1 m), then the app goes silent: BLE stalled or app closed */
    fc_setup(PV2_S_IMU | PV2_S_BARO);
    now = 1000; heartbeat();
    plant_t pl = { 0, 0, 0.42f, 0 };
    fc_vert_t v = { 1, 0, 0, 0, 0, 0, 1.0f, 0xFF, 0, 0 };
    const uint8_t arm[2] = { 0xA5, 0x5A };
    (void)post(PV2_OP_ARM, 1, arm, 2, &v);
    (void)post(PV2_OP_TAKEOFF, 2, NULL, 0, &v);
    for (int k = 0; k < 8000 / 4; ++k) {
        now += 4; if (k % 25 == 0) heartbeat();
        fc_setpoint_t sp; fc_setpoint(&sp);
        plant_step(&pl, sp.thrust, sp.armed, 0.004f);
        v.h = pl.h; v.vz = pl.vz; v.a_up = pl.a;
        fc_service(now, 1, &v);
    }
    (void)drain_events(0);
    const uint32_t t0 = appl_last_ms();
    uint32_t t_short = 0, t_long = 0, t_land = 0;
    fc_state_t s;
    for (int k = 0; k < 20000 / 4; ++k) {
        now += 4;
        fc_setpoint_t sp; fc_setpoint(&sp);
        plant_step(&pl, sp.thrust, sp.armed, 0.004f);
        v.h = pl.h; v.vz = pl.vz; v.a_up = pl.a;
        fc_service(now, 1, &v);
        fc_snapshot(&s);
        if (!t_short && (s.failsafe & PV2_FS_APP_LINK)) t_short = now;
        fc_event_t e;
        while (fc_take_event(&e)) {
            if (e.code == PV2_E_FAILSAFE_APP_LINK) { t_long = now; CHECK(e.level == PV2_EV_ERROR, "event 20 ERROR"); }
            if (e.code == PV2_E_DISARMED) t_land = now;
        }
    }
    CHECK(t_short - t0 > FC_FS_SHORT_MS && t_short - t0 <= FC_FS_SHORT_MS + 8, "short failsafe at 1 s (%u)", (unsigned)(t_short - t0));
    CHECK(t_long - t0 > FC_FS_LONG_MS && t_long - t0 <= FC_FS_LONG_MS + 8, "long failsafe at 3 s (%u)", (unsigned)(t_long - t0));
    CHECK(t_land && !s.armed && pl.h < 0.01f, "lands by itself and disarms (h %.2f)", pl.h);

    /* GAP disconnect: the failsafe starts earlier, never later */
    fc_setup(PV2_S_IMU);
    now = 50000; heartbeat();
    g_fc_dbg_in_air = 1;
    (void)post(PV2_OP_ARM, 1, arm, 2, NULL);
    fc_service(now, 1, NULL);
    appl_on_disconnect();
    now += 8; fc_service(now, 1, NULL);
    fc_snapshot(&s);
    CHECK(s.failsafe & PV2_FS_APP_LINK, "disconnect -> short failsafe at once");
    g_fc_dbg_in_air = 0;
}


/* ------------------------------------------------------------ manual collective (HANDOVER_FW §6, §7) */
static void rc(int16_t thr, int16_t yaw, int16_t pitch, int16_t roll)
{
    uint8_t b[9];
    memcpy(&b[0], &thr, 2); memcpy(&b[2], &yaw, 2); memcpy(&b[4], &pitch, 2); memcpy(&b[6], &roll, 2); b[8] = 0;
    cmd(PV2_OP_RC, 0, b, sizeof b);
}

/* runs the core with the plant for ms; thr != INT16_MIN: RC at 25 Hz, else heartbeats only */
static void run_plant(plant_t *pl, fc_vert_t *v, uint32_t ms, int16_t thr)
{
    for (uint32_t k = 0; k < ms / 4; ++k) {
        now += 4;
        if (k % 10 == 0) { if (thr != INT16_MIN) rc(thr, 0, 0, 0); else heartbeat(); }
        fc_setpoint_t sp; fc_setpoint(&sp);
        if (pl) { plant_step(pl, sp.thrust, sp.armed, 0.004f); v->h = pl->h; v->vz = pl->vz; v->a_up = pl->a; }
        fc_service(now, 1, v);
    }
}

static void test_manual_collective(void)
{
    const uint8_t arm[2] = { 0xA5, 0x5A };
    fc_state_t s; fc_setpoint_t sp; fc_res_t r;

    /* 1. stale RC on the ground: a stick left up never lifts the drone off */
    fc_setup(PV2_S_IMU | PV2_S_BARO);
    now = 1000;
    plant_t pl = { 0, 0, 0.42f, 0 };
    fc_vert_t v = { 1, 0, 0, 0, 0, 0, 1.0f, 0xFF, 0, 0 };
    run_plant(&pl, &v, 200, 800);                       /* cockpit closed with the throttle high */
    run_plant(&pl, &v, 600, INT16_MIN);                 /* RC older than 0.5 s: slide-to-arm allowed */
    r = post(PV2_OP_ARM, 1, arm, 2, &v);
    CHECK(r.result == PV2_ACK_OK, "slide-to-arm with an old RC %d/%d", r.result, r.reason);
    run_plant(&pl, &v, 2000, INT16_MIN);
    fc_setpoint(&sp); fc_snapshot(&s);
    CHECK(sp.thrust < 0.01f && !s.in_air && pl.h < 0.01f, "stale RC on the ground: idle (thrust %.2f)", sp.thrust);

    /* 6. takeoff in ALT_HOLD learns the hover; ANGLE with the stick at the centre holds it */
    run_plant(&pl, &v, 100, 0);
    r = post(PV2_OP_TAKEOFF, 2, NULL, 0, &v);
    CHECK(r.result == PV2_ACK_OK, "takeoff");
    run_plant(&pl, &v, 12000, 0);
    fc_snapshot(&s);
    CHECK(fabsf(s.hover - 0.42f) < 0.03f, "hover learned in ALT_HOLD (%.3f, plant 0.42)", s.hover);
    fc_setpoint(&sp);
    const float thr_hold = sp.thrust;
    uint8_t m = PV2_MODE_ANGLE;
    r = post(PV2_OP_SET_MODE, 3, &m, 1, &v);
    fc_setpoint(&sp);
    CHECK(r.result == PV2_ACK_OK && fabsf(sp.thrust - thr_hold) < 0.03f, "ALT_HOLD -> ANGLE: no thrust jump (%.3f -> %.3f)", thr_hold, sp.thrust);
    const float h0 = pl.h;
    run_plant(&pl, &v, 3000, 0);
    CHECK(fabsf(pl.h - h0) < 0.4f, "ANGLE, centre stick = hover (%.2f -> %.2f m)", h0, pl.h);
    m = PV2_MODE_ALT_HOLD;
    (void)post(PV2_OP_SET_MODE, 4, &m, 1, &v);
    fc_setpoint(&sp);
    CHECK(fabsf(sp.thrust - s.hover) < 0.05f, "ANGLE -> ALT_HOLD: starts from the hover (%.3f)", sp.thrust);
    m = PV2_MODE_ANGLE;
    (void)post(PV2_OP_SET_MODE, 5, &m, 1, &v);

    /* 2. HOLD in ANGLE with the throttle off the centre: not cancelled; a 30 % change cancels it */
    run_plant(&pl, &v, 500, 200);
    (void)post(PV2_OP_HOLD, 6, NULL, 0, &v);
    run_plant(&pl, &v, 1000, 200);
    fc_snapshot(&s);
    CHECK(s.autom == PV2_AUTO_HOLD, "HOLD kept with the collective stick at +20 %% (auto %d)", s.autom);
    run_plant(&pl, &v, 200, 600);
    fc_snapshot(&s);
    CHECK(s.autom == PV2_AUTO_NONE, "HOLD cancelled by +40 %% throttle (auto %d)", s.autom);

    /* 5. geofence in ANGLE: full collective does not climb above gfMaxAlt */
    { uint8_t why; (void)params_set(30, 5.0f, &why); }
    (void)drain_events(0);
    float hmax = 0;
    for (int k = 0; k < 40; ++k) { run_plant(&pl, &v, 250, 1000); if (pl.h > hmax) hmax = pl.h; }
    CHECK(hmax < 5.6f, "ANGLE stays below the 5 m ceiling (max %.2f)", hmax);
    CHECK(drain_events(PV2_E_GEOFENCE_BREACH) == 1, "EVENT 23 once");
    { uint8_t why; (void)params_set(30, 30.0f, &why); }

    /* 3. LAND in ANGLE with the stick at the centre: idle after the touchdown, no second lift-off */
    run_plant(&pl, &v, 3000, 0);
    (void)post(PV2_OP_LAND, 7, NULL, 0, &v);
    uint32_t t_td = 0; float h_after = 0;
    for (int k = 0; k < 2000 && s.armed; ++k) {
        run_plant(&pl, &v, 8, 0);
        fc_snapshot(&s);
        if (!t_td && s.landed) t_td = now;
        if (t_td) { fc_setpoint(&sp); if (s.armed) CHECK(sp.thrust < 0.01f, "idle after touchdown (%.2f)", sp.thrust); if (pl.h > h_after) h_after = pl.h; }
    }
    CHECK(t_td && !s.armed && h_after < 0.05f, "landed, stayed down, auto-disarmed (h %.2f)", h_after);

    /* 3b. a manual landing (stick at the bottom) releases the idle: a new take-off is possible */
    fc_setup(PV2_S_IMU | PV2_S_BARO);
    now = 100000; pl = (plant_t){ 0, 0, 0.42f, 0 }; v.h = 0; v.vz = 0;
    run_plant(&pl, &v, 100, -1000);
    (void)post(PV2_OP_ARM, 1, arm, 2, &v);
    run_plant(&pl, &v, 1500, 300);
    fc_snapshot(&s);
    CHECK(s.in_air && pl.h > 0.3f, "ANGLE manual lift-off (h %.2f)", pl.h);
    run_plant(&pl, &v, 4000, -1000);
    fc_snapshot(&s);
    CHECK(!s.in_air && s.landed && !s.land_idle, "manual touchdown, stick at the bottom: idle released");
    run_plant(&pl, &v, 800, 300);
    fc_snapshot(&s);
    CHECK(s.in_air, "re-take-off before the auto-disarm");
}

/* 4. no altitude estimate: in air from the collective, failsafe LAND open loop */
static void test_no_alt_estimate(void)
{
    const uint8_t arm[2] = { 0xA5, 0x5A };
    fc_state_t s; fc_setpoint_t sp;
    fc_setup(PV2_S_IMU);
    now = 1000;
    run_plant(NULL, NULL, 100, -1000);
    fc_res_t r = post(PV2_OP_ARM, 1, arm, 2, NULL);
    CHECK(r.result == PV2_ACK_OK, "arm without baro");
    for (uint32_t k = 0; k < 12000 / 4; ++k) { now += 4; if (k % 10 == 0) rc(200, 0, 0, 0); fc_service(now, 1, NULL); }
    fc_snapshot(&s);
    CHECK(s.armed && s.in_air && s.air_by_thrust, "in air from the collective, no 10 s auto-disarm in flight");

    /* app silent: HOLD (open-loop hover), then the long action LAND (RTH without GNSS) */
    float thr_hold = -1, thr_land_end = -1; uint32_t t_dis = 0;
    const uint32_t t0 = now;
    for (uint32_t k = 0; k < 30000 / 4; ++k) {
        now += 4; fc_service(now, 1, NULL);
        fc_snapshot(&s); fc_setpoint(&sp);
        if (now - t0 == 2000) thr_hold = sp.thrust;
        if (now - t0 == 3000 + 5000) thr_land_end = sp.thrust;
        if (!s.armed && !t_dis) t_dis = now;
    }
    /* no altitude estimate: no update, the hover in use is the seed, parameter hoverThr (0.42 here) */
    CHECK(fabsf(thr_hold - 0.42f) < 0.01f, "short failsafe: hover collective (%.3f)", thr_hold);
    CHECK(fabsf(thr_land_end - FC_LAND_OL_K * 0.42f) < 0.01f, "open-loop LAND at 0.85 x hover (%.3f)", thr_land_end);
    /* touchdown FC_LAND_OL_MS after the start of LAND; on the ground with the app silent (> 3 s) the disarm is immediate */
    CHECK(t_dis && t_dis - t0 > 2900 + FC_LAND_OL_MS && t_dis - t0 < 3000 + FC_LAND_OL_MS + 2100, "touchdown after the open-loop LAND, disarm (%u ms)", (unsigned)(t_dis - t0));

    /* manual touchdown without an estimate: stick at the bottom for 1 s */
    fc_setup(PV2_S_IMU);
    now = 100000;
    run_plant(NULL, NULL, 100, -1000);
    (void)post(PV2_OP_ARM, 1, arm, 2, NULL);
    run_plant(NULL, NULL, 1000, 200);
    run_plant(NULL, NULL, 1200, -1000);
    fc_snapshot(&s);
    CHECK(!s.in_air && s.landed, "manual touchdown without an altitude estimate");
}


/* ------------------------------------------------------------ phase 3 glue: ctrl_loop.c with the flight core */
static void ctrl_steps(int n960, const float q[4], const float gyro[3])
{
    for (int k = 0; k < n960; ++k) {
        if (k % 4 == 0) { ctrl_att_in_t a; memcpy(a.q, q, sizeof a.q); a.dt = 1.0f / 240.0f; a.step = (uint32_t)k; ctrl_attitude_step(&a); }
        ctrl_rate_in_t r; memcpy(r.gyro, gyro, sizeof r.gyro); r.dt = 1.0f / 960.0f; r.step = (uint32_t)k; ctrl_rate_step(&r);
    }
}

static void test_ctrl_loop(void)
{
    const uint8_t arm[2] = { 0xA5, 0x5A };
    const float q0[4] = { 1, 0, 0, 0 }, g0[3] = { 0, 0, 0 };
    fc_setup(PV2_S_IMU | PV2_S_BARO);
    ctrl_init();
    now = 1000;
    fc_vert_t v = { 1, 0, 0, 0, 0, 0, 1.0f, 0xFF, 0, 0 };
    run_plant(NULL, &v, 100, -1000);
    ctrl_steps(48, q0, g0);
    CHECK(ctrl_motor_pct(0) == 0 && !ctrl_service(now), "disarmed: no motor output");
    fc_res_t r = post(PV2_OP_ARM, 1, arm, 2, &v);
    CHECK(r.result == PV2_ACK_OK, "arm");
    ctrl_steps(48, q0, g0);
    const uint8_t idle = (uint8_t)(mixer_curve_inv(0.05f, 1.0f) * 100.0f + 0.5f);
    int same = 1;
    for (uint8_t i = 0; i < 4; ++i) same &= (ctrl_motor_pct(i) == idle);
    CHECK(same, "armed on the ground: every motor at idle (%u %%, expected %u %%)", ctrl_motor_pct(0), idle);
    /* tilted on the ground at idle: no torque, the motors stay equal */
    const float qt[4] = { 0.9962f, 0.0872f, 0, 0 };          /* 10 deg roll */
    ctrl_steps(480, qt, g0);
    same = 1;
    for (uint8_t i = 0; i < 4; ++i) same &= (ctrl_motor_pct(i) == idle);
    CHECK(same, "armed on a slope at idle: no differential thrust");
    /* the rate loop stops (IMU / estimator stalled) while armed: motors off after CTRL_STALL_MS */
    uint8_t st = 0;
    for (int k = 0; k < 20; ++k) { now += 4; fc_service(now, 1, &v); st = ctrl_service(now); }
    CHECK(st && ctrl_motor_pct(0) == 0, "rate loop stalled 80 ms while armed: motors off");
    ctrl_steps(4, q0, g0);
    now += 4;
    CHECK(!ctrl_service(now), "rate loop back: no longer stalled");
    /* disarm: outputs back to 0 */
    (void)post(PV2_OP_DISARM, 2, NULL, 0, &v);
    ctrl_steps(8, q0, g0);
    CHECK(ctrl_motor_pct(0) == 0, "disarmed: outputs 0");
    /* yaw stick right = clockwise = negative FLU rate */
    fc_setpoint_t sp;
    (void)post(PV2_OP_ARM, 3, arm, 2, &v);
    rc(-1000, 500, 0, 0); now += 4; fc_service(now, 1, &v);
    fc_setpoint(&sp);
    CHECK(sp.yaw_rate < 0.0f, "yaw stick right -> negative (clockwise) FLU yaw rate (%.3f)", sp.yaw_rate);
    (void)post(PV2_OP_DISARM, 4, NULL, 0, &v);
}


/* ------------------------------------------------------------ RC radio: arbitration and failsafe (input_arb.c, radio_fs.c) */
static void radio_send(int16_t thr, int16_t yaw, int16_t pitch, int16_t roll, uint16_t aux1)
{
    /* AETR: ch1 roll, ch2 pitch, ch3 throttle, ch4 yaw, ch5 AUX1; us -> CRSF 11-bit (us = v * 5/8 + 880) */
    uint16_t us[16];
    for (int c = 0; c < 16; ++c) us[c] = 1500;
    us[0] = (uint16_t)(1500 + roll / 2); us[1] = (uint16_t)(1500 + pitch / 2);
    us[2] = (uint16_t)(1000 + (thr + 1000) / 2); us[3] = (uint16_t)(1500 + yaw / 2); us[4] = aux1;
    uint8_t f[26] = { CRSF_ADDR_FC, 24, CRSF_T_RC_CHANNELS };
    memset(&f[3], 0, 22);
    uint32_t bit = 0;
    for (int c = 0; c < 16; ++c) {
        const uint16_t v = (uint16_t)(((uint32_t)(us[c] - 880u) * 8u + 4u) / 5u);
        for (int k = 0; k < 11; ++k, ++bit) if (v & (1u << k)) f[3 + bit / 8u] |= (uint8_t)(1u << (bit % 8u));
    }
    f[25] = rc_crsf_crc8(&f[2], 23);
    rc_input_feed(f, 26, now);
}

/* ms of flight core with the radio sending (aux1 0: radio silent) and the app sending heartbeats (app 0: silent) */
static void radio_run(uint32_t ms, int16_t thr, int16_t roll, uint16_t aux1, int app, const fc_vert_t *v)
{
    for (uint32_t k = 0; k < ms / 4u; ++k) {
        now += 4;
        if (aux1 && k % 2 == 0) radio_send(thr, 0, 0, roll, aux1);
        if (app && k % 25 == 0) heartbeat();
        rc_input_service(now);
        fc_service(now, 1, v);
    }
}

static int count_events(uint16_t code, int *level)
{
    fc_event_t e; int n = 0;
    while (fc_take_event(&e)) if (e.code == code) { n++; if (level) *level = e.level; }
    return n;
}

static void test_radio(void)
{
    const uint8_t arm[2] = { 0xA5, 0x5A };
    fc_state_t s; fc_setpoint_t sp; fc_res_t r;
    fc_setup(PV2_S_IMU);
    rc_input_init(); (void)rc_input_configure(RC_PROTO_CRSF, RC_MAP_AETR);
    now = 1000;
    fc_vert_t v = { 0 };
    /* 1. a radio at rest (throttle bottom, sticks centred, arm switch off): the app flies */
    radio_run(500, -1000, 0, 1000, 1, NULL);
    fc_snapshot(&s);
    CHECK(s.input == PV2_INPUT_APP, "radio at rest: the app has control (input %u)", s.input);
    r = post(PV2_OP_ARM, 1, arm, 2, NULL);
    CHECK(r.result == PV2_ACK_OK, "app ARM with an idle radio");
    (void)drain_events(0);
    /* 2. the pilot uses the radio: it takes control, EVENT 43 once, the sticks come from it */
    g_fc_dbg_in_air = 1;
    radio_run(300, -200, 500, 1000, 1, NULL);
    fc_snapshot(&s); fc_setpoint(&sp);
    int lvl = -1;
    CHECK(s.input == PV2_INPUT_ELRS && count_events(PV2_E_RADIO_OVERRIDE, &lvl) == 1 && lvl == PV2_EV_WARN, "radio stick moved: radio in control, EVENT 43 WARN once");
    CHECK(sp.roll > 0.1f, "radio roll right -> roll setpoint %.3f rad", sp.roll);
    /* 4. in the air, the radio link is lost; the app is silent too: only the radio failsafe acts */
    radio_run(200, -200, 0, 1000, 1, NULL);
    (void)drain_events(0);
    radio_run(1500, 0, 0, 0, 0, NULL);                       /* both silent */
    fc_snapshot(&s);
    lvl = -1;
    CHECK((s.failsafe & PV2_FS_RADIO) && !(s.failsafe & PV2_FS_APP_LINK) && s.autom == PV2_AUTO_LAND,
          "radio lost 1.5 s in the air: FS RADIO, fsRadio RTH -> LAND without GNSS (fs %02X auto %u)", s.failsafe, s.autom);
    CHECK(count_events(PV2_E_FAILSAFE_RADIO, &lvl) == 1 && lvl == PV2_EV_ERROR, "EVENT 21 ERROR once");
    /* 5. fsRadio = hover: the link back gives the control back */
    (void)post(PV2_OP_SET_MODE, 4, (const uint8_t[1]){ PV2_MODE_ANGLE }, 1, NULL);   /* the pilot cancels the LAND */
    { uint8_t why; (void)params_set(12, 0.0f, &why); }
    radio_run(300, -200, 0, 1000, 1, NULL);
    radio_run(1200, 0, 0, 0, 1, NULL);
    fc_snapshot(&s);
    CHECK(s.autom == PV2_AUTO_HOLD && (s.failsafe & PV2_FS_RADIO), "fsRadio hover: HOLD");
    radio_run(200, -200, 0, 1000, 1, NULL);
    fc_snapshot(&s);
    CHECK(s.autom == PV2_AUTO_NONE && !(s.failsafe & PV2_FS_RADIO), "radio back: control back to the pilot");
    /* 6. on the ground, armed, radio lost 3 s: disarm */
    g_fc_dbg_in_air = 0;
    radio_run(100, -1000, 0, 1000, 1, NULL);
    radio_run(3200, 0, 0, 0, 1, NULL);
    fc_snapshot(&s);
    CHECK(!s.armed, "armed on the ground, radio lost 3 s: disarmed");
    /* 3. the radio still has control (disarmed, not idle long enough): app ARM refused */
    radio_run(100, -1000, 0, 1800, 1, NULL);                   /* arm switch on */
    r = post(PV2_OP_ARM, 3, arm, 2, NULL);
    CHECK(r.result == PV2_ACK_REJECTED && r.reason == PV2_R_RADIO_OVERRIDE, "app ARM refused while the radio has control (%u/%u)", r.result, r.reason);
    /* 7. disarmed, the radio idle for 1 s: control back to the app */
    radio_run(1200, -1000, 0, 1000, 1, NULL);
    fc_snapshot(&s);
    CHECK(s.input == PV2_INPUT_APP, "disarmed and the radio idle 1 s: the app has control again (input %u)", s.input);
    { uint8_t why; (void)params_set(12, 2.0f, &why); }
    rc_input_init();
    (void)v;
}

static void test_alt_est(void)
{
    alt_est_cfg_t c; alt_est_default_cfg(&c);
    alt_est_t e; alt_est_init(&e, &c);
    srand(1);
    const float p0 = 101325.0f;
    float h = 0, v = 0, err2 = 0, verr2 = 0; int n = 0;
    float bias = 0.0f;                       /* baro drift */
    for (int k = 0; k < 240 * 60; ++k) {
        const float t = k / 240.0f;
        const float a = (t > 10 && t < 12) ? 0.5f : ((t > 12 && t < 14) ? -0.5f : 0.0f);
        v += a / 240.0f; h += v / 240.0f;
        const float an = a + 0.4f * ((rand() / (float)RAND_MAX) - 0.5f) * 3.46f + 0.05f;  /* noise + acc bias */
        alt_est_predict(&e, an, 1.0f / 240.0f);
        if (k % 5 == 0) {
            bias += 0.0005f;
            const float hb = h + bias + 0.25f * ((rand() / (float)RAND_MAX) - 0.5f) * 3.46f;
            alt_est_baro(&e, p0 * powf(1.0f - hb / 44330.0f, 1.0f / 0.190295f));
        }
        if (t > 20) { err2 += (alt_est_h(&e) - h - bias) * (alt_est_h(&e) - h - bias); verr2 += (alt_est_vz(&e) - v) * (alt_est_vz(&e) - v); n++; }
    }
    CHECK(e.valid, "baro valid");
    CHECK(sqrtf(err2 / n) < 0.15f, "baro-only height RMS %.3f m (follows the baro)", sqrtf(err2 / n));
    CHECK(sqrtf(verr2 / n) < 0.08f, "baro-only vz RMS %.3f m/s", sqrtf(verr2 / n));

    /* with ToF: the baro drift is estimated, the height follows the ToF */
    alt_est_init(&e, &c);
    h = 0; v = 0; bias = 0; err2 = 0; n = 0;
    for (int k = 0; k < 240 * 60; ++k) {
        const float t = k / 240.0f;
        const float a = (t > 10 && t < 12) ? 0.5f : ((t > 12 && t < 14) ? -0.5f : 0.0f);
        v += a / 240.0f; h += v / 240.0f;
        alt_est_predict(&e, a + 0.4f * ((rand() / (float)RAND_MAX) - 0.5f) * 3.46f, 1.0f / 240.0f);
        if (k % 5 == 0) {
            bias += 0.002f;
            alt_est_baro(&e, p0 * powf(1.0f - (h + bias) / 44330.0f, 1.0f / 0.190295f));
        }
        if (k % 8 == 0) {
            const float r = h + c.tof_offset_m + 0.01f * ((rand() / (float)RAND_MAX) - 0.5f);
            alt_est_tof(&e, r, 1, 1.0f);
        }
        if (t > 20) { const float d = (alt_est_h(&e) - e.ground) - h; err2 += d * d; n++; }
    }
    CHECK(sqrtf(err2 / n) < 0.03f, "ToF height RMS %.3f m with 0.1 m/s baro drift... (%.2f m drift)", sqrtf(err2 / n), bias);
    CHECK(e.tof_valid && !e.tof_ground, "ToF valid in the air");

    /* landed: below the minimum range -> on the ground */
    alt_est_tof(&e, 0.012f, 0, 1.0f);
    CHECK(e.tof_ground, "short reading = ground");
    /* step onto a table: re-anchored after 0.5 s */
    alt_est_init(&e, &c);
    for (int k = 0; k < 240 * 4; ++k) { alt_est_predict(&e, 0, 1.0f / 240); if (k % 8 == 0) alt_est_tof(&e, 1.0f, 1, 1.0f); }
    for (int k = 0; k < 240 * 2; ++k) { alt_est_predict(&e, 0, 1.0f / 240); if (k % 8 == 0) alt_est_tof(&e, 0.3f, 1, 1.0f); }
    CHECK(fabsf(e.tof_h - 0.2887f) < 0.02f && fabsf(alt_est_h(&e) - e.ground - e.tof_h) < 0.05f, "re-anchored (tof_h %.3f)", e.tof_h);

    /* accel_up: level at rest = 0, 30 deg roll with 1 g/cos -> 0 */
    const float q0[4] = { 1, 0, 0, 0 }, f0[3] = { 0, 0, 1.0f };
    CHECK(fabsf(alt_est_accel_up(q0, f0, 1.0f)) < 1e-4f, "accel_up level");
    const float qr[4] = { cosf(0.2618f), sinf(0.2618f), 0, 0 }, fr[3] = { 0, 0, 1.0f / cosf(0.5236f) };
    CHECK(fabsf(alt_est_accel_up(qr, fr, 1.0f)) < 1e-3f, "accel_up tilted %.4f", alt_est_accel_up(qr, fr, 1.0f));
}

/* ---- simulated VL53L1X: 16-bit registers ---- */
static uint8_t s_vreg[0x200];
static int32_t v_wr(void *h, uint16_t r, const uint8_t *d, uint16_t n) { memcpy(&s_vreg[r], d, n); if (r == 0x87 && d[0] == 0x40) s_vreg[0x31] = 0x01; if (r == 0x86) s_vreg[0x31] = 0x00; return 0; }
static int32_t v_rd(void *h, uint16_t r, uint8_t *d, uint16_t n) { memcpy(d, &s_vreg[r], n); return 0; }
static uint8_t flow_item(uint8_t ok, uint8_t in_air, uint8_t q)
{
    checks_in_t in; memset(&in, 0, sizeof in);
    in.sensors = PV2_S_IMU | PV2_S_BARO | PV2_S_TOF | PV2_S_FLOW;
    in.imu_ok = 1; in.est_running = 1; in.alt_valid = 1;
    in.flow_ok = ok; in.in_air = in_air; in.flow_q = q;
    pv2_check_t it[PV2_CHECKS_MAX]; uint8_t rdy, bf;
    const uint8_t n = checks_build(&in, it, &rdy, &bf);
    for (uint8_t i = 0; i < n; ++i) if (it[i].id == PV2_CHK_FLOW) return it[i].state;
    return 0xFF;
}

static void test_checks_flow(void)
{
    CHECK(flow_item(1, 0, 5) == PV2_CK_OK, "on the ground: low quality is expected (below 80 mm)");
    CHECK(flow_item(0, 0, 0) == PV2_CK_WARN, "on the ground: bridge / sensor not working -> WARN");
    CHECK(flow_item(1, 1, 30) == PV2_CK_WARN && flow_item(1, 1, 80) == PV2_CK_OK, "in the air: WARN below 50 %%");
}

static void test_drivers(void)
{
    memset(s_vreg, 0, sizeof s_vreg);
    s_vreg[0xE5] = 1; s_vreg[0x10F] = 0xEA; s_vreg[0x110] = 0xCC;
    s_vreg[0x30] = 0x01;                  /* active-high data ready -> polarity 1 */
    s_vreg[0x31] = 0x01;
    s_vreg[0xDE] = 0x00; s_vreg[0xDF] = 0x40;
    vl53_t d; const vl53_bus_t b = { v_wr, v_rd, NULL, NULL };
    CHECK(vl53_init(&d, &b, 33) == 0, "vl53 init");
    CHECK(s_vreg[0x2D + 0x46 - 0x2D] == 0x20 && s_vreg[0x87] == 0x40, "default config written, ranging");
    CHECK(s_vreg[0x60] == 0x0F && s_vreg[0x5E] == 0x00 && s_vreg[0x5F] == 0x60, "long mode, 33 ms");
    s_vreg[0x31] = 0x01; s_vreg[0x89] = 9; s_vreg[0x96] = 0x04; s_vreg[0x97] = 0xD2;
    vl53_sample_t smp;
    CHECK(vl53_poll(&d, &smp) == 1 && smp.mm == 1234 && smp.status == 0, "vl53 sample %u st %u", smp.mm, smp.status);

    const uint8_t raw[FLOW_FRAME_LEN] = { 0x03, 7, 0x10, 0x00, 0xF6, 0xFF, 90, 0x34, 0x12, 100, 0 };
    flow_frame_t fr; flow_decode(raw, &fr);
    CHECK(fr.dx == 16 && fr.dy == -10 && fr.squal == 90 && fr.shutter == 0x1234 && fabsf(fr.dt_s - 0.01f) < 1e-6f, "flow decode");
}

/* ---- AppTask protocol ---- */
static size_t fake_init(uint8_t *dst, size_t cap)
{
    uint8_t pl[137] = { 0 };
    return tlm_encode(PV2_FT_INIT, pl, sizeof pl, dst, cap);
}

static void app_run(int ms, fc_vert_t *v, int hb)
{
    for (int k = 0; k < ms / 4; ++k) {
        now += 4;
        if (hb && (k % 25) == 0) heartbeat();
        fc_service(now, 1, v);
        if ((k & 1) == 0) app_step(now);
        drain();
    }
}

static void test_app(void)
{
    host_flash_erase();
    tlm_ble_reset();
    appl_reset();
    now = 1000;
    const float hi[3] = { 0, 0, 0 };
    (void)app_task_boot(hi);
    {
        fsh_est_t e; memset(&e, 0, sizeof e);
        e.imu_ok = 1; e.running = 1; e.alt_valid = 1; e.q[0] = 1; e.flow_q = 0xFF; e.tof_mm = 0xFFFF;
        e.sensors = PV2_S_IMU | PV2_S_MAG_FC | PV2_S_BARO; e.baro_pa = 101000;
        e.att_unc_deg = 0.5f; e.loop_hz = 240;
        fsh_est_write(&e);
        fsh_apply_t a; while (fsh_apply_take(&a)) {}
    }
    { const fc_config_t c = { 0, 4, 0, NULL, NULL, NULL, 0, NULL }; fc_init(&c); }
    app_task_cfg_t ac; memset(&ac, 0, sizeof ac);
    ac.hw_rev = 'N'; ac.n_motors = 4; ac.uid[0] = 0x12345678; ac.get_init = fake_init;
    app_init(&ac);

    fc_vert_t v = { 1, 0, 0, 0, 0, 0, 1.0f, 0xFF, 0, 0 };
    tlm_ble_set_state(TLM_BLE_CONNECTED);
    tlm_ble_set_state(TLM_BLE_STREAMING);
    appl_on_connect();
    int f0 = s_nf;
    app_run(40, &v, 1);
    CHECK(s_nf - f0 >= 3 && s_f[f0].type == PV2_FT_CAPS && s_f[f0 + 1].type == PV2_FT_INIT && s_f[f0 + 2].type == PV2_FT_CHECKS,
          "subscribe order CAPS, INIT, CHECKS (got %02X %02X %02X)", s_f[f0].type, s_f[f0 + 1].type, s_f[f0 + 2].type);
    {
        pv2_caps_t c; memcpy(&c, s_f[f0].p, sizeof c);
        CHECK(c.proto == 2 && (c.sensors & PV2_S_BARO) && !(c.sensors & PV2_S_TOF) && !strcmp(c.name, "ACwing #1") &&
              !strncmp(c.serial, "ACW-", 4), "CAPS content (sensors %04X name %s)", c.sensors, c.name);
    }
    app_run(2000, &v, 1);
    CHECK(count_type(f0, PV2_FT_FLIGHT) >= 18 && count_type(f0, PV2_FT_FLIGHT) <= 23, "FLIGHT 10 Hz (%d)", count_type(f0, PV2_FT_FLIGHT));
    CHECK(count_type(f0, PV2_FT_NAV) >= 9, "NAV sent on indoor models too");
    CHECK(count_type(f0, PV2_FT_SYSTEM) >= 3 && count_type(f0, PV2_FT_CHECKS) >= 4, "SYSTEM 2 Hz, CHECKS 2 Hz");
    CHECK(count_type(f0, PV2_FT_POWER) == 0, "no POWER without ADC");

    /* PARAM_SET + repeat with the same seq: one execution, cached ACK twice */
    f0 = s_nf;
    uint8_t p[6]; const uint16_t id = 1; const float val = 1.5f;
    memcpy(p, &id, 2); memcpy(p + 2, &val, 4);
    cmd(PV2_OP_PARAM_SET, 10, p, 6);
    app_run(40, &v, 1);
    cmd(PV2_OP_PARAM_SET, 10, p, 6);
    app_run(40, &v, 1);
    int na; const pv2_ack_t *a = find_ack(f0, PV2_OP_PARAM_SET, 10, &na);
    CHECK(a && a->result == PV2_ACK_OK && na == 2, "PARAM_SET ACK x%d", na);
    CHECK(count_type(f0, PV2_FT_PARAM) == 1, "one PARAM echo (no second execution)");
    {
        pv2_param_t pr; for (int i = f0; i < s_nf; ++i) if (s_f[i].type == PV2_FT_PARAM) memcpy(&pr, s_f[i].p, sizeof pr);
        CHECK(pr.id == 1 && pr.value == 1.5f && (pr.flags & 1), "PARAM dirty echo");
    }
    /* out of range */
    const float bad = 9.0f; memcpy(p + 2, &bad, 4);
    cmd(PV2_OP_PARAM_SET, 11, p, 6); app_run(40, &v, 1);
    a = find_ack(f0, PV2_OP_PARAM_SET, 11, NULL);
    CHECK(a && a->result == PV2_ACK_INVALID && a->reason == PV2_R_OUT_OF_RANGE, "out of range");

    /* PARAM_SAVE: ACK, EVENT 60, full list paced */
    f0 = s_nf;
    cmd(PV2_OP_PARAM_SAVE, 12, NULL, 0); app_run(400, &v, 1);
    a = find_ack(f0, PV2_OP_PARAM_SAVE, 12, NULL);
    CHECK(a && a->result == PV2_ACK_OK, "save ack");
    CHECK(find_event(f0, PV2_E_PARAMS_SAVED) == 1, "event 60");
    const uint16_t vis = params_visible_count(PV2_S_IMU | PV2_S_MAG_FC | PV2_S_BARO);
    CHECK(count_type(f0, PV2_FT_PARAM) == vis, "list after save (%d of %u)", count_type(f0, PV2_FT_PARAM), vis);

    /* the saved value survives a reboot */
    params_defaults();
    (void)app_task_boot(hi);
    { fsh_apply_t x; while (fsh_apply_take(&x)) {} }
    CHECK(PARAM(P_TAKEOFF_ALT) == 1.5f, "param from flash (%.2f)", PARAM(P_TAKEOFF_ALT));
    app_init(&ac);

    /* ARM: checks fail (accelerometer never calibrated) -> CHECKS_FAILING */
    f0 = s_nf;
    const uint8_t arm[2] = { 0xA5, 0x5A };
    app_run(200, &v, 1);
    cmd(PV2_OP_ARM, 20, arm, 2); app_run(40, &v, 1);
    a = find_ack(f0, PV2_OP_ARM, 20, NULL);
    CHECK(a && a->result == PV2_ACK_REJECTED && a->reason == PV2_R_CHECKS_FAILING, "arm blocked by checks (%d/%d)", a ? a->result : -1, a ? a->reason : -1);
    /* armRequireChecks = 0: arm, TAKEOFF accepted with the baro */
    const uint16_t idarc = 50; const float zero = 0.0f;
    memcpy(p, &idarc, 2); memcpy(p + 2, &zero, 4);
    cmd(PV2_OP_PARAM_SET, 21, p, 6); app_run(40, &v, 1);
    cmd(PV2_OP_ARM, 22, arm, 2); app_run(40, &v, 1);
    a = find_ack(f0, PV2_OP_ARM, 22, NULL);
    CHECK(a && a->result == PV2_ACK_OK, "arm ok (%d/%d)", a ? a->result : -1, a ? a->reason : -1);
    CHECK(find_event(f0, PV2_E_ARMED) == 1, "event 1");
    cmd(PV2_OP_ARM, 22, arm, 2); app_run(40, &v, 1);     /* repeat: cached ACK, not executed again */
    int nack; (void)find_ack(f0, PV2_OP_ARM, 22, &nack);
    CHECK(nack == 2 && find_event(f0, PV2_E_ARMED) == 1, "repeated ARM not executed twice");
    /* flash save refused while armed */
    cmd(PV2_OP_PARAM_SAVE, 23, NULL, 0); app_run(40, &v, 1);
    a = find_ack(f0, PV2_OP_PARAM_SAVE, 23, NULL);
    CHECK(a && a->result == PV2_ACK_REJECTED && a->reason == PV2_R_ARMED, "save armed refused");
    const uint16_t alt = 0;
    cmd(PV2_OP_TAKEOFF, 24, &alt, 2); app_run(40, &v, 1);
    a = find_ack(f0, PV2_OP_TAKEOFF, 24, NULL);
    CHECK(a && a->result == PV2_ACK_OK, "takeoff via app (%d/%d)", a ? a->result : -1, a ? a->reason : -1);
    {
        pv2_event_hdr_t h; int found = 0;
        for (int i = f0; i < s_nf; ++i) if (s_f[i].type == PV2_FT_EVENT) { memcpy(&h, s_f[i].p, sizeof h); if (h.code == 3) { found = 1; break; } }
        CHECK(found && h.arg == 150, "EVENT 3 with the saved takeoffAlt (arg %d)", (int)h.arg);
    }
    /* KILL: event 70 CRITICAL x3 */
    const uint8_t kill[4] = { 0x4B, 0x49, 0x4C, 0x4C };
    f0 = s_nf;
    cmd(PV2_OP_KILL, 25, kill, 4); app_run(400, &v, 1);
    CHECK(find_event(f0, PV2_E_MOTOR_KILLED) == 3, "KILL event x3 (%d)", find_event(f0, PV2_E_MOTOR_KILLED));
    CHECK(find_event(f0, PV2_E_DISARMED) == 1, "disarmed");
    /* unknown opcode */
    cmd(0x5F, 26, NULL, 0); app_run(40, &v, 1);
    a = find_ack(f0, 0x5F, 26, NULL);
    CHECK(a && a->result == PV2_ACK_UNSUPPORTED, "unknown op");
    /* REQUEST POWER -> UNSUPPORTED (no ADC), REQUEST CAPS -> CAPS */
    const uint8_t rq = PV2_FT_POWER, rc2 = PV2_FT_CAPS;
    int fc0 = count_type(0, PV2_FT_CAPS);
    cmd(PV2_OP_REQUEST, 27, &rq, 1); cmd(PV2_OP_REQUEST, 28, &rc2, 1); app_run(40, &v, 1);
    CHECK(find_ack(f0, PV2_OP_REQUEST, 27, NULL)->result == PV2_ACK_UNSUPPORTED, "request power");
    CHECK(count_type(0, PV2_FT_CAPS) == fc0 + 1, "request caps");
    /* SET_NAME */
    const char nm[16] = "Banco";
    cmd(PV2_OP_SET_NAME, 29, nm, 16); app_run(40, &v, 1);
    {
        pv2_caps_t c; for (int i = f0; i < s_nf; ++i) if (s_f[i].type == PV2_FT_CAPS) memcpy(&c, s_f[i].p, sizeof c);
        CHECK(!strcmp(c.name, "Banco"), "name %s", c.name);
    }
    /* the Flow board plugged in while disarmed: CAPS with TOF|FLOW, TAKEOFF ends in POS_HOLD */
    g_ext_sensors = PV2_S_TOF | PV2_S_FLOW;
    fc0 = count_type(0, PV2_FT_CAPS);
    app_run(40, &v, 1);
    CHECK(count_type(0, PV2_FT_CAPS) == fc0 + 1, "CAPS re-sent on a new sensor");
    {
        pv2_caps_t c; memcpy(&c, s_f[s_nf - 1].type == PV2_FT_CAPS ? s_f[s_nf - 1].p : s_f[s_nf - 1].p, sizeof c);
        for (int i = f0; i < s_nf; ++i) if (s_f[i].type == PV2_FT_CAPS) memcpy(&c, s_f[i].p, sizeof c);
        CHECK((c.sensors & (PV2_S_TOF | PV2_S_FLOW)) == (PV2_S_TOF | PV2_S_FLOW), "CAPS sensors %04X", c.sensors);
    }
    g_ext_sensors = 0;
    /* flash failure is reported, not hidden */
    g_flash_fail = 1;
    cmd(PV2_OP_PARAM_SAVE, 30, NULL, 0); app_run(40, &v, 1);
    a = find_ack(f0, PV2_OP_PARAM_SAVE, 30, NULL);
    CHECK(a && a->result == PV2_ACK_REJECTED, "flash error -> REJECTED");
    g_flash_fail = 0;
    /* STREAM_RATES: FLIGHT at 2 Hz for this session */
    const uint8_t rates[5] = { 2, 0, 0, 0, 0 };
    cmd(PV2_OP_STREAM_RATES, 31, rates, 5); app_run(40, &v, 1);
    f0 = s_nf;
    app_run(2000, &v, 1);
    CHECK(count_type(f0, PV2_FT_FLIGHT) >= 3 && count_type(f0, PV2_FT_FLIGHT) <= 5, "STREAM_RATES: FLIGHT 2 Hz (%d in 2 s)", count_type(f0, PV2_FT_FLIGHT));
    /* new connection: the cache restarts, CAPS/INIT/CHECKS again */
    tlm_ble_set_state(TLM_BLE_CONNECTED);
    tlm_ble_set_state(TLM_BLE_STREAMING);
    f0 = s_nf;
    app_run(20, &v, 1);
    CHECK(s_nf - f0 >= 3 && s_f[f0].type == PV2_FT_CAPS && s_f[f0 + 1].type == PV2_FT_INIT && s_f[f0 + 2].type == PV2_FT_CHECKS, "re-subscribe order");
    /* ... and the stream rates are the defaults again */
    f0 = s_nf;
    app_run(2000, &v, 1);
    CHECK(count_type(f0, PV2_FT_FLIGHT) >= 18, "new session: FLIGHT back at 10 Hz (%d in 2 s)", count_type(f0, PV2_FT_FLIGHT));

    /* an ELRS receiver (CRSF): CAPS radio bit, SYSTEM link statistics, CHECKS item 6 */
    {
        rc_input_init();
        (void)rc_input_configure(RC_PROTO_CRSF, RC_MAP_AETR);
        uint8_t fr[26] = { CRSF_ADDR_FC, 24, CRSF_T_RC_CHANNELS };
        for (int b = 0; b < 22; ++b) fr[3 + b] = 0;            /* all channels 172 (stick bottom) is fine */
        fr[25] = rc_crsf_crc8(&fr[2], 23);
        uint8_t ls[14] = { CRSF_ADDR_FC, 12, CRSF_T_LINK_STATS, 62, 70, 88, 7, 0, 2, 3, 80, 100, 6 };
        ls[13] = rc_crsf_crc8(&ls[2], 11);
        f0 = s_nf;
        for (int k = 0; k < 300; ++k) {                        /* 600 ms of frames at 500 Hz, link stats at 10 Hz */
            rc_input_feed(fr, 26, now);
            if (k % 50 == 0) rc_input_feed(ls, 14, now);
            rc_input_service(now);
            app_run(2, &v, 1);
        }
        app_run(1000, &v, 1);                                  /* the receiver keeps sending */
        int caps_radio = 0, lq = -1, rssi = 0, item6 = -1, val6 = -1;
        for (int i = f0; i < s_nf; ++i) {
            if (s_f[i].type == PV2_FT_CAPS) { pv2_caps_t c; memcpy(&c, s_f[i].p, sizeof c); caps_radio = !!(c.sensors & PV2_S_ELRS); }
            if (s_f[i].type == PV2_FT_SYSTEM) { pv2_system_t y; memcpy(&y, s_f[i].p, sizeof y); lq = y.lq; rssi = y.rssi; }
            if (s_f[i].type == PV2_FT_CHECKS) {
                const uint8_t n = s_f[i].p[1];
                for (uint8_t k = 0; k < n; ++k) if (s_f[i].p[2 + 4 * k] == PV2_CHK_RADIO) { item6 = s_f[i].p[3 + 4 * k]; int16_t vv; memcpy(&vv, &s_f[i].p[4 + 4 * k], 2); val6 = vv; }
            }
        }
        CHECK(caps_radio, "receiver seen: CAPS radio bit");
        CHECK(lq == 88 && rssi == -62, "SYSTEM lq %d rssi %d from the CRSF link statistics", lq, rssi);
        CHECK(item6 == PV2_CK_OK && val6 == 88, "CHECKS 6 radio OK, LQ %d", val6);
        rc_input_init();                                       /* no receiver for the following tests */
    }
}

static void test_ble_priority(void)
{
    /* HI frames (ACK) overtake LO frames (STATE) at a frame boundary */
    tlm_ble_reset();
    tlm_ble_set_state(TLM_BLE_STREAMING);
    tlm_ble_open_streams();
    uint8_t f[128], st[71] = { 0 };
    const size_t n = tlm_encode(PV2_FT_STATE, st, sizeof st, f, sizeof f);
    for (int i = 0; i < 20; ++i) (void)tlm_ble_push(f, (uint16_t)n);
    uint8_t b[20];
    (void)tlm_ble_peek(b, sizeof b); tlm_ble_consume(20);          /* a STATE frame half sent */
    const size_t na = pv2_ack(0x12, 1, 0, 0, f, sizeof f);
    (void)tlm_ble_push_hi(f, (uint16_t)na);
    uint8_t out[300];
    const uint16_t k = tlm_ble_peek(out, sizeof out);
    tlm_ble_consume(k);
    /* the rest of the STATE frame (77 - 20 = 57 bytes), then the ACK */
    CHECK(k > 57 + 10 && out[57] == 0xA5 && out[59] == PV2_FT_ACK, "ACK right after the partial frame");
    /* LO full does not block HI */
    tlm_ble_reset(); tlm_ble_set_state(TLM_BLE_STREAMING); tlm_ble_open_streams();
    int lo = 0; while (tlm_ble_push(f, (uint16_t)n) == 0 && lo < 1000) lo++;
    CHECK(tlm_ble_push_hi(f, (uint16_t)na) == 0, "HI accepted with LO full (%d LO frames)", lo);
}

int main(void)
{
    s_bin = fopen("frames.bin", "wb");
    test_ack_cache();
    test_takeoff_rules();
    test_takeoff_land();
    test_takeoff_tof();
    test_app_link_failsafe();
    test_manual_collective();
    test_no_alt_estimate();
    test_ctrl_loop();
    test_radio();
    test_alt_est();
    test_drivers();
    test_checks_flow();
    test_app();
    if (s_bin) fclose(s_bin);
    s_bin = NULL;
    test_ble_priority();
    printf("%d passed, %d failed\n", s_pass, s_fail);
    return s_fail ? 1 : 0;
}
