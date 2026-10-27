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
typedef struct { float h, vz, hover; } plant_t;
static void plant_step(plant_t *pl, float thrust, uint8_t armed, float dt)
{
    float a = armed ? 9.81f * (thrust / pl->hover - 1.0f) - 0.6f * pl->vz : -9.81f;
    pl->vz += a * dt;
    pl->h  += pl->vz * dt;
    if (pl->h <= 0.0f) { pl->h = 0.0f; if (pl->vz < 0.0f) pl->vz = 0.0f; }
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
    const fc_config_t c = { sensors, 4, 0, NULL, NULL };
    fc_init(&c);
    params_defaults();
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
    plant_t pl = { 0, 0, 0.42f };
    fc_vert_t v = { 1, 0, 0, 0, 0, 0, 1.0f, 0xFF };
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
        v.h = pl.h; v.vz = pl.vz;
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
        v.h = pl.h; v.vz = pl.vz;
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
    plant_t pl = { 0, 0, 0.40f };
    fc_vert_t v = { 1, 0, 0, 0, 1, 0, 1.0f, 80 };
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
        v.h = pl.h; v.vz = pl.vz;
        v.tof_h = pl.h; v.tof_ground = pl.h < 0.03f; v.tof_valid = !v.tof_ground;
        fc_service(now, 1, &v);
    }
    fc_snapshot(&s);
    CHECK(s.mode == PV2_MODE_POS_HOLD && s.autom == PV2_AUTO_NONE, "POS_HOLD after takeoff with FLOW (mode %d)", s.mode);
    CHECK(fabsf(pl.h - 1.5f) < 0.1f, "holds 1.5 m (%.2f)", pl.h);
    r = post(PV2_OP_LAND, 3, NULL, 0, &v);
    float vmin_low = 0, vmin_high = 0;
    int landed = 0;
    for (int k = 0; k < 15000 / 4; ++k) {
        now += 4; if (k % 25 == 0) heartbeat();
        fc_setpoint_t sp; fc_setpoint(&sp);
        plant_step(&pl, sp.thrust, sp.armed, 0.004f);
        v.h = pl.h; v.vz = pl.vz;
        v.tof_h = pl.h; v.tof_ground = pl.h < 0.03f; v.tof_valid = !v.tof_ground;
        fc_service(now, 1, &v);
        if (pl.h < 0.8f && pl.h > 0.05f && pl.vz < vmin_low) vmin_low = pl.vz;
        if (pl.h > 1.1f && pl.vz < vmin_high) vmin_high = pl.vz;
        fc_event_t e; while (fc_take_event(&e)) landed += (e.code == PV2_E_LANDED);
    }
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
        v.h = pl.h; v.vz = pl.vz; v.tof_h = pl.h; v.tof_ground = pl.h < 0.03f; v.tof_valid = !v.tof_ground;
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
        v.h = pl.h; v.vz = pl.vz; v.tof_h = pl.h;
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
    plant_t pl = { 0, 0, 0.42f };
    fc_vert_t v = { 1, 0, 0, 0, 0, 0, 1.0f, 0xFF };
    const uint8_t arm[2] = { 0xA5, 0x5A };
    (void)post(PV2_OP_ARM, 1, arm, 2, &v);
    (void)post(PV2_OP_TAKEOFF, 2, NULL, 0, &v);
    for (int k = 0; k < 8000 / 4; ++k) {
        now += 4; if (k % 25 == 0) heartbeat();
        fc_setpoint_t sp; fc_setpoint(&sp);
        plant_step(&pl, sp.thrust, sp.armed, 0.004f);
        v.h = pl.h; v.vz = pl.vz;
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
        v.h = pl.h; v.vz = pl.vz;
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
    { const fc_config_t c = { 0, 4, 0, NULL, NULL }; fc_init(&c); }
    app_task_cfg_t ac; memset(&ac, 0, sizeof ac);
    ac.hw_rev = 'N'; ac.n_motors = 4; ac.uid[0] = 0x12345678; ac.get_init = fake_init;
    app_init(&ac);

    fc_vert_t v = { 1, 0, 0, 0, 0, 0, 1.0f, 0xFF };
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
    /* new connection: the cache restarts, CAPS/INIT/CHECKS again */
    tlm_ble_set_state(TLM_BLE_CONNECTED);
    tlm_ble_set_state(TLM_BLE_STREAMING);
    f0 = s_nf;
    app_run(20, &v, 1);
    CHECK(s_nf - f0 >= 3 && s_f[f0].type == PV2_FT_CAPS && s_f[f0 + 1].type == PV2_FT_INIT && s_f[f0 + 2].type == PV2_FT_CHECKS, "re-subscribe order");
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
    test_alt_est();
    test_drivers();
    test_app();
    if (s_bin) fclose(s_bin);
    s_bin = NULL;
    test_ble_priority();
    printf("%d passed, %d failed\n", s_pass, s_fail);
    return s_fail ? 1 : 0;
}
