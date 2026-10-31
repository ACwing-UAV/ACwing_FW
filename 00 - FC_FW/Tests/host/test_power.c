/*
 * Host tests of the battery chain: estimator (battery.c), BattTask signals,
 * battery failsafes in the flight core, BATTERY calibration, CHECKS item 1,
 * POWER frame, configuration image v1 -> v2 migration.
 */
#include "proto_v2.h"
#include "tlm_ble.h"
#include "app_link.h"
#include "fc_core.h"
#include "params.h"
#include "cfg_store.h"
#include "fc_shared.h"
#include "app_task.h"
#include "battery.h"
#include "batt_task.h"
#include "calib_mgr.h"
#include "checks.h"
#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include <math.h>

extern int g_flash_fail, g_flash_writes;
void host_flash_erase(void);

static int s_fail, s_pass;
#define CHECK(c, ...) do { if (c) s_pass++; else { s_fail++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)
#define NEAR(a, b, tol) (fabsf((float)(a) - (float)(b)) <= (tol))

static uint32_t now;

/* ------------------------------------------------------------ frames out */
typedef struct { uint8_t type, len; uint8_t p[255]; } frm_t;
static frm_t s_f[4096];
static int   s_nf;
static FILE *s_bin;                         /* power_frames.bin: decoded by the app's own decoders */

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
static const pv2_ack_t *find_ack(int from, uint8_t op, uint8_t seq)
{
    for (int i = from; i < s_nf; ++i)
        if (s_f[i].type == PV2_FT_ACK) {
            const pv2_ack_t *x = (const pv2_ack_t *)s_f[i].p;
            if (x->op == op && x->seq == seq) return x;
        }
    return NULL;
}
static void cmd(uint8_t op, uint8_t seq, const void *p, uint8_t n)
{
    uint8_t b[20];
    b[0] = op; b[1] = seq;
    if (n) memcpy(b + 2, p, n);
    appl_on_cmd_write(b, (uint16_t)(2 + n), now);
}

/* ------------------------------------------------------------ fake ADC */
static float s_pack_v, s_pack_i;            /* the "real" battery */
static int   s_adc_ok = 1;
#define SCALE_TRUE 112.0f                    /* the real divider is 1.8 % off nominal (110) */
static int fake_adc(float *pv, float *pi)
{
    if (!s_adc_ok) return -1;
    *pv = s_pack_v * 10.0f / SCALE_TRUE;
    *pi = s_pack_i * 0.1f;                   /* 0.1 V/A */
    return 0;
}

/* ------------------------------------------------------------ fc events */
static fc_event_t s_ev[64];
static int        s_nev;
static void on_event(const fc_event_t *e) { if (s_nev < 64) s_ev[s_nev++] = *e; }
static int events(uint16_t code)
{
    int n = 0;
    for (int i = 0; i < s_nev; ++i) n += (s_ev[i].code == code);
    return n;
}
static void drop_events(void) { fc_event_t e; while (fc_take_event(&e)) {} }

/* ================================================================== */
static void test_estimator(void)
{
    CHECK(batt_soc_from_ocv(3.0f) == 0.0f && batt_soc_from_ocv(4.3f) == 1.0f, "OCV ends");
    CHECK(NEAR(batt_soc_from_ocv(3.84f), 0.5f, 1e-3f), "OCV 3.84 V = 50 %%");
    float prev = -1.0f; int mono = 1;
    for (float v = 3.2f; v < 4.25f; v += 0.01f) { const float s = batt_soc_from_ocv(v); if (s < prev) mono = 0; prev = s; }
    CHECK(mono, "OCV curve monotonic");

    const batt_cfg_t cfg = { 110.0f, 1.0f, 0.1f, 1 };
    batt_limits_t L = { 3.6f, 3.5f, 3.3f, 450.0f, 0 };

    /* no battery (USB only) */
    batt_init(&cfg);
    for (int k = 0; k < 100; ++k) batt_step(0.0f, 0.0f, 20, 0, &L);
    CHECK(!batt_state()->present && batt_state()->cells == 0, "no battery: not present, 0 cells");

    /* cell count */
    const float packs[] = { 4.05f, 7.6f, 11.4f, 16.0f, 25.0f };
    const uint8_t cells[] = { 1, 2, 3, 4, 6 };
    for (int j = 0; j < 5; ++j) {
        batt_init(&cfg);
        for (int k = 0; k < 40; ++k) batt_step(packs[j] / 11.0f, 0.0f, 20, 0, &L);
        CHECK(batt_state()->present && batt_state()->cells == cells[j], "%.2f V -> %uS (got %u)",
              packs[j], cells[j], batt_state()->cells);
    }
    /* the parameter wins */
    batt_init(&cfg);
    L.cells = 2;
    for (int k = 0; k < 40; ++k) batt_step(7.0f / 11.0f, 0.0f, 20, 0, &L);
    CHECK(batt_state()->cells == 2, "battCells forces the count");
    L.cells = 0;

    /* warning level: 2 s below the threshold, kept while armed, left disarmed */
    batt_init(&cfg);
    for (int k = 0; k < 50; ++k) batt_step(3.9f / 11.0f, 0.0f, 20, 1, &L);
    CHECK(batt_state()->cells == 0, "no detection while armed");
    batt_init(&cfg);
    for (int k = 0; k < 50; ++k) batt_step(3.9f / 11.0f, 0.0f, 20, 0, &L);
    CHECK(batt_state()->cells == 1 && batt_state()->level == PV2_BATT_OK, "1S at 3.9 V, OK");
    int t_low = -1;
    for (int k = 0; k < 200; ++k) {
        batt_step(3.45f / 11.0f, 0.0f, 20, 1, &L);
        if (t_low < 0 && batt_state()->level == PV2_BATT_CRITICAL) t_low = k * 20;
    }
    /* the cell voltage is low-pass filtered (0.5 s) before the 2 s timer */
    CHECK(t_low >= 2000 && t_low <= 3400, "CRITICAL after >= 2 s below (%d ms)", t_low);
    for (int k = 0; k < 300; ++k) batt_step(3.8f / 11.0f, 0.0f, 20, 1, &L);
    CHECK(batt_state()->level == PV2_BATT_CRITICAL, "kept while armed (no flicker on throttle cut)");
    for (int k = 0; k < 300; ++k) batt_step(3.8f / 11.0f, 0.0f, 20, 0, &L);
    CHECK(batt_state()->level == PV2_BATT_OK, "left when disarmed and recovered (level %u)", batt_state()->level);

    /* a 1 s dip does not count */
    for (int k = 0; k < 50; ++k) batt_step(3.2f / 11.0f, 0.0f, 20, 1, &L);
    for (int k = 0; k < 50; ++k) batt_step(3.8f / 11.0f, 0.0f, 20, 1, &L);
    CHECK(batt_state()->level == PV2_BATT_OK, "short sag ignored");

    /* coulomb counting and time left: 3 A for 60 s on 450 mAh */
    batt_init(&cfg);
    for (int k = 0; k < 50; ++k) batt_step(4.0f / 11.0f, 0.0f, 20, 0, &L);
    const float soc0 = batt_state()->soc;
    for (int k = 0; k < 3000; ++k) batt_step((4.0f - 3.0f * 0.04f) / 11.0f, 0.3f, 20, 1, &L);
    const batt_state_t *b = batt_state();
    CHECK(NEAR(b->used_mah, 50.0f, 0.5f), "used 50 mAh (%.2f)", b->used_mah);
    CHECK(NEAR(b->current, 3.0f, 0.01f), "current 3 A (%.3f)", b->current);
    CHECK(NEAR(soc0 - b->soc, 50.0f / 450.0f, 0.01f), "soc down by used/capacity (%.3f)", soc0 - b->soc);
    {
        const float expect = (b->soc - BATT_RESERVE_SOC) * 450.0f / 3000.0f * 3600.0f;
        CHECK(b->remain_s > 0.0f && NEAR(b->remain_s, expect, 10.0f), "remain %.0f s (expect %.0f)", b->remain_s, expect);
    }
    for (int k = 0; k < 10; ++k) batt_step(4.0f / 11.0f, 0.0f, 20, 0, &L);
    CHECK(batt_state()->remain_s < 0.0f, "remain n/a when disarmed");

    /* internal resistance from load steps: pack 2S, 25 mΩ per cell */
    batt_init(&cfg);
    for (int k = 0; k < 50; ++k) batt_step(8.0f / 11.0f, 0.0f, 20, 0, &L);
    for (int k = 0; k < 2000; ++k) {
        const float i = ((k / 25) & 1) ? 6.0f : 1.0f;
        batt_step((8.0f - i * 2.0f * 0.025f) / 11.0f, i * 0.1f, 20, 1, &L);
    }
    CHECK(NEAR(batt_state()->r_cell, 0.025f, 0.003f), "R per cell 25 mΩ (%.4f)", batt_state()->r_cell);
    CHECK(NEAR(batt_state()->cell_rest_v, 4.0f, 0.01f), "rest voltage compensated (%.3f)", batt_state()->cell_rest_v);

    /* voltage only: no current sensor -> no remain, pct from the voltage */
    const batt_cfg_t cv = { 110.0f, 1.0f, 0.1f, 0 };
    batt_init(&cv);
    for (int k = 0; k < 500; ++k) batt_step(3.84f / 11.0f, 0.5f, 20, 1, &L);
    CHECK(batt_state()->current == 0.0f && batt_state()->remain_s < 0.0f, "no sensor: I 0, remain n/a");

    /* battery removed */
    batt_init(&cfg);
    for (int k = 0; k < 50; ++k) batt_step(4.0f / 11.0f, 0.0f, 20, 0, &L);
    for (int k = 0; k < 20; ++k) batt_step(0.0f, 0.0f, 20, 0, &L);
    CHECK(!batt_state()->present && batt_state()->cells == 0, "battery removed");
}

/* ================================================================== */
static void bt_setup(uint16_t sensors)
{
    appl_reset();
    params_defaults();
    const fc_config_t c = { sensors, 4, 0, NULL, NULL, on_event, 1 };
    fc_init(&c);
    fc_set_ground_info(1, 0, 0);
    s_nev = 0;
    const batt_task_cfg_t bc = { { 110.0f, 1.0f, 0.1f, 1 }, fake_adc };
    batt_task_init(&bc);
    s_adc_ok = 1;
}

static fc_res_t post(uint8_t op, uint8_t seq, const void *p, uint8_t n)
{
    fc_cmd_t c; memset(&c, 0, sizeof c);
    c.op = op; c.seq = seq; c.len = n;
    if (n) memcpy(c.p, p, n);
    (void)fc_post(&c);
    fc_vert_t v = { 1, 0, 0, 0, 0, 0, 1.0f, 0xFF, 0 };
    fc_service(now, 1, &v);
    fc_res_t r; memset(&r, 0, sizeof r);
    while (fc_take_result(&r)) {}
    return r;
}

/* BattTask every 20 ms, flight core every 4 ms, app heartbeat 4 Hz */
static void run(int ms, int batt_alive)
{
    fc_vert_t v = { 1, 0, 0, 0, 0, 0, 1.0f, 0xFF, 0 };
    for (int k = 0; k < ms / 4; ++k) {
        now += 4;
        if ((k % 62) == 0) cmd(PV2_OP_HEARTBEAT, 0, NULL, 0);
        if (batt_alive && (now % 20u) == 0u) batt_task_step(now);
        fc_service(now, 1, &v);
    }
}

static void test_failsafes(void)
{
    const uint8_t arm[2] = { PV2_ARM_MAGIC0, PV2_ARM_MAGIC1 };
    fc_state_t s;

    /* ---- the battery monitor must have spoken before arming ---- */
    now = 100000;
    bt_setup(PV2_S_IMU);
    fc_res_t r = post(PV2_OP_ARM, 1, arm, 2);
    CHECK(r.result == PV2_ACK_REJECTED && r.reason == PV2_R_BATTERY_LOW, "no BattTask yet: ARM -> BATTERY_LOW");

    s_pack_v = 3.95f; s_pack_i = 0.1f;
    run(1000, 1);
    r = post(PV2_OP_ARM, 2, arm, 2);
    CHECK(r.result == PV2_ACK_OK, "ARM with a good battery (%d/%d)", r.result, r.reason);
    g_fc_dbg_in_air = 1;
    run(100, 1);
    fc_snapshot(&s);
    CHECK(s.in_air, "in air (bench flag)");

    /* LOW: event 30 once, no action */
    s_pack_v = 3.55f;
    run(3000, 1);
    fc_snapshot(&s);
    CHECK(events(PV2_E_BATTERY_LOW) == 1 && s.autom == PV2_AUTO_NONE && !(s.failsafe & PV2_FS_BATTERY),
          "LOW: EVENT 30, no action");
    {
        int arg = 0; for (int i = 0; i < s_nev; ++i) if (s_ev[i].code == PV2_E_BATTERY_LOW) arg = s_ev[i].arg;
        /* 3.55 V read through the uncalibrated divider (110 vs 112): 3.49 V */
        CHECK(arg > 3450 && arg < 3600, "EVENT 30 arg = cell mV (%d)", arg);
    }

    /* CRITICAL: EVENT 31 then 22, LAND (no GNSS), FS bit; the pilot may override */
    s_pack_v = 3.45f;
    run(3000, 1);
    fc_snapshot(&s);
    CHECK(events(PV2_E_BATTERY_CRITICAL) == 1 && events(PV2_E_FAILSAFE_BATTERY) == 1, "CRITICAL: EVENT 31 + 22");
    {
        int i31 = -1, i22 = -1;
        for (int i = 0; i < s_nev; ++i) { if (s_ev[i].code == 31 && i31 < 0) i31 = i; if (s_ev[i].code == 22 && i22 < 0) i22 = i; }
        CHECK(i31 >= 0 && i22 > i31 && s_ev[i22].arg == PV2_AUTO_LAND, "31 before 22, action LAND");
    }
    CHECK(s.autom == PV2_AUTO_LAND && (s.failsafe & PV2_FS_BATTERY), "LAND, FLIGHT.failsafe bit 2");

    /* app link short silence while landing for the battery: still LAND */
    run(1500, 1);                       /* heartbeats keep coming in run(): simulate silence below */
    {
        fc_vert_t v = { 1, 0, 0, 0, 0, 0, 1.0f, 0xFF, 0 };
        for (int k = 0; k < 400; ++k) { now += 4; if ((now % 20u) == 0u) batt_task_step(now); fc_service(now, 1, &v); }
    }
    fc_snapshot(&s);
    CHECK(s.autom == PV2_AUTO_LAND && (s.failsafe & PV2_FS_APP_LINK), "link loss flagged, battery landing kept");
    run(600, 1);

    const uint8_t mode = PV2_MODE_ANGLE;
    r = post(PV2_OP_SET_MODE, 3, &mode, 1);
    fc_snapshot(&s);
    CHECK(r.result == PV2_ACK_OK && s.autom == PV2_AUTO_NONE, "CRITICAL: the pilot can take over (SET_MODE)");
    run(500, 1);
    fc_snapshot(&s);
    CHECK(s.autom == PV2_AUTO_NONE && events(PV2_E_FAILSAFE_BATTERY) == 1, "no second trigger after override");

    /* EMERGENCY: EVENT 32, LAND, nothing but KILL overrides */
    s_pack_v = 3.2f;
    run(3000, 1);
    fc_snapshot(&s);
    CHECK(events(PV2_E_BATTERY_EMERGENCY) == 1 && s.autom == PV2_AUTO_LAND, "EMERGENCY: EVENT 32, LAND");
    r = post(PV2_OP_SET_MODE, 4, &mode, 1);
    CHECK(r.result == PV2_ACK_REJECTED && r.reason == PV2_R_BATTERY_LOW, "SET_MODE refused");
    r = post(PV2_OP_HOLD, 5, NULL, 0);
    fc_snapshot(&s);
    CHECK(r.result == PV2_ACK_REJECTED && s.autom == PV2_AUTO_LAND, "HOLD refused");
    {
        fc_vert_t v = { 1, 0, 0, 0, 0, 0, 1.0f, 0xFF, 0 };
        for (int k = 0; k < 1000; ++k) { now += 4; if ((now % 20u) == 0u) batt_task_step(now); fc_service(now, 1, &v); }
    }
    fc_snapshot(&s);
    CHECK(s.autom == PV2_AUTO_LAND, "app link loss cannot replace the landing");
    run(600, 1);
    const uint8_t kill[4] = { 0x4B, 0x49, 0x4C, 0x4C };
    r = post(PV2_OP_KILL, 6, kill, 4);
    fc_snapshot(&s);
    CHECK(r.result == PV2_ACK_OK && !s.armed, "KILL still works");
    g_fc_dbg_in_air = 0;

    /* on the ground with a critical battery: ARM refused */
    run(500, 1);
    r = post(PV2_OP_ARM, 7, arm, 2);
    CHECK(r.result == PV2_ACK_REJECTED && r.reason == PV2_R_BATTERY_LOW, "ARM refused below battCrit");

    /* ---- BattTask stops: EVENT 33, battery unknown ---- */
    now = 200000;
    bt_setup(PV2_S_IMU);
    s_pack_v = 4.1f;
    run(1000, 1);
    drop_events();
    s_nev = 0;
    run(1500, 0);
    fc_snapshot(&s);
    CHECK(events(PV2_E_BATTERY_MONITOR_LOST) == 1 && s.batt_stale, "BattTask silent > 1 s: EVENT 33, stale");
    r = post(PV2_OP_ARM, 8, arm, 2);
    CHECK(r.result == PV2_ACK_REJECTED && r.reason == PV2_R_BATTERY_LOW, "stale: ARM refused");
    run(200, 1);
    fc_snapshot(&s);
    CHECK(!s.batt_stale, "heartbeat back");

    /* ---- the last flight's charge ---- */
    now = 300000;
    bt_setup(PV2_S_IMU);
    s_pack_v = 4.1f; s_pack_i = 0.0f;
    run(1000, 1);
    r = post(PV2_OP_ARM, 9, arm, 2);
    s_pack_i = 3.0f;
    run(20000, 1);                     /* auto-disarm after 10 s on the ground: 3 A x 10 s = 8.3 mAh */
    s_pack_i = 0.0f;
    r = post(PV2_OP_DISARM, 10, NULL, 0);
    run(100, 1);
    batt_flight_t fl; memset(&fl, 0, sizeof fl);
    batt_task_last_flight(&fl);
    CHECK(fl.count == 1 && NEAR(fl.used_mah, 8.33f * 110.0f / 110.0f, 0.3f) && fl.scale_i == 1.0f, "last flight %.1f mAh", fl.used_mah);
}

/* ================================================================== */
static float s_cb_vbat = 4.0f, s_cb_used = 400.0f, s_cb_si = 1.0f;
static uint8_t s_applied_type; static cal_set_t s_applied;
static void cb_batt(float *v, float *u, float *si) { *v = s_cb_vbat; *u = s_cb_used; *si = s_cb_si; }
static void cb_apply(uint8_t t, const cal_set_t *s) { s_applied_type = t; s_applied = *s; }

static void test_calibration(void)
{
    static cal_set_t active;
    cal_set_defaults(&active, NULL);
    CHECK(active.battery.scale_v == 110.0f && active.battery.scale_i == 1.0f, "default scales");
    cal_config_t cc = { PV2_S_IMU, 0, cb_apply, cb_batt };
    uint8_t why;

    cc.sensors = PV2_S_IMU;
    cal_init(&cc, &active);
    CHECK(cal_start(PV2_CAL_BATTERY, 0, 0, 0, &why) == PV2_ACK_REJECTED && why == PV2_R_SENSOR_MISSING,
          "BATTERY without CURRENT -> SENSOR_MISSING");
    cc.sensors = PV2_S_IMU | PV2_S_CURRENT;
    cal_init(&cc, &active);
    CHECK(cal_start(PV2_CAL_BATTERY, 0, 1, 0, &why) == PV2_ACK_REJECTED && why == PV2_R_ARMED, "armed -> ARMED");
    CHECK(cal_start(PV2_CAL_BATTERY, 0, 0, 0, &why) == PV2_ACK_OK, "start");
    uint8_t f[64];
    size_t n = cal_frame(f, sizeof f);
    {
        pv2_calib_hdr_t h; memcpy(&h, f + 4, sizeof h);
        pv2_cal_battery_t d; memcpy(&d, f + 4 + 6, sizeof d);
        CHECK(n == 6 + 6 + 8 && h.type == 6 && h.state == PV2_CS_WAITING_USER && h.step == 1, "WAITING_USER step 1");
        CHECK(d.vbat_mv == 4000 && d.scale_v_x10 == 1100 && d.scale_i_e4 == 10000 && d.used_mah == 400, "CALIB data");
    }
    CHECK(cal_set_ref(PV2_CAL_BATTERY, 0, 40.0f, &why) == PV2_ACK_INVALID && why == PV2_R_OUT_OF_RANGE, "V out of range");
    CHECK(cal_set_ref(PV2_CAL_BATTERY, 0, 6.0f, &why) == PV2_ACK_INVALID, "V far from the reading refused");
    CHECK(cal_set_ref(PV2_CAL_BATTERY, 0, 4.08f, &why) == PV2_ACK_OK, "SET_REF volts");
    CHECK(s_applied_type == PV2_CAL_BATTERY && NEAR(s_applied.battery.scale_v, 110.0f * 4.08f / 4.0f, 1e-3f),
          "scaleV = scaleV*measured/read, applied (%.3f)", s_applied.battery.scale_v);
    s_cb_si = 1.0f;                                 /* the flight was counted with scaleI 1 */
    CHECK(cal_set_ref(PV2_CAL_BATTERY, 1, 10.0f, &why) == PV2_ACK_INVALID, "mAh out of range");
    CHECK(cal_set_ref(PV2_CAL_BATTERY, 1, 440.0f, &why) == PV2_ACK_OK, "SET_REF mAh");
    CHECK(NEAR(s_applied.battery.scale_i, 1.1f, 1e-4f), "scaleI = 440/400 (%.4f)", s_applied.battery.scale_i);
    n = cal_frame(f, sizeof f);
    {
        pv2_calib_hdr_t h; memcpy(&h, f + 4, sizeof h);
        CHECK(h.state == PV2_CS_DONE && h.step == 3, "DONE step 3");
    }
    CHECK(cal_save_check(&why) == PV2_ACK_OK, "save check");
    CHECK(NEAR(cal_result()->battery.scale_i, 1.1f, 1e-4f), "result carries the scales");
    cal_saved();
    (void)cal_frame(f, sizeof f);

    /* skip the current step with CAPTURE; abort restores the previous scales */
    s_cb_used = 5.0f;
    CHECK(cal_start(PV2_CAL_BATTERY, 0, 0, 0, &why) == PV2_ACK_OK, "start again");
    CHECK(cal_set_ref(PV2_CAL_BATTERY, 1, 440.0f, &why) == PV2_ACK_REJECTED, "no flight recorded -> refused");
    CHECK(cal_set_ref(PV2_CAL_BATTERY, 0, 3.9f, &why) == PV2_ACK_OK, "volts");
    CHECK(cal_abort(&why) == PV2_ACK_OK && s_applied.battery.scale_v == active.battery.scale_v, "abort reverts");
    CHECK(cal_start(PV2_CAL_BATTERY, 0, 0, 0, &why) == PV2_ACK_OK && cal_capture(0, &why) == PV2_ACK_OK, "CAPTURE skips");
    n = cal_frame(f, sizeof f);
    { pv2_calib_hdr_t h; memcpy(&h, f + 4, sizeof h); CHECK(h.state == PV2_CS_DONE && h.step == 3, "skip -> DONE"); }
    (void)cal_abort(&why);

    /* scale counted with: a second calibration from the same flight does not compound */
    s_cb_used = 400.0f; s_cb_si = 1.0f;
    CHECK(cal_start(PV2_CAL_BATTERY, 0, 0, 0, &why) == PV2_ACK_OK, "start 3");
    CHECK(cal_set_ref(PV2_CAL_BATTERY, 1, 440.0f, &why) == PV2_ACK_OK && NEAR(s_applied.battery.scale_i, 1.1f, 1e-4f),
          "same flight, same answer");
    (void)cal_abort(&why);
}

/* ================================================================== */
static void test_checks(void)
{
    checks_in_t in; memset(&in, 0, sizeof in);
    in.sensors = PV2_S_IMU; in.imu_ok = 1; in.est_running = 1; in.cal_done_mask = 1;
    pv2_check_t it[PV2_CHECKS_MAX]; uint8_t ready, bf;
    checks_build(&in, it, &ready, &bf);
    CHECK(it[0].id == PV2_CHK_BATTERY && it[0].state == PV2_CK_NA, "no monitor: N/A");
    in.batt_monitor = 1; in.batt_present = 1; in.batt_warn = 3.6f; in.batt_crit = 3.5f;
    const float cv[3] = { 3.9f, 3.55f, 3.45f };
    const uint8_t st[3] = { PV2_CK_OK, PV2_CK_WARN, PV2_CK_FAIL };
    for (int k = 0; k < 3; ++k) {
        in.batt_cell_v = cv[k];
        checks_build(&in, it, &ready, &bf);
        int16_t v; memcpy(&v, &it[0].value, 2);
        CHECK(it[0].state == st[k] && v == (int16_t)lrintf(cv[k] * 1000.0f) && bf == (st[k] == PV2_CK_FAIL),
              "cell %.2f -> state %u value %d", cv[k], it[0].state, v);
    }
    in.batt_cell_v = 3.9f; in.batt_level = PV2_BATT_CRITICAL;
    checks_build(&in, it, &ready, &bf);
    CHECK(it[0].state == PV2_CK_FAIL && !ready, "latched CRITICAL -> FAIL");
    in.batt_level = 0; in.batt_present = 0;
    checks_build(&in, it, &ready, &bf);
    CHECK(it[0].state == PV2_CK_WARN, "no battery (USB) -> WARN");
    in.batt_stale = 1;
    checks_build(&in, it, &ready, &bf);
    CHECK(it[0].state == PV2_CK_FAIL, "monitor stopped -> FAIL");
}

/* ================================================================== */
static size_t fake_init(uint8_t *dst, size_t cap) { (void)dst; (void)cap; return 0; }

static void test_power_frame(void)
{
    s_bin = fopen("power_frames.bin", "wb");
    host_flash_erase();
    tlm_ble_reset();
    appl_reset();
    now = 400000;
    const float hi[3] = { 0, 0, 0 };
    (void)app_task_boot(hi);
    {
        fsh_est_t e; memset(&e, 0, sizeof e);
        e.imu_ok = 1; e.running = 1; e.q[0] = 1; e.flow_q = 0xFF; e.tof_mm = 0xFFFF; e.sensors = PV2_S_IMU;
        fsh_est_write(&e);
        fsh_apply_t a; while (fsh_apply_take(&a)) {}
    }
    bt_setup(PV2_S_IMU | PV2_S_CURRENT);
    app_task_cfg_t ac; memset(&ac, 0, sizeof ac);
    ac.hw_rev = 'A'; ac.n_motors = 4; ac.get_init = fake_init;
    ac.batt_monitor = 1; ac.board_sensors = PV2_S_CURRENT;
    app_init(&ac);
    tlm_ble_set_state(TLM_BLE_CONNECTED);
    tlm_ble_set_state(TLM_BLE_STREAMING);
    appl_on_connect();

    s_pack_v = 3.9f * 3.0f; s_pack_i = 1.25f;   /* 3S */
    const int f0 = s_nf;
    fc_vert_t v = { 1, 0, 0, 0, 0, 0, 1.0f, 0xFF, 0 };
    for (int k = 0; k < 500; ++k) {
        now += 4;
        if ((k % 62) == 0) cmd(PV2_OP_HEARTBEAT, 0, NULL, 0);
        if ((now % 20u) == 0u) batt_task_step(now);
        fc_service(now, 1, &v);
        if ((k & 1) == 0) app_step(now);
        drain();
    }
    const int np = count_type(f0, PV2_FT_POWER);
    CHECK(np >= 9 && np <= 11, "POWER 5 Hz (%d in 2 s)", np);
    pv2_power_t p; memset(&p, 0, sizeof p);
    for (int i = f0; i < s_nf; ++i) if (s_f[i].type == PV2_FT_POWER) memcpy(&p, s_f[i].p, sizeof p);
    /* the divider is 112 in reality, 110 nominal: the reading is 1.8 % low until calibrated */
    const float vread = 11.7f * 110.0f / SCALE_TRUE;
    CHECK(p.cells == 3 && NEAR(p.vbat_mv, vread * 1000.0f, 15.0f), "POWER vbat %u mV (expect %.0f), %uS",
          p.vbat_mv, vread * 1000.0f, p.cells);
    CHECK(p.current_ca == 125 && p.warn == PV2_BATT_OK && p.remain_s == 0xFFFFu, "POWER current %d cA, warn, remain n/a",
          p.current_ca);
    int caps_cells = -1;
    for (int i = f0; i < s_nf; ++i) if (s_f[i].type == PV2_FT_CAPS) { pv2_caps_t c; memcpy(&c, s_f[i].p, sizeof c); caps_cells = c.cells; }
    {
        /* CAPS at subscribe was sent before the cells were counted; REQUEST it */
        const uint8_t req = PV2_FT_CAPS;
        const int f1 = s_nf;
        cmd(PV2_OP_REQUEST, 40, &req, 1);
        for (int k = 0; k < 10; ++k) { now += 4; fc_service(now, 1, &v); app_step(now); drain(); }
        for (int i = f1; i < s_nf; ++i) if (s_f[i].type == PV2_FT_CAPS) { pv2_caps_t c; memcpy(&c, s_f[i].p, sizeof c); caps_cells = c.cells; }
    }
    CHECK(caps_cells == 3, "CAPS cells %d", caps_cells);

    /* BATTERY calibration through the protocol: volts with a multimeter */
    int f1 = s_nf;
    uint8_t p2[6] = { PV2_CAL_BATTERY, 0 };
    cmd(PV2_OP_CAL_START, 41, p2, 1);
    for (int k = 0; k < 20; ++k) { now += 4; fc_service(now, 1, &v); app_step(now); drain(); }
    const pv2_ack_t *a = find_ack(f1, PV2_OP_CAL_START, 41);
    CHECK(a && a->result == PV2_ACK_OK, "CAL_START(6) %d/%d", a ? a->result : -1, a ? a->reason : -1);
    const float meas = 11.70f;
    memcpy(p2 + 2, &meas, 4);
    cmd(PV2_OP_CAL_SET_REF, 42, p2, 6);
    for (int k = 0; k < 200; ++k) {
        now += 4;
        if ((now % 20u) == 0u) batt_task_step(now);
        fc_service(now, 1, &v);
        if ((k & 1) == 0) app_step(now);
        drain();
    }
    a = find_ack(f1, PV2_OP_CAL_SET_REF, 42);
    CHECK(a && a->result == PV2_ACK_OK, "SET_REF volts via the protocol");
    batt_state_t b; batt_read(&b);
    CHECK(NEAR(b.vbat, 11.70f, 0.02f), "after calibration the reading matches the multimeter (%.3f)", b.vbat);
    cmd(PV2_OP_CAL_CAPTURE, 43, NULL, 0);
    cmd(PV2_OP_CAL_SAVE, 44, NULL, 0);
    for (int k = 0; k < 20; ++k) { now += 4; fc_service(now, 1, &v); app_step(now); drain(); }
    a = find_ack(f1, PV2_OP_CAL_SAVE, 44);
    cfg_image_t img;
    CHECK(a && a->result == PV2_ACK_OK && cfg_load(&img) == 0 && NEAR(img.cal.battery.scale_v, SCALE_TRUE, 0.2f),
          "CAL_SAVE stores scaleV (%.2f)", img.cal.battery.scale_v);

    /* a flight: its charge is saved when disarmed, for the current calibration */
    const uint8_t arm[2] = { PV2_ARM_MAGIC0, PV2_ARM_MAGIC1 };
    { uint8_t why; (void)params_set(50, 0.0f, &why); }   /* accel never calibrated on this test drone */
    for (int k = 0; k < 60; ++k) { now += 4; fc_service(now, 1, &v); app_step(now); drain(); }   /* ground info 10 Hz */
    f1 = s_nf;
    cmd(PV2_OP_ARM, 45, arm, 2);
    s_pack_i = 10.0f;
    for (int k = 0; k < 3500; ++k) {        /* armed 10 s (auto-disarm): 27.8 mAh */
        now += 4;
        if ((k % 62) == 0) cmd(PV2_OP_HEARTBEAT, 0, NULL, 0);
        if ((now % 20u) == 0u) batt_task_step(now);
        fc_service(now, 1, &v);
        if ((k & 1) == 0) app_step(now);
        drain();
    }
    a = find_ack(f1, PV2_OP_ARM, 45);
    CHECK(a && a->result == PV2_ACK_OK, "ARM %d/%d", a ? a->result : -1, a ? a->reason : -1);
    fc_state_t s; fc_snapshot(&s);
    CHECK(!s.armed, "auto-disarm (no take-off in 10 s)");
    CHECK(cfg_load(&img) == 0 && img.batt_used_mah > 20.0f && img.batt_used_scale_i == 1.0f,
          "last flight saved (%.1f mAh)", img.batt_used_mah);
    tlm_ble_set_state(TLM_BLE_OFF);
    if (s_bin) fclose(s_bin);
    s_bin = NULL;
}

/* ================================================================== */
/* version 1 image, as firmware 0.4.0 wrote it */
typedef struct {
    float off[3], scale[3]; float roll, pitch; float bias[3]; float hi[3], cscale[3];
    uint8_t rev, pad[3]; uint32_t done;
} cal_v1_t;
typedef struct {
    uint32_t magic; uint16_t version, size; uint32_t seq;
    float params[24]; char name[16]; cal_v1_t cal, bak; uint32_t bak_mask; uint32_t crc;
} img_v1_t;
extern const void *cfg_flash_page(int p);
extern int cfg_flash_write(int p, const void *d, uint32_t len);

static void test_migration(void)
{
    host_flash_erase();
    img_v1_t v1; memset(&v1, 0, sizeof v1);
    v1.magic = CFG_MAGIC; v1.version = 1; v1.size = sizeof v1; v1.seq = 7;
    for (int i = 0; i < 24; ++i) v1.params[i] = g_param_def[i].def;
    v1.params[P_TAKEOFF_ALT] = 1.7f;
    v1.params[P_BATT_CRIT] = 3.45f;
    memcpy(v1.name, "Bench", 5);
    v1.cal.off[0] = 0.01f; v1.cal.bias[2] = -0.002f; v1.cal.hi[1] = 12.5f; v1.cal.cscale[0] = 1.02f;
    v1.cal.rev = 5; v1.cal.done = 0x0F;
    v1.bak_mask = 4;
    v1.crc = cfg_crc32(&v1, offsetof(img_v1_t, crc));
    (void)cfg_flash_write(0, &v1, sizeof v1);

    cfg_image_t img;
    CHECK(cfg_load(&img) == 0, "v1 image loaded");
    float v[PARAM_COUNT];
    cfg_params_get(&img, v);
    CHECK(v[P_TAKEOFF_ALT] == 1.7f && v[P_BATT_CRIT] == 3.45f && v[P_LOG_MODE] == g_param_def[P_LOG_MODE].def &&
          v[P_BATT_CAPACITY] == 0.0f, "parameters kept, new ones default");
    CHECK(!strcmp(img.name, "Bench") && img.cal.accel.off[0] == 0.01f && img.cal.gyro.bias[2] == -0.002f &&
          img.cal.compass.hi[1] == 12.5f && img.cal.compass.scale[0] == 1.02f && img.cal.motors.rev_mask == 5 &&
          img.cal.done_mask == 0x0F && img.bak_mask == 4, "name and calibrations kept");
    CHECK(img.cal.battery.scale_v == 110.0f && img.cal.battery.scale_i == 1.0f, "battery scales default");
    /* next save writes version 2 on the other page, read back as v2 */
    CHECK(cfg_save(&img) == 0, "save v2");
    cfg_image_t img2;
    CHECK(cfg_load(&img2) == 0 && img2.version == CFG_VERSION && img2.seq == 8 && !strcmp(img2.name, "Bench"),
          "reload v2 (seq %u)", (unsigned)img2.seq);
    /* an id the firmware does not know is ignored, a missing one takes its default */
    img2.params[3].id = 999;
    cfg_params_get(&img2, v);
    CHECK(v[P_MAX_TILT] == g_param_def[P_MAX_TILT].def, "unknown id ignored, missing -> default");
}

int main(void)
{
    test_estimator();
    test_failsafes();
    test_calibration();
    test_checks();
    test_power_frame();
    test_migration();
    printf("power: %d passed, %d failed\n", s_pass, s_fail);
    return s_fail ? 1 : 0;
}
