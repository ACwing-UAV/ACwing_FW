/*
 * Host tests of the USB link as a ground link (ADR 0009):
 *  - USB carries telemetry, parameters and calibrations, never flight commands;
 *  - ARM, TAKEOFF and MOTOR_TEST are refused while a USB host is connected,
 *    whatever link sends them;
 *  - with BLE and USB both open, the first link opened has control of the
 *    configuration, the other one reads only;
 *  - ACKs go back to the link of the command; frames of the three USB
 *    producers (LogTask, AppTask, TelemetryTask) never interleave.
 * Runs the real LogTask, AppTask, flight core, BLE rings and USB link.
 */
#include "log_store.h"
#include "log_flash_ram.h"
#include "log_task.h"
#include "usb_link.h"
#include "proto_v2.h"
#include "telemetry.h"
#include "fc_core.h"
#include "app_link.h"
#include "params.h"
#include "app_task.h"
#include "tlm_ble.h"
#include "fc_shared.h"
#include "calib_mgr.h"
#include <stdio.h>
#include <string.h>

static int s_fail, s_pass;
#define CHECK(c, ...) do { if (c) s_pass++; else { s_fail++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static uint32_t now;

/* ------------------------------------------------------------------ frames seen on each link */
typedef struct { uint8_t type, len; uint8_t p[255]; } frm_t;
typedef struct { frm_t f[6000]; int n; uint8_t acc[32768]; size_t na; int bad; } link_t;
static link_t U, B;          /* USB, BLE */

static void parse(link_t *L)
{
    size_t i = 0;
    while (L->na - i >= 6) {
        if (L->acc[i] != 0xA5 || L->acc[i + 1] != 0x5A) { i++; L->bad++; continue; }
        const uint8_t len = L->acc[i + 3];
        if (L->na - i < (size_t)len + 6u) break;
        const uint16_t crc = tlm_crc16(&L->acc[i + 2], len + 2u);
        if (crc != (uint16_t)(L->acc[i + 4 + len] | L->acc[i + 5 + len] << 8)) { L->bad++; i++; continue; }
        if (L->n < 6000) { L->f[L->n].type = L->acc[i + 2]; L->f[L->n].len = len; memcpy(L->f[L->n].p, &L->acc[i + 4], len); L->n++; }
        i += len + 6u;
    }
    memmove(L->acc, L->acc + i, L->na - i); L->na -= i;
}

/* the board binding: drains the USB rings in odd-sized pieces, as the endpoint would */
static uint8_t s_host_reads = 1;
static void usb_pump(void)
{
    if (!usbl_host()) { usbl_tx_flush(); return; }
    if (!s_host_reads) return;
    uint8_t b[300];
    uint32_t n;
    while ((n = usbl_tx_peek(b, 37 + now % 200)) != 0) {
        usbl_tx_consume(n);
        if (U.na + n <= sizeof U.acc) { memcpy(U.acc + U.na, b, n); U.na += n; }
    }
    parse(&U);
}

static void ble_drain(void)
{
    uint8_t buf[244];
    uint16_t n;
    while ((n = tlm_ble_peek(buf, sizeof buf)) != 0) {
        tlm_ble_consume(n);
        if (B.na + n <= sizeof B.acc) { memcpy(B.acc + B.na, buf, n); B.na += n; }
    }
    parse(&B);
}

static void usb_cmd(uint8_t op, uint8_t seq, const void *a, uint8_t n)
{
    uint8_t pl[32], f[64];
    pl[0] = op; pl[1] = seq;
    if (n) memcpy(pl + 2, a, n);
    const size_t k = pv2_frame(PV2_FT_CMD, pl, (uint8_t)(n + 2), f, sizeof f);
    usbl_rx_isr(f, (uint32_t)k);
}

static void ble_cmd(uint8_t op, uint8_t seq, const void *a, uint8_t n)
{
    uint8_t b[20];
    b[0] = op; b[1] = seq;
    if (n) memcpy(b + 2, a, n);
    appl_on_cmd_write(b, (uint16_t)(2 + n), now);
}

static const pv2_ack_t *ack_in(const link_t *L, int from, uint8_t op, uint8_t seq)
{
    for (int i = from; i < L->n; ++i)
        if (L->f[i].type == PV2_FT_ACK) {
            const pv2_ack_t *a = (const pv2_ack_t *)L->f[i].p;
            if (a->op == op && a->seq == seq) return a;
        }
    return NULL;
}

static int count(const link_t *L, int from, uint8_t type)
{
    int c = 0;
    for (int i = from; i < L->n; ++i) if (L->f[i].type == type) c++;
    return c;
}

static const uint8_t *last_of(const link_t *L, int from, uint8_t type)
{
    const uint8_t *p = NULL;
    for (int i = from; i < L->n; ++i) if (L->f[i].type == type) p = L->f[i].p;
    return p;
}

/* CHECKS payload: ready u8, n u8, then {id, state, value i16}; item 12 state */
static int usb_item(const uint8_t *chk, uint8_t *ready)
{
    if (!chk) return -1;
    *ready = chk[0];
    for (uint8_t k = 0; k < chk[1]; ++k) {
        const uint8_t *it = chk + 2 + 4 * k;
        if (it[0] == PV2_CHK_USB) return it[1];
    }
    return -1;
}

static int s_ble_hb = 1;
static void run(int ms)
{
    for (int k = 0; k < ms; ++k) {
        now++;
        if (s_ble_hb && now % 200u == 0u) { const uint8_t hb[2] = { PV2_OP_HEARTBEAT, 0 }; appl_on_cmd_write(hb, 2, now); }
        if (now % 4u == 0u) { fc_vert_t v = { 1, 1.0f, 0, 0, 0, 0, 1.0f, 0xFF, 0, 0, { 0, 0 }, 0, 1.0f, { 0, 0, 0 }, 0, 0, 0, { 0, 0, 0 } }; fc_service(now, 1, &v); }
        if (now % 2u == 0u) log_task_step(now);
        if (now % 5u == 0u) { app_step(now); ble_drain(); }
    }
}

static void ble_connect(void)
{
    tlm_ble_set_state(TLM_BLE_CONNECTED);
    tlm_ble_set_state(TLM_BLE_STREAMING);
    appl_on_connect();
}
static void ble_disconnect(void) { appl_on_disconnect(); tlm_ble_set_state(TLM_BLE_READY); }

static void setup(void)
{
    static uint8_t ram[16 * 1024];
    static log_flash_t fl;
    memset(&U, 0, sizeof U); memset(&B, 0, sizeof B);
    memset(ram, 0, sizeof ram);
    log_flash_ram_init(&fl, ram, sizeof ram);
    appl_reset();
    tlm_ble_reset();
    params_defaults();
    { uint8_t why; params_set(50, 0.0f, &why); }          /* armRequireChecks off: only the USB rule is left */
    const fc_config_t fcc = { PV2_S_IMU, 4, 1, NULL, NULL, NULL, 0, usbl_host };
    fc_init(&fcc);
    fc_set_ground_info(1, 0, 0);
    {
        fsh_est_t e; memset(&e, 0, sizeof e);
        e.imu_ok = 1; e.running = 1; e.q[0] = 1; e.flow_q = 0xFF; e.tof_mm = 0xFFFF; e.sensors = PV2_S_IMU;
        fsh_est_write(&e);
    }
    const log_task_cfg_t lc = { &fl, 'N', { 0, 6, 0 }, NULL, usb_pump, NULL };
    log_task_init(&lc);                                     /* resets the USB link too */
    app_task_cfg_t ac; memset(&ac, 0, sizeof ac);
    ac.hw_rev = 'N'; ac.n_motors = 4; ac.motors_built = 1;
    app_init(&ac);
    now = 100000;
    s_ble_hb = 1;
    s_host_reads = 1;
}

/* ================================================================== USB only */
static void test_usb_only(void)
{
    setup();
    run(100);
    CHECK(U.n == 0, "no host: nothing on USB");

    /* a PC opens the port */
    usbl_set_configured(1); usbl_set_dtr(1);
    run(1200);
    CHECK(count(&U, 0, PV2_FT_CAPS) >= 1 && count(&U, 0, PV2_FT_CHECKS) >= 2, "USB open: CAPS and CHECKS");
    CHECK(count(&U, 0, PV2_FT_FLIGHT) >= 9 && count(&U, 0, PV2_FT_SYSTEM) >= 2, "USB open: FLIGHT 10 Hz, SYSTEM (%d, %d)",
          count(&U, 0, PV2_FT_FLIGHT), count(&U, 0, PV2_FT_SYSTEM));
    uint8_t ready = 9;
    const int st = usb_item(last_of(&U, 0, PV2_FT_CHECKS), &ready);
    CHECK(st == PV2_CK_FAIL && ready == 0, "CHECKS: item 12 USB in FAIL, not ready (%d, %u)", st, ready);

    /* parameters */
    int f0 = U.n;
    uint8_t a[6]; const uint16_t id = 1; const float v = 2.5f;
    memcpy(a, &id, 2); memcpy(a + 2, &v, 4);
    usb_cmd(PV2_OP_PARAM_SET, 10, a, 6);
    usb_cmd(PV2_OP_PARAM_LIST, 11, NULL, 0);
    run(600);
    const pv2_ack_t *k = ack_in(&U, f0, PV2_OP_PARAM_SET, 10);
    CHECK(k && k->result == PV2_ACK_OK, "PARAM_SET over USB: OK");
    CHECK(ack_in(&U, f0, PV2_OP_PARAM_LIST, 11) && count(&U, f0, PV2_FT_PARAM) >= params_visible_count(PV2_S_IMU),
          "PARAM_LIST over USB: %d PARAM frames", count(&U, f0, PV2_FT_PARAM));

    /* flight commands are refused, whatever they are */
    f0 = U.n;
    const uint8_t arm[2] = { PV2_ARM_MAGIC0, PV2_ARM_MAGIC1 };
    usb_cmd(PV2_OP_ARM, 20, arm, 2);
    usb_cmd(PV2_OP_KILL, 21, "KILL", 4);
    usb_cmd(PV2_OP_RC, 22, (const uint8_t[9]){ 0 }, 9);
    usb_cmd(PV2_OP_MOTOR_TEST, 23, (const uint8_t[5]){ 1, 8, 0xD0, 0x07, 0x50 }, 5);
    usb_cmd(PV2_OP_SEND_INIT, 24, NULL, 0);
    run(100);
    int ok = 1;
    for (uint8_t s = 20; s <= 23; ++s) {
        const pv2_ack_t *x = ack_in(&U, f0, s == 20 ? PV2_OP_ARM : s == 21 ? PV2_OP_KILL : s == 22 ? PV2_OP_RC : PV2_OP_MOTOR_TEST, s);
        if (!x || x->result != PV2_ACK_REJECTED || x->reason != PV2_R_NOT_OVER_USB) ok = 0;
    }
    { const pv2_ack_t *x = ack_in(&U, f0, PV2_OP_SEND_INIT, 24); if (!x || x->result != PV2_ACK_UNSUPPORTED) ok = 0; }
    fc_state_t f; memset(&f, 0, sizeof f); fc_snapshot(&f);
    appl_rc_t rc; uint32_t t;
    CHECK(ok && !f.armed && !appl_rc_read(&rc, &t), "ARM, KILL, RC, MOTOR_TEST over USB: REJECTED / NOT_OVER_USB, nothing reaches the core");

    /* calibration started over USB is aborted when the port closes */
    f0 = U.n;
    usb_cmd(PV2_OP_CAL_START, 30, (const uint8_t[2]){ PV2_CAL_GYRO, 0 }, 2);
    run(50);
    k = ack_in(&U, f0, PV2_OP_CAL_START, 30);
    CHECK(k && k->result == PV2_ACK_OK && cal_active(), "CAL_START over USB: OK");
    usbl_set_dtr(0);                                        /* program closed; still a host: open 3 s after the last command */
    run(1000);
    CHECK(cal_active(), "port closed less than 3 s ago: calibration still running");
    run(2500);
    CHECK(!cal_active(), "USB link closed: calibration aborted");

    /* a new USB session forgets the ACKs of the previous one (a new program starts its
     * sequence numbers again): seq 10 is executed, not answered from the old cache */
    usbl_set_dtr(1);
    run(50);
    f0 = U.n;
    usb_cmd(PV2_OP_PARAM_SET, 10, a, 6);
    run(50);
    k = ack_in(&U, f0, PV2_OP_PARAM_SET, 10);
    CHECK(k && k->result == PV2_ACK_OK && count(&U, f0, PV2_FT_PARAM) == 1, "new USB session: seq 10 executed again");
    /* within a session, a repeated (op, seq) gets the cached ACK and is not executed twice */
    f0 = U.n;
    usb_cmd(PV2_OP_PARAM_SET, 10, a, 6);
    run(50);
    k = ack_in(&U, f0, PV2_OP_PARAM_SET, 10);
    CHECK(k && k->result == PV2_ACK_OK && count(&U, f0, PV2_FT_PARAM) == 0, "repeated seq: cached ACK, not executed again");
    CHECK(U.bad == 0, "USB stream: every frame whole (%d bad bytes)", U.bad);
}

/* ================================================================== ARM inhibit, from BLE */
static void test_arm_inhibit(void)
{
    setup();
    ble_connect();
    run(300);
    usbl_set_configured(1);                                 /* cable to a PC, no program on the port */
    run(300);
    CHECK(!usbl_open(now) && usbl_host(), "host without a program: not open, but a host");
    int f0 = B.n;
    const uint8_t arm[2] = { PV2_ARM_MAGIC0, PV2_ARM_MAGIC1 };
    ble_cmd(PV2_OP_ARM, 1, arm, 2);
    ble_cmd(PV2_OP_MOTOR_TEST, 2, (const uint8_t[5]){ 1, 8, 0xD0, 0x07, 0x50 }, 5);
    run(100);
    const pv2_ack_t *k = ack_in(&B, f0, PV2_OP_ARM, 1);
    CHECK(k && k->result == PV2_ACK_REJECTED && k->reason == PV2_R_USB_CONNECTED, "ARM over BLE with a USB host: USB_CONNECTED");
    k = ack_in(&B, f0, PV2_OP_MOTOR_TEST, 2);
    CHECK(k && k->reason == PV2_R_USB_CONNECTED, "MOTOR_TEST with a USB host: USB_CONNECTED");
    run(600);
    uint8_t ready = 9;
    CHECK(usb_item(last_of(&B, 0, PV2_FT_CHECKS), &ready) == PV2_CK_FAIL && ready == 0, "the app sees CHECKS item 12 in FAIL");
    CHECK(U.n == 0, "nothing on USB: no program has the port open");

    /* cable out: ARM accepted */
    usbl_set_configured(0);
    run(200);
    f0 = B.n;
    ble_cmd(PV2_OP_ARM, 3, arm, 2);
    run(100);
    k = ack_in(&B, f0, PV2_OP_ARM, 3);
    fc_state_t f; memset(&f, 0, sizeof f); fc_snapshot(&f);
    CHECK(k && k->result == PV2_ACK_OK && f.armed, "USB unplugged: ARM OK");
    run(600);
    ready = 9;
    CHECK(usb_item(last_of(&B, 0, PV2_FT_CHECKS), &ready) == PV2_CK_OK, "CHECKS item 12 OK");

    /* plugged in while armed: nothing changes in the air, but no TAKEOFF */
    usbl_set_configured(1);
    run(50);
    f0 = B.n;
    ble_cmd(PV2_OP_TAKEOFF, 4, (const uint8_t[2]){ 100, 0 }, 2);
    ble_cmd(PV2_OP_DISARM, 5, NULL, 0);
    run(100);
    k = ack_in(&B, f0, PV2_OP_TAKEOFF, 4);
    const pv2_ack_t *d = ack_in(&B, f0, PV2_OP_DISARM, 5);
    memset(&f, 0, sizeof f); fc_snapshot(&f);
    CHECK(k && k->reason == PV2_R_USB_CONNECTED && d && d->result == PV2_ACK_OK && !f.armed,
          "armed then USB: TAKEOFF refused, DISARM still works");
}

/* ================================================================== first link has control */
static void test_owner(void)
{
    uint8_t a[6]; const uint16_t id = 1; const float v = 3.0f;
    memcpy(a, &id, 2); memcpy(a + 2, &v, 4);
    const uint8_t get[2] = { 1, 0 };

    /* USB first, then BLE */
    setup();
    usbl_set_configured(1); usbl_set_dtr(1);
    run(200);
    ble_connect();
    run(300);
    int fb = B.n, fu = U.n;
    ble_cmd(PV2_OP_PARAM_SET, 7, a, 6);
    ble_cmd(PV2_OP_PARAM_GET, 8, get, 2);
    usb_cmd(PV2_OP_PARAM_GET, 7, get, 2);                  /* same seq as a BLE command: separate ACK caches */
    run(100);
    const pv2_ack_t *k = ack_in(&B, fb, PV2_OP_PARAM_SET, 7);
    CHECK(k && k->result == PV2_ACK_REJECTED && k->reason == PV2_R_OTHER_LINK, "USB first: PARAM_SET over BLE -> OTHER_LINK");
    k = ack_in(&B, fb, PV2_OP_PARAM_GET, 8);
    CHECK(k && k->result == PV2_ACK_OK, "USB first: PARAM_GET over BLE allowed");
    k = ack_in(&U, fu, PV2_OP_PARAM_GET, 7);
    CHECK(k && k->result == PV2_ACK_OK && !ack_in(&U, fu, PV2_OP_PARAM_SET, 7) && !ack_in(&B, fb, PV2_OP_PARAM_GET, 7),
          "each ACK only on its own link, same seq on both");
    run(300);
    CHECK(count(&B, fb, PV2_FT_FLIGHT) >= 3 && count(&U, fu, PV2_FT_FLIGHT) >= 3, "telemetry on both links");
    fu = U.n;
    usb_cmd(PV2_OP_PARAM_SET, 9, a, 6);
    run(50);
    k = ack_in(&U, fu, PV2_OP_PARAM_SET, 9);
    CHECK(k && k->result == PV2_ACK_OK, "USB first: PARAM_SET over USB OK");

    /* the owner goes away: control passes to the other link */
    usbl_set_configured(0);
    run(100);
    fb = B.n;
    ble_cmd(PV2_OP_PARAM_SET, 10, a, 6);
    run(50);
    k = ack_in(&B, fb, PV2_OP_PARAM_SET, 10);
    CHECK(k && k->result == PV2_ACK_OK, "USB gone: BLE has control");

    /* BLE first, then USB */
    setup();
    ble_connect();
    run(200);
    usbl_set_configured(1); usbl_set_dtr(1);
    run(200);
    fu = U.n; fb = B.n;
    usb_cmd(PV2_OP_PARAM_SET, 1, a, 6);
    usb_cmd(PV2_OP_CAL_START, 2, (const uint8_t[2]){ PV2_CAL_GYRO, 0 }, 2);
    usb_cmd(PV2_OP_REQUEST, 3, (const uint8_t[1]){ PV2_FT_CAPS }, 1);
    ble_cmd(PV2_OP_PARAM_SET, 1, a, 6);
    run(100);
    k = ack_in(&U, fu, PV2_OP_PARAM_SET, 1);
    const pv2_ack_t *c = ack_in(&U, fu, PV2_OP_CAL_START, 2);
    CHECK(k && k->reason == PV2_R_OTHER_LINK && c && c->reason == PV2_R_OTHER_LINK, "BLE first: USB writes -> OTHER_LINK");
    k = ack_in(&U, fu, PV2_OP_REQUEST, 3);
    CHECK(k && k->result == PV2_ACK_OK && count(&U, fu, PV2_FT_CAPS) >= 1, "BLE first: REQUEST over USB allowed");
    k = ack_in(&B, fb, PV2_OP_PARAM_SET, 1);
    CHECK(k && k->result == PV2_ACK_OK, "BLE first: PARAM_SET over BLE OK");
    ble_disconnect();
    run(100);
    fu = U.n;
    usb_cmd(PV2_OP_PARAM_SET, 4, a, 6);
    run(50);
    k = ack_in(&U, fu, PV2_OP_PARAM_SET, 4);
    CHECK(k && k->result == PV2_ACK_OK, "BLE gone: USB has control");

    /* the app subscribes again: only the BLE entries of the ACK cache are forgotten */
    ble_connect();
    run(200);
    fu = U.n; fb = B.n;
    usb_cmd(PV2_OP_PARAM_SET, 4, a, 6);                     /* USB retry of the same command */
    ble_cmd(PV2_OP_PARAM_SET, 1, a, 6);                     /* new BLE session, a seq used in the old one */
    run(100);
    k = ack_in(&U, fu, PV2_OP_PARAM_SET, 4);
    CHECK(k && k->result == PV2_ACK_OK && count(&U, fu, PV2_FT_PARAM) == 0, "BLE re-subscribe: USB retry still answered from its cache, not re-executed");
    k = ack_in(&B, fb, PV2_OP_PARAM_SET, 1);
    CHECK(k && k->result == PV2_ACK_REJECTED && k->reason == PV2_R_OTHER_LINK, "BLE re-subscribe: BLE seq 1 evaluated again (USB has control)");
    CHECK(U.bad == 0 && B.bad == 0, "every frame whole on both links");
}

/* ================================================================== three USB producers */
static void test_interleave(void)
{
    setup();
    usbl_set_configured(1); usbl_set_dtr(1);
    run(100);
    /* the TelemetryTask's frames (STATUS-like, 40 B) at 1 kHz, while the AppTask
     * streams and the LogTask lists the logs */
    s_host_reads = 0;
    int puts = 0;
    for (int k = 0; k < 2000; ++k) {
        uint8_t pl[40], f[64];
        memset(pl, k & 0xFF, sizeof pl);
        const size_t n = tlm_encode(0x03, pl, sizeof pl, f, sizeof f);
        if (usbl_put_frame(USBL_SRC_TLM, f, (uint32_t)n) == 0) puts++;
        if (k == 500) { s_host_reads = 1; usb_cmd(PV2_OP_LOG_LIST, 1, NULL, 0); }
        run(1);
    }
    run(200);
    CHECK(U.bad == 0, "three producers on one endpoint: no frame cut or mixed (%d bad bytes)", U.bad);
    CHECK(count(&U, 0, 0x03) == puts && ack_in(&U, 0, PV2_OP_LOG_LIST, 1) && count(&U, 0, PV2_FT_FLIGHT) > 10,
          "every TLM frame (%d of %d), the log ACK and the AppTask streams arrived", count(&U, 0, 0x03), puts);
    CHECK(usbl_tx_dropped(USBL_SRC_TLM) == (uint32_t)(2000 - puts), "a full ring drops whole frames, counted");
    /* host gone: the rings are emptied */
    usbl_set_configured(0);
    run(50);
    uint8_t b[16];
    CHECK(usbl_tx_peek(b, sizeof b) == 0 && usbl_put_frame(USBL_SRC_TLM, b, 0) == -1, "no host: rings flushed");
}

int main(void)
{
    test_usb_only();
    test_arm_inhibit();
    test_owner();
    test_interleave();
    printf("link: %d passed, %d failed\n", s_pass, s_fail);
    return s_fail ? 1 : 0;
}
