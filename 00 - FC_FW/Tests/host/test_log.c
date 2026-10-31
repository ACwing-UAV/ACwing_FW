/*
 * Host tests of the flight log: W25Q driver against a simulated chip, the
 * sector-ring store (wrap, re-mount, power loss), the record queues, the
 * LogTask recording policy and the USB download protocol.
 * Writes log_sample.bin (one downloaded log) for the PC decoder tests.
 */
#include "w25q.h"
#include "log_store.h"
#include "log_flash_ram.h"
#include "log_queue.h"
#include "log_feed.h"
#include "log_task.h"
#include "usb_link.h"
#include "proto_v2.h"
#include "telemetry.h"
#include "fc_core.h"
#include "app_link.h"
#include "params.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int s_fail, s_pass;
#define CHECK(c, ...) do { if (c) s_pass++; else { s_fail++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

/* ================================================================== simulated W25Q (1 MB) */
#define CHIP_SIZE (1u << 20)
static uint8_t  s_chip[CHIP_SIZE];
static uint8_t  s_wel;
static int      s_busy;               /* status polls left with BUSY = 1 */
static int      s_pp_count, s_se_count, s_cs_errors;

static int chip_xfer(void *ctx, const uint8_t *tx, uint32_t ntx, uint8_t *rx, uint32_t nrx)
{
    (void)ctx;
    const uint8_t c = tx[0];
    if (s_busy && c != 0x05u) { s_cs_errors++; return 0; }        /* chip ignores commands while busy */
    const uint32_t a = ntx >= 4 ? ((uint32_t)tx[1] << 16 | (uint32_t)tx[2] << 8 | tx[3]) % CHIP_SIZE : 0;
    switch (c) {
    case 0x9F: rx[0] = 0xEF; rx[1] = 0x40; rx[2] = 0x14; break;
    case 0xAB: break;
    case 0x05: rx[0] = (uint8_t)((s_busy ? 1u : 0u) | (s_wel ? 2u : 0u)); if (s_busy) s_busy--; break;
    case 0x06: s_wel = 1; break;
    case 0x03: for (uint32_t k = 0; k < nrx; ++k) rx[k] = s_chip[(a + k) % CHIP_SIZE]; break;
    case 0x20:
        if (!s_wel) { s_cs_errors++; break; }
        memset(&s_chip[a & ~0xFFFu], 0xFF, 4096); s_wel = 0; s_busy = 5; s_se_count++; break;
    case 0xC7:
        if (!s_wel) { s_cs_errors++; break; }
        memset(s_chip, 0xFF, sizeof s_chip); s_wel = 0; s_busy = 50; break;
    default: s_cs_errors++; break;
    }
    return 0;
}
static int chip_xfer_w(void *ctx, const uint8_t *tx, uint32_t ntx, const uint8_t *d, uint32_t n)
{
    (void)ctx; (void)ntx;
    if (tx[0] != 0x02 || !s_wel || s_busy) { s_cs_errors++; return 0; }
    const uint32_t a = ((uint32_t)tx[1] << 16 | (uint32_t)tx[2] << 8 | tx[3]) % CHIP_SIZE;
    for (uint32_t k = 0; k < n; ++k) {
        const uint32_t x = (a & ~0xFFu) | ((a + k) & 0xFFu);           /* wraps inside the page */
        s_chip[x] &= d[k];
    }
    s_wel = 0; s_busy = 2; s_pp_count++;
    return 0;
}

static w25q_t      s_w;
static log_flash_t s_fl;

static void chip_setup(void)
{
    memset(s_chip, 0x5A, sizeof s_chip);                             /* not erased: the store must cope */
    const w25q_bus_t b = { chip_xfer, chip_xfer_w, NULL, NULL, NULL };
    CHECK(w25q_init(&s_w, &b) == 0 && s_w.size == CHIP_SIZE, "W25Q JEDEC id, 1 MB part (size %u)", (unsigned)s_w.size);
    s_fl.ctx = &s_w; s_fl.size = s_w.size;
    s_fl.read = w25q_lf_read; s_fl.prog = w25q_lf_prog; s_fl.erase = w25q_lf_erase;
}

/* ================================================================== record parser (as the PC tools) */
typedef struct { int n[256]; int bad; uint32_t last_t; int t_back; uint32_t att_seq_err; } parse_t;

static void parse_log(const uint8_t *b, uint32_t n, parse_t *p)
{
    memset(p, 0, sizeof *p);
    uint32_t last_att = 0;
    for (uint32_t s = 0; s + LOG_SECTOR <= n; s += LOG_SECTOR) {
        log_sector_hdr_t h; memcpy(&h, b + s, sizeof h);
        if (h.magic != LOG_MAGIC || h.crc != log_crc16(b + s, LOG_HDR_SIZE - 2)) { p->bad++; continue; }
        uint32_t i = s + LOG_HDR_SIZE;
        while (i + 3 <= s + LOG_SECTOR && b[i] != 0xFF) {
            const uint8_t t = b[i], len = b[i + 1];
            if (i + 3u + len > s + LOG_SECTOR || log_crc8(b + i, 2u + len) != b[i + 2 + len]) { p->bad++; break; }
            p->n[t]++;
            if (t == LR_ATT) {
                log_att_t a; memcpy(&a, b + i + 2, sizeof a);
                if (a.t_ms < last_att) p->t_back++;
                if (a.gyro[0] != (int16_t)(a.t_ms / 20u % 1000u)) p->att_seq_err++;
                last_att = a.t_ms;
            }
            i += 3u + len;
        }
    }
}

/* ================================================================== store */
static int s_append_err;
static void put_att(uint32_t t)
{
    log_att_t a; memset(&a, 0, sizeof a);
    a.t_ms = t; a.q[0] = 32767; a.gyro[0] = (int16_t)(t / 20u % 1000u);
    if (log_append(LR_ATT, &a, sizeof a) != 0) s_append_err++;
}

static uint8_t s_buf[CHIP_SIZE];

static void test_store(void)
{
    chip_setup();
    CHECK(log_mount(&s_fl) == 0 && log_count() == 0, "mount: garbage flash = no logs");
    CHECK(log_open(1000) == 0 && log_is_open(), "open");
    for (uint32_t k = 0; k < 1000; ++k) put_att(1000 + k * 20);
    CHECK(log_close() == 0 && !log_is_open(), "close");
    CHECK(log_count() == 1 && log_entry(0)->id == 1 && log_entry(0)->nsect == 7 && !log_entry(0)->truncated,
          "one log, 7 sectors (27 B x 1000 = 27 kB) -> %u", log_entry(0)->nsect);
    CHECK(s_cs_errors == 0, "no command sent while the chip was busy (%d)", s_cs_errors);

    const uint32_t sz = log_size(1);
    CHECK(log_read(1, 0, s_buf, sz) == 0, "read back");
    parse_t p; parse_log(s_buf, sz, &p);
    CHECK(p.n[LR_ATT] == 1000 && p.bad == 0 && p.att_seq_err == 0, "1000 records, all CRC ok (%d, bad %d)", p.n[LR_ATT], p.bad);

    /* re-mount finds the same log, the next one gets id 2 */
    CHECK(log_mount(&s_fl) == 0 && log_count() == 1 && log_entry(0)->nsect == 7, "re-mount");
    CHECK(log_open(50000) == 0, "open 2");
    for (uint32_t k = 0; k < 100; ++k) put_att(50000 + k * 20);
    log_close();
    CHECK(log_count() == 2 && log_entry(1)->id == 2, "second log id 2");

    /* power loss: records in the RAM page are lost, the rest is readable */
    CHECK(log_open(60000) == 0, "open 3");
    for (uint32_t k = 0; k < 300; ++k) put_att(60000 + k * 20);
    CHECK(log_mount(&s_fl) == 0 && log_count() == 3, "re-mount after power loss: 3 logs");
    log_read(3, 0, s_buf, log_size(3));
    parse_log(s_buf, log_size(3), &p);
    /* the record cut at the last programmed page fails its CRC: the reader skips it */
    CHECK(p.n[LR_ATT] >= 300 - 10 && p.n[LR_ATT] < 300 && p.bad <= 1, "lost at most one page (%d of 300, bad %d)", p.n[LR_ATT], p.bad);

    /* wrap: fill the 1 MB ring 1.5 times with 10-sector logs */
    for (int L = 0; L < 40; ++L) {
        if (log_open(100000u + (uint32_t)L) != 0) s_append_err++;
        for (uint32_t k = 0; k < 1450; ++k) put_att(k * 20);
        log_close();
    }
    CHECK(log_used_pct() == 100, "ring full: 100 %% (%u)", log_used_pct());
    const uint8_t n1 = log_count();
    const log_entry_t first = *log_entry(0), last = *log_entry(n1 - 1u);
    CHECK(last.id == 43 && last.nsect == 10, "newest log id 43 (%u)", last.id);
    CHECK(first.truncated || first.nsect == 10, "oldest log whole or truncated");
    uint32_t total = 0;
    for (uint8_t i = 0; i < n1; ++i) total += log_entry(i)->nsect;
    CHECK(total == CHIP_SIZE / LOG_SECTOR, "every sector belongs to a log (%u)", (unsigned)total);
    CHECK(log_mount(&s_fl) == 0 && log_count() == n1 && log_entry(0)->id == first.id &&
          log_entry(0)->nsect == first.nsect && log_entry(0)->truncated == first.truncated,
          "re-mount after wrap gives the same list");
    log_read(last.id, 0, s_buf, log_size(last.id));
    parse_log(s_buf, log_size(last.id), &p);
    CHECK(p.n[LR_ATT] == 1450 && p.bad == 0, "newest log intact after wrap");
    if (first.truncated) {
        log_read(first.id, 0, s_buf, log_size(first.id));
        parse_log(s_buf, log_size(first.id), &p);
        CHECK(p.bad == 0 && p.n[LR_ATT] > 0, "truncated log still decodes");
    }
    CHECK(s_cs_errors == 0 && s_append_err == 0, "no bus errors (%d), no append errors (%d)", s_cs_errors, s_append_err);

    CHECK(log_erase_all() == 0 && log_count() == 0 && log_used_pct() == 0, "erase all");
    CHECK(log_mount(&s_fl) == 0 && log_count() == 0, "empty after erase and re-mount");
    CHECK(log_open(1) == 0 && log_entry(0)->id == 1, "ids restart at 1");
    log_close();
}

/* ================================================================== queues */
static void test_queue(void)
{
    log_q_reset();
    uint8_t p[40], t;
    memset(p, 7, sizeof p);
    CHECK(log_q_put(LOG_SRC_BATT, LR_POWER, p, 19) == 0, "put");
    int ok = 0;
    for (int k = 0; k < 100; ++k) ok += (log_q_put(LOG_SRC_BATT, LR_POWER, p, 19) == 0);
    CHECK(ok == 511 / 21 - 1 && log_q_take_drops(LOG_SRC_BATT) == 100 - ok, "full ring drops whole records (%d)", ok);
    uint32_t r = log_q_get(LOG_SRC_BATT, &t, p, sizeof p);
    CHECK(r == 20 && t == LR_POWER && p[18] == 7, "get");
    CHECK(log_q_take_drops(LOG_SRC_BATT) == 0, "drops counted once");
}

/* ================================================================== LogTask + USB */
static uint8_t     s_ram[48 * 1024];
static log_flash_t s_rf;
static uint32_t    now;
static uint32_t    clock_ms(void) { return now; }

typedef struct { uint8_t type, len; uint8_t p[255]; } frm_t;
static frm_t s_f[8192];
static int   s_nf;
static uint8_t s_usb_acc[16384];
static size_t  s_usb_na;

static void pump(void)
{
    uint8_t b[512];
    uint32_t n;
    while ((n = usbl_tx_peek(b, sizeof b)) != 0) {
        usbl_tx_consume(n);
        memcpy(s_usb_acc + s_usb_na, b, n); s_usb_na += n;
    }
    size_t i = 0;
    while (s_usb_na - i >= 6) {
        if (s_usb_acc[i] != 0xA5 || s_usb_acc[i + 1] != 0x5A) { i++; continue; }
        const uint8_t len = s_usb_acc[i + 3];
        if (s_usb_na - i < (size_t)len + 6u) break;
        const uint16_t crc = tlm_crc16(&s_usb_acc[i + 2], len + 2u);
        if (crc == (uint16_t)(s_usb_acc[i + 4 + len] | s_usb_acc[i + 5 + len] << 8) && s_nf < 8192) {
            s_f[s_nf].type = s_usb_acc[i + 2]; s_f[s_nf].len = len;
            memcpy(s_f[s_nf].p, &s_usb_acc[i + 4], len); s_nf++;
        }
        i += len + 6u;
    }
    memmove(s_usb_acc, s_usb_acc + i, s_usb_na - i); s_usb_na -= i;
}

static void usb_cmd(uint8_t op, uint8_t seq, const void *a, uint8_t n)
{
    uint8_t pl[32], f[64];
    pl[0] = op; pl[1] = seq;
    if (n) memcpy(pl + 2, a, n);
    const size_t k = pv2_frame(PV2_FT_CMD, pl, (uint8_t)(n + 2), f, sizeof f);
    /* split in two USB packets on purpose */
    usbl_rx_isr(f, (uint32_t)k / 2);
    usbl_rx_isr(f + k / 2, (uint32_t)(k - k / 2));
}

static const pv2_ack_t *ack_of(int from, uint8_t op, uint8_t seq, uint8_t res_wanted)
{
    for (int i = from; i < s_nf; ++i)
        if (s_f[i].type == PV2_FT_ACK) {
            const pv2_ack_t *a = (const pv2_ack_t *)s_f[i].p;
            if (a->op == op && a->seq == seq && (res_wanted == 0xFF || a->result == res_wanted)) return a;
        }
    return NULL;
}

static void fc_tick(int armed_wanted)
{
    static uint8_t armed;
    if (armed_wanted && !armed) {
        fc_cmd_t c; memset(&c, 0, sizeof c);
        c.op = PV2_OP_ARM; c.seq = 1; c.len = 2; c.p[0] = PV2_ARM_MAGIC0; c.p[1] = PV2_ARM_MAGIC1;
        fc_post(&c); armed = 1;
    } else if (!armed_wanted && armed) {
        fc_cmd_t c; memset(&c, 0, sizeof c);
        c.op = PV2_OP_DISARM; c.seq = 2;
        fc_post(&c); armed = 0;
    }
    uint8_t hb[2] = { PV2_OP_HEARTBEAT, 0 };
    if (now % 200u == 0u) appl_on_cmd_write(hb, 2, now);
    fc_vert_t v = { 1, 1.0f, 0, 0, 0, 0, 1.0f, 0xFF, 0 };
    fc_service(now, 1, &v);
    fc_res_t r; while (fc_take_result(&r)) {}
    fc_event_t e; while (fc_take_event(&e)) {}
}

/* the ImuTask producer at 50 Hz, the BattTask at 10 Hz */
static void produce(void)
{
    if (now % 20u == 0u) {
        const float q[4] = { 1, 0, 0, 0 };
        const float w[3] = { (float)(now / 20u % 1000u) / 1000.0f, 0, 0 };
        const float a[3] = { 0, 0, 1 };
        logf_att(now, q, w, a);
        fc_vert_t v = { 1, 1.0f, 0, 0, 0, 0, 1.0f, 0xFF, 0 };
        fc_setpoint_t sp; memset(&sp, 0, sizeof sp);
        logf_vert(now, &v, &sp, 0xFFFF, 101000);
    }
    if (now % 100u == 0u) {
        fc_state_t s; memset(&s, 0, sizeof s); fc_snapshot(&s);
        logf_flight(now, &s);
        batt_state_t b; memset(&b, 0, sizeof b); b.vbat = 3.9f; b.present = 1; b.cells = 1;
        logf_power(now, &b);
    }
}

static void run(int ms, int armed, int log_step_every)
{
    for (int k = 0; k < ms; ++k) {
        now++;
        if (now % 4u == 0u) fc_tick(armed);
        produce();
        if (now % (uint32_t)log_step_every == 0u) log_task_step(now);
    }
}

static void test_task(void)
{
    memset(s_ram, 0x00, sizeof s_ram);
    log_flash_ram_init(&s_rf, s_ram, sizeof s_ram);
    appl_reset();
    params_defaults();
    const fc_config_t fcc = { PV2_S_IMU, 4, 0, NULL, NULL, logf_event, 0 };
    fc_init(&fcc);
    fc_set_ground_info(1, 0, 0);
    logf_set_clock(clock_ms);
    const log_task_cfg_t lc = { &s_rf, 'N', { 0, 5, 0 }, NULL, pump, NULL };
    log_task_init(&lc);
    CHECK(log_task_pct() == 0, "RAM log mounted, 0 %%");
    now = 10000;

    /* logMode 1: nothing while disarmed */
    run(2000, 0, 10);
    CHECK(log_count() == 0, "disarmed: no log");
    /* a 6 s flight */
    run(6000, 1, 10);
    CHECK(log_count() == 1 && log_is_open(), "armed: log open");
    run(3000, 0, 10);
    CHECK(log_count() == 1 && !log_is_open(), "closed after the post-roll");

    /* USB: list */
    int f0 = s_nf;
    usb_cmd(PV2_OP_LOG_LIST, 1, NULL, 0);
    run(20, 0, 10);
    CHECK(ack_of(f0, PV2_OP_LOG_LIST, 1, PV2_ACK_OK) != NULL, "LOG_LIST ack");
    pv2_log_entry_t e; memset(&e, 0, sizeof e);
    int ne = 0;
    for (int i = f0; i < s_nf; ++i) if (s_f[i].type == PV2_FT_LOG_ENTRY) { memcpy(&e, s_f[i].p, sizeof e); ne++; }
    CHECK(ne == 1 && e.id == 1 && e.count == 1 && e.flags == 0 && e.bytes == e.sectors * 4096u && e.t0_ms >= 12000,
          "LOG_ENTRY id %u sectors %u t0 %u", e.id, e.sectors, (unsigned)e.t0_ms);

    /* USB: read the whole log in one command */
    f0 = s_nf;
    uint8_t a[10]; const uint16_t id = 1; const uint32_t off = 0, len = 0;
    memcpy(a, &id, 2); memcpy(a + 2, &off, 4); memcpy(a + 6, &len, 4);
    usb_cmd(PV2_OP_LOG_READ, 2, a, 10);
    run(2000, 0, 1);
    static uint8_t img[48 * 1024];
    uint32_t got = 0, holes = 0;
    for (int i = f0; i < s_nf; ++i) if (s_f[i].type == PV2_FT_LOG_DATA) {
        pv2_log_data_hdr_t h; memcpy(&h, s_f[i].p, sizeof h);
        const uint32_t k = s_f[i].len - sizeof h;
        if (h.offset != got) holes++;
        memcpy(img + h.offset, s_f[i].p + sizeof h, k);
        got = h.offset + k;
    }
    const pv2_ack_t *ak = ack_of(f0, PV2_OP_LOG_READ, 2, 0xFF);
    CHECK(ak && ak->result == PV2_ACK_OK && got == e.bytes && holes == 0, "LOG_READ: %u of %u bytes, ACK after the data",
          (unsigned)got, (unsigned)e.bytes);
    parse_t p; parse_log(img, got, &p);
    CHECK(p.bad == 0 && p.n[LR_INFO] == 1, "decodes, one LR_INFO");
    CHECK(p.n[LR_ATT] >= 345 && p.n[LR_ATT] <= 360 && p.n[LR_VERT] == p.n[LR_ATT], "50 Hz for ~7 s (%d ATT)", p.n[LR_ATT]);
    CHECK(p.n[LR_FLIGHT] >= 68 && p.n[LR_POWER] >= 68 && p.n[LR_DROP] == 0, "10 Hz FLIGHT/POWER, no drops (%d, %d)",
          p.n[LR_FLIGHT], p.n[LR_POWER]);
    CHECK(p.n[LR_EVENT] == 2 && p.t_back == 0, "ARMED and DISARMED events, time monotonic (ev %d)", p.n[LR_EVENT]);
    {
        FILE *f = fopen("log_sample.bin", "wb");
        if (f) { fwrite(img, 1, got, f); fclose(f); }
    }

    /* partial read with offset */
    f0 = s_nf;
    const uint32_t off2 = 4100, len2 = 1000;
    memcpy(a + 2, &off2, 4); memcpy(a + 6, &len2, 4);
    usb_cmd(PV2_OP_LOG_READ, 3, a, 10);
    run(50, 0, 1);
    uint32_t sum = 0, first_off = 0xFFFFFFFF;
    for (int i = f0; i < s_nf; ++i) if (s_f[i].type == PV2_FT_LOG_DATA) {
        pv2_log_data_hdr_t h; memcpy(&h, s_f[i].p, sizeof h);
        if (first_off == 0xFFFFFFFF) first_off = h.offset;
        sum += s_f[i].len - sizeof h;
        if (memcmp(img + h.offset, s_f[i].p + sizeof h, s_f[i].len - sizeof h)) sum = 0xDEAD;
    }
    CHECK(first_off == 4100 && sum == 1000 && ack_of(f0, PV2_OP_LOG_READ, 3, PV2_ACK_OK), "offset read");

    /* bad id, garbage on the line, unknown command */
    f0 = s_nf;
    const uint16_t bad = 99; memcpy(a, &bad, 2);
    usb_cmd(PV2_OP_LOG_READ, 4, a, 10);
    { const uint8_t junk[7] = { 0xA5, 0x5A, 0x20, 0x02, 0x50, 0x09, 0x00 }; usbl_rx_isr(junk, 7); }   /* bad CRC */
    usb_cmd(PV2_OP_ARM, 5, NULL, 0);
    run(50, 0, 1);
    const pv2_ack_t *x = ack_of(f0, PV2_OP_LOG_READ, 4, 0xFF);
    CHECK(x && x->result == PV2_ACK_INVALID && x->reason == PV2_R_OUT_OF_RANGE, "unknown id -> INVALID/OUT_OF_RANGE");
    x = ack_of(f0, PV2_OP_ARM, 5, 0xFF);
    CHECK(x && x->result == PV2_ACK_UNSUPPORTED && usbl_rx_errors() == 1, "only log commands over USB; bad CRC dropped");

    /* recording pauses while the PC talks: logMode always + USB traffic */
    { uint8_t why; params_set(70, 2.0f, &why); }
    run(1000, 0, 10);
    CHECK(log_count() == 1, "always mode, USB active: paused");
    run(6000, 0, 10);
    CHECK(log_count() == 2 && log_is_open(), "5 s after the last command: recording again");

    /* armed: no download, no erase */
    { uint8_t why; params_set(70, 1.0f, &why); }
    run(100, 1, 10);
    f0 = s_nf;
    memcpy(a, &id, 2);
    usb_cmd(PV2_OP_LOG_READ, 6, a, 10);
    usb_cmd(PV2_OP_LOG_ERASE, 7, "WIPE", 4);
    run(50, 1, 10);
    x = ack_of(f0, PV2_OP_LOG_READ, 6, 0xFF);
    const pv2_ack_t *y = ack_of(f0, PV2_OP_LOG_ERASE, 7, 0xFF);
    CHECK(x && x->reason == PV2_R_ARMED && y && y->reason == PV2_R_ARMED, "armed: READ and ERASE refused");
    run(3000, 0, 10);

    /* erase */
    f0 = s_nf;
    usb_cmd(PV2_OP_LOG_ERASE, 8, "WIPF", 4);
    usb_cmd(PV2_OP_LOG_ERASE, 9, "WIPE", 4);
    run(50, 0, 10);
    CHECK(ack_of(f0, PV2_OP_LOG_ERASE, 8, PV2_ACK_INVALID) && ack_of(f0, PV2_OP_LOG_ERASE, 9, PV2_ACK_IN_PROGRESS) &&
          ack_of(f0, PV2_OP_LOG_ERASE, 9, PV2_ACK_OK), "ERASE: magic, IN_PROGRESS then OK");
    CHECK(log_count() == 0 && log_task_pct() == 0, "empty after ERASE");

    /* no log flash */
    const log_task_cfg_t none = { NULL, 'N', { 0, 5, 0 }, NULL, pump, NULL };
    log_task_init(&none);
    f0 = s_nf;
    usb_cmd(PV2_OP_LOG_LIST, 10, NULL, 0);
    run(20, 0, 10);
    x = ack_of(f0, PV2_OP_LOG_LIST, 10, 0xFF);
    CHECK(log_task_pct() == 0xFF && x && x->reason == PV2_R_SENSOR_MISSING, "no flash: pct 0xFF, SENSOR_MISSING");
}

int main(void)
{
    test_store();
    test_queue();
    test_task();
    printf("log: %d passed, %d failed\n", s_pass, s_fail);
    return s_fail ? 1 : 0;
}
