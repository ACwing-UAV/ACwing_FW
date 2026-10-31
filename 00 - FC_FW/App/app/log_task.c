/**
 * @file  log_task.c
 * @brief LogTask (see log_task.h).
 */
#include "log_task.h"
#include "log_queue.h"
#include "usb_link.h"
#include "proto_v2.h"
#include "fc_core.h"
#include "params.h"
#include <string.h>

static log_task_cfg_t   s_cfg;
static uint8_t          s_mounted;
static volatile uint8_t s_pct = 0xFFu;
static uint8_t          s_was_armed;
static uint32_t         s_disarm_ms, s_usb_ms;
static uint8_t          s_have_usb;

/* events seen while no log was open (e.g. ARMED just before the log opens) */
#define PRE_EVENTS 8u
#define PRE_EVENT_MS 2000u
static log_event_t s_pre[PRE_EVENTS];
static uint8_t     s_npre;

/* LOG_READ in progress */
static struct {
    uint8_t  on, seq;
    uint16_t id;
    uint32_t off, end;
} s_rd;

void log_task_init(const log_task_cfg_t *cfg)
{
    s_cfg = *cfg;
    log_q_reset();
    usbl_reset();
    s_mounted = (uint8_t)(s_cfg.flash && log_mount(s_cfg.flash) == 0);
    s_pct = s_mounted ? log_used_pct() : 0xFFu;
    s_was_armed = 0u; s_disarm_ms = 0u; s_have_usb = 0u; s_npre = 0u;
    memset(&s_rd, 0, sizeof s_rd);
}

uint8_t log_task_pct(void)  { return s_pct; }
uint8_t log_task_busy(void) { return s_rd.on; }

static void pump(void) { if (s_cfg.usb_pump) s_cfg.usb_pump(); }

static void ack(uint8_t op, uint8_t seq, uint8_t res, uint8_t why)
{
    const pv2_ack_t a = { op, seq, res, why };
    (void)usbl_send(PV2_FT_ACK, &a, sizeof a);
    pump();
}

/* ------------------------------------------------------------------ recording */
static void close_log(void)
{
    if (!log_is_open()) return;
    (void)log_close();
}

static void open_log(uint32_t now)
{
    if (log_open(now) != 0) return;
    log_info_t in;
    memset(&in, 0, sizeof in);
    memcpy(in.fw, s_cfg.fw, 3);
    in.hw_rev = s_cfg.hw_rev;
    in.format = LOG_FORMAT;
    in.t0_ms = now;
    in.sensors = s_cfg.sensors ? s_cfg.sensors() : 0u;
    (void)log_append(LR_INFO, &in, sizeof in);
    for (uint8_t i = 0; i < s_npre; ++i)
        if ((now - s_pre[i].t_ms) < PRE_EVENT_MS) (void)log_append(LR_EVENT, &s_pre[i], sizeof s_pre[i]);
    s_npre = 0;
}

static void drain(uint32_t now)
{
    uint8_t buf[LOG_REC_MAX], type;
    for (uint8_t src = 0; src < LOG_SRC_COUNT; ++src) {
        uint32_t r;
        while ((r = log_q_get(src, &type, buf, sizeof buf)) != 0u) {
            if (log_is_open()) (void)log_append(type, buf, (uint8_t)(r - 1u));
            else if (type == LR_EVENT && r - 1u == sizeof(log_event_t)) {
                if (s_npre == PRE_EVENTS) { memmove(&s_pre[0], &s_pre[1], sizeof s_pre - sizeof s_pre[0]); s_npre--; }
                memcpy(&s_pre[s_npre++], buf, sizeof(log_event_t));
            }
        }
        const uint16_t lost = log_q_take_drops(src);
        if (lost && log_is_open()) {
            const log_drop_t d = { now, lost, src };
            (void)log_append(LR_DROP, &d, sizeof d);
        }
    }
}

static void recording(uint32_t now, uint8_t armed)
{
    if (!armed && s_was_armed) s_disarm_ms = now;
    s_was_armed = armed;
    const int mode = (int)(PARAM(P_LOG_MODE) + 0.5f);
    const uint8_t usb = (uint8_t)(s_have_usb && (now - s_usb_ms) < LOG_USB_PAUSE_MS && !armed);
    uint8_t want = 0u;
    if (mode == PARAM_LOG_ALWAYS) want = 1u;
    else if (mode == PARAM_LOG_ARMED) want = (uint8_t)(armed || (s_disarm_ms && (now - s_disarm_ms) < LOG_POSTROLL_MS));
    if (usb) want = 0u;
    /* producers queue whenever recording is possible: records that arrive
     * before the log opens are dropped, except the events */
    log_q_set_active((uint8_t)(mode != PARAM_LOG_OFF && !usb));

    if (want && !log_is_open()) open_log(now);
    drain(now);
    if (!want && log_is_open()) close_log();
}

/* ------------------------------------------------------------------ USB commands */
static void cmd_list(uint8_t seq)
{
    const uint8_t n = log_count();
    for (uint8_t i = 0; i < n; ++i) {
        const log_entry_t *e = log_entry(i);
        pv2_log_entry_t m;
        m.id = e->id; m.index = i; m.count = n;
        m.flags = (uint8_t)((e->truncated ? 1u : 0u) | (e->active ? 2u : 0u));
        m.sectors = e->nsect; m.t0_ms = e->t0_ms;
        m.bytes = (uint32_t)e->nsect * LOG_SECTOR;
        if (usbl_send(PV2_FT_LOG_ENTRY, &m, sizeof m) != 0) {
            pump();                                       /* frees space if the host reads */
            (void)usbl_send(PV2_FT_LOG_ENTRY, &m, sizeof m);
        }
    }
    ack(PV2_OP_LOG_LIST, seq, PV2_ACK_OK, PV2_R_NONE);        /* after the entries: the list is complete */
}

static void cmd(const uint8_t *c, uint32_t n, uint8_t armed)
{
    const uint8_t op = c[0], seq = c[1];
    const uint8_t *p = &c[2];
    const uint32_t pl = n - 2u;
    if (op == PV2_OP_HEARTBEAT) return;
    switch (op) {
    case PV2_OP_LOG_LIST:
        if (!s_mounted) { ack(op, seq, PV2_ACK_REJECTED, PV2_R_SENSOR_MISSING); return; }
        cmd_list(seq);
        return;
    case PV2_OP_LOG_READ: {
        if (!s_mounted) { ack(op, seq, PV2_ACK_REJECTED, PV2_R_SENSOR_MISSING); return; }
        if (armed)      { ack(op, seq, PV2_ACK_REJECTED, PV2_R_ARMED); return; }
        if (s_rd.on)    { ack(op, seq, PV2_ACK_BUSY, PV2_R_NONE); return; }
        if (pl < 10u)   { ack(op, seq, PV2_ACK_INVALID, PV2_R_NONE); return; }
        uint16_t id; uint32_t off, len;
        memcpy(&id, p, 2); memcpy(&off, p + 2, 4); memcpy(&len, p + 6, 4);
        close_log();                                     /* the newest log becomes readable */
        const uint32_t size = log_size(id);
        if (!size || off > size) { ack(op, seq, PV2_ACK_INVALID, PV2_R_OUT_OF_RANGE); return; }
        if (len == 0u || len > size - off) len = size - off;
        s_rd.on = 1u; s_rd.seq = seq; s_rd.id = id; s_rd.off = off; s_rd.end = off + len;
        return;
    }
    case PV2_OP_LOG_ERASE:
        if (!s_mounted) { ack(op, seq, PV2_ACK_REJECTED, PV2_R_SENSOR_MISSING); return; }
        if (armed)      { ack(op, seq, PV2_ACK_REJECTED, PV2_R_ARMED); return; }
        if (pl < 4u || memcmp(p, "WIPE", 4) != 0) { ack(op, seq, PV2_ACK_INVALID, PV2_R_NONE); return; }
        close_log();
        s_rd.on = 0u;
        ack(op, seq, PV2_ACK_IN_PROGRESS, PV2_R_NONE);
        {
            int r;
            if (s_cfg.erase_chip && log_count() > 0u) {
                r = s_cfg.erase_chip();
                if (r == 0) r = log_mount(s_cfg.flash);    /* rebuild the (empty) list */
            } else r = log_erase_all();
            ack(op, seq, r == 0 ? PV2_ACK_OK : PV2_ACK_REJECTED, PV2_R_NONE);
        }
        return;
    default:
        ack(op, seq, PV2_ACK_UNSUPPORTED, PV2_R_NONE);
        return;
    }
}

static void read_stream(void)
{
    if (!s_rd.on) return;
    uint8_t f[sizeof(pv2_log_data_hdr_t) + PV2_LOG_DATA_MAX];
    while (s_rd.off < s_rd.end && usbl_tx_free() >= sizeof f + 6u + 16u) {
        uint32_t k = s_rd.end - s_rd.off;
        if (k > PV2_LOG_DATA_MAX) k = PV2_LOG_DATA_MAX;
        pv2_log_data_hdr_t h = { s_rd.id, s_rd.off };
        memcpy(f, &h, sizeof h);
        if (log_read(s_rd.id, s_rd.off, f + sizeof h, k) != 0) {
            s_rd.on = 0u;
            ack(PV2_OP_LOG_READ, s_rd.seq, PV2_ACK_REJECTED, PV2_R_NONE);
            return;
        }
        (void)usbl_send(PV2_FT_LOG_DATA, f, (uint8_t)(sizeof h + k));
        s_rd.off += k;
    }
    pump();
    if (s_rd.off >= s_rd.end && usbl_tx_free() >= 16u) {
        s_rd.on = 0u;
        ack(PV2_OP_LOG_READ, s_rd.seq, PV2_ACK_OK, PV2_R_NONE);
    }
}

/* ------------------------------------------------------------------ step */
void log_task_step(uint32_t now)
{
    fc_state_t f;
    memset(&f, 0, sizeof f);
    fc_snapshot(&f);

    uint8_t c[USBL_CMD_MAX];
    uint32_t n;
    while ((n = usbl_poll(c, sizeof c)) != 0u) {
        s_usb_ms = now; s_have_usb = 1u;
        cmd(c, n, f.armed);
    }
    read_stream();

    if (s_mounted) {
        recording(now, f.armed);
        s_pct = log_used_pct();
    } else {
        /* no log flash: empty the queues anyway */
        uint8_t buf[LOG_REC_MAX], type;
        for (uint8_t src = 0; src < LOG_SRC_COUNT; ++src) {
            while (log_q_get(src, &type, buf, sizeof buf)) {}
            (void)log_q_take_drops(src);
        }
    }
    pump();
}

/* ------------------------------------------------------------------ RTOS shell */
#ifndef HOST_TEST
#include "cmsis_os2.h"

static void LogTask(void *arg)
{
    (void)arg;
    for (;;) {
        log_task_step(osKernelGetTickCount());
        osDelay(log_task_busy() ? 1u : 10u);
    }
}

void log_task_start(const log_task_cfg_t *cfg)
{
    log_task_init(cfg);
    /* lowest of the application tasks: erases and page programs may block it */
    static const osThreadAttr_t a = { .name = "log", .priority = osPriorityLow,
                                      .stack_size = 1536 };
    (void)osThreadNew(LogTask, NULL, &a);
}
#endif
