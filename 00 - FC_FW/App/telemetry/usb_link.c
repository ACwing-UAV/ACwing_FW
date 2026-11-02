/**
 * @file  usb_link.c
 * @brief USB byte-stream link (see usb_link.h).
 */
#include "usb_link.h"
#include "proto_v2.h"
#include "telemetry.h"
#include "spsc.h"
#include <string.h>

static uint8_t           s_rx[USBL_RX_SIZE];
static volatile uint32_t s_rx_h, s_rx_t;

typedef struct {
    uint8_t           b[USBL_TX_SIZE];
    volatile uint32_t h, t;
    uint32_t          dropped;
} ring_t;
static ring_t s_tx[USBL_SRC_COUNT];

/* consumer: ring being drained and the bytes left of the snapshot taken when it
 * was chosen (whole frames only: producers publish whole frames) */
static uint8_t  s_cur;
static uint32_t s_cur_left;

/* host state */
static volatile uint8_t  s_cfgd, s_dtr, s_ever;
static volatile uint32_t s_cmd_ms;

/* parser */
static uint8_t  s_acc[4 + 255 + 2];
static uint32_t s_na;
static uint32_t s_err;

void usbl_reset(void)
{
    s_rx_h = s_rx_t = 0;
    memset(s_tx, 0, sizeof s_tx);
    s_cur = 0; s_cur_left = 0;
    s_na = 0; s_err = 0;
    s_cfgd = s_dtr = s_ever = 0; s_cmd_ms = 0;
}

/* ------------------------------------------------------------------ host state */
void usbl_set_configured(uint8_t on) { s_cfgd = on ? 1u : 0u; if (!on) s_dtr = 0u; }
void usbl_set_dtr(uint8_t on)        { s_dtr = on ? 1u : 0u; }
uint8_t usbl_host(void)              { return s_cfgd; }

uint8_t usbl_open(uint32_t now)
{
    if (!s_cfgd) return 0u;
    if (s_dtr) return 1u;
    return (uint8_t)(s_ever && (now - s_cmd_ms) < USBL_OPEN_MS);
}

/* ------------------------------------------------------------------ RX */
void usbl_rx_isr(const uint8_t *p, uint32_t n)
{
    uint32_t h = s_rx_h;
    const uint32_t t = s_rx_t;
    for (uint32_t k = 0; k < n; ++k) {
        if (h - t >= USBL_RX_SIZE) break;
        s_rx[h % USBL_RX_SIZE] = p[k];
        h++;
    }
    SPSC_BARRIER();
    s_rx_h = h;
}

static int rx_byte(uint8_t *b)
{
    const uint32_t t = s_rx_t;
    if (t == s_rx_h) return 0;
    SPSC_BARRIER();
    *b = s_rx[t % USBL_RX_SIZE];
    s_rx_t = t + 1u;
    return 1;
}

uint32_t usbl_rx_errors(void) { return s_err; }

/* drops the first byte of s_acc and moves to the next possible "A5 5A" */
static void resync(void)
{
    uint32_t j = 1;
    while (j < s_na) {
        if (s_acc[j] == 0xA5u && (j + 1u >= s_na || s_acc[j + 1u] == 0x5Au)) break;
        j++;
    }
    memmove(s_acc, &s_acc[j], s_na - j);
    s_na -= j;
}

/* a complete frame at the start of s_acc: 1 valid, 0 need more bytes, -1 bad (resynced) */
static int frame_ready(void)
{
    if (s_na < 4u) return 0;
    const uint32_t len = s_acc[3];
    if (s_na < len + 6u) return 0;
    const uint16_t crc = tlm_crc16(&s_acc[2], len + 2u);
    if ((uint16_t)(s_acc[4u + len] | (s_acc[5u + len] << 8)) != crc) { s_err++; resync(); return -1; }
    return 1;
}

uint32_t usbl_poll(uint8_t *cmd, uint32_t cap, uint32_t now)
{
    for (;;) {
        /* bytes left over after a resync may already hold the next frame */
        int r;
        while ((r = frame_ready()) == -1) {}
        if (r == 1) {
            const uint32_t len = s_acc[3];
            const uint8_t type = s_acc[2];
            uint32_t n = 0;
            if (type == PV2_FT_CMD && len >= 2u) {                 /* only commands come this way */
                n = len > cap ? cap : len;
                memcpy(cmd, &s_acc[4], n);
            }
            memmove(s_acc, &s_acc[len + 6u], s_na - (len + 6u));
            s_na -= len + 6u;
            if (n) { s_cmd_ms = now; s_ever = 1u; return n; }
            continue;
        }
        uint8_t b;
        if (!rx_byte(&b)) return 0;
        /* resynchronise on A5 5A */
        if (s_na == 0u && b != 0xA5u) continue;
        if (s_na == 1u && b != 0x5Au) { s_na = (b == 0xA5u) ? 1u : 0u; continue; }
        s_acc[s_na++] = b;
    }
}

/* ------------------------------------------------------------------ TX producers */
uint32_t usbl_tx_free_src(uint8_t src)
{
    if (src >= USBL_SRC_COUNT) return 0u;
    return USBL_TX_SIZE - (s_tx[src].h - s_tx[src].t);
}

uint32_t usbl_tx_dropped(uint8_t src) { return src < USBL_SRC_COUNT ? s_tx[src].dropped : 0u; }

int usbl_put_frame(uint8_t src, const uint8_t *f, uint32_t n)
{
    if (src >= USBL_SRC_COUNT || !n) return -1;
    ring_t *r = &s_tx[src];
    if (usbl_tx_free_src(src) < n) { r->dropped++; return -1; }
    const uint32_t h = r->h;
    for (uint32_t k = 0; k < n; ++k) r->b[(h + k) % USBL_TX_SIZE] = f[k];
    SPSC_BARRIER();
    r->h = h + n;                                 /* published whole */
    return 0;
}

int usbl_send_src(uint8_t src, uint8_t type, const void *payload, uint8_t len)
{
    uint8_t f[4 + 255 + 2];
    const size_t n = pv2_frame(type, payload, len, f, sizeof f);
    if (!n) return -1;
    return usbl_put_frame(src, f, (uint32_t)n);
}

/* ------------------------------------------------------------------ TX consumer */
/* picks the next ring with data when the current snapshot is used up */
static void choose(void)
{
    if (s_cur_left) return;
    for (uint8_t k = 1; k <= USBL_SRC_COUNT; ++k) {
        const uint8_t c = (uint8_t)((s_cur + k) % USBL_SRC_COUNT);
        const uint32_t n = s_tx[c].h - s_tx[c].t;
        if (n) { SPSC_BARRIER(); s_cur = c; s_cur_left = n; return; }
    }
}

uint32_t usbl_tx_peek(uint8_t *buf, uint32_t cap)
{
    choose();
    if (!s_cur_left) return 0u;
    const ring_t *r = &s_tx[s_cur];
    uint32_t n = s_cur_left;
    if (n > cap) n = cap;
    for (uint32_t k = 0; k < n; ++k) buf[k] = r->b[(r->t + k) % USBL_TX_SIZE];
    return n;
}

void usbl_tx_consume(uint32_t n)
{
    if (n > s_cur_left) n = s_cur_left;
    s_tx[s_cur].t += n;
    s_cur_left -= n;
}

void usbl_tx_flush(void)
{
    for (uint8_t c = 0; c < USBL_SRC_COUNT; ++c) s_tx[c].t = s_tx[c].h;
    s_cur_left = 0u;
}
