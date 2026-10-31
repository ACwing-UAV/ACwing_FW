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
static uint8_t           s_tx[USBL_TX_SIZE];
static volatile uint32_t s_tx_h, s_tx_t;

/* parser */
static uint8_t  s_acc[4 + 255 + 2];
static uint32_t s_na;
static uint32_t s_err;

void usbl_reset(void)
{
    s_rx_h = s_rx_t = 0; s_tx_h = s_tx_t = 0; s_na = 0; s_err = 0;
}

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

uint32_t usbl_poll(uint8_t *cmd, uint32_t cap)
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
            if (n) return n;
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

uint32_t usbl_tx_free(void) { return USBL_TX_SIZE - (s_tx_h - s_tx_t); }

int usbl_send(uint8_t type, const void *payload, uint8_t len)
{
    uint8_t f[4 + 255 + 2];
    const size_t n = pv2_frame(type, payload, len, f, sizeof f);
    if (!n || usbl_tx_free() < n) return -1;
    uint32_t h = s_tx_h;
    for (size_t k = 0; k < n; ++k) s_tx[(h + k) % USBL_TX_SIZE] = f[k];
    SPSC_BARRIER();
    s_tx_h = h + (uint32_t)n;
    return 0;
}

uint32_t usbl_tx_peek(uint8_t *buf, uint32_t cap)
{
    const uint32_t t = s_tx_t, h = s_tx_h;
    uint32_t n = h - t;
    if (n > cap) n = cap;
    for (uint32_t k = 0; k < n; ++k) buf[k] = s_tx[(t + k) % USBL_TX_SIZE];
    return n;
}

void usbl_tx_consume(uint32_t n) { s_tx_t += n; }
