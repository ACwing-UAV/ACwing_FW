/**
 * @file  rc_input.c
 * @brief RC receiver input (see rc_input.h).
 */
#include "rc_input.h"
#include "rc_crsf.h"
#include "rc_sbus.h"
#include "rc_ibus.h"
#include "rc_dsm.h"
#include "spsc.h"
#include <string.h>

/* ---- parsers: only the selected one is used ---- */
static union { rc_crsf_t crsf; rc_sbus_t sbus; rc_ibus_t ibus; rc_dsm_t dsm; } s_p;
static uint8_t  s_proto, s_map;

/* ---- writer state (RcTask) ---- */
static rc_state_t s_w;
static rc_link_t  s_link;
static uint32_t   s_last_byte_ms;
static uint8_t    s_have_byte;
static uint32_t   s_win_ms, s_win_frames;
static uint8_t    s_have_win;
static uint16_t   s_sbus_lost, s_sbus_seen;        /* SBUS frame-lost ratio -> lq */

/* ---- published copy ---- */
static volatile uint32_t s_seq;
static rc_state_t        s_pub;

static void publish(void)
{
    s_seq++;
    SPSC_BARRIER();
    s_pub = s_w;
    SPSC_BARRIER();
    s_seq++;
}

static int16_t clamp1000(int32_t v) { return (int16_t)(v > 1000 ? 1000 : (v < -1000 ? -1000 : v)); }

int16_t rc_stick_from_us(uint16_t us)    { return clamp1000(((int32_t)us - 1500) * 2); }
int16_t rc_throttle_from_us(uint16_t us) { return clamp1000(((int32_t)us - 1000) * 2 - 1000); }

static void reset_parser(void)
{
    switch (s_proto) {
    case RC_PROTO_CRSF: rc_crsf_init(&s_p.crsf); break;
    case RC_PROTO_SBUS: rc_sbus_init(&s_p.sbus); break;
    case RC_PROTO_IBUS: rc_ibus_init(&s_p.ibus); break;
    case RC_PROTO_DSM:  rc_dsm_init(&s_p.dsm);   break;
    default: break;
    }
}

void rc_input_init(void)
{
    memset(&s_w, 0, sizeof s_w);
    memset(&s_link, 0, sizeof s_link);
    s_link.lq = 0xFFu; s_link.rssi = -128; s_link.snr = -128;
    s_w.link = s_link;
    s_proto = RC_PROTO_NONE; s_map = RC_MAP_AETR;
    s_have_byte = 0u; s_have_win = 0u; s_win_frames = 0u;
    s_sbus_lost = s_sbus_seen = 0u;
    s_seq = 0u;
    publish();
}

uint8_t rc_input_configure(uint8_t protocol, uint8_t map)
{
    if (protocol >= RC_PROTO_COUNT) protocol = RC_PROTO_NONE;
    s_map = (map == RC_MAP_TAER) ? RC_MAP_TAER : RC_MAP_AETR;
    if (protocol == s_proto) return 0u;
    s_proto = protocol;
    reset_parser();
    s_w.valid = 0u;
    s_link.lq = 0xFFu; s_link.rssi = -128; s_link.snr = -128; s_link.tx_power_mw = 0u;
    s_sbus_lost = s_sbus_seen = 0u;
    publish();
    return 1u;
}

uint8_t rc_input_protocol(void) { return s_proto; }

void rc_input_uart_cfg(uint8_t protocol, rc_uart_cfg_t *o)
{
    memset(o, 0, sizeof *o);
    switch (protocol) {
    case RC_PROTO_CRSF: o->baud = CRSF_BAUD; break;
    case RC_PROTO_SBUS: o->baud = SBUS_BAUD; o->parity_even = 1u; o->stop2 = 1u; o->inverted = 1u; break;
    case RC_PROTO_IBUS: o->baud = IBUS_BAUD; break;
    case RC_PROTO_DSM:  o->baud = DSM_BAUD;  break;
    default: break;
    }
}

/* channels -> sticks (channel map), AUX */
static void apply_frame(const rc_frame_t *f, uint32_t now)
{
    static const uint8_t k_aetr[4] = { 2, 3, 1, 0 };   /* thr, yaw, pitch, roll <- channel index */
    static const uint8_t k_taer[4] = { 0, 3, 2, 1 };
    const uint8_t *m = (s_map == RC_MAP_TAER) ? k_taer : k_aetr;
    if (f->n < 4u) return;
    s_w.thr   = rc_throttle_from_us(f->us[m[0]]);
    s_w.yaw   = rc_stick_from_us(f->us[m[1]]);
    s_w.pitch = rc_stick_from_us(f->us[m[2]]);
    s_w.roll  = rc_stick_from_us(f->us[m[3]]);
    for (uint8_t i = 0; i < RC_N_AUX; ++i) s_w.aux_us[i] = (uint16_t)((4u + i < f->n) ? f->us[4u + i] : 0u);
    s_w.valid = (uint8_t)!f->failsafe;
    if (s_w.valid) { s_w.t_ms = now; s_w.ever = 1u; s_w.frames++; s_win_frames++; }
}

/* SBUS: link quality from the share of frames the receiver did not get (last ~100) */
static void sbus_quality(const rc_frame_t *f)
{
    s_sbus_seen++;
    if (f->frame_lost) s_sbus_lost++;
    if (s_sbus_seen >= 100u) {
        s_link.lq = (uint8_t)(100u - (s_sbus_lost * 100u) / s_sbus_seen);
        s_sbus_seen = s_sbus_lost = 0u;
    }
}

static uint8_t feed_one(uint8_t b, uint32_t now, rc_frame_t *f)
{
    switch (s_proto) {
    case RC_PROTO_CRSF: return rc_crsf_feed(&s_p.crsf, b, f, &s_link);
    case RC_PROTO_SBUS: return rc_sbus_feed(&s_p.sbus, b, f);
    case RC_PROTO_IBUS: return rc_ibus_feed(&s_p.ibus, b, f);
    case RC_PROTO_DSM:  return rc_dsm_feed(&s_p.dsm, b, now, f);
    default:            return RC_NONE;
    }
}

void rc_input_feed(const uint8_t *b, uint32_t n, uint32_t now)
{
    /* CRSF / SBUS / i-BUS: a pause longer than a frame restarts the parser (resync after noise) */
    if (s_have_byte && (now - s_last_byte_ms) > 3u && s_proto != RC_PROTO_DSM) reset_parser();
    if (n) { s_last_byte_ms = now; s_have_byte = 1u; }
    uint8_t changed = 0u;
    for (uint32_t i = 0; i < n; ++i) {
        rc_frame_t f;
        const uint8_t r = feed_one(b[i], now, &f);
        if (r == RC_GOT_CHANNELS) {
            if (s_proto == RC_PROTO_SBUS) sbus_quality(&f);
            apply_frame(&f, now);
            changed = 1u;
        } else if (r == RC_GOT_LINK) {
            changed = 1u;
            if (s_link.lq == 0u) s_w.valid = 0u;       /* CRSF: the transmitter is gone */
        }
    }
    if (changed) { s_w.link = s_link; publish(); }
}

void rc_input_service(uint32_t now)
{
    uint8_t changed = 0u;
    if (s_w.valid && (now - s_w.t_ms) > RC_STALE_MS) { s_w.valid = 0u; changed = 1u; }
    if (!s_have_win) { s_win_ms = now; s_have_win = 1u; }
    if ((now - s_win_ms) >= 1000u) {
        s_link.pkt_rate_hz = (uint16_t)((s_win_frames * 1000u) / (now - s_win_ms));
        s_win_frames = 0u; s_win_ms = now;
        s_w.link = s_link;
        changed = 1u;
    }
    if (changed) publish();
}

uint8_t rc_input_read(rc_state_t *out)
{
    for (int tries = 0; tries < 8; ++tries) {
        const uint32_t s0 = s_seq;
        if (s0 & 1u) continue;
        SPSC_BARRIER();
        rc_state_t v = s_pub;
        SPSC_BARRIER();
        if (s_seq == s0) { *out = v; return v.ever; }
    }
    return 0u;
}
