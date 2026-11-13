/**
 * Host tests of the RC receiver input: the CRSF, SBUS, i-BUS and DSM parsers (drivers/rc) and rc_input
 * (protocol selection, channel map, normalisation, failsafe, staleness, link statistics). Frames are built here
 * the way the receivers send them.
 */
#include "rc_input.h"
#include "rc_crsf.h"
#include "rc_sbus.h"
#include "rc_ibus.h"
#include "rc_dsm.h"
#include <stdio.h>
#include <string.h>

static int s_fail, s_pass;
#define CHECK(c, ...) do { if (c) s_pass++; else { s_fail++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

/* us -> 11-bit value (CRSF / SBUS: us = v * 5/8 + 880) and packing LSB first */
static uint16_t to11(uint16_t us) { return (uint16_t)(((uint32_t)(us - 880u) * 8u + 4u) / 5u); }
static void pack11(const uint16_t *us, uint8_t n, uint8_t *dst)
{
    memset(dst, 0, (n * 11u + 7u) / 8u);
    uint32_t bit = 0;
    for (uint8_t c = 0; c < n; ++c) {
        const uint16_t v = to11(us[c]);
        for (int k = 0; k < 11; ++k, ++bit) if (v & (1u << k)) dst[bit / 8u] |= (uint8_t)(1u << (bit % 8u));
    }
}

static uint8_t crsf_channels(const uint16_t *us, uint8_t *f)
{
    f[0] = CRSF_ADDR_FC; f[1] = 24; f[2] = CRSF_T_RC_CHANNELS;
    pack11(us, 16, &f[3]);
    f[25] = rc_crsf_crc8(&f[2], 23);
    return 26;
}
static uint8_t crsf_link(uint8_t rssi_neg, uint8_t lq, int8_t snr, uint8_t pwr, uint8_t *f)
{
    f[0] = CRSF_ADDR_FC; f[1] = 12; f[2] = CRSF_T_LINK_STATS;
    const uint8_t pl[10] = { rssi_neg, 120, lq, (uint8_t)snr, 0, 2, pwr, 90, 100, 5 };
    memcpy(&f[3], pl, 10);
    f[13] = rc_crsf_crc8(&f[2], 11);
    return 14;
}
static void sbus_frame(const uint16_t *us, uint8_t flags, uint8_t *f)
{
    f[0] = 0x0F; pack11(us, 16, &f[1]); f[23] = flags; f[24] = 0x00;
}
static void ibus_frame(const uint16_t *us, uint8_t *f)
{
    f[0] = 0x20; f[1] = 0x40;
    for (int c = 0; c < 14; ++c) { f[2 + 2 * c] = (uint8_t)us[c]; f[3 + 2 * c] = (uint8_t)(us[c] >> 8); }
    uint16_t s = 0xFFFF;
    for (int i = 0; i < 30; ++i) s = (uint16_t)(s - f[i]);
    f[30] = (uint8_t)s; f[31] = (uint8_t)(s >> 8);
}

static const uint16_t k_us[16] = { 1100, 1700, 1300, 1950, 1000, 1500, 2000, 1250, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500 };

static void test_crsf(void)
{
    rc_crsf_t p; rc_crsf_init(&p);
    rc_frame_t fr; rc_link_t ln; memset(&ln, 0, sizeof ln);
    uint8_t f[64];
    const uint8_t n = crsf_channels(k_us, f);
    uint8_t got = 0;
    for (uint8_t i = 0; i < n; ++i) got |= rc_crsf_feed(&p, f[i], &fr, &ln);
    int ok = (got == RC_GOT_CHANNELS && fr.n == 16);
    for (int c = 0; c < 16; ++c) ok &= (fr.us[c] >= k_us[c] - 1 && fr.us[c] <= k_us[c] + 1);
    CHECK(ok, "CRSF channels decoded within 1 us");
    f[10] ^= 0x40;                                         /* corrupt a payload byte */
    got = 0;
    for (uint8_t i = 0; i < n; ++i) got |= rc_crsf_feed(&p, f[i], &fr, &ln);
    CHECK(got == RC_NONE, "CRSF bad CRC rejected");
    const uint8_t m = crsf_link(65, 97, 9, 3, f);
    got = 0;
    for (uint8_t i = 0; i < m; ++i) got |= rc_crsf_feed(&p, f[i], &fr, &ln);
    CHECK(got == RC_GOT_LINK && ln.rssi == -65 && ln.lq == 97 && ln.snr == 9 && ln.tx_power_mw == 100, "CRSF link statistics");
}

static void test_sbus(void)
{
    rc_sbus_t p; rc_sbus_init(&p);
    rc_frame_t fr;
    uint8_t f[25], noise[7] = { 0x33, 0x0F, 0x12, 0x99, 0x00, 0x0F, 0x01 };
    sbus_frame(k_us, 0x00, f);
    uint8_t got = 0;
    for (int i = 0; i < 7; ++i) got |= rc_sbus_feed(&p, noise[i], &fr);   /* garbage first */
    for (int i = 0; i < 25; ++i) got |= rc_sbus_feed(&p, f[i], &fr);
    for (int i = 0; i < 25; ++i) got |= rc_sbus_feed(&p, f[i], &fr);
    CHECK(got == RC_GOT_CHANNELS && fr.us[3] >= 1949 && fr.us[3] <= 1951 && !fr.failsafe, "SBUS resyncs after noise and decodes");
    sbus_frame(k_us, 0x0C, f);                             /* frame lost + failsafe */
    for (int i = 0; i < 25; ++i) (void)rc_sbus_feed(&p, f[i], &fr);
    CHECK(fr.failsafe && fr.frame_lost, "SBUS failsafe and frame-lost flags");
}

static void test_ibus(void)
{
    rc_ibus_t p; rc_ibus_init(&p);
    rc_frame_t fr;
    uint8_t f[32];
    ibus_frame(k_us, f);
    uint8_t got = 0;
    for (int i = 0; i < 32; ++i) got |= rc_ibus_feed(&p, f[i], &fr);
    CHECK(got == RC_GOT_CHANNELS && fr.n == 14 && fr.us[0] == 1100 && fr.us[13] == 1500, "i-BUS decoded");
    f[5] ^= 1;
    got = 0;
    for (int i = 0; i < 32; ++i) got |= rc_ibus_feed(&p, f[i], &fr);
    CHECK(got == RC_NONE, "i-BUS bad checksum rejected");
}

static void test_dsm(void)
{
    rc_dsm_t p; rc_dsm_init(&p);
    rc_frame_t fr;
    /* DSMX 2048: channel 0 = 1024 (1500 us), channel 2 = 200 (~1020 us), slot unused = 0xFFFF */
    uint8_t f[16] = { 0, 0xB2 };
    const uint16_t w[7] = { (0u << 11) | 1024u, (2u << 11) | 200u, (1u << 11) | 1700u, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF };
    for (int i = 0; i < 7; ++i) { f[2 + 2 * i] = (uint8_t)(w[i] >> 8); f[3 + 2 * i] = (uint8_t)w[i]; }
    uint8_t got = 0; uint32_t t = 100;
    for (int i = 0; i < 16; ++i) got |= rc_dsm_feed(&p, f[i], t, &fr);
    CHECK(got == RC_GOT_CHANNELS && fr.us[0] >= 1498 && fr.us[0] <= 1501 && fr.us[2] >= 1018 && fr.us[2] <= 1021, "DSMX 2048 decoded (%u %u)", fr.us[0], fr.us[2]);
    /* DSM2 1024 after an 11 ms gap: channel 3 = 512 -> ~1500 us */
    uint8_t g[16] = { 0, 0x01 };
    const uint16_t x = (3u << 10) | 512u;
    for (int i = 0; i < 7; ++i) { g[2 + 2 * i] = 0xFF; g[3 + 2 * i] = 0xFF; }
    g[2] = (uint8_t)(x >> 8); g[3] = (uint8_t)x;
    got = 0; t += 11;
    for (int i = 0; i < 16; ++i) got |= rc_dsm_feed(&p, g[i], t, &fr);
    CHECK(got == RC_GOT_CHANNELS && fr.us[3] >= 1498 && fr.us[3] <= 1502 && fr.us[0] >= 1498, "DSM2 1024 decoded, other channels kept");
}

static void test_input(void)
{
    rc_input_init();
    rc_state_t s;
    CHECK(rc_input_read(&s) == 0, "no receiver before the first frame");
    (void)rc_input_configure(RC_PROTO_CRSF, RC_MAP_AETR);
    uint8_t f[64];
    /* AETR: ch1 roll 1100, ch2 pitch 1700, ch3 throttle 1300, ch4 yaw 1950, AUX1 1000 */
    const uint8_t n = crsf_channels(k_us, f);
    rc_input_feed(f, n, 1000);
    CHECK(rc_input_read(&s) && s.valid, "valid after one frame");
    CHECK(s.roll == -800 && s.pitch == 400 && s.thr == -400 && s.yaw == 900 && s.aux_us[0] >= 999 && s.aux_us[0] <= 1001,
          "AETR: roll %d pitch %d thr %d yaw %d", s.roll, s.pitch, s.thr, s.yaw);
    (void)rc_input_configure(RC_PROTO_CRSF, RC_MAP_TAER);
    rc_input_feed(f, n, 1004);
    (void)rc_input_read(&s);
    CHECK(s.thr == -800 && s.roll == 400 && s.pitch == -400 && s.yaw == 900, "TAER: thr %d roll %d pitch %d", s.thr, s.roll, s.pitch);
    /* staleness */
    rc_input_service(1004 + RC_STALE_MS + 1);
    (void)rc_input_read(&s);
    CHECK(!s.valid && s.ever, "no frame for %u ms: not valid, receiver still known", RC_STALE_MS);
    /* CRSF link stats with LQ 0: the transmitter is gone */
    rc_input_feed(f, n, 2000);
    const uint8_t m = crsf_link(110, 0, -5, 1, f);
    rc_input_feed(f, m, 2002);
    (void)rc_input_read(&s);
    CHECK(!s.valid && s.link.lq == 0, "CRSF LQ 0: not valid");
    /* SBUS with the failsafe flag: not valid; LQ from the lost frames */
    CHECK(rc_input_configure(RC_PROTO_SBUS, RC_MAP_AETR) == 1, "protocol change reported (UART to reconfigure)");
    uint8_t sb[25];
    uint32_t t = 3000;
    for (int k = 0; k < 100; ++k) { sbus_frame(k_us, (k % 10 == 0) ? 0x04 : 0x00, sb); t += 9; rc_input_feed(sb, 25, t); }
    (void)rc_input_read(&s);
    CHECK(s.valid && s.link.lq == 90, "SBUS: 10 %% lost frames -> LQ 90 (%u)", s.link.lq);
    sbus_frame(k_us, 0x08, sb); t += 9; rc_input_feed(sb, 25, t);
    (void)rc_input_read(&s);
    CHECK(!s.valid, "SBUS failsafe flag: not valid");
    rc_input_service(t + 1000);
    (void)rc_input_read(&s);
    CHECK(s.link.pkt_rate_hz > 0, "packet rate measured (%u Hz)", s.link.pkt_rate_hz);
    /* resync after a pause: half a frame, a pause, then a whole frame */
    sbus_frame(k_us, 0, sb);
    rc_input_feed(sb, 12, t + 1100);
    const uint32_t fr0 = s.frames;
    rc_input_feed(sb, 25, t + 1110);
    (void)rc_input_read(&s);
    CHECK(s.frames == fr0 + 1 && s.valid, "a pause restarts the parser: the next whole frame is decoded");
    /* serial settings */
    rc_uart_cfg_t u;
    rc_input_uart_cfg(RC_PROTO_SBUS, &u);
    CHECK(u.baud == 100000 && u.parity_even && u.stop2 && u.inverted, "SBUS 100k 8E2 inverted");
    rc_input_uart_cfg(RC_PROTO_CRSF, &u);
    CHECK(u.baud == 420000 && !u.inverted, "CRSF 420k 8N1");
    CHECK(rc_throttle_from_us(1000) == -1000 && rc_throttle_from_us(2000) == 1000 && rc_stick_from_us(1500) == 0, "scales");
}

int main(void)
{
    test_crsf();
    test_sbus();
    test_ibus();
    test_dsm();
    test_input();
    printf("rc: %d passed, %d failed\n", s_pass, s_fail);
    return s_fail ? 1 : 0;
}
