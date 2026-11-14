/**
 * Host test of the GNSS input (GNSS step 1):
 *   - the UBX parser (sync, checksum, resync, oversize payloads), the frame and CFG-VALSET builders, NAV-PVT decoding;
 *   - the M10 driver on a simulated DDC target (0xFD/0xFE byte count, 0xFF stream, plain writes): configuration,
 *     ACK, NMEA backlog, a NAV-PVT split across reads, no answer -> retries -> failure;
 *   - the IIS2MDC driver on a register model;
 *   - the ExtTask with the GPS board: detection (CAPS bits), the latest-value slot, silence -> lost -> probed again;
 *   - the NAV frame fields and CHECKS items 3, 4, 5.
 * Synthetic frames only: the bytes on the real receiver are checked on the board (issue "GNSS on the board").
 */
#include "ubx.h"
#include "m10.h"
#include "iis2mdc.h"
#include "ext_sens.h"
#include "gnss_nav.h"
#include "checks.h"
#include "proto_v2.h"
#include <stdio.h>
#include <string.h>

static int s_fail, s_pass;
#define CHECK(c, ...) do { if (c) s_pass++; else { s_fail++; printf("FAIL %d: ", __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

/* ------------------------------------------------------------ synthetic NAV-PVT */
static void put32(uint8_t *p, uint32_t v) { for (int i = 0; i < 4; ++i) p[i] = (uint8_t)(v >> (8 * i)); }
static void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }

static uint16_t make_pvt(uint8_t *out, uint16_t cap, uint32_t itow, uint8_t fix, uint8_t ok, uint8_t sv,
                         int32_t lat, int32_t lon, int32_t hmsl_mm, int32_t vn, int32_t ve, int32_t vd)
{
    uint8_t pl[UBX_LEN_NAV_PVT];
    memset(pl, 0, sizeof pl);
    put32(&pl[0], itow);
    put16(&pl[4], 2026); pl[6] = 11; pl[7] = 14; pl[8] = 10; pl[9] = 30; pl[10] = 5; pl[11] = 0x07;
    pl[20] = fix; pl[21] = ok ? 0x01 : 0x00; pl[23] = sv;
    put32(&pl[24], (uint32_t)lon); put32(&pl[28], (uint32_t)lat);
    put32(&pl[32], (uint32_t)(hmsl_mm + 48000)); put32(&pl[36], (uint32_t)hmsl_mm);
    put32(&pl[40], 1200); put32(&pl[44], 2100);
    put32(&pl[48], (uint32_t)vn); put32(&pl[52], (uint32_t)ve); put32(&pl[56], (uint32_t)vd);
    put32(&pl[60], 1500); put32(&pl[64], (uint32_t)-4500000); put32(&pl[68], 300);
    put16(&pl[76], 145);
    return ubx_frame(UBX_CLS_NAV, UBX_ID_NAV_PVT, pl, sizeof pl, out, cap);
}

/* ------------------------------------------------------------ simulated M10 (DDC) */
static uint8_t  g_out[4096];          /* receiver -> host stream */
static uint16_t g_head, g_tail;
static int      g_present, g_fail, g_ack, g_valsets, g_reads_ff;
static uint8_t  g_last_write[128];
static uint16_t g_last_len;

static void g_push(const uint8_t *d, uint16_t n) { for (uint16_t i = 0; i < n; ++i) g_out[(g_head++) % sizeof g_out] = d[i]; }
static uint16_t g_avail(void) { return (uint16_t)(g_head - g_tail); }

static int32_t g_rd(void *h, uint8_t reg, uint8_t *d, uint16_t n)
{
    (void)h;
    if (!g_present || g_fail) return -1;
    if (reg == M10_REG_AVAIL) { const uint16_t a = g_avail(); d[0] = (uint8_t)(a >> 8); if (n > 1) d[1] = (uint8_t)a; return 0; }
    if (reg == M10_REG_STREAM) {
        g_reads_ff++;
        for (uint16_t i = 0; i < n; ++i) d[i] = g_avail() ? g_out[(g_tail++) % sizeof g_out] : 0xFFu;
        return 0;
    }
    return -1;
}
static int32_t g_wr(void *h, uint8_t reg, const uint8_t *d, uint16_t n)
{
    (void)h;
    if (!g_present || g_fail) return -1;
    g_last_write[0] = reg; memcpy(&g_last_write[1], d, n); g_last_len = (uint16_t)(n + 1u);
    if (reg == UBX_SYNC1 && d[0] == UBX_SYNC2 && d[1] == UBX_CLS_CFG && d[2] == UBX_ID_CFG_VALSET) {
        g_valsets++;
        if (g_ack) {
            const uint8_t pl[2] = { UBX_CLS_CFG, UBX_ID_CFG_VALSET };
            uint8_t f[16];
            g_push(f, ubx_frame(UBX_CLS_ACK, UBX_ID_ACK_ACK, pl, 2, f, sizeof f));
        }
    }
    return 0;
}

/* ------------------------------------------------------------ simulated IIS2MDC */
static uint8_t m_reg[0x80];
static int     m_present, m_fail;
static int32_t m_rd(void *h, uint8_t reg, uint8_t *d, uint16_t n)
{
    (void)h;
    if (!m_present || m_fail) return -1;
    memcpy(d, &m_reg[reg], n);
    if (reg == IIS2MDC_REG_STATUS) m_reg[IIS2MDC_REG_STATUS] = 0;    /* read clears Zyxda */
    return 0;
}
static int32_t m_wr(void *h, uint8_t reg, const uint8_t *d, uint16_t n)
{
    (void)h;
    if (!m_present || m_fail) return -1;
    memcpy(&m_reg[reg], d, n);
    if (reg == IIS2MDC_REG_CFG_A && (d[0] & IIS2MDC_SOFT_RST)) m_reg[IIS2MDC_REG_CFG_A] = 0;
    return 0;
}
static void m_reset(void) { memset(m_reg, 0, sizeof m_reg); m_reg[IIS2MDC_REG_WHO_AM_I] = IIS2MDC_WHO_AM_I_VAL; }
static void m_sample(int16_t x, int16_t y, int16_t z)
{
    m_reg[IIS2MDC_REG_STATUS] = 0x0F;
    const int16_t v[3] = { x, y, z };
    for (int i = 0; i < 3; ++i) { m_reg[0x68 + 2 * i] = (uint8_t)v[i]; m_reg[0x69 + 2 * i] = (uint8_t)((uint16_t)v[i] >> 8); }
}

/* absent ToF and flow board */
static int32_t no16w(void *h, uint16_t r, const uint8_t *d, uint16_t n) { (void)h; (void)r; (void)d; (void)n; return -1; }
static int32_t no16r(void *h, uint16_t r, uint8_t *d, uint16_t n)       { (void)h; (void)r; (void)d; (void)n; return -1; }
static int32_t no8w(void *h, uint8_t r, const uint8_t *d, uint16_t n)   { (void)h; (void)r; (void)d; (void)n; return -1; }
static int32_t no8r(void *h, uint8_t r, uint8_t *d, uint16_t n)         { (void)h; (void)r; (void)d; (void)n; return -1; }

/* ------------------------------------------------------------ tests */
static void test_parser(void)
{
    ubx_parser_t p; memset(&p, 0, sizeof p); ubx_parser_reset(&p);
    uint8_t f[128];
    const uint16_t n = make_pvt(f, sizeof f, 1000, UBX_FIX_3D, 1, 14, 455000000, 91000000, 120500, 100, -200, 30);
    CHECK(n == 100, "NAV-PVT frame is 100 bytes (%u)", n);
    /* garbage, a false sync, then the frame */
    const uint8_t junk[] = { '$', 'G', 'N', 0xB5, 0x00, 0xB5 };
    int got = 0;
    for (unsigned i = 0; i < sizeof junk; ++i) got += ubx_parse_byte(&p, junk[i]);
    for (uint16_t i = 0; i < n; ++i) got += ubx_parse_byte(&p, f[i]);
    CHECK(got == 1 && p.cls == UBX_CLS_NAV && p.id == UBX_ID_NAV_PVT && p.len == 92, "frame found after junk");
    ubx_pvt_t v;
    CHECK(ubx_decode_pvt(p.buf, p.len, &v) == 0, "decode");
    CHECK(v.itow_ms == 1000 && v.fix_type == 3 && v.num_sv == 14 && v.lat_e7 == 455000000 && v.lon_e7 == 91000000 &&
          v.h_msl_mm == 120500 && v.vel_ned_mms[0] == 100 && v.vel_ned_mms[1] == -200 && v.vel_ned_mms[2] == 30 &&
          v.pdop_x100 == 145 && v.h_acc_mm == 1200 && v.head_mot_e5 == -4500000 && v.year == 2026, "fields");
    CHECK(ubx_pvt_usable(&v), "3D + gnssFixOK usable");
    CHECK(ubx_decode_pvt(p.buf, 91, &v) == -1, "wrong length refused");

    /* a corrupted checksum is counted and the parser recovers on the next frame */
    f[50] ^= 0x5A;
    got = 0;
    for (uint16_t i = 0; i < n; ++i) got += ubx_parse_byte(&p, f[i]);
    f[50] ^= 0x5A;
    for (uint16_t i = 0; i < n; ++i) got += ubx_parse_byte(&p, f[i]);
    CHECK(got == 1 && p.bad_ck == 1, "bad checksum dropped, next frame taken (%d, %u)", got, (unsigned)p.bad_ck);

    /* a payload larger than the buffer is skipped without overrun */
    static uint8_t big_pl[300], big[320];
    const uint16_t bn = ubx_frame(0x0A, 0x04, big_pl, sizeof big_pl, big, sizeof big);
    got = 0;
    for (uint16_t i = 0; i < bn; ++i) got += ubx_parse_byte(&p, big[i]);
    for (uint16_t i = 0; i < n; ++i) got += ubx_parse_byte(&p, f[i]);
    CHECK(got == 1 && p.skipped == 1 && p.cls == UBX_CLS_NAV, "oversize skipped, next frame taken");

    /* fix types */
    ubx_pvt_t w = v; w.fix_type = UBX_FIX_2D;             CHECK(!ubx_pvt_usable(&w) && gnss_fix_code(&w) == 2, "2D not usable");
    w = v; w.flags = 0;                                    CHECK(!ubx_pvt_usable(&w) && gnss_fix_code(&w) == 0, "no gnssFixOK");
    w = v; w.invalid_llh = 1;                              CHECK(!ubx_pvt_usable(&w), "invalid LLH");
    w = v; w.fix_type = UBX_FIX_GNSS_DR;                   CHECK(ubx_pvt_usable(&w) && gnss_fix_code(&w) == 3, "GNSS+DR is 3D");
}

static void test_valset(void)
{
    uint8_t f[64];
    const uint16_t n = m10_config_frame(f, sizeof f);
    /* header 6 + version/layers/reserved 4 + 2 L (5 each) + U1 (5) + U2 (6) + E1 (5) + checksum 2 */
    CHECK(n == 6 + 4 + 5 + 5 + 5 + 6 + 5 + 2, "VALSET length (%u)", n);
    CHECK(f[0] == 0xB5 && f[1] == 0x62 && f[2] == 0x06 && f[3] == 0x8A && f[4] == (uint8_t)(n - 8) && f[5] == 0, "VALSET header");
    CHECK(f[6] == 0 && f[7] == UBX_LAYER_RAM, "version 0, RAM layer only");
    /* first key: CFG-I2COUTPROT-UBX = 1, little-endian */
    CHECK(f[10] == 0x01 && f[11] == 0x00 && f[12] == 0x72 && f[13] == 0x10 && f[14] == 1, "key 1");
    CHECK(f[19] == 0, "NMEA off");
    /* rate: key 0x30210001, U2 200 */
    CHECK(f[25] == 0x01 && f[28] == 0x30 && f[29] == 200 && f[30] == 0, "rate 200 ms");
    CHECK(f[35] == UBX_DYN_AIRBORNE_4G, "airborne < 4 g");
    uint8_t a, b; ubx_checksum(&f[2], (uint16_t)(n - 4), &a, &b);
    CHECK(f[n - 2] == a && f[n - 1] == b, "checksum");
    CHECK(ubx_frame(1, 7, f, 60, f, 40) == 0, "frame too long for the buffer");
}

static void test_m10(void)
{
    const sensor_bus_t bus = { g_wr, g_rd, NULL, NULL };
    m10_t g;
    g_head = g_tail = 0; g_present = 0; g_ack = 1; g_valsets = 0;
    CHECK(m10_init(&g, &bus, 0) < 0, "absent receiver");

    /* present, with an NMEA backlog in its buffer from the boot: the ACK comes after it */
    g_present = 1;
    const char *nmea = "$GNRMC,103005.00,V,,,,,,,141126,,,N*6B\r\n$GNGGA,103005.00,,,,,0,00,99.99,,,,,,*7C\r\n";
    for (int k = 0; k < 6; ++k) g_push((const uint8_t *)nmea, (uint16_t)strlen(nmea));
    CHECK(m10_init(&g, &bus, 0) == 0 && g.state == M10_CONFIG && g_valsets == 1, "probe sends the configuration");
    CHECK(g_last_write[0] == 0xB5 && g_last_write[1] == 0x62 && g_last_len == 38, "VALSET written as plain bytes (%u)", g_last_len);
    ubx_pvt_t v;
    uint32_t t = 0; int polls = 0;
    while (g.state != M10_RUN && polls < 20) { (void)m10_poll(&g, t += 10, &v); polls++; }
    CHECK(g.state == M10_RUN && polls >= 4, "ACK found behind the backlog after %d polls (128 B each)", polls);

    /* a NAV-PVT arrives across two polls */
    uint8_t f[128];
    const uint16_t n = make_pvt(f, sizeof f, 2000, UBX_FIX_3D, 1, 12, 1, 2, 3000, 0, 0, 0);
    g_push(f, 60);
    CHECK(m10_poll(&g, t += 10, &v) == M10_NONE, "half a frame");
    g_push(&f[60], (uint16_t)(n - 60));
    CHECK(m10_poll(&g, t += 10, &v) == M10_NEW_PVT && v.itow_ms == 2000 && g.pvt_count == 1, "frame completed");
    CHECK(m10_poll(&g, t += 10, &v) == M10_NONE, "nothing new");

    /* three frames waiting (300 B): drained in three polls of at most M10_POLL_MAX bytes, oldest first */
    for (uint32_t k = 0; k < 3; ++k) { const uint16_t m = make_pvt(f, sizeof f, 3000 + k, 3, 1, 12, 1, 2, 3, 0, 0, 0); g_push(f, m); }
    const int r1 = m10_poll(&g, t += 10, &v); const uint32_t i1 = v.itow_ms;
    const int r2 = m10_poll(&g, t += 10, &v); const uint32_t i2 = v.itow_ms;
    const int r3 = m10_poll(&g, t += 10, &v);
    CHECK(r1 == M10_NEW_PVT && i1 == 3000 && r2 == M10_NEW_PVT && i2 == 3001 && r3 == M10_NEW_PVT && v.itow_ms == 3002 &&
          g_avail() == 0, "backlog drained in order (%u %u %u)", i1, i2, v.itow_ms);
    /* two whole frames in one poll: the newest is returned */
    for (uint32_t k = 0; k < 2; ++k) { const uint16_t m = make_pvt(f, sizeof f, 3100 + k, 3, 1, 12, 1, 2, 3, 0, 0, 0); g_push(f, m); }
    (void)m10_poll(&g, t += 10, &v);
    CHECK(m10_poll(&g, t += 10, &v) == M10_NEW_PVT && v.itow_ms == 3101, "newest of a poll kept");

    /* no ACK and no NAV-PVT: the VALSET is sent again every second, then the driver gives up */
    g_head = g_tail = 0; g_ack = 0; g_valsets = 0;
    CHECK(m10_init(&g, &bus, 0) == 0, "probe");
    int r = 0; t = 0;
    while (t < 5000 && (r = m10_poll(&g, t += 10, &v)) != M10_E_CONFIG) { }
    CHECK(r == M10_E_CONFIG && g_valsets == M10_CFG_TRIES && t >= 3000 && t <= 3010, "gave up after %d VALSET at %u ms", g_valsets, (unsigned)t);

    /* configuration without the ACK, but NAV-PVT flowing: configured */
    g_head = g_tail = 0;
    CHECK(m10_init(&g, &bus, 0) == 0, "probe");
    g_push(f, make_pvt(f, sizeof f, 4000, 3, 1, 9, 1, 2, 3, 0, 0, 0));
    CHECK(m10_poll(&g, 10, &v) == M10_NEW_PVT && g.state == M10_RUN, "NAV-PVT stands for the ACK");
    g_fail = 1;
    CHECK(m10_poll(&g, 20, &v) == M10_E_BUS, "bus error reported");
    g_fail = 0;
}

static void test_mag(void)
{
    const sensor_bus_t bus = { m_wr, m_rd, NULL, NULL };
    iis2mdc_t m;
    m_present = 0;
    CHECK(iis2mdc_init(&m, &bus) < 0, "absent");
    m_present = 1; m_reset();
    m_reg[IIS2MDC_REG_WHO_AM_I] = 0x41;
    CHECK(iis2mdc_init(&m, &bus) < 0, "wrong WHO_AM_I");
    m_reset();
    CHECK(iis2mdc_init(&m, &bus) == 0 && m_reg[IIS2MDC_REG_CFG_A] == IIS2MDC_CFG_A_50HZ &&
          m_reg[IIS2MDC_REG_CFG_B] == IIS2MDC_CFG_B && m_reg[IIS2MDC_REG_CFG_C] == IIS2MDC_CFG_C, "configured");
    int16_t raw[3];
    CHECK(iis2mdc_poll(&m, raw) == 0, "no data");
    m_sample(-300, 150, 2000);
    CHECK(iis2mdc_poll(&m, raw) == 1 && raw[0] == -300 && raw[1] == 150 && raw[2] == 2000, "sample");
}

static void test_ext(void)
{
    ext_buses_t b = { { no16w, no16r, NULL, NULL }, { no8w, no8r, NULL, NULL },
                      { g_wr, g_rd, NULL, NULL }, { m_wr, m_rd, NULL, NULL } };
    g_head = g_tail = 0; g_present = 0; g_ack = 1; m_present = 0; m_reset();
    ext_init(&b);
    uint32_t now = 0;
    ext_gnss_t gf;
    for (int k = 0; k < 100; ++k, now += 10) ext_step(now);
    CHECK(ext_sensors() == 0 && !ext_gnss_read(&gf), "no GPS board");

    /* the GPS board plugged in: found at the 5 s probe */
    g_present = 1; m_present = 1;
    for (; now < 5100; now += 10) ext_step(now);
    CHECK(ext_sensors() == (PV2_S_GNSS | PV2_S_MAG_GPS), "GNSS + MAG_GPS found (%04X)", ext_sensors());
    CHECK(!ext_gnss_read(&gf), "no solution yet");

    /* solutions at 5 Hz and compass samples at 50 Hz */
    uint8_t f[128];
    int mags = 0;
    for (int k = 0; k < 100; ++k, now += 10) {
        if (k % 20 == 0) g_push(f, make_pvt(f, sizeof f, 10000u + (uint32_t)k, 3, 1, 15, 455000000, 91000000, 98700, 120, 40, -10));
        if (k % 2 == 0) m_sample(100, -50, (int16_t)(400 + k));
        ext_step(now);
        ext_sample_t s;
        while (ext_take(&s)) if (s.kind == EXT_MAG) mags++;
    }
    CHECK(ext_gnss_read(&gf) && gf.count == 5 && gf.pvt.itow_ms == 10080u && gf.pvt.num_sv == 15, "latest solution (%u)", (unsigned)gf.count);
    CHECK(mags == 50, "compass samples (%d)", mags);

    /* NAV frame and checks */
    pv2_nav_t nav; memset(&nav, 0, sizeof nav);
    gnss_nav_fill(&nav, 1, 1, &gf, now);
    CHECK(nav.fix == 3 && nav.sats == 15 && nav.hdop_x100 == 145 && nav.lat_e7 == 455000000 && nav.lon_e7 == 91000000 &&
          nav.alt_msl_cm == 9870 && nav.vel_ned_cms[0] == 12 && nav.vel_ned_cms[1] == 4 && nav.vel_ned_cms[2] == -1, "NAV fields");
    memset(&nav, 0, sizeof nav);
    gnss_nav_fill(&nav, 1, 1, &gf, now + 2500);
    CHECK(nav.fix == 0 && nav.sats == 0 && nav.lat_e7 == 0, "stale solution: no fix");
    gnss_nav_fill(&nav, 0, 0, &gf, now);
    CHECK(nav.fix == 0xFF, "no GPS board: 0xFF");

    {
        checks_in_t in; memset(&in, 0, sizeof in);
        pv2_check_t it[PV2_CHECKS_MAX]; uint8_t ready, bf;
        in.sensors = PV2_S_IMU | PV2_S_MAG_FC | PV2_S_GNSS | PV2_S_MAG_GPS;
        in.imu_ok = 1; in.est_running = 1; in.cal_done_mask = 1u << (PV2_CAL_ACCEL - 1u);
        in.gnss_fix = 3; in.gnss_sats = 15;
        const uint8_t n = checks_build(&in, it, &ready, &bf);
        uint8_t st_gnss = 0xFF, st_home = 0xFF, st_cmp = 0xFF; int32_t v_gnss = -1;
        for (uint8_t i = 0; i < n; ++i) {
            if (it[i].id == PV2_CHK_GNSS) { st_gnss = it[i].state; v_gnss = it[i].value; }
            if (it[i].id == PV2_CHK_HOME) st_home = it[i].state;
            if (it[i].id == PV2_CHK_COMPASS) st_cmp = it[i].state;
        }
        CHECK(st_gnss == PV2_CK_OK && v_gnss == 15, "GNSS item OK with 15 sats");
        CHECK(st_home == PV2_CK_WARN, "home WARN until set");
        CHECK(st_cmp == PV2_CK_WARN, "uncalibrated compass WARN while the GNSS does not navigate");
        in.gnss_sats = 6;
        (void)checks_build(&in, it, &ready, &bf);
        for (uint8_t i = 0; i < n; ++i) if (it[i].id == PV2_CHK_GNSS) CHECK(it[i].state == PV2_CK_WARN, "6 sats WARN");
        in.gnss_nav = 1;
        (void)checks_build(&in, it, &ready, &bf);
        for (uint8_t i = 0; i < n; ++i) if (it[i].id == PV2_CHK_COMPASS) CHECK(it[i].state == PV2_CK_FAIL, "uncalibrated compass FAIL once the GNSS navigates");
        in.sensors = PV2_S_IMU;
        const uint8_t n2 = checks_build(&in, it, &ready, &bf);
        for (uint8_t i = 0; i < n2; ++i) if (it[i].id == PV2_CHK_GNSS || it[i].id == PV2_CHK_HOME) CHECK(it[i].state == PV2_CK_NA, "N/A without the GPS board");
    }

    /* the receiver goes silent: lost after 2 s, found again at the next probe */
    for (int k = 0; k < 230; ++k, now += 10) ext_step(now);
    CHECK(!(ext_sensors() & PV2_S_GNSS) && (ext_sensors() & PV2_S_MAG_GPS), "GNSS lost after 2 s of silence (%04X)", ext_sensors());
    for (int k = 0; k < 510; ++k, now += 10) ext_step(now);
    CHECK(ext_sensors() & PV2_S_GNSS, "probed again");

    /* the compass stops answering: lost after 50 errors */
    m_fail = 1;
    for (int k = 0; k < 60; ++k, now += 10) ext_step(now);
    CHECK(!(ext_sensors() & PV2_S_MAG_GPS), "compass lost");
    m_fail = 0;
}

int main(void)
{
    test_parser();
    test_valset();
    test_m10();
    test_mag();
    test_ext();
    printf("gnss: %d passed, %d failed\n", s_pass, s_fail);
    return s_fail ? 1 : 0;
}
