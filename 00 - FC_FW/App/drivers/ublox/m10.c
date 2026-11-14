/**
 * @file  m10.c
 * @brief u-blox M10 on DDC (see m10.h).
 */
#include "m10.h"
#include <string.h>

uint16_t m10_config_frame(uint8_t *out, uint16_t cap)
{
    static const ubx_kv_t kv[] = {
        { UBX_KEY_I2COUTPROT_UBX,     1u },
        { UBX_KEY_I2COUTPROT_NMEA,    0u },
        { UBX_KEY_MSGOUT_NAV_PVT_I2C, 1u },
        { UBX_KEY_RATE_MEAS,          M10_RATE_MS },
        { UBX_KEY_NAVSPG_DYNMODEL,    UBX_DYN_AIRBORNE_4G },
    };
    return ubx_valset(UBX_LAYER_RAM, kv, (uint8_t)(sizeof kv / sizeof kv[0]), out, cap);
}

static int32_t send_config(m10_t *g, uint32_t now)
{
    uint8_t f[64];
    const uint16_t n = m10_config_frame(f, sizeof f);
    g->cfg_ms = now;
    g->cfg_tries++;
    /* the first byte as the "register": the same bytes on the wire as a plain write */
    return (n && g->bus.write(g->bus.handle, f[0], &f[1], (uint16_t)(n - 1u)) == 0) ? 0 : M10_E_BUS;
}

int32_t m10_init(m10_t *g, const sensor_bus_t *bus, uint32_t now)
{
    memset(g, 0, sizeof *g);
    g->bus = *bus;
    ubx_parser_reset(&g->p);
    uint8_t a[2];
    if (g->bus.read(g->bus.handle, M10_REG_AVAIL, a, 2u) != 0) return M10_E_BUS;
    if (send_config(g, now) != 0) return M10_E_BUS;
    g->state = M10_CONFIG;
    return 0;
}

int32_t m10_poll(m10_t *g, uint32_t now, ubx_pvt_t *pvt)
{
    if (g->state == M10_OFF) return M10_E_CONFIG;
    if (g->state == M10_CONFIG && (now - g->cfg_ms) >= M10_CFG_TIMEOUT_MS) {
        if (g->cfg_tries >= M10_CFG_TRIES) { g->state = M10_OFF; return M10_E_CONFIG; }
        if (send_config(g, now) != 0) return M10_E_BUS;
    }
    uint8_t a[2];
    if (g->bus.read(g->bus.handle, M10_REG_AVAIL, a, 2u) != 0) return M10_E_BUS;
    uint16_t avail = (uint16_t)((a[0] << 8) | a[1]);
    if (avail > M10_POLL_MAX) avail = M10_POLL_MAX;

    int32_t r = M10_NONE;
    uint8_t buf[M10_CHUNK];
    while (avail) {
        const uint16_t n = avail > M10_CHUNK ? M10_CHUNK : avail;
        if (g->bus.read(g->bus.handle, M10_REG_STREAM, buf, n) != 0) return M10_E_BUS;
        avail = (uint16_t)(avail - n);
        for (uint16_t i = 0; i < n; ++i) {
            if (!ubx_parse_byte(&g->p, buf[i])) continue;
            if (g->p.cls == UBX_CLS_ACK && g->p.len == 2u && g->p.buf[0] == UBX_CLS_CFG && g->p.buf[1] == UBX_ID_CFG_VALSET) {
                if (g->p.id == UBX_ID_ACK_ACK) { g->state = M10_RUN; g->nak = 0u; }
                else if (g->p.id == UBX_ID_ACK_NAK) g->nak = 1u;
            } else if (g->p.cls == UBX_CLS_NAV && g->p.id == UBX_ID_NAV_PVT &&
                       ubx_decode_pvt(g->p.buf, g->p.len, pvt) == 0) {
                g->state = M10_RUN;                    /* NAV-PVT on I2C: the configuration is in place */
                g->pvt_count++;
                r = M10_NEW_PVT;
            }
        }
    }
    return r;
}
