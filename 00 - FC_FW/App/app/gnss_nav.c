/**
 * @file  gnss_nav.c
 * @brief GNSS solution -> NAV frame fields (see gnss_nav.h).
 */
#include "gnss_nav.h"

static int16_t sat16(int32_t v) { return (int16_t)(v > 32767 ? 32767 : (v < -32768 ? -32768 : v)); }

uint8_t gnss_fix_code(const ubx_pvt_t *p)
{
    if (!(p->flags & 0x01u)) return 0u;
    switch (p->fix_type) {
    case UBX_FIX_3D: case UBX_FIX_GNSS_DR: return 3u;
    case UBX_FIX_2D: return 2u;
    default: return 0u;
    }
}

uint8_t gnss_fresh(const ext_gnss_t *g, uint32_t now) { return (uint8_t)((uint32_t)(now - g->t_ms) <= EXT_GNSS_SILENT_MS); }

uint8_t gnss_good(const ext_gnss_t *g, uint32_t now) { return (uint8_t)(gnss_fresh(g, now) && ubx_pvt_usable(&g->pvt)); }

void gnss_nav_fill(pv2_nav_t *n, uint8_t fitted, uint8_t have, const ext_gnss_t *g, uint32_t now)
{
    if (!fitted) { n->fix = 0xFFu; n->sats = 0u; n->hdop_x100 = 0xFFFFu; return; }
    n->fix = 0u; n->sats = 0u; n->hdop_x100 = 0xFFFFu;
    if (!have || !gnss_fresh(g, now)) return;
    n->fix = gnss_fix_code(&g->pvt);
    n->sats = g->pvt.num_sv;
    n->hdop_x100 = g->pvt.pdop_x100;
    if (!ubx_pvt_usable(&g->pvt)) return;
    n->lat_e7 = g->pvt.lat_e7;
    n->lon_e7 = g->pvt.lon_e7;
    n->alt_msl_cm = g->pvt.h_msl_mm / 10;
    for (int i = 0; i < 3; ++i) n->vel_ned_cms[i] = sat16(g->pvt.vel_ned_mms[i] / 10);
}
