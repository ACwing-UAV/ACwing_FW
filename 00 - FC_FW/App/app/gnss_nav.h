/**
 * @file  gnss_nav.h
 * @brief The GNSS solution as the telemetry and the checks see it (NAV frame, CHECKS item 4). Host-testable.
 *
 * The NAV frame keeps its layout (HANDOVER_FW §4): fix 0 / 2 / 3, or 0xFF without the GPS board; sats; hdop_x100
 * carries the **pDOP** of NAV-PVT (the message has no hDOP; pDOP >= hDOP, so it never looks better than it is);
 * lat / lon / altitude MSL / NED velocity from the receiver while the solution is usable (3D, gnssFixOK, position
 * valid) and fresh (< EXT_GNSS_SILENT_MS old). Otherwise the frame keeps the baro altitude and the flow velocity.
 */
#ifndef GNSS_NAV_H
#define GNSS_NAV_H

#include <stdint.h>
#include "ext_sens.h"
#include "proto_v2.h"

/** NAV fix code: 0 none, 2 2D, 3 3D (3 also for GNSS + dead reckoning) */
uint8_t gnss_fix_code(const ubx_pvt_t *p);
/** the solution is younger than EXT_GNSS_SILENT_MS */
uint8_t gnss_fresh(const ext_gnss_t *g, uint32_t now_ms);
/** usable and fresh: position and velocity can be shown / used */
uint8_t gnss_good(const ext_gnss_t *g, uint32_t now_ms);

/** fills the GNSS fields of *n. fitted: PV2_S_GNSS found; have: ext_gnss_read() returned 1 */
void gnss_nav_fill(pv2_nav_t *n, uint8_t fitted, uint8_t have, const ext_gnss_t *g, uint32_t now_ms);

#endif /* GNSS_NAV_H */
