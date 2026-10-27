/**
 * @file  checks.c
 * @brief CHECKS items (see checks.h). Reference: checks() in simDrone.ts.
 */
#include "checks.h"
#include <string.h>

static void put(pv2_check_t *it, uint8_t id, uint8_t st, int32_t v)
{
    it->id = id;
    it->state = st;
    const int16_t s = (int16_t)(v > 32767 ? 32767 : (v < -32768 ? -32768 : v));
    memcpy(&it->value, &s, sizeof s);
}

uint8_t checks_build(const checks_in_t *in, pv2_check_t *it, uint8_t *ready, uint8_t *battery_fail)
{
    const uint16_t S = in->sensors;
    const uint32_t done = in->cal_done_mask;
    uint8_t n = 0;

    /* 1 battery: no voltage measurement in this firmware yet (ADC) -> N/A */
    put(&it[n++], PV2_CHK_BATTERY, PV2_CK_NA, 0);

    /* 2 IMU & level: FAIL if the IMU is missing or the accelerometer was never calibrated */
    put(&it[n++], PV2_CHK_IMU,
        (!in->imu_ok || !(done & (1u << (PV2_CAL_ACCEL - 1u)))) ? PV2_CK_FAIL : PV2_CK_OK, 0);

    /* 3 compass: WARN above 15 % interference or if a calibration is recommended */
    if (!(S & PV2_S_MAG_FC)) put(&it[n++], PV2_CHK_COMPASS, PV2_CK_NA, 0);
    else {
        const int32_t i = (int32_t)(in->mag_interf_pct + 0.5f);
        const uint8_t never = !(done & (1u << (PV2_CAL_COMPASS - 1u)));
        uint8_t st = (i > 15 || never) ? PV2_CK_WARN : PV2_CK_OK;
        if (never && (S & PV2_S_GNSS)) st = PV2_CK_FAIL;      /* GNSS models need it */
        put(&it[n++], PV2_CHK_COMPASS, st, i);
    }

    /* 4 GNSS, 5 home, 6 ELRS: not fitted */
    put(&it[n++], PV2_CHK_GNSS,  PV2_CK_NA, 0);
    put(&it[n++], PV2_CHK_HOME,  PV2_CK_NA, 0);
    put(&it[n++], PV2_CHK_RADIO, PV2_CK_NA, 0);

    /* 7 EKF */
    put(&it[n++], PV2_CHK_EKF,
        !in->est_running ? PV2_CK_FAIL : (in->att_unc_deg > 3.0f ? PV2_CK_WARN : PV2_CK_OK), 0);

    /* 8 geofence / failsafe: without an altitude estimate the altitude fence
     * cannot be enforced -> WARN */
    {
        const uint8_t alt = (S & (PV2_S_BARO | PV2_S_TOF)) != 0u && in->alt_valid;
        const int32_t v = (S & PV2_S_GNSS) ? (int32_t)in->gf_max_dist : (int32_t)in->gf_max_alt;
        put(&it[n++], PV2_CHK_FAILSAFE, alt ? PV2_CK_OK : PV2_CK_WARN, v);
    }

    /* 9 optical flow. On the ground the sensor is below its 80 mm minimum
     * height, so the quality is low by design: only the sensor is checked.
     * In the air: WARN below 50 % quality. */
    if (!(S & PV2_S_FLOW)) put(&it[n++], PV2_CHK_FLOW, PV2_CK_NA, 0);
    else {
        const uint8_t q = (in->flow_q == 0xFFu) ? 0u : in->flow_q;
        uint8_t st;
        if (!in->flow_ok)    st = PV2_CK_WARN;
        else if (!in->in_air) st = PV2_CK_OK;
        else                 st = (q < 50u) ? PV2_CK_WARN : PV2_CK_OK;
        put(&it[n++], PV2_CHK_FLOW, st, q);
    }

    /* 10 throttle low on the active input (§7.1) */
    {
        const uint8_t high = in->rc_fresh && in->rc_thr > (in->hold_mode ? 100 : -900);
        put(&it[n++], PV2_CHK_THROTTLE, high ? PV2_CK_FAIL : PV2_CK_OK, 0);
    }

    /* 11 calibration: low byte required, high byte recommended (bit = type - 1) */
    {
        uint8_t req = 0, rec = 0;
        if (!(done & (1u << (PV2_CAL_ACCEL - 1u)))) req |= 1u << (PV2_CAL_ACCEL - 1u);
        if (!(done & (1u << (PV2_CAL_LEVEL - 1u)))) rec |= 1u << (PV2_CAL_LEVEL - 1u);
        if (!(done & (1u << (PV2_CAL_GYRO  - 1u)))) rec |= 1u << (PV2_CAL_GYRO - 1u);
        if ((S & PV2_S_MAG_FC) &&
            (!(done & (1u << (PV2_CAL_COMPASS - 1u))) || in->mag_interf_pct > 30.0f))
            rec |= 1u << (PV2_CAL_COMPASS - 1u);
        put(&it[n++], PV2_CHK_CALIBRATION, req ? PV2_CK_FAIL : (rec ? PV2_CK_WARN : PV2_CK_OK),
            (int32_t)((uint16_t)req | ((uint16_t)rec << 8)));
    }

    uint8_t rdy = 1, bf = 0;
    for (uint8_t k = 0; k < n; ++k) {
        if (it[k].state == PV2_CK_FAIL) rdy = 0;
        if (it[k].id == PV2_CHK_BATTERY && it[k].state == PV2_CK_FAIL) bf = 1;
    }
    *ready = rdy;
    *battery_fail = bf;
    return n;
}
