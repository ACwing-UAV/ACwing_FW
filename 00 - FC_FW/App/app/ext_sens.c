/**
 * @file  ext_sens.c
 * @brief ExtTask: VL53L1X, flow bridge, M10 GNSS and IIS2MDC on the Qwiic bus (see ext_sens.h).
 */
#include "ext_sens.h"
#include "flow_bridge.h"
#include "m10.h"
#include "iis2mdc.h"
#include "proto_v2.h"
#include "spsc.h"

#define EXT_TOF_BUDGET_MS 33u

SPSC_DEFINE(s_q, ext_sample_t, 16u);

static ext_buses_t       s_bus;
static vl53_t            s_tof;
static flow_t            s_flow;
static m10_t             s_gnss;
static iis2mdc_t         s_mag;
static uint8_t           s_gnss_ok;
static uint32_t          s_gnss_last;          /* last NAV-PVT, or the probe time */
static uint32_t          s_next_probe, s_tof_err, s_flow_err, s_gnss_err, s_mag_err;
/* GNSS latest-value slot (seqlock, one writer: ExtTask) */
static volatile uint32_t s_gseq;
static ext_gnss_t        s_gfix;
static uint8_t           s_probed;
static volatile uint16_t s_sens;
static volatile uint32_t s_err;

void ext_init(const ext_buses_t *b)
{
    s_bus = *b;
    s_tof.ok = 0; s_flow.ok = 0; s_gnss_ok = 0; s_mag.ok = 0;
    s_tof_err = s_flow_err = s_gnss_err = s_mag_err = 0; s_err = 0;
    s_gseq = 0; s_gfix.count = 0;
    s_probed = 0; s_sens = 0;
    spsc_reset(&s_q);
}

static void publish(void)
{
    s_sens = (uint16_t)((s_tof.ok ? PV2_S_TOF : 0u) | (s_flow.ok ? PV2_S_FLOW : 0u) |
                        (s_gnss_ok ? PV2_S_GNSS : 0u) | (s_mag.ok ? PV2_S_MAG_GPS : 0u));
}

/* a device is fitted when its bus has a read function (the host tests leave some out) */
static uint8_t fitted(const sensor_bus_t *b) { return (uint8_t)(b->read != 0); }

static uint8_t missing(void)
{
    return (uint8_t)(!s_tof.ok || !s_flow.ok || (fitted(&s_bus.gnss) && !s_gnss_ok) || (fitted(&s_bus.mag) && !s_mag.ok));
}

static void gnss_store(const ubx_pvt_t *p, uint32_t now)
{
    s_gseq++;
    SPSC_BARRIER();
    s_gfix.pvt = *p;
    s_gfix.t_ms = now;
    s_gfix.count++;
    SPSC_BARRIER();
    s_gseq++;
}

/* a lost device: probe again in 5 s */
static void lost(uint8_t *ok, uint32_t now) { *ok = 0; s_next_probe = now + EXT_PROBE_MS; }

static void gnss_step(uint32_t now)
{
    ubx_pvt_t p;
    const int32_t r = m10_poll(&s_gnss, now, &p);
    if (r == M10_E_CONFIG) { lost(&s_gnss_ok, now); return; }
    if (r < 0) {
        s_err++;
        if (++s_gnss_err > EXT_ERR_LIMIT) lost(&s_gnss_ok, now);
        return;
    }
    s_gnss_err = 0;
    if (r == M10_NEW_PVT) { s_gnss_last = now; gnss_store(&p, now); }
    else if ((now - s_gnss_last) > EXT_GNSS_SILENT_MS && s_gnss.state == M10_RUN) lost(&s_gnss_ok, now);
}

static void mag_step(uint32_t now)
{
    int16_t m[3];
    const int32_t r = iis2mdc_poll(&s_mag, m);
    if (r < 0) {
        s_err++;
        if (++s_mag_err > EXT_ERR_LIMIT) lost(&s_mag.ok, now);
        return;
    }
    s_mag_err = 0;
    if (r > 0) {
        const ext_sample_t s = { .kind = EXT_MAG, .mag = { m[0], m[1], m[2] }, .t_ms = now };
        (void)spsc_push(&s_q, &s);
    }
}

void ext_step(uint32_t now)
{
    /* probe the missing devices at the first step and every 5 s */
    if ((!s_probed || (int32_t)(now - s_next_probe) >= 0) && missing()) {
        s_probed = 1;
        s_next_probe = now + EXT_PROBE_MS;
        if (!s_tof.ok && vl53_init(&s_tof, &s_bus.tof, EXT_TOF_BUDGET_MS) == 0) s_tof_err = 0;
        if (!s_flow.ok && flow_init(&s_flow, &s_bus.flow) == 0) s_flow_err = 0;
        if (fitted(&s_bus.gnss) && !s_gnss_ok && m10_init(&s_gnss, &s_bus.gnss, now) == 0) {
            s_gnss_ok = 1; s_gnss_err = 0; s_gnss_last = now;
        }
        if (fitted(&s_bus.mag) && !s_mag.ok && iis2mdc_init(&s_mag, &s_bus.mag) == 0) s_mag_err = 0;
        publish();
    }

    if (s_tof.ok) {
        vl53_sample_t v;
        const int32_t r = vl53_poll(&s_tof, &v);
        if (r < 0) {
            s_err++;
            if (++s_tof_err > EXT_ERR_LIMIT) { s_tof.ok = 0; s_next_probe = now + EXT_PROBE_MS; }
        } else {
            s_tof_err = 0;
            if (r > 0) {
                const ext_sample_t s = { .kind = EXT_TOF, .status = v.status, .tof_mm = v.mm, .t_ms = now };
                (void)spsc_push(&s_q, &s);
            }
        }
    }
    if (s_flow.ok) {
        flow_frame_t f;
        const int32_t r = flow_read(&s_flow, &f);
        if (r < 0) {
            s_err++;
            if (++s_flow_err > EXT_ERR_LIMIT) { s_flow.ok = 0; s_next_probe = now + EXT_PROBE_MS; }
        } else {
            /* every frame goes up, also without new motion: the FC needs the
             * sensor status and SQUAL on the ground too (below the PMW3901
             * minimum height every sample is rejected by the bridge) */
            s_flow_err = 0;
            const ext_sample_t s = { .kind = EXT_FLOW, .status = f.status,
                                     .dx = r > 0 ? f.dx : 0, .dy = r > 0 ? f.dy : 0,
                                     .squal = f.squal, .dt_s = r > 0 ? f.dt_s : 0.0f, .t_ms = now };
            (void)spsc_push(&s_q, &s);
        }
    }
    if (s_gnss_ok) gnss_step(now);
    if (s_mag.ok)  mag_step(now);
    publish();
}

int      ext_take(ext_sample_t *s) { return spsc_pop(&s_q, s); }
uint16_t ext_sensors(void)         { return s_sens; }
uint32_t ext_errors(void)          { return s_err; }

int ext_gnss_read(ext_gnss_t *out)
{
    for (int tries = 0; tries < 8; ++tries) {
        const uint32_t s0 = s_gseq;
        if (s0 & 1u) continue;
        SPSC_BARRIER();
        const ext_gnss_t v = s_gfix;
        SPSC_BARRIER();
        if (s_gseq == s0) {
            if (!v.count) return 0;
            *out = v;
            return 1;
        }
    }
    return 0;
}

/* ------------------------------------------------------------------ RTOS shell */
#ifndef HOST_TEST
#include "cmsis_os2.h"
#include "ext_i2c.h"

static void ExtTask(void *arg)
{
    (void)arg;
    osDelay(500);                                /* let the IMU start first */
    for (;;) {
        ext_step(osKernelGetTickCount());
        osDelay(EXT_PERIOD_MS);
    }
}

void ext_sens_start(void)
{
    ext_buses_t b;
    ext_i2c_tof_bus(&b.tof, VL53L1X_ADDR_7BIT);
    ext_i2c_flow_bus(&b.flow, FLOW_BRIDGE_ADDR_7BIT);
    ext_i2c_flow_bus(&b.gnss, M10_ADDR_7BIT);         /* 8-bit registers: 0xFD / 0xFE / 0xFF */
    ext_i2c_flow_bus(&b.mag, IIS2MDC_ADDR_7BIT);
    ext_init(&b);
    static const osThreadAttr_t a = { .name = "ext", .priority = osPriorityBelowNormal,
                                      .stack_size = 1024 };   /* + the 64-byte DDC chunk and the NAV-PVT */
    (void)osThreadNew(ExtTask, NULL, &a);
}
#endif
