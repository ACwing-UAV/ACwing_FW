/**
 * @file  ext_sens.c
 * @brief ExtTask: VL53L1X + flow bridge on the Qwiic bus (see ext_sens.h).
 */
#include "ext_sens.h"
#include "flow_bridge.h"
#include "proto_v2.h"
#include "spsc.h"

#define EXT_TOF_BUDGET_MS 33u

SPSC_DEFINE(s_q, ext_sample_t, 16u);

static ext_buses_t       s_bus;
static vl53_t            s_tof;
static flow_t            s_flow;
static uint32_t          s_next_probe, s_tof_err, s_flow_err;
static uint8_t           s_probed;
static volatile uint16_t s_sens;
static volatile uint32_t s_err;

void ext_init(const ext_buses_t *b)
{
    s_bus = *b;
    s_tof.ok = 0; s_flow.ok = 0;
    s_tof_err = s_flow_err = 0; s_err = 0;
    s_probed = 0; s_sens = 0;
    spsc_reset(&s_q);
}

static void publish(void)
{
    s_sens = (uint16_t)((s_tof.ok ? PV2_S_TOF : 0u) | (s_flow.ok ? PV2_S_FLOW : 0u));
}

void ext_step(uint32_t now)
{
    /* probe the missing devices at the first step and every 5 s */
    if ((!s_probed || (int32_t)(now - s_next_probe) >= 0) && (!s_tof.ok || !s_flow.ok)) {
        s_probed = 1;
        s_next_probe = now + EXT_PROBE_MS;
        if (!s_tof.ok && vl53_init(&s_tof, &s_bus.tof, EXT_TOF_BUDGET_MS) == 0) s_tof_err = 0;
        if (!s_flow.ok && flow_init(&s_flow, &s_bus.flow) == 0) s_flow_err = 0;
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
    publish();
}

int      ext_take(ext_sample_t *s) { return spsc_pop(&s_q, s); }
uint16_t ext_sensors(void)         { return s_sens; }
uint32_t ext_errors(void)          { return s_err; }

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
    ext_init(&b);
    static const osThreadAttr_t a = { .name = "ext", .priority = osPriorityBelowNormal,
                                      .stack_size = 768 };
    (void)osThreadNew(ExtTask, NULL, &a);
}
#endif
