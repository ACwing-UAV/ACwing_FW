/**
 * @file  ext_sens.c
 * @brief ExtTask: VL53L1X + flow bridge on I2C1 (see ext_sens.h).
 */
#include "ext_sens.h"
#include "imu_i2c.h"
#include "vl53l1x.h"
#include "flow_bridge.h"
#include "proto_v2.h"
#include "spsc.h"
#include "cmsis_os2.h"
#include "main.h"

#define EXT_I2C_TIMEOUT_MS 3u
#define EXT_PERIOD_MS      10u
#define EXT_PROBE_MS       5000u
#define EXT_ERR_LIMIT      50u      /* consecutive errors: device lost, probe again */

SPSC_DEFINE(s_q, ext_sample_t, 16u);

static volatile uint16_t s_sens;
static volatile uint32_t s_err;

/* ---- HAL binding: 16-bit registers (ToF), 8-bit registers (bridge) ---- */
static int32_t wr16(void *h, uint16_t reg, const uint8_t *d, uint16_t len)
{
    const uint16_t a8 = (uint16_t)((uintptr_t)h << 1);
    if (imu_i2c_lock()) return -1;
    const int32_t r = HAL_I2C_Mem_Write(imu_i2c_handle(), a8, reg, I2C_MEMADD_SIZE_16BIT,
                                        (uint8_t *)d, len, EXT_I2C_TIMEOUT_MS) == HAL_OK ? 0 : -1;
    imu_i2c_unlock();
    return r;
}

static int32_t rd16(void *h, uint16_t reg, uint8_t *d, uint16_t len)
{
    const uint16_t a8 = (uint16_t)((uintptr_t)h << 1);
    if (imu_i2c_lock()) return -1;
    const int32_t r = HAL_I2C_Mem_Read(imu_i2c_handle(), a8, reg, I2C_MEMADD_SIZE_16BIT,
                                       d, len, EXT_I2C_TIMEOUT_MS) == HAL_OK ? 0 : -1;
    imu_i2c_unlock();
    return r;
}

static int32_t wr8(void *h, uint8_t reg, const uint8_t *d, uint16_t len)
{
    const uint16_t a8 = (uint16_t)((uintptr_t)h << 1);
    if (imu_i2c_lock()) return -1;
    const int32_t r = HAL_I2C_Mem_Write(imu_i2c_handle(), a8, reg, I2C_MEMADD_SIZE_8BIT,
                                        (uint8_t *)d, len, EXT_I2C_TIMEOUT_MS) == HAL_OK ? 0 : -1;
    imu_i2c_unlock();
    return r;
}

static int32_t rd8(void *h, uint8_t reg, uint8_t *d, uint16_t len)
{
    const uint16_t a8 = (uint16_t)((uintptr_t)h << 1);
    if (imu_i2c_lock()) return -1;
    const int32_t r = HAL_I2C_Mem_Read(imu_i2c_handle(), a8, reg, I2C_MEMADD_SIZE_8BIT,
                                       d, len, EXT_I2C_TIMEOUT_MS) == HAL_OK ? 0 : -1;
    imu_i2c_unlock();
    return r;
}

static void dly(uint32_t ms) { osDelay(ms); }

/* ------------------------------------------------------------------ */
static void ExtTask(void *arg)
{
    (void)arg;
    static vl53_t tof;
    static flow_t flow;
    const vl53_bus_t tb = { wr16, rd16, dly, (void *)(uintptr_t)VL53L1X_ADDR_7BIT };
    const sensor_bus_t fb = { wr8, rd8, dly, (void *)(uintptr_t)FLOW_BRIDGE_ADDR_7BIT };
    uint32_t next_probe = 0, tof_err = 0, flow_err = 0;

    osDelay(500);                                /* let the IMU start first */
    for (;;) {
        const uint32_t now = osKernelGetTickCount();
        if ((int32_t)(now - next_probe) >= 0 && (!tof.ok || !flow.ok)) {
            next_probe = now + EXT_PROBE_MS;
            if (!tof.ok && vl53_init(&tof, &tb, 33) == 0) tof_err = 0;
            if (!flow.ok && flow_init(&flow, &fb) == 0) flow_err = 0;
            s_sens = (uint16_t)((tof.ok ? PV2_S_TOF : 0u) | (flow.ok ? PV2_S_FLOW : 0u));
        }

        if (tof.ok) {
            vl53_sample_t v;
            const int32_t r = vl53_poll(&tof, &v);
            if (r < 0) { s_err++; if (++tof_err > EXT_ERR_LIMIT) tof.ok = 0; }
            else {
                tof_err = 0;
                if (r > 0) {
                    const ext_sample_t s = { .kind = EXT_TOF, .status = v.status, .tof_mm = v.mm,
                                             .t_ms = osKernelGetTickCount() };
                    (void)spsc_push(&s_q, &s);
                }
            }
        }
        if (flow.ok) {
            flow_frame_t f;
            const int32_t r = flow_read(&flow, &f);
            if (r < 0) { s_err++; if (++flow_err > EXT_ERR_LIMIT) flow.ok = 0; }
            else {
                flow_err = 0;
                if (r > 0) {
                    const ext_sample_t s = { .kind = EXT_FLOW, .status = f.status, .dx = f.dx, .dy = f.dy,
                                             .squal = f.squal, .dt_s = f.dt_s,
                                             .t_ms = osKernelGetTickCount() };
                    (void)spsc_push(&s_q, &s);
                }
            }
        }
        s_sens = (uint16_t)((tof.ok ? PV2_S_TOF : 0u) | (flow.ok ? PV2_S_FLOW : 0u));
        osDelay(EXT_PERIOD_MS);
    }
}

void ext_sens_start(void)
{
    static const osThreadAttr_t a = { .name = "ext", .priority = osPriorityBelowNormal,
                                      .stack_size = 768 };
    spsc_reset(&s_q);
    (void)osThreadNew(ExtTask, NULL, &a);
}

int      ext_take(ext_sample_t *s) { return spsc_pop(&s_q, s); }
uint16_t ext_sensors(void)         { return s_sens; }
uint32_t ext_errors(void)          { return s_err; }
