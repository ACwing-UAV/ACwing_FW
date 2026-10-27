/**
 * @file  app_main.c
 * @brief Phase 1: multi-rate attitude estimation on STM32WB55, control hooks in place.
 *
 *   INT1 LSM6DSV16X (FIFO threshold, 960 Hz)
 *        |  EXTI ISR: timestamp DWT + osThreadFlagsSet
 *        v
 *   ImuTask  (osPriorityRealtime)
 *        FIFO status (2 bytes) -> DMA burst of N words -> parse -> est_process():
 *        propagation 960 Hz, ctrl_rate_step, acc update 240 Hz,
 *        ctrl_attitude_step, update mag ~50 Hz
 *        |  qTlm (240 Hz snapshots, never blocking)
 *        v
 *   TelemetryTask (osPriorityBelowNormal)   UART DMA -> VCP ST-LINK
 *        |  (APP_TLM_BLE = 1) same frames, STATE decimated, no REPLAY
 *        v
 *   tlm_ble ring  ->  BleTxThread (ble_tlm_svc.c)  ->  GATT notifications
 *                     (BLE stack on the Cortex-M0+, IPCC)
 *
 * ImuTask remains the only high-priority task: BLE never touches it.
 *
 * Protocol v2 (ACwing Pilot app, HANDOVER_FW.md):
 *   ImuTask also runs the flight core (fc_core: arming, modes, TAKEOFF/LAND/
 *   HOLD, failsafes) at ~250 Hz and the vertical estimator (alt_est: baro on
 *   the sensor hub, ToF from the ExtTask), and it is the only task that feeds
 *   the independent watchdog. AppTask (app_task.c) handles commands,
 *   parameters, flash, calibrations and the v2 frames. ExtTask (ext_sens.c)
 *   polls the ACwing Flow board on the Qwiic bus.
 */
#include "app_main.h"
#include "app_config.h"
#include "cmsis_os2.h"
#include "imu_i2c.h"
#include "imu_lsm6dsv16x.h"
#include "est_pipeline.h"
#include "control_hooks.h"
#include "motors.h"
#include "telemetry.h"
#include "tlm_uart.h"
#include "tlm_ble.h"
#include "so3.h"
#include "tlm_build.h"
#include "fc_core.h"
#include "fc_shared.h"
#include "app_task.h"
#include "alt_est.h"
#include "ext_sens.h"
#include "proto_v2.h"
#include <string.h>
#include <math.h>

extern I2C_HandleTypeDef  hi2c1;     /* PB8 SCL (D15) / PB9 SDA (D14)          */
extern UART_HandleTypeDef huart1;    /* PB6/PB7 -> ST-LINK virtual COM port     */

/* PA15 = D5 = LSM6DSV16X INT1 on the IKS4A1. CubeMX label "IMU_INT1". */
#ifndef IMU_INT1_Pin
#error "In CubeMX, assign the label IMU_INT1 to pin PA15 (GPIO_EXTI15)"
#endif

#define FLAG_IMU_IRQ 0x0001u

/* ---- watchdog: LSI 32 kHz / 32 = 1 kHz, reload 500 -> 0.5 s ----
 * Fed ONLY by the ImuTask (the control loop). A stalled BLE stack, CPU2,
 * AppTask or flash write cannot keep it alive; a stalled control loop resets
 * the MCU (motors off at reset). Frozen while the debugger halts the core. */
#define APP_IWDG_RELOAD   500u
#define FC_SERVICE_MS     4u        /* flight core ~250 Hz */

/* ------------------------------------------------------------------ */
static osThreadId_t       s_imuTask;
static osMessageQueueId_t qTlm;
static volatile uint32_t  s_irqCyc;
static volatile uint32_t  s_irqCount, s_tlmDrop, s_busErr, s_fifoOvr;
/* Bring-up diagnostics (Live Expressions in the debugger) */
static volatile uint32_t  s_procCycMax, s_procCycLast;   /* CPU cycles of parse+est_process */
static volatile uint32_t  s_wordsMax;
static volatile uint32_t  s_drainReads;   /* immediate re-reads (INT1 edge recovery) */
static volatile float     s_odrHz;        /* IMU sample rate measured against the MCU clock */
static volatile int8_t    s_imuInit = 1;          /* 1 = in progress, 0 = OK, <0 error */
static imu_parse_stats_t  s_psTot;                /* cumulative FIFO counters        */

static imu_t   s_imu;
static est_t   s_est;
static uint8_t s_fifo[IMU_FIFO_MAX_WORDS * IMU_FIFO_WORD_BYTES];
static imu_event_t s_ev[IMU_FIFO_MAX_WORDS];

typedef struct {
    uint16_t len;
    uint8_t  bytes[TLM_FRAME_MAX];
} tlm_msg_t;

uint8_t                  g_initFrame[TLM_FRAME_MAX];
volatile uint16_t        g_initLen;

/* ---- calibration in effect (ImuTask copy, applied at event level) ---- */
static cal_set_t      s_calset;
static est_config_t   s_ec;                  /* estimator config (restart after a calibration) */
static volatile float g_level_trim_deg[2];   /* LEVEL: roll, pitch trim for the attitude loop */
static volatile uint8_t s_est_restart;

/* ---- vertical estimate, flow, statistics ---- */
static alt_est_t s_alt;
static uint8_t   s_flow_q = 0xFFu;
static float     s_flow_v[2];
static uint32_t  s_flow_ms;                  /* last frame from the bridge            */
static uint8_t   s_flow_ok;                  /* bridge alive and PMW3901 OK           */
static uint8_t   s_flow_usable;              /* above the PMW3901 minimum height      */
static uint16_t  s_tof_mm = 0xFFFFu;
static uint32_t  s_tof_ms;
static float     s_vib2;                     /* EMA of (|a| - 1 g)^2, g^2 */
static uint16_t  s_acc_clip;
static float     s_loop_hz, s_jitter_us, s_cpu_pct;
static uint32_t  s_cpu_cyc;
static float     s_baro_temp;
static uint16_t  s_imu_sensors;
static volatile uint32_t s_lastGyroMs;       /* last FIFO read with gyro samples */
#define IMU_STALE_MS 50u                     /* no gyro for 50 ms: estimate not usable */

/* ------------------------------------------------------------------ */
static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

void app_exti(uint16_t pin)
{
    if (pin != IMU_INT1_Pin) return;
    s_irqCyc = DWT->CYCCNT;
    s_irqCount++;
    if (s_imuTask) (void)osThreadFlagsSet(s_imuTask, FLAG_IMU_IRQ);
}

void app_i2c_rx_cplt(I2C_HandleTypeDef *h)    { imu_i2c_rx_cplt_isr(h); }
void app_i2c_error(I2C_HandleTypeDef *h)      { imu_i2c_error_isr(h); }
void app_uart_tx_cplt(UART_HandleTypeDef *h)  { tlm_uart_tx_cplt_isr(h); }

/* ---------------- pipeline callbacks (ImuTask context) ---------------- */
static tlm_msg_t s_msg;     /* built here, copied into the queue */

static void push_frame(size_t len)
{
    if (!len) return;
    s_msg.len = (uint16_t)len;
    if (osMessageQueuePut(qTlm, &s_msg, 0, 0) != osOK) s_tlmDrop++;
}

static void on_state(const est_t *e, void *user)
{
    (void)user;
    static uint32_t n;
    {
        float q[4];
        iekf_get_quat(&e->f, q);
        const float hwin = (float)APP_STEPS_PER_ACC / (float)APP_STEP_HZ;
        alt_est_predict(&s_alt, alt_est_accel_up(q, e->acc_mean, e->f.nom_acc), hwin);
        const float am = sqrtf(e->acc_mean[0]*e->acc_mean[0] + e->acc_mean[1]*e->acc_mean[1] +
                               e->acc_mean[2]*e->acc_mean[2]);
        const float d = am - (e->f.nom_acc > 0.5f ? e->f.nom_acc : 1.0f);
        s_vib2 += 0.004f * (d*d - s_vib2);            /* ~1 s at 240 Hz */

        /* raw samples for a calibration in progress (~48 Hz) */
        static uint8_t k;
        if (g_fsh_cal_want && (++k % 5u) == 0u) {
            cal_sample_t c;
            for (int i = 0; i < 3; ++i) {
                const float sc = (s_calset.accel.scale[i] > 0.0f) ? s_calset.accel.scale[i] : 1.0f;
                c.acc[i]  = e->acc_mean[i] / sc + s_calset.accel.off[i];
                c.gyro[i] = e->gyro_mean[i] + s_calset.gyro.bias[i];
                const float ms = (s_calset.compass.scale[i] > 0.0f) ? s_calset.compass.scale[i] : 1.0f;
                c.mag[i]  = e->mag[i] / ms + s_calset.compass.hi[i];
            }
            memcpy(c.q, q, sizeof q);
            c.mag_new = (uint8_t)((e->flags & EST_F_MAG_NEW) != 0u);
            c.running = e->running;
            (void)fsh_cal_push(&c);
        }
    }
#if APP_TLM_REPLAY
    const uint32_t dec = APP_TLM_DECIM_REPLAY;
#else
    const uint32_t dec = APP_TLM_DECIMATION;
#endif
    if ((n++ % dec) == 0)
        push_frame(tlm_build_state(e, s_msg.bytes, sizeof s_msg.bytes));
#if APP_TLM_REPLAY
    push_frame(tlm_build_replay(e, s_msg.bytes, sizeof s_msg.bytes));
#endif
}

static void on_init(const est_t *e, void *user)
{
    (void)user;
    g_initLen = (uint16_t)tlm_build_init(e, (float)APP_STEP_HZ / (float)APP_STEPS_PER_ACC,
                                         g_initFrame, sizeof g_initFrame);
    /* queued immediately: it must precede the first REPLAY */
    memcpy(s_msg.bytes, g_initFrame, g_initLen);
    push_frame(g_initLen);
}

/* ================================================================== */
static void iwdg_start(void)
{
    DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_IWDG_STOP;   /* halted by the debugger: no reset */
    IWDG->KR  = 0xCCCCu;                                  /* start (LSI forced on)             */
    IWDG->KR  = 0x5555u;                                  /* unlock PR / RLR                    */
    IWDG->PR  = 3u;                                       /* /32 -> 1 kHz                       */
    IWDG->RLR = APP_IWDG_RELOAD;
    uint32_t t = 0;
    while (IWDG->SR != 0u && ++t < 100000u) { }           /* registers updated                  */
    IWDG->KR  = 0xAAAAu;
}

/* Calibration in effect, applied to the raw events before the estimator:
 * gyro bias, accelerometer offset/scale (the compass is applied by the
 * estimator: hard iron + scale). Also counts accelerometer clipping. */
static void correct_events(imu_event_t *ev, size_t n)
{
    for (size_t j = 0; j < n; ++j) {
        float *v = ev[j].v;
        switch (ev[j].type) {
        case IMU_EV_GYRO:
            for (int i = 0; i < 3; ++i) v[i] -= s_calset.gyro.bias[i];
            break;
        case IMU_EV_ACC:
            if (fabsf(v[0]) > 15.9f || fabsf(v[1]) > 15.9f || fabsf(v[2]) > 15.9f) {
                if (s_acc_clip < 0xFFFFu) s_acc_clip++;
            }
            for (int i = 0; i < 3; ++i) {
                const float sc = (s_calset.accel.scale[i] > 0.0f) ? s_calset.accel.scale[i] : 1.0f;
                v[i] = sc * (v[i] - s_calset.accel.off[i]);
            }
            break;
        case IMU_EV_BARO:
            alt_est_baro(&s_alt, v[0]);
            s_baro_temp = v[1];
            break;
        default:
            break;
        }
    }
}

/* A calibration from the AppTask (DONE or RESTORE): takes effect here. */
static void take_calibrations(void)
{
    fsh_apply_t a;
    while (fsh_apply_take(&a)) {
        switch (a.type) {
        case PV2_CAL_GYRO: {
            /* the corrected gyro changes by -delta: move the IEKF bias with it */
            for (int i = 0; i < 3; ++i) s_est.f.b[i] -= a.set.gyro.bias[i] - s_calset.gyro.bias[i];
            s_calset.gyro = a.set.gyro;
            break;
        }
        case PV2_CAL_ACCEL:
            s_calset.accel = a.set.accel;
            s_est_restart = 1u;                  /* gravity reference changes */
            break;
        case PV2_CAL_COMPASS:
            s_calset.compass = a.set.compass;
            memcpy(s_ec.mag_hard_iron, a.set.compass.hi, sizeof s_ec.mag_hard_iron);
            memcpy(s_ec.mag_scale, a.set.compass.scale, sizeof s_ec.mag_scale);
            s_est_restart = 1u;                  /* magnetic reference changes */
            break;
        case PV2_CAL_LEVEL:
            s_calset.level = a.set.level;
            g_level_trim_deg[0] = a.set.level.roll_deg;
            g_level_trim_deg[1] = a.set.level.pitch_deg;
            break;
        default:
            cal_copy_type(&s_calset, &a.set, a.type);
            break;
        }
    }
    if (s_est_restart) {
        /* only reachable while disarmed (calibrations are refused when armed) */
        s_est_restart = 0u;
        est_init(&s_est, &s_ec, on_state, on_init, NULL);
        s_est.trace_enable = APP_TLM_REPLAY;
    }
}

/* ToF and flow samples from the ExtTask */
static void take_ext(uint32_t now, float cos_tilt, const float omega[3])
{
    ext_sample_t x;
    while (ext_take(&x)) {
        if (x.kind == EXT_TOF) {
            alt_est_tof(&s_alt, (float)x.tof_mm * 1e-3f, (uint8_t)(x.status == 0u), cos_tilt);
            s_tof_mm = (x.status == 0u) ? x.tof_mm : 0xFFFFu;
            s_tof_ms = now;
        } else if (x.kind == EXT_FLOW) {
            s_flow_ms = now;
            s_flow_ok = (uint8_t)((x.status & 0x01u) != 0u);
            uint32_t q = (uint32_t)x.squal * 100u / 120u;              /* SQUAL ~120 on a good floor */
            if (!s_flow_ok) q = 0u;
            s_flow_q = (uint8_t)(q > 100u ? 100u : q);
            /* height gate: the PMW3901 works from 80 mm; the ToF gives the height
             * above the floor (no valid ToF = no height = flow not usable) */
            if (!s_flow_ok || !s_alt.tof_valid || s_alt.tof_ground || s_alt.tof_h < APP_FLOW_MIN_H) s_flow_usable = 0u;
            else if (s_alt.tof_h >= APP_FLOW_MIN_H + 0.02f) s_flow_usable = s_flow_ok;
            if (!s_flow_usable) { s_flow_v[0] = s_flow_v[1] = 0.0f; continue; }
            if (x.dt_s <= 1e-3f) continue;                              /* status only */
            /* PMW3901: ~0.0205 rad per count (42 deg / 35 px). Flow rate minus the
             * body rotation, times the height, is the ground speed. Axis signs
             * (APP_FLOW_SIGN_*) follow the frame-C2 mounting and are checked by
             * the FLOW_TOF calibration (match %). */
            const float k = 0.0205f / x.dt_s;
            const float fx = APP_FLOW_SIGN_X * (float)x.dy * k;      /* about body y -> x motion */
            const float fy = APP_FLOW_SIGN_Y * (float)x.dx * k;
            const float h  = s_alt.tof_h;
            float v[2] = { (fx - omega[1]) * h, (fy + omega[0]) * h };
            for (int i = 0; i < 2; ++i) s_flow_v[i] += 0.3f * (v[i] - s_flow_v[i]);
        }
    }
    if (s_flow_ms && (now - s_flow_ms) > 200u) {                       /* bridge silent */
        s_flow_q = 0u; s_flow_ok = 0u; s_flow_usable = 0u;
        s_flow_v[0] = s_flow_v[1] = 0.0f;
    }
    if (!(ext_sensors() & PV2_S_FLOW)) { s_flow_q = 0xFFu; s_flow_ok = 0u; s_flow_usable = 0u; }
    if (s_tof_ms && (now - s_tof_ms) > 200u) s_tof_mm = 0xFFFFu;
}

static void publish_est(uint32_t now)
{
    static uint32_t last, lastWin, winN;
    static uint32_t lastIrq;
    if ((now - last) < 20u) return;                 /* 50 Hz */
    last = now;

    /* loop rate, jitter, CPU over 1 s windows */
    if ((now - lastWin) >= 1000u) {
        const uint32_t dtw = now - lastWin;
        const uint32_t irqs = s_irqCount - lastIrq;
        s_loop_hz = (float)(s_est.win_count - winN) * 1000.0f / (float)dtw;
        s_cpu_pct = (float)s_cpu_cyc * 100.0f / ((float)SystemCoreClock * (float)dtw * 1e-3f);
        (void)irqs;
        winN = s_est.win_count; lastIrq = s_irqCount; lastWin = now; s_cpu_cyc = 0;
    }

    fsh_est_t o;
    memset(&o, 0, sizeof o);
    o.imu_ok = (uint8_t)(s_imuInit == 0);
    o.running = s_est.running;
    o.acc_gated = (uint8_t)(s_est.f.acc_infl > 2.0f);
    o.mag_gated = (uint8_t)(s_est.f.mag_infl > 2.0f);
    iekf_get_quat(&s_est.f, o.q);
    {
        const float *P = s_est.f.P;
        float m = P[0];
        if (P[IEKF_NX + 1] > m) m = P[IEKF_NX + 1];
        if (P[2 * IEKF_NX + 2] > m) m = P[2 * IEKF_NX + 2];
        o.att_unc_deg = s_est.running ? sqrtf(m > 0.0f ? m : 0.0f) * 57.29578f : 180.0f;
    }
    o.mag_interf_pct = fabsf(s_est.f.last_mag_dev) * 100.0f;
    o.vib_mg = sqrtf(s_vib2) * 1000.0f;
    o.acc_clip = s_acc_clip;
    o.loop_hz = s_loop_hz;
    o.jitter_us = s_jitter_us;
    o.cpu_pct = s_cpu_pct;
    o.imu_temp_c = s_baro_temp;                     /* the board temperature next to the IMU */
    o.alt_valid = s_alt.valid;
    o.tof_valid = s_alt.tof_valid;
    o.tof_ground = s_alt.tof_ground;
    o.h = alt_est_h(&s_alt);
    o.vz = alt_est_vz(&s_alt);
    o.tof_h = s_alt.tof_h;
    o.tof_mm = s_tof_mm;
    o.flow_q = s_flow_q;
    o.flow_ok = s_flow_ok;
    o.flow_usable = s_flow_usable;
    o.vel_body[0] = s_flow_v[0]; o.vel_body[1] = s_flow_v[1];
    o.baro_pa = s_alt.baro_ok ? (uint32_t)(s_alt.pressure + 0.5f) : 0u;
    o.sensors = s_imu_sensors;
    fsh_est_write(&o);
}

/* flight core + vertical sources, ~250 Hz from the ImuTask loop */
static void flight_service(uint32_t *lastSvc)
{
    const uint32_t now = osKernelGetTickCount();
    if ((now - *lastSvc) < FC_SERVICE_MS) return;
    *lastSvc = now;

    take_calibrations();

    float q[4];
    iekf_get_quat(&s_est.f, q);
    const float ct = 1.0f - 2.0f * (q[1]*q[1] + q[2]*q[2]);         /* R33 */
    take_ext(now, ct, s_est.omega);

    /* a stalled IMU (bus error, sensor reset) makes the estimate unusable at once */
    const uint8_t est_ok = (uint8_t)(s_est.running && (now - s_lastGyroMs) < IMU_STALE_MS);
    fc_vert_t v;
    v.valid = s_alt.valid && est_ok;
    v.h = alt_est_h(&s_alt);
    v.vz = alt_est_vz(&s_alt);
    v.tof_valid = s_alt.tof_valid;
    v.tof_ground = s_alt.tof_ground;
    v.tof_h = s_alt.tof_h;
    v.cos_tilt = ct;
    v.flow_q = s_flow_q;
    v.flow_usable = s_flow_usable;
    fc_service(now, est_ok, &v);
    {
        static uint8_t wasArmed;
        fc_setpoint_t sp;
        fc_setpoint(&sp);
        if (sp.armed && !wasArmed) s_acc_clip = 0u;     /* SYSTEM.accClip counts since arm */
        wasArmed = sp.armed;
        /* sp goes to the attitude / rate loops and the mixer in phase 3 */
    }
    publish_est(now);
}

/* ================================================================== */
static void ImuTask(void *arg)
{
    (void)arg;
    sensor_bus_t bus;
    imu_i2c_make_bus(&bus, &hi2c1, IMU_I2C_ADDR_7BIT);

    const imu_config_t icfg = {
        .odr = APP_IMU_ODR, .watermark_words = APP_IMU_WATERMARK,
        .gy_lpf1_bw = APP_GY_LPF1_BW, .xl_lpf2_enable = APP_XL_LPF2_ENABLE,
        .xl_lpf2_bw = APP_XL_LPF2_BW, .mag_enable = 1, .mag_fast = APP_MAG_FAST,
        .baro_enable = 1
    };
    for (;;) {
        const int32_t st = imu_init(&s_imu, &bus, &icfg);
        s_imuInit = (int8_t)st;           /* visible in the STATUS packet */
        if (st == IMU_OK) break;
#ifdef LED1_GPIO_Port
        HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
#endif
        /* no estimate: the core cannot arm, but it answers commands */
        fc_service(osKernelGetTickCount(), 0u, NULL);
        osDelay(200);
    }
    s_imu_sensors = (uint16_t)(PV2_S_IMU | (s_imu.mag_enabled ? PV2_S_MAG_FC : 0u) |
                               (s_imu.baro_enabled ? PV2_S_BARO : 0u));
    {
        alt_est_cfg_t ac;
        alt_est_default_cfg(&ac);
        alt_est_init(&s_alt, &ac);
    }

    est_config_t ec;
    memset(&ec, 0, sizeof ec);
    ec.gyro_per_step    = (APP_IMU_ODR == IMU_ODR_1920HZ) ? 2u : 1u;
    ec.steps_per_acc    = APP_STEPS_PER_ACC;
    ec.calib_samples    = APP_CALIB_SAMPLES;
    ec.calib_gyro_still = APP_CALIB_GYRO_STILL;
    ec.sigma_vec_min    = IEKF_SIGMA_VEC_MIN;
    iekf_default_config(&ec.ekf);
    ec.ekf.sigma_gyro = IEKF_SIGMA_GYRO;   ec.ekf.sigma_bias = IEKF_SIGMA_BIAS;
    ec.ekf.acc_tol    = IEKF_ACC_TOL;      ec.ekf.mag_tol    = IEKF_MAG_TOL;
    ec.ekf.p0_att     = IEKF_P0_ATT;       ec.ekf.p0_bias    = IEKF_P0_BIAS;
    ec.ekf.estimate_bias = IEKF_ESTIMATE_BIAS;
    ec.ekf.adaptive      = IEKF_ADAPTIVE;
    ec.ekf.tilt_comp_mag = IEKF_TILT_COMP_MAG;
    ec.ekf.acc_nis_gate  = IEKF_ACC_NIS_GATE;
    ec.ekf.mag_nis_gate  = IEKF_MAG_NIS_GATE;
    ec.ekf.infl_decay    = expf(-((float)APP_STEPS_PER_ACC / (float)APP_STEP_HZ) / IEKF_INFL_TAU_S);
    ec.ekf.dip_tol       = IEKF_DIP_TOL;
    ec.ekf.decouple      = IEKF_DECOUPLE;
    {
        const float hacc = (float)APP_STEPS_PER_ACC / (float)APP_STEP_HZ;   /* s between acc updates */
        ec.ekf.infl_max        = IEKF_INFL_MAX;
        ec.ekf.lockout_updates = (uint16_t)(IEKF_LOCKOUT_S / hacc + 0.5f);
        ec.ekf.lockout_p_bump  = IEKF_LOCKOUT_BUMP;
        ec.ekf.grace_updates   = (uint16_t)(IEKF_GRACE_S / hacc + 0.5f);
        ec.ekf.persist_decay   = expf(-hacc / IEKF_PERSIST_TAU_S);
        ec.ekf.persist_ratio   = IEKF_PERSIST_RATIO;
        ec.ekf.persist_min     = IEKF_PERSIST_MIN;
    }
    ec.still_acc_tol     = APP_STILL_ACC_TOL;
    ec.still_gyro_thr    = APP_STILL_GYRO_THR;
    ec.still_time_s      = APP_STILL_TIME_S;
    ec.ekf.reset_jacobian = IEKF_RESET_JACOBIAN;
    static const float M[9] = MAG_AXIS_MAP;
    memcpy(ec.mag_map, M, sizeof M);
    /* saved calibrations (posted by app_task_boot) */
    {
        fsh_apply_t a;
        while (fsh_apply_take(&a)) cal_copy_type(&s_calset, &a.set, a.type);
        g_level_trim_deg[0] = s_calset.level.roll_deg;
        g_level_trim_deg[1] = s_calset.level.pitch_deg;
    }
    memcpy(ec.mag_hard_iron, s_calset.compass.hi, sizeof ec.mag_hard_iron);
    memcpy(ec.mag_scale, s_calset.compass.scale, sizeof ec.mag_scale);
    const float tsh = 1.0f / IMU_SH_HZ(APP_MAG_FAST);
    ec.mag_latency_s     = 0.5f * tsh;
    ec.mag_latency_std_s = tsh * 0.288675f;            /* 1/sqrt(12) */
#if APP_COMPARE_ORIGINAL
    ec.ekf.acc_nis_gate   = 0.0f;   ec.ekf.mag_nis_gate = 0.0f;
    ec.ekf.dip_tol        = 0.0f;   ec.ekf.decouple     = 0;
    ec.ekf.reset_jacobian = 0;      ec.still_time_s     = 0.0f;
    ec.ekf.lockout_updates = 0;
    ec.mag_latency_std_s  = 0.0f;   /* the MEAN-latency compensation stays: it is an input */
#endif
    s_ec = ec;
    est_init(&s_est, &ec, on_state, on_init, NULL);
    s_est.trace_enable = APP_TLM_REPLAY;
    ctrl_init();

    /* Sample period of the IMU, measured against the MCU clock (HSE crystal).
     * The LSM6DSV16X runs on its own RC oscillator, a few % off the nominal ODR
     * (916 Hz measured on the board instead of 960): integrating the gyro with
     * the nominal dt scales every rotation by that error. The period is the
     * DWT time between two INT1 edges >= 2 s apart divided by the gyro samples
     * read in between (counted from the FIFO tags), smoothed; until the first
     * window the nominal value is used (static calibration, board still).
     * A per-read measurement is not used: drain reads have no edge of their own
     * and fell back to the nominal dt (filter time 1.3 % behind real time). */
    const float dtNom = 1.0f / s_imu.odr_hz;
    const float cyc2s = 1.0f / (float)SystemCoreClock;
    const uint32_t winCyc = 2u * SystemCoreClock;       /* 2 s, < 2^32 cycles */
    float    dtEst = dtNom;
    uint32_t refCyc = 0, refN = 0, lastCyc = 0;
    uint8_t  haveRef = 0, haveEst = 0;
    s_odrHz = s_imu.odr_hz;

    /* INT1 is a LEVEL ("FIFO level >= watermark") while the EXTI fires on its
     * RISING EDGE. If one read falls behind (start-up of the BLE stack, a bus
     * retry, ...), the FIFO stays above the watermark after the read, INT1 never
     * goes low again and no further edge arrives: the task would live on the
     * 5 ms timeout, read big chunks, hit the DMA timeout and lose samples
     * (seen with BLE: 0 IRQ/s, ~39 % of the gyro samples, ~50 bus errors/s).
     * Hence: after a read that found much more than the watermark, or after a
     * wake-up by timeout, read again at once; the FIFO drops below the
     * watermark, INT1 goes low and the next sample produces an edge again. */
    iwdg_start();
    uint32_t wait_ms = 5u;
    uint32_t lastSvc = osKernelGetTickCount();
    for (;;) {
        IWDG->KR = 0xAAAAu;                    /* feed: the control loop is alive */
        flight_service(&lastSvc);
        const uint32_t fl = osThreadFlagsWait(FLAG_IMU_IRQ, osFlagsWaitAny, wait_ms);
        const int by_timeout = (fl & osFlagsError) != 0u;
        const uint32_t cyc = s_irqCyc;

        uint16_t words = 0;
        if (imu_fifo_level(&s_imu, &words) != IMU_OK) { s_busErr++; wait_ms = 5u; continue; }
        if (words == 0) { wait_ms = 5u; continue; }
        if (words > IMU_FIFO_MAX_WORDS) { words = IMU_FIFO_MAX_WORDS; s_fifoOvr++; }

        /* timeout only for a stuck bus: 48 words = 336 bytes take ~8 ms at 400 kHz */
        if (imu_i2c_read_dma(IMU_FIFO_REG, s_fifo,
                             (uint16_t)(words * IMU_FIFO_WORD_BYTES), 12) != 0) { s_busErr++; wait_ms = 1u; continue; }
        if (words >= 2u * APP_IMU_WATERMARK || (by_timeout && wait_ms != 0u)) {
            wait_ms = 0u;                      /* drain now: let INT1 fall */
            s_drainReads++;
        } else {
            wait_ms = 5u;
        }

        imu_parse_stats_t ps = { 0 };
        const size_t n = imu_fifo_parse(&s_imu, s_fifo, words, s_ev, IMU_FIFO_MAX_WORDS, &ps);
        correct_events(s_ev, n);
        if (ps.gyro) s_lastGyroMs = osKernelGetTickCount();
        s_psTot.gyro += ps.gyro;  s_psTot.acc += ps.acc;  s_psTot.mag += ps.mag;
        s_psTot.mag_dup += ps.mag_dup;  s_psTot.sh_nack += ps.sh_nack;

        /* sample period estimate (see above) */
        const uint8_t fresh = (cyc != lastCyc);           /* a new INT1 edge since the last read */
        lastCyc = cyc;
        if (haveRef) refN += ps.gyro;                     /* gyro samples read after the reference */
        if (fresh) {
            /* INT1 period jitter (SYSTEM.jitter): edges one watermark apart */
            static uint32_t prevEdge;
            const float expP = dtEst * (float)(APP_IMU_WATERMARK / 2u);
            const float per  = (float)(uint32_t)(cyc - prevEdge) * cyc2s;
            if (prevEdge && per < 1.5f * expP)
                s_jitter_us += 0.05f * (fabsf(per - expP) * 1e6f - s_jitter_us);
            prevEdge = cyc;
            if (!haveRef) {
                haveRef = 1; refCyc = cyc; refN = 0;
            } else if ((uint32_t)(cyc - refCyc) >= winCyc && refN > 0u) {
                const float meas = (float)(uint32_t)(cyc - refCyc) * cyc2s / (float)refN;
                if (meas > 0.8f * dtNom && meas < 1.25f * dtNom) {
                    dtEst = haveEst ? dtEst + 0.2f * (meas - dtEst) : meas;
                    haveEst = 1;
                    s_odrHz = 1.0f / dtEst;
                }
                refCyc = cyc; refN = 0;
            }
        }
        const float dt = dtEst;

        const uint32_t c0 = DWT->CYCCNT;
        est_process(&s_est, s_ev, n, dt);
        s_procCycLast = DWT->CYCCNT - c0;
        s_cpu_cyc += s_procCycLast;
        if (s_procCycLast > s_procCycMax) s_procCycMax = s_procCycLast;
        if (words > s_wordsMax) s_wordsMax = words;
    }
}

/* ================================================================== */
/* One frame to every link: UART always, BLE with its own selection */
static void tx_frame(const uint8_t *f, uint16_t len)
{
    (void)tlm_uart_send(f, len, APP_TLM_TX_TIMEOUT_MS);
#if APP_TLM_BLE
    if (len < TLM_OVERHEAD) return;
    switch (f[2]) {
    case TLM_TYPE_STATE: {
        const uint16_t seq = (uint16_t)(f[4] | ((uint16_t)f[5] << 8));   /* tlm_state_t.seq */
        if (tlm_ble_want_state(seq)) (void)tlm_ble_push(f, len);
        break;
    }
    case TLM_TYPE_REPLAY:
        break;                                  /* ~44 kB/s: USB only */
    default:                                    /* INIT, STATUS */
        (void)tlm_ble_push(f, len);
        break;
    }
#endif
}

static void send_status(uint8_t *frame, size_t cap)
{
    tlm_status_t st;
    st.uptime_ms    = osKernelGetTickCount();
    st.irq_count    = s_irqCount;
    st.bus_err      = s_busErr;
    st.fifo_ovr     = s_fifoOvr;
    st.tlm_drop     = s_tlmDrop;
    st.uart_drop    = tlm_uart_dropped();
    st.proc_cyc_max = s_procCycMax;
    st.n_gyro = s_psTot.gyro;  st.n_acc = s_psTot.acc;  st.n_mag = s_psTot.mag;
    st.n_mag_dup = s_psTot.mag_dup;  st.n_sh_nack = s_psTot.sh_nack;
    st.words_max   = (uint16_t)s_wordsMax;
    st.calib_count = (uint16_t)s_est.cal.n;
    st.imu_init    = s_imuInit;
    st.running     = s_est.running;
    st.ble_drop    = tlm_ble_dropped();
    st.ble_mtu     = tlm_ble_mtu();
    st.ble_state   = (uint8_t)tlm_ble_state();
    st.ble_err     = tlm_ble_error();
    const size_t len = tlm_encode(TLM_TYPE_STATUS, &st, sizeof st, frame, cap);
    if (len) tx_frame(frame, (uint16_t)len);
}

static void TelemetryTask(void *arg)
{
    (void)arg;
    static tlm_msg_t msg;
    uint8_t  frame[128];
    uint32_t n = 0;
    (void)tlm_uart_init(&huart1);

    uint32_t nextStatus = osKernelGetTickCount();
    for (;;) {
        /* STATUS at 10 Hz and LED heartbeat: independent of IMU and filter */
        const uint32_t now = osKernelGetTickCount();
        if ((int32_t)(now - nextStatus) >= 0) {
            nextStatus += 100u;
            send_status(frame, sizeof frame);
#ifdef LED2_GPIO_Port
            if ((now / 100u) % 5u == 0u) HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
#endif
        }

#if APP_TLM_BLE
        /* a BLE client has just subscribed (or asked): INIT right away */
        if (g_initLen && tlm_ble_take_init_request())
            (void)tlm_ble_push(g_initFrame, g_initLen);
#endif
        if (osMessageQueueGet(qTlm, &msg, NULL, 10) != osOK) continue;
        tx_frame(msg.bytes, msg.len);
        /* INIT repeated ~once per second for late listeners */
        if (g_initLen && (++n % APP_INIT_EVERY_N) == 0)
            tx_frame(g_initFrame, g_initLen);
    }
}

/* ================================================================== */
static size_t get_init(uint8_t *dst, size_t cap)
{
    const uint16_t n = g_initLen;
    if (!n || n > cap) return 0;
    memcpy(dst, g_initFrame, n);
    return n;
}

static void motor_test_hook(uint8_t m, float duty) { motors_test(m, duty); }

void app_start(void)
{
    dwt_init();
    motors_init();                       /* disarmed, MOTORS_ENABLE = 0 in phase 1 */
    qTlm = osMessageQueueNew(16, sizeof(tlm_msg_t), NULL);
    imu_i2c_bus_init(&hi2c1);

    /* flash image: parameters, name, calibrations (posted to the ImuTask) */
    {
        static const float H[3] = MAG_HARD_IRON;
        (void)app_task_boot(H);
    }
    {
        const fc_config_t fc = { .sensors = 0u, .n_motors = 4u, .motors_built = MOTORS_ENABLE,
                                 .motors_arm = motors_arm, .motor_test = motor_test_hook };
        fc_init(&fc);
    }

    static const osThreadAttr_t aI = { .name = "imu", .priority = osPriorityRealtime,
                                       .stack_size = 3072 };
    static const osThreadAttr_t aT = { .name = "tlm", .priority = osPriorityBelowNormal,
                                       .stack_size = 1024 };
    s_imuTask = osThreadNew(ImuTask, NULL, &aI);
    (void)osThreadNew(TelemetryTask, NULL, &aT);
    {
        app_task_cfg_t ac;
        memset(&ac, 0, sizeof ac);
        ac.hw_rev = APP_HW_REV;
        ac.n_motors = 4u;
        ac.motors_built = MOTORS_ENABLE;
        ac.uid[0] = HAL_GetUIDw0(); ac.uid[1] = HAL_GetUIDw1(); ac.uid[2] = HAL_GetUIDw2();
        ac.get_init = get_init;
        app_task_start(&ac);
    }
    ext_sens_start();
}
