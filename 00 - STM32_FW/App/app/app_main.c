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
 *
 * Phase 2: BLE (stack on the Cortex-M0+) is added as a TelemetryTask transport;
 * ImuTask remains the only high-priority task.
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
#include "so3.h"
#include "tlm_build.h"
#include <string.h>
#include <math.h>

extern I2C_HandleTypeDef  hi2c1;     /* PB8 SCL (D15) / PB9 SDA (D14)          */
extern UART_HandleTypeDef huart1;    /* PB6/PB7 -> ST-LINK virtual COM port     */

/* PA15 = D5 = LSM6DSV16X INT1 on the IKS4A1. CubeMX label "IMU_INT1". */
#ifndef IMU_INT1_Pin
#error "In CubeMX, assign the label IMU_INT1 to pin PA15 (GPIO_EXTI15)"
#endif

#define FLAG_IMU_IRQ 0x0001u

/* ------------------------------------------------------------------ */
static osThreadId_t       s_imuTask;
static osMessageQueueId_t qTlm;
static volatile uint32_t  s_irqCyc;
static volatile uint32_t  s_irqCount, s_tlmDrop, s_busErr, s_fifoOvr;
/* Bring-up diagnostics (Live Expressions in the debugger) */
static volatile uint32_t  s_procCycMax, s_procCycLast;   /* CPU cycles of parse+est_process */
static volatile uint32_t  s_wordsMax;
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

static uint8_t          g_initFrame[TLM_FRAME_MAX];
static volatile uint16_t g_initLen;

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
static void ImuTask(void *arg)
{
    (void)arg;
    sensor_bus_t bus;
    imu_i2c_make_bus(&bus, &hi2c1, IMU_I2C_ADDR_7BIT);

    const imu_config_t icfg = {
        .odr = APP_IMU_ODR, .watermark_words = APP_IMU_WATERMARK,
        .gy_lpf1_bw = APP_GY_LPF1_BW, .xl_lpf2_enable = APP_XL_LPF2_ENABLE,
        .xl_lpf2_bw = APP_XL_LPF2_BW, .mag_enable = 1, .mag_fast = APP_MAG_FAST
    };
    for (;;) {
        const int32_t st = imu_init(&s_imu, &bus, &icfg);
        s_imuInit = (int8_t)st;           /* visible in the STATUS packet */
        if (st == IMU_OK) break;
#ifdef LED1_GPIO_Port
        HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
#endif
        osDelay(200);
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
    static const float M[9] = MAG_AXIS_MAP, H[3] = MAG_HARD_IRON;
    memcpy(ec.mag_map, M, sizeof M);
    memcpy(ec.mag_hard_iron, H, sizeof H);
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
    est_init(&s_est, &ec, on_state, on_init, NULL);
    s_est.trace_enable = APP_TLM_REPLAY;
    ctrl_init();

    const float dtNom = 1.0f / s_imu.odr_hz;
    const float cyc2s = 1.0f / (float)SystemCoreClock;
    uint32_t prevCyc = 0;
    uint8_t  first = 1;

    for (;;) {
        /* timeout = safety net in case an INT1 edge is missed */
        (void)osThreadFlagsWait(FLAG_IMU_IRQ, osFlagsWaitAny, 5);
        const uint32_t cyc = s_irqCyc;

        uint16_t words = 0;
        if (imu_fifo_level(&s_imu, &words) != IMU_OK) { s_busErr++; continue; }
        if (words == 0) continue;
        if (words > IMU_FIFO_MAX_WORDS) { words = IMU_FIFO_MAX_WORDS; s_fifoOvr++; }

        if (imu_i2c_read_dma(IMU_FIFO_REG, s_fifo,
                             (uint16_t)(words * IMU_FIFO_WORD_BYTES), 3) != 0) { s_busErr++; continue; }

        imu_parse_stats_t ps = { 0 };
        const size_t n = imu_fifo_parse(&s_imu, s_fifo, words, s_ev, IMU_FIFO_MAX_WORDS, &ps);
        s_psTot.gyro += ps.gyro;  s_psTot.acc += ps.acc;  s_psTot.mag += ps.mag;
        s_psTot.mag_dup += ps.mag_dup;  s_psTot.sh_nack += ps.sh_nack;

        /* measured sample period: time between two IRQs / gyro samples read.
         * Absorbs the tolerance of the sensor's internal oscillator. */
        float dt = dtNom;
        if (!first && ps.gyro > 0) {
            const float meas = (float)(cyc - prevCyc) * cyc2s / (float)ps.gyro;
            if (meas > 0.8f*dtNom && meas < 1.25f*dtNom) dt = meas;
        }
        first = 0;
        prevCyc = cyc;

        const uint32_t c0 = DWT->CYCCNT;
        est_process(&s_est, s_ev, n, dt);
        s_procCycLast = DWT->CYCCNT - c0;
        if (s_procCycLast > s_procCycMax) s_procCycMax = s_procCycLast;
        if (words > s_wordsMax) s_wordsMax = words;
    }
}

/* ================================================================== */
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
    const size_t len = tlm_encode(TLM_TYPE_STATUS, &st, sizeof st, frame, cap);
    if (len) (void)tlm_uart_send(frame, (uint16_t)len, APP_TLM_TX_TIMEOUT_MS);
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

        if (osMessageQueueGet(qTlm, &msg, NULL, 10) != osOK) continue;
        (void)tlm_uart_send(msg.bytes, msg.len, APP_TLM_TX_TIMEOUT_MS);
        /* INIT repeated ~once per second for late listeners */
        if (g_initLen && (++n % APP_INIT_EVERY_N) == 0)
            (void)tlm_uart_send(g_initFrame, g_initLen, APP_TLM_TX_TIMEOUT_MS);
    }
}

/* ================================================================== */
void app_start(void)
{
    dwt_init();
    motors_init();                       /* disarmed, MOTORS_ENABLE = 0 in phase 1 */
    qTlm = osMessageQueueNew(16, sizeof(tlm_msg_t), NULL);

    static const osThreadAttr_t aI = { .name = "imu", .priority = osPriorityRealtime,
                                       .stack_size = 2048 };
    static const osThreadAttr_t aT = { .name = "tlm", .priority = osPriorityBelowNormal,
                                       .stack_size = 1024 };
    s_imuTask = osThreadNew(ImuTask, NULL, &aI);
    (void)osThreadNew(TelemetryTask, NULL, &aT);
}
