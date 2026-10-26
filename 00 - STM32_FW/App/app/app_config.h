/**
 * @file  app_config.h
 * @brief Phase-1 configuration: P-NUCLEO-WB55 (NUCLEO-WB55RG) + X-NUCLEO-IKS4A1
 *        in Mode 3 (LSM6DSV16X is the sensor-hub master, LIS2MDL sits behind it).
 *        Every tunable parameter of the application lives here.
 */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#define DEG2RAD_F 0.017453292519943295f

/* ---------------- IMU ---------------- */
/* Bring-up: 960 Hz with I2C at 400 kHz. 1920 Hz requires Fast-mode Plus (1 MHz);
 * try it only after checking the bus edges with an oscilloscope: the IKS4A1 has
 * level translators and 4k7 pull-ups. The pipeline adapts automatically. */
#define APP_IMU_ODR            IMU_ODR_960HZ
#define APP_IMU_WATERMARK      4u      /* FIFO words: 2 gyro + 2 acc -> IRQ at 960 Hz */
#define APP_GY_LPF1_BW         LSM6DSV16X_GY_LIGHT    /* gyro anti-alias filter: to be tuned */
#define APP_XL_LPF2_ENABLE     1
#define APP_XL_LPF2_BW         LSM6DSV16X_XL_STRONG
#define APP_MAG_FAST           1       /* LIS2MDL 100 Hz, hub 120 Hz: halves the age of mag data */

/* ---------------- rates (derived) ---------------- */
#define APP_STEP_HZ            960u    /* propagation + rate loop                */
#define APP_STEPS_PER_ACC      4u      /* -> 240 Hz acc update + attitude loop   */

/* ---------------- static calibration ---------------- */
#define APP_CALIB_SAMPLES      100u    /* mag samples at ~50 Hz -> ~2 s standing still */
#define APP_CALIB_GYRO_STILL   (5.0f * DEG2RAD_F)

/* ---------------- IEKF ----------------
 * State: attitude (3) + gyro bias (3). IEKF_ESTIMATE_BIAS = 0 -> 3 states.
 * sigma_gyro is the EFFECTIVE gyro process noise (Q = sigma^2 * dt).
 * It must cover not only the sensor noise (~0.003 deg/s/rtHz from the datasheet)
 * but also what the model ignores: scale-factor error, misalignment,
 * acceleration sensitivity. In simulation with 1% scale error and 1 deg of
 * misalignment: sigma 0.1 -> RMSE 1.17 deg, sigma 0.5 -> 0.42 deg.
 * Hence 0.5: it can be lowered only after calibrating gyro scale and
 * misalignment (see README.md, "Embedded improvements"). */
#define IEKF_SIGMA_GYRO        (0.5f * DEG2RAD_F)
#define IEKF_SIGMA_BIAS        1e-4f
#define IEKF_SIGMA_VEC_MIN     0.01f
#define IEKF_ACC_TOL           0.08f
#define IEKF_MAG_TOL           0.08f
#define IEKF_P0_ATT            ((2.0f * DEG2RAD_F) * (2.0f * DEG2RAD_F))
#define IEKF_P0_BIAS           ((0.2f * DEG2RAD_F) * (0.2f * DEG2RAD_F))
#define IEKF_ESTIMATE_BIAS     1
#define IEKF_ADAPTIVE          1
#define IEKF_TILT_COMP_MAG     1

/* ---------------- robustness (external accelerations / magnetic disturbances) ----
 * Accelerometer: R inflated by NIS/gate when the innovation is inconsistent with
 * the covariance (this also catches accelerations PERPENDICULAR to gravity, which
 * the magnitude check barely sees), with a memory of IEKF_INFL_TAU_S.
 * Magnetometer: de-weighted when the measured field-gravity angle departs from
 * the calibration value (independent of the estimate: it does not slow down yaw
 * recovery).
 * Stillness detector: board at rest -> the accelerometer is trusted again at once. */
#define IEKF_ACC_NIS_GATE      4.0f
#define IEKF_MAG_NIS_GATE      0.0f                  /* not recommended, see iekf.h */
#define IEKF_INFL_TAU_S        0.5f
#define IEKF_DIP_TOL           (5.0f * DEG2RAD_F)
#define IEKF_DECOUPLE          0                     /* tried: does not help, see README */
/* "Lockout" protection: without it, a true attitude error during motion inflates
 * R_acc forever (recovery > 20 s). The inflation is capped and, if the innovation
 * is persistent in the world frame (a true error, not shaking), after
 * IEKF_LOCKOUT_S the covariance is reopened and the accelerometer trusted again. */
#define IEKF_RESET_JACOBIAN    1
#define IEKF_INFL_MAX          100.0f
#define IEKF_LOCKOUT_S         0.5f
#define IEKF_LOCKOUT_BUMP      ((10.0f * DEG2RAD_F) * (10.0f * DEG2RAD_F))
#define IEKF_GRACE_S           0.25f
#define IEKF_PERSIST_TAU_S     1.0f
#define IEKF_PERSIST_RATIO     0.6f
#define IEKF_PERSIST_MIN       0.02f

/* ---------------- diagnostic comparison profile ----------------
 * 1 = filter as in the earlier "works better" version: no inflation cap,
 *     no lockout protection, no error reset. */
#define APP_PROFILE_PREVIOUS   0
#if APP_PROFILE_PREVIOUS
#undef  IEKF_RESET_JACOBIAN
#define IEKF_RESET_JACOBIAN    0
#undef  IEKF_INFL_MAX
#define IEKF_INFL_MAX          1e4f
#undef  IEKF_LOCKOUT_S
#define IEKF_LOCKOUT_S         0.0f
#endif
#define APP_STILL_ACC_TOL      0.02f
#define APP_STILL_GYRO_THR     (3.0f * DEG2RAD_F)
#define APP_STILL_TIME_S       0.3f

/* ---------------- comparison with the original MATLAB filter ----------------
 * 1 = disables the extensions that the original iekfAttitude.m does not have
 *     (NIS inflation, dip check, stillness detector, variance due to the age of
 *     the mag sample, error reset). The embedded filter must then match the
 *     original up to float32 precision: this proves that the C code implements
 *     the equations validated in MATLAB.
 * 0 = full configuration: the comparison quantifies the effect of the extensions. */
#define APP_COMPARE_ORIGINAL   0

/* ---------------- magnetometer -> body (LSM6DSV16X frame) ----------------
 * Calibration of 2026-10-07 (calibMagAxes.m on the board): LIS2MDL axes rotated
 * by 180 deg about x w.r.t. the LSM6DSV16X (y and z flipped).
 * Large hard-iron (~194 uT, ~4x the Earth field): recalibrate whenever anything
 * near the sensor changes (cables, battery, screws, motors). */
#define MAG_AXIS_MAP   { 1.0f, 0.0f, 0.0f,   0.0f, -1.0f, 0.0f,   0.0f, 0.0f, -1.0f }
#define MAG_HARD_IRON  { 31.61f, 16.44f, -26.23f }   /* uT, body frame */

/* ---------------- telemetry (phase 1: ST-LINK virtual COM port) ---------------- */
#define APP_TLM_DECIMATION     1u      /* 1 = every acc update (240 Hz), ~18.5 kB/s */
#define APP_TLM_DECIM_REPLAY   8u      /* with REPLAY on, STATE drops to 30 Hz:
                                          REPLAY already carries q and bias at 240 Hz */
#define APP_TLM_TX_TIMEOUT_MS  20u     /* max wait for the UART DMA before dropping */
#define APP_INIT_EVERY_N       480u    /* repeat INIT ~once per second (2 frames per window) */
#define APP_TLM_REPLAY         0       /* 1 only for the MATLAB comparison: raises the
                                          telemetry from ~19 to ~44 kB/s, too much for
                                          the real-time viewer -> record with
                                          tlmCapture.m and analyse offline */

/* ---------------- telemetry over BLE (phase 2, see the README) ----------------
 * Same frames as the UART, as a byte stream on a GATT notification. The UART
 * keeps working unchanged (REPLAY and comparison stay USB-only). */
#define APP_TLM_BLE            1       /* 1 once STM32_WPAN (BLE) is enabled in CubeMX
                                          and the CPU2 wireless stack is flashed      */
#define APP_BLE_STATE_DECIM    4u      /* STATE over BLE: 240/4 = 60 Hz (~5.4 kB/s in
                                          total with STATUS and INIT). Changeable at run
                                          time with the CMD characteristic.            */
#define APP_BLE_FLUSH_MS       10u     /* TX thread period: groups frames in one
                                          notification when the MTU allows it          */
#define APP_BLE_CONN_INT_MIN   12u     /* x1.25 ms = 15 ms (iOS minimum)               */
#define APP_BLE_CONN_INT_MAX   24u     /* x1.25 ms = 30 ms (iOS: max >= min + 15 ms)   */
#define APP_BLE_SUPERV_TIMEOUT 400u    /* x10 ms   = 4 s                               */

#endif /* APP_CONFIG_H */
