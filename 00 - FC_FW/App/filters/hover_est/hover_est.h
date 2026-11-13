/**
 * @file  hover_est.h
 * @brief Online estimate of the hover thrust (ADR 0010 §2): 1-state Kalman filter.
 *
 * State: t_h, the normalised thrust (0..1, ADR 0010 §1) that holds the drone in a hover. In the vertical axis, with
 * t_c = t cos(tilt) the vertical part of the commanded collective,
 *
 *     a_up = g (t_c / t_h - 1)            measurement: the vertical acceleration, accelerometer bias removed
 *     H    = -g t_c / t_h^2
 *
 * Predict: P += q dt (slow changes: battery mostly removed by the mixer, tray, damage). Update at 50 Hz with
 * R = sigma_a^2; an innovation with NIS > 9 is rejected and R is inflated x4 for 0.5 s, so impacts and hard
 * manoeuvres do not pull the estimate. The caller decides when the model holds (in air, not near the floor, tilt and
 * collective not saturated) and aligns t_c with its effect (one actuator lag earlier).
 *
 * Host-testable: no HAL, no RTOS.
 */
#ifndef HOVER_EST_H
#define HOVER_EST_H

#include <stdint.h>

#define HOVER_EST_MIN      0.25f    /**< clamp of the estimate                        */
#define HOVER_EST_MAX      0.85f
#define HOVER_EST_WARN_LO  0.35f    /**< outside: wrong tray parameter, lost prop... */
#define HOVER_EST_WARN_HI  0.75f

typedef struct {
    float   th;          /**< estimate                                   */
    float   P;           /**< variance                                   */
    float   q;           /**< process noise, 1/s                          */
    float   r;           /**< measurement variance, (m/s^2)^2             */
    float   gate;        /**< NIS threshold                               */
    float   infl_s;      /**< remaining time with R inflated              */
    float   nis;         /**< last NIS                                    */
    uint32_t n_upd, n_rej;
} hover_est_t;

/** seed (parameter hoverThr, or the value learned in an earlier flight); sigma0 = initial 1-sigma */
void    hover_est_init(hover_est_t *e, float seed, float sigma0);
void    hover_est_predict(hover_est_t *e, float dt);
/** a_up m/s^2 (+ up, gravity and accelerometer bias removed), t_c vertical collective, dt since the last update;
 *  returns 1 if used, 0 if rejected by the gate */
uint8_t hover_est_update(hover_est_t *e, float a_up, float t_c, float dt);
/** 1 when the estimate is outside the plausible band */
static inline uint8_t hover_est_abnormal(const hover_est_t *e) { return (uint8_t)(e->th < HOVER_EST_WARN_LO || e->th > HOVER_EST_WARN_HI); }

#endif /* HOVER_EST_H */
