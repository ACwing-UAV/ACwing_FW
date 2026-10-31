/**
 * @file  batt_task.h
 * @brief BattTask: owns the battery ADC and the battery monitor (battery.c).
 *
 * A task of its own, so the battery logic stays out of the flight loop:
 *
 * - every 20 ms it reads the two ADC pins (board binding), runs batt_step()
 *   and publishes the battery state (batt_read(), seqlock);
 * - when the warning level or the presence changes it **signals the flight
 *   core** with fc_batt_signal() (SPSC queue drained in fc_service());
 * - every step it refreshes the heartbeat (fc_batt_heartbeat()): if the task
 *   stops, the flight core treats the battery as unknown;
 * - BATTERY calibration scales arrive from the AppTask through an SPSC queue
 *   (batt_task_post_scales()).
 *
 * The flight core keeps the decisions (land, RTH, events), because it owns
 * the flight state. batt_task_step() is plain C and runs on the host; the
 * RTOS loop is at the end of batt_task.c.
 */
#ifndef BATT_TASK_H
#define BATT_TASK_H

#include <stdint.h>
#include "battery.h"

#define BATT_TASK_PERIOD_MS  20u

typedef struct {
    batt_cfg_t batt;                 /**< sensor gains, scales at boot          */
    /** ADC binding: pin voltages of VBAT and CUR; 0 = sample ready */
    int     (*adc_read)(float *vbat_pin, float *cur_pin);
} batt_task_cfg_t;

/** last flight summary (AppTask persists it for the BATTERY calibration) */
typedef struct {
    uint16_t count;                  /**< flights finished since boot           */
    float    used_mah;               /**< charge used between ARM and DISARM    */
    float    scale_i;                /**< current scale it was counted with     */
} batt_flight_t;

void batt_task_init(const batt_task_cfg_t *cfg);
void batt_task_step(uint32_t now_ms);

/** AppTask -> BattTask: new calibration scales; 0 = queued */
int  batt_task_post_scales(float scale_v, float scale_i);

/** last flight (any task, seqlock) */
void batt_task_last_flight(batt_flight_t *out);

#ifndef HOST_TEST
void batt_task_start(const batt_task_cfg_t *cfg);
#endif

#endif /* BATT_TASK_H */
