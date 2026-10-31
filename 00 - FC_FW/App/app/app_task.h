/**
 * @file  app_task.h
 * @brief AppTask: ACwing Pilot protocol v2 (HANDOVER_FW.md) on the drone side.
 *
 * Owns, in one low-priority task (period 5 ms):
 *   - the command dispatcher (ACK cache, core commands forwarded to the flight
 *     core, parameters, calibrations, requests, name);
 *   - the flash store (parameters, name, calibrations + one backup per type);
 *   - the v2 telemetry scheduler: CAPS, FLIGHT, NAV, MOTORS, SYSTEM, CHECKS,
 *     CALIB, EVENT (x3 for ERROR and above), PARAM lists paced to the TX pool;
 *   - the pre-flight checks and the ground information for the flight core.
 *
 * Nothing in here is on the flight path: the flight core (ImuTask) keeps
 * flying and runs the failsafes if this task, BLE or the CPU2 stall.
 * Every frame goes to the high-priority BLE ring (tlm_ble_push_hi).
 */
#ifndef APP_TASK_H
#define APP_TASK_H

#include <stdint.h>
#include <stddef.h>
#include "calib_mgr.h"

#define APP_FW_MAJOR  0u
#define APP_FW_MINOR  5u
#define APP_FW_PATCH  0u

typedef struct {
    char     hw_rev;              /**< CAPS hwRev: 'N' Nucleo bench, 'A' ACwing FC rev A */
    uint8_t  n_motors;
    uint8_t  motors_built;
    uint32_t uid[3];              /**< MCU unique id -> serial "ACW-xxxxxxxx"         */
    /** INIT frame of the estimator (copy; 0 if not ready yet) */
    size_t (*get_init)(uint8_t *dst, size_t cap);
    uint8_t  batt_monitor;        /**< a BattTask runs: POWER, CHECKS item 1, BATTERY calibration */
    uint16_t board_sensors;       /**< CAPS bits of the board itself (CURRENT, FLASH_LOG)        */
    /** flash log used %, 0xFF = no log; may be NULL */
    uint8_t (*log_pct)(void);
} app_task_cfg_t;

/** Before the scheduler starts: loads the flash image (parameters, name,
 * calibrations) and posts the saved calibrations to the ImuTask
 * (fsh_apply_post). Returns the calibration set in effect. */
const cal_set_t *app_task_boot(const float mag_hi_default[3]);

void app_task_start(const app_task_cfg_t *cfg);   /**< app_init + the RTOS task */

/** protocol logic, also used by the host test */
void app_init(const app_task_cfg_t *cfg);
void app_step(uint32_t now_ms);

#endif /* APP_TASK_H */
