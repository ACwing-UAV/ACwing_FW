/**
 * @file  battery.h
 * @brief Battery monitor: voltage, current, consumed charge, cell count,
 *        state of charge, flight time left and the warning level used by the
 *        battery failsafes (HANDOVER_FW.md §4 POWER, §6, §9 item 1, §10.6).
 *
 * Inputs are the voltages at the two ADC pins (the board binding converts the
 * counts with VREFINT). The module runs in the flight-control context
 * (ImuTask, 50 Hz) and publishes a snapshot that any task can read.
 *
 * - **Voltage.** `vbat = pin × scaleV / 10` (scaleV 110 = the 100k/10k divider
 *   of the ACwing FC). BATTERY calibration corrects scaleV.
 * - **Current.** `I = pin / sens × scaleI`, sens 0.1 V/A on the ACwing ESC 1S
 *   (TSC213 + 2 mΩ). BATTERY calibration corrects scaleI from the charge the
 *   charger put back.
 * - **Cells.** Parameter battCells, or detected once per battery connection
 *   (disarmed): `ceil(vbat / 4.35 V)`, 1…6.
 * - **Warning level.** From the *sagged* (under load) cell voltage, low-pass
 *   filtered: a level is entered after the cell voltage stayed below its
 *   threshold for 2 s; it is left only when disarmed (or the battery is
 *   replaced), so a level reached in flight does not flicker when the
 *   throttle is reduced.
 * - **State of charge.** Rest voltage `V + I·R` (R estimated on line from the
 *   load steps) through a LiPo OCV curve, anchored while the battery rests,
 *   then coulomb counting when the capacity (battCapacity) is known.
 *
 * Host-testable: no HAL, no RTOS.
 */
#ifndef BATTERY_H
#define BATTERY_H

#include <stdint.h>

#define BATT_CELL_DETECT_V   4.35f   /**< max cell voltage used to count cells  */
#define BATT_PRESENT_V       2.5f    /**< pack above this = battery connected   */
#define BATT_ABSENT_V        1.5f
#define BATT_LEVEL_MS        2000u   /**< below a threshold this long -> level  */
#define BATT_RECOVER_MS      2000u   /**< disarmed, above threshold + hyst      */
#define BATT_RECOVER_HYST    0.05f   /**< V/cell                                */
#define BATT_RESERVE_SOC     0.20f   /**< charge kept for landing (remain)      */
#define BATT_R_CELL_DEFAULT  0.040f  /**< Ω per cell before any estimate        */

typedef struct {
    float   scale_v;       /**< ×10 divider ratio (110 = ×11)              */
    float   scale_i;       /**< current multiplier (1 = nominal sensor)     */
    float   sens_v_per_a;  /**< current sensor gain, V/A                    */
    uint8_t has_current;   /**< a current sensor is wired                   */
} batt_cfg_t;

/** published state */
typedef struct {
    uint8_t  present;      /**< a battery is connected                       */
    uint8_t  cells;        /**< 0 = not known                                */
    uint8_t  level;        /**< PV2_BATT_OK / LOW / CRITICAL / EMERGENCY     */
    uint8_t  pct;          /**< 0…100                                        */
    float    vbat;         /**< V, pack, filtered (τ 0.1 s)                  */
    float    current;      /**< A, filtered (τ 0.05 s); 0 without a sensor   */
    float    used_mah;     /**< since power-on                               */
    float    cell_v;       /**< V, sagged, filtered (τ 0.5 s): failsafes, CHECKS */
    float    cell_rest_v;  /**< V, sag compensated: state of charge          */
    float    r_cell;       /**< Ω per cell, estimate                         */
    float    soc;          /**< 0…1                                          */
    float    remain_s;     /**< s of flight left, < 0 = n/a                  */
    float    vbat_raw;     /**< V, last sample, unfiltered (calibration)     */
} batt_state_t;

/** thresholds and settings (from the parameters) */
typedef struct {
    float   warn, crit, emerg;   /**< V per cell                              */
    float   capacity_mah;        /**< 0 = unknown                             */
    uint8_t cells;               /**< 0 = auto                                */
} batt_limits_t;

void batt_init(const batt_cfg_t *cfg);
void batt_set_scales(float scale_v, float scale_i);
void batt_get_cfg(batt_cfg_t *out);

/** one sample: pin voltages, time step, armed flag (flight-control context) */
void batt_step(float pin_v, float cur_pin_v, uint32_t dt_ms, uint8_t armed, const batt_limits_t *lim);

/** state as computed by the last step (same context) */
const batt_state_t *batt_state(void);

/** seqlock snapshot for other tasks; keeps *out on a race */
void batt_read(batt_state_t *out);

/** LiPo open-circuit voltage → state of charge 0…1 (exposed for the tests) */
float batt_soc_from_ocv(float cell_v);

#endif /* BATTERY_H */
