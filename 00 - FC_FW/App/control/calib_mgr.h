/**
 * @file  calib_mgr.h
 * @brief Sensor calibrations of protocol v2 (HANDOVER_FW.md §10).
 *
 * Runs in the AppTask. The ImuTask streams raw samples (calibration removed)
 * while a calibration is active; fits run here, never in the flight loop.
 * Results go to RAM through the apply hook (the estimator takes them in the
 * ImuTask context); CAL_SAVE makes the AppTask write them to flash and keep
 * the previous set as the backup for CAL_RESTORE.
 *
 * Available on the current hardware (LSM6DSV16X + LIS2MDL):
 *   1 ACCEL six positions, 2 LEVEL trim, 3 GYRO bias, 4 COMPASS (FC compass).
 * Not built (REJECTED / SENSOR_MISSING): 5 FLOW_TOF, 6 BATTERY, 8 STICKS, and
 * 7 MOTORS while the motor outputs are disabled.
 *
 * Host-testable: no HAL, no RTOS.
 */
#ifndef CALIB_MGR_H
#define CALIB_MGR_H

#include <stdint.h>
#include <stddef.h>

/* ---- calibration sets (RAM and flash) ---- */
typedef struct { float off[3], scale[3]; } cal_accel_t;     /**< g: a = scale * (raw - off) */
typedef struct { float roll_deg, pitch_deg; } cal_level_t;  /**< trim added to the attitude setpoint */
typedef struct { float bias[3]; } cal_gyro_t;                /**< rad/s: w = raw - bias */
typedef struct { float hi[3], scale[3]; } cal_compass_t;    /**< uT body: m = scale * (raw - hi) */
typedef struct { uint8_t rev_mask, pad[3]; } cal_motors_t;
typedef struct { float scale_v, scale_i; } cal_battery_t;  /**< vbat = pin*scale_v/10, I = I_nom*scale_i */

typedef struct {
    cal_accel_t   accel;
    cal_level_t   level;
    cal_gyro_t    gyro;
    cal_compass_t compass;
    cal_motors_t  motors;
    cal_battery_t battery;
    uint32_t      done_mask;     /**< bit (type-1): this calibration was done and saved */
} cal_set_t;

void cal_set_defaults(cal_set_t *s, const float mag_hi_default[3]);
/** copies the part of 'src' that belongs to 'type' into 'dst' */
void cal_copy_type(cal_set_t *dst, const cal_set_t *src, uint8_t type);

/* ---- samples from the ImuTask (raw: calibration removed) ---- */
typedef struct {
    float   acc[3];    /**< g, body */
    float   gyro[3];   /**< rad/s, body */
    float   mag[3];    /**< uT, body (axis map applied, no hard/soft iron) */
    float   q[4];      /**< attitude, body -> world NWU [w x y z] */
    uint8_t mag_new;
    uint8_t running;   /**< estimator running */
} cal_sample_t;

/** apply hook: puts 'type' of 'set' in effect (RAM), in the estimator context */
typedef void (*cal_apply_fn)(uint8_t type, const cal_set_t *set);

/** battery readings for the BATTERY calibration (AppTask context):
 *  vbat with the scale in effect, and the last flight's charge with the
 *  current scale it was counted with */
typedef void (*cal_batt_fn)(float *vbat, float *last_used_mah, float *last_scale_i);

typedef struct {
    uint16_t     sensors;        /**< CAPS bits */
    uint8_t      motors_built;
    cal_apply_fn apply;
    cal_batt_fn  batt;           /**< NULL: BATTERY -> SENSOR_MISSING */
} cal_config_t;

/** active: the set in effect (RAM), owned by the caller. DONE applies the new
 * values to it (and to the estimator through the hook); ABORT, a failure or a
 * lost app link put the previous values back. */
void    cal_init(const cal_config_t *cfg, cal_set_t *active);
void    cal_set_sensors(uint16_t sensors);       /**< expansion board detected later */

uint8_t cal_start(uint8_t type, uint8_t option, uint8_t armed, uint32_t now_ms, uint8_t *reason);
uint8_t cal_capture(uint32_t now_ms, uint8_t *reason);
uint8_t cal_abort(uint8_t *reason);
uint8_t cal_set_ref(uint8_t type, uint8_t index, float value, uint8_t *reason);
/** CAL_SAVE validation: returns the ACK; on OK the caller writes the result
 * (cal_result()) to flash and then calls cal_saved() */
uint8_t cal_save_check(uint8_t *reason);
const cal_set_t *cal_result(void);
uint8_t cal_type(void);
void    cal_saved(void);         /**< state SAVED (one frame), then IDLE */

void    cal_feed(const cal_sample_t *s, uint32_t now_ms);
void    cal_tick(uint32_t now_ms);

uint8_t cal_active(void);        /**< a calibration exists (any state but IDLE) */
uint8_t cal_running(void);       /**< active and not DONE: blocks arming / other starts */
uint8_t cal_wants_samples(void);
/** 1 if the state/step changed since the last call (send a CALIB frame now) */
uint8_t cal_take_changed(void);
/** 1 + type once when a calibration failed (EVENT 51), 0 otherwise */
uint8_t cal_take_failed(void);
/** CALIB frame for the current calibration; 0 if none */
size_t  cal_frame(uint8_t *out, size_t cap);

#endif /* CALIB_MGR_H */
