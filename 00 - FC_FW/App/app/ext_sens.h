/**
 * @file  ext_sens.h
 * @brief Expansion sensors on the Qwiic bus. Own low-priority task (ExtTask).
 *        ACwing Flow rev A: VL53L1X ToF and the optical-flow bridge.
 *        ACwing GPS: u-blox M10 (DDC, 0x42) and its IIS2MDC compass (0x1E).
 *
 * Logic (ext_init / ext_step) is portable and host-tested: the devices are
 * probed at boot and every 5 s while one is missing (the board can be plugged
 * in later), polled every 10 ms, declared lost after 50 consecutive bus
 * errors, and every new sample is handed to the flight-control context
 * through a lock-free SPSC queue (ext_take). The GNSS solution is a
 * latest-value slot instead (seqlock, ext_gnss_read): 5 Hz, only the newest
 * fix matters. A device whose bus has no read function is not fitted. The
 * bus bindings are in
 * bsp/ext_i2c.c; the RTOS task shell is at the end of ext_sens.c.
 */
#ifndef EXT_SENS_H
#define EXT_SENS_H

#include <stdint.h>
#include "vl53l1x.h"
#include "sensor_bus.h"
#include "ubx.h"

#define EXT_PERIOD_MS   10u
#define EXT_PROBE_MS    5000u
#define EXT_ERR_LIMIT   50u      /**< consecutive errors: device lost, probe again */
#define EXT_GNSS_SILENT_MS 2000u /**< no NAV-PVT for this long: GNSS lost, probe again */
enum { EXT_TOF = 1, EXT_FLOW = 2, EXT_MAG = 3 };

typedef struct {
    uint8_t  kind;            /**< EXT_TOF / EXT_FLOW */
    uint8_t  status;          /**< ToF range status (0 valid) / flow STATUS byte */
    uint16_t tof_mm;
    int16_t  dx, dy;          /**< flow counts (0 without new motion) */
    uint8_t  squal;
    float    dt_s;            /**< flow: time covered by dx/dy, 0 = status only */
    int16_t  mag[3];          /**< EXT_MAG: IIS2MDC raw, sensor axes, IIS2MDC_MG_PER_LSB */
    uint32_t t_ms;
} ext_sample_t;

typedef struct {
    vl53_bus_t   tof;
    sensor_bus_t flow;
    sensor_bus_t gnss;        /**< M10 DDC (8-bit registers)                        */
    sensor_bus_t mag;         /**< IIS2MDC of the GPS board                         */
} ext_buses_t;

/** latest GNSS solution */
typedef struct {
    ubx_pvt_t pvt;
    uint32_t  t_ms;           /**< ExtTask time of reception                        */
    uint32_t  count;          /**< NAV-PVT received since boot                      */
} ext_gnss_t;

/* ---- logic (ExtTask context; host-testable) ---- */
void     ext_init(const ext_buses_t *buses);
void     ext_step(uint32_t now_ms);       /**< one 10 ms cycle */

/* ---- RTOS shell (firmware only) ---- */
void     ext_sens_start(void);            /**< creates the task (after imu_i2c_bus_init) */

/* ---- readers ---- */
int      ext_take(ext_sample_t *s);       /**< flight context: 1 if a sample was taken */
uint16_t ext_sensors(void);               /**< PV2_S_TOF | FLOW | GNSS | MAG_GPS currently detected */
/** any task: 1 with *g filled if a solution was ever received (check t_ms for its age) */
int      ext_gnss_read(ext_gnss_t *g);
uint32_t ext_errors(void);

#endif /* EXT_SENS_H */
