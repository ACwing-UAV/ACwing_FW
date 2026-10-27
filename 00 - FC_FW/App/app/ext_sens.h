/**
 * @file  ext_sens.h
 * @brief Expansion sensors on the Qwiic bus (ACwing Flow rev A): VL53L1X ToF
 *        and the optical-flow bridge. Own low-priority task (ExtTask).
 *
 * Logic (ext_init / ext_step) is portable and host-tested: the devices are
 * probed at boot and every 5 s while one is missing (the board can be plugged
 * in later), polled every 10 ms, declared lost after 50 consecutive bus
 * errors, and every new sample is handed to the flight-control context
 * through a lock-free SPSC queue (ext_take). The bus bindings are in
 * bsp/ext_i2c.c; the RTOS task shell is at the end of ext_sens.c.
 */
#ifndef EXT_SENS_H
#define EXT_SENS_H

#include <stdint.h>
#include "vl53l1x.h"
#include "sensor_bus.h"

#define EXT_PERIOD_MS   10u
#define EXT_PROBE_MS    5000u
#define EXT_ERR_LIMIT   50u      /* consecutive errors: device lost, probe again */

enum { EXT_TOF = 1, EXT_FLOW = 2 };

typedef struct {
    uint8_t  kind;            /* EXT_TOF / EXT_FLOW */
    uint8_t  status;          /* ToF range status (0 valid) / flow STATUS byte */
    uint16_t tof_mm;
    int16_t  dx, dy;          /* flow counts (0 without new motion) */
    uint8_t  squal;
    float    dt_s;            /* flow: time covered by dx/dy, 0 = status only */
    uint32_t t_ms;
} ext_sample_t;

typedef struct {
    vl53_bus_t   tof;
    sensor_bus_t flow;
} ext_buses_t;

/* ---- logic (ExtTask context; host-testable) ---- */
void     ext_init(const ext_buses_t *buses);
void     ext_step(uint32_t now_ms);       /* one 10 ms cycle */

/* ---- RTOS shell (firmware only) ---- */
void     ext_sens_start(void);            /* creates the task (after imu_i2c_bus_init) */

/* ---- readers ---- */
int      ext_take(ext_sample_t *s);       /* flight context: 1 if a sample was taken */
uint16_t ext_sensors(void);               /* PV2_S_TOF | PV2_S_FLOW currently detected */
uint32_t ext_errors(void);

#endif /* EXT_SENS_H */
