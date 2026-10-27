/**
 * @file  ext_sens.h
 * @brief Expansion sensors on the Qwiic bus (ACwing Flow rev A): VL53L1X ToF
 *        and the optical-flow bridge. Own low-priority task (ExtTask).
 *
 * The task probes the devices at boot and every 5 s while one is missing
 * (the board can be plugged in later), polls them every 10 ms and hands each
 * new sample to the flight-control context through a lock-free SPSC queue
 * (ext_take). Bus errors never block the ImuTask: every transaction takes the
 * I2C lock with priority inheritance and lasts a few hundred microseconds.
 */
#ifndef EXT_SENS_H
#define EXT_SENS_H

#include <stdint.h>

enum { EXT_TOF = 1, EXT_FLOW = 2 };

typedef struct {
    uint8_t  kind;            /* EXT_TOF / EXT_FLOW */
    uint8_t  status;          /* ToF range status (0 valid) / flow STATUS byte */
    uint16_t tof_mm;
    int16_t  dx, dy;          /* flow counts */
    uint8_t  squal;
    float    dt_s;            /* flow: time covered by dx/dy */
    uint32_t t_ms;
} ext_sample_t;

void     ext_sens_start(void);            /* creates the task (after imu_i2c_bus_init) */
int      ext_take(ext_sample_t *s);       /* flight context: 1 if a sample was taken */
uint16_t ext_sensors(void);               /* PV2_S_TOF | PV2_S_FLOW currently detected */
uint32_t ext_errors(void);

#endif /* EXT_SENS_H */
