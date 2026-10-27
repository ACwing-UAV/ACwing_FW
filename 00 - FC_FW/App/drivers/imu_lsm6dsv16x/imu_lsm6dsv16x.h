/**
 * @file  imu_lsm6dsv16x.h
 * @brief IMU library: LSM6DSV16X (gyro + acc) with the LIS2MDL and the barometer
 *        (LPS22DF on the X-NUCLEO-IKS4A1 in Mode 3, LPS22HB on the ACwing FC rev A)
 *        read by the internal sensor hub and placed in the same FIFO.
 *
 * Built on top of the official ST driver lsm6dsv16x-pid (third_party/), which
 * provides register access; this library adds:
 *   - the complete configuration for this use case (FIFO, filters, sensor hub);
 *   - the LIS2MDL configuration THROUGH the sensor hub (in Mode 3 the
 *     magnetometer is not visible from the MCU);
 *   - a parser for the FIFO read in a single burst (one I2C/DMA transaction)
 *     that returns the events in the temporal order they were written.
 *
 * Dependencies: sensor_bus.h + lsm6dsv16x_reg.[ch]. No HAL, no RTOS.
 */
#ifndef IMU_LSM6DSV16X_H
#define IMU_LSM6DSV16X_H

#include <stdint.h>
#include <stddef.h>
#include "sensor_bus.h"
#include "lsm6dsv16x_reg.h"

#define IMU_I2C_ADDR_7BIT       0x6Bu   /* SA0 = 1 (default IKS4A1: 0xD6/0xD7) */
#define IMU_FIFO_REG            LSM6DSV16X_FIFO_DATA_OUT_TAG   /* 0x78 */
#define IMU_FIFO_WORD_BYTES     7u
#define IMU_FIFO_MAX_WORDS      48u     /* burst buffer: 336 bytes */

typedef enum { IMU_ODR_960HZ = 0, IMU_ODR_1920HZ = 1 } imu_odr_t;

typedef struct {
    imu_odr_t odr;              /* same ODR for gyro and acc                          */
    uint8_t   watermark_words;  /* 4 = 2 gyro + 2 acc -> one interrupt every 2 samples */
    uint8_t   gy_lpf1_bw;       /* lsm6dsv16x_filt_gy_lp1_bandwidth_t                */
    uint8_t   xl_lpf2_enable;
    uint8_t   xl_lpf2_bw;       /* lsm6dsv16x_filt_xl_lp2_bandwidth_t                */
    uint8_t   mag_enable;       /* LIS2MDL via sensor hub (requires IKS4A1 Mode 3)   */
    uint8_t   mag_fast;         /* 0: LIS2MDL 50 Hz / hub 60 Hz; 1: 100 Hz / 120 Hz   */
    uint8_t   baro_enable;      /* LPS22DF/LPS22HB via sensor hub, optional: a missing
                                   part leaves baro_enabled = 0 and is not an error   */
} imu_config_t;

typedef enum { IMU_BARO_NONE = 0, IMU_BARO_LPS22DF = 1, IMU_BARO_LPS22HB = 2 } imu_baro_t;

/* Sensor-hub re-read rate, used to compute the age of the mag sample */
#define IMU_SH_HZ(fast)   ((fast) ? 120.0f : 60.0f)

typedef enum {
    IMU_OK = 0, IMU_ERR_BUS = -1, IMU_ERR_ID = -2, IMU_ERR_MAG_ID = -3,
    IMU_ERR_SH_TIMEOUT = -4, IMU_ERR_ARG = -5
} imu_status_t;

typedef enum { IMU_EV_GYRO = 1, IMU_EV_ACC = 2, IMU_EV_MAG = 3, IMU_EV_BARO = 4 } imu_ev_type_t;

typedef struct {
    uint8_t type;      /* imu_ev_type_t                               */
    float   v[3];      /* gyro rad/s, acc g, mag uT (sensor frame);
                          baro: v[0] Pa, v[1] degC                      */
} imu_event_t;

typedef struct {
    uint32_t words, gyro, acc, mag, mag_dup, sh_nack, empty, unknown;
    uint32_t baro, baro_dup;
} imu_parse_stats_t;

typedef struct {
    stmdev_ctx_t ctx;
    sensor_bus_t bus;
    float        odr_hz;
    uint8_t      mag_enabled;
    uint8_t      last_mag_raw[6];   /* to discard sensor-hub duplicates       */
    uint8_t      have_mag;
    uint8_t      baro_enabled, baro_model, baro_addr8, baro_slot;
    uint8_t      last_baro_raw[5];
    uint8_t      have_baro;
} imu_t;

int32_t imu_init(imu_t *imu, const sensor_bus_t *bus, const imu_config_t *cfg);

/* Number of 7-byte words in the FIFO */
int32_t imu_fifo_level(imu_t *imu, uint16_t *words);

/* Blocking read via sensor_bus (for tests or when DMA is not used) */
int32_t imu_fifo_read_burst(imu_t *imu, uint8_t *raw, uint16_t words);

/* Decodes 'words' words read in a burst from IMU_FIFO_REG.
 * Returns the number of events written to ev (<= cap). Pure function of the data;
 * the only state is the magnetometer duplicate filter. */
size_t  imu_fifo_parse(imu_t *imu, const uint8_t *raw, uint16_t words,
                       imu_event_t *ev, size_t cap, imu_parse_stats_t *st);

#endif /* IMU_LSM6DSV16X_H */
