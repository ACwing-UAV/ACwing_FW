/**
 * @file  iis2mdc.h
 * @brief IIS2MDC magnetometer on a direct I2C bus (the compass of the ACwing GPS board, 7-bit address 0x1E).
 *
 * The FC compass (LIS2MDL on the bench, IIS2MDC on rev A) sits behind the LSM6DSV16X sensor hub and is driven by
 * imu_lsm6dsv16x.c; this driver is for a part on the Qwiic bus. Same register map as the LIS2MDL:
 *
 *   0x4F WHO_AM_I = 0x40
 *   0x60 CFG_REG_A  bit7 temperature compensation, bit5 soft reset, ODR bits 3..2, mode bits 1..0 (00 continuous)
 *   0x61 CFG_REG_B  bit1 offset cancellation, bit0 low-pass filter (ODR/4)
 *   0x62 CFG_REG_C  bit4 block data update
 *   0x67 STATUS     bit3 Zyxda (new X, Y, Z)
 *   0x68 OUTX_L ... 0x6D OUTZ_H, i16 LE, 1.5 mG/LSB
 *
 * Configured at 50 Hz continuous, temperature compensation, offset cancellation, LPF, BDU. Axes are the sensor
 * axes: the mounting on the GPS board (and the GPS board on the frame) is a later calibration item.
 */
#ifndef IIS2MDC_H
#define IIS2MDC_H

#include <stdint.h>
#include "sensor_bus.h"

#define IIS2MDC_ADDR_7BIT    0x1Eu
#define IIS2MDC_WHO_AM_I_VAL 0x40u
#define IIS2MDC_REG_WHO_AM_I 0x4Fu
#define IIS2MDC_REG_CFG_A    0x60u
#define IIS2MDC_REG_CFG_B    0x61u
#define IIS2MDC_REG_CFG_C    0x62u
#define IIS2MDC_REG_STATUS   0x67u
#define IIS2MDC_REG_OUTX_L   0x68u
#define IIS2MDC_MG_PER_LSB   1.5f

#define IIS2MDC_CFG_A_50HZ   0x88u   /**< temperature compensation, ODR 50 Hz, continuous */
#define IIS2MDC_CFG_B        0x03u   /**< offset cancellation, LPF                        */
#define IIS2MDC_CFG_C        0x10u   /**< BDU                                             */
#define IIS2MDC_SOFT_RST     0x20u

typedef struct {
    sensor_bus_t bus;
    uint8_t      ok;
} iis2mdc_t;

/** 0 = found and configured */
int32_t iis2mdc_init(iis2mdc_t *m, const sensor_bus_t *bus);
/** 1 = new sample in raw[3] (LSB), 0 = none, <0 bus error */
int32_t iis2mdc_poll(iis2mdc_t *m, int16_t raw[3]);

#endif /* IIS2MDC_H */
