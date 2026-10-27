/**
 * @file  vl53l1x.h
 * @brief Minimal VL53L1X time-of-flight driver (ACwing Flow rev A, Qwiic 0x29).
 *
 * Register sequences and the default configuration block are those of ST's
 * VL53L1X Ultra Lite Driver (ULD, STSW-IMG009, BSD-3-Clause, see vl53l1x.c).
 * Only what the flight controller needs: init, long distance mode, 33 ms
 * timing budget (~30 Hz continuous ranging), non-blocking poll.
 *
 * No HAL, no RTOS: the bus is given as function pointers (16-bit registers,
 * big-endian data as on the device). Host-testable with a simulated bus.
 */
#ifndef VL53L1X_H
#define VL53L1X_H

#include <stdint.h>

#define VL53L1X_ADDR_7BIT   0x29u
#define VL53L1X_MODEL_ID    0xEACCu

typedef struct {
    int32_t (*write)(void *h, uint16_t reg, const uint8_t *d, uint16_t len);
    int32_t (*read) (void *h, uint16_t reg, uint8_t *d, uint16_t len);
    void    (*delay_ms)(uint32_t ms);
    void    *handle;
} vl53_bus_t;

typedef struct {
    vl53_bus_t bus;
    uint8_t    int_pol;         /* data-ready level on GPIO__TIO_HV_STATUS bit 0 */
    uint8_t    ok;
} vl53_t;

/* Range status after the ULD mapping: 0 valid, 1 sigma fail, 2 signal fail,
 * 4 out of bounds, 7 wrap-around, others < 255 unusual, 255 unknown. */
typedef struct {
    uint16_t mm;
    uint8_t  status;
    uint16_t signal_kcps;       /* peak signal rate (MCPS x 1000 / 128 x 8) */
} vl53_sample_t;

/* 0 = sensor found and ranging; < 0 = absent or error (the board works without it) */
int32_t vl53_init(vl53_t *d, const vl53_bus_t *bus, uint16_t budget_ms);
/* 1 = new sample in *s, 0 = none yet, < 0 = bus error */
int32_t vl53_poll(vl53_t *d, vl53_sample_t *s);

#endif /* VL53L1X_H */
