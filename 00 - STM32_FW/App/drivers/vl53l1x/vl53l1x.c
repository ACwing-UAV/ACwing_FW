/**
 * @file  vl53l1x.c
 * @brief VL53L1X driver (see vl53l1x.h).
 *
 * Derived from ST's VL53L1X ULD:
 *   COPYRIGHT(c) 2018 STMicroelectronics. Redistribution and use in source and
 *   binary forms, with or without modification, are permitted provided that the
 *   BSD-3-Clause conditions are met (copyright notice, this list of conditions
 *   and the disclaimer below retained; no endorsement by STMicroelectronics).
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *   AND ANY EXPRESS OR IMPLIED WARRANTIES ARE DISCLAIMED.
 */
#include "vl53l1x.h"

#define REG_SOFT_RESET              0x0000u
#define REG_VHV_TIMEOUT_LOOP_BOUND  0x0008u
#define REG_VHV_START               0x000Bu
#define REG_GPIO_HV_MUX_CTRL        0x0030u
#define REG_GPIO_TIO_HV_STATUS      0x0031u
#define REG_PHASECAL_TIMEOUT        0x004Bu
#define REG_RANGE_TIMEOUT_A_HI      0x005Eu
#define REG_RANGE_VCSEL_PERIOD_A    0x0060u
#define REG_RANGE_TIMEOUT_B_HI      0x0061u
#define REG_RANGE_VCSEL_PERIOD_B    0x0063u
#define REG_RANGE_VALID_PHASE_HIGH  0x0069u
#define REG_INTERMEASUREMENT        0x006Cu
#define REG_SD_WOI_SD0              0x0078u
#define REG_SD_INITIAL_PHASE_SD0    0x007Au
#define REG_INTERRUPT_CLEAR         0x0086u
#define REG_MODE_START              0x0087u
#define REG_RESULT_RANGE_STATUS     0x0089u
#define REG_RESULT_DISTANCE         0x0096u
#define REG_OSC_CALIBRATE_VAL       0x00DEu
#define REG_FIRMWARE_SYSTEM_STATUS  0x00E5u
#define REG_MODEL_ID                0x010Fu

/* ULD VL51L1X_DEFAULT_CONFIGURATION, registers 0x2D..0x87 */
static const uint8_t k_default[0x87 - 0x2D + 1] = {
    0x00, 0x01, 0x01, 0x01, 0x02, 0x00, 0x02, 0x08, 0x00, 0x08, 0x10, 0x01, 0x01, 0x00, 0x00, 0x00,
    0x00, 0xff, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x0b, 0x00, 0x00, 0x02, 0x0a, 0x21,
    0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xc8, 0x00, 0x00, 0x38, 0xff, 0x01, 0x00, 0x08, 0x00,
    0x00, 0x01, 0xdb, 0x0f, 0x01, 0xf1, 0x0d, 0x01, 0x68, 0x00, 0x80, 0x08, 0xb8, 0x00, 0x00, 0x00,
    0x00, 0x0f, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x0f, 0x0d, 0x0e, 0x0e, 0x00,
    0x00, 0x02, 0xc7, 0xff, 0x9B, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00
};

static int32_t wr8(vl53_t *d, uint16_t reg, uint8_t v) { return d->bus.write(d->bus.handle, reg, &v, 1); }
static int32_t wr16(vl53_t *d, uint16_t reg, uint16_t v)
{
    const uint8_t b[2] = { (uint8_t)(v >> 8), (uint8_t)v };
    return d->bus.write(d->bus.handle, reg, b, 2);
}
static int32_t wr32(vl53_t *d, uint16_t reg, uint32_t v)
{
    const uint8_t b[4] = { (uint8_t)(v >> 24), (uint8_t)(v >> 16), (uint8_t)(v >> 8), (uint8_t)v };
    return d->bus.write(d->bus.handle, reg, b, 4);
}
static int32_t rd8(vl53_t *d, uint16_t reg, uint8_t *v) { return d->bus.read(d->bus.handle, reg, v, 1); }
static int32_t rd16(vl53_t *d, uint16_t reg, uint16_t *v)
{
    uint8_t b[2];
    const int32_t r = d->bus.read(d->bus.handle, reg, b, 2);
    *v = (uint16_t)(((uint16_t)b[0] << 8) | b[1]);
    return r;
}
static void dly(vl53_t *d, uint32_t ms) { if (d->bus.delay_ms) d->bus.delay_ms(ms); }

static int32_t data_ready(vl53_t *d, uint8_t *rdy)
{
    uint8_t t;
    if (rd8(d, REG_GPIO_TIO_HV_STATUS, &t)) return -1;
    *rdy = (uint8_t)((t & 1u) == d->int_pol);
    return 0;
}

static uint8_t map_status(uint8_t s)
{
    switch (s & 0x1Fu) {
    case 9:  return 0;  case 6:  return 1;  case 4:  return 2;  case 8:  return 3;
    case 5:  return 4;  case 3:  return 5;  case 19: return 6;  case 7:  return 7;
    case 12: return 9;  case 18: return 10; case 22: return 11; case 23: return 12;
    case 13: return 13; default: return 255;
    }
}

int32_t vl53_init(vl53_t *d, const vl53_bus_t *bus, uint16_t budget_ms)
{
    d->bus = *bus;
    d->ok = 0;

    /* boot: firmware ready (up to ~1.2 ms after XSHUT, allow 100 ms) */
    uint8_t st = 0;
    for (int i = 0; i < 100; ++i) {
        if (rd8(d, REG_FIRMWARE_SYSTEM_STATUS, &st) == 0 && (st & 1u)) break;
        dly(d, 1);
    }
    if (!(st & 1u)) return -1;
    uint16_t id = 0;
    if (rd16(d, REG_MODEL_ID, &id) || id != VL53L1X_MODEL_ID) return -2;

    /* SensorInit */
    if (d->bus.write(d->bus.handle, 0x2Du, k_default, sizeof k_default)) return -1;
    uint8_t mux = 0;
    if (rd8(d, REG_GPIO_HV_MUX_CTRL, &mux)) return -1;
    d->int_pol = (uint8_t)!((mux & 0x10u) >> 4);
    if (wr8(d, REG_INTERRUPT_CLEAR, 0x01) || wr8(d, REG_MODE_START, 0x40)) return -1;
    uint8_t rdy = 0;
    for (int i = 0; i < 150 && !rdy; ++i) { if (data_ready(d, &rdy)) return -1; if (!rdy) dly(d, 1); }
    if (!rdy) return -3;
    if (wr8(d, REG_INTERRUPT_CLEAR, 0x01) || wr8(d, REG_MODE_START, 0x00)) return -1;
    if (wr8(d, REG_VHV_TIMEOUT_LOOP_BOUND, 0x09) || wr8(d, REG_VHV_START, 0x00)) return -1;

    /* long distance mode (up to 4 m indoors) */
    if (wr8(d, REG_PHASECAL_TIMEOUT, 0x0A) || wr8(d, REG_RANGE_VCSEL_PERIOD_A, 0x0F) ||
        wr8(d, REG_RANGE_VCSEL_PERIOD_B, 0x0D) || wr8(d, REG_RANGE_VALID_PHASE_HIGH, 0xB8) ||
        wr16(d, REG_SD_WOI_SD0, 0x0F0D) || wr16(d, REG_SD_INITIAL_PHASE_SD0, 0x0E0E)) return -1;
    /* timing budget (long mode table of the ULD) */
    uint16_t a, b;
    switch (budget_ms) {
    case 20:  a = 0x001E; b = 0x0022; break;
    case 50:  a = 0x00AD; b = 0x00C6; break;
    case 100: a = 0x01CC; b = 0x01EA; break;
    default:  a = 0x0060; b = 0x006E; budget_ms = 33; break;
    }
    if (wr16(d, REG_RANGE_TIMEOUT_A_HI, a) || wr16(d, REG_RANGE_TIMEOUT_B_HI, b)) return -1;
    /* inter-measurement period = budget (continuous back-to-back ranging) */
    uint16_t pll = 0;
    if (rd16(d, REG_OSC_CALIBRATE_VAL, &pll)) return -1;
    pll &= 0x3FFu;
    if (wr32(d, REG_INTERMEASUREMENT, (uint32_t)((float)pll * (float)budget_ms * 1.075f))) return -1;

    if (wr8(d, REG_INTERRUPT_CLEAR, 0x01) || wr8(d, REG_MODE_START, 0x40)) return -1;
    d->ok = 1;
    return 0;
}

int32_t vl53_poll(vl53_t *d, vl53_sample_t *s)
{
    if (!d->ok) return -1;
    uint8_t rdy = 0;
    if (data_ready(d, &rdy)) return -1;
    if (!rdy) return 0;
    /* RESULT__RANGE_STATUS (0x89) .. FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0 (0x96)
     * .. PEAK_SIGNAL_COUNT_RATE (0x98): one burst */
    uint8_t r[0x99 - 0x89 + 1];
    if (d->bus.read(d->bus.handle, REG_RESULT_RANGE_STATUS, r, sizeof r)) return -1;
    if (wr8(d, REG_INTERRUPT_CLEAR, 0x01)) return -1;
    s->status = map_status(r[0]);
    s->mm = (uint16_t)(((uint16_t)r[0x96 - 0x89] << 8) | r[0x97 - 0x89]);
    s->signal_kcps = (uint16_t)((((uint32_t)r[0x98 - 0x89] << 8) | r[0x99 - 0x89]) * 8u);
    return 1;
}
