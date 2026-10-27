/**
 * @file  pmw3901.c
 * @brief PMW3901MB driver (see pmw3901.h).
 *
 * The power-up check and the "performance optimisation" register sequence are
 * taken from the Bitcraze PMW3901 Arduino driver:
 *   Copyright (c) 2017 Bitcraze AB. MIT License: permission is hereby granted,
 *   free of charge, to deal in the Software without restriction, subject to
 *   including the copyright notice and this permission notice. THE SOFTWARE IS
 *   PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.
 */
#include "pmw3901.h"

#define PMW_PRODUCT_ID       0x49u
#define PMW_INV_PRODUCT_ID   0xB6u
#define PMW_REG_MOTION_BURST 0x16u

static const pmw_bus_t *s_b;

static void wr(uint8_t reg, uint8_t v)
{
    s_b->cs(0);
    s_b->delay_us(50);
    (void)s_b->xfer((uint8_t)(reg | 0x80u));
    (void)s_b->xfer(v);
    s_b->delay_us(50);
    s_b->cs(1);
    s_b->delay_us(200);
}

static uint8_t rd(uint8_t reg)
{
    s_b->cs(0);
    s_b->delay_us(50);
    (void)s_b->xfer((uint8_t)(reg & 0x7Fu));
    s_b->delay_us(50);
    const uint8_t v = s_b->xfer(0);
    s_b->delay_us(100);
    s_b->cs(1);
    s_b->delay_us(20);
    return v;
}

static const uint8_t k_init1[][2] = {
    {0x7F,0x00},{0x61,0xAD},{0x7F,0x03},{0x40,0x00},{0x7F,0x05},{0x41,0xB3},{0x43,0xF1},{0x45,0x14},
    {0x5B,0x32},{0x5F,0x34},{0x7B,0x08},{0x7F,0x06},{0x44,0x1B},{0x40,0xBF},{0x4E,0x3F},{0x7F,0x08},
    {0x65,0x20},{0x6A,0x18},{0x7F,0x09},{0x4F,0xAF},{0x5F,0x40},{0x48,0x80},{0x49,0x80},{0x57,0x77},
    {0x60,0x78},{0x61,0x78},{0x62,0x08},{0x63,0x50},{0x7F,0x0A},{0x45,0x60},{0x7F,0x00},{0x4D,0x11},
    {0x55,0x80},{0x74,0x1F},{0x75,0x1F},{0x4A,0x78},{0x4B,0x78},{0x44,0x08},{0x45,0x50},{0x64,0xFF},
    {0x65,0x1F},{0x7F,0x14},{0x65,0x60},{0x66,0x08},{0x63,0x78},{0x7F,0x15},{0x48,0x58},{0x7F,0x07},
    {0x41,0x0D},{0x43,0x14},{0x4B,0x0E},{0x45,0x0F},{0x44,0x42},{0x4C,0x80},{0x7F,0x10},{0x5B,0x02},
    {0x7F,0x07},{0x40,0x41},{0x70,0x00}
};
static const uint8_t k_init2[][2] = {
    {0x32,0x44},{0x7F,0x07},{0x40,0x40},{0x7F,0x06},{0x62,0xF0},{0x63,0x00},{0x7F,0x0D},{0x48,0xC0},
    {0x6F,0xD5},{0x7F,0x00},{0x5B,0xA0},{0x4E,0xA8},{0x5A,0x50},{0x40,0x80}
};

int pmw_init(const pmw_bus_t *bus)
{
    s_b = bus;
    /* hardware reset, then the SPI port reset (NCS high-low-high) */
    s_b->cs(1);
    s_b->reset(0);
    s_b->delay_us(1000);
    s_b->reset(1);
    s_b->delay_us(50000);
    s_b->cs(0);
    s_b->delay_us(1000);
    s_b->cs(1);
    s_b->delay_us(1000);

    wr(0x3A, 0x5A);                                  /* power-up reset */
    s_b->delay_us(5000);
    if (rd(0x00) != PMW_PRODUCT_ID || rd(0x5F) != PMW_INV_PRODUCT_ID) return -1;
    for (uint8_t r = 0x02; r <= 0x06; ++r) (void)rd(r);   /* clear the motion registers */
    s_b->delay_us(1000);

    for (unsigned i = 0; i < sizeof k_init1 / sizeof k_init1[0]; ++i) wr(k_init1[i][0], k_init1[i][1]);
    s_b->delay_us(100000);
    for (unsigned i = 0; i < sizeof k_init2 / sizeof k_init2[0]; ++i) wr(k_init2[i][0], k_init2[i][1]);
    return 0;
}

void pmw_burst(pmw_burst_t *b)
{
    uint8_t r[12];
    s_b->cs(0);
    s_b->delay_us(50);
    (void)s_b->xfer(PMW_REG_MOTION_BURST);
    s_b->delay_us(50);                               /* tSRAD-MOTBR >= 35 us */
    for (int i = 0; i < 12; ++i) r[i] = s_b->xfer(0);
    s_b->cs(1);
    s_b->delay_us(20);
    b->motion      = r[0];
    b->observation = r[1];
    b->dx          = (int16_t)((uint16_t)r[2] | ((uint16_t)r[3] << 8));
    b->dy          = (int16_t)((uint16_t)r[4] | ((uint16_t)r[5] << 8));
    b->squal       = r[6];
    b->raw_sum     = r[7];
    b->raw_max     = r[8];
    b->raw_min     = r[9];
    b->shutter     = (uint16_t)(((uint16_t)r[10] << 8) | r[11]);
}

uint8_t pmw_valid(const pmw_burst_t *b)
{
    return (uint8_t)!(b->squal < 0x19u && (b->shutter >> 8) == 0x1Fu);
}
