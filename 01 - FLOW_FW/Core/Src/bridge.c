/**
 * @file  bridge.c
 * @brief Register file + I2C target protocol (see bridge.h, flow_regs.h).
 */
#include "bridge.h"
#include "flow_regs.h"
#include <string.h>

static volatile int32_t  s_dx, s_dy;      /* accumulated since the last FRAME read */
static volatile uint32_t s_dt_us;
static volatile uint8_t  s_count, s_squal, s_status;
static volatile uint16_t s_shutter;

static uint8_t           s_ptr, s_first, s_wr;
static uint8_t           s_img[FLOW_REG_SPACE];
static uint8_t           s_rd;            /* index in s_img during a read */
static volatile uint8_t  s_ctrl, s_reads;

void bridge_init(void)
{
    s_dx = s_dy = 0; s_dt_us = 0;
    s_count = s_squal = 0; s_status = 0; s_shutter = 0;
    s_ptr = 0; s_first = 0; s_wr = 0; s_rd = 0; s_ctrl = 0; s_reads = 0;
}

void bridge_set_sensor_ok(uint8_t ok)
{
    if (ok) s_status |= FLOW_S_SENSOR_OK;
    else    s_status &= (uint8_t)~FLOW_S_SENSOR_OK;
}

void bridge_add_sample(int16_t dx, int16_t dy, uint8_t squal, uint16_t shutter,
                       uint8_t valid, uint32_t dt_us)
{
    s_squal = squal;
    s_shutter = shutter;
    if (!valid) { s_status |= FLOW_S_REJECTED; return; }
    s_status &= (uint8_t)~FLOW_S_REJECTED;
    s_dx += dx;
    s_dy += dy;
    s_dt_us += dt_us;
    s_count++;
    s_status |= FLOW_S_NEW;
}

static int16_t sat16(int32_t v, uint8_t *ovf)
{
    if (v > 32767)  { *ovf = 1; return 32767; }
    if (v < -32768) { *ovf = 1; return -32768; }
    return (int16_t)v;
}

static void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }

/* register image; a read starting at FRAME latches and clears the accumulators */
static void snapshot(uint8_t latch)
{
    memset(s_img, 0, sizeof s_img);
    s_img[FLOW_R_WHO_AM_I] = FLOW_WHO_AM_I_VAL;
    s_img[FLOW_R_VERSION]  = FLOW_FW_VERSION;
    uint8_t ovf = 0;
    const int16_t dx = sat16(s_dx, &ovf), dy = sat16(s_dy, &ovf);
    uint32_t dt = s_dt_us / 100u;
    if (dt > 0xFFFFu) dt = 0xFFFFu;
    s_img[FLOW_R_STATUS] = (uint8_t)(s_status | (ovf ? FLOW_S_OVERFLOW : 0u));
    s_img[FLOW_R_COUNT]  = s_count;
    put16(&s_img[FLOW_R_DX], (uint16_t)dx);
    put16(&s_img[FLOW_R_DY], (uint16_t)dy);
    s_img[FLOW_R_SQUAL]  = s_squal;
    put16(&s_img[FLOW_R_SHUTTER], s_shutter);
    put16(&s_img[FLOW_R_DT], (uint16_t)dt);
    if (latch) {
        /* keep the sub-100 us remainder so no time is lost between frames */
        s_dx = 0; s_dy = 0;
        s_dt_us -= dt * 100u;
        if (dt == 0xFFFFu) s_dt_us = 0;
        s_status &= (uint8_t)~FLOW_S_NEW;
        s_reads++;
    }
}

void bridge_i2c_addr_write(void) { s_first = 1; s_wr = 0; }

void bridge_i2c_rx(uint8_t b)
{
    if (s_first) { s_ptr = b; s_first = 0; return; }
    if (s_ptr == FLOW_R_CTRL) s_ctrl |= b;           /* the only writable register */
    s_ptr++;
    s_wr++;
}

void bridge_i2c_addr_read(void)
{
    snapshot(s_ptr == FLOW_R_FRAME);
    s_rd = s_ptr;
}

uint8_t bridge_i2c_tx(void)
{
    const uint8_t v = (s_rd < FLOW_REG_SPACE) ? s_img[s_rd] : 0xFFu;
    if (s_rd < 0xFFu) s_rd++;
    return v;
}

void bridge_i2c_stop(void) { s_first = 0; }

uint8_t bridge_take_ctrl(void)
{
    const uint8_t c = s_ctrl;
    s_ctrl = 0;
    return c;
}

uint8_t bridge_reads(void) { return s_reads; }
