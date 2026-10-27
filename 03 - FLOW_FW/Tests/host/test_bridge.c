/**
 * Host test of the ACwing Flow bridge firmware.
 *  - the bridge register map equals the FC driver's (flow_bridge.h);
 *  - the FC driver (flow_init / flow_read) talks to the bridge through a
 *    simulated I2C bus that calls the same bridge_i2c_* functions as the ISR;
 *  - accumulation, latch and clear, saturation, rejected samples, CTRL;
 *  - PMW3901: product-id check, init sequence, motion burst decoding.
 */
#include "bridge.h"
#include "flow_regs.h"
#include "pmw3901.h"
#include "flow_bridge.h"
#include <stdio.h>
#include <string.h>

static int s_fail, s_pass;
#define CHECK(c, ...) do { if (c) s_pass++; else { s_fail++; printf("FAIL %d: ", __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

/* ---- simulated I2C master (as HAL_I2C_Mem_Read / Mem_Write) ---- */
static int32_t i2c_rd(void *h, uint8_t reg, uint8_t *d, uint16_t n)
{
    (void)h;
    bridge_i2c_addr_write(); bridge_i2c_rx(reg);       /* pointer, repeated start */
    bridge_i2c_addr_read();
    for (uint16_t i = 0; i < n; ++i) d[i] = bridge_i2c_tx();
    bridge_i2c_stop();
    return 0;
}
static int32_t i2c_wr(void *h, uint8_t reg, const uint8_t *d, uint16_t n)
{
    (void)h;
    bridge_i2c_addr_write(); bridge_i2c_rx(reg);
    for (uint16_t i = 0; i < n; ++i) bridge_i2c_rx(d[i]);
    bridge_i2c_stop();
    return 0;
}

/* ---- simulated PMW3901 ---- */
static uint8_t  pm_reg[256], pm_id = 0x49, pm_inv = 0xB6;
static int      pm_sel, pm_n, pm_writes, pm_addr, pm_burst_i, pm_reset_low;
static uint8_t  pm_burst[12];
static void     pm_cs(uint8_t l) { pm_sel = !l; pm_n = 0; }
static void     pm_reset(uint8_t l) { if (!l) pm_reset_low++; }
static void     pm_delay(uint32_t us) { (void)us; }
static uint8_t  pm_xfer(uint8_t b)
{
    if (!pm_sel) return 0xFF;
    if (pm_n++ == 0) { pm_addr = b; pm_burst_i = 0; return 0; }
    if (pm_addr & 0x80) { pm_reg[pm_addr & 0x7F] = b; pm_writes++; return 0; }
    if (pm_addr == 0x16) return pm_burst[pm_burst_i++ % 12];
    if (pm_addr == 0x00) return pm_id;
    if (pm_addr == 0x5F) return pm_inv;
    return pm_reg[pm_addr];
}
static const pmw_bus_t k_pm = { pm_xfer, pm_cs, pm_reset, pm_delay };

int main(void)
{
    /* register map: one definition on each side, same numbers */
    CHECK(FLOW_I2C_ADDR == FLOW_BRIDGE_ADDR_7BIT && FLOW_WHO_AM_I_VAL == FLOW_BRIDGE_WHO_AM_I, "address / id");
    CHECK(FLOW_R_FRAME == FLOW_REG_FRAME && FLOW_FRAME_BYTES == FLOW_FRAME_LEN && FLOW_R_CTRL == FLOW_REG_CTRL, "frame / ctrl");
    CHECK(FLOW_S_SENSOR_OK == FLOW_ST_SENSOR_OK && FLOW_S_NEW == FLOW_ST_NEW && FLOW_S_OVERFLOW == FLOW_ST_OVERFLOW, "status bits");

    /* FC driver against the bridge */
    bridge_init();
    flow_t f;
    const sensor_bus_t bus = { i2c_wr, i2c_rd, NULL, NULL };
    CHECK(flow_init(&f, &bus) == 0 && f.version == FLOW_FW_VERSION, "flow_init (version %02X)", f.version);
    flow_frame_t fr;
    CHECK(flow_read(&f, &fr) == 0 && !(fr.status & FLOW_S_SENSOR_OK), "no data, sensor not ok yet");

    bridge_set_sensor_ok(1);
    for (int i = 0; i < 10; ++i) bridge_add_sample(3, -2, 120, 0x0123, 1, 10000);
    CHECK(flow_read(&f, &fr) == 1, "new data");
    CHECK(fr.dx == 30 && fr.dy == -20 && fr.count == 10 && fr.squal == 120 && fr.shutter == 0x0123, "frame %d %d %u", fr.dx, fr.dy, fr.count);
    CHECK(fr.dt_s > 0.0999f && fr.dt_s < 0.1001f && (fr.status & FLOW_S_SENSOR_OK), "dt %.4f", fr.dt_s);
    CHECK(flow_read(&f, &fr) == 0 && fr.dx == 0 && fr.dy == 0 && fr.dt_s == 0.0f, "cleared after the latch");

    /* reading another register does not latch */
    bridge_add_sample(5, 5, 100, 0, 1, 10000);
    uint8_t sq;
    (void)i2c_rd(NULL, FLOW_R_SQUAL, &sq, 1);
    CHECK(sq == 100 && flow_read(&f, &fr) == 1 && fr.dx == 5, "SQUAL read kept the accumulators");

    /* rejected samples: not accumulated, flagged */
    bridge_add_sample(50, 50, 10, 0x1F00, 0, 10000);
    CHECK(flow_read(&f, &fr) == 0 && fr.dx == 0 && (fr.status & FLOW_S_REJECTED) && fr.squal == 10, "rejected sample");

    /* saturation */
    for (int i = 0; i < 20; ++i) bridge_add_sample(3000, -3000, 100, 0, 1, 1000);
    CHECK(flow_read(&f, &fr) == 1 && fr.dx == 32767 && fr.dy == -32768 && (fr.status & FLOW_S_OVERFLOW), "saturated");
    CHECK(flow_read(&f, &fr) == 0 && !(fr.status & FLOW_S_OVERFLOW), "overflow cleared");

    /* sub-100 us remainder carried to the next frame */
    bridge_add_sample(1, 1, 100, 0, 1, 10050);
    (void)flow_read(&f, &fr);
    bridge_add_sample(1, 1, 100, 0, 1, 10050);
    (void)flow_read(&f, &fr);
    CHECK(fr.dt_s > 0.01009f && fr.dt_s < 0.01011f, "remainder kept (%.5f)", fr.dt_s);

    /* past the map */
    uint8_t far[3];
    (void)i2c_rd(NULL, 0x1F, far, 3);
    CHECK(far[0] == 0x00 && far[1] == 0xFF && far[2] == 0xFF, "past the map: 0xFF");

    /* CTRL */
    const uint8_t c = FLOW_CTRL_REINIT;
    (void)i2c_wr(NULL, FLOW_R_CTRL, &c, 1);
    CHECK(bridge_take_ctrl() == FLOW_CTRL_REINIT && bridge_take_ctrl() == 0, "CTRL");
    const uint8_t junk = 0xAA;
    (void)i2c_wr(NULL, FLOW_R_SQUAL, &junk, 1);
    CHECK(bridge_take_ctrl() == 0, "read-only registers ignore writes");

    /* PMW3901 */
    CHECK(pmw_init(&k_pm) == 0 && pm_reset_low == 1, "pmw init");
    CHECK(pm_writes == 1 + 59 + 14 && pm_reg[0x40] == 0x80 && pm_reg[0x3A] == 0x5A, "init sequence (%d writes)", pm_writes);
    pm_id = 0x00;
    CHECK(pmw_init(&k_pm) != 0, "absent sensor detected");
    pm_id = 0x49;
    const uint8_t burst[12] = { 0x80, 0, 0x10, 0x00, 0xF0, 0xFF, 0x50, 0, 0, 0, 0x01, 0x23 };
    memcpy(pm_burst, burst, 12);
    pmw_burst_t b;
    pmw_burst(&b);
    CHECK(b.dx == 16 && b.dy == -16 && b.squal == 0x50 && b.shutter == 0x0123 && pmw_valid(&b), "burst");
    b.squal = 0x10; b.shutter = 0x1F40;
    CHECK(!pmw_valid(&b), "low quality at max shutter rejected");

    printf("%d passed, %d failed\n", s_pass, s_fail);
    return s_fail ? 1 : 0;
}
