/**
 * Host test of the ExtTask logic (app/ext_sens.c) with simulated Qwiic devices:
 * a VL53L1X register model and the REAL flow-bridge firmware logic
 * (01 - FLOW_FW/Core/Src/bridge.c) behind a simulated I2C bus, reached
 * through the FC flow_bridge driver: FC <-> bridge end to end.
 */
#include "ext_sens.h"
#include "bridge.h"
#include "flow_regs.h"
#include "proto_v2.h"
#include <stdio.h>
#include <string.h>

static int s_fail, s_pass;
#define CHECK(c, ...) do { if (c) s_pass++; else { s_fail++; printf("FAIL %d: ", __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

/* ---- VL53L1X model ---- */
static uint8_t v_reg[0x200];
static int     v_present, v_fail, v_inits;
static int32_t v_wr(void *h, uint16_t r, const uint8_t *d, uint16_t n)
{
    (void)h;
    if (!v_present || v_fail) return -1;
    memcpy(&v_reg[r], d, n);
    if (r == 0x87 && d[0] == 0x40) { v_reg[0x31] = 0x01; v_inits++; }
    if (r == 0x86) v_reg[0x31] = 0x00;
    return 0;
}
static int32_t v_rd(void *h, uint16_t r, uint8_t *d, uint16_t n)
{
    (void)h;
    if (!v_present || v_fail) return -1;
    memcpy(d, &v_reg[r], n);
    return 0;
}
static void v_reset_model(void)
{
    memset(v_reg, 0, sizeof v_reg);
    v_reg[0xE5] = 1; v_reg[0x10F] = 0xEA; v_reg[0x110] = 0xCC; v_reg[0x30] = 0x01; v_reg[0xDF] = 0x40;
}
static void v_new_range(uint16_t mm) { v_reg[0x31] = 0x01; v_reg[0x89] = 9; v_reg[0x96] = (uint8_t)(mm >> 8); v_reg[0x97] = (uint8_t)mm; }

/* ---- flow bridge behind a simulated I2C target ---- */
static int     b_present, b_probes;
static int32_t b_rd(void *h, uint8_t reg, uint8_t *d, uint16_t n)
{
    (void)h;
    if (reg == FLOW_R_WHO_AM_I) b_probes++;
    if (!b_present) return -1;                       /* NACK */
    bridge_i2c_addr_write(); bridge_i2c_rx(reg); bridge_i2c_addr_read();
    for (uint16_t i = 0; i < n; ++i) d[i] = bridge_i2c_tx();
    bridge_i2c_stop();
    return 0;
}
static int32_t b_wr(void *h, uint8_t reg, const uint8_t *d, uint16_t n)
{
    (void)h;
    if (!b_present) return -1;
    bridge_i2c_addr_write(); bridge_i2c_rx(reg);
    for (uint16_t i = 0; i < n; ++i) bridge_i2c_rx(d[i]);
    bridge_i2c_stop();
    return 0;
}

static int drain(int *tof, int *flow, int *flow_new, ext_sample_t *last_flow)
{
    ext_sample_t s; int n = 0;
    while (ext_take(&s)) {
        n++;
        if (s.kind == EXT_TOF && tof) (*tof)++;
        if (s.kind == EXT_FLOW) { if (flow) (*flow)++; if (s.dt_s > 0.0f && flow_new) (*flow_new)++; if (last_flow) *last_flow = s; }
    }
    return n;
}

int main(void)
{
    ext_buses_t b = { { v_wr, v_rd, NULL, NULL }, { b_wr, b_rd, NULL, NULL } };
    v_reset_model(); bridge_init();
    v_present = 0; b_present = 0;
    ext_init(&b);
    uint32_t now = 0;

    /* nothing plugged: no sensor, probed again only every 5 s */
    for (int k = 0; k < 700; ++k, now += 10) ext_step(now);      /* 7 s */
    CHECK(ext_sensors() == 0, "no sensors");
    CHECK(b_probes == 2, "probes at 0 s and 5 s (%d)", b_probes);
    (void)drain(NULL, NULL, NULL, NULL);

    /* the Flow board is plugged in at 7 s: found at the 10 s probe */
    v_present = 1; b_present = 1; bridge_set_sensor_ok(1);
    for (int k = 0; k < 200; ++k, now += 10) ext_step(now);      /* up to 9 s */
    CHECK(ext_sensors() == 0, "not before the next probe");
    for (int k = 0; k < 200; ++k, now += 10) ext_step(now);      /* up to 11 s */
    CHECK(ext_sensors() == (PV2_S_TOF | PV2_S_FLOW), "found at 10 s (%04X)", ext_sensors());

    /* polling: ToF sample on data ready; flow frame every cycle, motion only when new */
    int tof = 0, flow = 0, fnew = 0; ext_sample_t lf; memset(&lf, 0, sizeof lf);
    (void)drain(NULL, NULL, NULL, NULL);
    for (int k = 0; k < 10; ++k, now += 10) {
        if (k % 3 == 0) v_new_range(1234);
        if (k == 5) bridge_add_sample(12, -7, 110, 0x0100, 1, 10000);
        ext_step(now);
        (void)drain(&tof, &flow, &fnew, k == 5 ? &lf : NULL);
    }
    CHECK(tof == 4, "ToF samples on data ready (%d)", tof);
    CHECK(flow == 10 && fnew == 1, "flow frames every cycle, one with motion (%d, %d)", flow, fnew);
    CHECK(lf.dx == 12 && lf.dy == -7 && lf.squal == 110 && (lf.status & FLOW_S_SENSOR_OK), "flow frame through the bridge");

    /* sensor below its minimum height: bridge rejects, FC still gets status + SQUAL */
    bridge_add_sample(50, 50, 8, 0x1F00, 0, 10000);
    ext_step(now); now += 10;
    {
        ext_sample_t s; int got = 0;
        while (ext_take(&s)) if (s.kind == EXT_FLOW) { got = 1; CHECK(s.dt_s == 0.0f && s.dx == 0 && s.squal == 8 && (s.status & FLOW_S_REJECTED), "status-only frame on the ground"); }
        CHECK(got, "frame sent without motion");
    }

    /* the ToF stops answering: lost after 50 errors, the flow keeps going, re-probed */
    v_fail = 1;
    for (int k = 0; k < 60; ++k, now += 10) ext_step(now);
    CHECK(ext_sensors() == PV2_S_FLOW, "ToF lost (%04X)", ext_sensors());
    CHECK(ext_errors() >= 51, "errors counted");
    v_fail = 0; v_reset_model();
    const int inits = v_inits;
    for (int k = 0; k < 520; ++k, now += 10) ext_step(now);
    CHECK(ext_sensors() == (PV2_S_TOF | PV2_S_FLOW) && v_inits > inits, "ToF back after the 5 s probe");

    printf("ext: %d passed, %d failed\n", s_pass, s_fail);
    return s_fail ? 1 : 0;
}
