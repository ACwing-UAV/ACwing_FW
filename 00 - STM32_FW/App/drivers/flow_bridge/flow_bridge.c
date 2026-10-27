/**
 * @file  flow_bridge.c
 */
#include "flow_bridge.h"

static int16_t le16(const uint8_t *p) { return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8)); }

int32_t flow_init(flow_t *f, const sensor_bus_t *bus)
{
    f->bus = *bus;
    f->ok = 0;
    uint8_t id[2] = { 0, 0 };
    if (f->bus.read(f->bus.handle, FLOW_REG_WHO_AM_I, id, 2)) return -1;
    if (id[0] != FLOW_BRIDGE_WHO_AM_I) return -2;
    f->version = id[1];
    f->ok = 1;
    return 0;
}

void flow_decode(const uint8_t r[FLOW_FRAME_LEN], flow_frame_t *fr)
{
    fr->status  = r[0];
    fr->count   = r[1];
    fr->dx      = le16(&r[2]);
    fr->dy      = le16(&r[4]);
    fr->squal   = r[6];
    fr->shutter = (uint16_t)le16(&r[7]);
    fr->dt_s    = (float)(uint16_t)le16(&r[9]) * 1e-4f;
}

int32_t flow_read(flow_t *f, flow_frame_t *fr)
{
    if (!f->ok) return -1;
    uint8_t r[FLOW_FRAME_LEN];
    if (f->bus.read(f->bus.handle, FLOW_REG_FRAME, r, sizeof r)) return -1;
    flow_decode(r, fr);
    return (fr->status & FLOW_ST_NEW) ? 1 : 0;
}
