/**
 * @file  flow_bridge.h
 * @brief ACwing Flow rev A optical-flow bridge (STM32C011 at 0x3B, PMW3901 behind it).
 *
 * The bridge reads the PMW3901 motion burst at 100 Hz over SPI and serves the
 * accumulated motion over I2C. Bridge firmware: 01 - FLOW_FW (its flow_regs.h
 * holds the same map; its host test checks that the two agree). Without a
 * programmed bridge flow_init() fails and the FLOW sensor bit stays 0.
 *
 * Register map (8-bit addresses, auto-increment, little-endian):
 *
 *   addr  name      access  content
 *   0x00  WHO_AM_I  R       0xF1
 *   0x01  VERSION   R       bridge firmware version (major << 4 | minor)
 *   0x02  STATUS    R       bit0 PMW3901 ok (product id checked), bit1 new data
 *                           since the last FRAME read, bit2 overflow (|dX| or |dY|
 *                           saturated), bit3 last sample rejected (no surface
 *                           features: low SQUAL at maximum shutter)
 *   0x03  COUNT     R       u8, +1 per PMW3901 sample accumulated (wraps)
 *   0x04  DX        R       i16, counts accumulated since the last FRAME read
 *   0x06  DY        R       i16
 *   0x08  SQUAL     R       u8, surface quality of the last sample (0..~170)
 *   0x09  SHUTTER   R       u16, shutter of the last sample
 *   0x0B  DT        R       u16, time covered by DX/DY, units of 100 us
 *   0x10  CTRL      W       bit0 soft reset of the bridge, bit1 re-init PMW3901
 *
 *   FRAME = 0x02..0x0C (11 bytes) read in ONE transaction: the bridge latches
 *   the frame on the read of 0x02 and clears DX, DY, DT and STATUS.bit1.
 *
 * Axes (sensor-side view of the flow board): DX along +s, DY along +t. With the
 * frame-C2 mounting (t -> forward) the firmware maps them to the body axes.
 */
#ifndef FLOW_BRIDGE_H
#define FLOW_BRIDGE_H

#include <stdint.h>
#include "sensor_bus.h"

#define FLOW_BRIDGE_ADDR_7BIT  0x3Bu
#define FLOW_BRIDGE_WHO_AM_I   0xF1u
#define FLOW_REG_WHO_AM_I      0x00u
#define FLOW_REG_VERSION       0x01u
#define FLOW_REG_FRAME         0x02u
#define FLOW_FRAME_LEN         11u
#define FLOW_REG_CTRL          0x10u

#define FLOW_ST_SENSOR_OK      0x01u
#define FLOW_ST_NEW            0x02u
#define FLOW_ST_OVERFLOW       0x04u
#define FLOW_ST_REJECTED       0x08u

typedef struct {
    sensor_bus_t bus;
    uint8_t      version;
    uint8_t      ok;
} flow_t;

typedef struct {
    uint8_t  status, count, squal;
    int16_t  dx, dy;
    uint16_t shutter;
    float    dt_s;
} flow_frame_t;

int32_t flow_init(flow_t *f, const sensor_bus_t *bus);           /* 0 = bridge found */
int32_t flow_read(flow_t *f, flow_frame_t *fr);                 /* 1 = new data, 0 = none, <0 error */
/* pure decoder (host tests) */
void    flow_decode(const uint8_t raw[FLOW_FRAME_LEN], flow_frame_t *fr);

#endif /* FLOW_BRIDGE_H */
