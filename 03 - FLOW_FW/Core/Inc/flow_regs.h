/**
 * @file  flow_regs.h
 * @brief I2C register map of the ACwing Flow bridge (STM32C011 at 0x3B).
 *
 * The SAME map is used by the flight controller driver
 * (00 - STM32_FW/App/drivers/flow_bridge/flow_bridge.h); the host test checks
 * that the two agree.
 *
 *   addr  name      access  content
 *   0x00  WHO_AM_I  R       0xF1
 *   0x01  VERSION   R       bridge firmware version (major << 4 | minor)
 *   0x02  STATUS    R       bit0 PMW3901 ok, bit1 new data since the last FRAME
 *                           read, bit2 overflow (DX or DY saturated), bit3 the
 *                           last sample was rejected (no surface features)
 *   0x03  COUNT     R       u8, +1 per PMW3901 sample accumulated (wraps)
 *   0x04  DX        R       i16, counts accumulated since the last FRAME read
 *   0x06  DY        R       i16
 *   0x08  SQUAL     R       u8, surface quality of the last sample
 *   0x09  SHUTTER   R       u16, shutter of the last sample
 *   0x0B  DT        R       u16, time covered by DX/DY, units of 100 us
 *   0x10  CTRL      W       bit0 soft reset of the bridge, bit1 re-init PMW3901
 *
 * FRAME = 0x02..0x0C (11 bytes) read in ONE transaction: the frame is latched
 * when a read starts at 0x02, and DX, DY, DT and STATUS.bit1 are cleared.
 * Little-endian, auto-increment; unmapped addresses read 0x00 (0xFF past 0x1F).
 */
#ifndef FLOW_REGS_H
#define FLOW_REGS_H

#define FLOW_I2C_ADDR       0x3Bu
#define FLOW_WHO_AM_I_VAL   0xF1u
#define FLOW_FW_VERSION     0x10u     /* 1.0 */

#define FLOW_R_WHO_AM_I     0x00u
#define FLOW_R_VERSION      0x01u
#define FLOW_R_STATUS       0x02u
#define FLOW_R_COUNT        0x03u
#define FLOW_R_DX           0x04u
#define FLOW_R_DY           0x06u
#define FLOW_R_SQUAL        0x08u
#define FLOW_R_SHUTTER      0x09u
#define FLOW_R_DT           0x0Bu
#define FLOW_R_CTRL         0x10u
#define FLOW_R_FRAME        FLOW_R_STATUS
#define FLOW_FRAME_BYTES    11u
#define FLOW_REG_SPACE      0x20u

#define FLOW_S_SENSOR_OK    0x01u
#define FLOW_S_NEW          0x02u
#define FLOW_S_OVERFLOW     0x04u
#define FLOW_S_REJECTED     0x08u

#define FLOW_CTRL_RESET     0x01u
#define FLOW_CTRL_REINIT    0x02u

#endif /* FLOW_REGS_H */
