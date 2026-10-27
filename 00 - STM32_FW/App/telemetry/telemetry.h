/**
 * @file  telemetry.h
 * @brief Telemetry packet format (shared with 01 - Tools/00 - Matlab/Telemetry/parseTelemetry.m
 *        and 01 - Tools/01 - Python/acwing_tools/telemetry.py).
 *
 * Frame (little-endian):
 *   0xA5 0x5A | TYPE | LEN | PAYLOAD[LEN] | CRC16 (LSB, MSB)
 *   CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) computed over TYPE..PAYLOAD.
 *
 * TYPE 0x01 STATE  : one filter sample (attitude + bias + raw sensors)
 * TYPE 0x02 INIT   : parameters used by the filter, re-sent periodically.
 *                    Lets MATLAB re-run iekfAttitude.m with EXACTLY the same
 *                    conditions (compareEmbeddedVsMatlab.m).
 * TYPE 0x03 STATUS : diagnostics, always sent
 * TYPE 0x04 REPLAY : trace of the filter calls + resulting state
 *
 * The same frames go over BLE as a byte stream (tlm_ble.h): no BLE-specific
 * packet format.
 */
#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>
#include <stddef.h>

#define TLM_SYNC0       0xA5u
#define TLM_SYNC1       0x5Au
#define TLM_TYPE_STATE  0x01u
#define TLM_TYPE_INIT   0x02u
#define TLM_TYPE_STATUS 0x03u   /* 10 Hz diagnostics, ALWAYS sent (also during calibration) */
#define TLM_TYPE_REPLAY 0x04u   /* trace of the filter calls + resulting state (parity)   */
#define TLM_OVERHEAD    6u

/* STATE packet flags */
#define TLM_F_RUNNING    0x01u
#define TLM_F_MAG_NEW    0x02u   /* magnetometer update performed on this sample     */
#define TLM_F_ACC_GATED  0x04u   /* acc de-weighted (magnitude or NIS)                */
#define TLM_F_MAG_GATED  0x08u
#define TLM_F_OVERRUN    0x10u   /* samples lost upstream (queue full)                */
#define TLM_F_STILL      0x20u   /* stillness detector active                         */

/* INIT packet flags */
#define TLM_I_EST_BIAS   0x01u
#define TLM_I_ADAPTIVE   0x02u
#define TLM_I_TILT_COMP  0x04u
#define TLM_I_DECOUPLE   0x08u
#define TLM_I_RESET_JAC  0x10u

/* WARNING: "packed" structs = float fields NOT 4-byte aligned
 * (e.g. tlm_state_t.q is at offset 6). Never pass the address of a field to a
 * function that writes floats through a pointer: on Cortex-M4F the compiler uses
 * VSTR, which on an unaligned address raises UsageFault -> HardFault.
 * Always write into a local variable first, then copy with memcpy. */
#pragma pack(push, 1)
typedef struct {
    uint16_t seq;        /* filter sample index (0 = initial state R0)         */
    uint32_t t_us;       /* filter time: sum of the dt actually used          */
    float    q[4];       /* body->world, [w x y z]                             */
    float    bias[3];    /* rad/s                                              */
    float    acc[3];     /* g,    body frame                                   */
    float    gyro[3];    /* rad/s                                              */
    float    mag[3];     /* uT,   body frame, hard-iron already removed        */
    uint8_t  flags;
} tlm_state_t;           /* 71 bytes */

typedef struct {
    float   q0[4];
    float   g_ref[3];
    float   m_ref[3];
    float   b0[3];
    float   sigma_acc, sigma_mag, sigma_gyro, sigma_bias;
    float   nom_acc, nom_mag;
    float   fs_hz;
    float   acc_tol, mag_tol;
    float   p0_att, p0_bias;
    float   acc_nis_gate, mag_nis_gate, infl_decay, dip_tol;
    float   infl_max, lockout_p_bump, persist_decay, persist_ratio, persist_min;
    uint16_t lockout_updates, grace_updates;
    uint8_t flags;
} tlm_init_t;            /* 137 bytes */

typedef struct {
    uint32_t uptime_ms;
    uint32_t irq_count;      /* INT1 interrupts received                */
    uint32_t bus_err;        /* I2C errors (FIFO status or DMA burst)   */
    uint32_t fifo_ovr;       /* reads truncated to IMU_FIFO_MAX_WORDS   */
    uint32_t tlm_drop;       /* frames lost on the telemetry queue     */
    uint32_t uart_drop;      /* frames not sent (UART DMA busy)         */
    uint32_t proc_cyc_max;   /* max CPU cycles of est_process           */
    uint32_t n_gyro, n_acc, n_mag, n_mag_dup, n_sh_nack;   /* cumulative FIFO words */
    uint16_t words_max;      /* max words read in one interrupt         */
    uint16_t calib_count;    /* calibration samples collected           */
    int8_t   imu_init;       /* 1 = in progress, 0 = OK, <0 = imu_status_t */
    uint8_t  running;        /* 1 = filter running                      */
    /* ---- phase 2: BLE link (all 0 with APP_TLM_BLE = 0) ---- */
    uint32_t ble_drop;       /* frames not queued for BLE (link too slow) */
    uint16_t ble_mtu;        /* ATT_MTU in use                           */
    uint8_t  ble_state;      /* tlm_ble_state_t: 0 off, 1 advertising,
                                2 connected, 3 streaming                 */
    uint8_t  ble_err;        /* last BLE stack error code (0 = none)     */
} tlm_status_t;              /* 62 bytes (54 in phase 1) */
#pragma pack(pop)

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(tlm_state_t) == 71, "tlm_state_t layout");
_Static_assert(sizeof(tlm_init_t)  == 137, "tlm_init_t layout");
_Static_assert(sizeof(tlm_status_t) == 62, "tlm_status_t layout");
#endif

/* REPLAY (variable length, max TLM_REPLAY_MAX payload bytes):
 *   uint32 win | uint8 nops | uint8 flags(bit0 = trace truncated)
 *   ops[...]   (encoding in est_pipeline.h)
 *   float q[4] b[3] Pdiag[6] acc_infl mag_infl     (state AFTER the window) */
#define TLM_REPLAY_TAIL  60u
#define TLM_REPLAY_MAX   (6u + 140u + TLM_REPLAY_TAIL)
#define TLM_FRAME_MAX    (TLM_REPLAY_MAX + TLM_OVERHEAD)

uint16_t tlm_crc16(const uint8_t *data, size_t len);
/* Writes the complete frame into out; returns its length, 0 if cap is too small */
size_t   tlm_encode(uint8_t type, const void *payload, uint8_t len, uint8_t *out, size_t cap);

#endif /* TELEMETRY_H */
