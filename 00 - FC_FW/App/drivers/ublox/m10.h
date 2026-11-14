/**
 * @file  m10.h
 * @brief u-blox M10 GNSS receiver on I2C (DDC, 7-bit address 0x42), UBX only.
 *
 * DDC registers (u-blox integration manual):
 *   0xFD, 0xFE  bytes waiting in the output buffer, u16 big-endian
 *   0xFF        data stream: every read returns the next bytes (0xFF when empty)
 *   writes      a UBX message is written as plain bytes. On sensor_bus_t (register, data) the first byte of the
 *               frame (0xB5) goes as the "register" and the rest as the data: on the wire it is the same transfer.
 *
 * Start-up (m10_init): the device must answer a read of 0xFD; then one CFG-VALSET in the RAM layer only (no flash
 * wear, applied again at every probe, also after a power cycle of the GPS board):
 *   UBX on I2C on, NMEA on I2C off, NAV-PVT on every solution, M10_RATE_MS, dynamic model airborne < 4 g.
 * The receiver is configured when it answers ACK-ACK, or when a NAV-PVT arrives (the ACK may be lost in the
 * stream backlog of NMEA left from the boot). Without either in M10_CFG_TIMEOUT_MS the VALSET is sent again,
 * M10_CFG_TRIES times, then m10_poll() fails (the caller probes again later).
 *
 * m10_poll() reads at most M10_POLL_MAX bytes per call, in chunks of M10_CHUNK (a 64-byte read is ~1.6 ms at
 * 400 kHz: the HAL timeout of the Qwiic binding is 3 ms). At 100 calls/s that is 12.8 kB/s, ten times NAV-PVT
 * at 5 Hz. Portable: sensor_bus_t only.
 */
#ifndef M10_H
#define M10_H

#include <stdint.h>
#include "sensor_bus.h"
#include "ubx.h"

#define M10_ADDR_7BIT       0x42u
#define M10_REG_AVAIL       0xFDu
#define M10_REG_STREAM      0xFFu
#define M10_CHUNK           64u
#define M10_POLL_MAX        128u
#define M10_RATE_MS         200u     /**< 5 Hz navigation solution                                  */
#define M10_CFG_TIMEOUT_MS  1000u
#define M10_CFG_TRIES       3u

enum { M10_OFF = 0, M10_CONFIG = 1, M10_RUN = 2 };
/** m10_poll() results */
#define M10_NEW_PVT   1
#define M10_NONE      0
#define M10_E_BUS    (-1)
#define M10_E_CONFIG (-2)

typedef struct {
    sensor_bus_t bus;
    ubx_parser_t p;
    uint8_t      state;              /**< M10_*                                    */
    uint8_t      cfg_tries;
    uint8_t      nak;                /**< the last VALSET was refused (ACK-NAK)    */
    uint32_t     cfg_ms;             /**< time of the last VALSET                  */
    uint32_t     pvt_count;
} m10_t;

/** probe and send the configuration; 0 = the device answered, <0 not there */
int32_t m10_init(m10_t *g, const sensor_bus_t *bus, uint32_t now_ms);
/** reads the stream; M10_NEW_PVT with *pvt filled (the last one if several), M10_NONE, or an error */
int32_t m10_poll(m10_t *g, uint32_t now_ms, ubx_pvt_t *pvt);

/** the configuration frame (host tests, documentation) */
uint16_t m10_config_frame(uint8_t *out, uint16_t cap);

#endif /* M10_H */
