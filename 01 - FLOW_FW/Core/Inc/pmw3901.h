/**
 * @file  pmw3901.h
 * @brief PixArt PMW3901MB optical flow sensor over SPI (mode 3, <= 2 MHz).
 *
 * Register sequences from the Bitcraze PMW3901 driver (MIT licence, see
 * pmw3901.c). The bus is given as callbacks: no registers here, host-testable.
 */
#ifndef PMW3901_H
#define PMW3901_H

#include <stdint.h>

typedef struct {
    uint8_t (*xfer)(uint8_t b);         /**< one SPI byte, full duplex */
    void    (*cs)(uint8_t level);       /**< NCS: 0 = selected          */
    void    (*reset)(uint8_t level);    /**< NRESET: 0 = in reset       */
    void    (*delay_us)(uint32_t us);
} pmw_bus_t;

typedef struct {
    uint8_t  motion, observation;
    int16_t  dx, dy;
    uint8_t  squal, raw_sum, raw_max, raw_min;
    uint16_t shutter;
} pmw_burst_t;

int     pmw_init(const pmw_bus_t *bus);           /**< 0 = product id ok, configured */
void    pmw_burst(pmw_burst_t *b);                /**< motion burst (register 0x16)  */
/** PixArt rule: SQUAL < 0x19 with the shutter at its maximum = no usable features */
uint8_t pmw_valid(const pmw_burst_t *b);

#endif /* PMW3901_H */
