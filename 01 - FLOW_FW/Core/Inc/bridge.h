/**
 * @file  bridge.h
 * @brief Register file of the flow bridge and the I2C target protocol.
 *
 * Pure C (no registers): the main loop adds PMW3901 samples, the I2C
 * interrupt calls the bridge_i2c_* functions. The two sides share the
 * accumulators: bridge_add_sample() must run with the I2C interrupt masked
 * (the main loop wraps it in a short critical section).
 */
#ifndef BRIDGE_H
#define BRIDGE_H

#include <stdint.h>

void    bridge_init(void);
void    bridge_set_sensor_ok(uint8_t ok);
/** one motion burst: deltas since the previous burst, dt_us = time between the
 * two bursts; valid = 0 for a sample without surface features */
void    bridge_add_sample(int16_t dx, int16_t dy, uint8_t squal, uint16_t shutter,
                          uint8_t valid, uint32_t dt_us);

/* ---- I2C target, interrupt context ---- */
void    bridge_i2c_addr_write(void);       /**< address matched, master writes */
void    bridge_i2c_rx(uint8_t b);           /**< first byte = register pointer */
void    bridge_i2c_addr_read(void);         /**< address matched, master reads: snapshot */
uint8_t bridge_i2c_tx(void);                /**< next byte to send */
void    bridge_i2c_stop(void);

/* ---- main loop ---- */
uint8_t bridge_take_ctrl(void);             /**< CTRL bits written by the master, cleared */
uint8_t bridge_reads(void);                 /**< FRAME reads since boot (LED activity), wraps */

#endif /* BRIDGE_H */
