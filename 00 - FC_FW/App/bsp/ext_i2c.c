/**
 * @file  ext_i2c.c
 * @brief HAL binding of the Qwiic devices (see ext_i2c.h).
 */
#include "ext_i2c.h"
#include "imu_i2c.h"
#include "cmsis_os2.h"
#include "main.h"

#define EXT_I2C_TIMEOUT_MS 3u

static int32_t xfer(void *h, uint16_t reg, uint16_t regsz, uint8_t *d, uint16_t len, int write)
{
    const uint16_t a8 = (uint16_t)((uintptr_t)h << 1);
    if (imu_i2c_lock()) return -1;
    const HAL_StatusTypeDef st = write
        ? HAL_I2C_Mem_Write(imu_i2c_handle(), a8, reg, regsz, d, len, EXT_I2C_TIMEOUT_MS)
        : HAL_I2C_Mem_Read (imu_i2c_handle(), a8, reg, regsz, d, len, EXT_I2C_TIMEOUT_MS);
    imu_i2c_unlock();
    return st == HAL_OK ? 0 : -1;
}

static int32_t wr16(void *h, uint16_t reg, const uint8_t *d, uint16_t n) { return xfer(h, reg, I2C_MEMADD_SIZE_16BIT, (uint8_t *)d, n, 1); }
static int32_t rd16(void *h, uint16_t reg, uint8_t *d, uint16_t n)       { return xfer(h, reg, I2C_MEMADD_SIZE_16BIT, d, n, 0); }
static int32_t wr8(void *h, uint8_t reg, const uint8_t *d, uint16_t n)   { return xfer(h, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *)d, n, 1); }
static int32_t rd8(void *h, uint8_t reg, uint8_t *d, uint16_t n)         { return xfer(h, reg, I2C_MEMADD_SIZE_8BIT, d, n, 0); }
static void    dly(uint32_t ms) { osDelay(ms); }

void ext_i2c_tof_bus(vl53_bus_t *b, uint8_t addr7)
{
    b->write = wr16; b->read = rd16; b->delay_ms = dly; b->handle = (void *)(uintptr_t)addr7;
}

void ext_i2c_flow_bus(sensor_bus_t *b, uint8_t addr7)
{
    b->write = wr8; b->read = rd8; b->delay_ms = dly; b->handle = (void *)(uintptr_t)addr7;
}
