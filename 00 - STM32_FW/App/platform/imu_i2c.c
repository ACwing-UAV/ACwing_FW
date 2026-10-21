/**
 * @file  imu_i2c.c
 */
#include "imu_i2c.h"

#define I2C_TIMEOUT_MS 10u
#define FLAG_DMA_DONE  0x0100u   /* bits distinct from the IMU IRQ flag used by the same task */
#define FLAG_DMA_ERR   0x0200u

static I2C_HandleTypeDef *s_hi2c;
static uint16_t           s_addr8;
static osThreadId_t       s_waiter;

static int32_t wr(void *h, uint8_t reg, const uint8_t *d, uint16_t len)
{
    (void)h;
    return HAL_I2C_Mem_Write(s_hi2c, s_addr8, reg, I2C_MEMADD_SIZE_8BIT,
                             (uint8_t *)d, len, I2C_TIMEOUT_MS) == HAL_OK ? 0 : -1;
}

static int32_t rd(void *h, uint8_t reg, uint8_t *d, uint16_t len)
{
    (void)h;
    return HAL_I2C_Mem_Read(s_hi2c, s_addr8, reg, I2C_MEMADD_SIZE_8BIT,
                            d, len, I2C_TIMEOUT_MS) == HAL_OK ? 0 : -1;
}

static void dly(uint32_t ms)
{
    if (osKernelGetState() == osKernelRunning) osDelay(ms); else HAL_Delay(ms);
}

void imu_i2c_make_bus(sensor_bus_t *bus, I2C_HandleTypeDef *hi2c, uint8_t addr7)
{
    s_hi2c        = hi2c;
    s_addr8       = (uint16_t)(addr7 << 1);
    bus->write    = wr;
    bus->read     = rd;
    bus->delay_ms = dly;
    bus->handle   = NULL;
}

int32_t imu_i2c_read_dma(uint8_t reg, uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
    s_waiter = osThreadGetId();
    (void)osThreadFlagsClear(FLAG_DMA_DONE | FLAG_DMA_ERR);
    if (HAL_I2C_Mem_Read_DMA(s_hi2c, s_addr8, reg, I2C_MEMADD_SIZE_8BIT, buf, len) != HAL_OK)
        return -1;
    uint32_t f = osThreadFlagsWait(FLAG_DMA_DONE | FLAG_DMA_ERR, osFlagsWaitAny, timeout_ms);
    if ((int32_t)f < 0 || (f & FLAG_DMA_ERR)) return -1;
    return 0;
}

void imu_i2c_rx_cplt_isr(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == s_hi2c && s_waiter) (void)osThreadFlagsSet(s_waiter, FLAG_DMA_DONE);
}

void imu_i2c_error_isr(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == s_hi2c && s_waiter) (void)osThreadFlagsSet(s_waiter, FLAG_DMA_ERR);
}
