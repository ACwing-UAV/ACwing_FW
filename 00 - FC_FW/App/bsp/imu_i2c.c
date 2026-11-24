/**
 * @file  imu_i2c.c
 */
#include "imu_i2c.h"

#define I2C_TIMEOUT_MS 10u
#define FLAG_DMA_DONE  0x0100u   /* bits distinct from the IMU IRQ flag used by the same task */
#define FLAG_DMA_ERR   0x0200u

#define LOCK_TIMEOUT_MS 5u

static I2C_HandleTypeDef *s_hi2c;
static uint16_t           s_addr8;
static osThreadId_t       s_waiter;
static osMutexId_t        s_lock;

/* ---- bus lock (IMU + expansion devices on the same I2C) ----
 * Priority inheritance: the ImuTask waits at most for one short expansion
 * transaction (a few hundred us at 400 kHz). Before the kernel runs, or
 * before imu_i2c_bus_init, there is no lock and only one user. */
void imu_i2c_bus_init(I2C_HandleTypeDef *hi2c)
{
    static const osMutexAttr_t a = { .name = "i2c1", .attr_bits = osMutexPrioInherit };
    s_hi2c = hi2c;
    if (!s_lock) s_lock = osMutexNew(&a);
}

int imu_i2c_lock(void)
{
    if (!s_lock || osKernelGetState() != osKernelRunning) return 0;
    return osMutexAcquire(s_lock, LOCK_TIMEOUT_MS) == osOK ? 0 : -1;
}

void imu_i2c_unlock(void)
{
    if (s_lock && osKernelGetState() == osKernelRunning) (void)osMutexRelease(s_lock);
}

I2C_HandleTypeDef *imu_i2c_handle(void) { return s_hi2c; }

static int32_t wr(void *h, uint8_t reg, const uint8_t *d, uint16_t len)
{
    (void)h;
    if (imu_i2c_lock()) return -1;
    const int32_t r = HAL_I2C_Mem_Write(s_hi2c, s_addr8, reg, I2C_MEMADD_SIZE_8BIT,
                                        (uint8_t *)d, len, I2C_TIMEOUT_MS) == HAL_OK ? 0 : -1;
    imu_i2c_unlock();
    return r;
}

static int32_t rd(void *h, uint8_t reg, uint8_t *d, uint16_t len)
{
    (void)h;
    if (imu_i2c_lock()) return -1;
    const int32_t r = HAL_I2C_Mem_Read(s_hi2c, s_addr8, reg, I2C_MEMADD_SIZE_8BIT,
                                       d, len, I2C_TIMEOUT_MS) == HAL_OK ? 0 : -1;
    imu_i2c_unlock();
    return r;
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
    if (imu_i2c_lock()) return -1;
    s_waiter = osThreadGetId();
    (void)osThreadFlagsClear(FLAG_DMA_DONE | FLAG_DMA_ERR);
    if (HAL_I2C_Mem_Read_DMA(s_hi2c, s_addr8, reg, I2C_MEMADD_SIZE_8BIT, buf, len) != HAL_OK) {
        imu_i2c_unlock();
        return -1;
    }
    uint32_t f = osThreadFlagsWait(FLAG_DMA_DONE | FLAG_DMA_ERR, osFlagsWaitAny, timeout_ms);
    imu_i2c_unlock();
    if ((int32_t)f < 0 || (f & FLAG_DMA_ERR)) return -1;
    return 0;
}

/* ---- bus recovery (#106) ----
 * NUCLEO-WB55 + IKS4A1: I2C1 on PB8 (SCL) / PB9 (SDA), AF4 (stm32wbxx_hal_msp.c). */
#ifndef IMU_I2C_SCL_PORT
#define IMU_I2C_SCL_PORT GPIOB
#define IMU_I2C_SCL_PIN  GPIO_PIN_8
#define IMU_I2C_SDA_PORT GPIOB
#define IMU_I2C_SDA_PIN  GPIO_PIN_9
#endif

static void half_bit(void)                       /* ~5 us: 100 kHz, slow enough for any slave */
{
    const uint32_t n = SystemCoreClock / 200000u;
    for (volatile uint32_t i = 0; i < n / 4u; ++i) { __NOP(); }
}

int imu_i2c_recover(void)
{
    if (!s_hi2c) return -1;
    const int locked = (imu_i2c_lock() == 0);
    (void)HAL_I2C_DeInit(s_hi2c);                /* stops the peripheral and its DMA, pins back to analog */

    GPIO_InitTypeDef g = { 0 };
    g.Mode  = GPIO_MODE_OUTPUT_OD;
    g.Pull  = GPIO_NOPULL;                       /* the board pull-ups */
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_WritePin(IMU_I2C_SCL_PORT, IMU_I2C_SCL_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IMU_I2C_SDA_PORT, IMU_I2C_SDA_PIN, GPIO_PIN_SET);
    g.Pin = IMU_I2C_SCL_PIN; HAL_GPIO_Init(IMU_I2C_SCL_PORT, &g);
    g.Pin = IMU_I2C_SDA_PIN; HAL_GPIO_Init(IMU_I2C_SDA_PORT, &g);
    half_bit();
    for (int k = 0; k < 9; ++k) {                /* a slave stuck mid-byte gets the clocks it waits for */
        HAL_GPIO_WritePin(IMU_I2C_SCL_PORT, IMU_I2C_SCL_PIN, GPIO_PIN_RESET); half_bit();
        HAL_GPIO_WritePin(IMU_I2C_SCL_PORT, IMU_I2C_SCL_PIN, GPIO_PIN_SET);   half_bit();
    }
    /* STOP: SDA low -> high while SCL is high */
    HAL_GPIO_WritePin(IMU_I2C_SCL_PORT, IMU_I2C_SCL_PIN, GPIO_PIN_RESET); half_bit();
    HAL_GPIO_WritePin(IMU_I2C_SDA_PORT, IMU_I2C_SDA_PIN, GPIO_PIN_RESET); half_bit();
    HAL_GPIO_WritePin(IMU_I2C_SCL_PORT, IMU_I2C_SCL_PIN, GPIO_PIN_SET);   half_bit();
    HAL_GPIO_WritePin(IMU_I2C_SDA_PORT, IMU_I2C_SDA_PIN, GPIO_PIN_SET);   half_bit();

    /* as MX_I2C1_Init: hi2c->Init is still the CubeMX configuration; the MSP re-does pins, DMA and IRQs */
    int ok = HAL_I2C_Init(s_hi2c) == HAL_OK;
    ok = ok && HAL_I2CEx_ConfigAnalogFilter(s_hi2c, I2C_ANALOGFILTER_ENABLE) == HAL_OK;
    ok = ok && HAL_I2CEx_ConfigDigitalFilter(s_hi2c, 0) == HAL_OK;
    if (locked) imu_i2c_unlock();
    return ok ? 0 : -1;
}

void imu_i2c_rx_cplt_isr(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == s_hi2c && s_waiter) (void)osThreadFlagsSet(s_waiter, FLAG_DMA_DONE);
}

void imu_i2c_error_isr(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == s_hi2c && s_waiter) (void)osThreadFlagsSet(s_waiter, FLAG_DMA_ERR);
}
