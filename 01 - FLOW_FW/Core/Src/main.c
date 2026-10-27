/**
 * @file  main.c
 * @brief ACwing Flow rev A bridge firmware (STM32C011F4U6).
 *
 *   PMW3901 --SPI1 (mode 3, 1.5 MHz)--> STM32C011 --I2C1 target 0x3B--> Qwiic --> ACwing FC
 *
 * Pins (ACwing_Flow_A schematic, checked against ST open pin data):
 *   PA0 TOF_XSHUT   left in its reset state (analog, high impedance): the
 *                   10 k pull-up keeps the VL53L1X running. Never drive it low.
 *   PA1 TOF_INT     not used (the FC polls the ToF), high impedance
 *   PA2 PMW_NRESET  output, high = running
 *   PA3 PMW_MOTION  not used (motion burst polled at 100 Hz)
 *   PA4 PMW_NCS     output, high = deselected
 *   PA5/6/7         SPI1 SCK / MISO / MOSI (AF0)
 *   PA8 LED_K       output, low = LED on
 *   PA13/PA14       SWD (untouched)
 *   PB6/PB7         I2C1 SCL / SDA (AF6, open drain, pull-ups on the FC)
 *
 * Main loop: every 10 ms a motion burst; the deltas of good samples are added
 * to the accumulators with the time they cover (bridge.c). The I2C interrupt
 * serves the register map and latches a frame when the FC reads from 0x02.
 * The independent watchdog (~0.5 s) is fed by the main loop only.
 *
 * LED: on = sensor OK and the FC reading frames; slow blink (1 Hz) = sensor
 * OK, no reads from the FC; fast blink (5 Hz) = PMW3901 not found (retried
 * every second); short flash every 2 s = sensor OK but no surface features.
 */
#include "stm32c0xx.h"
#include "bridge.h"
#include "pmw3901.h"
#include "flow_regs.h"

#define SYSCLK_HZ      48000000u
#define LOOP_MS        10u
#define REINIT_MS      1000u

static volatile uint32_t s_ms;

void SysTick_Handler(void) { s_ms++; }

/* ------------------------------------------------------------------ board */
static void iwdg_feed(void) { IWDG->KR = 0xAAAAu; }

static void clock_init(void)
{
    /* 1 wait state above 24 MHz, then HSISYS = HSI48 / 1 */
    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | FLASH_ACR_LATENCY_0;
    while ((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_0) { }
    RCC->CR &= ~RCC_CR_HSIDIV;
    SystemCoreClockUpdate();
    SysTick_Config(SYSCLK_HZ / 1000u);
}

static void gpio_mode(GPIO_TypeDef *g, uint32_t pin, uint32_t mode, uint32_t af)
{
    g->MODER = (g->MODER & ~(3u << (2u * pin))) | (mode << (2u * pin));
    g->OSPEEDR |= 3u << (2u * pin);
    volatile uint32_t *afr = &g->AFR[pin >> 3];
    *afr = (*afr & ~(0xFu << (4u * (pin & 7u)))) | (af << (4u * (pin & 7u)));
}

static void board_init(void)
{
    RCC->IOPENR  |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN;
    RCC->APBENR1 |= RCC_APBENR1_I2C1EN;
    RCC->APBENR2 |= RCC_APBENR2_SPI1EN | RCC_APBENR2_TIM14EN;
    (void)RCC->APBENR2;

    /* outputs first at their idle level: NCS high, NRESET high, LED off */
    GPIOA->BSRR = (1u << 4) | (1u << 2) | (1u << 8);
    gpio_mode(GPIOA, 2, 1u, 0);                     /* PMW_NRESET out  */
    gpio_mode(GPIOA, 4, 1u, 0);                     /* PMW_NCS out     */
    gpio_mode(GPIOA, 8, 1u, 0);                     /* LED out         */
    gpio_mode(GPIOA, 3, 0u, 0);                     /* MOTION in       */
    /* PA0 / PA1 (ToF) untouched: analog after reset = high impedance */
    gpio_mode(GPIOA, 5, 2u, 0);                     /* SPI1 SCK  AF0   */
    gpio_mode(GPIOA, 6, 2u, 0);                     /* SPI1 MISO AF0   */
    gpio_mode(GPIOA, 7, 2u, 0);                     /* SPI1 MOSI AF0   */
    GPIOB->OTYPER |= (1u << 6) | (1u << 7);         /* I2C open drain  */
    gpio_mode(GPIOB, 6, 2u, 6);                     /* I2C1 SCL AF6    */
    gpio_mode(GPIOB, 7, 2u, 6);                     /* I2C1 SDA AF6    */

    /* TIM14: free-running 1 MHz counter for the microsecond delays */
    TIM14->PSC = (SYSCLK_HZ / 1000000u) - 1u;
    TIM14->ARR = 0xFFFFu;
    TIM14->EGR = TIM_EGR_UG;
    TIM14->CR1 = TIM_CR1_CEN;

    /* SPI1: master, mode 3 (CPOL = CPHA = 1), 48 MHz / 32 = 1.5 MHz (PMW3901 <= 2 MHz), 8 bit */
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_CPOL | SPI_CR1_CPHA | SPI_CR1_SSM | SPI_CR1_SSI |
                (4u << SPI_CR1_BR_Pos);
    SPI1->CR2 = (7u << SPI_CR2_DS_Pos) | SPI_CR2_FRXTH;
    SPI1->CR1 |= SPI_CR1_SPE;

    /* I2C1 target at 0x3B, clock stretching allowed. TIMINGR for the target
     * only needs the data setup/hold: PRESC 5 (125 ns), SCLDEL 3 (500 ns),
     * SDADEL 2 (250 ns): standard and fast mode (400 kHz). */
    I2C1->CR1 = 0;
    I2C1->TIMINGR = (5u << I2C_TIMINGR_PRESC_Pos) | (3u << I2C_TIMINGR_SCLDEL_Pos) |
                    (2u << I2C_TIMINGR_SDADEL_Pos) | (3u << I2C_TIMINGR_SCLH_Pos) | 9u;
    I2C1->OAR1 = 0;
    I2C1->OAR1 = I2C_OAR1_OA1EN | ((uint32_t)FLOW_I2C_ADDR << 1);
    I2C1->CR1 = I2C_CR1_ADDRIE | I2C_CR1_RXIE | I2C_CR1_TXIE | I2C_CR1_STOPIE |
                I2C_CR1_NACKIE | I2C_CR1_ERRIE | I2C_CR1_PE;
    NVIC_SetPriority(I2C1_IRQn, 0);
    NVIC_EnableIRQ(I2C1_IRQn);
}

static void iwdg_start(void)
{
    DBG->APBFZ1 |= DBG_APB_FZ1_DBG_IWDG_STOP;      /* debugger halt: no reset */
    IWDG->KR  = 0xCCCCu;
    IWDG->KR  = 0x5555u;
    IWDG->PR  = 3u;                                /* LSI 32 kHz / 32 = 1 kHz */
    IWDG->RLR = 500u;                              /* 0.5 s                   */
    while (IWDG->SR != 0u) { }
    iwdg_feed();
}

/* ------------------------------------------------------------------ I2C target */
void I2C1_IRQHandler(void)
{
    const uint32_t isr = I2C1->ISR;
    if (isr & I2C_ISR_ADDR) {
        if (isr & I2C_ISR_DIR) {                   /* master reads */
            I2C1->ISR |= I2C_ISR_TXE;              /* flush a stale byte */
            bridge_i2c_addr_read();
        } else {
            bridge_i2c_addr_write();
        }
        I2C1->ICR = I2C_ICR_ADDRCF;
    }
    if (isr & I2C_ISR_RXNE) bridge_i2c_rx((uint8_t)I2C1->RXDR);
    if (isr & I2C_ISR_TXIS) I2C1->TXDR = bridge_i2c_tx();
    if (isr & I2C_ISR_NACKF) I2C1->ICR = I2C_ICR_NACKCF;
    if (isr & I2C_ISR_STOPF) {
        I2C1->ICR = I2C_ICR_STOPCF;
        I2C1->ISR |= I2C_ISR_TXE;
        bridge_i2c_stop();
    }
    if (isr & (I2C_ISR_BERR | I2C_ISR_ARLO | I2C_ISR_OVR))
        I2C1->ICR = I2C_ICR_BERRCF | I2C_ICR_ARLOCF | I2C_ICR_OVRCF;
}

/* ------------------------------------------------------------------ PMW3901 bus */
static void delay_us(uint32_t us)
{
    while (us) {
        const uint32_t step = (us > 10000u) ? 10000u : us;   /* < 16-bit counter range */
        const uint16_t t0 = (uint16_t)TIM14->CNT;
        while ((uint16_t)((uint16_t)TIM14->CNT - t0) < step) { }
        us -= step;
        iwdg_feed();                               /* long init delays (100 ms) */
    }
}

static uint8_t spi_xfer(uint8_t b)
{
    while (!(SPI1->SR & SPI_SR_TXE)) { }
    *(volatile uint8_t *)&SPI1->DR = b;
    while (!(SPI1->SR & SPI_SR_RXNE)) { }
    return *(volatile uint8_t *)&SPI1->DR;
}

static void pin_cs(uint8_t l)    { GPIOA->BSRR = l ? (1u << 4) : (1u << (4 + 16)); }
static void pin_reset(uint8_t l) { GPIOA->BSRR = l ? (1u << 2) : (1u << (2 + 16)); }
static void led(uint8_t on)      { GPIOA->BSRR = on ? (1u << (8 + 16)) : (1u << 8); }

static const pmw_bus_t k_bus = { spi_xfer, pin_cs, pin_reset, delay_us };

/* ------------------------------------------------------------------ main */
int main(void)
{
    clock_init();
    board_init();
    bridge_init();
    iwdg_start();

    uint8_t  ok = (uint8_t)(pmw_init(&k_bus) == 0);
    bridge_set_sensor_ok(ok);
    uint32_t t_next = s_ms, t_try = s_ms, t_reads = s_ms;
    uint16_t last_us = (uint16_t)TIM14->CNT;
    uint8_t  reads_seen = bridge_reads(), fc_active = 0, valid = 1;

    for (;;) {
        iwdg_feed();

        const uint8_t c = bridge_take_ctrl();
        if (c & FLOW_CTRL_RESET) NVIC_SystemReset();
        if ((c & FLOW_CTRL_REINIT) || (!ok && (s_ms - t_try) >= REINIT_MS)) {
            t_try = s_ms;
            bridge_set_sensor_ok(0);
            ok = (uint8_t)(pmw_init(&k_bus) == 0);
            bridge_set_sensor_ok(ok);
            last_us = (uint16_t)TIM14->CNT;
        }

        if ((int32_t)(s_ms - t_next) >= 0) {
            t_next += LOOP_MS;
            if ((int32_t)(s_ms - t_next) > 0) t_next = s_ms + LOOP_MS;   /* after a re-init */
            if (ok) {
                pmw_burst_t b;
                pmw_burst(&b);
                const uint16_t now_us = (uint16_t)TIM14->CNT;
                const uint32_t dt = (uint16_t)(now_us - last_us);         /* <= 65 ms: fine at 10 ms */
                last_us = now_us;
                valid = pmw_valid(&b);
                __disable_irq();
                bridge_add_sample(b.dx, b.dy, b.squal, b.shutter, valid, dt);
                __enable_irq();
            }
        }

        /* the FC is reading frames: any FRAME read in the last second */
        if (bridge_reads() != reads_seen) { reads_seen = bridge_reads(); t_reads = s_ms; }
        fc_active = (uint8_t)((s_ms - t_reads) < 1000u);

        if (!ok)            led((s_ms / 100u) & 1u);                 /* 5 Hz            */
        else if (!valid)    led((s_ms % 2000u) < 50u);               /* short flash     */
        else if (!fc_active) led((s_ms / 500u) & 1u);                /* 1 Hz            */
        else                led(1);
    }
}
