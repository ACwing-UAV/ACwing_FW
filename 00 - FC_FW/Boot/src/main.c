/**
 * @file  main.c
 * @brief ACwing FC bootloader: MCUboot on the STM32WB55 (ADR 0015 §1).
 *
 *   1. clock to 64 MHz (HSI16 -> PLL): the SHA-256 of a 230 KB slot takes ~0.3 s instead of ~5 s at the 4 MHz MSI;
 *   2. boot_go(): swap a new image in for a test boot, revert an unconfirmed one, check the signature of slot 0;
 *   3. clock back to the reset state (MSI 4 MHz, PLL off): the application's SystemClock_Config starts from there;
 *   4. jump to the application vector table (its SystemInit sets VTOR, USER_VECT_TAB_ADDRESS).
 * Nothing valid to start: stay here with the LED pattern off and wait for a debugger / ROM DFU (ADR 0015 §6).
 */
#include "stm32wbxx.h"
#include "bootutil/bootutil.h"
#include "bootutil/image.h"
#include "flash_map_backend/flash_map_backend.h"
#include "acwing_flash_layout.h"

extern const acw_flash_ops_t g_wb_flash_ops;

static void clock_64mhz(void)
{
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY)) { }
    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | FLASH_ACR_LATENCY_3WS | FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN;
    while ((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_3WS) { }
    RCC->CR &= ~RCC_CR_PLLON;
    while (RCC->CR & RCC_CR_PLLRDY) { }
    /* HSI16 / M=1 * N=8 = 128 MHz VCO, / R=2 = 64 MHz */
    RCC->PLLCFGR = (2u << RCC_PLLCFGR_PLLSRC_Pos) | (0u << RCC_PLLCFGR_PLLM_Pos) | (8u << RCC_PLLCFGR_PLLN_Pos) |
                   (1u << RCC_PLLCFGR_PLLR_Pos) | RCC_PLLCFGR_PLLREN;
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) { }
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | (3u << RCC_CFGR_SW_Pos);
    while (((RCC->CFGR & RCC_CFGR_SWS) >> RCC_CFGR_SWS_Pos) != 3u) { }
}

static void clock_reset_state(void)
{
    RCC->CR |= RCC_CR_MSION;
    while (!(RCC->CR & RCC_CR_MSIRDY)) { }
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW);              /* MSI */
    while ((RCC->CFGR & RCC_CFGR_SWS) != 0u) { }
    RCC->CR &= ~RCC_CR_PLLON;
    while (RCC->CR & RCC_CR_PLLRDY) { }
    RCC->PLLCFGR = 0x22041000u;                         /* reset value */
    /* the flash latency stays at 3 WS: always safe; SystemClock_Config sets it again */
}

static void __attribute__((noreturn)) jump(uint32_t vectors)
{
    const uint32_t sp = ((const uint32_t *)(uintptr_t)vectors)[0];
    const uint32_t pc = ((const uint32_t *)(uintptr_t)vectors)[1];
    __disable_irq();
    SysTick->CTRL = 0;
    SCB->VTOR = vectors;
    __DSB(); __ISB();
    __set_MSP(sp);
    __enable_irq();
    ((void (*)(void))(uintptr_t)pc)();
    for (;;) { }
}

int main(void)
{
    clock_64mhz();
    acw_flash_set_ops(&g_wb_flash_ops);
    struct boot_rsp rsp;
    const fih_ret ok = boot_go(&rsp);
    if (FIH_NOT_EQ(ok, FIH_SUCCESS)) {
        for (;;) { __WFI(); }                            /* no valid image: debugger / ROM DFU */
    }
    const uint32_t vectors = ACW_FLASH_BASE + rsp.br_image_off + rsp.br_hdr->ih_hdr_size;
    clock_reset_state();
    jump(vectors);
}
