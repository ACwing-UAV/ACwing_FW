/**
 * @file  startup.c
 * @brief Minimal start-up of the bootloader: vector table (no interrupts are used), .data copy, .bss clear, main.
 */
#include <stdint.h>

extern uint32_t _estack, _sidata, _sdata, _edata, _sbss, _ebss;
int main(void);

void Reset_Handler(void)
{
    uint32_t *s = &_sidata, *d = &_sdata;
    while (d < &_edata) *d++ = *s++;
    for (d = &_sbss; d < &_ebss; ) *d++ = 0u;
    (void)main();
    for (;;) { }
}

static void fault(void) { for (;;) { } }

__attribute__((section(".isr_vector"), used))
const void *const g_vectors[16] = {
    &_estack, (const void *)Reset_Handler,
    (const void *)fault, (const void *)fault, (const void *)fault, (const void *)fault, (const void *)fault,
    0, 0, 0, 0, (const void *)fault, (const void *)fault, 0, (const void *)fault, (const void *)fault,
};

/* libc assert (bootutil_loader.c uses assert() directly): stop here, no printf / heap behind it */
void __assert_func(const char *file, int line, const char *func, const char *expr)
{
    (void)file; (void)line; (void)func; (void)expr;
    for (;;) { }
}
