/* MCUboot assert: no libc assert in the bootloader (it would pull printf and the heap); stop instead. */
#ifndef ACW_MCUBOOT_ASSERT_H
#define ACW_MCUBOOT_ASSERT_H
#define ASSERT(x) do { if (!(x)) { for (;;) { } } } while (0)
#define assert(x) ASSERT(x)
#endif
