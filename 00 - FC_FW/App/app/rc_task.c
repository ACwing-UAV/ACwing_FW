/**
 * @file  rc_task.c
 * @brief RcTask (see rc_task.h). 2 ms period: a CRSF frame every 2-4 ms, SBUS 9-14 ms.
 */
#include "app_config.h"
#if APP_RC_UART && !defined(HOST_TEST)
#include "rc_task.h"
#include "rc_uart.h"
#include "rc_input.h"
#include "params.h"
#include "cmsis_os2.h"

#define RC_TASK_MS     2u
#define RC_PARAM_MS    200u      /* parameters checked at 5 Hz */

static void follow_params(void)
{
    const uint8_t proto = (uint8_t)(PARAM(P_RC_PROTOCOL) + 0.5f);
    const uint8_t map = (uint8_t)(PARAM(P_RC_MAP) + 0.5f);
    if (rc_input_configure(proto, map)) {
        rc_uart_cfg_t c;
        rc_input_uart_cfg(rc_input_protocol(), &c);
        (void)rc_uart_open(&c);
    }
}

static void RcTask(void *arg)
{
    (void)arg;
    uint8_t buf[128];
    uint32_t last_param = 0;
    rc_input_init();
    follow_params();
    for (;;) {
        const uint32_t now = osKernelGetTickCount();
        if ((now - last_param) >= RC_PARAM_MS) { last_param = now; follow_params(); }
        const uint32_t n = rc_uart_read(buf, sizeof buf);
        rc_input_feed(buf, n, now);
        rc_input_service(now);
        osDelay(RC_TASK_MS);
    }
}

void rc_task_start(void)
{
    static const osThreadAttr_t a = { .name = "rc", .priority = osPriorityAboveNormal, .stack_size = 768 };
    (void)osThreadNew(RcTask, NULL, &a);
}
#endif
