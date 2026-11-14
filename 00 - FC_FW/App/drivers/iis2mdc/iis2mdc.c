/**
 * @file  iis2mdc.c
 * @brief IIS2MDC on a direct I2C bus (see iis2mdc.h).
 */
#include "iis2mdc.h"
#include <string.h>

static int32_t wr(iis2mdc_t *m, uint8_t reg, uint8_t v) { return m->bus.write(m->bus.handle, reg, &v, 1u); }

int32_t iis2mdc_init(iis2mdc_t *m, const sensor_bus_t *bus)
{
    memset(m, 0, sizeof *m);
    m->bus = *bus;
    uint8_t id = 0u;
    if (m->bus.read(m->bus.handle, IIS2MDC_REG_WHO_AM_I, &id, 1u) != 0 || id != IIS2MDC_WHO_AM_I_VAL) return -1;
    if (wr(m, IIS2MDC_REG_CFG_A, IIS2MDC_SOFT_RST) != 0) return -1;
    if (m->bus.delay_ms) m->bus.delay_ms(5u);
    if (wr(m, IIS2MDC_REG_CFG_A, IIS2MDC_CFG_A_50HZ) != 0 ||
        wr(m, IIS2MDC_REG_CFG_B, IIS2MDC_CFG_B) != 0 ||
        wr(m, IIS2MDC_REG_CFG_C, IIS2MDC_CFG_C) != 0) return -1;
    uint8_t a = 0u;
    if (m->bus.read(m->bus.handle, IIS2MDC_REG_CFG_A, &a, 1u) != 0 || a != IIS2MDC_CFG_A_50HZ) return -1;
    m->ok = 1u;
    return 0;
}

int32_t iis2mdc_poll(iis2mdc_t *m, int16_t raw[3])
{
    uint8_t d[7];
    /* STATUS and the outputs in one transaction: 0x67..0x6D */
    if (m->bus.read(m->bus.handle, IIS2MDC_REG_STATUS, d, 7u) != 0) return -1;
    if (!(d[0] & 0x08u)) return 0;
    for (int i = 0; i < 3; ++i) raw[i] = (int16_t)(d[1 + 2 * i] | (d[2 + 2 * i] << 8));
    return 1;
}
