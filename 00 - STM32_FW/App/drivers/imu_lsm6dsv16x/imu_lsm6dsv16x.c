/**
 * @file  imu_lsm6dsv16x.c
 *
 * The sensor-hub configuration sequence follows the official ST example
 * lsm6dsv16x_sensor_hub.c (STMems_Standard_C_drivers), with explicit timeouts
 * instead of infinite loops.
 */
#include "imu_lsm6dsv16x.h"
#include <string.h>

/* ---- LIS2MDL (registers from datasheet / lis2mdl-pid) ---- */
#define LIS2MDL_ADDR_8BIT   0x3Du    /* same as LIS2MDL_I2C_ADD in lis2mdl_reg.h */
#define LIS2MDL_WHO_AM_I    0x4Fu
#define LIS2MDL_ID          0x40u
#define LIS2MDL_CFG_A       0x60u
#define LIS2MDL_CFG_B       0x61u
#define LIS2MDL_CFG_C       0x62u
#define LIS2MDL_OUTX_L      0x68u
#define LIS2MDL_UT_PER_LSB  0.15f

/* ---- LPS22DF (IKS4A1) / LPS22HB (ACwing FC rev A), behind the same hub ---- */
#define LPS22_ADDR_8BIT_H   0xBAu    /* SA0 = 1 -> 0x5D */
#define LPS22_ADDR_8BIT_L   0xB8u    /* SA0 = 0 -> 0x5C */
#define LPS22_WHO_AM_I      0x0Fu
#define LPS22DF_ID          0xB4u
#define LPS22HB_ID          0xB1u
#define LPS22_CTRL_REG1     0x10u
#define LPS22_CTRL_REG2     0x11u
#define LPS22_PRESS_OUT_XL  0x28u    /* PRESS XL, L, H, TEMP L, H */

#define DEG2RAD_F           0.017453292519943295f
#define SH_POLL_MAX         50u      /* x 5 ms */

static void dly(imu_t *imu, uint32_t ms) { if (imu->bus.delay_ms) imu->bus.delay_ms(ms); }

/* Waits for the sensor hub to complete one cycle (triggered by the XL at 120 Hz) */
static int32_t sh_run_once(imu_t *imu)
{
    lsm6dsv16x_data_ready_t     drdy;
    lsm6dsv16x_status_master_t  ms;
    int16_t raw[3];
    int32_t ret = 0;

    ret |= lsm6dsv16x_sh_master_set(&imu->ctx, PROPERTY_ENABLE);
    ret |= lsm6dsv16x_xl_data_rate_set(&imu->ctx, LSM6DSV16X_ODR_AT_120Hz);
    ret |= lsm6dsv16x_acceleration_raw_get(&imu->ctx, raw);
    if (ret) return IMU_ERR_BUS;

    uint32_t i = 0;
    do { dly(imu, 5); if (lsm6dsv16x_flag_data_ready_get(&imu->ctx, &drdy)) return IMU_ERR_BUS; }
    while (!drdy.drdy_xl && ++i < SH_POLL_MAX);
    i = 0;
    do { dly(imu, 5); if (lsm6dsv16x_sh_status_get(&imu->ctx, &ms)) return IMU_ERR_BUS; }
    while (!ms.sens_hub_endop && ++i < SH_POLL_MAX);

    ret |= lsm6dsv16x_sh_master_set(&imu->ctx, PROPERTY_DISABLE);
    ret |= lsm6dsv16x_xl_data_rate_set(&imu->ctx, LSM6DSV16X_ODR_OFF);
    if (ret) return IMU_ERR_BUS;
    return ms.sens_hub_endop ? IMU_OK : IMU_ERR_SH_TIMEOUT;
}

static int32_t sh_write(imu_t *imu, uint8_t addr8, uint8_t reg, uint8_t val)
{
    lsm6dsv16x_sh_cfg_write_t w = { addr8, reg, val };
    if (lsm6dsv16x_xl_data_rate_set(&imu->ctx, LSM6DSV16X_ODR_OFF)) return IMU_ERR_BUS;
    if (lsm6dsv16x_sh_cfg_write(&imu->ctx, &w))                     return IMU_ERR_BUS;
    if (lsm6dsv16x_sh_write_mode_set(&imu->ctx, LSM6DSV16X_ONLY_FIRST_CYCLE)) return IMU_ERR_BUS;
    return sh_run_once(imu);
}

static int32_t sh_read(imu_t *imu, uint8_t addr8, uint8_t reg, uint8_t *buf, uint8_t len)
{
    lsm6dsv16x_sh_cfg_read_t r = { addr8, reg, len };
    if (lsm6dsv16x_xl_data_rate_set(&imu->ctx, LSM6DSV16X_ODR_OFF))       return IMU_ERR_BUS;
    if (lsm6dsv16x_sh_slv_cfg_read(&imu->ctx, 0, &r))                      return IMU_ERR_BUS;
    if (lsm6dsv16x_sh_slave_connected_set(&imu->ctx, LSM6DSV16X_SLV_0))    return IMU_ERR_BUS;
    int32_t st = sh_run_once(imu);
    if (st) return st;
    return lsm6dsv16x_sh_read_data_raw_get(&imu->ctx, buf, len) ? IMU_ERR_BUS : IMU_OK;
}

static int32_t mag_setup(imu_t *imu, uint8_t fast)
{
    uint8_t id = 0;
    int32_t st = sh_read(imu, LIS2MDL_ADDR_8BIT, LIS2MDL_WHO_AM_I, &id, 1);
    if (st) return st;
    if (id != LIS2MDL_ID) return IMU_ERR_MAG_ID;

    if ((st = sh_write(imu, LIS2MDL_ADDR_8BIT, LIS2MDL_CFG_A, 0x20u))) return st;    /* SOFT_RST              */
    dly(imu, 10);
    if ((st = sh_write(imu, LIS2MDL_ADDR_8BIT, LIS2MDL_CFG_C, 0x10u))) return st;    /* BDU                   */
    if ((st = sh_write(imu, LIS2MDL_ADDR_8BIT, LIS2MDL_CFG_B, 0x03u))) return st;    /* OFF_CANC | LPF        */
    /* COMP_TEMP_EN | ODR 50 or 100 Hz | continuous. The sensor hub re-reads at 60 or
     * 120 Hz: ~1 read in 6 is a duplicate, discarded by the parser. */
    return sh_write(imu, LIS2MDL_ADDR_8BIT, LIS2MDL_CFG_A, fast ? 0x8Cu : 0x88u);
}

/* Optional barometer: a missing or silent part is not an error (the IMU and
 * the attitude estimate must start without it). 0 = found and configured. */
static int32_t baro_setup(imu_t *imu)
{
    static const uint8_t addr[2] = { LPS22_ADDR_8BIT_H, LPS22_ADDR_8BIT_L };
    for (unsigned k = 0; k < 2u; ++k) {
        uint8_t id = 0;
        if (sh_read(imu, addr[k], LPS22_WHO_AM_I, &id, 1) != IMU_OK) continue;
        if (id == LPS22DF_ID) {
            /* CTRL_REG2: SWRESET, then BDU | EN_LPFP (ODR/4); CTRL_REG1: ODR 50 Hz, AVG 16 */
            if (sh_write(imu, addr[k], LPS22_CTRL_REG2, 0x04u)) return -1;
            dly(imu, 5);
            if (sh_write(imu, addr[k], LPS22_CTRL_REG2, 0x18u)) return -1;
            if (sh_write(imu, addr[k], LPS22_CTRL_REG1, 0x2Au)) return -1;
            imu->baro_model = IMU_BARO_LPS22DF;
        } else if (id == LPS22HB_ID) {
            /* CTRL_REG2: SWRESET (IF_ADD_INC kept); CTRL_REG1: ODR 50 Hz | EN_LPFP | BDU */
            if (sh_write(imu, addr[k], LPS22_CTRL_REG2, 0x14u)) return -1;
            dly(imu, 5);
            if (sh_write(imu, addr[k], LPS22_CTRL_REG1, 0x4Au)) return -1;
            imu->baro_model = IMU_BARO_LPS22HB;
        } else {
            continue;
        }
        imu->baro_addr8 = addr[k];
        return 0;
    }
    return -1;
}

int32_t imu_init(imu_t *imu, const sensor_bus_t *bus, const imu_config_t *cfg)
{
    if (!imu || !bus || !cfg || !bus->read || !bus->write) return IMU_ERR_ARG;
    memset(imu, 0, sizeof *imu);
    imu->bus          = *bus;
    imu->ctx.write_reg = bus->write;
    imu->ctx.read_reg  = bus->read;
    imu->ctx.mdelay    = bus->delay_ms;
    imu->ctx.handle    = bus->handle;

    uint8_t id = 0;
    if (lsm6dsv16x_device_id_get(&imu->ctx, &id)) return IMU_ERR_BUS;
    if (id != LSM6DSV16X_ID)                      return IMU_ERR_ID;
    if (lsm6dsv16x_sw_por(&imu->ctx))             return IMU_ERR_BUS;
    dly(imu, 10);
    if (lsm6dsv16x_block_data_update_set(&imu->ctx, PROPERTY_ENABLE)) return IMU_ERR_BUS;

    /* 1) magnetometer behind the sensor hub */
    if (cfg->mag_enable) {
        int32_t st = mag_setup(imu, cfg->mag_fast);
        if (st) return st;
        imu->mag_enabled = 1;
    }
    if (cfg->baro_enable && baro_setup(imu) == 0) imu->baro_enabled = 1;
    /* hub slots are consecutive from 0: mag first, then baro */
    imu->baro_slot = imu->mag_enabled ? 1u : 0u;

    /* 2) full scales and filters */
    int32_t ret = 0;
    ret |= lsm6dsv16x_gy_full_scale_set(&imu->ctx, LSM6DSV16X_2000dps);
    ret |= lsm6dsv16x_xl_full_scale_set(&imu->ctx, LSM6DSV16X_16g);
    ret |= lsm6dsv16x_filt_gy_lp1_set(&imu->ctx, PROPERTY_ENABLE);
    ret |= lsm6dsv16x_filt_gy_lp1_bandwidth_set(&imu->ctx,
                (lsm6dsv16x_filt_gy_lp1_bandwidth_t)cfg->gy_lpf1_bw);
    ret |= lsm6dsv16x_filt_xl_lp2_set(&imu->ctx, cfg->xl_lpf2_enable ? PROPERTY_ENABLE : PROPERTY_DISABLE);
    if (cfg->xl_lpf2_enable)
        ret |= lsm6dsv16x_filt_xl_lp2_bandwidth_set(&imu->ctx,
                    (lsm6dsv16x_filt_xl_lp2_bandwidth_t)cfg->xl_lpf2_bw);
    if (ret) return IMU_ERR_BUS;

    /* 3) FIFO in stream mode, gyro + acc + (mag) */
    const int fast = (cfg->odr == IMU_ODR_1920HZ);
    ret |= lsm6dsv16x_fifo_watermark_set(&imu->ctx, cfg->watermark_words);
    ret |= lsm6dsv16x_fifo_xl_batch_set(&imu->ctx, fast ? LSM6DSV16X_XL_BATCHED_AT_1920Hz
                                                        : LSM6DSV16X_XL_BATCHED_AT_960Hz);
    ret |= lsm6dsv16x_fifo_gy_batch_set(&imu->ctx, fast ? LSM6DSV16X_GY_BATCHED_AT_1920Hz
                                                        : LSM6DSV16X_GY_BATCHED_AT_960Hz);
    if (imu->mag_enabled)  ret |= lsm6dsv16x_fifo_sh_batch_slave_set(&imu->ctx, 0, PROPERTY_ENABLE);
    if (imu->baro_enabled) ret |= lsm6dsv16x_fifo_sh_batch_slave_set(&imu->ctx, imu->baro_slot, PROPERTY_ENABLE);
    ret |= lsm6dsv16x_fifo_mode_set(&imu->ctx, LSM6DSV16X_STREAM_MODE);
    if (ret) return IMU_ERR_BUS;

    /* 4) sensor hub: continuous read of LIS2MDL OUTX_L..OUTZ_H and of the
     *    barometer PRESS_OUT_XL..TEMP_OUT_H */
    if (imu->mag_enabled || imu->baro_enabled) {
        if (imu->mag_enabled) {
            lsm6dsv16x_sh_cfg_read_t r = { LIS2MDL_ADDR_8BIT, LIS2MDL_OUTX_L, 6 };
            ret |= lsm6dsv16x_sh_slv_cfg_read(&imu->ctx, 0, &r);
        }
        if (imu->baro_enabled) {
            lsm6dsv16x_sh_cfg_read_t r = { imu->baro_addr8, LPS22_PRESS_OUT_XL, 5 };
            ret |= lsm6dsv16x_sh_slv_cfg_read(&imu->ctx, imu->baro_slot, &r);
        }
        ret |= lsm6dsv16x_sh_data_rate_set(&imu->ctx, cfg->mag_fast ? LSM6DSV16X_SH_120Hz
                                                                     : LSM6DSV16X_SH_60Hz);
        ret |= lsm6dsv16x_sh_slave_connected_set(&imu->ctx,
                   (imu->mag_enabled && imu->baro_enabled) ? LSM6DSV16X_SLV_0_1 : LSM6DSV16X_SLV_0);
        ret |= lsm6dsv16x_sh_master_set(&imu->ctx, PROPERTY_ENABLE);
        if (ret) return IMU_ERR_BUS;
    }

    /* 5) FIFO threshold on INT1 */
    lsm6dsv16x_pin_int_route_t pin = { 0 };
    pin.fifo_th = PROPERTY_ENABLE;
    if (lsm6dsv16x_pin_int1_route_set(&imu->ctx, &pin)) return IMU_ERR_BUS;

    /* 6) start: high-performance mode, same ODR */
    const lsm6dsv16x_data_rate_t odr = fast ? LSM6DSV16X_ODR_AT_1920Hz : LSM6DSV16X_ODR_AT_960Hz;
    ret |= lsm6dsv16x_xl_mode_set(&imu->ctx, LSM6DSV16X_XL_HIGH_PERFORMANCE_MD);
    ret |= lsm6dsv16x_gy_mode_set(&imu->ctx, LSM6DSV16X_GY_HIGH_PERFORMANCE_MD);
    ret |= lsm6dsv16x_xl_data_rate_set(&imu->ctx, odr);
    ret |= lsm6dsv16x_gy_data_rate_set(&imu->ctx, odr);
    if (ret) return IMU_ERR_BUS;

    imu->odr_hz = fast ? 1920.0f : 960.0f;
    return IMU_OK;
}

int32_t imu_fifo_level(imu_t *imu, uint16_t *words)
{
    lsm6dsv16x_fifo_status_t s;
    if (lsm6dsv16x_fifo_status_get(&imu->ctx, &s)) return IMU_ERR_BUS;
    *words = s.fifo_level;
    return IMU_OK;
}

int32_t imu_fifo_read_burst(imu_t *imu, uint8_t *raw, uint16_t words)
{
    if (words > IMU_FIFO_MAX_WORDS) words = IMU_FIFO_MAX_WORDS;
    return imu->bus.read(imu->bus.handle, IMU_FIFO_REG, raw,
                         (uint16_t)(words * IMU_FIFO_WORD_BYTES)) ? IMU_ERR_BUS : IMU_OK;
}

static int16_t le16(const uint8_t *p) { return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8)); }

size_t imu_fifo_parse(imu_t *imu, const uint8_t *raw, uint16_t words,
                      imu_event_t *ev, size_t cap, imu_parse_stats_t *st)
{
    /* 2000 dps -> 70 mdps/LSB ; 16 g -> 0.488 mg/LSB (as lsm6dsv16x_from_fs*_) */
    const float kg = 70.0e-3f * DEG2RAD_F;
    const float ka = 0.488e-3f;
    size_t n = 0;

    for (uint16_t w = 0; w < words && n < cap; ++w) {
        const uint8_t *p   = &raw[w * IMU_FIFO_WORD_BYTES];
        const uint8_t  tag = (uint8_t)(p[0] >> 3);          /* TAG_SENSOR[4:0] = bit 7..3 */
        const uint8_t *d   = &p[1];
        if (st) st->words++;

        switch (tag) {
        case LSM6DSV16X_GY_NC_TAG:
            ev[n].type = IMU_EV_GYRO;
            for (int i = 0; i < 3; ++i) ev[n].v[i] = (float)le16(&d[2*i]) * kg;
            n++; if (st) st->gyro++;
            break;
        case LSM6DSV16X_XL_NC_TAG:
            ev[n].type = IMU_EV_ACC;
            for (int i = 0; i < 3; ++i) ev[n].v[i] = (float)le16(&d[2*i]) * ka;
            n++; if (st) st->acc++;
            break;
        case LSM6DSV16X_SENSORHUB_SLAVE0_TAG:
        case LSM6DSV16X_SENSORHUB_SLAVE1_TAG: {
            const uint8_t slot = (tag == LSM6DSV16X_SENSORHUB_SLAVE0_TAG) ? 0u : 1u;
            if (imu->baro_enabled && slot == imu->baro_slot) {
                /* the hub re-reads at 60/120 Hz, the baro converts at 50 Hz */
                if (imu->have_baro && memcmp(d, imu->last_baro_raw, 5) == 0) { if (st) st->baro_dup++; break; }
                memcpy(imu->last_baro_raw, d, 5);
                imu->have_baro = 1;
                int32_t pr = (int32_t)((uint32_t)d[0] | ((uint32_t)d[1] << 8) | ((uint32_t)d[2] << 16));
                if (pr & 0x00800000) pr -= 0x01000000;               /* 24-bit two's complement */
                ev[n].type = IMU_EV_BARO;
                ev[n].v[0] = (float)pr * (100.0f / 4096.0f);          /* Pa  (LSB = 1/4096 hPa) */
                ev[n].v[1] = (float)le16(&d[3]) * 0.01f;              /* degC                   */
                ev[n].v[2] = 0.0f;
                n++; if (st) st->baro++;
                break;
            }
            if (!imu->mag_enabled || slot != 0u) { if (st) st->unknown++; break; }
            if (imu->have_mag && memcmp(d, imu->last_mag_raw, 6) == 0) {
                if (st) st->mag_dup++;                      /* re-read of the same sample      */
                break;
            }
            memcpy(imu->last_mag_raw, d, 6);
            imu->have_mag = 1;
            ev[n].type = IMU_EV_MAG;
            for (int i = 0; i < 3; ++i) ev[n].v[i] = (float)le16(&d[2*i]) * LIS2MDL_UT_PER_LSB;
            n++; if (st) st->mag++;
            break;
        }
        case LSM6DSV16X_SENSORHUB_NACK_TAG:
            if (st) st->sh_nack++;
            break;
        case LSM6DSV16X_FIFO_EMPTY:
            if (st) st->empty++;
            break;
        default:
            if (st) st->unknown++;
            break;
        }
    }
    return n;
}
