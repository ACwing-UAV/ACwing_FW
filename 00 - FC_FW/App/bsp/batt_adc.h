/**
 * @file  batt_adc.h
 * @brief ADC1 binding of the BattTask: VBAT and current pins, VREFINT.
 *
 * Injected group of three conversions (VBAT, CUR, VREFINT), 16× hardware
 * oversampling, 247.5-cycle sampling (the 100k/10k divider is a 9 kΩ
 * source), started by software. batt_adc_read() returns the previous
 * conversion and starts the next one, so the BattTask never waits for the
 * ADC. Pin voltages are scaled with VREFINT and its factory calibration, so
 * the result does not depend on the exact 3.1 V supply.
 *
 * Register level (LL), no CubeMX ADC configuration needed. Channels and pins
 * come from app_config.h (APP_BATT_*).
 */
#ifndef BATT_ADC_H
#define BATT_ADC_H

int  batt_adc_init(void);                        /**< 0 = ok */
/** 0 = a new sample: pin voltages in volts */
int  batt_adc_read(float *vbat_pin, float *cur_pin);

#endif /* BATT_ADC_H */
