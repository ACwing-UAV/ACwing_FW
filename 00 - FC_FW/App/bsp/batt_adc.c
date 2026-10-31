/**
 * @file  batt_adc.c
 * @brief ADC1 injected-group binding for the battery (see batt_adc.h).
 *        Reference: RM0434 §16 (ADC), STM32WB55 datasheet (VREFINT_CAL).
 */
#include "batt_adc.h"
#include "app_config.h"
#include "stm32wbxx_hal.h"
#include "stm32wbxx_ll_adc.h"
#include "stm32wbxx_ll_bus.h"
#include "stm32wbxx_ll_gpio.h"
#include "stm32wbxx_ll_rcc.h"

static uint8_t s_started;

static void delay_us(uint32_t us)
{
    const uint32_t n = us * (SystemCoreClock / 1000000u) / 4u;
    for (volatile uint32_t i = 0; i < n; ++i) { }
}

static void analog_pin(GPIO_TypeDef *port, uint32_t pin)
{
    if (port == GPIOA) LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    if (port == GPIOC) LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
    LL_GPIO_SetPinMode(port, pin, LL_GPIO_MODE_ANALOG);
    LL_GPIO_SetPinPull(port, pin, LL_GPIO_PULL_NO);
}

int batt_adc_init(void)
{
    analog_pin(APP_BATT_VBAT_PORT, APP_BATT_VBAT_PIN);
    analog_pin(APP_BATT_CUR_PORT, APP_BATT_CUR_PIN);

    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC);
    /* synchronous clock HCLK/4 = 16 MHz: independent of the ADC kernel clock mux */
    LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_CLOCK_SYNC_PCLK_DIV4);
    LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_VREFINT);

    if (LL_ADC_IsEnabled(ADC1)) return 0;
    LL_ADC_DisableDeepPowerDown(ADC1);
    LL_ADC_EnableInternalRegulator(ADC1);
    delay_us(LL_ADC_DELAY_INTERNAL_REGUL_STAB_US + 10u);

    LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);
    uint32_t t = 0;
    while (LL_ADC_IsCalibrationOnGoing(ADC1)) if (++t > 1000000u) return -1;
    delay_us(10u);

    LL_ADC_SetResolution(ADC1, LL_ADC_RESOLUTION_12B);
    LL_ADC_SetDataAlignment(ADC1, LL_ADC_DATA_ALIGN_RIGHT);
    LL_ADC_SetChannelSamplingTime(ADC1, __LL_ADC_DECIMAL_NB_TO_CHANNEL(APP_BATT_VBAT_CH), LL_ADC_SAMPLINGTIME_247CYCLES_5);
    LL_ADC_SetChannelSamplingTime(ADC1, __LL_ADC_DECIMAL_NB_TO_CHANNEL(APP_BATT_CUR_CH),  LL_ADC_SAMPLINGTIME_247CYCLES_5);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_VREFINT, LL_ADC_SAMPLINGTIME_247CYCLES_5);
    LL_ADC_SetChannelSingleDiff(ADC1, __LL_ADC_DECIMAL_NB_TO_CHANNEL(APP_BATT_VBAT_CH), LL_ADC_SINGLE_ENDED);
    LL_ADC_SetChannelSingleDiff(ADC1, __LL_ADC_DECIMAL_NB_TO_CHANNEL(APP_BATT_CUR_CH),  LL_ADC_SINGLE_ENDED);

    /* injected group: VBAT, CUR, VREFINT, software trigger */
    LL_ADC_INJ_SetTriggerSource(ADC1, LL_ADC_INJ_TRIG_SOFTWARE);
    LL_ADC_INJ_SetSequencerLength(ADC1, LL_ADC_INJ_SEQ_SCAN_ENABLE_3RANKS);
    LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_1, __LL_ADC_DECIMAL_NB_TO_CHANNEL(APP_BATT_VBAT_CH));
    LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_2, __LL_ADC_DECIMAL_NB_TO_CHANNEL(APP_BATT_CUR_CH));
    LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_3, LL_ADC_CHANNEL_VREFINT);
    /* 16x oversampling, shift 4: 12-bit average */
    LL_ADC_SetOverSamplingScope(ADC1, LL_ADC_OVS_GRP_INJECTED);
    LL_ADC_ConfigOverSamplingRatioShift(ADC1, LL_ADC_OVS_RATIO_16, LL_ADC_OVS_SHIFT_RIGHT_4);

    LL_ADC_ClearFlag_ADRDY(ADC1);
    LL_ADC_Enable(ADC1);
    t = 0;
    while (!LL_ADC_IsActiveFlag_ADRDY(ADC1)) if (++t > 1000000u) return -1;
    delay_us(LL_ADC_DELAY_VREFINT_STAB_US);
    LL_ADC_ClearFlag_JEOS(ADC1);
    LL_ADC_INJ_StartConversion(ADC1);
    s_started = 1u;
    return 0;
}

int batt_adc_read(float *vbat_pin, float *cur_pin)
{
    if (!s_started || !LL_ADC_IsActiveFlag_JEOS(ADC1)) return -1;
    const uint32_t v  = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1);
    const uint32_t c  = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_2);
    const uint32_t vr = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_3);
    LL_ADC_ClearFlag_JEOS(ADC1);
    LL_ADC_INJ_StartConversion(ADC1);                 /* next sample, read at the next call */
    if (vr == 0u) return -1;
    /* VDDA = 3.0 V × VREFINT_CAL / VREFINT (factory calibration at 3.0 V) */
    const float vdda = (float)VREFINT_CAL_VREF * 1e-3f * (float)(*VREFINT_CAL_ADDR) / (float)vr;
    *vbat_pin = vdda * (float)v / 4095.0f;
    *cur_pin  = vdda * (float)c / 4095.0f;
    return 0;
}
