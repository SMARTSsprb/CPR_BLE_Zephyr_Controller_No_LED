/**
 * @file    battery.c
 * @brief   Battery ADC Voltage & Percentage Monitor Implementation (Zephyr RTOS)
 * @version 9.0.0
 */

#include "battery.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(battery, LOG_LEVEL_INF);

static const struct device *adc_dev = NULL;
#define ADC_CH_BATTERY 2 /* GPIO 2 / ADC1 Channel 2 */

int battery_init(void)
{
    adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc0));
    if (!device_is_ready(adc_dev)) {
        LOG_WRN("ADC0 device not ready for battery");
        return -ENODEV;
    }

    struct adc_channel_cfg ch_cfg = {
        .gain = ADC_GAIN_1_4,
        .reference = ADC_REF_INTERNAL,
        .acquisition_time = ADC_ACQ_TIME_DEFAULT,
        .channel_id = ADC_CH_BATTERY,
    };

    adc_channel_setup(adc_dev, &ch_cfg);
    LOG_INF("Battery ADC initialized on Channel %d (GPIO 2)", ADC_CH_BATTERY);
    return 0;
}

uint16_t battery_get_voltage_mv(void)
{
    if (!adc_dev || !device_is_ready(adc_dev)) {
        return 3950; /* Fallback nominal 3.95V */
    }

    int16_t raw_sample = 0;
    struct adc_sequence sequence = {
        .channels = BIT(ADC_CH_BATTERY),
        .buffer = &raw_sample,
        .buffer_size = sizeof(raw_sample),
        .resolution = 12,
    };

    int ret = adc_read(adc_dev, &sequence);
    if (ret != 0 || raw_sample <= 0) {
        return 3950;
    }

    /* Convert raw 12-bit ADC (3.3V full scale) to mV at pin */
    int32_t pin_mv = (raw_sample * 3300) / 4095;
    if (pin_mv < 500) {
        return 3950; /* USB 5V test mode fallback */
    }

    /* Apply 2:1 resistive divider ratio */
    return (uint16_t)(pin_mv * BATT_DIVIDER_RATIO);
}

uint8_t battery_get_percentage(void)
{
    uint16_t mv = battery_get_voltage_mv();

    if (mv >= BATT_MAX_VOLTAGE_MV) {
        return 100;
    } else if (mv <= BATT_MIN_VOLTAGE_MV) {
        return 0;
    } else {
        return (uint8_t)(((uint32_t)(mv - BATT_MIN_VOLTAGE_MV) * 100UL) /
                         (BATT_MAX_VOLTAGE_MV - BATT_MIN_VOLTAGE_MV));
    }
}

