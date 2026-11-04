/**
 * @file    aed_sensor.c
 * @brief   49E Linear Hall Effect Sensor Driver Implementation (Zephyr RTOS)
 *          Handles Chest (A2) on ADC1_CH0 (GPIO 0) and Hip (A1) on ADC1_CH1 (GPIO 1).
 *          Enhanced with 16-sample oversampling and hysteresis for thin/small magnets.
 * @version 9.3.0
 */

#include "aed_sensor.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>
#include <stdlib.h>

LOG_MODULE_REGISTER(aed_sensor, LOG_LEVEL_INF);

static const struct device *adc_dev = NULL;
static aed_mode_t s_aed_mode        = AED_MODE_DISABLED;
static int16_t    s_baseline_chest  = 0;
static int16_t    s_baseline_hip    = 0;

static bool    s_chest_attached     = false;
static bool    s_hip_attached       = false;
static uint8_t s_chest_confirm_count = 0;
static uint8_t s_hip_confirm_count   = 0;

static int adc_read_channel(uint8_t channel, int16_t *out_val)
{
    if (!adc_dev || !device_is_ready(adc_dev)) {
        *out_val = 2048; /* Dummy midpoint if ADC not ready */
        return -ENODEV;
    }

    struct adc_channel_cfg ch_cfg = {
        .gain = ADC_GAIN_1_4,
        .reference = ADC_REF_INTERNAL,
        .acquisition_time = ADC_ACQ_TIME_DEFAULT,
        .channel_id = channel,
    };

    adc_channel_setup(adc_dev, &ch_cfg);

    int16_t sample = 0;
    struct adc_sequence sequence = {
        .channels = BIT(channel),
        .buffer = &sample,
        .buffer_size = sizeof(sample),
        .resolution = AED_ADC_RESOLUTION_BITS,
    };

    int ret = adc_read(adc_dev, &sequence);
    if (ret == 0) {
        *out_val = sample;
    }
    return ret;
}

static int16_t read_hall_average(uint8_t channel, uint8_t samples)
{
    int32_t sum = 0;
    int16_t val = 0;
    for (uint8_t i = 0; i < samples; ++i) {
        adc_read_channel(channel, &val);
        sum += val;
        k_busy_wait(200);
    }
    return (int16_t)(sum / samples);
}

int aed_sensor_init(void)
{
    adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc0));
    if (!device_is_ready(adc_dev)) {
        LOG_WRN("ADC0 device not ready");
    } else {
        LOG_INF("ADC0 initialized for Dual 49E Hall sensors (Chest=GPIO0/CH0, Hip=GPIO1/CH1)");
    }

    s_aed_mode = AED_MODE_DISABLED;
    s_chest_attached = false;
    s_hip_attached   = false;
    s_chest_confirm_count = 0;
    s_hip_confirm_count   = 0;
    return 0;
}

void aed_sensor_calibrate(void)
{
    if (s_aed_mode == AED_MODE_CHEST) {
        s_baseline_chest = read_hall_average(AED_ADC_CH_CHEST, AED_CALIBRATION_SAMPLES);
        LOG_INF("Chest AED Calibrated (GPIO 0): Baseline=%d (Sens Threshold=%d, Hyst=%d)",
                s_baseline_chest, AED_DIFF_THRESHOLD, AED_HYSTERESIS_THRESHOLD);
    } else if (s_aed_mode == AED_MODE_HIP) {
        s_baseline_hip = read_hall_average(AED_ADC_CH_HIP, AED_CALIBRATION_SAMPLES);
        LOG_INF("Hip AED Calibrated (GPIO 1): Baseline=%d (Sens Threshold=%d, Hyst=%d)",
                s_baseline_hip, AED_DIFF_THRESHOLD, AED_HYSTERESIS_THRESHOLD);
    }
}

void aed_sensor_set_mode(aed_mode_t mode)
{
    s_aed_mode = mode;
    s_chest_attached = false;
    s_hip_attached   = false;
    s_chest_confirm_count = 0;
    s_hip_confirm_count   = 0;

    if (mode == AED_MODE_CHEST) {
        aed_sensor_calibrate();
        LOG_INF(">>> CHEST AED Pad Monitoring ACTIVATED on GPIO 0 (Cmd 'A2', Threshold=%d)", AED_DIFF_THRESHOLD);
    } else if (mode == AED_MODE_HIP) {
        aed_sensor_calibrate();
        LOG_INF(">>> HIP AED Pad Monitoring ACTIVATED on GPIO 1 (Cmd 'A1', Threshold=%d)", AED_DIFF_THRESHOLD);
    } else {
        LOG_INF("<<< AED Pad Monitoring DEACTIVATED (Cmd 'A0')");
    }
}

aed_mode_t aed_sensor_get_mode(void)
{
    return s_aed_mode;
}

bool aed_sensor_is_active(void)
{
    return (s_aed_mode != AED_MODE_DISABLED);
}

aed_snapshot_t aed_sensor_read(void)
{
    aed_snapshot_t snap = {
        .mode = s_aed_mode,
        .is_attached = false,
        .raw_adc = 0,
        .baseline = 0,
        .diff = 0
    };

    if (s_aed_mode == AED_MODE_DISABLED) return snap;

    uint8_t channel = (s_aed_mode == AED_MODE_CHEST) ? AED_ADC_CH_CHEST : AED_ADC_CH_HIP;
    int16_t current = read_hall_average(channel, AED_OVERSAMPLE_COUNT);
    int16_t baseline = (s_aed_mode == AED_MODE_CHEST) ? s_baseline_chest : s_baseline_hip;

    snap.raw_adc     = current;
    snap.baseline    = baseline;
    snap.diff        = abs(current - baseline);

    bool *p_attached = (s_aed_mode == AED_MODE_CHEST) ? &s_chest_attached : &s_hip_attached;
    uint8_t *p_confirm = (s_aed_mode == AED_MODE_CHEST) ? &s_chest_confirm_count : &s_hip_confirm_count;

    if (!(*p_attached)) {
        if (snap.diff >= AED_DIFF_THRESHOLD) {
            (*p_confirm)++;
            if (*p_confirm >= 2) {
                *p_attached = true;
                *p_confirm = 0;
            }
        } else {
            *p_confirm = 0;
        }
    } else {
        if (snap.diff <= AED_HYSTERESIS_THRESHOLD) {
            (*p_confirm)++;
            if (*p_confirm >= 2) {
                *p_attached = false;
                *p_confirm = 0;
            }
        } else {
            *p_confirm = 0;
        }
    }

    snap.is_attached = *p_attached;
    return snap;
}
