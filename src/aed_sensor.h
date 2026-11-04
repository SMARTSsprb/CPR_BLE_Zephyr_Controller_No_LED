/**
 * @file    aed_sensor.h
 * @brief   49E Linear Hall Effect Sensor Driver for AED Pad Placement (Zephyr RTOS)
 * @version 9.1.0
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "cpr_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    aed_mode_t mode;        /* Current mode: AED_MODE_CHEST or AED_MODE_HIP */
    bool       is_attached; /* True if pad attached (diff > threshold) */
    int16_t    raw_adc;     /* Current raw ADC value */
    int16_t    baseline;    /* Calibrated baseline */
    int16_t    diff;        /* Absolute difference from baseline */
} aed_snapshot_t;

int aed_sensor_init(void);

void aed_sensor_set_mode(aed_mode_t mode);

aed_mode_t aed_sensor_get_mode(void);

bool aed_sensor_is_active(void);

void aed_sensor_calibrate(void);

aed_snapshot_t aed_sensor_read(void);

#ifdef __cplusplus
}
#endif
