/**
 * @file    ir_sensor.h
 * @brief   Optical IR Sensors & CPR Sequence Tracker (Zephyr RTOS)
 * @version 9.0.0
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "cpr_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool ir1; /* True if shallow sensor tripped */
    bool ir2; /* True if good depth sensor tripped */
} ir_snapshot_t;

typedef void (*compression_callback_t)(depth_result_t result);
typedef void (*recoil_callback_t)(depth_result_t result);

int ir_sensor_init(void);

ir_snapshot_t ir_sensor_read_debounced(void);

void ir_sensor_register_callbacks(compression_callback_t comp_cb, recoil_callback_t recoil_cb);

void ir_sensor_update_tick(uint32_t dt_ms);

uint16_t cpm_tracker_get_current_rate(void);

#ifdef __cplusplus
}
#endif

