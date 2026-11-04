/**
 * @file    air_sensor.h
 * @brief   BMP280 Airway Barometric Pressure Sensor Driver (Zephyr RTOS)
 *          Reads pressure differential on I2C (GPIO 8 SDA, GPIO 9 SCL).
 * @version 9.3.0
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "cpr_config.h"

#ifdef __cplusplus
extern "C" {
#endif

int air_sensor_init(void);
void air_sensor_set_active(bool active);
bool air_sensor_is_active(void);
bool air_sensor_calibrate(void);
bool air_sensor_update(bool *out_air_detected);

#ifdef __cplusplus
}
#endif
